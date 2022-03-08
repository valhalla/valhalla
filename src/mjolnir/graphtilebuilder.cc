#include "mjolnir/graphtilebuilder.h"

#include "baldr/datetime.h"
#include "baldr/edgeinfo.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include <algorithm>
#include <boost/format.hpp>
#include <list>
#include <set>
#include <stdexcept>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

namespace {

std::vector<ComplexRestrictionBuilder> DeserializeRestrictions(char* restrictions,
                                                               size_t restrictions_size) {
  std::vector<ComplexRestrictionBuilder> builders;
  size_t offset = 0;
  while (offset < restrictions_size) {
    const ComplexRestriction* cr = reinterpret_cast<ComplexRestriction*>(restrictions + offset);
    ComplexRestrictionBuilder builder(*cr);
    if (cr->via_count()) {
      std::vector<GraphId> vias;
      vias.reserve(cr->via_count());
      const baldr::GraphId* via = reinterpret_cast<const baldr::GraphId*>(cr + 1);
      for (uint32_t i = 0; i < cr->via_count(); i++, ++via) {
        vias.push_back(*via);
      }
      builder.set_via_list(vias);
    }
    builders.push_back(std::move(builder));
    offset += cr->SizeOf();
  }
  return builders;
};

} // namespace

// Constructor given an existing tile. This is used to read in the tile
// data and then add to it (e.g. adding node connections between hierarchy
// levels. If the deserialize flag is set then all objects are serialized
// from memory into builders that can be added to and then stored using
// StoreTileData.
GraphTileBuilder::GraphTileBuilder(const std::string& tile_dir,
                                   const GraphId& graphid,
                                   bool deserialize,
                                   bool serialize_turn_lanes)
    : GraphTile(tile_dir, graphid), tile_dir_(tile_dir) {

  // Copy tile header to a builder (if tile exists). Always set the tileid
  if (header_) {
    header_builder_ = *header_;
  }
  header_builder_.set_graphid(graphid);

  // Done if not deserializing and creating builders for everything
  if (!deserialize) {
    textlistbuilder_.emplace_back("");
    text_offset_map_.emplace("", 0);
    text_list_offset_ = 1;

    // Add a dummy admin record at index 0 to be used if admin records are
    // not used/created or if none is found.
    AddAdmin("None", "None", "", "");
    return;
  }

  // Street name info. Unique set of offsets into the text list
  std::set<NameInfo> name_info;
  name_info.insert({0});

  // Copy nodes to the builder list
  size_t n = header_->nodecount();
  nodes_builder_.reserve(n);
  std::copy(nodes_, nodes_ + n, std::back_inserter(nodes_builder_));

  // Copy node transitions to the builder list
  n = header_->transitioncount();
  transitions_builder_.reserve(n);
  std::copy(transitions_, transitions_ + n, std::back_inserter(transitions_builder_));

  // Copy directed edges to the builder list
  n = header_->directededgecount();
  directededges_builder_.reserve(n);
  std::copy(directededges_, directededges_ + n, std::back_inserter(directededges_builder_));

  // Add extended directededge attributes (if available)
  if (header_->has_ext_directededge()) {
    // Copy extended directed edges to the builder list
    // NOTE: directed edge and directed edge extensions are assumed to have the
    // same length
    directededges_ext_builder_.reserve(n);
    std::copy(ext_directededges_, ext_directededges_ + n,
              std::back_inserter(directededges_ext_builder_));
  }

  // Create access restriction list
  for (uint32_t i = 0; i < header_->access_restriction_count(); i++) {
    access_restriction_builder_.emplace_back(std::move(access_restrictions_[i]));
  }

  // Create transit builders and add any text offsets to the set
  for (uint32_t i = 0; i < header_->departurecount(); i++) {
    departure_builder_.emplace_back(std::move(departures_[i]));
    name_info.insert({departures_[i].headsign_offset()});
  }
  for (uint32_t i = 0; i < header_->stopcount(); i++) {
    stop_builder_.emplace_back(std::move(transit_stops_[i]));
    name_info.insert({transit_stops_[i].one_stop_offset()});
    name_info.insert({transit_stops_[i].name_offset()});
  }
  for (uint32_t i = 0; i < header_->routecount(); i++) {
    route_builder_.emplace_back(std::move(transit_routes_[i]));
    name_info.insert({transit_routes_[i].one_stop_offset()});
    name_info.insert({transit_routes_[i].op_by_onestop_id_offset()});
    name_info.insert({transit_routes_[i].op_by_name_offset()});
    name_info.insert({transit_routes_[i].op_by_website_offset()});
    name_info.insert({transit_routes_[i].short_name_offset()});
    name_info.insert({transit_routes_[i].long_name_offset()});
    name_info.insert({transit_routes_[i].desc_offset()});
  }
  for (uint32_t i = 0; i < header_->schedulecount(); i++) {
    schedule_builder_.emplace_back(std::move(transit_schedules_[i]));
  }

  // Create sign builders
  for (uint32_t i = 0; i < header_->signcount(); i++) {
    name_info.insert({signs_[i].text_offset()});
    signs_builder_.emplace_back(signs_[i].index(), signs_[i].type(), signs_[i].is_route_num_type(),
                                signs_[i].tagged(), signs_[i].text_offset());
  }

  // Create turn lane builders
  for (uint32_t i = 0; i < header_->turnlane_count(); i++) {
    if (serialize_turn_lanes)
      name_info.insert({turnlanes_[i].text_offset()});
    turnlanes_builder_.emplace_back(turnlanes_[i].edgeindex(), turnlanes_[i].text_offset());
  }

  // Create admin builders
  for (uint32_t i = 0; i < header_->admincount(); i++) {
    admins_builder_.emplace_back(admins_[i].country_offset(), admins_[i].state_offset(),
                                 admins_[i].country_iso(), admins_[i].state_iso());
    name_info.insert({admins_[i].country_offset()});
    name_info.insert({admins_[i].state_offset()});
  }

  // Edge bins are gotten by parent

  // Create an ordered set of edge info offsets
  std::set<uint32_t> edge_info_offsets;
  for (auto& diredge : directededges_builder_) {
    edge_info_offsets.insert(diredge.edgeinfo_offset());
  }

  // At this time, complex restrictions are created AFTER all need for
  // serializing and adding to a tile - so we assume they are both empty.

  // Serializing complex restrictions would be difficult since they require a
  // temporary vector of the via graphIds to be constructed for the ComplexRestrictionBuilder.
  // This is possible, but non-trivial since the complex restriction data has a fixed size
  // structure plus the variable sized data (the via Ids).

  // EdgeInfo. Create list of EdgeInfoBuilders. Add to text offset set.
  edge_info_offset_ = 0;
  edgeinfo_offset_map_.clear();
  for (auto offset : edge_info_offsets) {
    // Verify the offsets match as we create the edge info builder list
    if (offset != edge_info_offset_) {
      LOG_WARN("GraphTileBuilder TileID: " + std::to_string(header_->graphid().tileid()) +
               " offset stored in directed edge: = " + std::to_string(offset) +
               " current ei offset= " + std::to_string(edge_info_offset_));
    }

    EdgeInfo ei(edgeinfo_ + offset, textlist_, textlist_size_);
    EdgeInfoBuilder eib;
    eib.set_wayid(ei.wayid());
    eib.set_mean_elevation(ei.mean_elevation());
    eib.set_bike_network(ei.bike_network());
    eib.set_speed_limit(ei.speed_limit());
    for (uint32_t nm = 0; nm < ei.name_count(); nm++) {
      NameInfo info = ei.GetNameInfo(nm);
      name_info.insert(info);
      eib.AddNameInfo(info);
    }
    eib.set_encoded_shape(ei.encoded_shape());
    edge_info_offset_ += eib.SizeOf();
    edgeinfo_list_.emplace_back(std::move(eib));

    // Associate the offset to the index in the edgeinfo list
    edgeinfo_offset_map_[offset] = &edgeinfo_list_.back();
  }

  // Text list
  for (auto ni = name_info.begin(); ni != name_info.end(); ++ni) {
    // compute the width of the entry by looking at the next offset or the end if its the last one
    auto next = std::next(ni);
    auto width = next != name_info.end() ? (next->name_offset_ - ni->name_offset_)
                                         : (textlist_size_ - ni->name_offset_);

    // Keep the bytes for this entry....remove null terminating char as it is added in StoreTileData
    textlistbuilder_.emplace_back(textlist_ + ni->name_offset_, width - 1);
    // Remember what offset they had
    text_offset_map_.emplace(textlistbuilder_.back(), ni->name_offset_);
    // Keep track of how large it is for storing it back to disk later
    text_list_offset_ += textlistbuilder_.back().length() + 1;
  }

  // Lane connectivity
  lane_connectivity_offset_ = lane_connectivity_size_;
  n = lane_connectivity_size_ / sizeof(LaneConnectivity);
  lane_connectivity_builder_.reserve(n);
  std::copy(lane_connectivity_, lane_connectivity_ + n,
            std::back_inserter(lane_connectivity_builder_));

  complex_restriction_forward_builder_ =
      DeserializeRestrictions(complex_restriction_forward_, complex_restriction_forward_size_);
  complex_restriction_reverse_builder_ =
      DeserializeRestrictions(complex_restriction_reverse_, complex_restriction_reverse_size_);
}

// Output the tile to file. Stores as binary data.
void GraphTileBuilder::StoreTileData() {
  // Get the name of the file
  filesystem::path filename(tile_dir_ + filesystem::path::preferred_separator +
                            GraphTile::FileSuffix(header_builder_.graphid()));

  // Make sure the directory exists on the system
  if (!filesystem::exists(filename.parent_path())) {
    filesystem::create_directories(filename.parent_path());
  }

  // Open file and truncate
  std::stringstream in_mem;
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (file.is_open()) {
    // Write the nodes
    header_builder_.set_nodecount(nodes_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(nodes_builder_.data()),
                 nodes_builder_.size() * sizeof(NodeInfo));

    // Write the node transitions
    header_builder_.set_transitioncount(transitions_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(transitions_builder_.data()),
                 transitions_builder_.size() * sizeof(NodeTransition));

    // Write the directed edges
    header_builder_.set_directededgecount(directededges_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(directededges_builder_.data()),
                 directededges_builder_.size() * sizeof(DirectedEdge));

    // Write extended directed edge attributes if they exist.
    if (directededges_ext_builder_.size() > 0) {
      if (directededges_ext_builder_.size() != directededges_builder_.size()) {
        LOG_ERROR("DirectedEdge extended attributes not same size as directed edges");
      } else {
        header_builder_.set_has_ext_directededge(true);
        in_mem.write(reinterpret_cast<const char*>(directededges_ext_builder_.data()),
                     directededges_ext_builder_.size() * sizeof(DirectedEdgeExt));
      }
    }

    // Sort and write the access restrictions
    header_builder_.set_access_restriction_count(access_restriction_builder_.size());
    std::sort(access_restriction_builder_.begin(), access_restriction_builder_.end());
    in_mem.write(reinterpret_cast<const char*>(access_restriction_builder_.data()),
                 access_restriction_builder_.size() * sizeof(AccessRestriction));

    // Sort and write the transit departures
    header_builder_.set_departurecount(departure_builder_.size());
    std::sort(departure_builder_.begin(), departure_builder_.end());
    in_mem.write(reinterpret_cast<const char*>(departure_builder_.data()),
                 departure_builder_.size() * sizeof(TransitDeparture));

    // Sort write the transit stops
    header_builder_.set_stopcount(stop_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(stop_builder_.data()),
                 stop_builder_.size() * sizeof(TransitStop));

    // Write the transit routes
    header_builder_.set_routecount(route_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(route_builder_.data()),
                 route_builder_.size() * sizeof(TransitRoute));

    // Write transit schedules
    header_builder_.set_schedulecount(schedule_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(schedule_builder_.data()),
                 schedule_builder_.size() * sizeof(TransitSchedule));

    // TODO add transfers later
    header_builder_.set_transfercount(0);

    // Write the signs
    std::stable_sort(signs_builder_.begin(), signs_builder_.end());
    header_builder_.set_signcount(signs_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(signs_builder_.data()),
                 signs_builder_.size() * sizeof(Sign));

    // Write turn lanes
    header_builder_.set_turnlane_count(turnlanes_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(turnlanes_builder_.data()),
                 turnlanes_builder_.size() * sizeof(TurnLanes));

    // Write the admins
    header_builder_.set_admincount(admins_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(admins_builder_.data()),
                 admins_builder_.size() * sizeof(Admin));

    // Edge bins can only be added after you've stored the tile

    // Write the forward complex restriction data
    header_builder_.set_complex_restriction_forward_offset(
        (sizeof(GraphTileHeader)) + (nodes_builder_.size() * sizeof(NodeInfo)) +
        (transitions_builder_.size() * sizeof(NodeTransition)) +
        (directededges_builder_.size() * sizeof(DirectedEdge)) +
        (directededges_ext_builder_.size() * sizeof(DirectedEdgeExt)) +
        (access_restriction_builder_.size() * sizeof(AccessRestriction)) +
        (departure_builder_.size() * sizeof(TransitDeparture)) +
        (stop_builder_.size() * sizeof(TransitStop)) +
        (route_builder_.size() * sizeof(TransitRoute)) +
        (schedule_builder_.size() * sizeof(TransitSchedule)) +
        // TODO - once transit transfers are added need to update here
        (signs_builder_.size() * sizeof(Sign)) + (turnlanes_builder_.size() * sizeof(TurnLanes)) +
        (admins_builder_.size() * sizeof(Admin)));
    uint32_t forward_restriction_size = 0;
    for (auto& complex_restriction : complex_restriction_forward_builder_) {
      in_mem << complex_restriction;
      forward_restriction_size += complex_restriction.SizeOf();
    }

    // Write the reverse complex restriction data
    header_builder_.set_complex_restriction_reverse_offset(
        header_builder_.complex_restriction_forward_offset() + forward_restriction_size);
    uint32_t reverse_restriction_size = 0;
    for (auto& complex_restriction : complex_restriction_reverse_builder_) {
      in_mem << complex_restriction;
      reverse_restriction_size += complex_restriction.SizeOf();
    }

    // Write the edge data
    header_builder_.set_edgeinfo_offset(header_builder_.complex_restriction_reverse_offset() +
                                        reverse_restriction_size);
    for (const auto& edgeinfo : edgeinfo_list_) {
      in_mem << edgeinfo;
    }

    // Write the names
    header_builder_.set_textlist_offset(header_builder_.edgeinfo_offset() + edge_info_offset_);
    for (const auto& text : textlistbuilder_) {
      in_mem << text << '\0';
    }

    // Add padding (if needed) to align to 8-byte word.
    int tmp = in_mem.tellp() % 8;
    int padding = (tmp > 0) ? 8 - tmp : 0;
    if (padding > 0 && padding < 8) {
      in_mem.write("\0\0\0\0\0\0\0\0", padding);
    }

    // Write lane connections
    header_builder_.set_lane_connectivity_offset(header_builder_.textlist_offset() +
                                                 text_list_offset_ + padding);
    std::sort(lane_connectivity_builder_.begin(), lane_connectivity_builder_.end());
    in_mem.write(reinterpret_cast<const char*>(lane_connectivity_builder_.data()),
                 lane_connectivity_builder_.size() * sizeof(LaneConnectivity));

    // Set the end offset
    header_builder_.set_end_offset(header_builder_.lane_connectivity_offset() +
                                   (lane_connectivity_builder_.size() * sizeof(LaneConnectivity)));

    // Sanity check for the end offset
    uint32_t curr =
        static_cast<uint32_t>(in_mem.tellp()) + static_cast<uint32_t>(sizeof(GraphTileHeader));
    if (header_builder_.end_offset() != curr) {
      LOG_ERROR("Mismatch in end offset " + std::to_string(header_builder_.end_offset()) +
                " vs in_mem stream " + std::to_string(curr) +
                " padding = " + std::to_string(padding));
    }

    LOG_DEBUG((boost::format("Write: %1% nodes = %2% directededges = %3% signs %4% edgeinfo offset "
                             "= %5% textlist offset = %6% lane connections = %7%") %
               filename % nodes_builder_.size() % directededges_builder_.size() %
               signs_builder_.size() % edge_info_offset_ % text_list_offset_ %
               lane_connectivity_builder_.size())
                  .str());
    LOG_DEBUG((boost::format("   admins = %1%  departures = %2% stops = %3% routes = %4%") %
               admins_builder_.size() % departure_builder_.size() % stop_builder_.size() %
               route_builder_.size())
                  .str());

    // Write the header then the rest of the tile from the in memory buffer
    file.write(reinterpret_cast<const char*>(&header_builder_), sizeof(GraphTileHeader));
    file << in_mem.rdbuf();
    file.close();
  } else {
    throw std::runtime_error("Failed to open file " + filename.string());
  }
}

// Update a graph tile with new nodes and directed edges. The rest of the
// tile contents remains the same.
void GraphTileBuilder::Update(const std::vector<NodeInfo>& nodes,
                              const std::vector<DirectedEdge>& directededges) {
  // Get the name of the file
  filesystem::path filename =
      tile_dir_ + filesystem::path::preferred_separator + GraphTile::FileSuffix(header_->graphid());

  // Make sure the directory exists on the system
  if (!filesystem::exists(filename.parent_path())) {
    filesystem::create_directories(filename.parent_path());
  }

  // Open file. Truncate so we replace the contents.
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (file.is_open()) {
    // Write the header
    file.write(reinterpret_cast<const char*>(header_), sizeof(GraphTileHeader));

    // Write the updated nodes. Make sure node count matches.
    if (nodes.size() != header_->nodecount()) {
      throw std::runtime_error("GraphTileBuilder::Update - node count has changed");
    }
    file.write(reinterpret_cast<const char*>(nodes.data()), nodes.size() * sizeof(NodeInfo));

    // Write node transitions
    file.write(reinterpret_cast<const char*>(transitions_),
               header_->transitioncount() * sizeof(NodeTransition));

    // Write the updated directed edges. Make sure edge count matches.
    if (directededges.size() != header_->directededgecount()) {
      throw std::runtime_error("GraphTileBuilder::Update - directed edge count has changed");
    }
    file.write(reinterpret_cast<const char*>(directededges.data()),
               directededges.size() * sizeof(DirectedEdge));

    // If there are extended directed edge attributes they would need to be written out here
    // (and likely added to the method)

    // Write the rest of the tiles
    auto begin = reinterpret_cast<const char*>(&access_restrictions_[0]);
    auto end = reinterpret_cast<const char*>(header()) + header()->end_offset();
    file.write(begin, end - begin);
    file.close();
  } else {
    throw std::runtime_error("GraphTileBuilder::Update - Failed to open file " + filename.string());
  }
}

// Gets a reference to the header builder.
GraphTileHeader& GraphTileBuilder::header_builder() {
  return header_builder_;
}

// Get the current list of node builders.
std::vector<NodeInfo>& GraphTileBuilder::nodes() {
  return nodes_builder_;
}

// Gets the current list of directed edge (builders).
std::vector<DirectedEdge>& GraphTileBuilder::directededges() {
  return directededges_builder_;
}

// Gets the current list of directed edge extension (builders).
std::vector<DirectedEdgeExt>& GraphTileBuilder::directededges_ext() {
  return directededges_ext_builder_;
}

// Add a transit departure.
void GraphTileBuilder::AddTransitDeparture(const TransitDeparture& departure) {
  departure_builder_.emplace_back(std::move(departure));
}

// Add a transit stop.
void GraphTileBuilder::AddTransitStop(const TransitStop& stop) {
  stop_builder_.emplace_back(std::move(stop));
}

// Add a transit route.
void GraphTileBuilder::AddTransitRoute(const TransitRoute& route) {
  route_builder_.emplace_back(std::move(route));
}

// Add a transit schedule.
void GraphTileBuilder::AddTransitSchedule(const TransitSchedule& schedule) {
  schedule_builder_.emplace_back(std::move(schedule));
}

// Add an access restriction.
void GraphTileBuilder::AddAccessRestriction(const AccessRestriction& access_restriction) {
  access_restriction_builder_.emplace_back(std::move(access_restriction));
}

// Add access restrictions
void GraphTileBuilder::AddAccessRestrictions(const std::vector<AccessRestriction>& restrictions) {
  access_restriction_builder_.clear();
  // Add restrictions to the list
  access_restriction_builder_ = restrictions;
}

// Add signs
void GraphTileBuilder::AddSigns(const uint32_t idx,
                                const std::vector<SignInfo>& signs,
                                const std::vector<std::string>& pronunciations) {
  // Iterate through the list of sign info (with sign text) and add sign
  // text to the text list. Skip signs with no text.
  for (size_t i = 0; i < signs.size(); ++i) {
    auto sign = signs[i];
    if (!(sign.text().empty())) {
      uint32_t offset = AddName(sign.text());
      signs_builder_.emplace_back(idx, sign.type(), sign.is_route_num(), sign.is_tagged(), offset);
      if (sign.has_phoneme()) {
        bool phoneme_on_node = sign.type() == Sign::Type::kJunctionName;
        uint32_t count = (sign.phoneme_start_index() + sign.phoneme_count()) - 1;
        for (uint32_t x = sign.phoneme_start_index(); x <= count; x++) {
          auto* p = const_cast<char*>(pronunciations[x].c_str());
          size_t pos = 0;
          std::string updated_pronunciation;

          while (pos < strlen(p)) {
            linguistic_text_header_t header =
                midgard::unaligned_read<linguistic_text_header_t>(p + pos);
            pos += 3;
            header.name_index_ = i;
            updated_pronunciation.append(std::string(reinterpret_cast<const char*>(&header), 3) +
                                         (p + pos));
            pos += header.length_;
          }

          uint32_t offset = AddName(updated_pronunciation);
          signs_builder_.emplace_back(idx, Sign::Type::kPronunciation, phoneme_on_node, true, offset);
        }
      }
    }
  }
}

// Add signs
void GraphTileBuilder::AddSigns(const uint32_t idx, const std::vector<SignInfo>& signs) {
  // Iterate through the list of sign info (with sign text) and add sign
  // text to the text list. Skip signs with no text.
  for (const auto& sign : signs) {
    if (!(sign.text().empty())) {
      uint32_t offset = 0;
      offset = AddName(sign.text());
      signs_builder_.emplace_back(idx, sign.type(), sign.is_route_num(), sign.is_tagged(), offset);
    }
  }
}

// Add lane connectivity
void GraphTileBuilder::AddLaneConnectivity(const std::vector<baldr::LaneConnectivity>& lc) {
  lane_connectivity_builder_.insert(lane_connectivity_builder_.end(), lc.begin(), lc.end());
  lane_connectivity_offset_ += sizeof(baldr::LaneConnectivity) * lc.size();
}

// Add forward complex restriction.
void GraphTileBuilder::AddForwardComplexRestriction(const ComplexRestrictionBuilder& res) {
  complex_restriction_forward_builder_.push_back(res);
}

// Add reverse complex restriction.
void GraphTileBuilder::AddReverseComplexRestriction(const ComplexRestrictionBuilder& res) {
  complex_restriction_reverse_builder_.push_back(res);
}

bool GraphTileBuilder::HasEdgeInfo(const uint32_t edgeindex,
                                   const baldr::GraphId& nodea,
                                   const baldr::GraphId& nodeb,
                                   uint32_t& edge_info_offset) {
  auto edge_tuple_item = EdgeTuple(edgeindex, nodea, nodeb);
  auto existing_edge_offset_item = edge_offset_map_.find(edge_tuple_item);
  if (existing_edge_offset_item != edge_offset_map_.cend()) {
    edge_info_offset = existing_edge_offset_item->second;
    return true;
  }
  return false;
}

// Add edge info
template <class shape_container_t>
uint32_t GraphTileBuilder::AddEdgeInfo(const uint32_t edgeindex,
                                       const GraphId& nodea,
                                       const baldr::GraphId& nodeb,
                                       const uint64_t wayid,
                                       const float elev,
                                       const uint32_t bike_network,
                                       const uint32_t speed_limit,
                                       const shape_container_t& lls,
                                       const std::vector<std::string>& names,
                                       const std::vector<std::string>& tagged_values,
                                       const std::vector<std::string>& pronunciations,
                                       const uint16_t types,
                                       bool& added,
                                       bool diff_names) {
  // If we haven't yet added edge info for this edge tuple
  auto edge_tuple_item = EdgeTuple(edgeindex, nodea, nodeb);
  auto existing_edge_offset_item = edge_offset_map_.find(edge_tuple_item);
  if (diff_names || existing_edge_offset_item == edge_offset_map_.end()) {
    // Add a new EdgeInfo to the list and get a reference to it
    edgeinfo_list_.emplace_back();
    EdgeInfoBuilder& edgeinfo = edgeinfo_list_.back();
    edgeinfo.set_wayid(wayid);
    edgeinfo.set_mean_elevation(elev);
    edgeinfo.set_bike_network(bike_network);
    edgeinfo.set_speed_limit(speed_limit);
    edgeinfo.set_shape(lls);

    // Add names to the common text/name list. Skip blank names.
    std::vector<NameInfo> name_info_list;
    name_info_list.reserve(std::min(names.size(), kMaxNamesPerEdge));
    size_t name_count = 0;
    size_t location = 0;
    for (const auto& name : names) {
      // Stop adding names if max count has been reached
      if (name_count == kMaxNamesPerEdge) {
        LOG_WARN("Too many names for edgeindex: " + std::to_string(edgeindex));
        break;
      }

      // Verify name is not empty
      if (!(name.empty())) {
        // Add name and add its offset to edge info's list.
        NameInfo ni{AddName(name)};
        ni.is_route_num_ = 0;
        ni.tagged_ = 0;
        if ((types & (1ULL << location))) {
          ni.is_route_num_ = 1; // set the ref bit.
        }
        name_info_list.emplace_back(ni);
        ++name_count;
      }
      location++;
    }
    for (const auto& name : tagged_values) {
      // Stop adding names if max count has been reached
      if (name_count == kMaxNamesPerEdge) {
        LOG_WARN("Too many names for edgeindex: " + std::to_string(edgeindex));
        break;
      }

      // Verify name is not empty
      if (!(name.empty())) {
        // Add name and add its offset to edge info's list.
        NameInfo ni{AddName(name)};
        ni.is_route_num_ = 0;
        ni.tagged_ = 1;
        name_info_list.emplace_back(ni);
        ++name_count;
      }
    }

    if (pronunciations.size()) {
      if (name_count != kMaxNamesPerEdge) {
        std::stringstream ss;
        for (const auto& pronunciation : pronunciations) {
          ss << pronunciation;
        }

        auto encode_tag = [](valhalla::baldr::TaggedValue tag) {
          return std::string(1, static_cast<std::string::value_type>(tag));
        };

        // Add pronunciations and add its offset to edge info's list.
        NameInfo ni{AddName(encode_tag(valhalla::baldr::TaggedValue::kPronunciation) + ss.str())};

        ni.is_route_num_ = 0;
        ni.tagged_ = 1;
        name_info_list.emplace_back(ni);
        ++name_count;
      } else
        LOG_WARN("Too many names for edgeindex: " + std::to_string(edgeindex));
    }

    edgeinfo.set_name_info_list(name_info_list);

    // Add to the map
    edge_offset_map_.emplace(edge_tuple_item, edge_info_offset_);

    // Set current edge offset
    uint32_t current_edge_offset = edge_info_offset_;

    // Update edge offset for next item
    edge_info_offset_ += edgeinfo.SizeOf();

    // Return the offset to this edge info
    added = true;
    return current_edge_offset;
  }

  // Already have this edge - return the offset
  added = false;
  return existing_edge_offset_item->second;
}
template uint32_t GraphTileBuilder::AddEdgeInfo<std::vector<PointLL>>(const uint32_t edgeindex,
                                                                      const GraphId&,
                                                                      const baldr::GraphId&,
                                                                      const uint64_t,
                                                                      const float,
                                                                      const uint32_t,
                                                                      const uint32_t,
                                                                      const std::vector<PointLL>&,
                                                                      const std::vector<std::string>&,
                                                                      const std::vector<std::string>&,
                                                                      const std::vector<std::string>&,
                                                                      const uint16_t,
                                                                      bool&,
                                                                      bool);
template uint32_t GraphTileBuilder::AddEdgeInfo<std::list<PointLL>>(const uint32_t edgeindex,
                                                                    const GraphId&,
                                                                    const baldr::GraphId&,
                                                                    const uint64_t,
                                                                    const float,
                                                                    const uint32_t,
                                                                    const uint32_t,
                                                                    const std::list<PointLL>&,
                                                                    const std::vector<std::string>&,
                                                                    const std::vector<std::string>&,
                                                                    const std::vector<std::string>&,
                                                                    const uint16_t,
                                                                    bool&,
                                                                    bool);

// AddEdgeInfo - accepts an encoded shape string.
uint32_t GraphTileBuilder::AddEdgeInfo(const uint32_t edgeindex,
                                       const baldr::GraphId& nodea,
                                       const baldr::GraphId& nodeb,
                                       const uint64_t wayid,
                                       const float elev,
                                       const uint32_t bike_network,
                                       const uint32_t speed_limit,
                                       const std::string& llstr,
                                       const std::vector<std::string>& names,
                                       const std::vector<std::string>& tagged_values,
                                       const std::vector<std::string>& pronunciations,
                                       const uint16_t types,
                                       bool& added,
                                       bool diff_names) {
  // If we haven't yet added edge info for this edge tuple
  auto edge_tuple_item = EdgeTuple(edgeindex, nodea, nodeb);
  auto existing_edge_offset_item = edge_offset_map_.find(edge_tuple_item);
  if (diff_names || existing_edge_offset_item == edge_offset_map_.end()) {
    // Add a new EdgeInfo to the list and get a reference to it
    edgeinfo_list_.emplace_back();
    EdgeInfoBuilder& edgeinfo = edgeinfo_list_.back();
    edgeinfo.set_wayid(wayid);
    edgeinfo.set_mean_elevation(elev);
    edgeinfo.set_bike_network(bike_network);
    edgeinfo.set_speed_limit(speed_limit);
    edgeinfo.set_encoded_shape(llstr);

    // Add names to the common text/name list. Skip blank names.
    std::vector<NameInfo> name_info_list;
    name_info_list.reserve(std::min(names.size(), kMaxNamesPerEdge));
    size_t name_count = 0;
    size_t location = 0;
    for (const auto& name : names) {
      // Stop adding names if max count has been reached
      if (name_count == kMaxNamesPerEdge) {
        LOG_WARN("Too many names for edgeindex: " + std::to_string(edgeindex));
        break;
      }

      // Verify name is not empty
      if (!(name.empty())) {
        // Add name and add its offset to edge info's list.
        NameInfo ni{AddName(name)};
        ni.is_route_num_ = 0;
        ni.tagged_ = 0;
        if ((types & (1ULL << location))) {
          ni.is_route_num_ = 1; // set the ref bit.
        }
        name_info_list.emplace_back(ni);
        ++name_count;
      }
      location++;
    }
    for (const auto& name : tagged_values) {
      // Stop adding names if max count has been reached
      if (name_count == kMaxNamesPerEdge) {
        LOG_WARN("Too many names for edgeindex: " + std::to_string(edgeindex));
        break;
      }

      // Verify name is not empty
      if (!(name.empty())) {
        // Add name and add its offset to edge info's list.
        NameInfo ni{AddName(name)};
        ni.is_route_num_ = 0;
        ni.tagged_ = 1;
        name_info_list.emplace_back(ni);
        ++name_count;
      }
    }

    if (pronunciations.size()) {
      if (name_count != kMaxNamesPerEdge) {
        std::stringstream ss;
        for (const auto& pronunciation : pronunciations) {
          ss << pronunciation;
        }

        auto encode_tag = [](valhalla::baldr::TaggedValue tag) {
          return std::string(1, static_cast<std::string::value_type>(tag));
        };

        // Add pronunciations and add its offset to edge info's list.
        NameInfo ni{AddName(encode_tag(valhalla::baldr::TaggedValue::kPronunciation) + ss.str())};

        ni.is_route_num_ = 0;
        ni.tagged_ = 1;
        name_info_list.emplace_back(ni);
        ++name_count;
      } else
        LOG_WARN("Too many names for edgeindex: " + std::to_string(edgeindex));
    }

    edgeinfo.set_name_info_list(name_info_list);

    // Add to the map
    edge_offset_map_.emplace(edge_tuple_item, edge_info_offset_);

    // Set current edge offset
    uint32_t current_edge_offset = edge_info_offset_;

    // Update edge offset for next item
    edge_info_offset_ += edgeinfo.SizeOf();

    // Return the offset to this edge info
    added = true;
    return current_edge_offset;
  }

  // Already have this edge - return the offset
  added = false;
  return existing_edge_offset_item->second;
}

// Set the mean elevation in the last added EdgeInfo.
void GraphTileBuilder::set_mean_elevation(const float elev) {
  EdgeInfoBuilder& edgeinfo = edgeinfo_list_.back();
  edgeinfo.set_mean_elevation(elev);
}

// Set the mean elevation to the EdgeInfo given the edge info offset. This requires
// a serialized tile builder.
void GraphTileBuilder::set_mean_elevation(const uint32_t offset, const float elev) {
  auto e = edgeinfo_offset_map_.find(offset);
  if (e == edgeinfo_offset_map_.end()) {
    LOG_ERROR("set_mean_elevation - could not find the EdgeInfo index given the offset");
    return;
  }
  e->second->set_mean_elevation(elev);
}

// Add a name to the text list
uint32_t GraphTileBuilder::AddName(const std::string& name) {
  if (name.empty()) {
    return 0;
  }

  // If nothing already used this name
  auto existing_text_offset = text_offset_map_.find(name);
  if (existing_text_offset == text_offset_map_.end()) {
    // Save the current offset and add name to text list
    uint32_t offset = text_list_offset_;
    textlistbuilder_.emplace_back(name);

    // Add name/offset pair to map and update text offset value
    // to length of string plus null terminator
    text_offset_map_.emplace(name, text_list_offset_);
    text_list_offset_ += (name.length() + 1);
    return offset;
  } else {
    // Return the offset to the existing name
    return existing_text_offset->second;
  }
}

// Add admin
uint32_t GraphTileBuilder::AddAdmin(const std::string& country_name,
                                    const std::string& state_name,
                                    const std::string& country_iso,
                                    const std::string& state_iso) {
  // Check if admin already exists
  auto existing_admin_info_offset_item =
      admin_info_offset_map_.find(country_iso + state_iso + state_name);
  if (existing_admin_info_offset_item == admin_info_offset_map_.end()) {
    // Add names and add to the admin builder
    uint32_t country_offset = AddName(country_name);
    uint32_t state_offset = AddName(state_name);
    admins_builder_.emplace_back(country_offset, state_offset, country_iso, state_iso);

    // Add to the map
    admin_info_offset_map_.emplace(country_iso + state_iso + state_name, admins_builder_.size() - 1);
    return admins_builder_.size() - 1;
  } else {
    // Already have this admin - return the offset
    return existing_admin_info_offset_item->second;
  }
}

// Gets a non-const node from existing tile data.
NodeInfo& GraphTileBuilder::node(const size_t idx) {
  if (idx < header_->nodecount()) {
    return nodes_[idx];
  }
  throw std::runtime_error("GraphTileBuilder NodeInfo index out of bounds");
}

// Get the node builder at the specified index.
NodeInfo& GraphTileBuilder::node_builder(const size_t idx) {
  if (idx < header_->nodecount()) {
    return nodes_builder_[idx];
  }
  throw std::runtime_error("GraphTileBuilder NodeInfo index out of bounds");
}

// Gets a non-const directed edge from existing tile data.
DirectedEdge& GraphTileBuilder::directededge(const size_t idx) {
  if (idx < header_->directededgecount()) {
    return directededges_[idx];
  }
  throw std::runtime_error("GraphTile DirectedEdge id out of bounds");
}

// Gets a non-const directed edge extension from existing tile data.
DirectedEdgeExt& GraphTileBuilder::directededge_ext(const size_t idx) {
  if (idx < header_->directededgecount()) {
    return ext_directededges_[idx];
  }
  throw std::runtime_error("GraphTile DirectedEdgeExt id out of bounds");
}

// Gets a pointer to directed edges within the list being built.
const DirectedEdge* GraphTileBuilder::directededges(const size_t idx) const {
  if (idx < header_->directededgecount()) {
    return &directededges_builder_[idx];
  }
  throw std::runtime_error("GraphTile DirectedEdge id out of bounds");
}

// Gets a pointer to directed edge extensions within the list being built.
const DirectedEdgeExt* GraphTileBuilder::directededges_ext(const size_t idx) const {
  if (idx < header_->directededgecount()) {
    return &directededges_ext_builder_[idx];
  }
  throw std::runtime_error("GraphTile DirectedEdgeExt id out of bounds");
}

// Get the directed edge builder at the specified index.
DirectedEdge& GraphTileBuilder::directededge_builder(const size_t idx) {
  if (idx < header_->directededgecount()) {
    return directededges_builder_[idx];
  }
  throw std::runtime_error("GraphTile DirectedEdge id out of bounds");
}

// Get the directed edge extension builder at the specified index.
DirectedEdgeExt& GraphTileBuilder::directededge_ext_builder(const size_t idx) {
  if (idx < header_->directededgecount()) {
    return directededges_ext_builder_[idx];
  }
  throw std::runtime_error("GraphTile DirectedEdgeExt id out of bounds");
}

// Gets a non-const access restriction from existing tile data.
AccessRestriction& GraphTileBuilder::accessrestriction(const size_t idx) {
  if (idx < header_->access_restriction_count()) {
    return access_restrictions_[idx];
  }
  throw std::runtime_error("GraphTileBuilder access restriction index is out of bounds");
}

// Gets an access restriction builder at the specified index.
AccessRestriction& GraphTileBuilder::accessrestriction_builder(const size_t idx) {
  if (idx < header_->access_restriction_count()) {
    return access_restriction_builder_[idx];
  }
  throw std::runtime_error("GraphTileBuilder access restriction index is out of bounds");
}

// Gets a non-const sign from existing tile data.
valhalla::baldr::Sign& GraphTileBuilder::sign(const size_t idx) {
  if (idx < header_->signcount()) {
    return signs_[idx];
  }
  throw std::runtime_error("GraphTileBuilder sign index is out of bounds");
}

// Gets a sign builder at the specified index.
valhalla::baldr::Sign& GraphTileBuilder::sign_builder(const size_t idx) {
  if (idx < header_->signcount()) {
    return signs_builder_[idx];
  }
  throw std::runtime_error("GraphTileBuilder sign index is out of bounds");
}

// Gets a turn lane at the specified index.
TurnLanes& GraphTileBuilder::turnlane_builder(const size_t idx) {
  if (idx < header_->turnlane_count()) {
    return turnlanes_[idx];
  }
  throw std::runtime_error("GraphTileBuilder turn lane index is out of bounds");
}

// Add turn lanes for a directed edge
void GraphTileBuilder::AddTurnLanes(const uint32_t idx, const std::string& str) {
  if (!str.empty()) {
    uint32_t offset = AddName(str);
    turnlanes_builder_.emplace_back(idx, offset);
  }
}

// Add turn lanes idx for a directed edge
void GraphTileBuilder::AddTurnLanes(const uint32_t idx, const uint32_t tl_idx) {
  turnlanes_builder_.emplace_back(idx, tl_idx);
}

// Add turn lanes
void GraphTileBuilder::AddTurnLanes(const std::vector<TurnLanes>& turn_lanes) {
  turnlanes_builder_.clear();
  turnlanes_builder_ = turn_lanes;
}

// Gets a const admin builder at specified index.
const Admin& GraphTileBuilder::admins_builder(size_t idx) {
  if (idx < admins_builder_.size()) {
    return admins_builder_.at(idx);
  }
  throw std::runtime_error("GraphTileBuilder admin index is out of bounds");
}

// Add the tile creation date
void GraphTileBuilder::AddTileCreationDate(const uint32_t tile_creation_date) {
  header_builder_.set_date_created(tile_creation_date);
}

// return this tiles' edges' bins and its edges' tweeners' bins
using tweeners_t = std::unordered_map<GraphId, std::array<std::vector<GraphId>, kBinCount>>;
std::array<std::vector<GraphId>, kBinCount> GraphTileBuilder::BinEdges(const graph_tile_ptr& tile,
                                                                       tweeners_t& tweeners) {
  assert(tile);
  std::array<std::vector<GraphId>, kBinCount> bins;
  // we store these at the highest level
  auto max_level = TileHierarchy::levels().back().level;
  // skip transit or other special levels and empty tiles
  if (tile->header()->graphid().level() > max_level || tile->header()->directededgecount() == 0) {
    return bins;
  }
  // is this the highest level
  auto max = tile->header()->graphid().level() == max_level;
  const auto& tiles = TileHierarchy::levels().back().tiles;

  // each edge please
  std::unordered_set<uint64_t> ids(tile->header()->directededgecount() / 2);
  const auto* start_edge = tile->directededge(0);
  for (const DirectedEdge* edge = start_edge; edge < start_edge + tile->header()->directededgecount();
       ++edge) {
    // dont bin these
    if (edge->use() == Use::kTransitConnection || edge->use() == Use::kPlatformConnection ||
        edge->use() == Use::kEgressConnection) {
      continue;
    }

    // get the shape or bail if none
    auto info = tile->edgeinfo(edge);
    const auto& shape = info.shape();
    if (shape.empty()) {
      continue;
    }

    // avoid duplicates and minimize leaving a tile for shape by:
    // writing the edge to the tile it originates in
    // not writing the edge to the tile it terminates in
    // writing the edge to tweeners if originating < terminating or the edge leaves and comes back
    auto start_id = tiles.TileId(edge->forward() ? shape.front() : shape.back());
    auto end_id = tiles.TileId(edge->forward() ? shape.back() : shape.front());
    auto intermediate = start_id < end_id;

    // if this starts and ends in the same tile and we've seen it already we can skip it
    if (start_id == end_id && !ids.insert(edge->edgeinfo_offset()).second) {
      continue;
    }

    // for each bin that got intersected
    auto intersection = tiles.Intersect(shape);
    GraphId edge_id(tile->header()->graphid().tileid(), tile->header()->graphid().level(),
                    edge - start_edge);
    for (const auto& i : intersection) {
      // as per the rules above about when to add intersections
      auto originating = i.first == start_id;
      auto terminating = i.first == end_id;
      auto loop_back = i.first != start_id && i.first != end_id && start_id == end_id;
      if (originating || (intermediate && !terminating) || loop_back) {
        // which set of bins, either this local set or tweeners to be added later
        auto& out_bins = originating && max
                             ? bins
                             : tweeners.insert({GraphId(i.first, max_level, 0), {}}).first->second;
        // keep the edge id
        for (auto bin : i.second) {
          out_bins[bin].push_back(edge_id);
        }
      }
    }
  }

  // give back this tiles bins
  return bins;
}

void GraphTileBuilder::AddBins(const std::string& tile_dir,
                               const graph_tile_ptr& tile,
                               const std::array<std::vector<GraphId>, kBinCount>& more_bins) {
  assert(tile);
  // read bins and append and keep track of how much is appended
  std::vector<GraphId> bins[kBinCount];
  uint32_t shift = 0;
  for (size_t i = 0; i < kBinCount; ++i) {
    auto bin = tile->GetBin(i % kBinsDim, i / kBinsDim);
    bins[i].assign(bin.begin(), bin.end());
    bins[i].insert(bins[i].end(), more_bins[i].cbegin(), more_bins[i].cend());
    shift += more_bins[i].size();
  }
  shift *= sizeof(GraphId);
  // update header bin indices
  uint32_t offsets[kBinCount] = {static_cast<uint32_t>(bins[0].size())};
  for (size_t i = 1; i < kBinCount; ++i) {
    offsets[i] = static_cast<uint32_t>(bins[i].size()) + offsets[i - 1];
  }
  // update header offsets
  // NOTE: if format changes to add more things here we need to make a change here as well
  GraphTileHeader header = *tile->header();
  header.set_edge_bin_offsets(offsets);
  header.set_complex_restriction_forward_offset(header.complex_restriction_forward_offset() + shift);
  header.set_complex_restriction_reverse_offset(header.complex_restriction_reverse_offset() + shift);
  header.set_edgeinfo_offset(header.edgeinfo_offset() + shift);
  header.set_textlist_offset(header.textlist_offset() + shift);
  header.set_lane_connectivity_offset(header.lane_connectivity_offset() + shift);
  header.set_end_offset(header.end_offset() + shift);
  // rewrite the tile
  filesystem::path filename =
      tile_dir + filesystem::path::preferred_separator + GraphTile::FileSuffix(header.graphid());
  if (!filesystem::exists(filename.parent_path())) {
    filesystem::create_directories(filename.parent_path());
  }
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  // open it
  if (file.is_open()) {
    // new header
    file.write(reinterpret_cast<const char*>(&header), sizeof(GraphTileHeader));
    // a bunch of stuff between header and bins
    const auto* begin = reinterpret_cast<const char*>(tile->header()) + sizeof(GraphTileHeader);
    const auto* end = reinterpret_cast<const char*>(tile->GetBin(0, 0).begin());
    file.write(begin, end - begin);
    // the updated bins
    for (const auto& bin : bins) {
      file.write(reinterpret_cast<const char*>(bin.data()), bin.size() * sizeof(GraphId));
    }
    // the rest of the stuff after bins
    begin = reinterpret_cast<const char*>(tile->GetBin(kBinsDim - 1, kBinsDim - 1).end());
    end = reinterpret_cast<const char*>(tile->header()) + tile->header()->end_offset();
    file.write(begin, end - begin);
  } // failed
  else {
    throw std::runtime_error("Failed to open file " + filename.string());
  }
}

// Add a predicted speed profile for a directed edge.
void GraphTileBuilder::AddPredictedSpeed(const uint32_t idx,
                                         const std::array<int16_t, kCoefficientCount>& coefficients,
                                         const size_t predicted_count_hint) {
  if (idx >= header_->directededgecount())
    throw std::runtime_error("GraphTileBuilder AddPredictedSpeed index is out of bounds");

  // On the first call, create both the place to store the indices for each edge into the speed data
  // But also preallocate space to hold the actual predicted data so the insert doesnt do reallocs
  if (speed_profile_offset_builder_.size() == 0) {
    speed_profile_offset_builder_.resize(header_->directededgecount());
    speed_profile_builder_.reserve(predicted_count_hint * kCoefficientCount);
  }

  // Set the offset to the predicted speed profile for this directed edge
  speed_profile_offset_builder_[idx] = speed_profile_builder_.size();

  // Append the profile
  speed_profile_builder_.insert(speed_profile_builder_.end(), coefficients.begin(),
                                coefficients.end());
}

// Updates a tile with predictive speed data. Also updates directed edges with
// free flow and constrained flow speeds and the predicted traffic flag. The
// predicted traffic is written after turn lane data.
void GraphTileBuilder::UpdatePredictedSpeeds(const std::vector<DirectedEdge>& directededges) {

  // Even if there are no predicted speeds there still may be updated directed edges
  // with free flow or constrained flow speeds - so don't return if no speed profiles

  // Get the name of the file
  filesystem::path filename = tile_dir_ + filesystem::path::preferred_separator +
                              GraphTile::FileSuffix(header_builder_.graphid());

  // Make sure the directory exists on the system
  if (!filesystem::exists(filename.parent_path()))
    filesystem::create_directories(filename.parent_path());

  // Open file and truncate
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (file.is_open()) {
    // Write a new header - add the offset to predicted speed data and the profile count.
    // Update the end offset (shift by the amount of predicted speed data added).
    size_t offset = header_->end_offset();
    header_builder_.set_end_offset(header_->end_offset() +
                                   (speed_profile_offset_builder_.size() * sizeof(uint32_t)) +
                                   (speed_profile_builder_.size() * sizeof(int16_t)));
    header_builder_.set_predictedspeeds_offset(offset);
    header_builder_.set_predictedspeeds_count(speed_profile_builder_.size() / kCoefficientCount);
    file.write(reinterpret_cast<const char*>(&header_builder_), sizeof(GraphTileHeader));

    // Copy the nodes (they are unchanged when adding predicted speeds).
    file.write(reinterpret_cast<const char*>(nodes_), header_->nodecount() * sizeof(NodeInfo));

    // Copy the node transitions (they are unchanged when adding predicted speeds).
    file.write(reinterpret_cast<const char*>(transitions_),
               header_->transitioncount() * sizeof(NodeTransition));

    // Write the updated directed edges. Make sure edge count matches.
    if (directededges.size() != header_->directededgecount()) {
      throw std::runtime_error("GraphTileBuilder::Update - directed edge count has changed");
    }
    file.write(reinterpret_cast<const char*>(directededges.data()),
               directededges.size() * sizeof(DirectedEdge));

    // Write out data from access restrictions to the end of lane connectivity data.
    auto begin = reinterpret_cast<const char*>(&access_restrictions_[0]);
    auto end = reinterpret_cast<const char*>(header()) + offset;
    file.write(begin, end - begin);

    // Append the speed profile indexes and profiles.
    file.write(reinterpret_cast<const char*>(speed_profile_offset_builder_.data()),
               speed_profile_offset_builder_.size() * sizeof(uint32_t));
    file.write(reinterpret_cast<const char*>(speed_profile_builder_.data()),
               speed_profile_builder_.size() * sizeof(int16_t));

    // Write the rest of the tiles. TBD (if anything is added after the speed profiles
    // then this will need to be updated)

    // Close the file
    file.close();
  }
}

} // namespace mjolnir
} // namespace valhalla
