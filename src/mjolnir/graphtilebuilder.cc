#include "mjolnir/graphtilebuilder.h"

#include "midgard/logging.h"
#include "baldr/datetime.h"
#include "baldr/edgeinfo.h"
#include "baldr/tilehierarchy.h"
#include <boost/format.hpp>
#include <boost/filesystem/operations.hpp>
#include <stdexcept>
#include <list>
#include <algorithm>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Constructor given an existing tile. This is used to read in the tile
// data and then add to it (e.g. adding node connections between hierarchy
// levels. If the deserialize flag is set then all objects are serialized
// from memory into builders that can be added to and then stored using
// StoreTileData.
GraphTileBuilder::GraphTileBuilder(const std::string& tile_dir,
                                   const GraphId& graphid, bool deserialize)
    : tile_dir_(tile_dir),
      GraphTile(tile_dir, graphid) {

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
    AddAdmin("None","None","","");
    return;
  }

  // Street name info. Unique set of offsets into the text list
  std::set<NameInfo> name_info;
  name_info.insert({0});

  // Create vectors of the fixed size objects
  size_t n = header_->nodecount();
  nodes_builder_.resize(n);
  memcpy(&nodes_builder_[0], nodes_, n * sizeof(NodeInfo));
  n = header_->directededgecount();
  directededges_builder_.resize(n);
  memcpy(&directededges_builder_[0], directededges_, n * sizeof(DirectedEdge));

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
    signs_builder_.emplace_back(signs_[i].edgeindex(), signs_[i].type(),
                                signs_[i].text_offset());
  }

  // Create admin builders
  for (uint32_t i = 0; i < header_->admincount(); i++) {
    admins_builder_.emplace_back(admins_[i].country_offset(),
                admins_[i].state_offset(), admins_[i].country_iso(),
                admins_[i].state_iso());
    name_info.insert({admins_[i].country_offset()});
    name_info.insert({admins_[i].state_offset()});
  }

  // Edge bins are gotten by parent

  // Create an ordered set of edge info offsets
  std::set<uint32_t> edge_info_offsets;
  for (auto& diredge : directededges_builder_) {
    edge_info_offsets.insert(diredge.edgeinfo_offset());
  }

  // create the forward list of complex restrictions.
  complex_restriction_forward_list_offset_ = 0;
  while (complex_restriction_forward_list_offset_ <
      complex_restriction_forward_size_) {

    ComplexRestriction cr(complex_restriction_forward_ + complex_restriction_forward_list_offset_);

    ComplexRestrictionBuilder crb;
    crb.set_from_id(cr.from_id());
    crb.set_to_id(cr.to_id());
    /** TODO - design common date/time structure
    crb.set_begin_time(cr.begin_time());
    crb.set_elapsed_time(cr.end_time());
    crb.set_begin_day(cr.begin_day());
    crb.set_end_day(cr.end_day());
    */
    crb.set_via_list(cr.GetVias());
    crb.set_modes(cr.modes());

    complex_restriction_forward_list_offset_ += crb.SizeOf();
    complex_restriction_forward_builder_.emplace_back(std::move(crb));
  }

  if (complex_restriction_forward_list_offset_ != complex_restriction_forward_size_) {
    LOG_WARN("GraphTileBuilder TileID: " +
          std::to_string(header_->graphid().tileid()) +
          " offsets are off for complex restrictions: = " +
          std::to_string(complex_restriction_forward_list_offset_) +
          " size = " + std::to_string(complex_restriction_forward_size_));
  }

  // create the reverse list of complex restrictions.
  complex_restriction_reverse_list_offset_ = 0;
  while (complex_restriction_reverse_list_offset_ < complex_restriction_reverse_size_) {

    ComplexRestriction cr(complex_restriction_reverse_ + complex_restriction_reverse_list_offset_);

    ComplexRestrictionBuilder crb;
    crb.set_from_id(cr.from_id());
    crb.set_to_id(cr.to_id());
    /** TODO
    crb.set_begin_time(cr.begin_time());
    crb.set_elapsed_time(cr.end_time());
    crb.set_begin_day(cr.begin_day());
    crb.set_end_day(cr.end_day());
    */
    crb.set_via_list(cr.GetVias());
    crb.set_modes(cr.modes());

    complex_restriction_reverse_list_offset_ += crb.SizeOf();
    complex_restriction_reverse_builder_.emplace_back(std::move(crb));
  }

  if (complex_restriction_reverse_list_offset_ != complex_restriction_reverse_size_) {
    LOG_WARN("GraphTileBuilder TileID: " +
          std::to_string(header_->graphid().tileid()) +
          " offsets are off for complex restrictions: = " +
          std::to_string(complex_restriction_reverse_list_offset_) +
          " size = " + std::to_string(complex_restriction_reverse_size_));
  }

  // EdgeInfo. Create list of EdgeInfoBuilders. Add to text offset set.
  edge_info_offset_ = 0;
  for (auto offset : edge_info_offsets) {
    // Verify the offsets match as we create the edge info builder list
    if (offset != edge_info_offset_) {
      LOG_WARN("GraphTileBuilder TileID: " +
            std::to_string(header_->graphid().tileid()) +
            " offset stored in directed edge: = " + std::to_string(offset) +
            " current ei offset= " + std::to_string(edge_info_offset_));
    }

    EdgeInfo ei(edgeinfo_ + offset, textlist_, textlist_size_);
    EdgeInfoBuilder eib;
    eib.set_wayid(ei.wayid());
    for (uint32_t nm = 0; nm < ei.name_count(); nm++) {
      NameInfo info = ei.GetNameInfo(nm);
      name_info.insert(info);
      eib.AddNameInfo(info);
    }
    eib.set_encoded_shape(ei.encoded_shape());
    edge_info_offset_ += eib.SizeOf();
    edgeinfo_list_.emplace_back(std::move(eib));
  }

  // Text list
  for (auto ni : name_info) {
    // Verify offsets as we add text. Identify any strings in the text list
    // that are not referenced by any objects.
    while (ni.name_offset_ != text_list_offset_) {
      std::string unused_string(textlist_ + text_list_offset_);
      textlistbuilder_.push_back(unused_string);
      text_offset_map_.emplace(unused_string, text_list_offset_);
      text_list_offset_ += unused_string.length() + 1;
      LOG_WARN("Unused text string: " + unused_string);
    }
    std::string str(textlist_ + ni.name_offset_);
    textlistbuilder_.push_back(str);
    uint32_t offset = ni.name_offset_;
    text_offset_map_.emplace(str, offset);
    text_list_offset_ += str.length() + 1;
  }

  // Lane connectivity
  lane_connectivity_offset_ = lane_connectivity_size_;
  n = lane_connectivity_size_ / sizeof(LaneConnectivity);
  lane_connectivity_builder_.reserve(n);
  std::copy(lane_connectivity_, lane_connectivity_ + n,
    std::back_inserter(lane_connectivity_builder_));

  // Edge elevation
  if (header_->has_edge_elevation()) {
    // Edge elevation count is the same as the directed edge count
    n = header_->directededgecount();
    edge_elevation_builder_.reserve(n);
    std::copy(edge_elevation_, edge_elevation_ + n,
        std::back_inserter(edge_elevation_builder_));
  }
}

// Output the tile to file. Stores as binary data.
void GraphTileBuilder::StoreTileData() {
  // Get the name of the file
  boost::filesystem::path filename = tile_dir_ + '/'
      + GraphTile::FileSuffix(header_builder_.graphid());

  // Make sure the directory exists on the system
  if (!boost::filesystem::exists(filename.parent_path()))
    boost::filesystem::create_directories(filename.parent_path());

  // Open file and truncate
  std::stringstream in_mem;
  std::ofstream file(filename.c_str(),
                     std::ios::out | std::ios::binary | std::ios::trunc);
  if (file.is_open()) {
    // Write the nodes
    header_builder_.set_nodecount(nodes_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(&nodes_builder_[0]),
               nodes_builder_.size() * sizeof(NodeInfo));

    // Write the directed edges
    header_builder_.set_directededgecount(directededges_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(&directededges_builder_[0]),
               directededges_builder_.size() * sizeof(DirectedEdge));

    // Sort and write the access restrictions
    header_builder_.set_access_restriction_count(access_restriction_builder_.size());
    std::sort(access_restriction_builder_.begin(), access_restriction_builder_.end());
    in_mem.write(reinterpret_cast<const char*>(&access_restriction_builder_[0]),
               access_restriction_builder_.size() * sizeof(AccessRestriction));

    // Sort and write the transit departures
    header_builder_.set_departurecount(departure_builder_.size());
    std::sort(departure_builder_.begin(), departure_builder_.end());
    in_mem.write(reinterpret_cast<const char*>(&departure_builder_[0]),
               departure_builder_.size() * sizeof(TransitDeparture));

    // Sort write the transit stops
    header_builder_.set_stopcount(stop_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(&stop_builder_[0]),
               stop_builder_.size() * sizeof(TransitStop));

    // Write the transit routes
    header_builder_.set_routecount(route_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(&route_builder_[0]),
               route_builder_.size() * sizeof(TransitRoute));

    // Write transit schedules
    header_builder_.set_schedulecount(schedule_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(&schedule_builder_[0]),
               schedule_builder_.size() * sizeof(TransitSchedule));

    // TODO add transfers later
    header_builder_.set_transfercount(0);

    // Write the signs
    header_builder_.set_signcount(signs_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(&signs_builder_[0]),
               signs_builder_.size() * sizeof(Sign));

    // Write the admins
    header_builder_.set_admincount(admins_builder_.size());
    in_mem.write(reinterpret_cast<const char*>(&admins_builder_[0]),
               admins_builder_.size() * sizeof(Admin));

    // Edge bins can only be added after you've stored the tile

    // Write the forward complex restriction data
    header_builder_.set_complex_restriction_forward_offset(
         (sizeof(GraphTileHeader))
             + (nodes_builder_.size() * sizeof(NodeInfo))
             + (directededges_builder_.size() * sizeof(DirectedEdge))
             + (access_restriction_builder_.size() * sizeof(AccessRestriction))
             + (departure_builder_.size() * sizeof(TransitDeparture))
             + (stop_builder_.size() * sizeof(TransitStop))
             + (route_builder_.size() * sizeof(TransitRoute))
             + (schedule_builder_.size() * sizeof(TransitSchedule))
             // TODO - once transit transfers are added need to update here
             + (signs_builder_.size() * sizeof(Sign))
             + (admins_builder_.size() * sizeof(Admin)));
    for (const auto& complex_restriction : complex_restriction_forward_builder_)
      in_mem << complex_restriction;

    // Write the reverse complex restriction data
    header_builder_.set_complex_restriction_reverse_offset(
            header_builder_.complex_restriction_forward_offset() +
            complex_restriction_forward_list_offset_);
    for (const auto& complex_restriction : complex_restriction_reverse_builder_)
      in_mem << complex_restriction;

    // Write the edge data
    header_builder_.set_edgeinfo_offset(
            header_builder_.complex_restriction_reverse_offset() +
            complex_restriction_reverse_list_offset_);
    for (const auto& edgeinfo : edgeinfo_list_)
      in_mem << edgeinfo;

    // Write the names
    header_builder_.set_textlist_offset(
            header_builder_.edgeinfo_offset() + edge_info_offset_);
    for (const auto& text : textlistbuilder_)
      in_mem << text << '\0';

    // Add padding (if needed) to align to 8-byte word.
    int tmp = in_mem.tellp() % 8;
    int padding = (tmp > 0) ? 8 - tmp : 0;
    if (padding > 0 && padding < 8) {
      in_mem.write("\0\0\0\0\0\0\0\0", padding);
    }

    // At this point there is no traffic segment data, so set traffic segment
    // Id offset and chunk offset to same value
    header_builder_.set_traffic_id_count(0);
    header_builder_.set_traffic_segmentid_offset(header_builder_.textlist_offset() +
                                                 text_list_offset_ + padding);
    header_builder_.set_traffic_chunk_offset(header_builder_.traffic_segmentid_offset());

    // Write lane connections
    header_builder_.set_lane_connectivity_offset(header_builder_.traffic_chunk_offset());
    std::sort(lane_connectivity_builder_.begin(), lane_connectivity_builder_.end());
    in_mem.write(reinterpret_cast<const char*>(&lane_connectivity_builder_[0]),
               lane_connectivity_builder_.size() * sizeof(LaneConnectivity));

    // Write the edge elevation data. Make sure that if it exists it has
    // the same count as directed edges.
    header_builder_.set_edge_elevation_offset(header_builder_.lane_connectivity_offset() +
       (lane_connectivity_builder_.size() * sizeof(LaneConnectivity)));
    if (edge_elevation_builder_.size() > 0) {
      if (edge_elevation_builder_.size() != directededges_builder_.size()) {
        LOG_ERROR("Edge elevation count is not equal to directed edge count!");
      }
      header_builder_.set_has_edge_elevation(true);
      in_mem.write(reinterpret_cast<const char*>(&edge_elevation_builder_[0]),
                         edge_elevation_builder_.size() * sizeof(EdgeElevation));
    }

    // Set the end offset
    header_builder_.set_end_offset(header_builder_.edge_elevation_offset() +
      (edge_elevation_builder_.size() * sizeof(EdgeElevation)));

    // Sanity check for the end offset
    uint32_t curr = static_cast<uint32_t>(in_mem.tellp()) +
                    static_cast<uint32_t>(sizeof(GraphTileHeader));
    if (header_builder_.end_offset() != curr) {
      LOG_ERROR("Mismatch in end offset " + std::to_string(header_builder_.end_offset()) +
                " vs in_mem stream " + std::to_string(curr) + " padding = " + std::to_string(padding));
    }

    LOG_DEBUG((boost::format("Write: %1% nodes = %2% directededges = %3% signs %4% edgeinfo offset = %5% textlist offset = %6% lane connections = %7%" )
      % filename % nodes_builder_.size() % directededges_builder_.size() % signs_builder_.size() % edge_info_offset_ % text_list_offset_ % lane_connectivity_builder_.size()).str());
    LOG_DEBUG((boost::format("   admins = %1%  departures = %2% stops = %3% routes = %4%" )
      % admins_builder_.size() % departure_builder_.size() % stop_builder_.size() % route_builder_.size()).str());

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
  boost::filesystem::path filename = tile_dir_ + '/' +
        GraphTile::FileSuffix(header_->graphid());

  // Make sure the directory exists on the system
  if (!boost::filesystem::exists(filename.parent_path()))
    boost::filesystem::create_directories(filename.parent_path());

  // Open file. Truncate so we replace the contents.
  std::ofstream file(filename.c_str(),
                     std::ios::out | std::ios::binary | std::ios::trunc);
  if (file.is_open()) {
    // Write the header
    file.write(reinterpret_cast<const char*>(header_), sizeof(GraphTileHeader));

    // Write the updated nodes. Make sure node count matches.
    if (nodes.size() != header_->nodecount()) {
      throw std::runtime_error("GraphTileBuilder::Update - node count has changed");
    }
    file.write(reinterpret_cast<const char*>(&nodes[0]),
               nodes.size() * sizeof(NodeInfo));

    // Write the updated directed edges. Make sure edge count matches.
    if (directededges.size() != header_->directededgecount()) {
      throw std::runtime_error("GraphTileBuilder::Update - directed edge count has changed");
    }
    file.write(reinterpret_cast<const char*>(&directededges[0]),
               directededges.size() * sizeof(DirectedEdge));

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

// Add a transit departure.
void GraphTileBuilder::AddTransitDeparture(const TransitDeparture& departure) {
  departure_builder_.emplace_back(std::move(departure));
}

// Add a transit stop.
void GraphTileBuilder::AddTransitStop(const TransitStop& stop)  {
  stop_builder_.emplace_back(std::move(stop));
}

// Add a transit route.
void GraphTileBuilder::AddTransitRoute(const TransitRoute& route)  {
  route_builder_.emplace_back(std::move(route));
}

// Add a transit schedule.
void GraphTileBuilder::AddTransitSchedule(const TransitSchedule& schedule)  {
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
                                const std::vector<SignInfo>& signs) {
  // Iterate through the list of sign info (with sign text) and add sign
  // text to the text list. Skip signs with no text.
  for (const auto& sign : signs) {
    if (!(sign.text().empty())) {
      uint32_t offset = AddName(sign.text());
      signs_builder_.emplace_back(idx, sign.type(), offset);
    }
  }
}

// Add lane connectivity
void GraphTileBuilder::AddLaneConnectivity(const std::vector<baldr::LaneConnectivity>& lc)
{
  lane_connectivity_builder_.insert(lane_connectivity_builder_.end(), lc.begin(), lc.end());
  lane_connectivity_offset_ += sizeof(baldr::LaneConnectivity) * lc.size();
}

bool GraphTileBuilder::HasEdgeInfo(const uint32_t edgeindex, const baldr::GraphId& nodea,
                     const baldr::GraphId& nodeb, uint32_t& edge_info_offset) {
  auto edge_tuple_item = EdgeTuple(edgeindex, nodea, nodeb);
  auto existing_edge_offset_item = edge_offset_map_.find(edge_tuple_item);
  if (existing_edge_offset_item != edge_offset_map_.cend()) {
    edge_info_offset = existing_edge_offset_item->second;
    return true;
  }
  return false;
}

// Add the complex restrictions and update the list offset.  The to, from, and vias should all point to edgeids.
void GraphTileBuilder::UpdateComplexRestrictions(const std::list<ComplexRestrictionBuilder>& complex_restriction_builder,
                                                 const bool forward) {

  if (forward) {
    complex_restriction_forward_list_offset_ = 0;
    complex_restriction_forward_builder_.clear();
    // Add a new complex restrictions to the lists
    complex_restriction_forward_builder_ = complex_restriction_builder;

    for (const auto& crb : complex_restriction_builder) {
      // Update edge offset for next item
      complex_restriction_forward_list_offset_ += crb.SizeOf();
    }
  } else {
    complex_restriction_reverse_list_offset_ = 0;
    complex_restriction_reverse_builder_.clear();
    // Add a new complex restrictions to the lists
    complex_restriction_reverse_builder_ = complex_restriction_builder;

    for (const auto& crb : complex_restriction_builder) {
      // Update edge offset for next item
      complex_restriction_reverse_list_offset_ += crb.SizeOf();
    }
  }
}

// Add edge info
template <class shape_container_t>
uint32_t GraphTileBuilder::AddEdgeInfo(const uint32_t edgeindex,
                                       const GraphId& nodea,
                                       const baldr::GraphId& nodeb,
                                       const uint64_t wayid,
                                       const shape_container_t& lls,
                                       const std::vector<std::string>& names,
                                       bool& added) {
  // If we haven't yet added edge info for this edge tuple
  auto edge_tuple_item = EdgeTuple(edgeindex, nodea, nodeb);
  auto existing_edge_offset_item = edge_offset_map_.find(edge_tuple_item);
  if (existing_edge_offset_item == edge_offset_map_.end()) {
    // Add a new EdgeInfo to the list and get a reference to it
    edgeinfo_list_.emplace_back();
    EdgeInfoBuilder& edgeinfo = edgeinfo_list_.back();
    edgeinfo.set_wayid(wayid);
    edgeinfo.set_shape(lls);

    // Add names to the common text/name list. Skip blank names.
    std::vector<NameInfo> name_info_list;
    name_info_list.reserve(std::min(names.size(), kMaxNamesPerEdge));
    size_t name_count = 0;
    for (const auto& name : names) {
      // Stop adding names if max count has been reached
      if (name_count == kMaxNamesPerEdge) {
        LOG_WARN("Too many names for edgeindex: " + std::to_string(edgeindex));
        break;
      }

      // Verify name is not empty
      if (!(name.empty())) {
        // Add name and add its offset to edge info's list.
        NameInfo ni({AddName(name)});
        name_info_list.emplace_back(ni);
        ++name_count;
      }
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
template uint32_t GraphTileBuilder::AddEdgeInfo<std::vector<PointLL> >
  (const uint32_t edgeindex, const GraphId&, const baldr::GraphId&,const uint64_t,
   const std::vector<PointLL>&, const std::vector<std::string>&, bool&);
template uint32_t GraphTileBuilder::AddEdgeInfo<std::list<PointLL> >
  (const uint32_t edgeindex, const GraphId&, const baldr::GraphId&,const uint64_t,
   const std::list<PointLL>&, const std::vector<std::string>&, bool&);

// AddEdgeInfo - accepts an encoded shape string.
uint32_t GraphTileBuilder::AddEdgeInfo(const uint32_t edgeindex,
                     const baldr::GraphId& nodea,
                     const baldr::GraphId& nodeb,
                     const uint64_t wayid,
                     const std::string& llstr,
                     const std::vector<std::string>& names,
                     bool& added){
  // If we haven't yet added edge info for this edge tuple
  auto edge_tuple_item = EdgeTuple(edgeindex, nodea, nodeb);
  auto existing_edge_offset_item = edge_offset_map_.find(edge_tuple_item);
  if (existing_edge_offset_item == edge_offset_map_.end()) {
    // Add a new EdgeInfo to the list and get a reference to it
    edgeinfo_list_.emplace_back();
    EdgeInfoBuilder& edgeinfo = edgeinfo_list_.back();
    edgeinfo.set_wayid(wayid);
    edgeinfo.set_encoded_shape(llstr);

    // Add names to the common text/name list. Skip blank names.
    std::vector<NameInfo> name_info_list;
    name_info_list.reserve(std::min(names.size(), kMaxNamesPerEdge));
    size_t name_count = 0;
    for (const auto& name : names) {
      // Stop adding names if max count has been reached
      if (name_count == kMaxNamesPerEdge) {
        LOG_WARN("Too many names for edgeindex: " + std::to_string(edgeindex));
        break;
      }

      // Verify name is not empty
      if (!(name.empty())) {
        // Add name and add its offset to edge info's list.
        NameInfo ni({AddName(name)});
        name_info_list.emplace_back(ni);
        ++name_count;
      }
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
            const std::string& state_name, const std::string& country_iso,
            const std::string& state_iso) {
  // Check if admin already exists
  auto existing_admin_info_offset_item = admin_info_offset_map_.find(country_iso+state_name);
  if (existing_admin_info_offset_item == admin_info_offset_map_.end()) {
    // Add names and add to the admin builder
    uint32_t country_offset = AddName(country_name);
    uint32_t state_offset   = AddName(state_name);
    admins_builder_.emplace_back(country_offset, state_offset,
                                 country_iso, state_iso);

    // Add to the map
    admin_info_offset_map_.emplace(country_iso+state_name, admins_builder_.size()-1);
    return admins_builder_.size()-1;
  } else {
    // Already have this admin - return the offset
    return existing_admin_info_offset_item->second;
  }
}

// Gets a non-const node from existing tile data.
NodeInfo& GraphTileBuilder::node(const size_t idx) {
  if (idx < header_->nodecount())
    return nodes_[idx];
  throw std::runtime_error("GraphTileBuilder NodeInfo index out of bounds");
}

// Get the node builder at the specified index.
NodeInfo& GraphTileBuilder::node_builder(const size_t idx) {
  if (idx < header_->nodecount())
    return nodes_builder_[idx];
  throw std::runtime_error("GraphTileBuilder NodeInfo index out of bounds");
}

// Gets a non-const directed edge from existing tile data.
DirectedEdge& GraphTileBuilder::directededge(const size_t idx) {
  if (idx < header_->directededgecount())
    return directededges_[idx];
  throw std::runtime_error("GraphTile DirectedEdge id out of bounds");
}

// Gets a pointer to directed edges within the list being built.
const DirectedEdge* GraphTileBuilder::directededges(const size_t idx) {
  if (idx < header_->directededgecount())
    return &directededges_builder_[idx];
  throw std::runtime_error("GraphTile DirectedEdge id out of bounds");
}

// Get the directed edge builder at the specified index.
DirectedEdge& GraphTileBuilder::directededge_builder(const size_t idx) {
  if (idx < header_->directededgecount())
    return directededges_builder_[idx];
  throw std::runtime_error("GraphTile DirectedEdge id out of bounds");
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
  if (idx < header_->access_restriction_count())
     return access_restriction_builder_[idx];
  throw std::runtime_error("GraphTileBuilder access restriction index is out of bounds");
}

// Gets a non-const sign from existing tile data.
Sign& GraphTileBuilder::sign(const size_t idx) {
  if (idx < header_->signcount())
    return signs_[idx];
  throw std::runtime_error("GraphTileBuilder sign index is out of bounds");
}

// Gets a sign builder at the specified index.
Sign& GraphTileBuilder::sign_builder(const size_t idx) {
  if (idx < header_->signcount())
    return signs_builder_[idx];
  throw std::runtime_error("GraphTileBuilder sign index is out of bounds");
}

// Gets a const admin builder at specified index.
const Admin& GraphTileBuilder::admins_builder(size_t idx) {
  if (idx < admins_builder_.size())
    return admins_builder_.at(idx);
  throw std::runtime_error("GraphTileBuilder admin index is out of bounds");
}

// Add the tile creation date
void GraphTileBuilder::AddTileCreationDate(const uint32_t tile_creation_date) {
  header_builder_.set_date_created(tile_creation_date);
}

//return this tiles' edges' bins and its edges' tweeners' bins
using tweeners_t = std::unordered_map<GraphId, std::array<std::vector<GraphId>, kBinCount> >;
std::array<std::vector<GraphId>, kBinCount> GraphTileBuilder::BinEdges(const GraphTile* tile, tweeners_t& tweeners) {
  std::array<std::vector<GraphId>, kBinCount> bins;
  //we store these at the highest level
  auto max_level = TileHierarchy::levels().rbegin()->first;
  //skip transit or other special levels and empty tiles
  if(tile->header()->graphid().level() > max_level || tile->header()->directededgecount() == 0)
    return bins;
  //is this the highest level
  auto max = tile->header()->graphid().level() == max_level;
  auto tiles = TileHierarchy::levels().rbegin()->second.tiles;

  //each edge please
  std::unordered_set<uint64_t> ids(tile->header()->directededgecount() / 2);
  const auto* start_edge = tile->directededge(0);
  for(const DirectedEdge* edge = start_edge; edge < start_edge + tile->header()->directededgecount(); ++edge) {
    //dont bin these
    if(edge->is_shortcut() || edge->IsTransition() || edge->use() == Use::kTransitConnection ||
       edge->use() == Use::kPlatformConnection || edge->use() == Use::kEgressConnection)
      continue;

    //get the shape or bail if none
    auto info = tile->edgeinfo(edge->edgeinfo_offset());
    const auto& shape = info.shape();
    if(shape.empty())
      continue;

    //avoid duplicates and minimize leaving a tile for shape by:
    //writing the edge to the tile it originates in
    //not writing the edge to the tile it terminates in
    //writing the edge to tweeners if originating < terminating or the edge leaves and comes back
    auto start_id = tiles.TileId(edge->forward() ? shape.front() : shape.back());
    auto end_id = tiles.TileId(edge->forward() ? shape.back() : shape.front());
    auto intermediate = start_id < end_id;

    //if this starts and ends in the same tile and we've seen it already we can skip it
    if(start_id == end_id && !ids.insert(edge->edgeinfo_offset()).second)
      continue;

    //for each bin that got intersected
    auto intersection = tiles.Intersect(shape);
    GraphId edge_id(tile->header()->graphid().tileid(), tile->header()->graphid().level(), edge - start_edge);
    for(const auto& i : intersection) {
      //as per the rules above about when to add intersections
      auto originating = i.first == start_id;
      auto terminating = i.first == end_id;
      auto loop_back = i.first != start_id && i.first != end_id && start_id == end_id;
      if(originating || (intermediate && !terminating) || loop_back) {
        //which set of bins, either this local set or tweeners to be added later
        auto& out_bins = originating && max ? bins : tweeners.insert({GraphId(i.first, max_level, 0), {}}).first->second;
        //keep the edge id
        for(auto bin : i.second)
          out_bins[bin].push_back(edge_id);
      }
    }
  }

  //give back this tiles bins
  return bins;
}

void GraphTileBuilder::AddBins(const std::string& tile_dir,
                const GraphTile* tile,
                const std::array<std::vector<GraphId>, kBinCount>& more_bins) {
  //read bins and append and keep track of how much is appended
  std::vector<GraphId> bins[kBinCount];
  uint32_t shift = 0;
  for(size_t i = 0; i < kBinCount; ++i) {
    auto bin = tile->GetBin(i % kBinsDim, i / kBinsDim);
    bins[i].assign(bin.begin(), bin.end());
    bins[i].insert(bins[i].end(), more_bins[i].cbegin(), more_bins[i].cend());
    shift += more_bins[i].size();
  }
  shift *= sizeof(GraphId);
  //update header bin indices
  uint32_t offsets[kBinCount] = { static_cast<uint32_t>(bins[0].size()) };
  for(size_t i = 1 ; i < kBinCount; ++i)
    offsets[i] = static_cast<uint32_t>(bins[i].size()) + offsets[i - 1];
  //update header offsets
  //NOTE: if format changes to add more things here we need to make a change here as well
  GraphTileHeader header = *tile->header();
  header.set_edge_bin_offsets(offsets);
  header.set_complex_restriction_forward_offset(header.complex_restriction_forward_offset() + shift);
  header.set_complex_restriction_reverse_offset(header.complex_restriction_reverse_offset() + shift);
  header.set_edgeinfo_offset(header.edgeinfo_offset() + shift);
  header.set_textlist_offset(header.textlist_offset() + shift);
  header.set_traffic_segmentid_offset(header.traffic_segmentid_offset() + shift);
  header.set_traffic_chunk_offset(header.traffic_chunk_offset() + shift);
  header.set_lane_connectivity_offset(header.lane_connectivity_offset() + shift);
  header.set_edge_elevation_offset(header.edge_elevation_offset() + shift);
  header.set_end_offset(header.end_offset() + shift);
  //rewrite the tile
  boost::filesystem::path filename = tile_dir + '/' + GraphTile::FileSuffix(header.graphid());
  if(!boost::filesystem::exists(filename.parent_path()))
    boost::filesystem::create_directories(filename.parent_path());
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  //open it
  if(file.is_open()) {
    //new header
    file.write(reinterpret_cast<const char*>(&header), sizeof(GraphTileHeader));
    //a bunch of stuff between header and bins
    const auto* begin = reinterpret_cast<const char*>(tile->header()) + sizeof(GraphTileHeader);
    const auto* end = reinterpret_cast<const char*>(tile->GetBin(0, 0).begin());
    file.write(begin, end - begin);
    //the updated bins
    for(const auto& bin : bins)
      file.write(reinterpret_cast<const char*>(bin.data()), bin.size() * sizeof(GraphId));
    //the rest of the stuff after bins
    begin = reinterpret_cast<const char*>(tile->GetBin(kBinsDim - 1, kBinsDim - 1).end());
    end = reinterpret_cast<const char*>(tile->header()) + tile->header()->end_offset();
    file.write(begin, end - begin);
  }//failed
  else
    throw std::runtime_error("Failed to open file " + filename.string());
}

// Initialize traffic segment association. Sizes the traffic segment Id list
// and sets them all to Invalid.
void GraphTileBuilder::InitializeTrafficSegments() {
  if(header_->traffic_id_count()) {
    traffic_segment_builder_.assign(traffic_segments_, traffic_segments_ + header_->traffic_id_count());
  } else {
    traffic_segment_builder_.resize(header_builder_.directededgecount());
  }
}

// Initialize traffic chunks. Copies existing chunks into the chunk builder.
// This is executed before adding "leftovers" and again before adding chunks.
void GraphTileBuilder::InitializeTrafficChunks() {
  // Assign the chunks to its builder counterpart
  if (traffic_chunk_size_ > 0) {
    size_t count = traffic_chunk_size_ / sizeof(TrafficChunk);
    traffic_chunk_builder_.assign(traffic_chunks_, traffic_chunks_ + count);
  }
}

// Add a traffic segment association - used when an edge associates to
// a single traffic segment.
void GraphTileBuilder::AddTrafficSegment(const GraphId& edgeid,
                        const TrafficChunk& seg) {
  // Check if edge Id is within range and part of this tile
  if (edgeid.Tile_Base() != header_builder_.graphid()) {
    LOG_ERROR("AddTrafficSegments - edge does not belong to this tile");
    return;
  }
  if (edgeid.id() >= header_builder_.directededgecount()) {
    LOG_ERROR("AddTrafficSegments - edge is not valid for this tile");
    return;
  }

  // If this segment is in the same tile we have a 1:1 association of an edge
  // to a segment in this tile - create a single segment association
  if (seg.segment_id().Tile_Base() == edgeid.Tile_Base()) {
    traffic_segment_builder_[edgeid.id()] = { seg.segment_id().id(),
          seg.starts_segment(), seg.ends_segment() };
  } else {
    // Associates to a single traffic segment but in a different tile. Set
    // the TrafficAssociation to represent a chunk count (1) and index. Store
    // a single TrafficChunk.
    traffic_segment_builder_[edgeid.id()] = { 1, traffic_chunk_builder_.size() };
    traffic_chunk_builder_.emplace_back(seg);
  }
}

// Add a traffic segment association - used when an edge associates to
// more than one traffic segment.
void GraphTileBuilder::AddTrafficSegments(const baldr::GraphId& edgeid,
                         const std::vector<baldr::TrafficChunk>& segs) {
  // Check if edge Id is within range and part of this tile
  if (edgeid.Tile_Base() != header_builder_.graphid()) {
    LOG_ERROR("AddTrafficSegments - edge does not belong to this tile");
    return;
  }
  if (edgeid.id() >= header_builder_.directededgecount()) {
    LOG_ERROR("AddTrafficSegments - edge is not valid for this tile");
    return;
  }

  // This edge associates to many segments or portions of segments.
  // Set the TrafficAssociation to represent a chunk count and index.
  // Store each TrafficChunk.
  traffic_segment_builder_[edgeid.id()] = { segs.size(), traffic_chunk_builder_.size() };
  for (const auto& seg : segs) {
    traffic_chunk_builder_.push_back(seg);
  }
}

/**
 * Updates a tile with traffic segment and chunk data. UpdateTrafficSegments
 * is called 3 times - first to add segments within the tile, then to add any
 * "leftover" segments for OSMLR segments that cross tiles, and then to add
 * "chunks". Need to make sure the "shift" for offsets to data after the
 * traffic information are only increased by the amount of "new" segments.
 */
void GraphTileBuilder::UpdateTrafficSegments(const bool update_dir_edges) {
  // Get the number of new segments and chunks added with this call.
  uint32_t new_segments = traffic_segment_builder_.size() -
                          header_->traffic_id_count();
  uint32_t new_chunks = traffic_chunk_builder_.size() -
      (header_->lane_connectivity_offset() - header_->traffic_chunk_offset()) / sizeof(TrafficChunk);

  // Update header to include the traffic segment count and update the
  // offset to chunks (based on size of traffic segments).
  // Padding should already be done so we start traffic info on an 8-byte boundary
  header_builder_.set_traffic_id_count(traffic_segment_builder_.size());
  header_builder_.set_traffic_chunk_offset(header_builder_.traffic_segmentid_offset() +
          traffic_segment_builder_.size() * sizeof(TrafficAssociation));

  // Shift offsets to anything that comes after traffic
  uint32_t shift = new_segments * sizeof(TrafficAssociation) + new_chunks * sizeof(TrafficChunk);
  header_builder_.set_lane_connectivity_offset(header_builder_.lane_connectivity_offset() + shift);
  header_builder_.set_edge_elevation_offset(header_builder_.edge_elevation_offset() + shift);
  header_builder_.set_end_offset(header_builder_.end_offset() + shift);

  // Get the name of the file
  boost::filesystem::path filename = tile_dir_ + '/'
      + GraphTile::FileSuffix(header_builder_.graphid());

  // Make sure the directory exists on the system
  if (!boost::filesystem::exists(filename.parent_path()))
    boost::filesystem::create_directories(filename.parent_path());

  // Open file and truncate
  std::stringstream in_mem;
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (file.is_open()) {
    // Write a new header
    file.write(reinterpret_cast<const char*>(&header_builder_), sizeof(GraphTileHeader));

    // Copy the tile contents from nodes to the beginning of traffic segments
    file.write(reinterpret_cast<const char*>(nodes_),
               header_->traffic_segmentid_offset() - sizeof(GraphTileHeader));

    // Append the traffic segment list
    file.write(reinterpret_cast<const char*>(&traffic_segment_builder_[0]),
               traffic_segment_builder_.size() * sizeof(TrafficAssociation));

    // Append the traffic chunks
    file.write(reinterpret_cast<const char*>(&traffic_chunk_builder_[0]),
               traffic_chunk_builder_.size() * sizeof(TrafficChunk));

    // Write rest of the stuff after traffic chunks (includes lane connectivity
    // and edge elevation...so far).
    const auto* begin = reinterpret_cast<const char*>(header_) +
                header_->lane_connectivity_offset();
    const auto* end = reinterpret_cast<const char*>(header_) +
                header_->end_offset();
    file.write(begin, end - begin);

    // Update directed edge flags
    if (update_dir_edges) {
      // Copy directed edges so we can modify them
      uint32_t n = header_->directededgecount();
      directededges_builder_.resize(n);
      memcpy(&directededges_builder_[0], directededges_, n * sizeof(DirectedEdge));

      // Iterate through directed edges and set traffic segment flag for aby
      // that have any traffic segments.
      for (uint32_t i = 0; i < n; i++) {
        const TrafficAssociation& t = traffic_segment_builder_[i];
        if (t.chunk() || t.count() == 1) {
          directededges_builder_[i].set_traffic_seg(true);
        }
      }

      // Write the updated directed edges
      size_t offset = sizeof(GraphTileHeader) + header_->nodecount() * sizeof(NodeInfo);
      file.seekp(offset, std::ios_base::beg);
      file.write(reinterpret_cast<const char*>(&directededges_builder_[0]),
                 directededges_builder_.size() * sizeof(DirectedEdge));
    }

    // Close the file
    file.close();
  }
}

// Gets the current list of directed edge (builders).
std::vector<EdgeElevation>& GraphTileBuilder::edge_elevations() {
  return edge_elevation_builder_;
}

}
}

