#include "mjolnir/graphtilebuilder.h"

#include <valhalla/midgard/logging.h>
#include <boost/format.hpp>
#include <boost/filesystem/operations.hpp>
#include <stdexcept>
#include <list>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Constructor
GraphTileBuilder::GraphTileBuilder()
    : GraphTile() {
  // Add an empty name to the list so offset 0 means blank name
  std::string str = "";
  textlistbuilder_.emplace_back(str);
  text_offset_map_.emplace(str, 0);
  text_list_offset_ = 1;
}

// Constructor given an existing tile. This is used to read in the tile
// data and then add to it (e.g. adding node connections between hierarchy
// levels. If the deserialize flag is set then all objects are serialized
// from memory into builders that can be added to and then stored using
// StoreTileData.
GraphTileBuilder::GraphTileBuilder(const baldr::TileHierarchy& hierarchy,
                                   const GraphId& graphid, bool deserialize)
    : GraphTile(hierarchy, graphid) {
  if (!deserialize) {
    // Done if not deserializing and creating builders for everything
    return;
  }

  // Copy tile header to a builder
  GraphTileHeader existinghdr = *(header_);
  header_builder_ = static_cast<GraphTileHeaderBuilder&>(existinghdr);

  // Unique set of offsets into the text list
  std::set<uint32_t> text_offsets;
  text_offsets.insert(0);

  // Create vectors of the fixed size objects
  size_t n = header_->nodecount();
  nodes_builder_.resize(n);
  memcpy(&nodes_builder_[0], nodes_, n * sizeof(NodeInfo));
  n = header_->directededgecount();
  directededges_builder_.resize(n);
  memcpy(&directededges_builder_[0], directededges_, n * sizeof(DirectedEdge));

  // Create transit builders and add any text offsets to the set
  for (uint32_t i = 0; i < header_->departurecount(); i++) {
    departure_builder_.emplace_back(std::move(departures_[i]));
    text_offsets.insert(departures_[i].headsign_offset());
  }
  for (uint32_t i = 0; i < header_->tripcount(); i++) {
    trip_builder_.emplace_back(std::move(transit_trips_[i]));
    text_offsets.insert(transit_trips_[i].short_name_offset());
    text_offsets.insert(transit_trips_[i].headsign_offset());
  }
  for (uint32_t i = 0; i < header_->stopcount(); i++) {
    stop_builder_.emplace_back(std::move(transit_stops_[i]));
    text_offsets.insert(transit_stops_[i].name_offset());
    text_offsets.insert(transit_stops_[i].desc_offset());
  }
  for (uint32_t i = 0; i < header_->routecount(); i++) {
    route_builder_.emplace_back(std::move(transit_routes_[i]));
    text_offsets.insert(transit_routes_[i].short_name_offset());
    text_offsets.insert(transit_routes_[i].long_name_offset());
    text_offsets.insert(transit_routes_[i].desc_offset());
  }
  for (uint32_t i = 0; i < header_->transfercount(); i++) {
    transfer_builder_.emplace_back(std::move(transit_transfers_[i]));
  }
  for (uint32_t i = 0; i < header_->calendarcount(); i++) {
    exception_builder_.emplace_back(std::move(transit_exceptions_[i]));
  }

  // Create sign builders
  for (uint32_t i = 0; i < header_->signcount(); i++) {
    text_offsets.insert(signs_[i].text_offset());
    signs_builder_.emplace_back(signs_[i].edgeindex(), signs_[i].type(),
                                signs_[i].text_offset());
  }

  // Create admin builders
  for (uint32_t i = 0; i < header_->admincount(); i++) {
    admins_builder_.emplace_back(admins_[i].country_offset(),
                admins_[i].state_offset(), admins_[i].country_iso(),
                admins_[i].state_iso(),admins_[i].start_dst(),
                admins_[i].end_dst());
    text_offsets.insert(admins_[i].country_offset());
    text_offsets.insert(admins_[i].state_offset());
  }

  // Create an ordered set of edge info offsets
  std::set<uint32_t> edge_info_offsets;
  for (auto& diredge : directededges_builder_) {
    edge_info_offsets.insert(diredge.edgeinfo_offset());
  }

  // EdgeInfo. Create list of EdgeInfoBuilders. Add to text offset set.
  edge_info_offset_ = 0;
  for (auto offset : edge_info_offsets) {
    // Verify the offsets match as we create the edge info builder list
    if (offset != edge_info_offset_) {
      LOG_ERROR("GraphTileBuilder TileID: " +
            std::to_string(header_->graphid().tileid()) +
            " offset stored in directed edge: = " + std::to_string(offset) +
            " current ei offset= " + std::to_string(edge_info_offset_));
    }
    EdgeInfo ei(edgeinfo_ + offset, textlist_, textlist_size_);
    EdgeInfoBuilder eib;
    eib.set_wayid(ei.wayid());
    for (uint32_t nm = 0; nm < ei.name_count(); nm++) {
      uint32_t name_offset = ei.GetStreetNameOffset(nm);
      text_offsets.insert(name_offset);
      eib.AddNameOffset(name_offset);
    }
    eib.set_encoded_shape(ei.encoded_shape());
    edge_info_offset_ += eib.SizeOf();
    edgeinfo_list_.emplace_back(std::move(eib));
  }

  // Text list
  for (auto offset : text_offsets) {
    // Verify offsets as we add text
    if (offset != text_list_offset_) {
      LOG_ERROR("Saved offset = " + std::to_string(offset) +
                " text_list_offset_= " +
                 std::to_string(text_list_offset_));
    }
    std::string str(textlist_ + offset);
    textlistbuilder_.push_back(str);
    text_offset_map_.emplace(str, offset);
    text_list_offset_ += str.length() + 1;
  }
}

// Output the tile to file. Stores as binary data.
void GraphTileBuilder::StoreTileData(const baldr::TileHierarchy& hierarchy,
                                     const GraphId& graphid) {
  // Get the name of the file
  boost::filesystem::path filename = hierarchy.tile_dir() + '/'
      + GraphTile::FileSuffix(graphid, hierarchy);

  // Make sure the directory exists on the system
  if (!boost::filesystem::exists(filename.parent_path()))
    boost::filesystem::create_directories(filename.parent_path());

  // Open file and truncate
  std::ofstream file(filename.c_str(),
                     std::ios::out | std::ios::binary | std::ios::trunc);
  if (file.is_open()) {
    // Configure the header
    header_builder_.set_graphid(graphid);
    header_builder_.set_nodecount(nodes_builder_.size());
    header_builder_.set_directededgecount(directededges_builder_.size());
    header_builder_.set_departurecount(departure_builder_.size());
    header_builder_.set_tripcount(trip_builder_.size());
    header_builder_.set_stopcount(stop_builder_.size());
    header_builder_.set_routecount(route_builder_.size());
    header_builder_.set_transfercount(transfer_builder_.size());
    header_builder_.set_calendarcount(exception_builder_.size());
    header_builder_.set_signcount(signs_builder_.size());
    header_builder_.set_admincount(admins_builder_.size());
    header_builder_.set_edgeinfo_offset(
        (sizeof(GraphTileHeaderBuilder))
            + (nodes_builder_.size() * sizeof(NodeInfoBuilder))
            + (directededges_builder_.size() * sizeof(DirectedEdgeBuilder))
            + (departure_builder_.size() * sizeof(TransitDeparture))
            + (trip_builder_.size() * sizeof(TransitTrip))
            + (stop_builder_.size() * sizeof(TransitStop))
            + (route_builder_.size() * sizeof(TransitRoute))
            + (transfer_builder_.size() * sizeof(TransitTransfer))
            + (exception_builder_.size() * sizeof(TransitCalendar))
            + (signs_builder_.size() * sizeof(SignBuilder))
            + (admins_builder_.size() * sizeof(AdminInfoBuilder)));

    header_builder_.set_textlist_offset(
        header_builder_.edgeinfo_offset() + edge_info_offset_);

    // Write the header.
    file.write(reinterpret_cast<const char*>(&header_builder_),
               sizeof(GraphTileHeaderBuilder));

    // Write the nodes
    file.write(reinterpret_cast<const char*>(&nodes_builder_[0]),
               nodes_builder_.size() * sizeof(NodeInfoBuilder));

    // Write the directed edges
    file.write(reinterpret_cast<const char*>(&directededges_builder_[0]),
               directededges_builder_.size() * sizeof(DirectedEdgeBuilder));

    // Sort and write the transit departures
    std::sort(departure_builder_.begin(), departure_builder_.end());
    file.write(reinterpret_cast<const char*>(&departure_builder_[0]),
               departure_builder_.size() * sizeof(TransitDeparture));

    // Sort and write the transit trips
    std::sort(trip_builder_.begin(), trip_builder_.end());
    file.write(reinterpret_cast<const char*>(&trip_builder_[0]),
               trip_builder_.size() * sizeof(TransitTrip));

    // Sort and write the transit stops
    std::sort(stop_builder_.begin(), stop_builder_.end());
    file.write(reinterpret_cast<const char*>(&stop_builder_[0]),
               stop_builder_.size() * sizeof(TransitStop));

    // Sort and write the transit routes
    std::sort(route_builder_.begin(), route_builder_.end());
    file.write(reinterpret_cast<const char*>(&route_builder_[0]),
               route_builder_.size() * sizeof(TransitRoute));

    // Sort and write the transit transfers
    std::sort(transfer_builder_.begin(), transfer_builder_.end());
    file.write(reinterpret_cast<const char*>(&transfer_builder_[0]),
               transfer_builder_.size() * sizeof(TransitTransfer));

    // Sort and write the transit calendar exceptions
    std::sort(exception_builder_.begin(), exception_builder_.end());
    file.write(reinterpret_cast<const char*>(&exception_builder_[0]),
               exception_builder_.size() * sizeof(TransitCalendar));

    // Write the signs
    file.write(reinterpret_cast<const char*>(&signs_builder_[0]),
               signs_builder_.size() * sizeof(SignBuilder));

    // Write the admins
    file.write(reinterpret_cast<const char*>(&admins_builder_[0]),
               admins_builder_.size() * sizeof(AdminInfoBuilder));

    // Write the edge data
    SerializeEdgeInfosToOstream(file);

    // Write the names
    SerializeTextListToOstream(file);

    LOG_DEBUG((boost::format("Write: %1% nodes = %2% directededges = %3% signs %4% edgeinfo offset = %5% textlist offset = %6%" )
      % filename % nodes_builder_.size() % directededges_builder_.size() % signs_builder_.size() % edge_info_offset_ % text_list_offset_).str());
    LOG_DEBUG((boost::format("   admins = %1%  departures = %2% stops = %3% trips %4% routes = %5%" )
      % admins_builder_.size() % departure_builder_.size() % stop_builder_.size() % trip_builder_.size() % route_builder_.size()).str());

    /*   size_t fsize = file.tellp();
     if (fsize % 8 != 0) {
     char empty[8] = {};
     file.write(empty, (8 - (fsize % 8)));
     }
     */
    size_ = file.tellp();
    file.close();
  } else {
    throw std::runtime_error("Failed to open file " + filename.string());
  }
}

// Update a graph tile with new header, nodes, and directed edges.
void GraphTileBuilder::Update(
    const baldr::TileHierarchy& hierarchy, const GraphTileHeaderBuilder& hdr,
    const std::vector<NodeInfoBuilder>& nodes,
    const std::vector<DirectedEdgeBuilder>& directededges) {

  // Get the name of the file
  boost::filesystem::path filename = hierarchy.tile_dir() + '/'
      + GraphTile::FileSuffix(hdr.graphid(), hierarchy);

  // Make sure the directory exists on the system
  if (!boost::filesystem::exists(filename.parent_path()))
    boost::filesystem::create_directories(filename.parent_path());

  // Open file. Truncate so we replace the contents.
  std::ofstream file(filename.c_str(),
                     std::ios::out | std::ios::binary | std::ios::trunc);
  if (file.is_open()) {

    // Write the updated header.
    file.write(reinterpret_cast<const char*>(&hdr),
               sizeof(GraphTileHeaderBuilder));

    // Write the updated nodes
    file.write(reinterpret_cast<const char*>(&nodes[0]),
               nodes.size() * sizeof(NodeInfoBuilder));

    // Write the updated directed edges
    file.write(reinterpret_cast<const char*>(&directededges[0]),
               directededges.size() * sizeof(DirectedEdgeBuilder));

    // Write the existing transit departures
    file.write(reinterpret_cast<const char*>(&departures_[0]),
               hdr.departurecount() * sizeof(TransitDeparture));

    // Write the existing transit trips
    file.write(reinterpret_cast<const char*>(&transit_trips_[0]),
               hdr.tripcount() * sizeof(TransitTrip));

    // Write the existing transit stops
    file.write(reinterpret_cast<const char*>(&transit_stops_[0]),
               hdr.stopcount() * sizeof(TransitStop));

    // Write the existing transit routes
    file.write(reinterpret_cast<const char*>(&transit_routes_[0]),
               hdr.routecount() * sizeof(TransitRoute));

    // Write the existing transit transfers
    file.write(reinterpret_cast<const char*>(&transit_transfers_[0]),
               hdr.transfercount() * sizeof(TransitTransfer));

    // Write the existing transit calendar exceptions
    file.write(reinterpret_cast<const char*>(&transit_exceptions_[0]),
               hdr.calendarcount() * sizeof(TransitCalendar));

    // Write the existing signs
    file.write(reinterpret_cast<const char*>(&signs_[0]),
               hdr.signcount() * sizeof(Sign));

    // Write the existing admins
    file.write(reinterpret_cast<const char*>(&admins_[0]),
               hdr.admincount() * sizeof(Admin));

    // Write the existing edgeinfo
    file.write(edgeinfo_, edgeinfo_size_);

    // Save existing text
    file.write(textlist_, textlist_size_);

    size_ = file.tellp();
    file.close();

  } else {
    throw std::runtime_error("Failed to open file " + filename.string());
  }
}

// Update a graph tile with new header, nodes, directed edges, and signs.
void GraphTileBuilder::Update(const baldr::TileHierarchy& hierarchy,
                const GraphTileHeaderBuilder& hdr,
                const std::vector<NodeInfoBuilder>& nodes,
                const std::vector<DirectedEdgeBuilder>& directededges,
                const std::vector<SignBuilder>& signs) {
  // Get the name of the file
  boost::filesystem::path filename = hierarchy.tile_dir() + '/' +
            GraphTile::FileSuffix(hdr.graphid(), hierarchy);

  // Make sure the directory exists on the system
  if (!boost::filesystem::exists(filename.parent_path()))
    boost::filesystem::create_directories(filename.parent_path());

  // Open file. Truncate so we replace the contents.
  std::ofstream file(filename.c_str(),
                     std::ios::out | std::ios::binary | std::ios::trunc);
  if (file.is_open()) {
    // Write the updated header.
    file.write(reinterpret_cast<const char*>(&hdr),
               sizeof(GraphTileHeaderBuilder));

    // Write the updated nodes
    file.write(reinterpret_cast<const char*>(&nodes[0]),
               nodes.size() * sizeof(NodeInfoBuilder));

    // Write the updated directed edges
    file.write(reinterpret_cast<const char*>(&directededges[0]),
               directededges.size() * sizeof(DirectedEdgeBuilder));

    // Write the existing transit departures
    file.write(reinterpret_cast<const char*>(&departures_[0]),
               hdr.departurecount() * sizeof(TransitDeparture));

    // Write the existing transit trips
    file.write(reinterpret_cast<const char*>(&transit_trips_[0]),
               hdr.tripcount() * sizeof(TransitTrip));

    // Write the existing transit stops
    file.write(reinterpret_cast<const char*>(&transit_stops_[0]),
               hdr.stopcount() * sizeof(TransitStop));

    // Write the existing transit routes
    file.write(reinterpret_cast<const char*>(&transit_routes_[0]),
               hdr.routecount() * sizeof(TransitRoute));

    // Write the existing transit transfers
    file.write(reinterpret_cast<const char*>(&transit_transfers_[0]),
               hdr.transfercount() * sizeof(TransitTransfer));

    // Write the existing transit calendar exceptions
    file.write(reinterpret_cast<const char*>(&transit_exceptions_[0]),
               hdr.calendarcount() * sizeof(TransitCalendar));

    // Write the updated signs
    file.write(reinterpret_cast<const char*>(&signs[0]),
               signs.size() * sizeof(SignBuilder));

    // Write the existing admins
    file.write(reinterpret_cast<const char*>(&admins_[0]),
               hdr.admincount() * sizeof(Admin));

    // Write the existing edgeinfo and textlist
    file.write(edgeinfo_, edgeinfo_size_);
    file.write(textlist_, textlist_size_);

    size_ = file.tellp();
    file.close();
  } else {
    throw std::runtime_error("Failed to open file " + filename.string());
  }
}

// Add a node and list of directed edges
void GraphTileBuilder::AddNodeAndDirectedEdges(
    NodeInfoBuilder& node,
    const std::vector<DirectedEdgeBuilder>& directededges) {
  // Set the index to the first directed edge from this node and
  // set its count. Add the node to the list
  node.set_edge_index(directededges_builder_.size());
  node.set_edge_count(directededges.size());
  nodes_builder_.push_back(node);

  // Add directed edges to the list
  for (const auto& directededge : directededges) {
    directededges_builder_.push_back(directededge);
  }
}

// Get the current list of node builders.
const std::vector<NodeInfoBuilder>& GraphTileBuilder::nodes() const {
  return nodes_builder_;
}

// Gets the current list of directed edge (builders).
const std::vector<DirectedEdgeBuilder>& GraphTileBuilder::directededges() const {
  return directededges_builder_;
}

// Clear the current list of nodes (builders).
void GraphTileBuilder::ClearNodes() {
  nodes_builder_.clear();
}

// Clear the current list of directed edges (builders).
void GraphTileBuilder::ClearDirectedEdges() {
  directededges_builder_.clear();
}

// Add a transit departure.
void GraphTileBuilder::AddTransitDeparture(const TransitDeparture& departure) {
  departure_builder_.emplace_back(std::move(departure));
}

// Add a transit trip.
void GraphTileBuilder::AddTransitTrip(const TransitTrip& trip) {
  trip_builder_.emplace_back(std::move(trip));
}

// Add a transit stop.
void GraphTileBuilder::AddTransitStop(const TransitStop& stop)  {
  stop_builder_.emplace_back(std::move(stop));
}

// Add a transit route.
void GraphTileBuilder::AddTransitRoute(const TransitRoute& route)  {
  route_builder_.emplace_back(std::move(route));
}

// Add a transit transfer.
void GraphTileBuilder::AddTransitTransfer(const TransitTransfer& transfer)  {
  transfer_builder_.emplace_back(std::move(transfer));
}

// Add a transit calendar exception.
void GraphTileBuilder::AddTransitCalendar(const TransitCalendar& exception)  {
  exception_builder_.emplace_back(std::move(exception));
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

// Add edge info
uint32_t GraphTileBuilder::AddEdgeInfo(const uint32_t edgeindex,
                                       const GraphId& nodea,
                                       const baldr::GraphId& nodeb,
                                       const uint64_t wayid,
                                       const std::vector<PointLL>& lls,
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
    std::vector<uint32_t> text_name_offset_list;
    text_name_offset_list.reserve(names.size());
    for (const auto& name : names) {
      if (!(name.empty())) {
        // Add name and add its offset to edge info's list.
        uint32_t offset = AddName(name);
        text_name_offset_list.emplace_back(offset);
      }
    }
    edgeinfo.set_text_name_offset_list(text_name_offset_list);

    // Add to the map
    edge_offset_map_.emplace(edge_tuple_item, edge_info_offset_);

    // Set current edge offset
    uint32_t current_edge_offset = edge_info_offset_;

    // Update edge offset for next item
    edge_info_offset_ += edgeinfo.SizeOf();

    // Return the offset to this edge info
    added = true;
    return current_edge_offset;
  } else {
    // Already have this edge - return the offset
    added = false;
    return existing_edge_offset_item->second;
  }
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
            const std::string& state_iso,const std::string& start_dst,
            const std::string& end_dst) {
  // Check if admin already exists
  auto existing_admin_info_offset_item = admin_info_offset_map_.find(country_iso+state_name);
  if (existing_admin_info_offset_item == admin_info_offset_map_.end()) {
    // Add names and add to the admin builder
    uint32_t country_offset = AddName(country_name);
    uint32_t state_offset   = AddName(state_name);
    admins_builder_.emplace_back(country_offset, state_offset,
                                 country_iso, state_iso,
                                 start_dst, end_dst);

    // Add to the map
    admin_info_offset_map_.emplace(country_iso+state_name, admins_builder_.size()-1);
    return admins_builder_.size()-1;
  } else {
    // Already have this admin - return the offset
    return existing_admin_info_offset_item->second;
  }
}

// Serialize the edge info list
void GraphTileBuilder::SerializeEdgeInfosToOstream(std::ostream& out) {
  for (const auto& edgeinfo : edgeinfo_list_) {
    out << edgeinfo;
  }
}

// Serialize the text list
void GraphTileBuilder::SerializeTextListToOstream(std::ostream& out) {
  for (const auto& text : textlistbuilder_) {
    out << text << '\0';
  }
}

// Gets a non-const node (builder) from existing tile data.
NodeInfoBuilder& GraphTileBuilder::node(const size_t idx) {
  if (idx < header_->nodecount())
    return static_cast<NodeInfoBuilder&>(nodes_[idx]);
  throw std::runtime_error("GraphTileBuilder NodeInfo index out of bounds");
}

// Get the node builder at the specified index.
NodeInfoBuilder& GraphTileBuilder::node_builder(const size_t idx) {
  if (idx < header_->nodecount())
    return nodes_builder_[idx];
  throw std::runtime_error("GraphTileBuilder NodeInfo index out of bounds");
}

// Gets a non-const directed edge (builder) from existing tile data.
DirectedEdgeBuilder& GraphTileBuilder::directededge(const size_t idx) {
  if (idx < header_->directededgecount())
    return static_cast<DirectedEdgeBuilder&>(directededges_[idx]);
  throw std::runtime_error("GraphTile DirectedEdge id out of bounds");
}

// Get the directed edge builder at the specified index.
DirectedEdgeBuilder& GraphTileBuilder::directededge_builder(const size_t idx) {
  if (idx < header_->directededgecount())
    return directededges_builder_[idx];
  throw std::runtime_error("GraphTile DirectedEdge id out of bounds");
}

// Gets a non-const sign (builder) from existing tile data.
SignBuilder& GraphTileBuilder::sign(const size_t idx) {
  if (idx < header_->signcount())
    return static_cast<SignBuilder&>(signs_[idx]);
  throw std::runtime_error("GraphTileBuilder sign index is out of bounds");
}

// Gets a sign builder at the specified index.
SignBuilder& GraphTileBuilder::sign_builder(const size_t idx) {
  if (idx < header_->signcount())
    return signs_builder_[idx];
  throw std::runtime_error("GraphTileBuilder sign index is out of bounds");
}

// Gets a const admin builder at specified index.
const AdminInfoBuilder& GraphTileBuilder::admins_builder(size_t idx) {
  if (idx < admins_builder_.size())
    return admins_builder_.at(idx);
  throw std::runtime_error("GraphTileBuilder admin index is out of bounds");
}

}
}

