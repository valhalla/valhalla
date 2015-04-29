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
}

// Constructor given an existing tile. This is used to read in the tile
// data and then add to it (e.g. adding node connections between hierarchy
// levels.
GraphTileBuilder::GraphTileBuilder(const baldr::TileHierarchy& hierarchy,
                                   const GraphId& graphid)
    : GraphTile(hierarchy, graphid) {
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

  // Open to the end of the file so we can immediately get size;
  std::ofstream file(filename.c_str(),
                     std::ios::out | std::ios::binary | std::ios::ate);
  if (file.is_open()) {
    // Configure the header
    header_builder_.set_graphid(graphid);
    header_builder_.set_nodecount(nodes_builder_.size());
    header_builder_.set_directededgecount(directededges_builder_.size());
    header_builder_.set_departurecount(departures_.size());
    header_builder_.set_tripcount(transit_trips_.size());
    header_builder_.set_stopcount(transit_stops_.size());
    header_builder_.set_routecount(transit_routes_.size());
    header_builder_.set_transfercount(transit_transfers_.size());
    header_builder_.set_calendarcount(transit_exceptions_.size());
    header_builder_.set_signcount(signs_builder_.size());
    header_builder_.set_admincount(admins_builder_.size());
    header_builder_.set_edgeinfo_offset(
        (sizeof(GraphTileHeaderBuilder))
            + (nodes_builder_.size() * sizeof(NodeInfoBuilder))
            + (directededges_builder_.size() * sizeof(DirectedEdgeBuilder))
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
    std::sort(departures_.begin(), departures_.end());
    file.write(reinterpret_cast<const char*>(&departures_[0]),
               departures_.size() * sizeof(TransitDeparture));

    // Sort and write the transit trips
    std::sort(transit_trips_.begin(), transit_trips_.end());
    file.write(reinterpret_cast<const char*>(&transit_trips_[0]),
               transit_trips_.size() * sizeof(TransitTrip));

    // Sort and write the transit stops
    std::sort(transit_stops_.begin(), transit_stops_.end());
    file.write(reinterpret_cast<const char*>(&transit_stops_[0]),
               transit_stops_.size() * sizeof(TransitStop));

    // Sort and write the transit routes
    std::sort(transit_routes_.begin(), transit_routes_.end());
    file.write(reinterpret_cast<const char*>(&transit_routes_[0]),
               transit_routes_.size() * sizeof(TransitRoute));

    // Sort and write the transit transfers
    std::sort(transit_transfers_.begin(), transit_transfers_.end());
    file.write(reinterpret_cast<const char*>(&transit_transfers_[0]),
               transit_transfers_.size() * sizeof(TransitTransfer));

    // Sort and write the transit calendar exceptions
    std::sort(transit_exceptions_.begin(), transit_exceptions_.end());
    file.write(reinterpret_cast<const char*>(&transit_exceptions_[0]),
               transit_exceptions_.size() * sizeof(TransitCalendar));

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

    LOG_DEBUG((boost::format("Write: %1% nodes = %2% directededges = %3% signs %4% edgeinfo offset = %5% textlist offset = %6% admininfo offset = %7%" )
            % filename % nodes_builder_.size() % directededges_builder_.size() % signs_builder_.size() % edge_info_offset_ % text_list_offset_ % admin_info_offset_).str());

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


// Update a graph tile with new header, nodes, and directed edges.
void GraphTileBuilder::Update(
    const baldr::TileHierarchy& hierarchy, GraphTileHeaderBuilder& hdr,
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

    hdr.set_admincount(admins_builder_.size());
    std::size_t addedsize = (admins_builder_.size() * sizeof(AdminInfoBuilder));

    hdr.set_edgeinfo_offset(hdr.edgeinfo_offset() + addedsize);
    hdr.set_textlist_offset(hdr.textlist_offset() + addedsize);

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

    // Write the admins
    file.write(reinterpret_cast<const char*>(&admins_builder_[0]),
               admins_builder_.size() * sizeof(AdminInfoBuilder));

    // Write the existing edgeinfo, and textlist
    file.write(edgeinfo_, edgeinfo_size_);

    // Save existing text
    file.write(textlist_, textlist_size_);

    // append the new text.
    SerializeTextListToOstream(file);

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
    const NodeInfoBuilder& node,
    const std::vector<DirectedEdgeBuilder>& directededges) {
  // Add the node to the list
  nodes_builder_.push_back(node);

  // For each directed edge need to set its common edge offset
  for (const auto& directededge : directededges) {
    // Add the directed edge to the list
    directededges_builder_.push_back(directededge);
  }
}

// Add a transit departure.
void GraphTileBuilder::AddTransitDeparture(const TransitDeparture& departure) {
  departures_.emplace_back(departure);
}

// Add a transit trip.
void GraphTileBuilder::AddTransitTrip(const TransitTrip& trip) {
  transit_trips_.emplace_back(trip);
}

// Add a transit stop.
void GraphTileBuilder::AddTransitStop(const TransitStop& stop)  {
  transit_stops_.emplace_back(stop);
}

// Add a transit route.
void GraphTileBuilder::AddTransitRoute(const TransitRoute& route)  {
  transit_routes_.emplace_back(route);
}

// Add a transit transfer.
void GraphTileBuilder::AddTransitTransfer(const TransitTransfer& transfer)  {
  transit_transfers_.emplace_back(transfer);
}

// Add a transit calendar exception.
void GraphTileBuilder::AddTransitCalendar(const TransitCalendar& exception)  {
  transit_exceptions_.emplace_back(exception);
}

// Add signs
void GraphTileBuilder::AddSigns(const uint32_t idx,
                                const std::vector<SignInfo>& signs) {
  // Iterate through the list of sign info (with sign text)
  for (const auto& sign : signs) {
    // Skip signs with no sign text
    if (sign.text().empty()) {
      continue;
    }

    // If nothing already used this sign text
    auto existing_text_offset = text_offset_map.find(sign.text());
    if (existing_text_offset == text_offset_map.end()) {
      // Add name to text list
      textlistbuilder_.emplace_back(sign.text());

      // Add sign to the list
      signs_builder_.emplace_back(idx, sign.type(), text_list_offset_);

      // Add text/offset pair to map
      text_offset_map.emplace(sign.text(), text_list_offset_);

      // Update text offset value to length of string plus null terminator
      text_list_offset_ += (sign.text().length() + 1);
    } else {
      // Name already exists. Add sign type and existing text offset to list
      signs_builder_.emplace_back(idx, sign.type(),
                                  existing_text_offset->second);
    }
  }
}

uint32_t GraphTileBuilder::AddEdgeInfo(const uint32_t edgeindex,
                                       const GraphId& nodea,
                                       const baldr::GraphId& nodeb,
                                       const uint64_t wayid,
                                       const std::vector<PointLL>& lls,
                                       const std::vector<std::string>& names,
                                       bool& added) {
  // If we haven't yet added edge info for this edge tuple
  auto edge_tuple_item = EdgeTuple(edgeindex, nodea, nodeb);
  auto existing_edge_offset_item = edge_offset_map.find(edge_tuple_item);
  if (existing_edge_offset_item == edge_offset_map.end()) {
    // Add a new EdgeInfo to the list and get a reference to it
    edgeinfo_list_.emplace_back();
    EdgeInfoBuilder& edgeinfo = edgeinfo_list_.back();
    edgeinfo.set_wayid(wayid);
    edgeinfo.set_shape(lls);

    ///////////////////////////////////////////////////////////////////////////
    // Put each name's index into the chunk of bytes containing all the names
    // in the tile
    std::vector<uint32_t> text_name_offset_list;
    text_name_offset_list.reserve(names.size());
    for (const auto& name : names) {
      // Skip blank names
      if (name.empty()) {
        continue;
      }

      // If nothing already used this name
      auto existing_text_offset = text_offset_map.find(name);
      if (existing_text_offset == text_offset_map.end()) {
        // Add name to text list
        textlistbuilder_.emplace_back(name);

        // Add name offset to list
        text_name_offset_list.emplace_back(text_list_offset_);

        // Add name/offset pair to map
        text_offset_map.emplace(name, text_list_offset_);

        // Update text offset value to length of string plus null terminator
        text_list_offset_ += (name.length() + 1);
      }  // Something was already using this name
      else {
        // Add existing offset to list
        text_name_offset_list.emplace_back(existing_text_offset->second);
      }
    }
    edgeinfo.set_text_name_offset_list(text_name_offset_list);

    // Add to the map
    edge_offset_map.emplace(edge_tuple_item, edge_info_offset_);

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

// Get admin index
uint32_t GraphTileBuilder::GetAdminIndex(const std::string& country_iso) {

  auto existing_admin_info_offset_item = admin_info_offset_map.find(country_iso);
  if (existing_admin_info_offset_item == admin_info_offset_map.end())
    return 0;
  return existing_admin_info_offset_item->second;
}

// Add admin
uint32_t GraphTileBuilder::AddAdmin(const std::string& country_name, const std::string& state_name,
                                    const std::string& country_iso, const std::string& state_iso,
                                    const std::string& start_dst, const std::string& end_dst) {

  auto existing_admin_info_offset_item = admin_info_offset_map.find(country_iso+state_name);
  if (existing_admin_info_offset_item == admin_info_offset_map.end()) {

    uint32_t country_offset = 0;
    uint32_t state_offset = 0;

    if (!country_name.empty()) {
      // If nothing already used this admin text
      auto existing_text_offset = text_offset_map.find(country_name);
      if (existing_text_offset == text_offset_map.end()) {
        // Add name to text list
        textlistbuilder_.emplace_back(country_name);

        // Append to the list. Get the size of the current list
        if (text_list_offset_ == 0)
          text_list_offset_ = textlist_size_;

        country_offset = text_list_offset_;

        // Add text/offset pair to map
        text_offset_map.emplace(country_name, text_list_offset_);

        // Update text offset value to length of string plus null terminator
        text_list_offset_ += (country_name.length() + 1);
      } else {
        // Name already exists. Add sign type and existing text offset to list
        country_offset = existing_text_offset->second;
      }
    }

    if (!state_name.empty()) {
      // If nothing already used this admin text
      auto existing_text_offset = text_offset_map.find(state_name);
      if (existing_text_offset == text_offset_map.end()) {
        // Add name to text list
        textlistbuilder_.emplace_back(state_name);

        // Append to the list. Get the size of the current list
        if (text_list_offset_ == 0)
          text_list_offset_ = textlist_size_;

        state_offset = text_list_offset_;

        // Add text/offset pair to map
        text_offset_map.emplace(state_name, text_list_offset_);

        // Update text offset value to length of string plus null terminator
        text_list_offset_ += (state_name.length() + 1);
      } else {
        // Name already exists. Add sign type and existing text offset to list
        state_offset = existing_text_offset->second;
      }
    }

    // Add admin to the list
    admins_builder_.emplace_back(country_offset, state_offset,
                                 country_iso, state_iso,
                                 start_dst, end_dst);

    // Add to the map
    admin_info_offset_map.emplace(country_iso+state_name, admins_builder_.size()-1);

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

// Gets a non-const directed edge (builder) from existing tile data.
DirectedEdgeBuilder& GraphTileBuilder::directededge(const size_t idx) {
  if (idx < header_->directededgecount())
    return static_cast<DirectedEdgeBuilder&>(directededges_[idx]);
  throw std::runtime_error("GraphTile DirectedEdge id out of bounds");
}

// Gets a non-const sign (builder) from existing tile data.
SignBuilder& GraphTileBuilder::sign(const size_t idx) {
  if (idx < header_->signcount())
    return static_cast<SignBuilder&>(signs_[idx]);
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

