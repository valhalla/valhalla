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
  boost::filesystem::path filename = hierarchy.tile_dir() + '/' +
          GraphTile::FileSuffix(graphid, hierarchy);

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
    header_builder_.set_signcount(signs_builder_.size());
    header_builder_.set_edgeinfo_offset(
        (sizeof(GraphTileHeaderBuilder))
            + (nodes_builder_.size() * sizeof(NodeInfoBuilder))
            + (directededges_builder_.size() * sizeof(DirectedEdgeBuilder))
            + (signs_builder_.size() * sizeof(SignBuilder)));
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

    // Write the signs
    file.write(reinterpret_cast<const char*>(&signs_builder_[0]),
               signs_builder_.size() * sizeof(SignBuilder));

    // Write the edge data
    SerializeEdgeInfosToOstream(file);

    // Write the names
    SerializeTextListToOstream(file);

    LOG_DEBUG((boost::format("Write: %1% nodes = %2% directededges = %3% signs %4% edgeinfo offset = %5% textlist offset = %6%")
      % filename % nodes_builder_.size() % directededges_builder_.size() % signs_builder_.size() % edge_info_offset_ % text_list_offset_).str());

    size_ = file.tellp();
    file.close();
  } else {
    throw std::runtime_error("Failed to open file " + filename.string());
  }
}

// Update a graph tile with new header, nodes, and directed edges.
void GraphTileBuilder::Update(const baldr::TileHierarchy& hierarchy,
                const GraphTileHeaderBuilder& hdr,
                const std::vector<NodeInfoBuilder>& nodes,
                const std::vector<DirectedEdgeBuilder>& directededges) {
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

    // Write the existing signs
    file.write(reinterpret_cast<const char*>(&signs_[0]),
               hdr.signcount() * sizeof(Sign));

    // Write the existing edgeinfo, and textlist
    file.write(edgeinfo_, edgeinfo_size_);
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

    // Write the updated signs
    file.write(reinterpret_cast<const char*>(&signs[0]),
               signs.size() * sizeof(SignBuilder));

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
    }
    else {
      // Name already exists. Add sign type and existing text offset to list
      signs_builder_.emplace_back(idx, sign.type(),
                  existing_text_offset->second);
    }
  }
}

uint32_t GraphTileBuilder::AddEdgeInfo(const uint32_t edgeindex,
             const GraphId& nodea, const baldr::GraphId& nodeb,
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
    edgeinfo.set_shape(lls);

    ///////////////////////////////////////////////////////////////////////////
    // Put each name's index into the chunk of bytes containing all the names
    // in the tile
    std::vector<uint32_t> street_name_offset_list;
    street_name_offset_list.reserve(names.size());
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
        street_name_offset_list.emplace_back(text_list_offset_);

        // Add name/offset pair to map
        text_offset_map.emplace(name, text_list_offset_);

        // Update text offset value to length of string plus null terminator
        text_list_offset_ += (name.length() + 1);
      } // Something was already using this name
      else {
        // Add existing offset to list
        street_name_offset_list.emplace_back(existing_text_offset->second);
      }
    }
    edgeinfo.set_street_name_offset_list(street_name_offset_list);

    // Add to the map
    edge_offset_map.emplace(edge_tuple_item, edge_info_offset_);

    // Set current edge offset
    uint32_t current_edge_offset = edge_info_offset_;

    // Update edge offset for next item
    edge_info_offset_ += edgeinfo.SizeOf();

    // Return the offset to this edge info
    added = true;
    return current_edge_offset;
  }
  else {
    // Already have this edge - return the offset
    added = false;
    return existing_edge_offset_item->second;
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

}
}

