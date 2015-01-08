#include "mjolnir/graphtilebuilder.h"

#include <boost/filesystem/operations.hpp>
#include <stdexcept>
#include <list>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

GraphTileBuilder::GraphTileBuilder()
    : GraphTile() {
}

// Constructor given an existing tile. This is used to read in the tile
// data and then add to it (e.g. adding node connections between hierarchy
// levels.
GraphTileBuilder::GraphTileBuilder(const std::string& basedir,
                                   const GraphId& graphid)
    : GraphTile(basedir, graphid) {
}

// Output the tile to file. Stores as binary data.

void GraphTileBuilder::StoreTileData(const std::string& basedirectory,
                                     const GraphId& graphid) {
  // Get the name of the file
  boost::filesystem::path filename = Filename(basedirectory, graphid);
  // Make sure the directory exists on the system
  if (!boost::filesystem::exists(filename.parent_path()))
    boost::filesystem::create_directories(filename.parent_path());

  // Open to the end of the file so we can immediately get size;
  std::ofstream file(filename.c_str(),
                     std::ios::out | std::ios::binary | std::ios::ate);
  if (file.is_open()) {
    // Configure the header
    header_builder_.set_nodecount(nodes_builder_.size());
    header_builder_.set_directededgecount(directededges_builder_.size());
    header_builder_.set_edgeinfo_offset(
        (sizeof(GraphTileHeaderBuilder))
            + (nodes_builder_.size() * sizeof(NodeInfoBuilder))
            + (directededges_builder_.size() * sizeof(DirectedEdgeBuilder)));
    header_builder_.set_textlist_offset(
        header_builder_.edgeinfo_offset() + edgeinfo_size_);

    // Write the header.
    file.write(reinterpret_cast<const char*>(&header_builder_),
               sizeof(GraphTileHeaderBuilder));

    // Write the nodes
    file.write(reinterpret_cast<const char*>(&nodes_builder_[0]),
               nodes_builder_.size() * sizeof(NodeInfoBuilder));

    // Write the directed edges
    file.write(reinterpret_cast<const char*>(&directededges_builder_[0]),
               directededges_builder_.size() * sizeof(DirectedEdgeBuilder));

    // Write the edge data
    SerializeEdgeInfosToOstream(file);

    // Write the names
    SerializeTextListToOstream(file);

    std::cout << "Write: " << filename << " nodes = " << nodes_builder_.size()
              << " directededges = " << directededges_builder_.size()
              << " edgeinfo size = " << edgeinfo_size_ << " textlist size = "
              << textlist_size_ << std::endl;

    size_ = file.tellp();
    file.close();
  } else {
    throw std::runtime_error("Failed to open file " + filename.string());
  }
}

// Update a graph tile with new header, nodes, and directed edges. This
// is used to add directed edges connecting two hierarchy levels
void GraphTileBuilder::Update(const std::string& basedirectory,
                const GraphTileHeaderBuilder& hdr,
                const std::vector<NodeInfoBuilder>& nodes,
                const std::vector<DirectedEdgeBuilder> directededges) {
  // Get the name of the file
  boost::filesystem::path filename = Filename(basedirectory, id_);

  // Make sure the directory exists on the system
  if (!boost::filesystem::exists(filename.parent_path()))
    boost::filesystem::create_directories(filename.parent_path());

  // Open to the end of the file so we can immediately get size;
  std::ofstream file(filename.c_str(),
                     std::ios::out | std::ios::binary | std::ios::ate);
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

    // Write the existing edgeinfo and textlist
    file.write(edgeinfo_, edgeinfo_size_);
    file.write(textlist_, textlist_size_);

    size_ = file.tellp();
    file.close();
  } else {
    throw std::runtime_error("Failed to open file " + filename.string());
  }
}

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

void GraphTileBuilder::SetEdgeInfoAndSize(
    const std::list<EdgeInfoBuilder>& edges,
    const std::size_t edgeinfo_size) {

  edgeinfos_builder_ = edges;

  // Set edgeinfo data size
  edgeinfo_size_ = edgeinfo_size;
}

void GraphTileBuilder::SetTextListAndSize(
    const std::list<std::string>& textlist,
    const std::size_t textlist_size) {

  textlist_builder_ = textlist;

  // Set textlist data size
  textlist_size_ = textlist_size;
}

void GraphTileBuilder::SerializeEdgeInfosToOstream(std::ostream& out) {
  for (const auto& edgeinfo : edgeinfos_builder_) {
    out << edgeinfo;
  }
}

void GraphTileBuilder::SerializeTextListToOstream(std::ostream& out) {
  for (const auto& text : textlist_builder_) {
    out << text << '\0';
  }
}

// Gets a non-const node (builder) from existing tile data.
NodeInfoBuilder& GraphTileBuilder::node(const size_t idx) {
 if (idx < header_->nodecount())
   return static_cast<NodeInfoBuilder&>(nodes_[idx]);
 throw std::runtime_error("GraphTileBuilder NodeInfo index out of bounds");
}

// Gets a non-const node (builder) from existing tile data.
DirectedEdgeBuilder& GraphTileBuilder::directededge(const size_t idx) {
  if (idx < header_->directededgecount())
    return static_cast<DirectedEdgeBuilder&>(directededges_[idx]);
  throw std::runtime_error("GraphTile DirectedEdge id out of bounds");
}

}
}

