#include "mjolnir/graphtilebuilder.h"

#include <boost/filesystem/operations.hpp>
#include <stdexcept>
#include <list>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

GraphTileBuilder::GraphTileBuilder()
    : GraphTile(),
      edgeinfo_size_(0),
      textlist_size_(0) {

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

    // TODO - rm later
    /*std::cout << ">>>>> header_builder_.nodecount_"
              << header_builder_.nodecount()
              << "  header_builder_.directededgecount_ = "
              << header_builder_.directededgecount()
              << "  header_builder_.edgeinfo_offset_ = "
              << header_builder_.edgeinfo_offset()
              << "  header_builder_.textlist_offset_ = "
              << header_builder_.textlist_offset() << std::endl;*/

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
    const std::list<std::string>& textlist, const uint32_t textlist_size) {

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

}
}

