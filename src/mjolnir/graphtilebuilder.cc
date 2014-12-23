#include "mjolnir/graphtilebuilder.h"

#include <boost/filesystem/operations.hpp>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

GraphTileBuilder::GraphTileBuilder() {

}

// Output the tile to file. Stores as binary data.

bool GraphTileBuilder::StoreTileData(const std::string& basedirectory,
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
    // Write the header. TODO - add edge info offset and name list offset
    header_builder_.set_nodecount(nodes_builder_.size());
    header_builder_.set_directededgecount(directededges_builder_.size());
    file.write(reinterpret_cast<const char*>(&header_builder_),
               sizeof header_builder_);

    // Write the nodes
    file.write(reinterpret_cast<const char*>(&nodes_builder_[0]),
               nodes_builder_.size() * sizeof(NodeInfoBuilder));

    // Write the directed edges
    file.write(reinterpret_cast<const char*>(&directededges_builder_[0]),
               directededges_builder_.size() * sizeof(DirectedEdgeBuilder));

    // Write the edge data
    file.write(reinterpret_cast<const char*>(&edgeinfo_builder_[0]),
               edgeinfo_size_);

    // Write the names
    file.write(reinterpret_cast<const char*>(&textlist_builder_[0]),
               textlist_size_);

    std::cout << "Write: " << filename << " nodes = " << nodes_builder_.size()
              << " directededges = " << directededges_builder_.size()
              << " edges = " << edgeinfo_builder_.size() << " textlist = "
              << textlist_builder_.size() << std::endl;

    file.close();
    return true;
  } else {
    std::cout << "Failed to open file " << filename << std::endl;
  }
  return false;
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
    const std::vector<EdgeInfoBuilder>& edges,
    const std::size_t edgeinfo_size) {

  // TODO - change to move?
  if (!edges.empty()) {
    edgeinfo_builder_.insert(edgeinfo_builder_.end(), edges.begin(),
                             edges.end());
  }

  // Set edgeinfo data size
  edgeinfo_size_ = edgeinfo_size;
}

void GraphTileBuilder::SetTextListAndSize(const std::vector<std::string>& textlist,
                        const std::size_t textlist_size) {
  // TODO move
  if (!textlist.empty()) {
    textlist_builder_.insert(textlist_builder_.end(), textlist.begin(),
                             textlist.end());
  // Set textlist data size
    textlist_size_ = textlist_size;
  }
}

}
}

