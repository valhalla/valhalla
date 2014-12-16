
#include "mjolnir/graphtilebuilder.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

GraphTileBuilder::GraphTileBuilder() {

}

// Output the tile to file. Stores as binary data.

bool GraphTileBuilder::StoreTileData(const std::string& basedirectory,
             const GraphId& graphid) {
  // Open to the end of the file so we can immediately get size;
   std::ofstream file(Filename(basedirectory, graphid),
           std::ios::out|std::ios::binary|std::ios::ate);
   if (file.is_open()) {
     // Write the header
     file.write(reinterpret_cast<const char*>(&header_), sizeof header_);

     // Write the nodes
     file.write(reinterpret_cast<const char*>(&nodes_[0]),
                nodes_builder_.size() * sizeof(NodeInfoBuilder));

     // Write the directed edges
     file.write(reinterpret_cast<const char*>(&directededges_[0]),
                directededges_builder_.size() * sizeof(DirectedEdgeBuilder));

     // Write the edge data

     // Write the names

     std::cout << "Write: " << Filename(basedirectory, graphid) << " nodes = " << nodes_builder_.size() <<
         " directededges = " << directededges_builder_.size() << std::endl;

     file.close();
     return true;
  }
  else {
    std::cout << "Failed to open file " << Filename(basedirectory, graphid) << std::endl;
  }
  return false;
}

bool GraphTileBuilder::AddNodeAndEdges(const NodeInfoBuilder& node,
           const std::vector<DirectedEdgeBuilder>& directededges) {
 //          const std::vector<EdgeInfoBuilder>& edges,
//           const std::vector<std::string>& names) {
   // Add the node to the list
   nodes_builder_.push_back(node);

   // For each directed edge need to set its common edge offset
   for (auto directededge : directededges) {

     // Check if the common edge is already stored for this tile
     // If so get its offset

     // Add the directed edge to the list
     directededges_builder_.push_back(directededge);
  }
  return true;
}
/*
unsigned int GraphTileBuilder::StoreEdgeInfo(const EdgeInfoBuilder& edge) {
  // TODO
  return 0;

}
**/

}
}

