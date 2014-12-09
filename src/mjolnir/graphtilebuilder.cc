
#include "mjolnir/graphtilebuilder.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

GraphTileBuilder::GraphTileBuilder() {

}

// Output the tile to file. Stores as binary data.

bool GraphTileBuilder::StoreTileData(const std::string& filename) {
  // Open to the end of the file so we can immediately get size;
   std::ofstream file(filename, std::ios::in|std::ios::binary|std::ios::ate);
   if (file.is_open()) {

     std::ofstream file(filename,
         std::ios::out | std::ios::binary | std::ios::app);

     // Write the header
     file.write(reinterpret_cast<const char*>(&header_), sizeof header_);

     // Write the nodes
     file.write(reinterpret_cast<const char*>(&nodes_[0]),
                nodes_.size() * sizeof(NodeInfoBuilder));

     // Write the directed edges
     file.write(reinterpret_cast<const char*>(&directededges_[0]),
                directededges_.size() * sizeof(DirectedEdgeBuilder));

     // Write the edge data

     // Write the names

     return true;
  }
  return false;
}

bool GraphTileBuilder::AddNodeAndEdges(const NodeInfoBuilder& node,
           const std::vector<DirectedEdgeBuilder>& directededges) {
 //          const std::vector<EdgeInfoBuilder>& edges,
//           const std::vector<std::string>& names) {
   // Set the index to the first directed edge in the tile
//     node.set*();

   // Add the node to the list
   nodes_.push_back(node);

   // For each directed edge need to set its common edge offset
   for (auto directededge : directededges) {

     // Check if the common edge is already stored for this tile
     // If so get its offset

     // Add the directed edge to the list
     directededges_.push_back(directededge);
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

