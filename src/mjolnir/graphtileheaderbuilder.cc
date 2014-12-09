#include "mjolnir/graphtileheaderbuilder.h"

namespace valhalla {
namespace mjolnir {

// Constructor
GraphTileHeaderBuilder::GraphTileHeaderBuilder()
    : GraphTileHeader() {
}

// Sets the number of nodes in this tile.
void GraphTileHeaderBuilder::set_nodecount(const unsigned int nodecount) {
  nodecount_ = nodecount;
}

// Sets the number of directed edges in this tile.
void GraphTileHeaderBuilder::set_directededgecount(
    const unsigned int directededgecount) {
  directededgecount_ = directededgecount;
}

// Sets the offset to the edge info.
void GraphTileHeaderBuilder::set_edgeinfo_offset(
    const unsigned int edgeinfo_offset) {
  edgeinfo_offset_ = edgeinfo_offset;
}

// Sets the offset to the name list.
void GraphTileHeaderBuilder::set_namelist_offset(
    const unsigned int namelist_offset) {
  namelist_offset_ = namelist_offset;
}

}
}
