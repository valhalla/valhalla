#include "baldr/graphtile.h"

namespace valhalla{
namespace baldr{

GraphTile::GraphTile() {
  nodes_ = nullptr;
  directededges_ = nullptr;
  edgeinfo_ = nullptr;
  namelist_ = nullptr;
}

const GraphTileHeader& GraphTile::header() const {
  return header_;
}

const NodeInfo* GraphTile::node(const GraphId& node) const {
  // TODO - do we want to validate the tile and level - or just assume
  // the correct tile is called? Validate we are not overflowing the list?
  return nodes_ + node.id();
}

const DirectedEdge* GraphTile::directededge(const GraphId& edge) const {
  return directededges_ + edge.id();
}

EdgeInfo* GraphTile::edgeinfo() const {
  // TODO - how do we call this - with an offset (from a directed edge?)
  return nullptr;
}

}
}
