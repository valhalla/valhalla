#include "hierarchybuilder.h"

#include "config.h"

#include <ostream>
#include <set>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

HierarchyBuilder::HierarchyBuilder(const boost::property_tree::ptree& pt)
    : tile_hierarchy_(pt),
      graphreader_(tile_hierarchy_) {

  //make sure there are 2 levels!
  if (tile_hierarchy_.levels().size() < 2)
    throw std::runtime_error("Bad tile hierarchy - need 2 levels");
}

bool HierarchyBuilder::Build() {
  // For each level
  for(auto base_level = tile_hierarchy_.levels().rbegin(), new_level = tile_hierarchy_.levels().rbegin()++;
      new_level != tile_hierarchy_.levels().rend(); ++base_level, ++new_level) {

    // Size the vector for new tiles
    tilednodes_.resize(new_level->second.tiles.TileCount());

    //TODO: actually build the next level
  }

  return true;
}

// Get the nodes that remain in the new level
bool HierarchyBuilder::GetNodesInNewLevel(const TileHierarchy::TileLevel& base_level, const TileHierarchy::TileLevel& new_level) {
  // Iterate through all tiles in the lower level
  // TODO - can be concurrent if we divide by rows for example
  uint32_t ntiles = base_level.tiles.TileCount();
  uint32_t baselevel = (uint32_t)base_level.level;
  GraphTile* graphtile = nullptr;
  for (uint32_t tileid = 0; tileid < ntiles; tileid++) {
    graphtile = graphreader_.GetGraphTile(GraphId(tileid, baselevel, 0));
    if (graphtile == nullptr) {
      // No tile - common case, just continue
      continue;
    }

    // Iterate through the nodes
    uint32_t nodecount = graphtile->header()->nodecount();
    GraphId priornode(tileid, baselevel, 0);
    const NodeInfo* nodeinfo = graphtile->node(priornode);
    for (uint32_t i = 0; i < nodecount; i++, nodeinfo++, priornode++) {
      // Skip any nodes with best road class > road class cutoff. They
      // will not be included in this hierarchy
      if (nodeinfo->bestrc() > static_cast<uint32_t>(new_level.importance))
        continue;

      // This node remains on the new hierarchy - add it to the new tile
      // and add the mapping from the old tile to this new node
      AddNewNode(graphtile, nodeinfo, priornode, new_level);

      // Test if the new node can be considered for contraction
      // TODO - do this here to simplify adding shortcuts later???
    }
  }
  return true;
}

void HierarchyBuilder::AddNewNode(GraphTile* graphtile,
                                  const NodeInfo* nodeinfo,
                                  const GraphId& priornode,
                                  const TileHierarchy::TileLevel& new_level) {
  // TODO - could make it so there is an easy mapping from base tiles
  // to new tiles (could be no overlap of bounds)
  uint32_t tileid = new_level.tiles.TileId(nodeinfo->latlng());
  uint32_t nodeid = tilednodes_[tileid].size();
  bool contract = CanContract(graphtile, priornode, nodeinfo);
  tilednodes_[tileid].push_back(NewNode(priornode, contract));

  // Create mapping from base level node to new node
  GraphId newnode(tileid, new_level.level, nodeid);
  nodemap_[priornode.value()] = newnode;
}

bool HierarchyBuilder::CanContract(GraphTile* graphtile,
                                   const GraphId& node,
                                   const NodeInfo* nodeinfo) const {
  // Get the number of directed edges  on this hierarchy
  std::vector<DirectedEdge*> edges;
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  const DirectedEdge* directededge = graphtile->directededge(nodeinfo->edge_index());
  for (unsigned int i = 0, n = nodeinfo->edge_count(); i < n;
              i++, directededge++, edgeid++) {
    // TODO - need access methods for DirectedEdge attributes!
//    if (directededge->roadclass() <= rcc_) {
//      edges.push_back(directededge);
//    }
  }

  // If n is not equal to 2 we cannot contract
  if (edges.size() != 2)
    return false;

  // If n == 2 we need to check attributes and names to see if we can create a
  // shortcut through this node
  return false;
}

}
}

