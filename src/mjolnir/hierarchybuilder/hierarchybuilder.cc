#include "hierarchybuilder.h"

#include "config.h"

#include <ostream>
#include <set>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

HierarchyBuilder::HierarchyBuilder(const boost::property_tree::ptree& pt,
                                   const uint32_t level)
    : level_(level),
      baselevel_(level + 1),
      rcc_(0),
      tile_hierarchy_(pt),
      basetiles_(*tile_hierarchy_.levels().rbegin()),
      newtiles_(*tile_hierarchy_.levels().rbegin()),
      graphreader_(nullptr) {
}

bool HierarchyBuilder::Build() {
  // Get the tile directory
  std::string tiledir = tile_hierarchy_.tile_dir();

  // Initialize GraphReader
  graphreader_ = new GraphReader(tiledir);

  // Get tile hierarchy information
  GetHierarchyLevels();

  return true;
}

bool HierarchyBuilder::GetHierarchyLevels() {
  // TODO - make sure there are 2 levels!
  if (tile_hierarchy_.levels().size() < 2)
    throw std::runtime_error("Bad tile hierarchy - need 2 levels");

  // Get the base level and the new level to be created
  for (auto tl : tile_hierarchy_.levels()) {
     if (tl.level == level_ - 1) {
       basetiles_ = tl;
     } else if (tl.level == level_) {
       newtiles_ = tl;
     }
   }

   // TODO - add road class cutoff to tile hierarchy!
   rcc_ = 6;
  // Validity tests
  if (newtiles_.level != basetiles_.level - 1)
    return false;

  // Tile size must be an integer multiplier (TODO - true)?

  // Size the vector for new tiles
  tilednodes_.resize(newtiles_.tiles.TileCount());

  return true;
}

// Get the nodes that remain in the new level
bool HierarchyBuilder::GetNodesInNewLevel() {
  // Iterate through all tiles in the lower level
  // TODO - can be concurrent if we divide by rows for example
  uint32_t ntiles = basetiles_.tiles.TileCount();
  uint32_t baselevel = (uint32_t)basetiles_.level;
  GraphTile* graphtile = nullptr;
  for (uint32_t tileid = 0; tileid < ntiles; tileid++) {
    graphtile = graphreader_->GetGraphTile(GraphId(tileid, baselevel, 0));
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
      if (nodeinfo->bestrc() > rcc_)
        continue;

      // This node remains on the new hierarchy - add it to the new tile
      // and add the mapping from the old tile to this new node
      AddNewNode(graphtile, nodeinfo, priornode);

      // Test if the new node can be considered for contraction
      // TODO - do this here to simplify adding shortcuts later???
    }
  }
  return true;
}

void HierarchyBuilder::AddNewNode(GraphTile* graphtile,
                                  const NodeInfo* nodeinfo,
                                  const GraphId& priornode) {
  // TODO - could make it so there is an easy mapping from base tiles
  // to new tiles (could be no overlap of bounds)
  uint32_t tileid = newtiles_.tiles.TileId(nodeinfo->latlng());
  uint32_t nodeid = tilednodes_[tileid].size();
  bool contract = CanContract(graphtile, priornode, nodeinfo);
  tilednodes_[tileid].push_back(NewNode(priornode, contract));

  // Create mapping from base level node to new node
  GraphId newnode(tileid, level_, nodeid);
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

