#ifndef VALHALLA_MJOLNIR_HIERARCHYBUILDER_H
#define VALHALLA_MJOLNIR_HIERARCHYBUILDER_H

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <boost/property_tree/ptree.hpp>

#include "valhalla/midgard/pointll.h"
#include "valhalla/baldr/tilehierarchy.h"
#include "valhalla/baldr/graphid.h"
#include "valhalla/baldr/graphreader.h"

namespace valhalla {
namespace mjolnir {

// Simple structure representing nodes in the new level
struct NewNode {
  baldr::GraphId basenode;
  bool contract;

  NewNode(const baldr::GraphId& bn, const bool c)
      : basenode(bn),
        contract(c) {
  }
}
;
/**
 * Class used to construct temporary data used to build the initial graph.
 */
class HierarchyBuilder {
 public:
  /**
   * Constructor
   */
  HierarchyBuilder(const boost::property_tree::ptree& pt);

  /**
   * Build the set of hierarchies based on the TileHierarchy configuration
   * and the current local hierarchy.
   */
  bool Build();

 protected:

  // Tile hierarchy/level information
  baldr::TileHierarchy tile_hierarchy_;

  // Graphreader
  baldr::GraphReader graphreader_;

  // Lists of nodes on the new hierarchy
  std::vector<std::vector<NewNode>> tilednodes_;

  // Mapping from base level node to new node
  std::map<uint64_t, baldr::GraphId> nodemap_;

  // Get the nodes that remain in the new level
  bool GetNodesInNewLevel(const baldr::TileHierarchy::TileLevel& base_level, const baldr::TileHierarchy::TileLevel& new_level);

  // Add a node to the new level. Map it back to the node on the base
  // level and create a mapping from the base level to the new node.
  void AddNewNode(baldr::GraphTile* graphtile, const baldr::NodeInfo* nodeinfo,
                  const baldr::GraphId& priornode, const baldr::TileHierarchy::TileLevel& new_level);

  // Check if the new node can be contracted. Use information from
  // the base level.
  bool CanContract(baldr::GraphTile* graphtile,
                   const baldr::GraphId& node,
                   const baldr::NodeInfo* nodeinfo) const;
};

}
}

#endif  // VALHALLA_MJOLNIR_HIERARCHYBUILDER_H
