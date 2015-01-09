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

// Simple structure to describe a connection between 2 levels
struct NodeConnection {
  baldr::GraphId basenode;
  baldr::GraphId newnode;

  NodeConnection(const baldr::GraphId& bn, const baldr::GraphId& nn)
      : basenode(bn),
        newnode(nn) {
  }

  // For sorting by Id
  bool operator < (const NodeConnection& other) const {
    return basenode.id() < other.basenode.id();
  }
};

// Simple structure representing nodes in the new level
struct NewNode {
  baldr::GraphId basenode;
  bool contract;

  NewNode(const baldr::GraphId& bn, const bool c)
      : basenode(bn),
        contract(c) {
  }
};

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

  /**
   * Get the nodes that remain in the new level in the hierarchy. Adds to
   * tiled lists of nodes (tilednodes_).
   * @param  base_level  Base level tile information
   * @param  new_level   Tile information for the new level.
   */
  bool GetNodesInNewLevel(const baldr::TileHierarchy::TileLevel& base_level,
                          const baldr::TileHierarchy::TileLevel& new_level);

  /**
   * Add a node to the new level. Map it back to the node on the base
   * level and create a mapping from the base level to the new node.
   * @param  graphtile  Tile access in the base level tile.
   * @param  nodeinfo   Node information in the base tile
   * @param  basenode   GraphId of the node in the base level.
   * @param  new_level  Tiling information for the new level
   */
  void AddNewNode(baldr::GraphTile* graphtile, const baldr::NodeInfo* nodeinfo,
                  const baldr::GraphId& basenode,
                  const baldr::TileHierarchy::TileLevel& new_level);

  /**
   * Check if the new node can be contracted to create a shortcut edge.
   * Use information from the base level to compare attributes and names.
   * @param  graphtile  Tile access in the base level tile.
   * @param  nodeinfo   Node information in the base tile.
   * @param  basenode   GraphId of the node in the base level.
   * @return  Returns true if the node can be contracted, false if not.
   */
  bool CanContract(baldr::GraphTile* graphtile,
                   const baldr::NodeInfo* nodeinfo,
                   const baldr::GraphId& basenode) const;

  /**
   * Form tiles in the new level
   */
  void FormTilesInNewLevel(const baldr::TileHierarchy::TileLevel& base_level,
                           const baldr::TileHierarchy::TileLevel& new_level);

  /**
   * Connect nodes in the base level to new nodes at the new level. This
   * inserts directed edges from the base node to the new node.
   */
  void ConnectBaseLevelToNewLevel(
                  const baldr::TileHierarchy::TileLevel& base_level,
                  const baldr::TileHierarchy::TileLevel& new_level);

  void AddConnectionsToBaseTile(const uint32_t basetileid,
                   const std::vector<NodeConnection>& connections);
};

}
}

#endif  // VALHALLA_MJOLNIR_HIERARCHYBUILDER_H
