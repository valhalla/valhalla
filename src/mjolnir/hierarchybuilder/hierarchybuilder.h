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
#include "valhalla/baldr/graphconstants.h"
#include "valhalla/baldr/graphreader.h"
#include "mjolnir/graphtilebuilder.h"

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
  // Debug information
  uint32_t contractcount_;
  uint32_t shortcutcount_;
  uint32_t nodecounts_[16];

  // Tile hierarchy/level information
  baldr::TileHierarchy tile_hierarchy_;

  // Graphreader
  baldr::GraphReader graphreader_;

  // Lists of nodes on the new hierarchy
  std::vector<std::vector<NewNode>> tilednodes_;

  // Mapping from base level node to new node
  std::unordered_map<uint64_t, baldr::GraphId> nodemap_;

  // Mapping superseded edges
  std::unordered_map<uint64_t, bool> supersededmap_;

  /**
   * Get the nodes that remain in the new level in the hierarchy. Adds to
   * tiled lists of nodes (tilednodes_).
   * @param  base_level  Base level tile information
   * @param  new_level   Tile information for the new level.
   */
  bool GetNodesInNewLevel(const baldr::TileHierarchy::TileLevel& base_level,
                          const baldr::TileHierarchy::TileLevel& new_level);

  /**
   * Check if the new node can be contracted to create a shortcut edge.
   * Uses information from the base level to compare attributes and names.
   * @param  graphtile  Tile access in the base level tile.
   * @param  nodeinfo   Node information in the base tile.
   * @param  basenode   GraphId of the node in the base level.
   * @param  rcc        Road class (importance) cutoff.
   * @return  Returns true if the node can be contracted, false if not.
   */
  bool CanContract(baldr::GraphTile* graphtile,
                   const baldr::NodeInfo* nodeinfo,
                   const baldr::GraphId& basenode,
                   const baldr::RoadClass rcc);

  /**
   * Get the opposing edge for the specified edge. It is the edge outbound
   * from the end node of the specified edge that ends at the start node
   * of the edge and has matching length.
   * @param node   Start node.
   * @param edge   Outbound directed edge.
   * @return  Returns the GraphId of the opposing edge.
   */
  GraphId GetOpposingEdge(const GraphId& node, const DirectedEdge* edge);

  /**
   * Is the edge superseded (used in a shortcut edge)?
   * @param  edge   Base GraphId of the edge
   * @return  Returns true if the edge is part of a shortcut, false if not.
   */
  bool IsSuperseded(const baldr::GraphId& edge) const;

  /**
   * Form tiles in the new level
   * @param  base_level  Base level tile information
   * @param  new_level   Tile information for the new level.
   */
  void FormTilesInNewLevel(const baldr::TileHierarchy::TileLevel& base_level,
                           const baldr::TileHierarchy::TileLevel& new_level);

  /**
   * Adds shortcut edges from the node.
   */
  uint32_t AddShortcutEdges(const NewNode& newnode,
                            const baldr::GraphId& nodea,
                            const baldr::NodeInfo* oldnodeinfo,
                            baldr::GraphTile* tile,
                            const baldr::RoadClass rcc,
                            GraphTileBuilder& tilebuilder,
                            std::vector<DirectedEdgeBuilder>& directededges);

  /**
   * Gets the connected edge at the contracted node.
   */
  const baldr::DirectedEdge* GetConnectedEdge(baldr::GraphId& nodeb,
                   baldr::GraphId& priornode, baldr::RoadClass rcc);

  /**
   * Connect edges on the shortcut. Appends shape.
   */
  float ConnectEdges(const baldr::GraphId& basenode,
                     const baldr::DirectedEdge* directededge,
                     std::vector<midgard::PointLL>& shape);

  /**
   * Connect nodes in the base level to new nodes at the new level. This
   * inserts directed edges from the base node to the new node.
   * @param  base_level  Base level tile information
   * @param  new_level   Tile information for the new level.
   */
  void ConnectBaseLevelToNewLevel(
                   const baldr::TileHierarchy::TileLevel& base_level,
                   const baldr::TileHierarchy::TileLevel& new_level);

  /**
   * Adds connections (directed edges) from the base tile to nodes in the
   * new tile. Updates the base tile.
   * @param  basetileid  Base tile Id to update.
   * @param  connections List of connections between nodes in the base tile
   *                     to nodes in the new tile.
   */
  void AddConnectionsToBaseTile(const uint32_t basetileid,
                   const std::vector<NodeConnection>& connections);
};

}
}

#endif  // VALHALLA_MJOLNIR_HIERARCHYBUILDER_H
