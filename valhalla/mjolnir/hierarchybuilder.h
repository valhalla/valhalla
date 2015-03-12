#ifndef VALHALLA_MJOLNIR_HIERARCHYBUILDER_H
#define VALHALLA_MJOLNIR_HIERARCHYBUILDER_H

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <boost/property_tree/ptree.hpp>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/mjolnir/graphtilebuilder.h>

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

// Simple structure to hold the 2 pair of directed edges at a node.
// First edge in the pair is incoming and second is outgoing
struct EdgePairs {
  std::pair<GraphId, GraphId> edge1;
  std::pair<GraphId, GraphId> edge2;
};

/**
 * Class used to construct temporary data used to build the initial graph.
 */
class HierarchyBuilder {
 public:

  /**
   * Build the set of hierarchies based on the TileHierarchy configuration
   * and the current local hierarchy.
   */
  static void Build(const boost::property_tree::ptree& pt);
};

}
}

#endif  // VALHALLA_MJOLNIR_HIERARCHYBUILDER_H
