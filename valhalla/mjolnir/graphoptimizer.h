#ifndef VALHALLA_MJOLNIR_GRAPHOPTIMIZER_H
#define VALHALLA_MJOLNIR_GRAPHOPTIMIZER_H

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

/**
 * Class used to optimize the graph. Creates opposing edge indexes.
 * TODO - intersection costing? elevation factors?
 */
class GraphOptimizer {
 public:
  /**
   * Constructor
   */
  GraphOptimizer(const boost::property_tree::ptree& pt);

  /**
   * Optimize the graph tiles.
   */
  bool Optimize();

 protected:
  // Tile hierarchy/level information
  baldr::TileHierarchy tile_hierarchy_;

  // Graphreader
  baldr::GraphReader graphreader_;

  /**
   * Get the opposing edge index for the specified edge. It is the index of
   * the outbound directed edge from the end node of the specified directed
   * edge that ends at the start node and has matching length.
   * @param node   Start node.
   * @param edge   Outbound directed edge.
   * @return  Returns the index of the opposing edge at the end node.
   */
  uint32_t GetOpposingEdgeIndex(const GraphId& node, DirectedEdge& edge);
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHOPTIMIZER_H
