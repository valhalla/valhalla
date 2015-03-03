#ifndef VALHALLA_MJOLNIR_GRAPHENHANCER_H
#define VALHALLA_MJOLNIR_GRAPHENHANCER_H

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <boost/property_tree/ptree.hpp>

#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/mjolnir/graphtilebuilder.h>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to enhance graph tile information at the local level.
 */
class GraphEnhancer {
 public:
  /**
   * Constructor
   * @param  pt  Property tree with tile/level information.
   */
  GraphEnhancer(const boost::property_tree::ptree& pt);

  /**
   * Enhance the local level graph tile information.
   */
  bool Enhance();

 private:
  // Tile hierarchy/level information
  baldr::TileHierarchy tile_hierarchy_;
  uint32_t local_level_;

  // Tile logic
  Tiles tiles_;

  // Graphreader
  baldr::GraphReader graphreader_;

  // Maximum density (km/km2)
  float maxdensity_;

  /**
   * Update directed edge speed based on density and other edge parameters like
   * surface type. TODO - add admin specific logic
   * @param  directededge  Directed edge to update.
   * @param  density       Road density.
   */
  void UpdateSpeed(DirectedEdgeBuilder& directededge, const float density);

  /**
   * Tests if the directed edge is unreachable by driving. If a driveable
   * edge cannot reach higher class roads and a search cannot expand after
   * a set number of iterations the edge is considered unreachable.
   * @param  directededge  Directed edge to test.
   * @return  Returns true if the edge is found to be unreachable.
   */
  bool IsUnreachable(DirectedEdgeBuilder& directededge);

  /**
   * Get the road density around the specified lat,lng position. This is a
   * value from 0-15 indicating a relative road density. THis can be used
   * in costing methods to help avoid dense, urban areas.
   * @param  ll            Lat,lng position
   * @param  localdensity (OUT) Local road density (within a smaller radius) -
   *                       might be useful for speed assignment. Units are
   *                       km of road per square kilometer
   * @return  Returns the relative road density (0-15) - higher values are
   *          more dense.
   */
  uint32_t GetDensity(const PointLL& ll, float& localdensity);

  /**
   * Process edge transitions from all other incoming edges onto the
   * specified directed edge.
   * @param  idx   Index of the directed edge - the to edge.
   * @param  directededge  Directed edge builder - set values.
   * @param  edge   Other directed edges at the node.
   * @param  ntrans  Number of transitions (either number of edges or max)
   * @param  start_heading  Headings of directed edges.
   */
  void ProcessEdgeTransitions(const uint32_t idx,
            DirectedEdgeBuilder& directededge,
            const DirectedEdge* edges, const uint32_t ntrans,
            uint32_t* heading);

  /**
   * Gets the stop likelihoood / impact at an intersection when transitioning
   * from one edge to another. This depends on the difference between the
   * classifications/importance of the from and to edge and the highest
   * classification of the remaining edges at the intersection. Low impact
   * values occur when the from and to roads are higher class roads than other
   * roads. There is less likelihood of having to stop in these cases (or stops
   * will usually be shorter duration). Where traffic lights are (or might be)
   * present it is more likely that a favorable "green" is present in the
   * direction of the higher classification. If classifications are all equal
   * the stop impact will depend on the classification. All directions are
   * likely to stop and duration is likely longer with higher classification
   * roads (e.g. a 4 way stop of tertiary roads is likely to be shorter than
   * a 4 way stop (with traffic light) at an intersection of 4 primary roads.
   * Higher stop impacts occur when the from and to edges are lower class
   * than the others. There is almost certainly a stop (stop sign, traffic
   * light) and longer waits are likely when a low class road crosses
   * a higher class road. Special cases occur for links (ramps/turn channels).
   * @return  Returns stop impact ranging from 0 (no likely impact) to
   *          7 - large impact.
   */
  uint32_t GetStopImpact(const uint32_t from, const uint32_t to,
                   const DirectedEdge* edges, const uint32_t count);

  /**
   * Get the index of the opposing edge at the end node. This is
   * on the local hierarchy (before adding transition and shortcut edges).
   * @param  startnode    Start node of the directed edge.
   * @param  directededge  Directed edge to match.
   */
  uint32_t GetOpposingEdgeIndex(const GraphId& startnode,
                                const DirectedEdge& edge);
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHENHANCER_H
