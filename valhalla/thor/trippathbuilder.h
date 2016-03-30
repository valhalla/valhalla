#ifndef VALHALLA_THOR_TRIPPATHBUILDER_H_
#define VALHALLA_THOR_TRIPPATHBUILDER_H_

#include <vector>
#include <map>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/trippath.pb.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/thor/pathinfo.h>

namespace valhalla {
namespace thor {

/**
 * Algorithm to create a trip path output from a list of directed edges.
 */
class TripPathBuilder {
 public:
  /**
   * Constructor.
   */
  TripPathBuilder();

  /**
   * Destructor
   */
  virtual ~TripPathBuilder();

  /**
   * Format the trip path output given the edges on the path.
   * For now just return length. TODO - modify to return trip path.
   */
  static odin::TripPath Build(baldr::GraphReader& graphreader,
             const std::vector<PathInfo>& path,
             baldr::PathLocation& origin,
             baldr::PathLocation& dest,
             const std::vector<baldr::PathLocation>& through_loc);

  /**
   * Add trip edge. (TODO more comments)
   * @param  idx  Index of the directed edge within the tile.
   * @param  trip_id       Trip Id (0 if not a transit edge).
   * @param  block_id      Transit block Id (0 if not a transit edge)
   * @param  mode          Travel mode for the edge: Biking, walking, etc.
   * @param  directededge  Directed edge information.
   * @param  trip_node     Trip node to add the edge information to.
   * @param  graphtile     Graph tile for accessing data.
   * @param  length_pct    Scale for the edge length for the partial distance
   *                       at begin and end edges
   */
  static odin::TripPath_Edge* AddTripEdge(const uint32_t idx,
                                          const uint32_t trip_id,
                                          const uint32_t block_id,
                                          const sif::TravelMode mode,
                                          const baldr::DirectedEdge* directededge,
                                          odin::TripPath_Node* trip_node,
                                          const baldr::GraphTile* graphtile,
                                          const float length_percentage = 1.f);

  /**
    * Add trip intersecting edge.
    * @param  edge_index  Index of the local intersecting path edge at intersection.
    * @param  prev_edge_index  Index of the local previous path edge at intersection.
    * @param  curr_edge_index  Index of the local current path edge at intersection.
    * @param  nodeinfo  Node information of the intersection.
    * @param  trip_node  Trip node that will store the intersecting edge information.
    * @param  intersecting_de Intersecting directed edge. Will be nullptr except when
    *                         on the local hierarchy.
    */
   static void AddTripIntersectingEdge(uint32_t local_edge_index,
                                       uint32_t prev_edge_index,
                                       uint32_t curr_edge_index,
                                       const baldr::NodeInfo* nodeinfo,
                                       odin::TripPath_Node* trip_node,
                                       const baldr::DirectedEdge* intersecting_de);

};

}
}

#endif  // VALHALLA_THOR_TRIPPATHBUILDER_H_
