#ifndef VALHALLA_THOR_TRIPPATHBUILDER_H_
#define VALHALLA_THOR_TRIPPATHBUILDER_H_

#include <cstdint>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/proto/trippath.pb.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/thor/attributes_controller.h>
#include <valhalla/thor/match_result.h>
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
  static odin::TripPath
  Build(const AttributesController& controller,
        baldr::GraphReader& graphreader,
        const std::shared_ptr<sif::DynamicCost>* mode_costing,
        const std::vector<PathInfo>& path,
        odin::Location& origin,
        odin::Location& dest,
        const std::list<odin::Location>& through_loc,
        const std::function<void()>* interrupt_callback = nullptr,
        std::unordered_map<size_t, std::pair<RouteDiscontinuity, RouteDiscontinuity>>*
            route_discontinuities = nullptr);

  /**
   * Add trip edge. (TODO more comments)
   * @param  controller    Controller to determine which attributes to set.
   * @param  edge          Identifier of an edge within the tiled, hierarchical graph.
   * @param  trip_id       Trip Id (0 if not a transit edge).
   * @param  block_id      Transit block Id (0 if not a transit edge)
   * @param  mode          Travel mode for the edge: Biking, walking, etc.
   * @param  directededge  Directed edge information.
   * @param  drive_right   Right side driving for this edge.
   * @param  trip_node     Trip node to add the edge information to.
   * @param  graphtile     Graph tile for accessing data.
   * @param   current_time Current time (seconds from midnight).
   * @param  length_percentage Scale for the edge length for the partial distance
   *                       at begin and end edges
   */
  static odin::TripPath_Edge* AddTripEdge(const AttributesController& controller,
                                          const baldr::GraphId& edge,
                                          const uint32_t trip_id,
                                          const uint32_t block_id,
                                          const sif::TravelMode mode,
                                          const uint8_t travel_type,
                                          const baldr::DirectedEdge* directededge,
                                          const bool drive_right,
                                          odin::TripPath_Node* trip_node,
                                          const baldr::GraphTile* graphtile,
                                          const uint32_t current_time,
                                          const float length_percentage = 1.f);

  /**
   * Add trip intersecting edge.
   * @param  controller   Controller to determine which attributes to set.
   * @param  directededge Directed edge on the path.
   * @param  prev_de  Previous directed edge on the path.
   * @param  local_edge_index  Index of the local intersecting path edge at intersection.
   * @param  nodeinfo  Node information of the intersection.
   * @param  trip_node  Trip node that will store the intersecting edge information.
   * @param  intersecting_de Intersecting directed edge. Will be nullptr except when
   *                         on the local hierarchy.
   */
  static void AddTripIntersectingEdge(const AttributesController& controller,
                                      const baldr::DirectedEdge* directededge,
                                      const baldr::DirectedEdge* prev_de,
                                      uint32_t local_edge_index,
                                      const baldr::NodeInfo* nodeinfo,
                                      odin::TripPath_Node* trip_node,
                                      const baldr::DirectedEdge* intersecting_de);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_TRIPPATHBUILDER_H_
