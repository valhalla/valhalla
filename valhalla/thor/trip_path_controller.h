#ifndef VALHALLA_THOR_TRIP_PATH_CONTROLLER_H_
#define VALHALLA_THOR_TRIP_PATH_CONTROLLER_H_

#include <string>
#include <unordered_map>

namespace valhalla {
namespace thor {

// Edge keys
constexpr auto kEdgeNames = "edge.names";
constexpr auto kEdgeLength = "edge.length";
constexpr auto kEdgeSpeed = "edge.speed";
constexpr auto kEdgeRoadClass = "edge.road_class";
constexpr auto kEdgeBeginHeading = "edge.begin_heading";
constexpr auto kEdgeEndHeading = "edge.end_heading";
constexpr auto kEdgeBeginShapeIndex = "edge.begin_shape_index";
constexpr auto kEdgeEndShapeIndex = "edge.end_shape_index";
constexpr auto kEdgeTraversability = "edge.traversability";
constexpr auto kEdgeUse = "edge.use";
constexpr auto kEdgeToll = "edge.toll";
constexpr auto kEdgeUnpaved = "edge.unpaved";
constexpr auto kEdgeTunnel = "edge.tunnel";
constexpr auto kEdgeBridge = "edge.bridge";
constexpr auto kEdgeRoundabout = "edge.roundabout";
constexpr auto kEdgeInternalIntersection = "edge.internal_intersection";
constexpr auto kEdgeDriveOnRight = "edge.drive_on_right";
constexpr auto kEdgeEndNodeIndex = "edge.end_node_index";
constexpr auto kEdgeSign = "edge.sign";
constexpr auto kEdgeTravelMode = "edge.travel_mode";
constexpr auto kEdgeVehicleType = "edge.vehicle_type";
constexpr auto kEdgePedestrianType = "edge.pedestrian_type";
constexpr auto kEdgeBicycleType = "edge.bicycle_type";
constexpr auto kEdgeTransitType = "edge.transit_type";
constexpr auto kEdgeTransitRouteInfo = "edge.transit_route_info";
constexpr auto kEdgeId = "edge.id";
constexpr auto kEdgeWayId = "edge.way_id";
constexpr auto kEdgeWeightedGrade = "edge.weighted_grade";
constexpr auto kEdgeMaxUpwardGrade = "edge.max_upward_grade";
constexpr auto kEdgeMaxDownwardGrade = "edge.max_downward_grade";

// Node keys
constexpr auto kNodeIntersectingEdge = "node.intersecting_edge"; //TODO expand
constexpr auto kNodeElapsedTime = "node.elapsed_time";
constexpr auto kNodeaAdminIndex = "node.admin_index";
constexpr auto kNodeType = "node.type";
constexpr auto kNodeFork = "node.fork";
constexpr auto kNodetransitStopInfo = "node.transit_stop_info"; // TODO expand

// Top level: admin list, full shape, and shape bounding box keys
constexpr auto kAdmin = "admin";
constexpr auto kShape = "shape";
constexpr auto kBoundingBox = "bounding_box";


/**
 * Trip path controller for attributes
 */
struct TripPathController {

  /*
   * Attributes that are required by the route action to make guidance instructions.
   */
  static const std::unordered_map<std::string, bool> kRouteAttributes;

  /*
   * Constructor that will use the route attributes by default.
   */
  TripPathController(
      const std::unordered_map<std::string, bool>& new_attributes =
          TripPathController::kRouteAttributes);

  /**
   * Enable all of the attributes.
   */
  void enable_all();

  /**
   * Disable all of the attributes.
   */
  void disable_all();

  std::unordered_map<std::string, bool> attributes;
};

}
}

#endif  // VALHALLA_THOR_TRIP_PATH_CONTROLLER_H_
