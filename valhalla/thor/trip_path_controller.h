#ifndef VALHALLA_THOR_TRIP_PATH_CONTROLLER_H_
#define VALHALLA_THOR_TRIP_PATH_CONTROLLER_H_

#include <string>
#include <unordered_map>

namespace valhalla {
namespace thor {

// Edge keys
const std::string kEdgeNames = "edge.names";
const std::string kEdgeLength = "edge.length";
const std::string kEdgeSpeed = "edge.speed";
const std::string kEdgeRoadClass = "edge.road_class";
const std::string kEdgeBeginHeading = "edge.begin_heading";
const std::string kEdgeEndHeading = "edge.end_heading";
const std::string kEdgeBeginShapeIndex = "edge.begin_shape_index";
const std::string kEdgeEndShapeIndex = "edge.end_shape_index";
const std::string kEdgeTraversability = "edge.traversability";
const std::string kEdgeUse = "edge.use";
const std::string kEdgeToll = "edge.toll";
const std::string kEdgeUnpaved = "edge.unpaved";
const std::string kEdgeTunnel = "edge.tunnel";
const std::string kEdgeBridge = "edge.bridge";
const std::string kEdgeRoundabout = "edge.roundabout";
const std::string kEdgeInternalIntersection = "edge.internal_intersection";
const std::string kEdgeDriveOnRight = "edge.drive_on_right";
const std::string kEdgeSurface = "edge.surface";
const std::string kEdgeSignExitNumber = "edge.sign.exit_number";
const std::string kEdgeSignExitBranch = "edge.sign.exit_branch";
const std::string kEdgeSignExitToward = "edge.sign.exit_toward";
const std::string kEdgeSignExitName = "edge.sign.exit_name";
const std::string kEdgeTravelMode = "edge.travel_mode";
const std::string kEdgeVehicleType = "edge.vehicle_type";
const std::string kEdgePedestrianType = "edge.pedestrian_type";
const std::string kEdgeBicycleType = "edge.bicycle_type";
const std::string kEdgeTransitType = "edge.transit_type";
const std::string kEdgeTransitRouteInfoOnestopId = "edge.transit_route_info.onestop_id";
const std::string kEdgeTransitRouteInfoBlockId = "edge.transit_route_info.block_id";
const std::string kEdgeTransitRouteInfoTripId = "edge.transit_route_info.trip_id";
const std::string kEdgeTransitRouteInfoShortName = "edge.transit_route_info.short_name";
const std::string kEdgeTransitRouteInfoLongName = "edge.transit_route_info.long_name";
const std::string kEdgeTransitRouteInfoHeadsign = "edge.transit_route_info.headsign";
const std::string kEdgeTransitRouteInfoColor = "edge.transit_route_info.color";
const std::string kEdgeTransitRouteInfoTextColor = "edge.transit_route_info.text_color";
const std::string kEdgeTransitRouteInfoDescription = "edge.transit_route_info.description";
const std::string kEdgeTransitRouteInfoOperatorOnestopId = "edge.transit_route_info.operator_onestop_id";
const std::string kEdgeTransitRouteInfoOperatorName = "edge.transit_route_info.operator_name";
const std::string kEdgeTransitRouteInfoOperatorUrl = "edge.transit_route_info.operator_url";
const std::string kEdgeId = "edge.id";
const std::string kEdgeWayId = "edge.way_id";
const std::string kEdgeWeightedGrade = "edge.weighted_grade";
const std::string kEdgeMaxUpwardGrade = "edge.max_upward_grade";
const std::string kEdgeMaxDownwardGrade = "edge.max_downward_grade";
const std::string kEdgeLaneCount = "edge.lane_count";
const std::string kEdgeCycleLane = "edge.cycle_lane";
const std::string kEdgeBicycleNetwork = "edge.bicycle_network";
const std::string kEdgeSidewalk = "edge.sidewalk";
const std::string kEdgeDensity = "edge.density";
const std::string kEdgeSpeedLimit = "edge.speed_limit";
const std::string kEdgeTruckSpeed = "edge.truck_speed";
const std::string kEdgeTruckRoute = "edge.truck_route";

// Node keys
const std::string kNodeIntersectingEdgeBeginHeading = "node.intersecting_edge.begin_heading";
const std::string kNodeIntersectingEdgeFromEdgeNameConsistency = "node.intersecting_edge.from_edge_name_consistency";
const std::string kNodeIntersectingEdgeToEdgeNameConsistency = "node.intersecting_edge.to_edge_name_consistency";
const std::string kNodeIntersectingEdgeDriveability = "node.intersecting_edge.driveability";
const std::string kNodeIntersectingEdgeCyclability = "node.intersecting_edge.cyclability";
const std::string kNodeIntersectingEdgeWalkability = "node.intersecting_edge.walkability";
const std::string kNodeElapsedTime = "node.elapsed_time";
const std::string kNodeaAdminIndex = "node.admin_index";
const std::string kNodeType = "node.type";
const std::string kNodeFork = "node.fork";
const std::string kNodeTransitStopInfoType = "node.transit_stop_info.type";
const std::string kNodeTransitStopInfoOnestopId = "node.transit_stop_info.onestop_id";
const std::string kNodetransitStopInfoName = "node.transit_stop_info.name";
const std::string kNodeTransitStopInfoArrivalDateTime = "node.transit_stop_info.arrival_date_time";
const std::string kNodeTransitStopInfoDepartureDateTime = "node.transit_stop_info.departure_date_time";
const std::string kNodeTransitStopInfoIsParentStop = "node.transit_stop_info.is_parent_stop";
const std::string kNodeTransitStopInfoAssumedSchedule = "node.transit_stop_info.assumed_schedule";
const std::string kNodeTransitStopInfoLatLon = "node.transit_stop_info.lat_lon";
const std::string kNodeTimeZone = "node.time_zone";

// Top level: osm chnageset, admin list, and full shape keys
const std::string kOsmChangeset = "osm_changeset";
const std::string kAdminCountryCode = "admin.country_code";
const std::string kAdminCountryText = "admin.country_text";
const std::string kAdminStateCode = "admin.state_code";
const std::string kAdminStateText = "admin.state_text";
const std::string kShape = "shape";

// Categories
const std::string kNodeCategory = "node.";
const std::string kAdminCategory = "admin.";


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

  /**
   * Returns true if any category attribute is enabled, false otherwise.
   */
  bool category_attribute_enabled(const std::string& category) const;

  std::unordered_map<std::string, bool> attributes;
};

}
}

#endif  // VALHALLA_THOR_TRIP_PATH_CONTROLLER_H_
