#ifndef VALHALLA_THOR_ATTRIBUTES_CONTROLLER_H_
#define VALHALLA_THOR_ATTRIBUTES_CONTROLLER_H_

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
const std::string kEdgeSignGuideBranch = "edge.sign.guide_branch";
const std::string kEdgeSignGuideToward = "edge.sign.guide_toward";
const std::string kEdgeSignJunctionName = "edge.sign.junction_name";
const std::string kEdgeSignGuidanceViewJunction = "edge.sign.guidance_view_junction";
const std::string kEdgeSignGuidanceViewSignboard = "edge.sign.guidance_view_signboard";
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
const std::string kEdgeTransitRouteInfoOperatorOnestopId =
    "edge.transit_route_info.operator_onestop_id";
const std::string kEdgeTransitRouteInfoOperatorName = "edge.transit_route_info.operator_name";
const std::string kEdgeTransitRouteInfoOperatorUrl = "edge.transit_route_info.operator_url";
const std::string kEdgeId = "edge.id";
const std::string kEdgeWayId = "edge.way_id";
const std::string kEdgeWeightedGrade = "edge.weighted_grade";
const std::string kEdgeMaxUpwardGrade = "edge.max_upward_grade";
const std::string kEdgeMaxDownwardGrade = "edge.max_downward_grade";
const std::string kEdgeMeanElevation = "edge.mean_elevation";
const std::string kEdgeLaneCount = "edge.lane_count";
const std::string kEdgeLaneConnectivity = "edge.lane_connectivity";
const std::string kEdgeCycleLane = "edge.cycle_lane";
const std::string kEdgeBicycleNetwork = "edge.bicycle_network";
const std::string kEdgeSacScale = "edge.sac_scale";
const std::string kEdgeShoulder = "edge.shoulder";
const std::string kEdgeSidewalk = "edge.sidewalk";
const std::string kEdgeDensity = "edge.density";
const std::string kEdgeSpeedLimit = "edge.speed_limit";
const std::string kEdgeTruckSpeed = "edge.truck_speed";
const std::string kEdgeTruckRoute = "edge.truck_route";
const std::string kEdgeDefaultSpeed = "edge.default_speed";
const std::string kEdgeDestinationOnly = "edge.destination_only";
const std::string kEdgeIsUrban = "edge.is_urban";
const std::string kEdgeTaggedValues = "edge.tagged_values";

// Node keys
const std::string kNodeIntersectingEdgeBeginHeading = "node.intersecting_edge.begin_heading";
const std::string kNodeIntersectingEdgeFromEdgeNameConsistency =
    "node.intersecting_edge.from_edge_name_consistency";
const std::string kNodeIntersectingEdgeToEdgeNameConsistency =
    "node.intersecting_edge.to_edge_name_consistency";
const std::string kNodeIntersectingEdgeDriveability = "node.intersecting_edge.driveability";
const std::string kNodeIntersectingEdgeCyclability = "node.intersecting_edge.cyclability";
const std::string kNodeIntersectingEdgeWalkability = "node.intersecting_edge.walkability";
const std::string kNodeIntersectingEdgeUse = "node.intersecting_edge.use";
const std::string kNodeIntersectingEdgeRoadClass = "node.intersecting_edge.road_class";
const std::string kNodeIntersectingEdgeLaneCount = "node.intersecting_edge.lane_count";
const std::string kNodeIntersectingEdgeSignInfo = "node.intersecting_edge.sign_info";
const std::string kNodeElapsedTime = "node.elapsed_time";
const std::string kNodeAdminIndex = "node.admin_index";
const std::string kNodeType = "node.type";
const std::string kNodeFork = "node.fork";
const std::string kNodeTransitPlatformInfoType = "node.transit_platform_info.type";
const std::string kNodeTransitPlatformInfoOnestopId = "node.transit_platform_info.onestop_id";
const std::string kNodeTransitPlatformInfoName = "node.transit_platform_info.name";
const std::string kNodeTransitPlatformInfoStationOnestopId =
    "node.transit_platform_info.station_onestop_id";
const std::string kNodeTransitPlatformInfoStationName = "node.transit_platform_info.station_name";
const std::string kNodeTransitPlatformInfoArrivalDateTime =
    "node.transit_platform_info.arrival_date_time";
const std::string kNodeTransitPlatformInfoDepartureDateTime =
    "node.transit_platform_info.departure_date_time";
const std::string kNodeTransitPlatformInfoIsParentStop = "node.transit_platform_info.is_parent_stop";
const std::string kNodeTransitPlatformInfoAssumedSchedule =
    "node.transit_platform_info.assumed_schedule";
const std::string kNodeTransitPlatformInfoLatLon = "node.transit_platform_info.lat_lon";
const std::string kNodeTransitStationInfoOnestopId = "node.transit_station_info.onestop_id";
const std::string kNodeTransitStationInfoName = "node.transit_station_info.name";
const std::string kNodeTransitStationInfoLatLon = "node.transit_station_info.lat_lon";
const std::string kNodeTransitEgressInfoOnestopId = "node.transit_egress_info.onestop_id";
const std::string kNodeTransitEgressInfoName = "node.transit_egress_info.name";
const std::string kNodeTransitEgressInfoLatLon = "node.transit_egress_info.lat_lon";
const std::string kNodeTimeZone = "node.time_zone";
const std::string kNodeTransitionTime = "node.transition_time";

// Top level: osm changeset, admin list, and full shape keys
const std::string kOsmChangeset = "osm_changeset";
const std::string kAdminCountryCode = "admin.country_code";
const std::string kAdminCountryText = "admin.country_text";
const std::string kAdminStateCode = "admin.state_code";
const std::string kAdminStateText = "admin.state_text";
const std::string kShape = "shape";
const std::string kIncidents = "incidents";

// Map matching ones nested to points and top level ones
const std::string kMatchedPoint = "matched.point";
const std::string kMatchedType = "matched.type";
const std::string kMatchedEdgeIndex = "matched.edge_index";
const std::string kMatchedBeginRouteDiscontinuity = "matched.begin_route_discontinuity";
const std::string kMatchedEndRouteDiscontinuity = "matched.end_route_discontinuity";
const std::string kMatchedDistanceAlongEdge = "matched.distance_along_edge";
const std::string kMatchedDistanceFromTracePoint = "matched.distance_from_trace_point";
const std::string kConfidenceScore = "confidence_score";
const std::string kRawScore = "raw_score";

// Per-shape attributes
const std::string kShapeAttributesTime = "shape_attributes.time";
const std::string kShapeAttributesLength = "shape_attributes.length";
const std::string kShapeAttributesSpeed = "shape_attributes.speed";
const std::string kShapeAttributesSpeedLimit = "shape_attributes.speed_limit";
const std::string kShapeAttributesClosure = "shape_attributes.closure";

// Categories
const std::string kNodeCategory = "node.";
const std::string kAdminCategory = "admin.";
const std::string kMatchedCategory = "matched.";
const std::string kShapeAttributesCategory = "shape_attributes.";

/**
 * Trip path controller for attributes
 */
struct AttributesController {

  /*
   * Attributes that are required by the route action to make guidance instructions.
   */
  static const std::unordered_map<std::string, bool> kDefaultAttributes;

  /*
   * Constructor that will use the default values for all of the attributes.
   */
  AttributesController();

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

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_ATTRIBUTES_CONTROLLER_H_
