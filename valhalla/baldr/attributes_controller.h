#pragma once
#include <valhalla/proto/options.pb.h>

#include <string>
#include <unordered_map>

namespace valhalla {
namespace baldr {

// Edge keys
constexpr std::string_view kEdgeNames = "edge.names";
constexpr std::string_view kEdgeLength = "edge.length";
constexpr std::string_view kEdgeSpeed = "edge.speed";
constexpr std::string_view kEdgeSpeedType = "edge.speed_type";
constexpr std::string_view kEdgeSpeedsFaded = "edge.speeds_faded";
constexpr std::string_view kEdgeSpeedsNonFaded = "edge.speeds_non_faded";
constexpr std::string_view kEdgeRoadClass = "edge.road_class";
constexpr std::string_view kEdgeBeginHeading = "edge.begin_heading";
constexpr std::string_view kEdgeEndHeading = "edge.end_heading";
constexpr std::string_view kEdgeBeginShapeIndex = "edge.begin_shape_index";
constexpr std::string_view kEdgeEndShapeIndex = "edge.end_shape_index";
constexpr std::string_view kEdgeTraversability = "edge.traversability";
constexpr std::string_view kEdgeUse = "edge.use";
constexpr std::string_view kEdgeToll = "edge.toll";
constexpr std::string_view kEdgeUnpaved = "edge.unpaved";
constexpr std::string_view kEdgeTunnel = "edge.tunnel";
constexpr std::string_view kEdgeBridge = "edge.bridge";
constexpr std::string_view kEdgeRoundabout = "edge.roundabout";
constexpr std::string_view kEdgeInternalIntersection = "edge.internal_intersection";
constexpr std::string_view kEdgeDriveOnRight = "edge.drive_on_right";
constexpr std::string_view kEdgeSurface = "edge.surface";
constexpr std::string_view kEdgeSignExitNumber = "edge.sign.exit_number";
constexpr std::string_view kEdgeSignExitBranch = "edge.sign.exit_branch";
constexpr std::string_view kEdgeSignExitToward = "edge.sign.exit_toward";
constexpr std::string_view kEdgeSignExitName = "edge.sign.exit_name";
constexpr std::string_view kEdgeSignGuideBranch = "edge.sign.guide_branch";
constexpr std::string_view kEdgeSignGuideToward = "edge.sign.guide_toward";
constexpr std::string_view kEdgeSignJunctionName = "edge.sign.junction_name";
constexpr std::string_view kEdgeSignGuidanceViewJunction = "edge.sign.guidance_view_junction";
constexpr std::string_view kEdgeSignGuidanceViewSignboard = "edge.sign.guidance_view_signboard";
constexpr std::string_view kEdgeTravelMode = "edge.travel_mode";
constexpr std::string_view kEdgeVehicleType = "edge.vehicle_type";
constexpr std::string_view kEdgePedestrianType = "edge.pedestrian_type";
constexpr std::string_view kEdgeBicycleType = "edge.bicycle_type";
constexpr std::string_view kEdgeTransitType = "edge.transit_type";
constexpr std::string_view kEdgeTransitRouteInfoOnestopId = "edge.transit_route_info.onestop_id";
constexpr std::string_view kEdgeTransitRouteInfoBlockId = "edge.transit_route_info.block_id";
constexpr std::string_view kEdgeTransitRouteInfoTripId = "edge.transit_route_info.trip_id";
constexpr std::string_view kEdgeTransitRouteInfoShortName = "edge.transit_route_info.short_name";
constexpr std::string_view kEdgeTransitRouteInfoLongName = "edge.transit_route_info.long_name";
constexpr std::string_view kEdgeTransitRouteInfoHeadsign = "edge.transit_route_info.headsign";
constexpr std::string_view kEdgeTransitRouteInfoColor = "edge.transit_route_info.color";
constexpr std::string_view kEdgeTransitRouteInfoTextColor = "edge.transit_route_info.text_color";
constexpr std::string_view kEdgeTransitRouteInfoDescription = "edge.transit_route_info.description";
constexpr std::string_view kEdgeTransitRouteInfoOperatorOnestopId =
    "edge.transit_route_info.operator_onestop_id";
constexpr std::string_view kEdgeTransitRouteInfoOperatorName =
    "edge.transit_route_info.operator_name";
constexpr std::string_view kEdgeTransitRouteInfoOperatorUrl = "edge.transit_route_info.operator_url";
constexpr std::string_view kEdgeId = "edge.id";
constexpr std::string_view kEdgeWayId = "edge.way_id";
constexpr std::string_view kEdgeWeightedGrade = "edge.weighted_grade";
constexpr std::string_view kEdgeMaxUpwardGrade = "edge.max_upward_grade";
constexpr std::string_view kEdgeMaxDownwardGrade = "edge.max_downward_grade";
constexpr std::string_view kEdgeMeanElevation = "edge.mean_elevation";
constexpr std::string_view kEdgeElevation = "edge.elevation";
constexpr std::string_view kEdgeLaneCount = "edge.lane_count";
constexpr std::string_view kEdgeLaneConnectivity = "edge.lane_connectivity";
constexpr std::string_view kEdgeCycleLane = "edge.cycle_lane";
constexpr std::string_view kEdgeBicycleNetwork = "edge.bicycle_network";
constexpr std::string_view kEdgeSacScale = "edge.sac_scale";
constexpr std::string_view kEdgeShoulder = "edge.shoulder";
constexpr std::string_view kEdgeSidewalk = "edge.sidewalk";
constexpr std::string_view kEdgeDensity = "edge.density";
constexpr std::string_view kEdgeSpeedLimit = "edge.speed_limit";
constexpr std::string_view kEdgeConditionalSpeedLimits = "edge.conditional_speed_limits";
constexpr std::string_view kEdgeTruckSpeed = "edge.truck_speed";
constexpr std::string_view kEdgeTruckRoute = "edge.truck_route";
constexpr std::string_view kEdgeDefaultSpeed = "edge.default_speed";
constexpr std::string_view kEdgeDestinationOnly = "edge.destination_only";
constexpr std::string_view kEdgeIsUrban = "edge.is_urban";
constexpr std::string_view kEdgeTaggedValues = "edge.tagged_values";
constexpr std::string_view kEdgeIndoor = "edge.indoor";
constexpr std::string_view kEdgeLandmarks = "edge.landmarks";
constexpr std::string_view kEdgeCountryCrossing = "edge.country_crossing";
constexpr std::string_view kEdgeForward = "edge.forward";
constexpr std::string_view kEdgeLevels = "edge.levels";
constexpr std::string_view kEdgeTrafficSignal = "edge.traffic_signal";

// Node keys
constexpr std::string_view kNodeIntersectingEdgeBeginHeading = "node.intersecting_edge.begin_heading";
constexpr std::string_view kNodeIntersectingEdgeFromEdgeNameConsistency =
    "node.intersecting_edge.from_edge_name_consistency";
constexpr std::string_view kNodeIntersectingEdgeToEdgeNameConsistency =
    "node.intersecting_edge.to_edge_name_consistency";
constexpr std::string_view kNodeIntersectingEdgeDriveability = "node.intersecting_edge.driveability";
constexpr std::string_view kNodeIntersectingEdgeCyclability = "node.intersecting_edge.cyclability";
constexpr std::string_view kNodeIntersectingEdgeWalkability = "node.intersecting_edge.walkability";
constexpr std::string_view kNodeIntersectingEdgeUse = "node.intersecting_edge.use";
constexpr std::string_view kNodeIntersectingEdgeRoadClass = "node.intersecting_edge.road_class";
constexpr std::string_view kNodeIntersectingEdgeLaneCount = "node.intersecting_edge.lane_count";
constexpr std::string_view kNodeIntersectingEdgeSignInfo = "node.intersecting_edge.sign_info";
constexpr std::string_view kNodeElapsedTime = "node.elapsed_time";
constexpr std::string_view kNodeAdminIndex = "node.admin_index";
constexpr std::string_view kNodeType = "node.type";
constexpr std::string_view kNodeTrafficSignal = "node.traffic_signal";
constexpr std::string_view kNodeFork = "node.fork";
constexpr std::string_view kNodeTransitPlatformInfoType = "node.transit_platform_info.type";
constexpr std::string_view kNodeTransitPlatformInfoOnestopId =
    "node.transit_platform_info.onestop_id";
constexpr std::string_view kNodeTransitPlatformInfoName = "node.transit_platform_info.name";
constexpr std::string_view kNodeTransitPlatformInfoStationOnestopId =
    "node.transit_platform_info.station_onestop_id";
constexpr std::string_view kNodeTransitPlatformInfoStationName =
    "node.transit_platform_info.station_name";
constexpr std::string_view kNodeTransitPlatformInfoArrivalDateTime =
    "node.transit_platform_info.arrival_date_time";
constexpr std::string_view kNodeTransitPlatformInfoDepartureDateTime =
    "node.transit_platform_info.departure_date_time";
constexpr std::string_view kNodeTransitPlatformInfoIsParentStop =
    "node.transit_platform_info.is_parent_stop";
constexpr std::string_view kNodeTransitPlatformInfoAssumedSchedule =
    "node.transit_platform_info.assumed_schedule";
constexpr std::string_view kNodeTransitPlatformInfoLatLon = "node.transit_platform_info.lat_lon";
constexpr std::string_view kNodeTransitStationInfoOnestopId = "node.transit_station_info.onestop_id";
constexpr std::string_view kNodeTransitStationInfoName = "node.transit_station_info.name";
constexpr std::string_view kNodeTransitStationInfoLatLon = "node.transit_station_info.lat_lon";
constexpr std::string_view kNodeTransitEgressInfoOnestopId = "node.transit_egress_info.onestop_id";
constexpr std::string_view kNodeTransitEgressInfoName = "node.transit_egress_info.name";
constexpr std::string_view kNodeTransitEgressInfoLatLon = "node.transit_egress_info.lat_lon";
constexpr std::string_view kNodeTimeZone = "node.time_zone";
constexpr std::string_view kNodeTransitionTime = "node.transition_time";

// Top level: osm changeset, admin list, and full shape keys
constexpr std::string_view kOsmChangeset = "osm_changeset";
constexpr std::string_view kAdminCountryCode = "admin.country_code";
constexpr std::string_view kAdminCountryText = "admin.country_text";
constexpr std::string_view kAdminStateCode = "admin.state_code";
constexpr std::string_view kAdminStateText = "admin.state_text";
constexpr std::string_view kShape = "shape";
constexpr std::string_view kIncidents = "incidents";

// Map matching ones nested to points and top level ones
constexpr std::string_view kMatchedPoint = "matched.point";
constexpr std::string_view kMatchedType = "matched.type";
constexpr std::string_view kMatchedEdgeIndex = "matched.edge_index";
constexpr std::string_view kMatchedBeginRouteDiscontinuity = "matched.begin_route_discontinuity";
constexpr std::string_view kMatchedEndRouteDiscontinuity = "matched.end_route_discontinuity";
constexpr std::string_view kMatchedDistanceAlongEdge = "matched.distance_along_edge";
constexpr std::string_view kMatchedDistanceFromTracePoint = "matched.distance_from_trace_point";
constexpr std::string_view kConfidenceScore = "confidence_score";
constexpr std::string_view kRawScore = "raw_score";

// Per-shape attributes
constexpr std::string_view kShapeAttributesTime = "shape_attributes.time";
constexpr std::string_view kShapeAttributesLength = "shape_attributes.length";
constexpr std::string_view kShapeAttributesSpeed = "shape_attributes.speed";
constexpr std::string_view kShapeAttributesSpeedLimit = "shape_attributes.speed_limit";
constexpr std::string_view kShapeAttributesClosure = "shape_attributes.closure";

// Categories
constexpr std::string_view kNodeCategory = "node.";
constexpr std::string_view kAdminCategory = "admin.";
constexpr std::string_view kMatchedCategory = "matched.";
constexpr std::string_view kShapeAttributesCategory = "shape_attributes.";

/**
 * Trip path controller for attributes
 */
struct AttributesController {

  // Attributes that are required by the route action to make guidance instructions.
  static const std::unordered_map<std::string_view, bool> kDefaultAttributes;

  static const std::unordered_set<std::string_view> kDefaultEnabledCategories;

  /**
   * Constructor that will use the default values for all of the attributes.
   */
  AttributesController();

  /**
   * Apply attribute filters from the request to the AttributesController. These filters
   * allow including or excluding specific attributes from the response in route,
   * trace_route, and trace_attributes actions.
   * @param options             request options
   * @param is_strict_filter    whether or not the include/exclude option is strict
   */
  AttributesController(const Options& options, bool is_strict_filter = false);

  bool operator()(const std::string_view& key) const {
    return attributes.at(key);
  }

  /**
   * Returns true if any category attribute is enabled, false otherwise.
   */
  bool category_attribute_enabled(const std::string_view& category) const {
    return enabled_categories.find(category) != enabled_categories.end();
  }

private:
  std::unordered_map<std::string_view, bool> attributes;

  std::unordered_set<std::string_view> enabled_categories;

  /**
   * Disable all of the attributes.
   */
  void disable_all();
};

} // namespace baldr
} // namespace valhalla
