#include "baldr/attributes_controller.h"
#include "midgard/logging.h"

#include <string>

namespace valhalla {
namespace baldr {

namespace {
std::unordered_set<std::string_view>
PrecomputeEnabledCategories(const std::unordered_map<std::string_view, bool>& attributes) {
  std::unordered_set<std::string_view> enabled_categories;
  for (const auto& pair : attributes) {
    if (!pair.second) {
      continue;
    }

    const auto dot_pos = pair.first.find('.');
    if (dot_pos == std::string_view::npos) {
      // it may happen for some attributes that they don't have category (e.g. kOsmChangeset)
      continue;
    }

    // must have a dot in the end
    auto category = pair.first.substr(0, dot_pos + 1);
    enabled_categories.insert(category);
  }
  return enabled_categories;
}
} // namespace

/*
 * Map of attributes that a user can request to enable or disable, and their defaults.
 * Most attributes are enabled by default but a few additional attributes are disabled
 * unless explicitly included with the filter attributes request option.
 */
const std::unordered_map<std::string_view, bool> AttributesController::kDefaultAttributes = {
    // Edge keys
    {kEdgeNames, true},
    {kEdgeLength, true},
    {kEdgeSpeed, true},
    {kEdgeSpeedType, true},
    {kEdgeSpeedsFaded, true},
    {kEdgeSpeedsNonFaded, true},
    {kEdgeRoadClass, true},
    {kEdgeBeginHeading, true},
    {kEdgeEndHeading, true},
    {kEdgeBeginShapeIndex, true},
    {kEdgeEndShapeIndex, true},
    {kEdgeTraversability, true},
    {kEdgeUse, true},
    {kEdgeToll, true},
    {kEdgeUnpaved, true},
    {kEdgeTunnel, true},
    {kEdgeBridge, true},
    {kEdgeRoundabout, true},
    {kEdgeInternalIntersection, true},
    {kEdgeDriveOnRight, true},
    {kEdgeSurface, true},
    {kEdgeSignExitNumber, true},
    {kEdgeSignExitBranch, true},
    {kEdgeSignExitToward, true},
    {kEdgeSignExitName, true},
    {kEdgeSignGuideBranch, true},
    {kEdgeSignGuideToward, true},
    {kEdgeSignJunctionName, true},
    {kEdgeSignGuidanceViewJunction, true},
    {kEdgeSignGuidanceViewSignboard, true},
    {kEdgeTravelMode, true},
    {kEdgeVehicleType, true},
    {kEdgePedestrianType, true},
    {kEdgeBicycleType, true},
    {kEdgeTransitType, true},
    {kEdgeTransitRouteInfoOnestopId, true},
    {kEdgeTransitRouteInfoBlockId, true},
    {kEdgeTransitRouteInfoTripId, true},
    {kEdgeTransitRouteInfoShortName, true},
    {kEdgeTransitRouteInfoLongName, true},
    {kEdgeTransitRouteInfoHeadsign, true},
    {kEdgeTransitRouteInfoColor, true},
    {kEdgeTransitRouteInfoTextColor, true},
    {kEdgeTransitRouteInfoDescription, true},
    {kEdgeTransitRouteInfoOperatorOnestopId, true},
    {kEdgeTransitRouteInfoOperatorName, true},
    {kEdgeTransitRouteInfoOperatorUrl, true},
    {kEdgeId, true},
    {kEdgeWayId, true},
    {kEdgeWeightedGrade, true},
    {kEdgeMaxUpwardGrade, true},
    {kEdgeMaxDownwardGrade, true},
    {kEdgeMeanElevation, true},
    {kEdgeLaneCount, true},
    {kEdgeLaneConnectivity, true},
    {kEdgeCycleLane, true},
    {kEdgeBicycleNetwork, true},
    {kEdgeElevation, false},
    {kEdgeSacScale, true},
    {kEdgeShoulder, true},
    {kEdgeSidewalk, true},
    {kEdgeDensity, true},
    {kEdgeSpeedLimit, true},
    {kEdgeConditionalSpeedLimits, true},
    {kEdgeTruckSpeed, true},
    {kEdgeTruckRoute, true},
    {kEdgeDefaultSpeed, true},
    {kEdgeDestinationOnly, true},
    {kEdgeIsUrban, false},
    {kEdgeTaggedValues, true},
    {kEdgeIndoor, true},
    {kEdgeLandmarks, true},
    {kEdgeCountryCrossing, true},
    {kEdgeForward, true},
    {kEdgeLevels, true},
    {kEdgeTrafficSignal, true},

    // Node keys
    {kIncidents, false},
    {kNodeIntersectingEdgeBeginHeading, true},
    {kNodeIntersectingEdgeFromEdgeNameConsistency, true},
    {kNodeIntersectingEdgeToEdgeNameConsistency, true},
    {kNodeIntersectingEdgeDriveability, true},
    {kNodeIntersectingEdgeCyclability, true},
    {kNodeIntersectingEdgeWalkability, true},
    {kNodeIntersectingEdgeUse, true},
    {kNodeIntersectingEdgeRoadClass, true},
    {kNodeIntersectingEdgeLaneCount, true},
    {kNodeIntersectingEdgeSignInfo, true},
    {kNodeElapsedTime, true},
    {kNodeAdminIndex, true},
    {kNodeType, true},
    {kNodeTrafficSignal, true},
    {kNodeFork, true},
    {kNodeTransitPlatformInfoType, true},
    {kNodeTransitPlatformInfoOnestopId, true},
    {kNodeTransitPlatformInfoName, true},
    {kNodeTransitPlatformInfoStationOnestopId, true},
    {kNodeTransitPlatformInfoStationName, true},
    {kNodeTransitPlatformInfoArrivalDateTime, true},
    {kNodeTransitPlatformInfoDepartureDateTime, true},
    {kNodeTransitPlatformInfoIsParentStop, true},
    {kNodeTransitPlatformInfoAssumedSchedule, true},
    {kNodeTransitPlatformInfoLatLon, true},
    {kNodeTransitStationInfoOnestopId, true},
    {kNodeTransitStationInfoName, true},
    {kNodeTransitStationInfoLatLon, true},
    {kNodeTransitEgressInfoOnestopId, true},
    {kNodeTransitEgressInfoName, true},
    {kNodeTransitEgressInfoLatLon, true},
    {kNodeTimeZone, true},
    {kNodeTransitionTime, true},

    // Top level: admin list, full shape, and shape bounding box keys
    {kOsmChangeset, true},
    {kAdminCountryCode, true},
    {kAdminCountryText, true},
    {kAdminStateCode, true},
    {kAdminStateText, true},
    {kShape, true},
    {kMatchedPoint, true},
    {kMatchedType, true},
    {kMatchedEdgeIndex, true},
    {kMatchedBeginRouteDiscontinuity, true},
    {kMatchedEndRouteDiscontinuity, true},
    {kMatchedDistanceAlongEdge, true},
    {kMatchedDistanceFromTracePoint, true},
    {kConfidenceScore, true},
    {kRawScore, true},

    // Per-shape attributes
    {kShapeAttributesTime, false},
    {kShapeAttributesLength, false},
    {kShapeAttributesSpeed, false},
    {kShapeAttributesSpeedLimit, false},
    {kShapeAttributesClosure, false},
};

const std::unordered_set<std::string_view> AttributesController::kDefaultEnabledCategories =
    PrecomputeEnabledCategories(kDefaultAttributes);

AttributesController::AttributesController() {
  attributes = kDefaultAttributes;
  enabled_categories = kDefaultEnabledCategories;
}

AttributesController::AttributesController(const Options& options, bool is_strict_filter) {
  // Set default controller
  attributes = kDefaultAttributes;

  switch (options.filter_action()) {
    case (FilterAction::include): {
      if (is_strict_filter)
        disable_all();
      for (const auto& filter_attribute : options.filter_attributes()) {
        try {
          attributes.at(filter_attribute) = true;
        } catch (...) { LOG_ERROR("Invalid filter attribute " + filter_attribute); }
      }
      break;
    }
    case (FilterAction::exclude): {
      for (const auto& filter_attribute : options.filter_attributes()) {
        try {
          attributes.at(filter_attribute) = false;
        } catch (...) { LOG_ERROR("Invalid filter attribute " + filter_attribute); }
      }
      break;
    }
    default:
      break;
  }

  // Set the edge elevation attributes based on elevation interval being set
  attributes.at(kEdgeElevation) = options.elevation_interval() > 0.0f;

  enabled_categories = PrecomputeEnabledCategories(attributes);
}

void AttributesController::disable_all() {
  for (auto& pair : attributes) {
    pair.second = false;
  }
}

} // namespace baldr
} // namespace valhalla
