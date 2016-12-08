#include <thor/trip_path_controller.h>
#include <string>


namespace valhalla {
namespace thor {

const std::unordered_map<std::string, bool> TripPathController::kRouteAttributes = {
  // Edge keys
  { kEdgeNames, true },
  { kEdgeLength, true },
  { kEdgeSpeed, true },
  { kEdgeRoadClass, true },
  { kEdgeBeginHeading, true },
  { kEdgeEndHeading, true },
  { kEdgeBeginShapeIndex, true },
  { kEdgeEndShapeIndex, true },
  { kEdgeTraversability, true },
  { kEdgeUse, true },
  { kEdgeToll, true },
  { kEdgeUnpaved, true },
  { kEdgeTunnel, true },
  { kEdgeBridge, true },
  { kEdgeRoundabout, true },
  { kEdgeInternalIntersection, true },
  { kEdgeDriveOnRight, true },
  { kEdgeEndNodeIndex, true },
  { kEdgeSignExitNumber, true },
  { kEdgeSignExitBranch, true },
  { kEdgeSignExitToward, true },
  { kEdgeSignExitName, true },
  { kEdgeTravelMode, true },
  { kEdgeVehicleType, true },
  { kEdgePedestrianType, true },
  { kEdgeBicycleType, true },
  { kEdgeTransitType, true },
  { kEdgeTransitRouteInfoOnestopId, true },
  { kEdgeTransitRouteInfoBlockId, true },
  { kEdgeTransitRouteInfoTripId, true },
  { kEdgeTransitRouteInfoShortName, true },
  { kEdgeTransitRouteInfoLongName, true },
  { kEdgeTransitRouteInfoHeadsign, true },
  { kEdgeTransitRouteInfoColor, true },
  { kEdgeTransitRouteInfoTextColor, true },
  { kEdgeTransitRouteInfoDescription, true },
  { kEdgeTransitRouteInfoOperatorOnestopId, true },
  { kEdgeTransitRouteInfoOperatorName, true },
  { kEdgeTransitRouteInfoOperatorUrl, true },
  { kEdgeId, true },
  { kEdgeWayId, true },
  { kEdgeWeightedGrade, true },
  { kEdgeMaxUpwardGrade, true },
  { kEdgeMaxDownwardGrade, true },

  // Node keys
  { kNodeIntersectingEdgeBeginHeading, true },
  { kNodeIntersectingEdgePrevNameConsistency, true },
  { kNodeIntersectingEdgeCurrNameConsistency, true },
  { kNodeIntersectingEdgeDriveability, true },
  { kNodeIntersectingEdgeCyclability, true },
  { kNodeIntersectingEdgeWalkability, true },
  { kNodeElapsedTime, true },
  { kNodeaAdminIndex, true },
  { kNodeType, true },
  { kNodeFork, true },
  { kNodeTransitStopInfoType, true },
  { kNodeTransitStopInfoOnestopId, true },
  { kNodetransitStopInfoName, true },
  { kNodeTransitStopInfoArrivalDateTime, true },
  { kNodeTransitStopInfoDepartureDateTime, true },
  { kNodeTransitStopInfoIsParentStop, true },
  { kNodeTransitStopInfoAssumedSchedule, true },
  { kNodeTransitStopInfoLatLon, true },

  // Top level: admin list, full shape, and shape bounding box keys
  { kAdminCountryCode, true },
  { kAdminCountryText, true },
  { kAdminStateCode, true },
  { kAdminStateText, true },
  { kShape, true }
};

TripPathController::TripPathController(
    const std::unordered_map<std::string, bool>& new_attributes) {
  attributes = new_attributes;
}

void TripPathController::enable_all() {
  for (auto& pair : attributes) {
    pair.second = true;
  }
}

void TripPathController::disable_all() {
  for (auto& pair : attributes) {
    pair.second = false;
  }
}

bool TripPathController::node_attribute_enabled() const {
  for (const auto& pair : attributes) {
    // if the key starts with the node key prefix and it is enabled
    // then return true
    if ((pair.first.compare(0, kNodeKeyPrefix.size(), kNodeKeyPrefix) == 0)
        && pair.second) {
      return true;
    }
  }
  return false;
}

}
}
