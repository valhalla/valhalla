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
  { kEdgeSign, true },
  { kEdgeTravelMode, true },
  { kEdgeVehicleType, true },
  { kEdgePedestrianType, true },
  { kEdgeBicycleType, true },
  { kEdgeTransitType, true },
  { kEdgeTransitRouteInfo, true },
  { kEdgeId, true },
  { kEdgeWayId, true },
  { kEdgeWeightedGrade, true },
  { kEdgeMaxUpwardGrade, true },
  { kEdgeMaxDownwardGrade, true },

  // Node keys
  { kNodeIntersectingEdge, true },
  { kNodeElapsedTime, true },
  { kNodeaAdminIndex, true },
  { kNodeType, true },
  { kNodeFork, true },
  { kNodetransitStopInfo, true },

  // Top level: admin list, full shape, and shape bounding box keys
  { kAdmin, true },
  { kShape, true },
  { kBoundingBox, true }
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
    pair.second = false;;
  }
}

}
}
