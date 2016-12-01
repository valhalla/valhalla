#include <thor/trip_path_controller.h>
#include <string>


namespace valhalla {
namespace thor {

const std::unordered_map<std::string, bool> TripPathController::kRouteAttributes = {
  { kEdgeNames, true },
  { kEdgeLength, true },
  { kEdgeSpeed, true },
  { kEdgeRoadClass, true },
  { kEdgeBeginHeading, true },
  { kEdgeEndHeading, true },
  { kEdgeBeginShapeIndex, true },
  { kEdgeEndShapeIndex, true },
  { kEdgeTraversability, true }
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
