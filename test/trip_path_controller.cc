#include "test.h"

#include "config.h"
#include "thor/trip_path_controller.h"

using namespace std;
using namespace valhalla::thor;

namespace {

void TryCtor() {
  TripPathController controller;
  if (controller.attributes != TripPathController::kRouteAttributes)
    throw runtime_error("Incorrect Constructor using default route attributes");
}

void TestCtor() {
  TryCtor();
}

void TryArgCtor(const std::unordered_map<std::string, bool>& new_attributes) {
  TripPathController controller(new_attributes);
  if (controller.attributes != new_attributes)
    throw runtime_error("Incorrect Constructor using argument attributes");
}

void TestArgCtor() {
  const std::unordered_map<std::string, bool> attributes = {
    { kEdgeNames, true },
    { kEdgeLength, false },
    { kEdgeSpeed, true },
    { kEdgeRoadClass, false },
    { kEdgeBeginHeading, true },
    { kEdgeEndHeading, false },
    { kEdgeBeginShapeIndex, true },
    { kEdgeEndShapeIndex, false },
    { kEdgeTraversability, true }
  };

  TryArgCtor(attributes);
}

void TryEnableAll() {
  TripPathController controller;
  controller.enable_all();
  for (auto& pair : controller.attributes) {
    // If any pair value is false then throw error
    if (!pair.second)
      throw runtime_error("Incorrect enable_all value for " + pair.first);
  }
}

void TestEnableAll() {
  TryEnableAll();
}

void TryDisableAll() {
  TripPathController controller;
  controller.disable_all();
  for (auto& pair : controller.attributes) {
    // If any pair value is true then throw error
    if (pair.second)
      throw runtime_error("Incorrect disable_all value for " + pair.first);
  }
}

void TestDisableAll() {
  TryDisableAll();
}

}

int main() {
  test::suite suite("trip_path_controller");

  // Test Constructor
  suite.test(TEST_CASE(TestCtor));

  // Test Constructor with argument
  suite.test(TEST_CASE(TestArgCtor));

  // Test enable_all
  suite.test(TEST_CASE(TestEnableAll));

  // Test disable_all
  suite.test(TEST_CASE(TestDisableAll));

  return suite.tear_down();
}
