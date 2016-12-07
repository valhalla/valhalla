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

void TryArgCtor(const std::unordered_map<std::string, bool>& new_attributes,
                size_t expected_size) {
  TripPathController controller(new_attributes);
  if (controller.attributes != new_attributes)
    throw runtime_error("Incorrect Constructor using argument attributes");
  if (controller.attributes.size() != expected_size)
    throw runtime_error("Incorrect Constructor using argument attributes size");
}

void TestArgCtor() {
  const std::unordered_map<std::string, bool> attributes = {
    { kEdgeNames, true },
    { kEdgeLength, false },
    { kEdgeSpeed, true },
    { kEdgeRoadClass, false }
  };

  TryArgCtor(attributes, attributes.size());
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

void TryNodeAttributeEnabled(const TripPathController& controller,
                             bool expected_response) {
  // If node_attribute_enabled does not equal expected response then throw error
  if (controller.node_attribute_enabled() != expected_response) {
    throw runtime_error(
        "Incorrect node_attribute_enabled response - expected: "
            + std::string(expected_response ? "true" : "false"));
  }
}

void TestNodeAttributeEnabled() {
  TripPathController controller;

  // Test default
  TryNodeAttributeEnabled(controller, true);

  // Test all node enabled
  controller.enable_all();
  TryNodeAttributeEnabled(controller, true);

  // Test all node disabled
  controller.disable_all();
  TryNodeAttributeEnabled(controller, false);

  // Test one node enabled
  controller.attributes.at(kNodeType) = true;
  TryNodeAttributeEnabled(controller, true);

  // Test some node enabled
  controller.attributes.at(kNodeType) = false;
  controller.attributes.at(kNodeIntersectingEdgeBeginHeading) = true;
  controller.attributes.at(kNodeTransitStopInfoType) = true;
  controller.attributes.at(kNodeElapsedTime) = true;
  controller.attributes.at(kNodeFork) = true;
  TryNodeAttributeEnabled(controller, true);

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

  // Test node_attribute_enabled
  suite.test(TEST_CASE(TestNodeAttributeEnabled));

  return suite.tear_down();
}
