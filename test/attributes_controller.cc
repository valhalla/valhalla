
#include "test.h"

#include "config.h"
#include "thor/attributes_controller.h"

using namespace std;
using namespace valhalla::thor;

namespace {

void TryCtor() {
  AttributesController controller;
  if (controller.attributes != AttributesController::kDefaultAttributes)
    throw runtime_error("Incorrect Constructor using default attributes");
}

void TestCtor() {
  TryCtor();
}

void TryArgCtor(size_t expected_size) {
  AttributesController controller;
  if (controller.attributes != AttributesController::kDefaultAttributes)
    throw runtime_error("Incorrect Constructor");
  if (controller.attributes.size() != expected_size)
    throw runtime_error("Incorrect Constructor size");
}

void TestArgCtor() {
  TryArgCtor(AttributesController::kDefaultAttributes.size());
}

void TryDisableAll() {
  AttributesController controller;
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

void TryCategoryAttributeEnabled(const AttributesController& controller,
                                 const std::string& category,
                                 bool expected_response) {
  // If category_attribute_enabled does not equal expected response then throw error
  if (controller.category_attribute_enabled(category) != expected_response) {
    throw runtime_error("Incorrect cateogry_attribute_enabled response - expected: " +
                        std::string(expected_response ? "true" : "false"));
  }
}

void TestNodeAttributeEnabled() {
  AttributesController controller;

  // Test default
  TryCategoryAttributeEnabled(controller, kNodeCategory, true);

  // Test all node disabled
  controller.disable_all();
  TryCategoryAttributeEnabled(controller, kNodeCategory, false);

  // Test one node enabled
  controller.attributes.at(kNodeType) = true;
  TryCategoryAttributeEnabled(controller, kNodeCategory, true);

  // Test some node enabled
  controller.attributes.at(kNodeType) = false;
  controller.attributes.at(kNodeIntersectingEdgeBeginHeading) = true;
  controller.attributes.at(kNodeTransitPlatformInfoType) = true;
  controller.attributes.at(kNodeElapsedTime) = true;
  controller.attributes.at(kNodeFork) = true;
  TryCategoryAttributeEnabled(controller, kNodeCategory, true);
}

void TestAdminAttributeEnabled() {
  AttributesController controller;

  // Test default
  TryCategoryAttributeEnabled(controller, kAdminCategory, true);

  // Test all admin disabled
  controller.disable_all();
  TryCategoryAttributeEnabled(controller, kAdminCategory, false);

  // Test one admin enabled
  controller.attributes.at(kAdminCountryCode) = true;
  TryCategoryAttributeEnabled(controller, kAdminCategory, true);

  // Test some admin enabled
  controller.attributes.at(kAdminCountryCode) = false;
  controller.attributes.at(kAdminCountryText) = true;
  controller.attributes.at(kAdminStateCode) = false;
  controller.attributes.at(kAdminStateText) = true;
  TryCategoryAttributeEnabled(controller, kAdminCategory, true);
}

} // namespace

int main() {
  test::suite suite("trip_path_controller");

  // Test Constructor
  suite.test(TEST_CASE(TestCtor));

  // Test Constructor with argument
  suite.test(TEST_CASE(TestArgCtor));

  // Test disable_all
  suite.test(TEST_CASE(TestDisableAll));

  // Test node category_attribute_enabled
  suite.test(TEST_CASE(TestNodeAttributeEnabled));

  // Test admin category_attribute_enabled
  suite.test(TEST_CASE(TestAdminAttributeEnabled));

  return suite.tear_down();
}
