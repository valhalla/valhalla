
#include "baldr/attributes_controller.h"
#include "config.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;

namespace {

TEST(AttrController, TestCtorDefautAttributes) {
  AttributesController controller;
  EXPECT_EQ(controller.attributes, AttributesController::kDefaultAttributes);
}

void TryArgCtor(size_t expected_size) {
  AttributesController controller;
  EXPECT_EQ(controller.attributes, AttributesController::kDefaultAttributes);
  EXPECT_EQ(controller.attributes.size(), expected_size);
}

TEST(AttrController, TestArgCtor) {
  TryArgCtor(AttributesController::kDefaultAttributes.size());
}

void TryDisableAll() {
  AttributesController controller;
  controller.disable_all();
  for (auto& pair : controller.attributes) {
    // If any pair value is true then throw error
    EXPECT_FALSE(pair.second) << ("Incorrect disable_all value for " + pair.first);
  }
}

TEST(AttrController, TestDisableAll) {
  TryDisableAll();
}

void TryCategoryAttributeEnabled(const AttributesController& controller,
                                 const std::string& category,
                                 bool expected_response) {
  // If category_attribute_enabled does not equal expected response then throw error
  EXPECT_EQ(controller.category_attribute_enabled(category), expected_response);
}

TEST(AttrController, TestNodeAttributeEnabled) {
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

TEST(AttrController, TestAdminAttributeEnabled) {
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

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
