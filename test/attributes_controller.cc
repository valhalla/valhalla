#include "baldr/attributes_controller.h"

#include <gtest/gtest.h>

using namespace std;
using namespace valhalla::baldr;

namespace {

TEST(AttrController, TestCtorDefaultAttributes) {
  AttributesController controller;
  // Test that default attributes work with operator()
  for (const auto& pair : AttributesController::kDefaultAttributes) {
    EXPECT_EQ(controller(pair.first), pair.second);
  }
}

TEST(AttrController, TestArgCtor) {
  AttributesController controller;
  // Test that all default attributes are present
  size_t expected_size = AttributesController::kDefaultAttributes.size();
  size_t actual_size = 0;
  for (const auto& pair : AttributesController::kDefaultAttributes) {
    // If controller can access it, count it
    try {
      controller(pair.first);
      ++actual_size;
    } catch (...) {
      // Skip if not accessible
    }
  }
  EXPECT_EQ(actual_size, expected_size);
}

void TryCategoryAttributeEnabled(const AttributesController& controller,
                                 std::string_view category,
                                 bool expected_response) {
  // If category_attribute_enabled does not equal expected response then throw error
  EXPECT_EQ(controller.category_attribute_enabled(category), expected_response);
}

TEST(AttrController, TestNodeAttributeEnabled) {
  // Test default - node category should be enabled by default
  AttributesController controller1;
  TryCategoryAttributeEnabled(controller1, kNodeCategory, true);
}

TEST(AttrController, TestAdminAttributeEnabled) {
  // Test default - admin category should be enabled by default
  AttributesController controller1;
  TryCategoryAttributeEnabled(controller1, kAdminCategory, true);
}

TEST(AttrController, TestExcludeFilter) {
  // Test excluding specific attributes
  valhalla::Options options;
  options.set_filter_action(valhalla::FilterAction::exclude);
  options.add_filter_attributes(std::string(kNodeType));
  options.add_filter_attributes(std::string(kEdgeNames));

  AttributesController controller(options);

  // Excluded attributes should be disabled
  EXPECT_FALSE(controller(kNodeType));
  EXPECT_FALSE(controller(kEdgeNames));

  // Other attributes should still be enabled (default)
  EXPECT_TRUE(controller(kNodeElapsedTime));
  EXPECT_TRUE(controller(kEdgeLength));

  // Node category should still be enabled (other node attributes are enabled)
  TryCategoryAttributeEnabled(controller, kNodeCategory, true);
}

TEST(AttrController, TestIncludeFilter) {
  // Test including specific attributes (non-strict mode)
  valhalla::Options options;
  options.set_filter_action(valhalla::FilterAction::include);
  options.add_filter_attributes(std::string(kNodeType));
  options.add_filter_attributes(std::string(kEdgeNames));

  AttributesController controller(options, false);

  // Included attributes should be enabled
  EXPECT_TRUE(controller(kNodeType));
  EXPECT_TRUE(controller(kEdgeNames));

  // Default attributes should still be enabled in non-strict mode
  EXPECT_TRUE(controller(kNodeElapsedTime));
  EXPECT_TRUE(controller(kEdgeLength));
}

TEST(AttrController, TestIncludeFilterStrict) {
  // Test including specific attributes (strict mode)
  valhalla::Options options;
  options.set_filter_action(valhalla::FilterAction::include);
  options.add_filter_attributes(std::string(kNodeType));
  options.add_filter_attributes(std::string(kNodeElapsedTime));

  AttributesController controller(options, true);

  // Only included attributes should be enabled
  EXPECT_TRUE(controller(kNodeType));
  EXPECT_TRUE(controller(kNodeElapsedTime));

  // Other attributes should be disabled
  EXPECT_FALSE(controller(kEdgeNames));
  EXPECT_FALSE(controller(kEdgeLength));
  EXPECT_FALSE(controller(kNodeFork));

  // Node category should be enabled (some node attributes are enabled)
  TryCategoryAttributeEnabled(controller, kNodeCategory, true);
}

TEST(AttrController, TestAllNodeAttributesDisabled) {
  // Test that node category is disabled when all node attributes are disabled
  valhalla::Options options;
  options.set_filter_action(valhalla::FilterAction::exclude);

  // Exclude all node attributes
  options.add_filter_attributes(std::string(kNodeIntersectingEdgeBeginHeading));
  options.add_filter_attributes(std::string(kNodeIntersectingEdgeFromEdgeNameConsistency));
  options.add_filter_attributes(std::string(kNodeIntersectingEdgeToEdgeNameConsistency));
  options.add_filter_attributes(std::string(kNodeIntersectingEdgeDriveability));
  options.add_filter_attributes(std::string(kNodeIntersectingEdgeCyclability));
  options.add_filter_attributes(std::string(kNodeIntersectingEdgeWalkability));
  options.add_filter_attributes(std::string(kNodeIntersectingEdgeUse));
  options.add_filter_attributes(std::string(kNodeIntersectingEdgeRoadClass));
  options.add_filter_attributes(std::string(kNodeIntersectingEdgeLaneCount));
  options.add_filter_attributes(std::string(kNodeIntersectingEdgeSignInfo));
  options.add_filter_attributes(std::string(kNodeElapsedTime));
  options.add_filter_attributes(std::string(kNodeAdminIndex));
  options.add_filter_attributes(std::string(kNodeType));
  options.add_filter_attributes(std::string(kNodeTrafficSignal));
  options.add_filter_attributes(std::string(kNodeFork));
  options.add_filter_attributes(std::string(kNodeTransitPlatformInfoType));
  options.add_filter_attributes(std::string(kNodeTransitPlatformInfoOnestopId));
  options.add_filter_attributes(std::string(kNodeTransitPlatformInfoName));
  options.add_filter_attributes(std::string(kNodeTransitPlatformInfoStationOnestopId));
  options.add_filter_attributes(std::string(kNodeTransitPlatformInfoStationName));
  options.add_filter_attributes(std::string(kNodeTransitPlatformInfoArrivalDateTime));
  options.add_filter_attributes(std::string(kNodeTransitPlatformInfoDepartureDateTime));
  options.add_filter_attributes(std::string(kNodeTransitPlatformInfoIsParentStop));
  options.add_filter_attributes(std::string(kNodeTransitPlatformInfoAssumedSchedule));
  options.add_filter_attributes(std::string(kNodeTransitPlatformInfoLatLon));
  options.add_filter_attributes(std::string(kNodeTransitStationInfoOnestopId));
  options.add_filter_attributes(std::string(kNodeTransitStationInfoName));
  options.add_filter_attributes(std::string(kNodeTransitStationInfoLatLon));
  options.add_filter_attributes(std::string(kNodeTransitEgressInfoOnestopId));
  options.add_filter_attributes(std::string(kNodeTransitEgressInfoName));
  options.add_filter_attributes(std::string(kNodeTransitEgressInfoLatLon));
  options.add_filter_attributes(std::string(kNodeTimeZone));
  options.add_filter_attributes(std::string(kNodeTransitionTime));

  AttributesController controller(options);

  // All node attributes should be disabled
  EXPECT_FALSE(controller(kNodeType));
  EXPECT_FALSE(controller(kNodeElapsedTime));
  EXPECT_FALSE(controller(kNodeFork));

  // Node category should be disabled
  TryCategoryAttributeEnabled(controller, kNodeCategory, false);

  // Admin category should still be enabled
  TryCategoryAttributeEnabled(controller, kAdminCategory, true);
}

TEST(AttrController, TestSomeNodeAttributesEnabled) {
  // Test strict include with only some node attributes
  valhalla::Options options;
  options.set_filter_action(valhalla::FilterAction::include);
  options.add_filter_attributes(std::string(kNodeType));
  options.add_filter_attributes(std::string(kNodeIntersectingEdgeBeginHeading));
  options.add_filter_attributes(std::string(kNodeTransitPlatformInfoType));
  options.add_filter_attributes(std::string(kNodeElapsedTime));
  options.add_filter_attributes(std::string(kNodeFork));

  AttributesController controller(options, true);

  // Included node attributes should be enabled
  EXPECT_TRUE(controller(kNodeType));
  EXPECT_TRUE(controller(kNodeIntersectingEdgeBeginHeading));
  EXPECT_TRUE(controller(kNodeTransitPlatformInfoType));
  EXPECT_TRUE(controller(kNodeElapsedTime));
  EXPECT_TRUE(controller(kNodeFork));

  // Other node attributes should be disabled
  EXPECT_FALSE(controller(kNodeAdminIndex));

  // Node category should be enabled (some node attributes are enabled)
  TryCategoryAttributeEnabled(controller, kNodeCategory, true);

  // Admin category should be disabled (no admin attributes included)
  TryCategoryAttributeEnabled(controller, kAdminCategory, false);
}

TEST(AttrController, TestAdminCategoryWithMixedAttributes) {
  // Test admin category with some attributes enabled
  valhalla::Options options;
  options.set_filter_action(valhalla::FilterAction::include);
  options.add_filter_attributes(std::string(kAdminCountryCode));
  options.add_filter_attributes(std::string(kAdminCountryText));
  options.add_filter_attributes(std::string(kAdminStateText));

  AttributesController controller(options, true);

  // Included admin attributes should be enabled
  EXPECT_TRUE(controller(kAdminCountryCode));
  EXPECT_TRUE(controller(kAdminCountryText));
  EXPECT_TRUE(controller(kAdminStateText));

  // Excluded admin attributes should be disabled
  EXPECT_FALSE(controller(kAdminStateCode));

  // Admin category should be enabled
  TryCategoryAttributeEnabled(controller, kAdminCategory, true);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
