#include "baldr/graphconstants.h"

#include <gtest/gtest.h>

#include <unordered_set>

using namespace valhalla::baldr;

namespace {

// every value in [0, count) has a unique name and the next one is unnamed
template <typename T> void expect_contiguous_names(uint8_t count, const std::string& unknown) {
  std::unordered_set<std::string> names;
  for (uint8_t i = 0; i < count; ++i) {
    auto name = to_string(static_cast<T>(i));
    EXPECT_NE(name, unknown) << "value " << int(i);
    EXPECT_TRUE(names.insert(name).second) << "duplicate name " << name;
  }
  EXPECT_EQ(to_string(static_cast<T>(count)), unknown);
}

TEST(GraphConstants, RoadClassStrings) {
  expect_contiguous_names<RoadClass>(static_cast<uint8_t>(RoadClass::kInvalid), "null");
  EXPECT_EQ(to_string(RoadClass::kServiceOther), "service_other");

  EXPECT_EQ(stringToRoadClass("Motorway"), RoadClass::kMotorway);
  EXPECT_EQ(stringToRoadClass("Unclassified"), RoadClass::kUnclassified);
  EXPECT_EQ(stringToRoadClass("ServiceOther"), RoadClass::kServiceOther);
}

TEST(GraphConstants, NodeTypeStrings) {
  expect_contiguous_names<NodeType>(static_cast<uint8_t>(NodeType::kElevator) + 1, "null");
  EXPECT_EQ(to_string(NodeType::kStreetIntersection), "street_intersection");
  EXPECT_EQ(to_string(NodeType::kBorderControl), "border_control");
}

TEST(GraphConstants, IntersectionTypeStrings) {
  expect_contiguous_names<IntersectionType>(static_cast<uint8_t>(IntersectionType::kFork) + 1,
                                            "null");
  EXPECT_EQ(to_string(IntersectionType::kDeadEnd), "dead-end");
}

TEST(GraphConstants, UseStrings) {
  std::unordered_set<std::string> names;
  for (uint8_t i = 0; i < static_cast<uint8_t>(Use::kSize); ++i) {
    auto name = to_string(static_cast<Use>(i));
    if (name != "null") {
      EXPECT_TRUE(names.insert(name).second) << "duplicate name " << name;
    }
  }
  EXPECT_EQ(names.size(), 35);
  EXPECT_EQ(to_string(Use::kRoad), "road");
  EXPECT_EQ(to_string(Use::kRailFerry), "rail-ferry");
  EXPECT_EQ(to_string(Use::kTransitConnection), "transit_connection");
  EXPECT_EQ(to_string(Use::kSize), "null");
}

TEST(GraphConstants, LanguageRoundTrip) {
  for (uint8_t i = static_cast<uint8_t>(Language::kAb); i <= static_cast<uint8_t>(Language::kMs);
       ++i) {
    auto lang = static_cast<Language>(i);
    auto name = to_string(lang);
    EXPECT_NE(name, "none") << "value " << int(i);
    EXPECT_EQ(stringLanguage(name), lang) << name;
  }
  EXPECT_EQ(to_string(Language::kNone), "none");
  EXPECT_EQ(stringLanguage("none"), Language::kNone);
  EXPECT_EQ(to_string(static_cast<Language>(0)), "none");
  EXPECT_EQ(stringLanguage("xx"), Language::kNone);
}

TEST(GraphConstants, EdgeAttributeStrings) {
  expect_contiguous_names<SpeedType>(static_cast<uint8_t>(SpeedType::kClassified) + 1, "null");
  expect_contiguous_names<CycleLane>(static_cast<uint8_t>(CycleLane::kSeparated) + 1, "null");
  expect_contiguous_names<SacScale>(static_cast<uint8_t>(SacScale::kDifficultAlpineHiking) + 1,
                                    "null");
  expect_contiguous_names<Surface>(static_cast<uint8_t>(Surface::kImpassable) + 1, "null");
  expect_contiguous_names<HOVEdgeType>(static_cast<uint8_t>(HOVEdgeType::kHOV3) + 1, "null");

  EXPECT_EQ(to_string(SacScale::kMountainHiking), "mountain hiking");
  EXPECT_EQ(to_string(Surface::kPavedSmooth), "paved_smooth");
  EXPECT_EQ(to_string(HOVEdgeType::kHOV2), "HOV-2");
}

} // namespace
