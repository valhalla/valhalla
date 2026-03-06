#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

class IncludePlatformTest : public ::testing::Test {
protected:
  static gurka::nodelayout layout;
  static gurka::ways ways;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
      A---------B--------C
                         |
                         D
    )";

    ways = {
        {"AB", {{"highway", "residential"}}},
        {"BC", {{"highway", "platform"}}},
        {"CD", {{"highway", "residential"}}},
    };

    layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  }
};

gurka::nodelayout IncludePlatformTest::layout = {};
gurka::ways IncludePlatformTest::ways = {};

TEST_F(IncludePlatformTest, CheckPlatformsExcluded) {
  // exclude roads under construction
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/platforms",
                               {{"mjolnir.include_platforms", "false"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  auto CD = gurka::findEdge(reader, layout, "BC", "C");
  EXPECT_EQ(std::get<0>(CD), baldr::GraphId{}) << "platforms should be excluded";
}

TEST_F(IncludePlatformTest, CheckPlatformsIncluded) {
  // include roads under construction
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/platforms",
                               {{"mjolnir.include_platforms", "true"}});

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "pedestrian");
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});
}

TEST(PlatformClassification, CheckOSMPlatformClassification) {
  const std::string ascii_map = R"(
    A---B---C
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "platform"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/platforms_classification",
                               {{"mjolnir.include_platforms", "true"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  baldr::GraphId bc_id, cb_id;
  const baldr::DirectedEdge* bc_edge = nullptr;
  const baldr::DirectedEdge* cb_edge = nullptr;
  std::tie(bc_id, bc_edge, cb_id, cb_edge) = gurka::findEdge(reader, map.nodes, "BC", "C");

  ASSERT_NE(bc_edge, nullptr);
  EXPECT_EQ(bc_edge->use(), baldr::Use::kPlatform);
  // Test that the platform is classified as low-priority road.
  EXPECT_EQ(bc_edge->classification(), baldr::RoadClass::kServiceOther)
      << "Expected platform to be kServiceOther, but got: "
      << static_cast<int>(bc_edge->classification());
}

TEST(PlatformDefaults, CheckAccessMaskAndUse) {
  const std::string ascii_map = R"(
    A---B
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "platform"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/platforms_access",
                               {{"mjolnir.include_platforms", "true"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  baldr::GraphId ab_id, ba_id;
  const baldr::DirectedEdge* ab_edge = nullptr;
  const baldr::DirectedEdge* ba_edge = nullptr;
  std::tie(ab_id, ab_edge, ba_id, ba_edge) = gurka::findEdge(reader, map.nodes, "AB", "B");

  ASSERT_NE(ab_edge, nullptr);
  ASSERT_NE(ba_edge, nullptr);
  EXPECT_EQ(ab_edge->use(), baldr::Use::kPlatform);
  EXPECT_EQ(ba_edge->use(), baldr::Use::kPlatform);

  // check pedestrian is able to access the platform
  EXPECT_TRUE(ab_edge->forwardaccess() & baldr::kPedestrianAccess);
  EXPECT_TRUE(ba_edge->forwardaccess() & baldr::kPedestrianAccess);
  EXPECT_TRUE(ab_edge->reverseaccess() & baldr::kPedestrianAccess);
  EXPECT_TRUE(ba_edge->reverseaccess() & baldr::kPedestrianAccess);
  // check vehicle is not able to access the platform
  EXPECT_EQ(ab_edge->forwardaccess() & baldr::kVehicularAccess, 0);
  EXPECT_EQ(ba_edge->forwardaccess() & baldr::kVehicularAccess, 0);
  EXPECT_EQ(ab_edge->reverseaccess() & baldr::kVehicularAccess, 0);
  EXPECT_EQ(ba_edge->reverseaccess() & baldr::kVehicularAccess, 0);
}

TEST(PlatformDefaults, CheckReasonableMappingRouting) {
  const std::string ascii_map = R"(
    A---B---C---D
  )";

  // residential road -> footway -> platform
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "footway"}}},
      {"CD", {{"highway", "platform"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/platforms_mapping_routing",
                               {{"mjolnir.include_platforms", "true"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  baldr::GraphId cd_id, dc_id;
  const baldr::DirectedEdge* cd_edge = nullptr;
  const baldr::DirectedEdge* dc_edge = nullptr;
  std::tie(cd_id, cd_edge, dc_id, dc_edge) = gurka::findEdge(reader, map.nodes, "CD", "D");

  ASSERT_NE(cd_edge, nullptr);
  EXPECT_EQ(cd_edge->use(), baldr::Use::kPlatform);
  // Test that the platform is classified as low-priority road
  EXPECT_EQ(cd_edge->classification(), baldr::RoadClass::kServiceOther)
      << "Expected platform to be kServiceOther, but got: "
      << static_cast<int>(cd_edge->classification());

  auto pedestrian_result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "pedestrian");
  // Test that the path is correct and pedestrian is walking through the platform
  gurka::assert::raw::expect_path(pedestrian_result, {"AB", "BC", "CD"});
}
