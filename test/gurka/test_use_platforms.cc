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

TEST_F(IncludePlatformTest, CheckIncludedPlatformProperties) {
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/platforms",
                               {{"mjolnir.include_platforms", "true"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  baldr::GraphId bc_id, cb_id;
  const baldr::DirectedEdge* bc_edge = nullptr;
  const baldr::DirectedEdge* cb_edge = nullptr;
  std::tie(bc_id, bc_edge, cb_id, cb_edge) = gurka::findEdge(reader, map.nodes, "BC", "C");

  ASSERT_NE(bc_edge, nullptr);
  ASSERT_NE(cb_edge, nullptr);
  EXPECT_EQ(bc_edge->use(), baldr::Use::kPlatform);
  EXPECT_EQ(cb_edge->use(), baldr::Use::kPlatform);
  // Test that the platform is classified as low-priority road
  EXPECT_EQ(bc_edge->classification(), baldr::RoadClass::kServiceOther);
  EXPECT_EQ(cb_edge->classification(), baldr::RoadClass::kServiceOther);

  // check pedestrian is able to access the platform
  EXPECT_TRUE(bc_edge->forwardaccess() & baldr::kPedestrianAccess);
  EXPECT_TRUE(cb_edge->forwardaccess() & baldr::kPedestrianAccess);
  EXPECT_TRUE(bc_edge->reverseaccess() & baldr::kPedestrianAccess);
  EXPECT_TRUE(cb_edge->reverseaccess() & baldr::kPedestrianAccess);
  // check vehicle is not able to access the platform
  EXPECT_EQ(bc_edge->forwardaccess() & baldr::kVehicularAccess, 0);
  EXPECT_EQ(cb_edge->forwardaccess() & baldr::kVehicularAccess, 0);
  EXPECT_EQ(bc_edge->reverseaccess() & baldr::kVehicularAccess, 0);
  EXPECT_EQ(cb_edge->reverseaccess() & baldr::kVehicularAccess, 0);
}
