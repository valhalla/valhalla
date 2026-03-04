#include "gurka.h"
#include "test.h"

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
  // See if routing works when platforms are excluded
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/platforms",
                               {{"mjolnir.include_platforms", "false"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  auto CD = gurka::findEdge(reader, layout, "BC", "C");
  EXPECT_EQ(std::get<0>(CD), baldr::GraphId{}) << "platforms should be excluded";
}

TEST_F(IncludePlatformTest, CheckPlatformsIncluded) {
  // See if routing works when platforms are included
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

  // Test that the platform is treated as kOther instead of kPrimary
  EXPECT_EQ(bc_edge->classification(), baldr::RoadClass::kServiceOther)
      << "Expected platform to be kServiceOther, but got: "
      << static_cast<int>(bc_edge->classification());
}
