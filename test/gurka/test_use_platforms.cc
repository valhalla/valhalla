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
