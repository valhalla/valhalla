#include "gurka.h"

#include <gtest/gtest.h>

#include <algorithm>

using namespace valhalla;

class BicycleSteps : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
      A--B----------------------------------------C
         |                                        |
         |                                        |
         X                                        |
         D----------------------------------------E
         |
         F
    )";

    const gurka::ways ways = {{"AB", {{"highway", "residential"}}},
                              {"BXD", {{"highway", "steps"}}},
                              {"BC", {{"highway", "residential"}}},
                              {"CE", {{"highway", "residential"}}},
                              {"DE", {{"highway", "residential"}}},
                              {"DF", {{"highway", "residential"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/bicycle_steps");
  }
};

gurka::map BicycleSteps::map = {};

TEST_F(BicycleSteps, DefaultAvoidsSteps) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "bicycle");
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CE", "DE", "DF"});
}

TEST_F(BicycleSteps, LowStepsFactorTakesSteps) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "bicycle",
                                 {{"/costing_options/bicycle/steps_factor", "1"}});
  gurka::assert::raw::expect_path(result, {"AB", "BXD", "DF"});
}

TEST_F(BicycleSteps, DefaultDoesNotSnapToSteps) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"X", "F"}, "bicycle");
  const auto paths = gurka::detail::get_paths(result);
  ASSERT_FALSE(paths.empty());
  EXPECT_EQ(std::count(paths[0].begin(), paths[0].end(), "BXD"), 0);
}

TEST_F(BicycleSteps, SnapToSteps) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"X", "F"}, "bicycle",
                                 {{"/costing_options/bicycle/snap_to_steps", "1"}});
  gurka::assert::raw::expect_path(result, {"BXD", "DF"});
}
