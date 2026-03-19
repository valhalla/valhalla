#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

class UseCurvatureTest : public ::testing::Test {
protected:
  static gurka::map map;
  static gurka::map map_speed;
  static gurka::map map_straight_fast;

  static void SetUpTestSuite() {
    constexpr double gridsize = 50;

    const std::string ascii_map = R"(
      A---------G
      |        /
      B     E-F
       \   /
        C-D
    )";

    const gurka::ways ways = {{"AG", {{"highway", "primary"}, {"name", "Straight Road"}}},
                              {"ABCDEFG", {{"highway", "primary"}, {"name", "Twisty Road"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/usecurvature");

    // Map for testing the avoid-curvature direction: the curvy road is faster at neutral
    // (maxspeed=60 vs maxspeed=20), so the router prefers it by default. With use_curvature=0
    // the curvature penalty (factor ~3x for c≈10) is large enough to flip the decision to the
    // slower but straight road.
    const gurka::ways ways_speed =
        {{"AG", {{"highway", "primary"}, {"name", "Straight Road"}, {"maxspeed", "20"}}},
         {"ABCDEFG", {{"highway", "primary"}, {"name", "Curvy Road"}, {"maxspeed", "60"}}}};

    const auto layout_speed = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map_speed = gurka::buildtiles(layout_speed, ways_speed, {}, {}, "test/data/usecurvature_speed");

    // Map for testing the prefer-curvature direction against a speed penalty: the straight road
    // is faster (maxspeed=60 vs maxspeed=50 on the curvy road), so it wins at neutral. The curvy
    // road is ~2x longer, giving a time ratio T_curvy/T_straight ≈ 2.4.
    //
    // With the prefer-curvy formula, curvy edges are discounted via exponential exponent mapping.
    // Break-even when pow(0.338, exp) = 1/2.4 ≈ 0.417 → exp ≈ 0.8. That exponent is reached at
    // use_curvature ≈ 0.54. Below that threshold straight still wins; above it curvy wins.
    const gurka::ways ways_straight_fast =
        {{"AG", {{"highway", "primary"}, {"name", "Straight Road"}, {"maxspeed", "60"}}},
         {"ABCDEFG", {{"highway", "primary"}, {"name", "Curvy Road"}, {"maxspeed", "50"}}}};

    const auto layout_straight_fast = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map_straight_fast = gurka::buildtiles(layout_straight_fast, ways_straight_fast, {}, {},
                                          "test/data/usecurvature_straight_fast");
  }
};

gurka::map UseCurvatureTest::map = {};
gurka::map UseCurvatureTest::map_speed = {};
gurka::map UseCurvatureTest::map_straight_fast = {};

TEST_F(UseCurvatureTest, TestCurvyRoadWithValue0) {

  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/motorcycle/use_curvature", "0"}};

  valhalla::Api default_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "motorcycle");

  valhalla::Api curvy_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "motorcycle", options);

  gurka::assert::raw::expect_path(default_route, {"Straight Road"});
  gurka::assert::raw::expect_path(curvy_route, {"Straight Road"});
}

TEST_F(UseCurvatureTest, CurvyRoadWithValue1) {

  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/motorcycle/use_curvature", "1"}};

  valhalla::Api default_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "motorcycle");

  valhalla::Api curvy_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "motorcycle", options);

  gurka::assert::raw::expect_path(default_route, {"Straight Road"});
  gurka::assert::raw::expect_path(curvy_route, {"Twisty Road"});
}

TEST_F(UseCurvatureTest, CurvyRoadWithValue0Point75) {

  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/motorcycle/use_curvature", "0.75"}};

  valhalla::Api default_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "motorcycle");

  valhalla::Api curvy_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "motorcycle", options);

  gurka::assert::raw::expect_path(default_route, {"Straight Road"});
  gurka::assert::raw::expect_path(curvy_route, {"Twisty Road"});
}

// The curvy road is faster (maxspeed=60 vs maxspeed=20 on the straight road), so the router
// prefers it at the neutral default (use_curvature=0.5, no curvature effect on cost).
TEST_F(UseCurvatureTest, NeutralDefaultPrefersFastCurvyRoad) {
  valhalla::Api result =
      gurka::do_action(valhalla::Options::route, map_speed, {"A", "G"}, "motorcycle");
  gurka::assert::raw::expect_path(result, {"Curvy Road"});
}

// With use_curvature=0 the curvature penalty (factor ~3x for high-curvature edges) is large
// enough to outweigh the speed advantage, flipping the decision to the slower straight road.
TEST_F(UseCurvatureTest, AvoidCurvyPrefersStraightRoad) {
  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/motorcycle/use_curvature", "0"}};
  valhalla::Api result =
      gurka::do_action(valhalla::Options::route, map_speed, {"A", "G"}, "motorcycle", options);
  gurka::assert::raw::expect_path(result, {"Straight Road"});
}

// The straight road is faster (maxspeed=60) and shorter, so it wins at neutral (use_curvature=0.5).
TEST_F(UseCurvatureTest, NeutralDefaultPrefersFastStraightRoad) {
  valhalla::Api result =
      gurka::do_action(valhalla::Options::route, map_straight_fast, {"A", "G"}, "motorcycle");
  gurka::assert::raw::expect_path(result, {"Straight Road"});
}

// At use_curvature=0.52 (below the ~0.54 threshold) the curvature preference is not yet strong
// enough to overcome the time penalty of the slower, longer curvy road — straight still wins.
TEST_F(UseCurvatureTest, PreferCurvyBelowThresholdStillPrefersStrait) {
  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/motorcycle/use_curvature", "0.52"}};
  valhalla::Api result = gurka::do_action(valhalla::Options::route, map_straight_fast, {"A", "G"},
                                          "motorcycle", options);
  gurka::assert::raw::expect_path(result, {"Straight Road"});
}

// At use_curvature=0.75 (well above the ~0.54 threshold) the curvature discount is strong enough
// to outweigh the speed+distance advantage of the straight road, so curvy wins.
TEST_F(UseCurvatureTest, PreferCurvyAboveThresholdOverridesSpeedPenalty) {
  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/motorcycle/use_curvature", "0.75"}};
  valhalla::Api result = gurka::do_action(valhalla::Options::route, map_straight_fast, {"A", "G"},
                                          "motorcycle", options);
  gurka::assert::raw::expect_path(result, {"Curvy Road"});
}
