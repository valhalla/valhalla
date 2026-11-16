#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;

const std::vector<std::string>& costing = {"auto",          "taxi",       "bus",       "truck",
                                           "motor_scooter", "motorcycle", "pedestrian"};

void validate_path(const valhalla::Api& result, const std::vector<std::string>& expected_names) {
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  [[maybe_unused]] auto leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, expected_names);
}

class UseTracksTest : public ::testing::Test {
protected:
  static gurka::map use_tracks_map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
    A-1----B--------E-----2-F
           |        |       |
           |        |       |
           |        |       |
           |        |       |
           C--------D       G-----H
                     \
                      I
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "residential"}}}, {"BE", {{"highway", "track"}, {"surface", "paved"}}},
        {"EF", {{"highway", "residential"}}}, {"FG", {{"highway", "track"}}},
        {"BC", {{"highway", "residential"}}}, {"CD", {{"highway", "residential"}}},
        {"DE", {{"highway", "residential"}}}, {"DI", {{"highway", "secondary"}}},
        {"GH", {{"highway", "residential"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    use_tracks_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_use_tracks");

    // set the same historical speeds for all roads
    test::customize_historical_traffic(use_tracks_map.config, [](baldr::DirectedEdge& e) {
      e.set_free_flow_speed(40);
      e.set_constrained_flow_speed(40);
      return std::nullopt;
    });
  }
};

gurka::map UseTracksTest::use_tracks_map = {};

TEST_F(UseTracksTest, test_default_value) {
  for (const auto& c : costing)
    if (c == "auto" || c == "taxi" || c == "bus" || c == "truck")
      // avoid tracks by default; use tracks only if the route starts or ends at 'track' edge
      validate_path(gurka::do_action(valhalla::Options::route, use_tracks_map, {"1", "2"}, c),
                    {"AB", "BC", "CD", "DE", "EF"});
    else
      // track tag shouldn't affect other costings too much
      validate_path(gurka::do_action(valhalla::Options::route, use_tracks_map, {"1", "2"}, c),
                    {"AB", "BE", "EF"});
}

TEST_F(UseTracksTest, test_use_tracks) {
  // use routes with tracks
  for (const auto& c : costing)
    validate_path(gurka::do_action(valhalla::Options::route, use_tracks_map, {"1", "2"}, c,
                                   {{"/costing_options/" + c + "/use_tracks", "1"}}),
                  {"AB", "BE", "EF"});
}

TEST_F(UseTracksTest, test_avoid_tracks) {
  // avoid tracks
  for (const auto& c : costing)
    validate_path(gurka::do_action(valhalla::Options::route, use_tracks_map, {"1", "2"}, c,
                                   {{"/costing_options/" + c + "/use_tracks", "0"}}),
                  {"AB", "BC", "CD", "DE", "EF"});
}

TEST_F(UseTracksTest, test_use_tracks_if_no_other_roads) {
  // use track if it's needed to complete the route
  for (const auto& c : costing) {
    validate_path(gurka::do_action(valhalla::Options::route, use_tracks_map, {"2", "H"}, c,
                                   {{"/costing_options/" + c + "/use_tracks", "0"}}),
                  {"EF", "FG", "GH"});
  }
}

class UseLitTest : public ::testing::Test {
protected:
  static gurka::map speed_map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
      A------B
      |      |
      |      |
      |      |
      |      |
      |      C
      |      |
      E------D
    )";

    const gurka::ways ways = {{"AB", {{"highway", "residential"}}},
                              {"AE", {{"highway", "residential"}, {"lit", "yes"}}},
                              {"BC", {{"highway", "residential"}}},
                              {"CD", {{"highway", "residential"}}},
                              {"ED", {{"highway", "residential"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    speed_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/uselit");
  }

  inline float getDuration(const valhalla::Api& route) {
    return route.directions().routes(0).legs(0).summary().time();
  }
};

gurka::map UseLitTest::speed_map = {};

TEST_F(UseLitTest, LitHighway) {

  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/pedestrian/use_lit", "1"}};

  valhalla::Api default_route =
      gurka::do_action(valhalla::Options::route, speed_map, {"A", "C"}, "pedestrian");

  valhalla::Api lit_route =
      gurka::do_action(valhalla::Options::route, speed_map, {"A", "C"}, "pedestrian", options);

  gurka::assert::raw::expect_path(default_route, {"AB", "BC"});
  gurka::assert::raw::expect_path(lit_route, {"AE", "ED", "CD"});
}

class UseCurvatureTest : public ::testing::Test {
protected:
  static gurka::map map;               // neutral map
  static gurka::map map_speed;         // curvy road has higher maxspeed
  static gurka::map map_straight_fast; // straight road has higher maxspeed

  static void SetUpTestSuite() {
    constexpr double gridsize = 50;

    const std::string ascii_map = R"(
      A---------G
      |        /
      B     E-F
       \   /
        C-D
    )";

    {
      const gurka::ways ways = {{"AG", {{"highway", "primary"}, {"name", "Straight Road"}}},
                                {"ABCDEFG", {{"highway", "primary"}, {"name", "Twisty Road"}}}};

      const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
      map = gurka::buildtiles(layout, ways, {}, {}, "test/data/usecurvature");
    }

    {
      gurka::ways ways = {{"AG",
                           {{"highway", "primary"}, {"name", "Straight Road"}, {"maxspeed", "20"}}},
                          {"ABCDEFG",
                           {{"highway", "primary"}, {"name", "Curvy Road"}, {"maxspeed", "60"}}}};

      const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
      map_speed = gurka::buildtiles(layout, ways, {}, {}, "test/data/usecurvature_speed");
    }

    {
      gurka::ways ways = {{"AG",
                           {{"highway", "primary"}, {"name", "Straight Road"}, {"maxspeed", "60"}}},
                          {"ABCDEFG",
                           {{"highway", "primary"}, {"name", "Curvy Road"}, {"maxspeed", "50"}}}};

      const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
      map_straight_fast =
          gurka::buildtiles(layout, ways, {}, {}, "test/data/usecurvature_straight_fast");
    }
  }
};

gurka::map UseCurvatureTest::map = {};
gurka::map UseCurvatureTest::map_speed = {};
gurka::map UseCurvatureTest::map_straight_fast = {};

TEST_F(UseCurvatureTest, UseCurvature1) {

  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/motorcycle/use_curvature", "1"}};

  valhalla::Api default_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "motorcycle");

  valhalla::Api curvy_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "motorcycle", options);

  gurka::assert::raw::expect_path(default_route, {"Straight Road"});
  gurka::assert::raw::expect_path(curvy_route, {"Twisty Road"});
}

TEST_F(UseCurvatureTest, UseCurvature2) {

  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/motorcycle/use_curvature", "0.75"}};

  valhalla::Api default_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "motorcycle");

  valhalla::Api curvy_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "motorcycle", options);

  gurka::assert::raw::expect_path(default_route, {"Straight Road"});
  gurka::assert::raw::expect_path(curvy_route, {"Twisty Road"});
}

// prefer the faster curvy road at use_curvature=0.5
TEST_F(UseCurvatureTest, NeutralDefaultPrefersFastCurvyRoad) {
  valhalla::Api result =
      gurka::do_action(valhalla::Options::route, map_speed, {"A", "G"}, "motorcycle");
  gurka::assert::raw::expect_path(result, {"Curvy Road"});
}

// prefer the faster straight road at use_curvature=0.5
TEST_F(UseCurvatureTest, NeutralDefaultPrefersFastStraightRoad) {
  valhalla::Api result =
      gurka::do_action(valhalla::Options::route, map_straight_fast, {"A", "G"}, "motorcycle");
  gurka::assert::raw::expect_path(result, {"Straight Road"});
}

// prefer the slower curvy road at use_curvature=0.75
TEST_F(UseCurvatureTest, PreferCurvyAboveThresholdOverridesSpeedPenalty) {
  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/motorcycle/use_curvature", "0.75"}};
  valhalla::Api result = gurka::do_action(valhalla::Options::route, map_straight_fast, {"A", "G"},
                                          "motorcycle", options);
  gurka::assert::raw::expect_path(result, {"Curvy Road"});
}
