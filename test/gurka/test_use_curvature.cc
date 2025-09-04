#include "baldr/graphconstants.h"
#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;

class UseCurvatureTest : public ::testing::Test {
protected:
  static gurka::map map;
  static gurka::map map_with_dead_end;

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

    /*
    Why do my tests not work if setup like so?
    const gurka::ways ways = {{"AG", {{"highway", "primary"}}},
                              {"AB", {{"highway", "primary"}}},
                              {"CD", {{"highway", "primary"}}},
                              {"DE", {{"highway", "primary"}}},
                              {"EF", {{"highway", "primary"}}},
                              {"FG", {{"highway", "primary"}}}};
    */

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/usecurvature");

    /*
    Why do these tests not work as expected? (see output attached to tests below)
    */
    const std::string ascii_map_with_dead_end = R"(
      A---------G
      |        /
      B     E-F-H     K
       \   /     \   /
        C-D       I-J
    )";

    const gurka::ways ways_with_dead_end =
        {{"AG", {{"highway", "primary"}, {"name", "Straight Road"}}},
         {"ABCDEFG", {{"highway", "primary"}, {"name", "Twisty Road"}}},
         {"FHIJK", {{"highway", "primary"}, {"name", "Dead End Road"}}}};

    const auto layout_with_dead_end =
        gurka::detail::map_to_coordinates(ascii_map_with_dead_end, gridsize);
    map_with_dead_end = gurka::buildtiles(layout_with_dead_end, ways_with_dead_end, {}, {},
                                          "test/data/usecurvature_with_dead_end");
  }

  inline float getDuration(const valhalla::Api& route) {
    return route.directions().routes(0).legs(0).summary().time();
  }
};

gurka::map UseCurvatureTest::map = {};
gurka::map UseCurvatureTest::map_with_dead_end = {};

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

TEST_F(UseCurvatureTest, CurvyRoadWithValue10) {

  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/motorcycle/use_curvature", "10"}};

  valhalla::Api default_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "motorcycle");

  valhalla::Api curvy_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "motorcycle", options);

  gurka::assert::raw::expect_path(default_route, {"Straight Road"});
  gurka::assert::raw::expect_path(curvy_route, {"Twisty Road"});
}

/*
FAILS WITH
valhalla-tests  | /src/valhalla/test/gurka/gurka.cc:1173: Failure
valhalla-tests  | Expected equality of these values:
valhalla-tests  |   actual_names
valhalla-tests  |     Which is: { "Twisty Road", "Twisty Road" }
valhalla-tests  |   expected_names
valhalla-tests  |     Which is: { "Twisty Road" }
valhalla-tests  | Actual path didn't match expected path.
valhalla-tests  |
valhalla-tests  | [  FAILED  ] UseCurvatureTest.CurvyRoadWithDeadEnd100 (6 ms)
*/
/*
TEST_F(UseCurvatureTest, CurvyRoadOnMapWithDeadEndWithValue1) {

  std::unordered_map<std::string, std::string> options =
      {{"/costing_options/motorcycle/use_curvature", "1"}};

  valhalla::Api default_route =
      gurka::do_action(valhalla::Options::route, map_with_dead_end, {"A", "G"}, "motorcycle");

  valhalla::Api curvy_route =
      gurka::do_action(valhalla::Options::route, map_with_dead_end, {"A", "G"}, "motorcycle",
options);

  gurka::assert::raw::expect_path(default_route, {"Straight Road"});
  gurka::assert::raw::expect_path(curvy_route, {"Twisty Road"});
}
*/

/*
FAILS WITH
valhalla-tests  | /src/valhalla/test/gurka/gurka.cc:1173: Failure
valhalla-tests  | Expected equality of these values:
valhalla-tests  |   actual_names
valhalla-tests  |     Which is: { "Twisty Road", "Twisty Road" }
valhalla-tests  |   expected_names
valhalla-tests  |     Which is: { "Twisty Road", "Dead End Road" }
valhalla-tests  | Actual path didn't match expected path.
valhalla-tests  |
valhalla-tests  | [  FAILED  ] UseCurvatureTest.CurvyRoadWithDeadEnd100 (6 ms)
*/
/*
TEST_F(UseCurvatureTest, CurvyRoadOnMapWithDeadEndWithValue100) {

  std::unordered_map<std::string, std::string> options =
      {{"/costing_options/motorcycle/use_curvature", "1"}};

  valhalla::Api default_route =
      gurka::do_action(valhalla::Options::route, map_with_dead_end, {"A", "G"}, "motorcycle");

  valhalla::Api curvy_route =
      gurka::do_action(valhalla::Options::route, map_with_dead_end, {"A", "G"}, "motorcycle",
options);

  gurka::assert::raw::expect_path(default_route, {"Straight Road"});
  gurka::assert::raw::expect_path(curvy_route, {"Twisty Road", "Dead End Road"});
}
*/