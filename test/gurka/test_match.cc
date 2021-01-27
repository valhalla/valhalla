#include "gurka.h"
#include "midgard/encoded.h"
#include "midgard/util.h"
#include <gtest/gtest.h>

using namespace valhalla;

/*************************************************************/
TEST(Standalone, BasicMatch) {

  const std::string ascii_map = R"(
      1
    A---2B-3-4C
              |
              |5
              D
         )";

  const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                            {"BC", {{"highway", "primary"}}},
                            {"CD", {{"highway", "primary"}}}};

  const double gridsize = 10;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/basic_match");

  auto result = gurka::do_action(valhalla::Options::trace_route, map, {"1", "2", "3", "4", "5"},
                                 "auto", {}, {}, nullptr, "via");

  gurka::assert::osrm::expect_match(result, {"AB", "CD"});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg::Maneuver::kStart,
                                                DirectionsLeg::Maneuver::kRight,
                                                DirectionsLeg::Maneuver::kDestination});

  gurka::assert::raw::expect_path_length(result, 0.100, 0.001);
}

TEST(Standalone, UturnMatch) {

  const std::string ascii_map = "A--1--2--B";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/uturn_match");
  std::vector<std::string> expected_names;
  for (const auto& test_case : std::vector<std::pair<std::string, float>>{{"break", 90},
                                                                          {"via", 90},
                                                                          {"through", 210},
                                                                          {"break_through", 210}}) {

    auto result = gurka::do_action(valhalla::Options::trace_route, map, {"1", "2", "1", "2"}, "auto",
                                   {{"/trace_options/penalize_immediate_uturn", "0"}}, {}, nullptr,
                                   test_case.first);

    // throughs or vias will make a uturn without a destination notification (left hand driving)
    std::vector<DirectionsLeg::Maneuver::Type> expected_maneuvers{
        DirectionsLeg::Maneuver::kStart,
        DirectionsLeg::Maneuver::kUturnRight,
        DirectionsLeg::Maneuver::kUturnRight,
        DirectionsLeg::Maneuver::kDestination,
    };
    expected_names = {"AB", "AB", "AB"};

    // break will have destination at the trace point
    if (test_case.first == "break") {
      expected_maneuvers = {
          DirectionsLeg::Maneuver::kStart, DirectionsLeg::Maneuver::kDestination,
          DirectionsLeg::Maneuver::kStart, DirectionsLeg::Maneuver::kDestination,
          DirectionsLeg::Maneuver::kStart, DirectionsLeg::Maneuver::kDestination,
      };
      expected_names = {"AB", "AB", "AB"};
    } // break through will have destinations at the trace points but will have to make uturns
    else if (test_case.first == "break_through") {
      expected_maneuvers = {
          DirectionsLeg::Maneuver::kStart,       DirectionsLeg::Maneuver::kDestination,
          DirectionsLeg::Maneuver::kStart,       DirectionsLeg::Maneuver::kUturnRight,
          DirectionsLeg::Maneuver::kDestination, DirectionsLeg::Maneuver::kStart,
          DirectionsLeg::Maneuver::kUturnRight,  DirectionsLeg::Maneuver::kDestination,
      };
      expected_names = {"AB", "AB", "AB", "AB", "AB"};
    }

    gurka::assert::osrm::expect_match(result, {"AB"});
    gurka::assert::raw::expect_path(result, expected_names);
    gurka::assert::raw::expect_maneuvers(result, expected_maneuvers);
    gurka::assert::raw::expect_path_length(result, test_case.second / 1000, 0.001);
  }
}

TEST(Standalone, UturnTrimmingAsan) {
  const std::string ascii_map = R"(
A--1--2--B
         |
         |
D--3--4--C--5--6--E)";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"DCE", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/uturn_asan");

  auto result =
      gurka::do_action(valhalla::Options::trace_route, map, {"2", "6", "3"}, "auto",
                       {{"/trace_options/penalize_immediate_uturn", "0"}}, {}, nullptr, "via");

  auto shape =
      midgard::decode<std::vector<midgard::PointLL>>(result.trip().routes(0).legs(0).shape());
  // TODO: Remove the duplicate 6 when we fix odin to handle uturn maneuver generation with only one
  // turn around point
  auto expected_shape = decltype(shape){
      map.nodes["2"], map.nodes["B"], map.nodes["C"], map.nodes["6"],
      map.nodes["6"], map.nodes["C"], map.nodes["3"],
  };
  EXPECT_EQ(shape.size(), expected_shape.size());
  for (int i = 0; i < shape.size(); ++i) {
    EXPECT_TRUE(shape[i].ApproximatelyEqual(expected_shape[i]));
  }
}
