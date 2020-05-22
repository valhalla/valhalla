#include "gurka.h"
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

  auto result = gurka::match(map, {"1", "2", "3", "4", "5"}, "via", "auto");

  gurka::assert::osrm::expect_match(result, {"AB", "BC", "CD"});
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg::Maneuver::kStart,
                                                DirectionsLeg::Maneuver::kContinue,
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

  for (const auto& test_case : std::vector<std::pair<std::string, float>>{{"break", 90},
                                                                          {"via", 90},
                                                                          {"through", 210},
                                                                          {"break_through", 210}}) {

    auto result = gurka::match(map, {"1", "2", "1", "2"}, test_case.first, "auto",
                               R"({"penalize_immediate_uturn":false})");

    // throughs or vias will make a uturn without a destination notification (left hand driving)
    std::vector<DirectionsLeg::Maneuver::Type> expected_maneuvers{
        DirectionsLeg::Maneuver::kStart,
        DirectionsLeg::Maneuver::kUturnRight,
        DirectionsLeg::Maneuver::kUturnRight,
        DirectionsLeg::Maneuver::kDestination,
    };
    // break will have destination at the trace point
    if (test_case.first == "break") {
      expected_maneuvers = {
          DirectionsLeg::Maneuver::kStart, DirectionsLeg::Maneuver::kDestination,
          DirectionsLeg::Maneuver::kStart, DirectionsLeg::Maneuver::kDestination,
          DirectionsLeg::Maneuver::kStart, DirectionsLeg::Maneuver::kDestination,
      };
    } // break through will have destinations at the trace points but will have to make uturns
    else if (test_case.first == "break_through") {
      expected_maneuvers = {
          DirectionsLeg::Maneuver::kStart,       DirectionsLeg::Maneuver::kDestination,
          DirectionsLeg::Maneuver::kStart,       DirectionsLeg::Maneuver::kUturnRight,
          DirectionsLeg::Maneuver::kDestination, DirectionsLeg::Maneuver::kStart,
          DirectionsLeg::Maneuver::kUturnRight,  DirectionsLeg::Maneuver::kDestination,
      };
    }

    gurka::assert::osrm::expect_match(result, {"AB"});
    gurka::assert::raw::expect_maneuvers(result, expected_maneuvers);
    gurka::assert::raw::expect_path_length(result, test_case.second / 1000, 0.001);
  }
}