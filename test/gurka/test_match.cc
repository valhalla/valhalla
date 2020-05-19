#include "gurka.h"
#include <gtest/gtest.h>

#include "midgard/encoded.h"
#include "midgard/util.h"

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

  auto result = gurka::match(map, {"1", "2", "3", "4", "5"}, false, "auto");

  gurka::assert::osrm::expect_match(result, {"AB", "BC", "CD"});
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kContinue,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // the shape is correct!?
  float len = 0;
  for (const auto& route : result.trip().routes()) {
    for (const auto& leg : route.legs()) {
      auto points = midgard::decode<std::vector<midgard::PointLL>>(leg.shape());
      len += midgard::length(points);
    }
  }
  EXPECT_NEAR(len, 100.f, 1.f);

  gurka::assert::raw::expect_path_length(result, 0.100, 0.001);
}

TEST(Standalone, UturnMatch) {
  midgard::logging::GetLogger();

  const std::string ascii_map = "A--1--2--B";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/uturn_match");

  auto result = gurka::match(map, {"1", "2", "1"}, false, "auto");

  gurka::assert::osrm::expect_match(result, {"AB"});
  gurka::assert::raw::expect_maneuvers(result,
                                       {DirectionsLeg_Maneuver_Type_kStart,
                                        DirectionsLeg_Maneuver_Type_kUturnRight, // left hand driving?
                                        DirectionsLeg_Maneuver_Type_kDestination});

  // the shape is correct!?
  float len = 0;
  for (const auto& route : result.trip().routes()) {
    for (const auto& leg : route.legs()) {
      auto points = midgard::decode<std::vector<midgard::PointLL>>(leg.shape());
      len += midgard::length(points);
    }
  }
  EXPECT_NEAR(len, 60.f, 1.f);

  gurka::assert::raw::expect_path_length(result, 0.060, 0.001);
}