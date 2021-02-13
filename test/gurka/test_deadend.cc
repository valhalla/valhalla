#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, BirectionalDeadend) {
  constexpr double gridsize = 100;
  const std::string ascii_map = R"(
    A-B-C-D-E
        |
        |1
        F···G
        |
        |2
        H
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}}, {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}}, {"DE", {{"highway", "primary"}}},
      {"CF", {{"highway", "service"}}}, {"FH", {{"highway", "service"}}},
      {"FG", {{"highway", "footway"}}},
  };

  const gurka::nodes nodes = {{"E", {{"barrier", "block"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);

  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/deadend");

  // NOTE: Origin and destination edges should not be connected, otherwise it will be considered a
  // trivial route and handled by a* instead of bidirectional.

  // Go into the first edge of the deadend with break_through
  {
    auto result =
        gurka::do_action(valhalla::Options::route, map, {"A", "1", "E"}, "auto",
                         {{"/locations/1/type", "break_through"}, {"/locations/1/radius", "0"}});

    // Ensure bidirectional a* was used
    ASSERT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    ASSERT_EQ(result.trip().routes(0).legs(1).algorithms(0), "bidirectional_a*");

    // Verify path
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CF", "CF", "CF", "CD", "DE"});

    // Verify maneuver types
    gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                  DirectionsLeg_Maneuver_Type_kRight,
                                                  DirectionsLeg_Maneuver_Type_kDestinationLeft,
                                                  DirectionsLeg_Maneuver_Type_kStartLeft,
                                                  DirectionsLeg_Maneuver_Type_kUturnRight,
                                                  DirectionsLeg_Maneuver_Type_kRight,
                                                  DirectionsLeg_Maneuver_Type_kDestination});
  }

  // Go into the last edge of the deadend with break_through
  {
    auto result =
        gurka::do_action(valhalla::Options::route, map, {"A", "2", "E"}, "auto",
                         {{"/locations/1/type", "break_through"}, {"/locations/1/radius", "0"}});

    // Ensure bidirectional a* was used
    ASSERT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    ASSERT_EQ(result.trip().routes(0).legs(1).algorithms(0), "bidirectional_a*");

    // Verify path
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CF", "FH", "FH", "FH", "CF", "CD", "DE"});

    // Verify maneuver types
    gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                  DirectionsLeg_Maneuver_Type_kRight,
                                                  DirectionsLeg_Maneuver_Type_kDestinationLeft,
                                                  DirectionsLeg_Maneuver_Type_kStartLeft,
                                                  DirectionsLeg_Maneuver_Type_kUturnRight,
                                                  DirectionsLeg_Maneuver_Type_kRight,
                                                  DirectionsLeg_Maneuver_Type_kDestination});
  }

  // // Come out of the deadend from 1
  {
    auto result = gurka::do_action(valhalla::Options::route, map, {"1", "A"}, "auto");

    // Ensure bidirectional a* was used
    ASSERT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");

    // Verify path
    gurka::assert::raw::expect_path(result, {"CF", "BC", "AB"});

    // Verify maneuver types
    gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStartRight,
                                                  DirectionsLeg_Maneuver_Type_kLeft,
                                                  DirectionsLeg_Maneuver_Type_kDestination});
  }

  // Come out of the deadend from 2
  {
    auto result = gurka::do_action(valhalla::Options::route, map, {"2", "E"}, "auto");

    // Ensure bidirectional a* was used
    ASSERT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");

    // Verify path
    gurka::assert::raw::expect_path(result, {"FH", "CF", "CD", "DE"});

    // Verify maneuver types
    gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStartRight,
                                                  DirectionsLeg_Maneuver_Type_kRight,
                                                  DirectionsLeg_Maneuver_Type_kDestination});
  }

  // U-turn at a barrier on E
  {
    auto result =
        gurka::do_action(valhalla::Options::route, map, {"H", "D", "H"}, "auto",
                         {{"/locations/1/type", "break_through"}, {"/locations/1/radius", "0"}});

    // Ensure bidirectional a* was used
    ASSERT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    ASSERT_EQ(result.trip().routes(0).legs(1).algorithms(0), "bidirectional_a*");

    // Verify path
    gurka::assert::raw::expect_path(result, {"FH", "CF", "CD", "CD", "DE", "DE", "CD", "CF", "FH"});

    // Verify maneuver types
    gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                  DirectionsLeg_Maneuver_Type_kRight,
                                                  DirectionsLeg_Maneuver_Type_kDestination,
                                                  DirectionsLeg_Maneuver_Type_kStart,
                                                  DirectionsLeg_Maneuver_Type_kUturnRight,
                                                  DirectionsLeg_Maneuver_Type_kLeft,
                                                  DirectionsLeg_Maneuver_Type_kDestination});
  }
}
