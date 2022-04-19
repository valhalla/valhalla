#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}};

const std::vector<std::string>& costing = {"auto",    "taxi",          "bus",        "truck",
                                           "bicycle", "motor_scooter", "motorcycle", "pedestrian"};

TEST(Standalone, UnnamedPencilPointUturns) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(

        A---B\
        C---D/ E---F---G
                   |   |
                   I---H
    )";

  const gurka::ways ways = {
      {"ABE", {{"highway", "primary"}, {"name", ""}, {"oneway", "-1"}}},
      {"CDE", {{"highway", "primary"}, {"name", ""}, {"oneway", "yes"}}},
      {"EFG", {{"highway", "primary"}, {"name", "Silverbrook Road"}}},
      {"GHIF", {{"highway", "secondary"}, {"name", "4th Street"}}},
  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_avoid_pencil_point_uturns",
                               build_config);

  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"C", "A"}, c);

    if (c == "bicycle") {

      // Verify maneuver types
      gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                    DirectionsLeg_Maneuver_Type_kSharpLeft,
                                                    DirectionsLeg_Maneuver_Type_kDestination});

      int maneuver_index = 1;

      // Verify that it takes the sharp left turn
      gurka::assert::raw::expect_instructions_at_maneuver_index(
          result, maneuver_index, "Make a sharp left.",
          "Make a sharp left. Then You will arrive at your destination.", "Make a sharp left.",
          "Make a sharp left. Then You will arrive at your destination.", "Continue for 50 meters.");
    } else if (c == "pedestrian") {

      // Verify maneuver types
      gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                    DirectionsLeg_Maneuver_Type_kSharpLeft,
                                                    DirectionsLeg_Maneuver_Type_kDestination});

      int maneuver_index = 1;

      // Verify that it takes the sharp left turn
      gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                                "Make a sharp left.",
                                                                "Make a sharp left.",
                                                                "Make a sharp left.",
                                                                "Make a sharp left.",
                                                                "Continue for 50 meters.");
    } else {

      // Verify maneuver types
      gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                    DirectionsLeg_Maneuver_Type_kSharpLeft,
                                                    DirectionsLeg_Maneuver_Type_kDestination});

      int maneuver_index = 1;

      // Verify that it takes the sharp left turn
      gurka::assert::raw::expect_instructions_at_maneuver_index(
          result, maneuver_index, "Make a sharp left.",
          "Make a sharp left. Then You will arrive at your destination.", "Make a sharp left.",
          "Make a sharp left. Then You will arrive at your destination.", "Continue for 50 meters.");
    }
  }
}
