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

TEST(Standalone, AvoidPencilPointUturns) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
               
        A---B\
        C---D/ E---F---G
                   |   |
                   I---H
    )";

  const gurka::ways ways = {
      {"ABE", {{"highway", "primary"}, {"name", "Silverbrook Road"}, {"oneway", "-1"}}},
      {"CDE", {{"highway", "primary"}, {"name", "Silverbrook Road"}, {"oneway", "yes"}}},
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
                                                    DirectionsLeg_Maneuver_Type_kUturnLeft,
                                                    DirectionsLeg_Maneuver_Type_kDestination});

      int maneuver_index = 1;

      // Verify the left uturn instructions
      gurka::assert::raw::expect_instructions_at_maneuver_index(
          result, maneuver_index, "Make a left U-turn to stay on Silverbrook Road.",
          "Make a left U-turn. Then You will arrive at your destination.",
          "Make a left U-turn to stay on Silverbrook Road.",
          "Make a left U-turn to stay on Silverbrook Road. Then You will arrive at your destination.",
          "Continue for 50 meters.");
    } else if (c == "pedestrian") {

      // Verify maneuver types
      gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                    DirectionsLeg_Maneuver_Type_kSharpLeft,
                                                    DirectionsLeg_Maneuver_Type_kDestination});

      int maneuver_index = 1;

      // Verify the sharp left instructions
      gurka::assert::raw::
          expect_instructions_at_maneuver_index(result, maneuver_index,
                                                "Make a sharp left to stay on Silverbrook Road.",
                                                "Make a sharp left.",
                                                "Make a sharp left to stay on Silverbrook Road.",
                                                "Make a sharp left to stay on Silverbrook Road.",
                                                "Continue for 50 meters.");
    } else {

      // Verify maneuver types
      gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                    DirectionsLeg_Maneuver_Type_kRight,
                                                    DirectionsLeg_Maneuver_Type_kLeft,
                                                    DirectionsLeg_Maneuver_Type_kDestination});

      int maneuver_index = 1;

      // Verify the avoiding the pencil point uturn and turn around via 4th instructions
      gurka::assert::raw::expect_instructions_at_maneuver_index(
          result, maneuver_index, "Turn right onto 4th Street.",
          "Turn right. Then Turn left onto Silverbrook Road.", "Turn right onto 4th Street.",
          "Turn right onto 4th Street. Then Turn left onto Silverbrook Road.",
          "Continue for 70 meters.");
    }
  }
}
