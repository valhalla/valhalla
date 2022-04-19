#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsSmallEndRampForks : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 20;

    const std::string ascii_map = R"(
 
                   J F 
                   | |
                   | |
          A-B     -D-E----K
             \-C<  | |
                  -G |
                   | |
                   H I
    )";

    const gurka::ways ways =
        {{"AB", {{"highway", "motorway"}, {"name", ""}, {"ref", "PA 283"}, {"oneway", "yes"}}},
         {"BC",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"oneway", "yes"},
           {"destination", "Elizabethtown;Hershey"},
           {"destination:ref", "PA 743"}}},
         {"CD",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"oneway", "yes"},
           {"destination", "Elizabethtown"},
           {"destination:ref", "PA 743 South"}}},
         {"DE", {{"highway", "motorway_link"}, {"name", ""}, {"oneway", "yes"}}},
         {"EF",
          {{"highway", "primary"}, {"name", "Hershey Road"}, {"ref", "PA 743"}, {"oneway", "yes"}}},
         {"CG",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"oneway", "yes"},
           {"destination", "Hershey"},
           {"destination:ref", "PA 743 North"}}},
         {"GH",
          {{"highway", "primary"}, {"name", "Hershey Road"}, {"ref", "PA 743"}, {"oneway", "yes"}}},
         {"JD",
          {{"highway", "primary"}, {"name", "Hershey Road"}, {"ref", "PA 743"}, {"oneway", "yes"}}},
         {"DG",
          {{"highway", "primary"}, {"name", "Hershey Road"}, {"ref", "PA 743"}, {"oneway", "yes"}}},
         {"IE",
          {{"highway", "primary"}, {"name", "Hershey Road"}, {"ref", "PA 743"}, {"oneway", "yes"}}},
         {"EK", {{"highway", "primary"}, {"name", "Ridge Road"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_small_end_ramp_forks",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsSmallEndRampForks::map = {};

///////////////////////////////////////////////////////////////////////////////
// Left
TEST_F(InstructionsSmallEndRampForks, LeftTurn) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kExitRight,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 2;

  // Verify the turn left instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn left onto Hershey Road/PA 743.",
      "Turn left. Then You will arrive at your destination.", "Turn left onto Hershey Road.",
      "Turn left onto Hershey Road, PA 743. Then You will arrive at your destination.",
      "Continue for 60 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Left/Straight
TEST_F(InstructionsSmallEndRampForks, KeepLeftContinue) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "K"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kExitRight,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kContinue,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 3;

  // Verify the keep left toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Continue on Ridge Road.", "", "Continue on Ridge Road.",
      "Continue on Ridge Road. Then You will arrive at your destination.", "Continue for 90 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Right
TEST_F(InstructionsSmallEndRampForks, RightTurn) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kExitRight,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 2;

  //  Verify the turn right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn right onto Hershey Road/PA 743.",
      "Turn right. Then You will arrive at your destination.", "Turn right onto Hershey Road.",
      "Turn right onto Hershey Road, PA 743. Then You will arrive at your destination.",
      "Continue for 40 meters.");
}
