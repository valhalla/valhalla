#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsCollapseMerge : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
           A
           |
           N
          /| \
    E----D-|-----------C     
    F------|-----G-----H     
           |   J
       L---|--<-K-----M
           | I
           |/
           O
           |
           B
         
    )";

    const gurka::ways ways =
        {{"ANOB", {{"highway", "primary"}, {"name", "Main Street"}}},
         {"CDE",
          {{"highway", "motorway"},
           {"ref", "A 76"},
           {"name", "Pennsylvania Turnpike"},
           {"oneway", "yes"}}},
         {"FGH",
          {{"highway", "motorway"},
           {"ref", "A 76"},
           {"name", "Pennsylvania Turnpike"},
           {"oneway", "yes"}}},
         {"LKM", {{"highway", "motorway"}, {"ref", "A 70"}, {"name", ""}, {"oneway", "yes"}}},
         {"ND",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"oneway", "yes"},
           {"destination", "Pittsburgh"},
           {"destination:ref", "A 76 West"}}},
         {"NG",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"oneway", "yes"},
           {"destination", "Philadelphia"},
           {"destination:ref", "A 76 East"}}},
         {"OI",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"oneway", "yes"},
           {"destination", "Philadelphia;Baltimore"},
           {"destination:ref", "A 76 East;A 70 East"}}},
         {"IJ", {{"highway", "motorway_link"}, {"name", ""}, {"oneway", "yes"}}},
         {"JG", {{"highway", "motorway_link"}, {"name", ""}, {"oneway", "yes"}}},
         {"IK",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"oneway", "yes"},
           {"destination", "Baltimore"},
           {"destination:ref", "A 70 East"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_merge",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsCollapseMerge::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsCollapseMerge, RightRampMerge) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRampRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Ramp right with collapsed merge
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn right to take the A 76 West ramp toward Pittsburgh.", "",
      "Turn right to take the A 76 West ramp.",
      "Turn right to take the A 76 West ramp toward Pittsburgh.", "Continue for 5 kilometers.");

  // Collapsed merge info
  //  ++maneuver_index;
  //
  //  // Merge left
  //  gurka::assert::raw::
  //      expect_instructions_at_maneuver_index(result, maneuver_index,
  //                                            "Merge left onto A 76/Pennsylvania Turnpike.",
  //                                            "Merge left.",
  //                                            "Merge left onto A 76.",
  //                                            "Merge left onto A 76, Pennsylvania Turnpike.",
  //                                            "Continue for 3 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsCollapseMerge, LeftRampMerge) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRampLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Ramp left with collapsed merge
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn left to take the A 76 East ramp toward Philadelphia.", "",
      "Turn left to take the A 76 East ramp.",
      "Turn left to take the A 76 East ramp toward Philadelphia.", "Continue for 8 kilometers.");

  // Collapsed merge info
  //  ++maneuver_index;
  //
  //  // Merge right
  //  gurka::assert::raw::
  //      expect_instructions_at_maneuver_index(result, maneuver_index,
  //                                            "Merge right onto A 76/Pennsylvania Turnpike.",
  //                                            "Merge right.",
  //                                            "Merge right onto A 76.",
  //                                            "Merge right onto A 76, Pennsylvania Turnpike.",
  //                                            "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsCollapseMerge, RightRampLeftForkMerge) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"B", "H"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRampRight,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Right ramp
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index,
      "Turn right to take the A 76 East/A 70 East ramp toward Philadelphia/Baltimore.", "",
      "Turn right to take the A 76 East ramp.",
      "Turn right to take the A 76 East, A 70 East ramp toward Philadelphia, Baltimore.",
      "Continue for 2.5 kilometers.");

  ++maneuver_index;

  // Keep left with collapsed merge
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep left to take A 76/Pennsylvania Turnpike.", "",
                                            "Keep left to take A 76.",
                                            "Keep left to take A 76, Pennsylvania Turnpike.",
                                            "Continue for 8 kilometers.");
  // Collapsed merge info
  //  ++maneuver_index;
  //
  //  // Merge left
  //  gurka::assert::raw::
  //      expect_instructions_at_maneuver_index(result, maneuver_index,
  //                                            "Merge left onto A 76/Pennsylvania Turnpike.",
  //                                            "Merge left.",
  //                                            "Merge left onto A 76.",
  //                                            "Merge left onto A 76, Pennsylvania Turnpike.",
  //                                            "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsCollapseMerge, RightRampRightForkMerge) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"B", "M"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRampRight,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Right ramp
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Turn right to take the A 70 East ramp toward Baltimore.",
                                            "", "Turn right to take the A 70 East ramp.",
                                            "Turn right to take the A 70 East ramp toward Baltimore.",
                                            "Continue for 2.5 kilometers.");

  ++maneuver_index;

  // Keep right with collapsed merge
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep right to take A 70 East toward Baltimore.", "",
                                            "Keep right to take A 70 East.",
                                            "Keep right to take A 70 East toward Baltimore.",
                                            "Continue for 6 kilometers.");

  // Collapsed merge info
  //  ++maneuver_index;
  //
  //  // Merge left
  //  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
  //                                                            "Merge left onto A 70.",
  //                                                            "Merge left.",
  //                                                            "Merge left onto A 70.",
  //                                                            "Merge left onto A 70.",
  //                                                            "Continue for 4 kilometers.");
}
