#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

//#############################################################################
class InstructionsSameNameTurnLeft : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
                E
                |
                |
                |
                |
                B----A
                |\----C
                |
                |
                |
                D
    )";

    const gurka::ways ways =
        {{"AB", {{"highway", "primary"}, {"name", "Dammenweg"}, {"ref", "N57"}, {"oneway", "yes"}}},
         {"BC", {{"highway", "primary"}, {"name", "Dammenweg"}, {"ref", "N57"}, {"oneway", "yes"}}},
         {"BD", {{"highway", "primary"}, {"name", ""}, {"ref", "N57"}}},
         {"BE", {{"highway", "secondary"}, {"name", "Stoofweg"}, {"ref", "N651"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_same_name_turn_left",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsSameNameTurnLeft::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsSameNameTurnLeft, SameNameTurnLeft) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // Verify maneuver types (turn_degree=270)
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the same name turn left instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Turn left to stay on N57.", "Turn left.",
                                                            "Turn left to stay on N57.",
                                                            "Turn left to stay on N57.",
                                                            "Continue for 5 kilometers.");
}

//#############################################################################
class InstructionsSameNameTurnLeft302 : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
                E
                |
                |
                |
                |
                B----A
               / \----C
              /
             /
            D
    )";

    const gurka::ways ways =
        {{"AB", {{"highway", "primary"}, {"name", "Dammenweg"}, {"ref", "N57"}, {"oneway", "yes"}}},
         {"BC", {{"highway", "primary"}, {"name", "Dammenweg"}, {"ref", "N57"}, {"oneway", "yes"}}},
         {"BD", {{"highway", "primary"}, {"name", ""}, {"ref", "N57"}}},
         {"BE", {{"highway", "secondary"}, {"name", "Stoofweg"}, {"ref", "N651"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {},
                            "test/data/gurka_instructions_same_name_turn_left_302",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsSameNameTurnLeft302::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsSameNameTurnLeft302, SameNameTurnLeft) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // Verify maneuver types (turn_degree=302)
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the same name turn left instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Turn left to stay on N57.", "Turn left.",
                                                            "Turn left to stay on N57.",
                                                            "Turn left to stay on N57.",
                                                            "Continue for 5 kilometers.");
}

//#############################################################################
class InstructionsSameNameLeftForward : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
                E
                |
                |
                |
                |
                B----A
             /   \----C
           /
        D
                
    )";

    const gurka::ways ways =
        {{"AB", {{"highway", "primary"}, {"name", "Dammenweg"}, {"ref", "N57"}, {"oneway", "yes"}}},
         {"BC", {{"highway", "primary"}, {"name", "Dammenweg"}, {"ref", "N57"}, {"oneway", "yes"}}},
         {"BD", {{"highway", "primary"}, {"name", ""}, {"ref", "N57"}}},
         {"BE", {{"highway", "secondary"}, {"name", "Stoofweg"}, {"ref", "N651"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map =
        gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_same_name_left_forward",
                          {{"mjolnir.admin",
                            {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsSameNameLeftForward::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsSameNameLeftForward, SameNameNoLeft) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // Verify maneuver types (turn_degree=329)
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});
}

//#############################################################################
class InstructionsSameNameTurnLeftToward : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
                E
                |
                |
                |
                |
                B----A
                |\----C
                |
                |
                |
                D
    )";

    const gurka::ways ways =
        {{"AB", {{"highway", "primary"}, {"name", "Dammenweg"}, {"ref", "N57"}, {"oneway", "yes"}}},
         {"BC", {{"highway", "primary"}, {"name", "Dammenweg"}, {"ref", "N57"}, {"oneway", "yes"}}},
         {"BD",
          {{"highway", "primary"},
           {"name", ""},
           {"ref", "N57"},
           {"destination:forward", "Serooskerke;Zierikzee;Burgh-Haamstede;Middelburg;Neeltje Jans"}}},
         {"BE", {{"highway", "secondary"}, {"name", "Stoofweg"}, {"ref", "N651"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {},
                            "test/data/gurka_instructions_same_name_turn_left_toward",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsSameNameTurnLeftToward::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsSameNameTurnLeftToward, SameNameTurnLeftToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // Verify maneuver types (turn_degree=270)
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the same name turn left instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn left toward Serooskerke/Zierikzee/Burgh-Haamstede/Middelburg.",
      "Turn left toward Serooskerke, Zierikzee.", "Turn left toward Serooskerke.",
      "Turn left toward Serooskerke, Zierikzee.", "Continue for 5 kilometers.");
}
