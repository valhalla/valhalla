#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsPencilPointUturn : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 10;

    const std::string ascii_map = R"(
        A---B\
        C---D/ E
    )";

    const gurka::ways ways =
        {{"ABE", {{"highway", "primary"}, {"name", "Silverbrook Road"}, {"oneway", "-1"}}},
         {"CDE", {{"highway", "primary"}, {"name", "Silverbrook Road"}, {"oneway", "yes"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_pencil_point_uturn",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsPencilPointUturn::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsPencilPointUturn, UturnLeft) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "A"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kUturnLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify left uturn instruction
  // prev2curr_turn_degree=216
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Make a left U-turn to stay on Silverbrook Road.",
      "Make a left U-turn. Then You will arrive at your destination.",
      "Make a left U-turn to stay on Silverbrook Road.",
      "Make a left U-turn to stay on Silverbrook Road. Then You will arrive at your destination.",
      "Continue for 50 meters.");
}
