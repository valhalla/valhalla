#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

//#############################################################################
class InstructionsSharpBendTrack : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
          A----------B----------
                                 >C--------F
          E----------D----------
    )";

    const gurka::ways ways =
        {{"AB", {{"highway", "primary"}, {"name", "下田石廊松崎線"}, {"ref", "16"}}},
         {"BC", {{"highway", "primary"}, {"name", "下田石廊松崎線"}, {"ref", "16"}}},
         {"CD", {{"highway", "primary"}, {"name", "下田石廊松崎線"}, {"ref", "16"}}},
         {"DE", {{"highway", "primary"}, {"name", "下田石廊松崎線"}, {"ref", "16"}}},
         {"CF", {{"highway", "track"}, {"name", ""}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_sharp_bend_track",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsSharpBendTrack::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsSharpBendTrack, IgnoreLeft) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 0;

  // Verify single maneuver prior to the right turn
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Drive east on 下田石廊松崎線/16.",
                                                            "Drive east.", "",
                                                            "Drive east on 下田石廊松崎線, 16.",
                                                            "Continue for 3 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsSharpBendTrack, IgnoreRight) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"E", "A"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 0;

  // Verify single maneuver prior to the right turn
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Drive east on 下田石廊松崎線/16.",
                                                            "Drive east.", "",
                                                            "Drive east on 下田石廊松崎線, 16.",
                                                            "Continue for 3 kilometers.");
}

//#############################################################################
class InstructionsSharpBendPath : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
          A----------B----------
                                 >C--------F
          E----------D----------
    )";

    const gurka::ways ways =
        {{"AB", {{"highway", "secondary"}, {"name", "船原西浦高原線"}, {"ref", "127"}}},
         {"BC", {{"highway", "secondary"}, {"name", "船原西浦高原線"}, {"ref", "127"}}},
         {"CD", {{"highway", "secondary"}, {"name", "船原西浦高原線"}, {"ref", "127"}}},
         {"DE", {{"highway", "secondary"}, {"name", "船原西浦高原線"}, {"ref", "127"}}},
         {"CF", {{"highway", "path"}, {"name", ""}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_sharp_bend_path",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsSharpBendPath::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsSharpBendPath, IgnoreLeft) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 0;

  // Verify single maneuver prior to the right turn
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Drive east on 船原西浦高原線/127.",
                                                            "Drive east.", "",
                                                            "Drive east on 船原西浦高原線, 127.",
                                                            "Continue for 3 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsSharpBendPath, IgnoreRight) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"E", "A"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 0;

  // Verify single maneuver prior to the right turn
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Drive east on 船原西浦高原線/127.",
                                                            "Drive east.", "",
                                                            "Drive east on 船原西浦高原線, 127.",
                                                            "Continue for 3 kilometers.");
}
//#############################################################################
class InstructionsSharpBendFootway : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
          A----------B----------
                                 >C--------F
          E----------D----------
    )";

    const gurka::ways ways = {{"AB", {{"highway", "tertiary"}, {"name", "富士見パークウェイ"}}},
                              {"BC", {{"highway", "tertiary"}, {"name", "富士見パークウェイ"}}},
                              {"CD", {{"highway", "tertiary"}, {"name", "富士見パークウェイ"}}},
                              {"DE", {{"highway", "tertiary"}, {"name", "富士見パークウェイ"}}},
                              {"CF", {{"highway", "footway"}, {"name", ""}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_sharp_bend_footway",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsSharpBendFootway::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsSharpBendFootway, IgnoreLeft) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 0;

  // Verify single maneuver prior to the right turn
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Drive east on 富士見パークウェイ.",
                                                            "Drive east.", "",
                                                            "Drive east on 富士見パークウェイ.",
                                                            "Continue for 3 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsSharpBendFootway, IgnoreRight) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"E", "A"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 0;

  // Verify single maneuver prior to the right turn
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Drive east on 富士見パークウェイ.",
                                                            "Drive east.", "",
                                                            "Drive east on 富士見パークウェイ.",
                                                            "Continue for 3 kilometers.");
}
