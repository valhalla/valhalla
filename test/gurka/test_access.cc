#include "exceptions.h"
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

TEST(Standalone, AccessPsvWay) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
                            L
                            |
                            |
        A---B---C---D---E---I---J
                |       |       |
                F-------G-------K
                |
                H

        M------N------O
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}},
      {"DE", {{"highway", "primary"}}},
      {"EG", {{"highway", "primary"}}},
      {"FG", {{"highway", "primary"}}},
      {"CF",
       {
           {"highway", "primary"},
           {"access", "psv"}, // access key wins over bus or taxi tag
           {"bike", "no"},
           {"bus", "no"},
       }},
      {"FH", {{"highway", "primary"}}},
      {"EI", {{"highway", "bus_guideway"}}},
      {"JI", {{"highway", "busway"}}},
      {"GK", {{"highway", "primary"}}},
      {"KJ", {{"highway", "primary"}}},
      {"LI", {{"highway", "primary"}}},
      {"MN", {{"highway", "residential"}, {"access", "no"}, {"bus", "permit"}, {"taxi", "permit"}}},
      {"NO", {{"highway", "residential"}, {"access", "no"}, {"bus", "permit"}, {"taxi", "permit"}}},
  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_access_psv_way", build_config);
  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, c);

    if (c == "bus" || c == "taxi")
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CF", "FH"});
    else
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EG", "FG", "FH"});
  }

  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"D", "J"}, c);

    if (c == "bus")
      gurka::assert::raw::expect_path(result, {"DE", "EI", "JI"});
    else
      gurka::assert::raw::expect_path(result, {"DE", "EG", "GK", "KJ"});
  }

  for (auto& c : costing) {
    if (c == "bus")
      EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, map, {"D", "L"}, c));
    else
      EXPECT_THROW(gurka::do_action(valhalla::Options::route, map, {"D", "L"}, c),
                   std::runtime_error);
  }

  // Test bus=permit overriding access=no
  auto result = gurka::do_action(valhalla::Options::route, map, {"M", "O"}, "bus");
  gurka::assert::raw::expect_path(result, {"MN", "NO"});

  // Test taxi=permit overriding access=no
  result = gurka::do_action(valhalla::Options::route, map, {"M", "O"}, "taxi");
  gurka::assert::raw::expect_path(result, {"MN", "NO"});
}

TEST(Standalone, AccessPsvNode) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(

        A---B---C---D---E
                |       |
                F       |
                |       |
                H-------G
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}}, {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}}, {"DE", {{"highway", "primary"}}},
      {"EG", {{"highway", "primary"}}}, {"HG", {{"highway", "primary"}}},
      {"CF", {{"highway", "primary"}}}, {"FH", {{"highway", "primary"}}},

  };

  const gurka::nodes nodes = {{"F",
                               {
                                   {"access", "psv"}, // access tag wins over bus or taxi tag
                                   {"taxi", "no"},
                                   {"bus", "no"},
                               }}};

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_access_psv_way", build_config);
  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, c);

    if (c == "bus" || c == "taxi")
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CF", "FH"});
    else
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EG", "HG"});
  }
}

class Accessibility : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {

    const std::string ascii_map = R"(
    A----B----C
    |    .
    D----E----F
    |    . \
    G----H----I)";

    // BE and EH are highway=path, so no cars
    // EI is a shortcut that's not accessible to bikes
    const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                              {"BC", {{"highway", "primary"}}},
                              {"DEF", {{"highway", "primary"}}},
                              {"GHI", {{"highway", "primary"}}},
                              {"ADG", {{"highway", "motorway"}}},
                              {"BE", {{"highway", "path"}}},
                              {"EI", {{"highway", "path"}, {"bicycle", "no"}}},
                              {"EH", {{"highway", "path"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/accessibility");
  }
};

gurka::map Accessibility::map = {};

/*************************************************************/
TEST_F(Accessibility, Auto1) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "F"}, "auto");
  gurka::assert::osrm::expect_steps(result, {"BC", "ADG", "DEF"});
  gurka::assert::raw::expect_path(result, {"BC", "AB", "ADG", "DEF", "DEF"});
}
TEST_F(Accessibility, Auto2) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "I"}, "auto");
  gurka::assert::osrm::expect_steps(result, {"BC", "ADG", "GHI"});
  gurka::assert::raw::expect_path(result, {"BC", "AB", "ADG", "ADG", "GHI", "GHI"});
}
TEST_F(Accessibility, WalkUsesShortcut1) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "F"}, "pedestrian");
  gurka::assert::osrm::expect_steps(result, {"BC", "BE", "DEF"});
  gurka::assert::raw::expect_path(result, {"BC", "BE", "DEF"});
}
TEST_F(Accessibility, WalkUsesBothShortcuts) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "I"}, "pedestrian");
  gurka::assert::osrm::expect_steps(result, {"BC", "BE", "EI"});
  gurka::assert::raw::expect_path(result, {"BC", "BE", "EI"});
}
TEST_F(Accessibility, BikeUsesShortcut) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "F"}, "bicycle");
  gurka::assert::osrm::expect_steps(result, {"BC", "BE", "DEF"});
  gurka::assert::raw::expect_path(result, {"BC", "BE", "DEF"});
}
TEST_F(Accessibility, BikeAvoidsSecondShortcut) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "I"}, "bicycle");
  gurka::assert::osrm::expect_steps(result, {"BC", "BE", "GHI"});
  gurka::assert::raw::expect_path(result, {"BC", "BE", "EH", "GHI"});
}
TEST_F(Accessibility, WalkAvoidsMotorway) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "pedestrian");
  gurka::assert::osrm::expect_steps(result, {"AB", "BE", "GHI"});
  gurka::assert::raw::expect_path(result, {"AB", "BE", "EH", "GHI"});
}
TEST_F(Accessibility, AutoUsesMotorway) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto");
  gurka::assert::osrm::expect_steps(result, {"ADG"});
  gurka::assert::raw::expect_path(result, {"ADG", "ADG"});
}

class MultipleBarriers : public ::testing::Test {
protected:
  static gurka::nodelayout layout;
  static gurka::ways ways;
  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
       C-----2----D
       |          |
       A-----1----B
    )";

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);

    ways = {
        {"AC2", {{"highway", "primary"}}},
        {"2DB", {{"highway", "primary"}}},
        {"A1", {{"highway", "primary"}}},
        {"1B", {{"highway", "primary"}}},
    };
  }

  void check_auto_path(const gurka::map& map, const std::vector<std::string>& expected_path) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
    gurka::assert::raw::expect_path(result, expected_path);
  }
};

gurka::nodelayout MultipleBarriers::layout = {};
gurka::ways MultipleBarriers::ways = {};

TEST_F(MultipleBarriers, DeniedBarrierAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"access", "no"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_no_access");
  check_auto_path(map, {"AC2", "2DB"});
}

TEST_F(MultipleBarriers, AllowedBarrierAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"access", "yes"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_allowed_access");
  check_auto_path(map, {"A1", "1B"});
}

TEST_F(MultipleBarriers, AllowedVehicleBarrierAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"motor_vehicle", "yes"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_vehicle_allowed");
  check_auto_path(map, {"A1", "1B"});
}

TEST_F(MultipleBarriers, NoInfoBarrierAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_no_access_info");
  check_auto_path(map, {"AC2", "2DB"});
}

TEST_F(MultipleBarriers, TwoBarriers) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}}},
      {"2", {{"barrier", "gate"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_two_gates");
  check_auto_path(map, {"A1", "1B"});
}

TEST_F(MultipleBarriers, ClosedLongRoute) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"motor_vehicle", "no"}}},
      {"2", {{"barrier", "gate"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_closed_long");
  check_auto_path(map, {"AC2", "2DB"});
}

TEST_F(MultipleBarriers, ClosedShortRoute) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}}},
      {"2", {{"barrier", "gate"}, {"motor_vehicle", "no"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_closed_short");
  check_auto_path(map, {"A1", "1B"});
}

TEST_F(MultipleBarriers, BothClosed) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"motor_vehicle", "no"}}},
      {"2", {{"barrier", "gate"}, {"access", "no"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_both_closed");
  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
    gurka::assert::raw::expect_path(result, {}, "Unexpected path found");
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
}

TEST_F(MultipleBarriers, BothPrivate) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"access", "private"}}},
      {"2", {{"barrier", "gate"}, {"access", "private"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_both_private");
  check_auto_path(map, {"A1", "1B"});
}

TEST_F(MultipleBarriers, ShortestPrivate) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"access", "private"}}},
      {"2", {{"barrier", "gate"}, {"access", "yes"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_one_private");
  check_auto_path(map, {"AC2", "2DB"});
}

TEST_F(MultipleBarriers, BollardPrivate) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "bollard"}, {"access", "private"}}},
      {"2", {{"barrier", "bollard"}, {"access", "yes"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_bollard_one_private");
  check_auto_path(map, {"AC2", "2DB"});
}

TEST_F(MultipleBarriers, BollardPrivateMotorVehicle) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "bollard"}, {"motor_vehicle", "private"}}},
      {"2", {{"barrier", "bollard"}, {"access", "yes"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_bollard_private_motor_vehicle");
  check_auto_path(map, {"AC2", "2DB"});
}

TEST_F(MultipleBarriers, BollardPrivateAndNoInfo) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "bollard"}, {"motor_vehicle", "private"}}},
      {"2", {{"barrier", "bollard"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_bollard_private_no_info");
  check_auto_path(map, {"A1", "1B"});
}

TEST_F(MultipleBarriers, BollardBothPrivate) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "bollard"}, {"access", "private"}}},
      {"2", {{"barrier", "bollard"}, {"access", "private"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_bollard_both_private");
  check_auto_path(map, {"A1", "1B"});
}

TEST_F(MultipleBarriers, BollardNoAccessInformation) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "bollard"}, {"access", "no"}}},
      {"2", {{"barrier", "bollard"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_bollard_no_access_info");
  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
    gurka::assert::raw::expect_path(result, {}, "Unexpected path found");
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
}

TEST_F(MultipleBarriers, CycleBarrier) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "cycle_barrier"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_cycle_barrier");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "bicycle");
  gurka::assert::raw::expect_path(result, {"A1", "1B"});
}

TEST_F(MultipleBarriers, BarrierWall) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "fence"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_wall");
  check_auto_path(map, {"AC2", "2DB"});
}

TEST_F(MultipleBarriers, BarrierWallWithAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "debris"}, {"motor_vehicle", "yes"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_wall_with_access");
  check_auto_path(map, {"A1", "1B"});
}

class AccessibleBarriers : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
        B-----1---C
        |          \
        |           \
        M-----2------N
        |             \
        A-----3--------D
        |             /
        F-----4------E
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}}}, {"B1C", {{"highway", "primary"}}},
        {"CD", {{"highway", "primary"}}}, {"AM", {{"highway", "primary"}}},
        {"M2", {{"highway", "primary"}}}, {"2N", {{"highway", "primary"}}},
        {"ND", {{"highway", "primary"}}}, {"A3D", {{"highway", "primary"}}},
        {"AF", {{"highway", "primary"}}}, {"F4", {{"highway", "primary"}}},
        {"4E", {{"highway", "primary"}}}, {"ED", {{"highway", "primary"}}},
    };

    const gurka::nodes nodes = {
        // gate is opened
        {"1", {{"barrier", "gate"}, {"access", "yes"}}},
        // access is not specified, huge penalty is added.
        {"2", {{"barrier", "gate"}}},
        // access is private. Penalty is added because of privateness.
        {"3", {{"barrier", "gate"}, {"access", "private"}}},
        // gate is opened for bicycle only.
        {"4", {{"barrier", "lift_gate"}, {"access", "no"}, {"bicycle", "yes"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/accessible_barriers");
  }
};

gurka::map AccessibleBarriers::map = {};

TEST_F(AccessibleBarriers, Auto) {
  const std::string cost = "auto";
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, cost);
  gurka::assert::raw::expect_path(result, {"AB", "B1C", "B1C", "CD"});
}

TEST_F(AccessibleBarriers, Bicycle) {
  const std::string cost = "bicycle";
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, cost);
  gurka::assert::raw::expect_path(result, {"AF", "F4", "4E", "ED"});
}

TEST_F(AccessibleBarriers, Pedestrian) {
  const std::string cost = "pedestrian";
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, cost);
  gurka::assert::raw::expect_path(result, {"AM", "M2", "2N", "ND"});
}

// Check that MTB tags override SAC scale and allow bicycle access
class MtbAccess : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {

    // A--B
    const std::string ascii_map = R"(A----B----C)";
    const gurka::ways ways = {{"AB",
                               {{"highway", "cycleway"},
                                {"sac_scale", "mountain_hiking"},
                                {"mtb:scale:uphill", "2"},
                                {"foot", "designated"}}},
                              {"BC",
                               {{"highway", "cycleway"},
                                {"sac_scale", "mountain_hiking"},
                                {"mtb:scale:uphill", "2"},
                                {"foot", "designated"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/mtb_access");
  }
};

gurka::map MtbAccess::map = {};

/*************************************************************/

TEST_F(MtbAccess, CheckMtbAccess) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "bicycle");
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}

void validate_path(const valhalla::Api& result, const std::vector<std::string>& expected_names) {
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  [[maybe_unused]] auto leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, expected_names);
}

TEST(Standalone, NodeAccess) {
  const std::string ascii_map = R"(
      A----B----C----D------------E
           |         |            |
           F         G            H
           |         |            |
           I----J----K------------L
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}}, {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}}, {"DE", {{"highway", "residential"}}},
      {"BF", {{"highway", "residential"}}}, {"DG", {{"highway", "residential"}}},
      {"EH", {{"highway", "residential"}}}, {"FI", {{"highway", "residential"}}},
      {"GK", {{"highway", "residential"}}}, {"HL", {{"highway", "residential"}}},
      {"IJ", {{"highway", "residential"}}}, {"JK", {{"highway", "residential"}}},
      {"KL", {{"highway", "residential"}}},
  };

  const gurka::nodes nodes = {{"F", {{"motor_vehicle", "no"}}}, {"G", {{"motorcar", "no"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_node_access");

  for (auto& c : costing) {
    if (c == "auto" || c == "taxi")
      validate_path(gurka::do_action(valhalla::Options::route, map, {"A", "I"}, c),
                    {"AB", "BC", "CD", "DE", "EH", "HL", "KL", "JK", "IJ"});
    else if (c == "bicycle" || c == "pedestrian")
      validate_path(gurka::do_action(valhalla::Options::route, map, {"A", "I"}, c),
                    {"AB", "BF", "FI"});
    else
      validate_path(gurka::do_action(valhalla::Options::route, map, {"A", "I"}, c),
                    {"AB", "BC", "CD", "DG", "GK", "JK", "IJ"});
  }
}

TEST(Standalone, RouteOnPrivateAccess) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
        A---B---C---D
            |   |   |
            E   F   G
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}},
      {"BE", {{"highway", "service"}, {"access", "private"}}},
      {"CF", {{"highway", "service"}, {"access", "private"}, {"service", "driveway"}}},
      {"DG", {{"highway", "service"}, {"access", "private"}, {"service", "parking_aisle"}}},
  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_route_on_private_access",
                               build_config);

  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, c);
    gurka::assert::raw::expect_path(result, {"AB", "BE"});

    result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, c);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CF"});

    result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, c);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG"});
  }
}

TEST(Standalone, AccessForwardBackward) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
          A--B--C-D-E
                |   |
                F   |
                |   |
                H---G
    )";

  const gurka::ways ways = {
      {"ABCDE", {{"highway", "primary"}}},
      {"CFH",
       {{"highway", "primary"},
        {"motor_vehicle:forward", "no"},
        {"vehicle:backward", "yes"},
        {"foot:forward", "no"},
        {"foot:backward", "yes"},
        {"bicycle:forward", "no"}}},
      {"HG", {{"highway", "primary"}}},
      {"EG",
       {{"highway", "primary"},
        {"vehicle:forward", "yes"},
        {"motor_vehicle:backward", "no"},
        {"foot:forward", "yes"},
        {"foot:backward", "no"},
        {"bicycle:backward", "no"}}},
  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_access_motor_vehicle", build_config);

  for (auto& c : costing) {
    // no problem forward for everyone
    auto result = gurka::do_action(valhalla::Options::route, map, {"D", "G"}, c);
    gurka::assert::raw::expect_path(result, {"ABCDE", "EG"});

    // reverse need to go around
    result = gurka::do_action(valhalla::Options::route, map, {"G", "D"}, c);
    gurka::assert::raw::expect_path(result, {"HG", "CFH", "ABCDE"});

    // no problem reverse for everyone
    result = gurka::do_action(valhalla::Options::route, map, {"G", "F"}, c);
    gurka::assert::raw::expect_path(result, {"HG", "CFH"});

    // forward need to go around
    result = gurka::do_action(valhalla::Options::route, map, {"F", "G"}, c);
    gurka::assert::raw::expect_path(result, {"CFH", "ABCDE", "EG"});
  }
}

TEST(Standalone, ViaFerrata) {
  const std::string ascii_map = R"(A----B----C)";
  const gurka::ways ways = {{"AB", {{"highway", "via_ferrata"}, {"sac_scale", "hiking"}}},
                            {"BC", {{"highway", "via_ferrata"}, {"sac_scale", "hiking"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/example");

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "pedestrian");
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}

TEST(Standalone, ViaFerrataDefault) {
  const std::string ascii_map = R"(A----B----C)";
  const gurka::ways ways = {{"AB", {{"highway", "via_ferrata"}}},
                            {"BC", {{"highway", "via_ferrata"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/example");

  try {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "pedestrian");
    gurka::assert::raw::expect_path(result, {}, "Unexpected path found");
  } catch (const valhalla_exception_t& e) {
    EXPECT_STREQ(e.what(), "No suitable edges near location");
  }
}

TEST(Standalone, AccessFerry) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
    A---B---C
        |   |
        |   |
        |   |
        D---E---F
                |
                |
                |
                G---H
    )";

  const gurka::ways ways = {
      {"ABC", {{"highway", "primary"}}},
      {"BD",
       {
           {"route", "ferry"},
           // this combination of tags disables only pedestrian and bicycle access
           {"access", "no"},
           {"motor_vehicle", "yes"},
       }},
      {"CE",
       {
           {"route", "ferry"},
           {"access", "no"},
           {"foot", "yes"},
           {"bicycle", "yes"},
       }},
      {"DEF", {{"highway", "primary"}}},
      {"FG",
       {
           {"route", "ferry"},
           // and this combination disables bus, taxi, truck access
           {"access", "no"},
           {"motorcar", "yes"},
           {"motorcycle", "yes"},
           {"foot", "yes"},
           {"bicycle", "yes"},
       }},
      {"GH", {{"highway", "primary"}}},
  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_access_ferry", build_config);

  for (auto& c : costing) {
    SCOPED_TRACE(c);
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, c);
    // `motor_vehicle` in the first ferry includes everything except bicycle and pedestrian
    if (c == "bicycle" || c == "pedestrian") {
      gurka::assert::raw::expect_path(result, {"ABC", "ABC", "CE", "DEF"}); // second ferry
    } else {
      gurka::assert::raw::expect_path(result, {"ABC", "BD", "DEF", "DEF"}); // first ferry
    }
  }

  // Route should fail for costing that cannot use the third ferry
  for (auto& c : costing) {
    if (c == "auto" || c == "bicycle" || c == "motorcycle" || c == "pedestrian") {
      EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, map, {"D", "H"}, c)) << c;
    } else {
      EXPECT_ANY_THROW(gurka::do_action(valhalla::Options::route, map, {"D", "H"}, c)) << c;
    }
  }
}

TEST(Standalone, DisusedFerry) {
  const std::string ascii_map = R"(
    A---B---C---D
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC",
       {
           {"disused:route", "ferry"},
           {"motor_vehicle", "yes"},
           {"vehicle", "yes"},
           {"foot", "yes"},
       }},
      {"CD", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {5.1079374, 52.0887174});
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_access_disused_ferry", build_config);

  // Route should fail for all costings due to disused ferry
  for (auto& c : costing) {
    EXPECT_ANY_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "D"}, c)) << c;
  }
}

TEST(Standalone, HighwayPedestrian) {
  const std::string ascii_map = R"(
    A---B---C---D
       / \
      /   \
     G     E---F
      \
       \
        J
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "service"}}},
      {"BC", {{"highway", "pedestrian"}}},
      {"CD", {{"highway", "service"}}},

      {"BE", {{"highway", "pedestrian"}, {"vehicle", "yes"}}}, // all allowed
      {"EF", {{"highway", "service"}}},

      // strange way to allow all except bicycles
      {"BG", {{"highway", "pedestrian"}, {"vehicle", "no"}, {"motor_vehicle", "yes"}}},
      {"GJ", {{"highway", "service"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_access_highway_pedestrian",
                               build_config);

  for (auto& c : costing) {
    // All except pedestrian costing should fail due to highway=pedestrian; bicycles get
    // dismounted access by default
    if (c == "pedestrian" || c == "bicycle") {
      EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "D"}, c)) << c;
    } else {
      EXPECT_ANY_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "D"}, c)) << c;
    }

    // highway:pedestrian + vehicle=yes enables all costings
    EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "F"}, c)) << c;

    // highway:pedestrian + vehicle=no + motor_vehicle=yes enabless all except bicycles
    if (c != "bicycle") {
      EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "J"}, c)) << c;
    } else {
      EXPECT_ANY_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "J"}, c)) << c;
    }
  }
}

class CombinedRestrictionTagValues : public ::testing::Test {
protected:
  static gurka::nodelayout layout;
  static gurka::ways ways;
  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
        A---B
     )";

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    ways = {
        {"AB", {{"highway", "primary"}, {"motor_vehicle", "forestry;agricultural"}}},
    };
  }

  void check_auto_path(const gurka::map& map, const std::vector<std::string>& expected_path) {
    try {
      auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
      gurka::assert::raw::expect_path(result, expected_path, "Unexpected path found");
    } catch (const std::runtime_error& e) {
      EXPECT_STREQ(e.what(), "No suitable edges near location");
    }
  }
};

gurka::nodelayout CombinedRestrictionTagValues::layout = {};
gurka::ways CombinedRestrictionTagValues::ways = {};

TEST_F(CombinedRestrictionTagValues, DeniedCombinedValueAccess) {
  const gurka::map map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/combined_restriction_tag_values");
  check_auto_path(map, {});
}

// With data_processing.bicycle_dismount_on_pedestrian_ways, footways and pedestrian ways without
// explicit bicycle tagging get bicycle access at walking pace (dismount) so the bicycle network is
// not severed at crossings and pedestrian zones. Pushing a bike follows pedestrian access rules:
// redundant foot/access tagging keeps the default as long as pedestrian access remains (access=no
// + foot=yes stays walkable, hence pushable).
TEST(Standalone, BicycleDismountDefault) {
  const std::string ascii_map = R"(
      A--B-C--D-E--F-G--H-I--J-K--L-M--N-O--P
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "cycleway"}}},
      {"BC", {{"highway", "footway"}, {"footway", "crossing"}}},
      {"CD", {{"highway", "cycleway"}}},
      {"DE", {{"highway", "footway"}, {"footway", "sidewalk"}}},
      {"EF", {{"highway", "cycleway"}}},
      {"FG", {{"highway", "pedestrian"}}},
      {"GH", {{"highway", "cycleway"}}},
      {"HI", {{"highway", "footway"}, {"foot", "yes"}}},
      {"IJ", {{"highway", "cycleway"}}},
      {"JK", {{"highway", "footway"}, {"access", "no"}, {"foot", "yes"}}},
      {"KL", {{"highway", "cycleway"}}},
      {"LM", {{"highway", "footway"}, {"bicycle", "dismount"}}},
      {"MN", {{"highway", "cycleway"}}},
      {"NO", {{"highway", "footway"}, {"bicycle", "yes"}}},
      {"OP", {{"highway", "cycleway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/bicycle_dismount_default");

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  for (const std::string way : {"BC", "DE", "FG", "HI", "JK", "LM"}) {
    const auto* edge = std::get<1>(gurka::findEdge(reader, map.nodes, way, way.substr(1)));
    ASSERT_NE(edge, nullptr) << way;
    EXPECT_TRUE(edge->forwardaccess() & baldr::kBicycleAccess) << way;
    EXPECT_TRUE(edge->reverseaccess() & baldr::kBicycleAccess) << way;
    EXPECT_TRUE(edge->dismount()) << way;
  }

  // an explicit bicycle=yes means riding, not pushing
  const auto* ridable = std::get<1>(gurka::findEdge(reader, map.nodes, "NO", "O"));
  ASSERT_NE(ridable, nullptr);
  EXPECT_TRUE(ridable->forwardaccess() & baldr::kBicycleAccess);
  EXPECT_FALSE(ridable->dismount());

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "P"}, "bicycle");
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI", "IJ", "JK",
                                           "KL", "LM", "MN", "NO", "OP"});
}

// Explicit tagging that forbids bicycles or pedestrians wins over the dismount default, including
// bicycle values without an access mapping (e.g. discouraged).
TEST(Standalone, BicycleDismountDefaultRespectsProhibitions) {
  const std::string ascii_map = R"(
      A--B-C--D-E--F-G--H-I--J-K--L-M--N
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "cycleway"}}},
      {"BC", {{"highway", "footway"}, {"bicycle", "no"}}},
      {"CD", {{"highway", "cycleway"}}},
      {"DE", {{"highway", "footway"}, {"bicycle", "discouraged"}}},
      {"EF", {{"highway", "cycleway"}}},
      {"FG", {{"highway", "footway"}, {"foot", "no"}}},
      {"GH", {{"highway", "cycleway"}}},
      {"HI", {{"highway", "footway"}, {"access", "no"}}},
      {"IJ", {{"highway", "cycleway"}}},
      {"JK", {{"highway", "footway"}, {"vehicle", "no"}}},
      {"KL", {{"highway", "cycleway"}}},
      {"LM", {{"highway", "footway"}, {"smoothness", "impassable"}}},
      {"MN", {{"highway", "cycleway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/bicycle_dismount_prohibited");

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  for (const std::string way : {"BC", "DE", "FG", "HI", "JK", "LM"}) {
    // ways that lose all access do not become edges at all, which is just as inaccessible
    const auto* edge = std::get<1>(gurka::findEdge(reader, map.nodes, way, way.substr(1)));
    if (edge) {
      EXPECT_FALSE(edge->forwardaccess() & baldr::kBicycleAccess) << way;
      EXPECT_FALSE(edge->reverseaccess() & baldr::kBicycleAccess) << way;
    }
  }

  EXPECT_ANY_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "N"}, "bicycle"));
}

// Pushing a bike is walking, so when include_pedestrian=false excludes pedestrian edges from the
// graph the dismount default does not apply and footways are filtered out as before, while
// explicit riding access keeps them alive.
TEST(Standalone, BicycleDismountDefaultNoPedestrians) {
  const std::string ascii_map = R"(
      A--B-C--D-E--F
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "cycleway"}}}, {"BC", {{"highway", "footway"}, {"footway", "crossing"}}},
      {"CD", {{"highway", "cycleway"}}}, {"DE", {{"highway", "footway"}, {"bicycle", "yes"}}},
      {"EF", {{"highway", "cycleway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/bicycle_dismount_no_pedestrian",
                               {{"mjolnir.include_pedestrian", "false"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  const auto* crossing = std::get<1>(gurka::findEdge(reader, map.nodes, "BC", "C"));
  EXPECT_EQ(crossing, nullptr);

  const auto* ridable = std::get<1>(gurka::findEdge(reader, map.nodes, "DE", "E"));
  ASSERT_NE(ridable, nullptr);
  EXPECT_TRUE(ridable->forwardaccess() & baldr::kBicycleAccess);
}

// dismount_factor controls how expensive pushing the bike is relative to riding: the default
// takes a short pushed link over a long road detour, a high factor rides around instead.
TEST(Standalone, BicycleDismountFactor) {
  const std::string ascii_map = R"(
      B--------------------A
      |                    |
      |                    |
      |                    |
      C--------------------D
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}},
      {"AD", {{"highway", "footway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 20);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/bicycle_dismount_factor");

  auto pushed = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "bicycle");
  gurka::assert::raw::expect_path(pushed, {"AD"});

  auto ridden = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "bicycle",
                                 {{"/costing_options/bicycle/dismount_factor", "25"}});
  gurka::assert::raw::expect_path(ridden, {"AB", "BC", "CD"});
}

// Opting out via the config option restores the previous behavior: untagged footways stay
// inaccessible to bicycles while an explicit bicycle=dismount keeps working.
TEST(Standalone, BicycleDismountDefaultOff) {
  const std::string ascii_map = R"(
      A--B-C--D-E--F
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "cycleway"}}}, {"BC", {{"highway", "footway"}, {"footway", "crossing"}}},
      {"CD", {{"highway", "cycleway"}}}, {"DE", {{"highway", "footway"}, {"bicycle", "dismount"}}},
      {"EF", {{"highway", "cycleway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/bicycle_dismount_default_off",
                        {{"mjolnir.data_processing.bicycle_dismount_on_pedestrian_ways", "false"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  const auto* crossing = std::get<1>(gurka::findEdge(reader, map.nodes, "BC", "C"));
  ASSERT_NE(crossing, nullptr);
  EXPECT_FALSE(crossing->forwardaccess() & baldr::kBicycleAccess);
  EXPECT_FALSE(crossing->reverseaccess() & baldr::kBicycleAccess);
  EXPECT_FALSE(crossing->dismount());

  const auto* dismount = std::get<1>(gurka::findEdge(reader, map.nodes, "DE", "E"));
  ASSERT_NE(dismount, nullptr);
  EXPECT_TRUE(dismount->forwardaccess() & baldr::kBicycleAccess);
  EXPECT_TRUE(dismount->reverseaccess() & baldr::kBicycleAccess);
  EXPECT_TRUE(dismount->dismount());

  EXPECT_ANY_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "bicycle"));
}
