#include "gurka.h"
#include "test.h"
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
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
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
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
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
  auto leg = result.trip().routes(0).legs(0);
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
