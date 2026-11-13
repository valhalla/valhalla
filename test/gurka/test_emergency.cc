#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {

using options_t = std::unordered_map<std::string, std::string>;

const std::string kDateTime = "2026-11-10T08:00";

// Each route is computed with all three searches, as each has its own access checks
const std::vector<options_t> kSearchVariants = {
    {},                                                          // bidirectional A*
    {{"/date_time/type", "1"}, {"/date_time/value", kDateTime}}, // forward A*
    {{"/date_time/type", "2"}, {"/date_time/value", kDateTime}}, // reverse A*
};

options_t with(options_t options, const options_t& more) {
  options.insert(more.begin(), more.end());
  return options;
}

} // namespace

/*************************************************************/
// ways and barriers only open to emergency vehicles

TEST(EmergencyAccess, EmergencyOnlyWayAndBarrier) {
  constexpr double gridsize_metres = 50;

  const std::string ascii_map = R"(
        A---B---C---D
            |   |
            E---F

        G---H---I
        |       |
        J-------K
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "service"}, {"access", "no"}, {"emergency", "yes"}}},
      {"CD", {{"highway", "primary"}}},
      {"BE", {{"highway", "primary"}}},
      {"EF", {{"highway", "primary"}}},
      {"FC", {{"highway", "primary"}}},
      {"GHI", {{"highway", "residential"}}},
      {"GJ", {{"highway", "residential"}}},
      {"JK", {{"highway", "residential"}}},
      {"KI", {{"highway", "residential"}}},
  };
  const gurka::nodes nodes = {
      {"H", {{"barrier", "bollard"}, {"emergency", "yes"}}},
  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/emergency_access");

  auto result = gurka::do_action(Options::route, map, {"A", "D"}, "auto");
  gurka::assert::raw::expect_path(result, {"AB", "BE", "EF", "FC", "CD"});
  result = gurka::do_action(Options::route, map, {"A", "D"}, "emergency");
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});

  result = gurka::do_action(Options::route, map, {"G", "I"}, "auto");
  gurka::assert::raw::expect_path(result, {"GJ", "JK", "KI"});
  result = gurka::do_action(Options::route, map, {"G", "I"}, "emergency");
  gurka::assert::raw::expect_path(result, {"GHI", "GHI"});
}

/*************************************************************/
// speed factors per road class

class EmergencySpeedFactorTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
      A------B------C------D
    )";

    const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"maxspeed", "50"}}},
                              {"BC", {{"highway", "residential"}, {"maxspeed", "30"}}},
                              {"CD", {{"highway", "motorway"}, {"maxspeed", "100"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/emergency_speed_factor");
  }

  static float duration(const std::vector<std::string>& waypoints,
                        const std::string& costing,
                        const options_t& options = {}) {
    auto route = gurka::do_action(Options::route, map, waypoints, costing, options);
    return route.directions().routes(0).legs(0).summary().time();
  }
};

gurka::map EmergencySpeedFactorTest::map = {};

TEST_F(EmergencySpeedFactorTest, DefaultsToAutoSpeeds) {
  EXPECT_FLOAT_EQ(duration({"A", "B"}, "emergency"), duration({"A", "B"}, "auto"));
  EXPECT_FLOAT_EQ(duration({"B", "C"}, "emergency"), duration({"B", "C"}, "auto"));
  EXPECT_FLOAT_EQ(duration({"C", "D"}, "emergency"), duration({"C", "D"}, "auto"));
}

TEST_F(EmergencySpeedFactorTest, FactorPerRoadClass) {
  const options_t options = {{"/costing_options/emergency/rural_speed_factor", "1.5"},
                             {"/costing_options/emergency/minor_road_speed_factor", "1.2"}};

  // 50 -> 75 km/h on the rural primary road, 30 -> 36 km/h on the residential road
  EXPECT_NEAR(duration({"A", "B"}, "emergency", options), duration({"A", "B"}, "auto") / 1.5f, 0.1f);
  EXPECT_NEAR(duration({"B", "C"}, "emergency", options), duration({"B", "C"}, "auto") / 1.2f, 0.1f);
  // the urban factor does not apply on rural roads
  EXPECT_FLOAT_EQ(duration({"A", "B"}, "emergency",
                           {{"/costing_options/emergency/urban_speed_factor", "1.5"}}),
                  duration({"A", "B"}, "auto"));
}

TEST_F(EmergencySpeedFactorTest, CappedByTopSpeed) {
  const options_t options = {{"/costing_options/emergency/motorway_speed_factor", "1.5"},
                             {"/costing_options/emergency/top_speed", "120"}};

  // 100 -> 150 km/h on the motorway, capped at 120 km/h
  EXPECT_NEAR(duration({"C", "D"}, "emergency", options), duration({"C", "D"}, "auto") / 1.2f, 0.1f);
}

TEST_F(EmergencySpeedFactorTest, OutOfRangeFallsBackToDefault) {
  auto route = gurka::do_action(Options::route, map, {"A", "B"}, "emergency",
                                {{"/costing_options/emergency/rural_speed_factor", "5"}});
  EXPECT_FLOAT_EQ(route.directions().routes(0).legs(0).summary().time(),
                  duration({"A", "B"}, "auto"));
  ASSERT_EQ(route.info().warnings_size(), 1);
  EXPECT_THAT(route.info().warnings(0).description(), testing::HasSubstr("rural_speed_factor"));
}

/*************************************************************/
// with ignore_oneways, only short stretches against the direction of travel

class EmergencyWrongWayTest : public ::testing::Test {
protected:
  // CB is a oneway from C to B, so A -> D has to take the detour BE, EF, FD
  static gurka::map build(const std::string& oneway_highway, double gridsize) {
    const std::string ascii_map = R"(
      A--B--C--D
         |     |
         |     |
         |     |
         E-----F
    )";

    const gurka::ways ways = {{"AB", {{"highway", "residential"}}},
                              {"CB", {{"highway", oneway_highway}, {"oneway", "yes"}}},
                              {"CD", {{"highway", "residential"}}},
                              {"BE", {{"highway", "residential"}}},
                              {"EF", {{"highway", "residential"}}},
                              {"FD", {{"highway", "residential"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    return gurka::buildtiles(layout, ways, {}, {},
                             "test/data/emergency_wrong_way_" + oneway_highway + "_" +
                                 std::to_string(static_cast<int>(gridsize)));
  }

  static gurka::map map;

  static void SetUpTestSuite() {
    // CB is 60 m long
    map = build("residential", 20);
  }

  static const std::vector<std::string> kDetour;
  static const std::vector<std::string> kWrongWay;
  static const options_t kIgnoreOneways;
};

gurka::map EmergencyWrongWayTest::map = {};
const std::vector<std::string> EmergencyWrongWayTest::kDetour = {"AB", "BE", "EF", "FD"};
const std::vector<std::string> EmergencyWrongWayTest::kWrongWay = {"AB", "CB", "CD"};
const options_t EmergencyWrongWayTest::kIgnoreOneways = {{"/costing_options/emergency/ignore_oneways",
                                                          "1"},
                                                         {"/costing_options/auto/ignore_oneways",
                                                          "1"}};

TEST_F(EmergencyWrongWayTest, ClosedByDefault) {
  for (const auto& search : kSearchVariants) {
    auto result = gurka::do_action(Options::route, map, {"A", "D"}, "emergency", search);
    gurka::assert::raw::expect_path(result, kDetour);
  }
}

TEST_F(EmergencyWrongWayTest, ShortEdgeWithIgnoreOneways) {
  for (const auto& search : kSearchVariants) {
    auto result =
        gurka::do_action(Options::route, map, {"A", "D"}, "emergency", with(search, kIgnoreOneways));
    gurka::assert::raw::expect_path(result, kWrongWay);
  }
}

TEST_F(EmergencyWrongWayTest, Matrix) {
  // CostMatrix expands from both ends, so both trees have to accept the wrong way
  auto closed = gurka::do_action(Options::sources_to_targets, map, {"A"}, {"D"}, "emergency");
  auto open =
      gurka::do_action(Options::sources_to_targets, map, {"A"}, {"D"}, "emergency", kIgnoreOneways);
  EXPECT_EQ(open.matrix().algorithm(), Matrix::CostMatrix);
  EXPECT_NEAR(closed.matrix().distances(0), 340, 10);
  EXPECT_NEAR(open.matrix().distances(0), 180, 10);
}

TEST_F(EmergencyWrongWayTest, NotOnLongEdges) {
  // CB is 150 m long, auto with ignore_oneways still takes it
  auto long_map = build("residential", 50);
  for (const auto& search : kSearchVariants) {
    auto result = gurka::do_action(Options::route, long_map, {"A", "D"}, "emergency",
                                   with(search, kIgnoreOneways));
    gurka::assert::raw::expect_path(result, kDetour);
  }
  auto result = gurka::do_action(Options::route, long_map, {"A", "D"}, "auto", kIgnoreOneways);
  gurka::assert::raw::expect_path(result, kWrongWay);
}

TEST_F(EmergencyWrongWayTest, NeverOnTrunkOrMotorway) {
  for (const std::string highway : {"trunk", "motorway"}) {
    auto fast_map = build(highway, 20);
    for (const auto& search : kSearchVariants) {
      auto result = gurka::do_action(Options::route, fast_map, {"A", "D"}, "emergency",
                                     with(search, kIgnoreOneways));
      gurka::assert::raw::expect_path(result, kDetour, highway);
    }
    auto result = gurka::do_action(Options::route, fast_map, {"A", "D"}, "auto", kIgnoreOneways);
    gurka::assert::raw::expect_path(result, kWrongWay, highway);
  }
}
