#include "baldr/rapidjson_utils.h"
#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class Use : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
                          A
                          |
                          |
                          B
                          | \
                          |  C
                          D
                          |
                          |
                          E
                          | \
                          |  F
                          G
  )";

    const gurka::ways ways = {{"AB", {{"highway", "motorway"}}},
                              {"BC",
                               {{"highway", "motorway_link"},
                                {"service", "rest_area"},
                                {"destination", "Bear Peak Rest Area"}}},
                              {"BD", {{"highway", "motorway"}}},
                              {"DE", {{"highway", "motorway"}}},
                              {"EF",
                               {{"highway", "motorway_link"},
                                {"service", "rest_area"},
                                {"amenity", "yes"},
                                {"destination", "Bear Peak Service Area"}}},
                              {"EG", {{"highway", "motorway"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_use",
                            {{"mjolnir.data_processing.use_rest_area", "true"}});
  }
};
gurka::map Use::map = {};

/*************************************************************/

TEST_F(Use, EdgeUse) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto");

  // rest_area
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(1).edge().use(), TripLeg::Use::TripLeg_Use_kRestAreaUse); // BC

  // service_area
  result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(3).edge().use(), TripLeg::Use::TripLeg_Use_kServiceAreaUse); // EF
}

TEST_F(Use, test_passing_rest_area) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");
  auto d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  // Assert expected number of routes, legs, steps
  ASSERT_EQ(d["routes"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"][0]["steps"].Size(), 2);
  auto steps = d["routes"][0]["legs"][0]["steps"].GetArray();

  // Expect maneuvers
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Expect only the second intersection of the first step to have rest_stop
  // Expect rest_stop to be of type=rest_area
  int step_idx = 0;
  EXPECT_EQ(steps[step_idx]["intersections"].Size(), 2);
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stop"));
  EXPECT_TRUE(steps[step_idx]["intersections"][1].HasMember("rest_stop"));
  EXPECT_STREQ(steps[step_idx]["intersections"][1]["rest_stop"]["type"].GetString(), "rest_area");
  EXPECT_STREQ(steps[step_idx]["intersections"][1]["rest_stop"]["name"].GetString(),
               "Bear Peak Rest Area");

  // Expect no rest stops on last step
  step_idx++;
  EXPECT_EQ(steps[step_idx]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stop"));
}

TEST_F(Use, test_entering_rest_area) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto");
  auto d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  // Assert expected number of routes, legs, steps
  ASSERT_EQ(d["routes"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"][0]["steps"].Size(), 3);
  auto steps = d["routes"][0]["legs"][0]["steps"].GetArray();

  // Expect maneuvers
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Expect no rest stops on first step
  int step_idx = 0;
  EXPECT_EQ(steps[step_idx]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stop"));

  // Expect no rest stops on second step
  step_idx++;
  EXPECT_EQ(steps[step_idx]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stop"));

  // Verify the second maneuver instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, step_idx, "Turn left toward Bear Peak Rest Area.",
      "Turn left toward Bear Peak Rest Area. Then You will arrive at your destination.",
      "Turn left toward Bear Peak Rest Area.",
      "Turn left toward Bear Peak Rest Area. Then You will arrive at your destination.",
      "Continue for 400 meters.");

  // Expect no rest stops on third step
  step_idx++;
  EXPECT_EQ(steps[step_idx]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stop"));
}

TEST_F(Use, test_passing_service_area) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"D", "G"}, "auto");
  auto d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  // Assert expected number of routes, legs, steps
  ASSERT_EQ(d["routes"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"][0]["steps"].Size(), 2);
  auto steps = d["routes"][0]["legs"][0]["steps"].GetArray();

  // Expect maneuvers
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Expect only the second intersection of the first step to have rest_stop
  // Expect rest_stop to be of type=service_area
  int step_idx = 0;
  EXPECT_EQ(steps[step_idx]["intersections"].Size(), 2);
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stop"));
  EXPECT_TRUE(steps[step_idx]["intersections"][1].HasMember("rest_stop"));
  EXPECT_STREQ(steps[step_idx]["intersections"][1]["rest_stop"]["type"].GetString(), "service_area");
  EXPECT_STREQ(steps[step_idx]["intersections"][1]["rest_stop"]["name"].GetString(),
               "Bear Peak Service Area");

  // Expect no rest stops on last step
  step_idx++;
  EXPECT_EQ(steps[step_idx]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stop"));
}

TEST_F(Use, test_entering_service_area) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"D", "F"}, "auto");
  auto d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  // Assert expected number of routes, legs, steps
  ASSERT_EQ(d["routes"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"][0]["steps"].Size(), 3);
  auto steps = d["routes"][0]["legs"][0]["steps"].GetArray();

  // Expect maneuvers
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Expect no rest stops on first step
  int step_idx = 0;
  EXPECT_EQ(steps[step_idx]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stop"));

  // Expect no rest stops on second step
  step_idx++;
  EXPECT_EQ(steps[step_idx]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stop"));

  // Verify the second maneuver instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, step_idx, "Turn left toward Bear Peak Service Area.",
      "Turn left toward Bear Peak Service Area. Then You will arrive at your destination.",
      "Turn left toward Bear Peak Service Area.",
      "Turn left toward Bear Peak Service Area. Then You will arrive at your destination.",
      "Continue for 400 meters.");

  // Expect no rest stops on third step
  step_idx++;
  EXPECT_EQ(steps[step_idx]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stop"));
}

class BicycleUse : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
      A--B--C--D
      |        |
      |        |
      E--F--G--H
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "residential"}, {"cyclestreet", "yes"}}},
        {"BC", {{"highway", "living_street"}, {"cyclestreet", "designated"}}},
        {"CD", {{"highway", "unclassified"}, {"bicycle_road", "yes"}}},
        {"AE", {{"highway", "unclassified"}, {"bicycle_road", "designated"}}},
        {"EFGH", {{"highway", "unclassified"}, {"bicycle_road", "no"}}},
        {"HD", {{"highway", "unclassified"}, {"cyclestreet", "no"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/cyclestreet_use");
  }
};

gurka::map BicycleUse::map = {};

TEST_F(BicycleUse, cycle_street_use) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  std::array<std::pair<std::string, std::string>, 6> cyclestreet_edges = {
      std::make_pair("A", "B"), std::make_pair("B", "A"), std::make_pair("C", "D"),
      std::make_pair("D", "C"), std::make_pair("A", "E"), std::make_pair("E", "A"),
  };

  for (const auto& nodes : cyclestreet_edges) {
    auto edge = gurka::findEdgeByNodes(*reader, map.nodes, nodes.first, nodes.second);
    EXPECT_EQ(std::get<1>(edge)->use(), baldr::Use::kCycleStreet)
        << "expected cycle street, got " << baldr::to_string(std::get<1>(edge)->use());
  }

  std::array<std::pair<std::string, std::string>, 4> non_cyclestreet_edges = {
      std::make_pair("E", "H"),
      std::make_pair("H", "E"),
      std::make_pair("H", "D"),
      std::make_pair("D", "H"),
  };

  for (const auto& nodes : non_cyclestreet_edges) {
    auto edge = gurka::findEdgeByNodes(*reader, map.nodes, nodes.first, nodes.second);
    EXPECT_NE(std::get<1>(edge)->use(), baldr::Use::kCycleStreet)
        << "expected  " << baldr::to_string(std::get<1>(edge)->use()) << " got cycle street";
  }
}
