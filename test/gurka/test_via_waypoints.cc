#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}};

class IntermediateLocations : public ::testing::TestWithParam<std::tuple<std::string, std::string>> {
protected:
  static gurka::map map;
  static double distance(const std::string& startNode, const std::string& endNode) {
    return map.nodes[startNode].Distance(map.nodes[endNode]);
  }

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 10;
    const std::string ascii_map = R"(A1234B5678C)";
    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}}},
        {"BC", {{"highway", "primary"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {0.00, 0.00});
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_via_waypoints", build_config);
  }
};
gurka::map IntermediateLocations::map = {};
Api api;
rapidjson::Document d;

/*************************************************************/

TEST_P(IntermediateLocations, test_single) {
  std::string date_time_type;
  std::string intermediate_type;
  std::tie(date_time_type, intermediate_type) = GetParam();

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "6", "B"}, "auto",
                                 {
                                     {"/locations/0/type", "break"},
                                     {"/locations/1/type", intermediate_type},
                                     {"/locations/2/type", "break"},
                                     {"/date_time/type", date_time_type},
                                     {"/date_time/value", "2111-11-11T11:11"},
                                 });
  auto d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  // if its a through then we have to keep going past 6 in the same direction so in that case
  // we get the edge name BC 3 times because we have it partially to reach 6, then partially to reach
  // C then we uturn and take the whole thing back to B
  if (intermediate_type == "through") {
    gurka::assert::raw::expect_path(result, {"AB", "BC", "BC", "BC"});
  } else if (intermediate_type == "via") {
    gurka::assert::raw::expect_path(result, {"AB", "BC", "BC"});
  }
  EXPECT_EQ(d["routes"].Size(), 1);
  EXPECT_EQ(d["routes"][0]["legs"].Size(), 1);
  EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"].Size(), 1);
  EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"][0]["waypoint_index"].GetInt(), 1);
  if (intermediate_type == "via" || date_time_type != "2") {
    EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"][0]["geometry_index"].GetInt(), 2);
    EXPECT_NEAR(d["routes"][0]["legs"][0]["via_waypoints"][0]["distance_from_start"].GetDouble(),
                distance("A", "6"), 1.0);
  } // arrive by does a reverse search so it prefers to start traveling left from 6 to B
  else {
    EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"][0]["geometry_index"].GetInt(), 3);
    EXPECT_NEAR(d["routes"][0]["legs"][0]["via_waypoints"][0]["distance_from_start"].GetDouble(),
                distance("A", "C") + distance("C", "6"), 1.0);
  }

  // a uturn right at 6
  if (intermediate_type == "via") {
    EXPECT_NEAR(d["routes"][0]["distance"].GetDouble(), distance("A", "6") + distance("B", "6"), 1.0);
  } // have to continue past 6 and come back
  else {
    EXPECT_NEAR(d["routes"][0]["distance"].GetDouble(), distance("A", "C") + distance("B", "C"), 1.0);
  }
}

TEST_P(IntermediateLocations, test_single_at_node) {
  std::string date_time_type;
  std::string intermediate_type;
  std::tie(date_time_type, intermediate_type) = GetParam();

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C", "B"}, "auto",
                                 {
                                     {"/locations/0/type", "break"},
                                     {"/locations/1/type", intermediate_type},
                                     {"/locations/2/type", "break"},
                                     {"/date_time/type", date_time_type},
                                     {"/date_time/value", "2111-11-11T11:11"},
                                 });
  auto d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  // since its at the node we see the edge name only twice once to hit the node and again to leave
  gurka::assert::raw::expect_path(result, {"AB", "BC", "BC"});
  EXPECT_EQ(d["routes"].Size(), 1);
  EXPECT_EQ(d["routes"][0]["legs"].Size(), 1);
  EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"].Size(), 1);
  EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"][0]["waypoint_index"].GetInt(), 1);
  EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"][0]["geometry_index"].GetInt(), 2);
  EXPECT_NEAR(d["routes"][0]["legs"][0]["via_waypoints"][0]["distance_from_start"].GetDouble(),
              distance("A", "C"), 1.0);
  EXPECT_NEAR(d["routes"][0]["distance"].GetDouble(), distance("A", "C") + distance("A", "B"), 1.0);
}

TEST_P(IntermediateLocations, test_multiple) {
  std::string date_time_type;
  std::string intermediate_type;
  std::tie(date_time_type, intermediate_type) = GetParam();

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "3", "5", "C"}, "auto",
                                 {
                                     {"/locations/0/type", "break"},
                                     {"/locations/1/type", intermediate_type},
                                     {"/locations/2/type", intermediate_type},
                                     {"/locations/3/type", "break"},
                                     {"/date_time/type", date_time_type},
                                     {"/date_time/value", "2111-11-11T11:11"},
                                 });
  auto d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  gurka::assert::raw::expect_path(result, {"AB", "AB", "BC", "BC"});
  EXPECT_EQ(d["routes"].Size(), 1);
  EXPECT_EQ(d["routes"][0]["legs"].Size(), 1);
  EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"].Size(), 2);
  EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"][0]["waypoint_index"].GetInt(), 1);
  EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"][0]["geometry_index"].GetInt(), 1);
  EXPECT_NEAR(d["routes"][0]["legs"][0]["via_waypoints"][0]["distance_from_start"].GetDouble(),
              distance("A", "3"), 1.0);
  EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"][1]["waypoint_index"].GetInt(), 2);
  EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"][1]["geometry_index"].GetInt(), 3);
  EXPECT_NEAR(d["routes"][0]["legs"][0]["via_waypoints"][1]["distance_from_start"].GetDouble(),
              distance("A", "5"), 1.0);
  EXPECT_NEAR(d["routes"][0]["distance"].GetDouble(), distance("A", "C"), 1.0);
}

TEST_P(IntermediateLocations, test_multiple_single_edge) {
  std::string date_time_type;
  std::string intermediate_type;
  std::tie(date_time_type, intermediate_type) = GetParam();

  auto result = gurka::do_action(valhalla::Options::route, map,
                                 {"A", "1", "2", "3", "4", "5", "6", "7", "8", "C"}, "auto",
                                 {
                                     {"/locations/0/type", "break"},
                                     {"/locations/1/type", intermediate_type},
                                     {"/locations/2/type", intermediate_type},
                                     {"/locations/3/type", intermediate_type},
                                     {"/locations/4/type", intermediate_type},
                                     {"/locations/5/type", intermediate_type},
                                     {"/locations/6/type", intermediate_type},
                                     {"/locations/7/type", intermediate_type},
                                     {"/locations/8/type", intermediate_type},
                                     {"/locations/9/type", "break"},
                                     {"/date_time/type", date_time_type},
                                     {"/date_time/value", "2111-11-11T11:11"},
                                 });
  auto d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  gurka::assert::raw::expect_path(result,
                                  {"AB", "AB", "AB", "AB", "AB", "BC", "BC", "BC", "BC", "BC"});

  EXPECT_EQ(d["routes"].Size(), 1);
  EXPECT_EQ(d["routes"][0]["legs"].Size(), 1);
  EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"].Size(), 8);
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"][i]["waypoint_index"].GetInt(), i + 1);
    EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"][i]["geometry_index"].GetInt(),
              i + (i > 3 ? 2 : 1));
    EXPECT_NEAR(d["routes"][0]["legs"][0]["via_waypoints"][i]["distance_from_start"].GetDouble(),
                distance("A", std::to_string(i + 1)), 1.0);
  }
  EXPECT_NEAR(d["routes"][0]["distance"].GetDouble(), distance("A", "C"), 1.0);
}

TEST_P(IntermediateLocations, test_back_to_back_via_uturns) {
  std::string date_time_type;
  std::string intermediate_type;
  std::tie(date_time_type, intermediate_type) = GetParam();

  // we already tested this earlier
  if (intermediate_type == "through")
    return;

  auto result =
      gurka::do_action(valhalla::Options::route, map, {"1", "3", "2", "3", "2", "3"}, "auto",
                       {
                           {"/locations/0/type", "break"},
                           {"/locations/1/type", intermediate_type},
                           {"/locations/2/type", intermediate_type},
                           {"/locations/3/type", intermediate_type},
                           {"/locations/4/type", intermediate_type},
                           {"/locations/5/type", "break"},
                           {"/date_time/type", date_time_type},
                           {"/date_time/value", "2111-11-11T11:11"},
                       });
  auto d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  // since its at the node we see the edge name only twice once to hit the node and again to leave
  gurka::assert::raw::expect_path(result, {"AB", "AB", "AB", "AB", "AB"});
  EXPECT_EQ(d["routes"].Size(), 1);
  EXPECT_EQ(d["routes"][0]["legs"].Size(), 1);
  EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"].Size(), 4);

  auto total_dist = distance("1", "3");
  for (int i = 1; i < 5; ++i) {
    EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"][i - 1]["waypoint_index"].GetInt(), i);
    EXPECT_EQ(d["routes"][0]["legs"][0]["via_waypoints"][i - 1]["geometry_index"].GetInt(), i);
    EXPECT_NEAR(d["routes"][0]["legs"][0]["via_waypoints"][i - 1]["distance_from_start"].GetDouble(),
                total_dist, 1.0);
    total_dist += distance("2", "3");
  }
  EXPECT_NEAR(d["routes"][0]["distance"].GetDouble(), total_dist, 1.0);
}

// 1 is depart_at and 2 is arrive_by and 3 is invariant
INSTANTIATE_TEST_SUITE_P(ViaWaypoints,
                         IntermediateLocations,
                         ::testing::ValuesIn(std::vector<std::tuple<std::string, std::string>>{
                             {"1", "through"},
                             {"2", "through"},
                             {"3", "through"},
                             {"1", "via"},
                             {"2", "via"},
                             {"3", "via"},
                         }));
