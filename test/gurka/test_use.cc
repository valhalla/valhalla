#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{{"mjolnir.shortcuts", "false"}};

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

    const gurka::ways ways =
        {{"AB", {{"highway", "motorway"}}},
         {"BC",
          {{"highway", "motorway_link"}, {"service", "rest_area"}, {"destination", "Rest Area"}}},
         {"BD", {{"highway", "motorway"}}},
         {"DE", {{"highway", "motorway"}}},
         {"EF", {{"highway", "motorway_link"}, {"service", "rest_area"}, {"amenity", "yes"}}},
         {"EG", {{"highway", "motorway"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_use", build_config);
  }
};
gurka::map Use::map = {};

/*************************************************************/

TEST_F(Use, EdgeUse) {
  auto result = gurka::route(map, "A", "C", "auto");

  // rest_area
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(1).edge().use(), TripLeg::Use::TripLeg_Use_kRestAreaUse); // BC

  // service_area
  result = gurka::route(map, "A", "F", "auto");

  leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(3).edge().use(), TripLeg::Use::TripLeg_Use_kServiceAreaUse); // EF
}

TEST_F(Use, test_passing_rest_area) {
  auto result = gurka::route(map, "A", "D", "auto");
  auto d = gurka::convert_to_json(result, valhalla::Options::Format::Options_Format_osrm);

  // Assert expected number of routes, legs, steps
  ASSERT_EQ(d["routes"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"][0]["steps"].Size(), 2);
  auto steps = d["routes"][0]["legs"][0]["steps"].GetArray();

  // Expect maneuvers
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Expect only the second intersection of the first step to have rest_stops
  // Expect rest_stops to be of type=rest_area
  int step_idx = 0;
  EXPECT_EQ(steps[step_idx]["intersections"].Size(), 2);
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stops"));
  EXPECT_TRUE(steps[step_idx]["intersections"][1].HasMember("rest_stops"));
  EXPECT_EQ(steps[step_idx]["intersections"][1]["rest_stops"].Size(), 1);
  EXPECT_STREQ(steps[step_idx]["intersections"][1]["rest_stops"][0]["type"].GetString(), "rest_area");

  // Expect no rest stops on last step
  step_idx++;
  EXPECT_EQ(steps[step_idx]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stops"));
}

TEST_F(Use, test_entering_rest_area) {
  auto result = gurka::route(map, "A", "C", "auto");
  auto d = gurka::convert_to_json(result, valhalla::Options::Format::Options_Format_osrm);

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
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stops"));

  // Expect no rest stops on second step
  step_idx++;
  EXPECT_EQ(steps[step_idx]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stops"));

  // Verify the second maneuver instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, step_idx, "Turn left toward Rest Area.", "Turn left toward Rest Area.",
      "Turn left toward Rest Area. Then You will arrive at your destination.",
      "Continue for 400 meters.");

  // Expect no rest stops on third step
  step_idx++;
  EXPECT_EQ(steps[step_idx]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[step_idx]["intersections"][0].HasMember("rest_stops"));
}

// TEST_F(Use, test_service_area_use) {
//   std::string locations = R"({"lon":)" + std::to_string(map.nodes["B"].lng()) + R"(,"lat":)" +
//                           std::to_string(map.nodes["B"].lat()) + R"(},{"lon":)" +
//                           std::to_string(map.nodes["F"].lng()) + R"(,"lat":)" +
//                           std::to_string(map.nodes["F"].lat()) + "}";

//   auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
//   valhalla::tyr::actor_t actor(map.config, *reader, true);
//   auto json = actor.route(
//       R"({"costing":"auto","format":"osrm","locations":[)" +
//           locations + R"(]})",
//       {}, &api);

//   // get the osrm json
//   d.Parse(json);
//   EXPECT_FALSE(d.HasParseError());

//   for (const auto& route : d["routes"].GetArray()) {
//     for (const auto& leg : route["legs"].GetArray()) {
//       for (const auto& step : leg["steps"].GetArray()) {
//         for (const auto& intersection : step["intersections"].GetArray()) {
//           if (intersection.HasMember("rest_stops")) {
//             EXPECT_EQ(intersection.HasMember("rest_stops"), true);
//             for (const auto& stop : intersection["rest_stops"].GetArray()) {
//               EXPECT_TRUE(stop["type"].IsString());
//               EXPECT_TRUE(stop["type"] == "service_area");
//             }
//           }
//         }
//       }
//     }
//   }
// }

// TEST_F(Use, test_all_use) {
//   std::string locations = R"({"lon":)" + std::to_string(map.nodes["B"].lng()) + R"(,"lat":)" +
//                           std::to_string(map.nodes["B"].lat()) + R"(},{"lon":)" +
//                           std::to_string(map.nodes["F"].lng()) + R"(,"lat":)" +
//                           std::to_string(map.nodes["F"].lat()) + "}";

//   auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
//   valhalla::tyr::actor_t actor(map.config, *reader, true);
//   auto json = actor.route(
//       R"({"costing":"auto","format":"osrm","locations":[)" +
//           locations + R"(]})",
//       {}, &api);

//   // get the osrm json
//   d.Parse(json);
//   EXPECT_FALSE(d.HasParseError());

//   for (const auto& route : d["routes"].GetArray()) {
//     for (const auto& leg : route["legs"].GetArray()) {
//       for (const auto& step : leg["steps"].GetArray()) {
//         for (const auto& intersection : step["intersections"].GetArray()) {
//           if (intersection.HasMember("rest_stops")) {
//             EXPECT_EQ(intersection.HasMember("rest_stops"), true);
//             const auto& stop = intersection["rest_stops"].GetObject();
//             EXPECT_TRUE(stop["type"].IsString());
//             EXPECT_FALSE(stop["type"] == "rest_area");
//             EXPECT_TRUE(stop["type"] == "service_area");
//           }
//         }
//       }
//     }
//   }
// }
