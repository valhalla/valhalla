#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

class ShortestTest : public ::testing::Test {
protected:
  static gurka::map shortest_map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 2000;

    const std::string ascii_map = R"(
          G----H
          |    | 
       A--B----C--D
          |    |
          E----F
    )";

    const gurka::ways ways = {
        // segments expected for shortest routes for all profiles
        {"AB", {{"highway", "primary"}, {"maxspeed", "20"}}},
        {"BC", {{"highway", "primary"}, {"maxspeed", "20"}}},
        {"CD", {{"highway", "primary"}, {"maxspeed", "20"}}},
        // segments expected for fastest routes for motor_vehicles
        {"BE", {{"highway", "primary"}, {"maxspeed", "100"}}},
        {"EF", {{"highway", "primary"}, {"maxspeed", "100"}}},
        {"FC", {{"highway", "primary"}, {"maxspeed", "100"}}},
        // segments expected for fastest routes for ped/bikes
        {"BG", {{"highway", "primary"}, {"motor_vehicle", "no"}}},
        {"GH", {{"highway", "primary"}, {"motor_vehicle", "no"}}},
        {"HC", {{"highway", "primary"}, {"motor_vehicle", "no"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    shortest_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/shortest");
  }

  inline void getSummary(const valhalla::Api& route, double& distance, double& duration) {
    auto summary = route.directions().routes(0).legs(0).summary();
    distance = summary.length();
    duration = summary.time();
  }

  void doTests(const std::string& profile,
               const std::vector<std::string>& fastest_path,
               const std::unordered_map<std::string, std::string>& shortest_options) {
    double fastest_l, fastest_t, shortest_l, shortest_t;

    valhalla::Api fastest = gurka::route(shortest_map, "A", "D", profile);
    getSummary(fastest, fastest_l, fastest_t);

    valhalla::Api shortest = gurka::route(shortest_map, "A", "D", profile, shortest_options);
    getSummary(shortest, shortest_l, shortest_t);

    gurka::assert::raw::expect_path(fastest, fastest_path);
    gurka::assert::raw::expect_path(shortest, {"AB", "BC", "CD"});
    ASSERT_GT(fastest_l, shortest_l);
    ASSERT_LT(fastest_t, shortest_t);
  }
};

gurka::map ShortestTest::shortest_map = {};

TEST_F(ShortestTest, AutoShortest) {
  std::string profile = "auto";
  doTests(profile, {"AB", "BE", "EF", "FC", "CD"},
          {{"/costing_options/" + profile + "/shortest", "1"}});
}

TEST_F(ShortestTest, TruckShortest) {
  std::string profile = "truck";
  doTests(profile, {"AB", "BE", "EF", "FC", "CD"},
          {{"/costing_options/" + profile + "/shortest", "1"}});
}

TEST_F(ShortestTest, MotorbikeShortest) {
  std::string profile = "motorcycle";
  doTests(profile, {"AB", "BE", "EF", "FC", "CD"},
          {{"/costing_options/" + profile + "/shortest", "1"}});
}

TEST_F(ShortestTest, ScooterShortest) {
  std::string profile = "motor_scooter";
  doTests(profile, {"AB", "BE", "EF", "FC", "CD"},
          {{"/costing_options/" + profile + "/shortest", "1"},
           {"/costing_options/" + profile + "/top_speed", "120"},
           {"/costing_options/" + profile + "/use_primary", "1.0"}});
}

TEST_F(ShortestTest, BikeShortest) {
  std::string profile = "bicycle";
  doTests(profile, {"AB", "BG", "GH", "HC", "CD"},
          {{"/costing_options/" + profile + "/shortest", "1"},
           {"/costing_options/" + profile + "/use_roads", "1.0"}});
}