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
          H-----I
          |     | 
       A--B--C--D--E
          |     |
          F-----G
    )";

    const gurka::ways ways = {
        // segments expected for shortest routes for all profiles
        {"AB", {{"highway", "tertiary"}, {"sac_scale", "hiking"}, {"service", "alley"}, {"maxspeed", "20"}, {"surface", "dirt"}}},
        {"BC", {{"highway", "tertiary"}, {"sac_scale", "hiking"}, {"service", "alley"}, {"maxspeed", "20"}, {"surface", "dirt"}}},
        {"CD", {{"highway", "tertiary"}, {"sac_scale", "hiking"}, {"service", "alley"}, {"maxspeed", "20"}, {"surface", "dirt"}}},
        {"DE", {{"highway", "tertiary"}, {"sac_scale", "hiking"}, {"service", "alley"}, {"maxspeed", "20"}, {"surface", "dirt"}}},
        // segments expected for fastest routes for motor_vehicles
        {"BF", {{"highway", "tertiary"}, {"maxspeed", "100"}}},
        {"FG", {{"highway", "tertiary"}, {"maxspeed", "100"}}},
        {"GD", {{"highway", "tertiary"}, {"maxspeed", "100"}}},
        // segments expected for fastest routes for ped/bikes
        {"BH", {{"highway", "path"}, {"foot", "yes"}, {"sidewalk", "right"}, {"surface", "paved"}}},
        {"HI", {{"highway", "path"}, {"foot", "yes"}, {"sidewalk", "right"}, {"surface", "paved"}}},
        {"ID", {{"highway", "path"}, {"foot", "yes"}, {"sidewalk", "right"}, {"surface", "paved"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    shortest_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/shortest");
  }

  inline float getSummary(const valhalla::Api& route) {
    return route.directions().routes(0).legs(0).summary().length();
  }

  void doTests(const std::string& profile,
               const std::vector<std::string>& fastest_path,
               const std::unordered_map<std::string, std::string>& shortest_options) {

    valhalla::Api fastest = gurka::route(shortest_map, "A", "E", profile);
    float fastest_l = getSummary(fastest);

    valhalla::Api shortest = gurka::route(shortest_map, "A", "E", profile, shortest_options);
    float shortest_l = getSummary(shortest);

    std::cout << "Lenghts: " << fastest_l << ", " << shortest_l << EOF;

    gurka::assert::raw::expect_path(fastest, fastest_path);
    gurka::assert::raw::expect_path(shortest, {"AB", "BC", "CD", "DE"});
    ASSERT_GT(fastest_l, shortest_l);
  }
};

gurka::map ShortestTest::shortest_map = {};

TEST_F(ShortestTest, AutoShortest) {
  std::string profile = "auto";
  doTests(profile, {"AB", "BF", "FG", "GD", "DE"},
          {{"/costing_options/" + profile + "/shortest", "1"}});
}

TEST_F(ShortestTest, TruckShortest) {
  std::string profile = "truck";
  doTests(profile, {"AB", "BF", "FG", "GD", "DE"},
          {{"/costing_options/" + profile + "/shortest", "1"}});
}

TEST_F(ShortestTest, MotorbikeShortest) {
  std::string profile = "motorcycle";
  doTests(profile, {"AB", "BF", "FG", "GD", "DE"},
          {{"/costing_options/" + profile + "/shortest", "1"}});
}

TEST_F(ShortestTest, BikeShortest) {
  std::string profile = "bicycle";
  doTests(profile, {"AB", "BH", "HI", "ID", "DE"},
          {{"/costing_options/" + profile + "/shortest", "1"}});
}

TEST_F(ShortestTest, PedestrianShortest) {
  std::string profile = "pedestrian";
  doTests(profile, {"AB", "BH", "HI", "ID", "DE"},
          {{"/costing_options/" + profile + "/shortest", "1"}});
}

TEST_F(ShortestTest, ScooterShortest) {
  std::string profile = "motor_scooter";
  doTests(profile, {"AB", "BF", "FG", "GD", "DE"},
          {{"/costing_options/" + profile + "/shortest", "1"}});
}
