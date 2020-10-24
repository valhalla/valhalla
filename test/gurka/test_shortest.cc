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
        {"AB",
         {{"highway", "residential"},
          {"maxspeed", "20"},    // slows down car/truck/motorbike/scooter
          {"surface", "dirt"}}}, // slows down bike/scooter
        {"BC", {{"highway", "residential"}, {"maxspeed", "20"}, {"surface", "dirt"}}},
        {"CD", {{"highway", "residential"}, {"maxspeed", "20"}, {"surface", "dirt"}}},
        {"DE", {{"highway", "residential"}, {"maxspeed", "20"}, {"surface", "dirt"}}},
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

  inline float getLength(const valhalla::Api& route) {
    return route.directions().routes(0).legs(0).summary().length();
  }

  void doTests(const std::string& profile,
               const std::vector<std::string>& fastest_path,
               const std::unordered_map<std::string, std::string>& shortest_options,
               const std::unordered_map<std::string, std::string>& fastest_options = {}) {

    valhalla::Api fastest = gurka::route(shortest_map, "A", "E", profile, fastest_options);
    float fastest_l = getLength(fastest);

    valhalla::Api shortest = gurka::route(shortest_map, "A", "E", profile, shortest_options);
    float shortest_l = getLength(shortest);

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
          {{"/costing_options/" + profile + "/shortest", "1"}},
          {{"/costing_options/" + profile + "/sidewalk_factor", "0.1"}}); // speed up "fastest edges"
}

TEST_F(ShortestTest, ScooterShortest) {
  std::string profile = "motor_scooter";
  doTests(profile, {"AB", "BF", "FG", "GD", "DE"},
          {{"/costing_options/" + profile + "/shortest", "1"}});
}

TEST(AutoShorter, deprecation) {
  // if both auto & auto_shorter costing options were provided, auto costing should be overridden
  Api request;
  std::string request_str =
      R"({"locations":[{"lat":52.114622,"lon":5.131816},{"lat":52.048267,"lon":5.074825}],"costing":"auto_shorter",)"
      R"("costing_options":{"auto":{"use_ferry":0.8}, "auto_shorter":{"use_ferry":0.1, "use_tolls": 0.77}}})";
  ParseApi(request_str, Options::route, request);

  ASSERT_EQ(request.options().costing(), valhalla::auto_);
  ASSERT_EQ(request.options().costing_options(valhalla::auto_).shortest(), true);
  ASSERT_EQ(request.options().costing_options(valhalla::auto_).use_ferry(), 0.1f);
  ASSERT_EQ(request.options().costing_options(valhalla::auto_).use_tolls(), 0.77f);
}
