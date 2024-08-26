#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

class ShortestTest : public ::testing::Test {
protected:
  static gurka::map shortest_map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
          H-------------I
          |             |
          |             |
          |             |
       A--B-------------D--E
          |             |
          |             |
          |             |
          F-------------G
    )";

    const gurka::ways ways = {
        // segments expected for shortest routes for all costings
        {"ABDE",
         {{"highway", "residential"},
          {"maxspeed", "20"},    // slows down car/truck/motorbike/scooter
          {"surface", "dirt"}}}, // slows down bike/scooter
        // segments expected for fastest routes for motor_vehicles
        {"BFGD", {{"highway", "tertiary"}, {"maxspeed", "100"}}},
        // segments expected for fastest routes for ped/bikes
        {"BHID", {{"highway", "path"}, {"foot", "yes"}, {"sidewalk", "right"}, {"surface", "paved"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    shortest_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/shortest");
  }

  inline float getLength(const valhalla::Api& route) {
    return route.directions().routes(0).legs(0).summary().length();
  }

  void doTests(const std::string& costing,
               const std::vector<std::string>& fastest_path,
               const std::unordered_map<std::string, std::string>& shortest_options,
               const std::unordered_map<std::string, std::string>& fastest_options = {}) {

    valhalla::Api fastest = gurka::do_action(valhalla::Options::route, shortest_map, {"A", "E"},
                                             costing, fastest_options);
    float fastest_l = getLength(fastest);

    valhalla::Api shortest = gurka::do_action(valhalla::Options::route, shortest_map, {"A", "E"},
                                              costing, shortest_options);
    float shortest_l = getLength(shortest);

    gurka::assert::raw::expect_path(fastest, fastest_path);
    gurka::assert::raw::expect_path(shortest, {"ABDE", "ABDE", "ABDE"});
    ASSERT_GT(fastest_l, shortest_l);
  }
};

gurka::map ShortestTest::shortest_map = {};

TEST_F(ShortestTest, AutoShortest) {
  std::string costing = "auto";
  doTests(costing, {"ABDE", "BFGD", "ABDE"}, {{"/costing_options/" + costing + "/shortest", "1"}});
}

TEST_F(ShortestTest, TruckShortest) {
  std::string costing = "truck";
  doTests(costing, {"ABDE", "BFGD", "ABDE"}, {{"/costing_options/" + costing + "/shortest", "1"}});
}

TEST_F(ShortestTest, MotorbikeShortest) {
  std::string costing = "motorcycle";
  doTests(costing, {"ABDE", "BFGD", "ABDE"}, {{"/costing_options/" + costing + "/shortest", "1"}});
}

TEST_F(ShortestTest, BikeShortest) {
  std::string costing = "bicycle";
  doTests(costing, {"ABDE", "BHID", "ABDE"}, {{"/costing_options/" + costing + "/shortest", "1"}});
}

TEST_F(ShortestTest, PedestrianShortest) {
  std::string costing = "pedestrian";
  doTests(costing, {"ABDE", "BHID", "ABDE"}, {{"/costing_options/" + costing + "/shortest", "1"}},
          {{"/costing_options/" + costing + "/sidewalk_factor", "0.1"}}); // speed up "fastest edges"
}

TEST_F(ShortestTest, ScooterShortest) {
  std::string costing = "motor_scooter";
  doTests(costing, {"ABDE", "BFGD", "ABDE"}, {{"/costing_options/" + costing + "/shortest", "1"}});
}

TEST(AutoShorter, deprecation) {
  // if both auto & auto_shorter costing options were provided, auto costing should be overridden
  Api request;
  std::string request_str =
      R"({"locations":[{"lat":52.114622,"lon":5.131816},{"lat":52.048267,"lon":5.074825}],"costing":"auto_shorter",)"
      R"("costing_options":{"auto":{"use_ferry":0.8}, "auto_shorter":{"use_ferry":0.1, "use_tolls": 0.77}}})";
  ParseApi(request_str, Options::route, request);

  ASSERT_EQ(request.options().costing_type(), Costing::auto_);
  const auto& options = request.options().costings().find(Costing::auto_)->second.options();
  ASSERT_EQ(options.shortest(), true);
  ASSERT_EQ(options.use_ferry(), 0.1f);
  ASSERT_EQ(options.use_tolls(), 0.77f);
}

TEST_F(ShortestTest, AutoUseDistance) {
  std::string costing = "auto";
  doTests(costing, {"ABDE", "BFGD", "ABDE"},
          {{"/costing_options/" + costing + "/use_distance", "1"}});
}
