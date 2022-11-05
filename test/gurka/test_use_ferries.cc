
#include "baldr/graphconstants.h"
#include "gurka.h"
#include "test.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

class useFerryTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
      B------C
      |      |
      |      |
      A      D
      |      |
      |      |
      E      F
        \  /
         G
    )";

    const gurka::ways ways = {{"ABCD",
                               {{"maxspeed", "20"}, {"route", "ferry"}, {"motor_vehicle", "yes"}}},
                              {"AEGFD", {{"highway", "trunk"}, {"maxspeed", "10"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/ferryopt");
  }

  void doTests(const std::string& costing,
               const std::unordered_map<std::string, std::string>& options) {

    valhalla::Api default_route =
        gurka::do_action(valhalla::Options::route, map, {"A", "D"}, costing);

    valhalla::Api capped_route =
        gurka::do_action(valhalla::Options::route, map, {"A", "D"}, costing, options);

    gurka::assert::raw::expect_path(default_route, {"ABCD"});
    gurka::assert::raw::expect_feature(default_route, "ferry", true);

    gurka::assert::raw::expect_path(capped_route, {"AEGFD"});
    gurka::assert::raw::expect_feature(capped_route, "ferry", false);
  }
};

gurka::map useFerryTest::map = {};

TEST_F(useFerryTest, Autocost) {
  doTests("auto", {{"/costing_options/auto/use_ferry", "0"}});
}

TEST_F(useFerryTest, Buscost) {
  doTests("bus", {{"/costing_options/bus/use_ferry", "0"}});
}

TEST_F(useFerryTest, Motorcyclecost) {
  doTests("motorcycle", {{"/costing_options/motorcycle/use_ferry", "0"}});
}

TEST_F(useFerryTest, Motorscootercost) {
  doTests("motor_scooter", {{"/costing_options/motor_scooter/use_ferry", "0"},
                            {"/costing_options/motor_scooter/ferry_cost", "0.5"}});
}

TEST_F(useFerryTest, Bicyclecost) {
  doTests("bicycle", {{"/costing_options/bicycle/use_ferry", "0"},
                      {"/costing_options/motor_scooter/ferry_cost", "0.5"}});
}

TEST_F(useFerryTest, PedestrianCost) {
  doTests("pedestrian", {{"/costing_options/pedestrian/use_ferry", "0"},
                         {"/costing_options/motor_scooter/ferry_cost", "0.5"}});
}

TEST_F(useFerryTest, Truckcost) {
  doTests("truck", {{"/costing_options/truck/use_ferry", "0"}});
}
