#include "baldr/graphconstants.h"
#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

class TopSpeedTest : public ::testing::Test {
protected:
  static gurka::map speed_map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
      B------C
      |      |
      |      |
      A      D
      |      |
      E------F
    )";

    const gurka::ways ways = {{"ABCD", {{"highway", "residential"}, {"maxspeed", "45"}}},
                              {"AEFD", {{"highway", "residential"}, {"maxspeed", "20"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    speed_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/topspeed");
  }

  inline float getDuration(const valhalla::Api& route) {
    return route.directions().routes(0).legs(0).summary().time();
  }

  void doTests(const std::string& costing,
               const std::unordered_map<std::string, std::string>& options) {

    valhalla::Api default_route =
        gurka::do_action(valhalla::Options::route, speed_map, {"A", "D"}, costing);
    float default_time = getDuration(default_route);

    valhalla::Api capped_route =
        gurka::do_action(valhalla::Options::route, speed_map, {"A", "D"}, costing, options);
    float capped_time = getDuration(capped_route);

    gurka::assert::raw::expect_path(default_route, {"ABCD"});
    gurka::assert::raw::expect_path(capped_route, {"AEFD"});
    gurka::assert::raw::expect_eta(capped_route, 19.8f, 0.001f);
    ASSERT_GT(capped_time, default_time);
  }
};

gurka::map TopSpeedTest::speed_map = {};

TEST_F(TopSpeedTest, AutoTopSpeed) {
  doTests("auto", {{"/costing_options/auto/top_speed", "20"}});
}

TEST_F(TopSpeedTest, TruckTopSpeed) {
  doTests("truck", {{"/costing_options/truck/top_speed", "20"}});
}

TEST_F(TopSpeedTest, ScooterTopSpeed) {
  doTests("motor_scooter", {{"/costing_options/motor_scooter/top_speed", "20"}});
}

TEST_F(TopSpeedTest, MotorcycleTopSpeed) {
  doTests("motorcycle", {{"/costing_options/motorcycle/top_speed", "20"}});
}

TEST_F(TopSpeedTest, BusTopSpeed) {
  doTests("bus", {{"/costing_options/bus/top_speed", "20"}});
}

TEST_F(TopSpeedTest, TaxiTopSpeed) {
  doTests("taxi", {{"/costing_options/taxi/top_speed", "20"}});
}

TEST_F(TopSpeedTest, ClampMaxSpeed) {
  Options options;
  rapidjson::Document dom;
  rapidjson::SetValueByPointer(dom, "/top_speed", 500);

  options.set_costing(Costing::auto_);
  sif::ParseSharedCostOptions(*rapidjson::GetValueByPointer(dom, ""), options.add_costing_options());

  ASSERT_EQ(options.costing_options().Get(0).top_speed(), baldr::kMaxAssumedSpeed);
}
