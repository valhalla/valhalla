#include "baldr/graphconstants.h"
#include "gurka.h"
#include "test.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

class FixedSpeedTest : public ::testing::Test {
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
    speed_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/fixedspeed");
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
    gurka::assert::raw::expect_eta(capped_route, 7.92f, 0.001f);
    ASSERT_LT(capped_time, default_time);
  }
};

gurka::map FixedSpeedTest::speed_map = {};

TEST_F(FixedSpeedTest, AutoFixedSpeed) {
  doTests("auto", {{"/costing_options/auto/fixed_speed", "50"}});
}

TEST_F(FixedSpeedTest, TruckFixedSpeed) {
  doTests("truck", {{"/costing_options/truck/fixed_speed", "50"}});
}

TEST_F(FixedSpeedTest, MotorcycleFixedSpeed) {
  doTests("motorcycle", {{"/costing_options/motorcycle/fixed_speed", "50"}});
}

TEST_F(FixedSpeedTest, MotorscooterFixedSpeed) {
  doTests("motor_scooter", {{"/costing_options/motor_scooter/fixed_speed", "50"}});
}

TEST_F(FixedSpeedTest, ClampMaxSpeed) {
  Options options;
  rapidjson::Document dom;
  rapidjson::SetValueByPointer(dom, "/fixed_speed", 500);

  options.set_costing_type(Costing::auto_);
  auto& co = (*options.mutable_costings())[Costing::auto_];
  sif::ParseBaseCostOptions(*rapidjson::GetValueByPointer(dom, ""), &co, {});

  ASSERT_EQ(co.options().fixed_speed(), baldr::kDisableFixedSpeed);
}

TEST_F(FixedSpeedTest, TopAndFixedSpeed) {
  doTests("auto",
          {{"/costing_options/auto/fixed_speed", "50"}, {"/costing_options/auto/top_speed", "80"}});
  doTests("auto",
          {{"/costing_options/auto/fixed_speed", "50"}, {"/costing_options/auto/top_speed", "50"}});
  doTests("auto",
          {{"/costing_options/auto/fixed_speed", "50"}, {"/costing_options/auto/top_speed", "10"}});
}