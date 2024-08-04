#include "baldr/graphconstants.h"
#include "gurka.h"
#include "test.h"
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
  rapidjson::SetValueByPointer(dom, "/auto/top_speed", 500);
  Costing co;

  options.set_costing_type(Costing::auto_);
  sif::ParseAutoCostOptions(dom, "/auto", &co);

  ASSERT_EQ(co.options().top_speed(), baldr::kMaxAssumedSpeed);
}

TEST(TopSpeed, CurrentLayerIsIgnored) {
  constexpr double gridsize = 10;

  const std::string ascii_map = R"(
      A---B----------C--D
          |          |
          E----------F
    )";

  const gurka::ways ways = {{"AB", {{"highway", "residential"}, {"maxspeed", "20"}}},
                            {"CD", {{"highway", "residential"}, {"maxspeed", "20"}}},
                            {"BC", {{"highway", "residential"}, {"maxspeed", "70"}}},
                            {"BEFC", {{"highway", "residential"}, {"maxspeed", "20"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  gurka::map map = gurka::buildtiles(layout, ways, {}, {}, "test/data/topspeed");

  map.config.put("mjolnir.traffic_extract", "test/data/topspeed/traffic.tar");

  // add live traffic
  test::build_live_traffic_data(map.config);
  test::customize_live_traffic_data(map.config, [&](baldr::GraphReader&, baldr::TrafficTile&, int,
                                                    valhalla::baldr::TrafficSpeed* traffic_speed) {
    traffic_speed->overall_encoded_speed = 20 >> 1;
    traffic_speed->encoded_speed1 = 20 >> 1;
    traffic_speed->breakpoint1 = 255;
  });

  valhalla::Api default_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto",
                       {{"/date_time/type", "0"}, {"/date_time/value", "current"}});
  valhalla::Api capped_route = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto",
                                                {{"/costing_options/auto/top_speed", "20"},
                                                 {"/date_time/type", "0"},
                                                 {"/date_time/value", "current"}});

  // for default route all edges has the same live speed so the shortest path is expected to be
  // returned(ABCD)
  gurka::assert::raw::expect_path(default_route, {"AB", "BC", "CD"});
  // for capped route, the longest route is expected to be returned because for BC edge:
  //  top_speed=20kmh
  //  average_edge_speed=70kmh(current layer is ignored when top_speed option is active)
  // 70 kmh > 20 kmh so BC edge is penalized
  gurka::assert::raw::expect_path(capped_route, {"AB", "BEFC", "CD"});
}
