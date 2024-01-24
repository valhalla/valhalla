#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla;

inline float getDuration(const valhalla::Api& route) {
  return route.directions().routes(0).legs(0).summary().time();
}

class TruckRestrictionTest : public ::testing::TestWithParam<std::pair<std::string, std::string>> {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
      A---B---C---D
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "residential"}}},
        // all restrictions should be higher than our defaults, so we can actually see the impact of
        // any single one
        {"BC",
         {{"highway", "residential"},
          {"maxheight", "5"},
          {"maxlength", "25"},
          {"maxwidth", "3"},
          {"hazmat", "destination"},
          {"maxaxles", "8"},
          {"maxaxleload", "10"}}},
        {"CD", {{"highway", "residential"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/truck_restrictions");
  }
};

gurka::map TruckRestrictionTest::map = {};

TEST_P(TruckRestrictionTest, NotAllowed) {
  std::string option, v;
  std::tie(option, v) = GetParam();

  // "no path could be found for input" should be raised if we exceed this costing option
  try {
    gurka::do_action(Options::route, map, {"A", "D"}, "truck",
                     {{"/costing_options/truck/" + option, v}});
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_F(TruckRestrictionTest, Allowed) {
  // without setting a costing option, we should get a path
  auto res = gurka::do_action(Options::route, map, {"A", "D"}, "truck");
  gurka::assert::raw::expect_path(res, {"AB", "BC", "CD"});
}

INSTANTIATE_TEST_SUITE_P(TruckRestrictions,
                         TruckRestrictionTest,
                         ::testing::Values(std::pair<std::string, std::string>{"height", "6"},
                                           std::pair<std::string, std::string>{"width", "4"},
                                           std::pair<std::string, std::string>{"length", "30"},
                                           std::pair<std::string, std::string>{"hazmat", "true"},
                                           std::pair<std::string, std::string>{"axle_load", "11"},
                                           std::pair<std::string, std::string>{"axle_count", "10"}));

TEST(TruckSpeed, MaxTruckSpeed) {
  constexpr double gridsize = 500;

  const std::string ascii_map = R"(
      A----------B
    )";

  const gurka::ways ways = {{"AB", {{"highway", "motorway"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  gurka::map map = gurka::buildtiles(layout, ways, {}, {}, "test/data/truckspeed");

  map.config.put("mjolnir.traffic_extract", "test/data/truckspeed/traffic.tar");

  test::build_live_traffic_data(map.config);

  valhalla::Api default_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "truck", {});

  // should be clamped to kMaxAssumedTruckSpeed
  valhalla::Api clamped_top_speed_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "truck",
                       {{"/costing_options/truck/top_speed", "100"},
                        {"/date_time/type", "0"},
                        {"/date_time/value", "current"}});

  // just below kMaxAssumedTruckSpeed
  valhalla::Api low_top_speed_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "truck",
                       {{"/costing_options/truck/top_speed", "70"},
                        {"/date_time/type", "0"},
                        {"/date_time/value", "current"}});

  test::customize_live_traffic_data(map.config, [&](baldr::GraphReader& reader,
                                                    baldr::TrafficTile& tile, u_int32_t index,
                                                    valhalla::baldr::TrafficSpeed* traffic_speed) {
    baldr::GraphId tile_id(tile.header->tile_id);
    auto AB = gurka::findEdge(reader, map.nodes, "AB", "B", tile_id);

    if (std::get<1>(AB) != nullptr && std::get<0>(AB).id() == index) {
      traffic_speed->overall_encoded_speed = 140 >> 1;
      traffic_speed->breakpoint1 = 255;
      traffic_speed->encoded_speed1 = 140 >> 1;
    } else {
      traffic_speed->overall_encoded_speed = UNKNOWN_TRAFFIC_SPEED_RAW - 1;
    }
  });

  valhalla::Api modified_traffic_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "truck",
                       {{"/date_time/type", "0"},
                        {"/date_time/value", "current"},
                        {"/costing_options/truck/speed_types/0", "current"}});

  test::customize_live_traffic_data(map.config, [&](baldr::GraphReader& reader,
                                                    baldr::TrafficTile& tile, u_int32_t index,
                                                    valhalla::baldr::TrafficSpeed* traffic_speed) {
    baldr::GraphId tile_id(tile.header->tile_id);
    auto AB = gurka::findEdge(reader, map.nodes, "AB", "B", tile_id);

    if (std::get<1>(AB) != nullptr && std::get<0>(AB).id() == index) {
      traffic_speed->overall_encoded_speed = 50 >> 1;
      traffic_speed->breakpoint1 = 255;
      traffic_speed->encoded_speed1 = 50 >> 1;
    } else {
      traffic_speed->overall_encoded_speed = UNKNOWN_TRAFFIC_SPEED_RAW - 1;
    }
  });

  valhalla::Api modified_traffic_low_speed_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "truck",
                       {{"/date_time/type", "0"},
                        {"/date_time/value", "current"},
                        {"/costing_options/truck/speed_types/0", "current"}});

  gurka::assert::raw::expect_path(default_route, {"AB"});

  auto default_time = getDuration(default_route);
  auto clamped_top_speed_time = getDuration(clamped_top_speed_route);
  auto low_top_speed_time = getDuration(low_top_speed_route);
  auto traffic_time = getDuration(modified_traffic_route);
  auto traffic_low_speed_time = getDuration(modified_traffic_low_speed_route);

  // default and clamped durations should be the same in this case
  ASSERT_EQ(default_time, clamped_top_speed_time);

  // expect a trip to take longer when a low top speed is set
  ASSERT_LT(default_time, low_top_speed_time);

  // expect duration to be equal to default if traffic speed is higher than kMaxAssumedTruckCost
  // and no truck specific speed tag is set on the way
  ASSERT_EQ(default_time, traffic_time);

  // expect lower traffic speeds (< kMaxAssumedTruckSpeed ) to lead to a lower duration
  ASSERT_LT(traffic_time, traffic_low_speed_time);
}
