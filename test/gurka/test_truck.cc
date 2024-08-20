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
    FAIL() << "Expected no path to be found";
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
                                           std::pair<std::string, std::string>{"hazmat", "1"},
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

  // should be clamped to edge speed
  valhalla::Api clamped_top_speed_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "truck",
                       {{"/costing_options/truck/top_speed", "115"},
                        {"/date_time/type", "0"},
                        {"/date_time/value", "current"}});

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

  // both default and set top_speeds exceeds edge speed, so use edge speed in both cases
  ASSERT_EQ(default_time, clamped_top_speed_time);

  // expect a trip to take longer when a low top speed is set
  ASSERT_LT(default_time, low_top_speed_time);

  // was clamped to 120 KPH, traffic speed was set to 140
  ASSERT_EQ(5500 / (120 / 3.6), traffic_time);

  // expect lower traffic speeds to lead to a lower duration
  ASSERT_LT(traffic_time, traffic_low_speed_time);
}

TEST(TruckSpeed, TopSpeed) {
  constexpr double gridsize = 500;

  const std::string ascii_map = R"(
      A----B
    )";

  const gurka::ways ways = {{"AB", {{"highway", "motorway"}, {"maxspeed", "120"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  gurka::map map = gurka::buildtiles(layout, ways, {}, {}, "test/data/truckspeed");

  valhalla::Api default_route = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "truck");

  valhalla::Api top_speed_route = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "truck",
                                                   {{"/costing_options/truck/top_speed", "110"}});

  valhalla::Api default_top_speed_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "truck",
                       {{"/costing_options/truck/top_speed", "120"}});

  auto default_dur = getDuration(default_route);
  auto top_speed_dur = getDuration(top_speed_route);
  auto default_top_speed_dur = getDuration(default_top_speed_route);

  ASSERT_LT(default_dur, top_speed_dur);
  ASSERT_EQ(default_dur, default_top_speed_dur);
}

// tag name, tag value, costing opt name, costing opt value, forward
using asymmetric_restriction_params_t =
    std::tuple<std::string, std::string, std::string, std::string, bool>;

class TruckAsymmetricAccessRestrictions
    : public ::testing::TestWithParam<asymmetric_restriction_params_t> {

protected:
  static std::string ascii_map;

  static void SetUpTestSuite() {

    ascii_map = R"(
      A---B---C---D
    )";
  }
};
std::string TruckAsymmetricAccessRestrictions::ascii_map = {};

// test forward/backward only access restrictions
TEST_P(TruckAsymmetricAccessRestrictions, ForwardBackward) {
  std::string tag_name;
  std::string tag_value;
  std::string co_opt;
  std::string co_value;
  bool forward;
  std::tie(tag_name, tag_value, co_opt, co_value, forward) = GetParam();

  std::string tag_name_full = tag_name + (forward ? ":forward" : ":backward");

  // middle edge has a forward/backward only access restriction
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}, {tag_name_full, tag_value}}},
      {"CD", {{"highway", "residential"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  gurka::map map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/truck_directed_access_restrictions");

  // we test a route in the direction of the access restriction, should fail...
  std::vector<std::string> locations_fail = {"A", "D"};
  // ...but should work in the other direction...
  std::vector<std::string> locations_success = {"D", "A"};
  // ...and this should be the succeeding route's path
  std::vector<std::string> expected_path_success = {"CD", "BC", "AB"};

  // reverse case if backward
  if (!forward) {
    locations_fail = {"D", "A"};
    locations_success = {"A", "D"};
    expected_path_success = {"AB", "BC", "CD"};
  }

  const std::unordered_map<std::string, std::string> costing_opts = {{co_opt, co_value}};
  try {
    std::vector<std::vector<std::string>> options;
    gurka::do_action(Options::route, map, locations_fail, "truck", costing_opts);
    FAIL() << "Expected no path to be found";
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };

  // other direction should work
  const auto success_route =
      gurka::do_action(Options::route, map, locations_success, "truck", costing_opts);
  gurka::assert::raw::expect_path(success_route, expected_path_success);
}

std::vector<asymmetric_restriction_params_t> buildParams() {
  std::vector<asymmetric_restriction_params_t> params;

  params.push_back(std::make_tuple("maxheight", "4", "/costing_options/truck/height", "7", true));
  params.push_back(std::make_tuple("maxheight", "4", "/costing_options/truck/height", "7", false));
  params.push_back(std::make_tuple("maxlength", "4", "/costing_options/truck/length", "5", true));
  params.push_back(std::make_tuple("maxlength", "4", "/costing_options/truck/length", "5", false));
  params.push_back(std::make_tuple("maxwidth", "2", "/costing_options/truck/length", "3", true));
  params.push_back(std::make_tuple("maxwidth", "2", "/costing_options/truck/length", "3", false));
  params.push_back(std::make_tuple("hazmat", "no", "/costing_options/truck/hazmat", "1", true));

  return params;
};

INSTANTIATE_TEST_SUITE_P(TruckAsymmetricAccessRestrictionsTest,
                         TruckAsymmetricAccessRestrictions,
                         ::testing::ValuesIn(buildParams()));

TEST(Standalone, TruckSpeeds) {
  constexpr double gridsize = 500;

  const std::string ascii_map = R"(
      A---B---C---D---E
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}, {"maxspeed:hgv:forward", "15"}}},
      {"BC", {{"highway", "residential"}, {"maxspeed:hgv:backward", "15"}}},
      // truck_speed > truck_speed_forward
      {"CD", {{"highway", "residential"}, {"maxspeed:hgv", "20"}, {"maxspeed:hgv:forward", "15"}}},
      // truck_speed < truck_speed_forward
      {"DE", {{"highway", "residential"}, {"maxspeed:hgv", "15"}, {"maxspeed:hgv:forward", "20"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  gurka::map map = gurka::buildtiles(layout, ways, {}, {}, "test/data/truck_speed");

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  const DirectedEdge *de_fwd, *de_rev, *de_both_1, *de_both_2;
  std::tie(std::ignore, de_fwd) = gurka::findEdgeByNodes(reader, layout, "A", "B");
  std::tie(std::ignore, de_rev) = gurka::findEdgeByNodes(reader, layout, "C", "B");
  std::tie(std::ignore, de_both_1) = gurka::findEdgeByNodes(reader, layout, "C", "D");
  std::tie(std::ignore, de_both_2) = gurka::findEdgeByNodes(reader, layout, "D", "E");
  EXPECT_EQ(de_fwd->truck_speed(), 15);
  EXPECT_EQ(de_rev->truck_speed(), 15);
  EXPECT_EQ(de_both_1->truck_speed(), 15);
  EXPECT_EQ(de_both_2->truck_speed(), 15);
}

TEST(Standalone, UseTruckRoute) {
  constexpr double gridsize = 500;

  const std::string ascii_map = R"(
      A------B
      |      |
      |      |
      |      |
      |      |
      |      C
      |      |
      E------D
    )";

  const gurka::ways ways = {{"AB", {{"highway", "residential"}}},
                            {"AE", {{"highway", "residential"}, {"hgv", "designated"}}},
                            {"BC", {{"highway", "residential"}}},
                            {"CD", {{"highway", "residential"}}},
                            {"ED", {{"highway", "residential"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  gurka::map map = gurka::buildtiles(layout, ways, {}, {}, "test/data/use_truck_route");

  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/truck/use_truck_route", "1"}};

  valhalla::Api default_route = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "truck");

  valhalla::Api use_truck_route =
      gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "truck", options);

  gurka::assert::raw::expect_path(default_route, {"AB", "BC"});
  gurka::assert::raw::expect_path(use_truck_route, {"AE", "ED", "CD"});
}
