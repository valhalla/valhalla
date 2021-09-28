#include "gurka.h"
#include "midgard/encoded.h"
#include "midgard/util.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla;

/*************************************************************/
TEST(Standalone, BasicMatch) {

  const std::string ascii_map = R"(
      1
    A---2B-3-4C
              |
              |5
              D
         )";

  const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                            {"BC", {{"highway", "primary"}}},
                            {"CD", {{"highway", "primary"}}}};

  const double gridsize = 10;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/basic_match");

  auto result = gurka::do_action(valhalla::Options::trace_route, map, {"1", "2", "3", "4", "5"},
                                 "auto", {}, {}, nullptr, "via");

  gurka::assert::osrm::expect_match(result, {"AB", "CD"});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg::Maneuver::kStart,
                                                DirectionsLeg::Maneuver::kRight,
                                                DirectionsLeg::Maneuver::kDestination});

  gurka::assert::raw::expect_path_length(result, 0.100, 0.001);
}

TEST(Standalone, UturnMatch) {

  const std::string ascii_map = "A--1--2--B";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/uturn_match");
  std::vector<std::string> expected_names;
  for (const auto& test_case : std::vector<std::pair<std::string, float>>{{"break", 90},
                                                                          {"via", 90},
                                                                          {"through", 210},
                                                                          {"break_through", 210}}) {

    auto result = gurka::do_action(valhalla::Options::trace_route, map, {"1", "2", "1", "2"}, "auto",
                                   {{"/trace_options/penalize_immediate_uturn", "0"}}, {}, nullptr,
                                   test_case.first);

    // throughs or vias will make a uturn without a destination notification (left hand driving)
    std::vector<DirectionsLeg::Maneuver::Type> expected_maneuvers{
        DirectionsLeg::Maneuver::kStart,
        DirectionsLeg::Maneuver::kUturnRight,
        DirectionsLeg::Maneuver::kUturnRight,
        DirectionsLeg::Maneuver::kDestination,
    };
    expected_names = {"AB", "AB", "AB"};

    // break will have destination at the trace point
    if (test_case.first == "break") {
      expected_maneuvers = {
          DirectionsLeg::Maneuver::kStart, DirectionsLeg::Maneuver::kDestination,
          DirectionsLeg::Maneuver::kStart, DirectionsLeg::Maneuver::kDestination,
          DirectionsLeg::Maneuver::kStart, DirectionsLeg::Maneuver::kDestination,
      };
      expected_names = {"AB", "AB", "AB"};
    } // break through will have destinations at the trace points but will have to make uturns
    else if (test_case.first == "break_through") {
      expected_maneuvers = {
          DirectionsLeg::Maneuver::kStart,       DirectionsLeg::Maneuver::kDestination,
          DirectionsLeg::Maneuver::kStart,       DirectionsLeg::Maneuver::kUturnRight,
          DirectionsLeg::Maneuver::kDestination, DirectionsLeg::Maneuver::kStart,
          DirectionsLeg::Maneuver::kUturnRight,  DirectionsLeg::Maneuver::kDestination,
      };
      expected_names = {"AB", "AB", "AB", "AB", "AB"};
    }

    gurka::assert::osrm::expect_match(result, {"AB"});
    gurka::assert::raw::expect_path(result, expected_names);
    gurka::assert::raw::expect_maneuvers(result, expected_maneuvers);
    gurka::assert::raw::expect_path_length(result, test_case.second / 1000, 0.001);
  }
}

TEST(Standalone, UturnTrimmingAsan) {
  const std::string ascii_map = R"(
A--1--2--B
         |
         |
D--3--4--C--5--6--E)";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"DCE", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/uturn_asan");

  auto result =
      gurka::do_action(valhalla::Options::trace_route, map, {"2", "6", "3"}, "auto",
                       {{"/trace_options/penalize_immediate_uturn", "0"}}, {}, nullptr, "via");

  auto shape =
      midgard::decode<std::vector<midgard::PointLL>>(result.trip().routes(0).legs(0).shape());
  auto expected_shape = decltype(shape){
      map.nodes["2"], map.nodes["B"], map.nodes["C"], map.nodes["6"], map.nodes["C"], map.nodes["3"],
  };
  EXPECT_EQ(shape.size(), expected_shape.size());
  for (int i = 0; i < shape.size(); ++i) {
    EXPECT_TRUE(shape[i].ApproximatelyEqual(expected_shape[i]));
  }
}

TEST(MapMatch, NodeSnapFix) {
  const std::string ascii_map = R"(
      B-C------------1----F---------D
  )";

  // The challenge posed by this case is that the gps point "B" is exactly the
  // lat/lon as the point "B" in the graph.
  gurka::nodelayout layout = gurka::detail::map_to_coordinates(ascii_map, 10);

  // Another thing about this case... if I declare the ways in this manner the
  // original code works fine. The reason, I believe, is that this creates one
  // edge with a trivial node-to-node (along the same edge) route.
  //  const gurka::ways ways = {
  //      {"BCFD", {{"highway", "primary"}}}
  //  };

  // To expose the issue and prove the fix, the ways must be declared in this
  // manner. This results in a routing solution along two edges.
  const gurka::ways ways = {
      {"BC", {{"highway", "primary"}}},
      {"CF", {{"highway", "primary"}}},
      {"FD", {{"highway", "primary"}}},
  };

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/mapmatch_node_snapping");

  auto result = gurka::do_action(valhalla::Options::trace_route, map, {"B", "1", "F"}, "auto", {}, {},
                                 nullptr, "via");

  auto shape =
      midgard::decode<std::vector<midgard::PointLL>>(result.trip().routes(0).legs(0).shape());

  auto expected_shape = decltype(shape){map.nodes["B"], map.nodes["C"], map.nodes["F"]};
  EXPECT_EQ(shape.size(), expected_shape.size());
  for (int i = 0; i < shape.size(); ++i) {
    EXPECT_TRUE(shape[i].ApproximatelyEqual(expected_shape[i]));
  }
}

/****************THIS BOILERPLATE WAS COPYPASTA'D FROM THE ROUTE TEST****************/
class TrafficBasedTest : public ::testing::Test {
protected:
  static gurka::map map;
  static uint32_t current, historical, constrained, freeflow;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
      b----1----c----2----d
      |         |         |
      |         |         |
      |         |         |
      |         |         |
      3         4         5
      |         |         |
      |         |         |
      |         |         |
      |         |         |
      a----6----f----7----e
      |         |         |
      |         |         |
      |         |         |
      |         |         |
      8         9         0
      |         |         |
      |         |         |
      |         |         |
      |         |         |
      g----A----h----B----i
    )";

    const gurka::ways ways = {
        {"abcdefaghie", {{"highway", "residential"}}},
        {"cfh", {{"highway", "tertiary"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/match_timedep",
                            {
                                {"mjolnir.shortcuts", "false"},
                                {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"},
                            });
    map.config.put("mjolnir.traffic_extract", "test/data/match_timedep/traffic.tar");

    // add live traffic
    test::build_live_traffic_data(map.config);
    test::customize_live_traffic_data(map.config, [&](baldr::GraphReader&, baldr::TrafficTile&, int,
                                                      valhalla::baldr::TrafficSpeed* traffic_speed) {
      traffic_speed->overall_encoded_speed = 50 >> 1;
      traffic_speed->encoded_speed1 = 50 >> 1;
      traffic_speed->breakpoint1 = 255;
    });

    test::customize_historical_traffic(map.config, [](DirectedEdge& e) {
      e.set_constrained_flow_speed(25);
      e.set_free_flow_speed(75);
      std::array<float, kBucketsPerWeek> historical;
      historical.fill(7);
      for (size_t i = 0; i < historical.size(); ++i) {
        // TODO: if we are in morning or evening set a different speed and add another test
      }
      return historical;
    });

    // tests below use saturday at 9am and thursday 4am (27 if for leap seconds)
    size_t second_of_week_constrained = 5 * 24 * 60 * 60 + 9 * 60 * 60 + 27;
    size_t second_of_week_freeflow = 3 * 24 * 60 * 60 + 4 * 60 * 60 + 27;
    auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
    for (auto tile_id : reader->GetTileSet()) {
      auto tile = reader->GetGraphTile(tile_id);
      for (const auto& e : tile->GetDirectedEdges()) {
        current = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0);
        EXPECT_EQ(current, 50);
        historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, second_of_week_constrained);
        EXPECT_EQ(historical, 7);
        constrained = tile->GetSpeed(&e, baldr::kConstrainedFlowMask, second_of_week_constrained);
        EXPECT_EQ(constrained, 25);
        freeflow = tile->GetSpeed(&e, baldr::kFreeFlowMask, second_of_week_freeflow);
        EXPECT_EQ(freeflow, 75);
      }
    }
  }
};

gurka::map TrafficBasedTest::map = {};
uint32_t TrafficBasedTest::current = 0, TrafficBasedTest::historical = 0,
         TrafficBasedTest::constrained = 0, TrafficBasedTest::freeflow = 0;

uint32_t speed_from_edge(const valhalla::Api& api) {
  uint32_t kmh = -1;
  const auto& nodes = api.trip().routes(0).legs(0).node();
  for (int i = 0; i < nodes.size() - 1; ++i) {
    const auto& node = nodes.Get(i);
    if (!node.has_edge())
      break;
    auto km = node.edge().length_km();
    auto h = (nodes.Get(i + 1).cost().elapsed_cost().seconds() -
              node.cost().elapsed_cost().seconds() - node.cost().transition_cost().seconds()) /
             3600.0;
    auto new_kmh = static_cast<uint32_t>(km / h + .5);
    if (kmh != -1)
      EXPECT_EQ(kmh, new_kmh);
    kmh = new_kmh;
  }
  return kmh;
}

// map matching currently only allows forward time tracking and invariant so types 0, 1, 3.
// type 2 arrive_by is not yet supported
TEST_F(TrafficBasedTest, forward) {
  {
    auto api = gurka::do_action(valhalla::Options::trace_route, map, {"4", "0"}, "auto");
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "map_snap");
    EXPECT_EQ(speed_from_edge(api), constrained);
  }

  {
    auto api = gurka::do_action(valhalla::Options::trace_route, map, {"A", "2"}, "auto",
                                {{"/date_time/type", "1"},
                                 {"/date_time/value", "2020-10-30T09:00"},
                                 {"/costing_options/auto/speed_types/0", "constrained"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "map_snap");
    EXPECT_EQ(speed_from_edge(api), constrained);
  }

  {
    auto api = gurka::do_action(valhalla::Options::trace_route, map, {"A", "2"}, "auto",
                                {{"/date_time/type", "3"},
                                 {"/date_time/value", "2020-10-30T09:00"},
                                 {"/costing_options/auto/speed_types/0", "predicted"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "map_snap");
    EXPECT_EQ(speed_from_edge(api), historical);
  }

  {
    auto api = gurka::do_action(valhalla::Options::trace_route, map, {"A", "2"}, "auto",
                                {{"/date_time/type", "0"},
                                 {"/date_time/value", "2020-10-30T09:00"},
                                 {"/costing_options/auto/speed_types/0", "current"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "map_snap");
    EXPECT_EQ(speed_from_edge(api), current);
  }

  {
    auto api = gurka::do_action(valhalla::Options::trace_route, map, {"A", "2"}, "auto",
                                {{"/date_time/type", "3"},
                                 {"/date_time/value", "2020-10-28T04:00"},
                                 {"/costing_options/auto/speed_types/0", "freeflow"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "map_snap");
    EXPECT_EQ(speed_from_edge(api), freeflow);
  }
}

TEST(MapMatchRoute, IgnoreRestrictions) {
  const std::string ascii_map = R"(
    A------B----C
     )";

  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}}},
      {"BC", {{"highway", "motorway"}}},
  };
  const gurka::nodelayout layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const gurka::relations relations = {{{
                                           {gurka::way_member, "AB", "from"},
                                           {gurka::way_member, "BC", "to"},
                                           {gurka::node_member, "B", "via"},
                                       },
                                       {{"type", "restriction"}, {"restriction", "no_straight_on"}}}};
  const gurka::map map =
      gurka::buildtiles(layout, ways, {}, relations, "test/data/mapmatch_restrictions",
                        {{"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"}});

  // ignore_restrictions when route with map_matching
  auto result = gurka::do_action(valhalla::Options::trace_route, map, {"A", "C"}, "auto",
                                 {{"/costing_options/auto/ignore_restrictions", "1"}});
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}
