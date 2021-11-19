#include "gurka.h"
#include "test.h"

using namespace valhalla;

class TrafficSmoothing : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
      A----B-----C-----D
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}}},
        {"BC", {{"highway", "primary"}}},
        {"CD", {{"highway", "primary"}}},
    };
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10000);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/traffic_evaluation",
                            {
                                {"mjolnir.shortcuts", "false"},
                                {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"},
                            });
    map.config.put("mjolnir.traffic_extract", "test/data/traffic_evaluation/traffic.tar");
    map.config.put("thor.invariant_recosting", false);

    // add live traffic
    test::build_live_traffic_data(map.config);
    test::customize_live_traffic_data(map.config, [&](baldr::GraphReader&, baldr::TrafficTile&, int,
                                                      valhalla::baldr::TrafficSpeed* traffic_speed) {
      traffic_speed->overall_encoded_speed = 52 >> 1;
      traffic_speed->encoded_speed1 = 52 >> 1;
      traffic_speed->breakpoint1 = 255;
    });

    test::customize_historical_traffic(map.config, [](DirectedEdge& e) {
      e.set_constrained_flow_speed(40);
      e.set_free_flow_speed(100);

      // speeds for every 5 min bucket of the week
      std::array<float, kBucketsPerWeek> historical;
      historical.fill(10);
      for (size_t i = 0; i < historical.size(); ++i) {
        size_t min_timestamp = (i % (24 * 12)) / 12;
        // speeds varies from 19 to 31 km/h.
        if (min_timestamp < 12) {
          historical[i] = 31 - min_timestamp;
        } else {
          historical[i] = 19 + (min_timestamp - 12);
        }
      }
      return historical;
    });
  }

  std::string make_request(std::string from,
                           std::string to,
                           std::string speed_types,
                           int date_time_type,
                           std::string date_time_value) const {
    const std::string query_pattern_with_speeds = R"({
      "locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],
      "costing": "auto",
      "costing_options":{"auto":{"speed_types":[%s]}},
      "date_time": { "type": "%d", "value": "%s" }
    })";
    return (boost::format(query_pattern_with_speeds) % std::to_string(map.nodes.at(from).lat()) %
            std::to_string(map.nodes.at(from).lng()) % std::to_string(map.nodes.at(to).lat()) %
            std::to_string(map.nodes.at(to).lng()) % speed_types % date_time_type % date_time_value)
        .str();
  }

  static gurka::map map;
  static uint32_t current, historical, constrained, freeflow;
};

gurka::map TrafficSmoothing::map = {};
uint32_t TrafficSmoothing::current = 0, TrafficSmoothing::historical = 0,
         TrafficSmoothing::constrained = 0, TrafficSmoothing::freeflow = 0;

TEST_F(TrafficSmoothing, LiveTrafficSpeed) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  for (auto tile_id : reader->GetTileSet()) {
    auto tile = reader->GetGraphTile(tile_id);
    for (const auto& e : tile->GetDirectedEdges()) {
      current = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0);
      EXPECT_EQ(current, 52);

      current = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 10000);
      EXPECT_EQ(current, 52);

      uint8_t* flow_sources = nullptr;
      current = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0, false, flow_sources, 0);
      EXPECT_EQ(current, 52);

      current = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0, false, flow_sources, 3600);
      EXPECT_NE(current, 52);

      current = tile->GetSpeed(&e, baldr::kCurrentFlowMask | baldr::kPredictedFlowMask, 1000, false,
                               flow_sources, 0);
      EXPECT_EQ(current, 52);

      current = tile->GetSpeed(&e, baldr::kCurrentFlowMask | baldr::kPredictedFlowMask, 1000, false,
                               flow_sources, 1000);
      EXPECT_NE(current, 52);

      current = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 1000);
      EXPECT_NE(current, 52);
    }
  }
}

TEST_F(TrafficSmoothing, HistoricalTrafficSpeed) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  for (auto tile_id : reader->GetTileSet()) {
    auto tile = reader->GetGraphTile(tile_id);
    for (const auto& e : tile->GetDirectedEdges()) {
      historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 100);
      EXPECT_EQ(historical, 31);

      historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 3 * 24 * 60 * 60);
      EXPECT_EQ(historical, 31);

      historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 60 * 60 * 5);
      EXPECT_EQ(historical, 26);

      historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 60 * 60 * 7);
      EXPECT_EQ(historical, 24);

      historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 60 * 60 * 13);
      EXPECT_EQ(historical, 20);

      historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 60 * 60 * 19);
      EXPECT_EQ(historical, 26);

      uint8_t* flow_sources = nullptr;
      historical = tile->GetSpeed(&e, baldr::kCurrentFlowMask | baldr::kPredictedFlowMask,
                                  60 * 60 * 19, false, flow_sources, false);
      EXPECT_NE(historical, 26);

      historical = tile->GetSpeed(&e, baldr::kCurrentFlowMask | baldr::kConstrainedFlowMask,
                                  60 * 60 * 19, false, flow_sources, false);
      EXPECT_NE(historical, 26);

      historical = tile->GetSpeed(&e, baldr::kCurrentFlowMask | baldr::kConstrainedFlowMask,
                                  60 * 60 * 19, false, flow_sources, true);
      EXPECT_NE(historical, 26);

      historical = tile->GetSpeed(&e, baldr::kConstrainedFlowMask, 60 * 60 * 19);
      EXPECT_NE(historical, 26);

      historical = tile->GetSpeed(&e, baldr::kFreeFlowMask, 60 * 60 * 19);
      EXPECT_NE(historical, 26);
    }
  }
}

TEST_F(TrafficSmoothing, ConstrainedTrafficSpeed) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  for (auto tile_id : reader->GetTileSet()) {
    auto tile = reader->GetGraphTile(tile_id);
    for (const auto& e : tile->GetDirectedEdges()) {
      constrained = tile->GetSpeed(&e, baldr::kConstrainedFlowMask, 60 * 60 * 12);
      EXPECT_EQ(constrained, 40);

      constrained =
          tile->GetSpeed(&e, baldr::kConstrainedFlowMask | baldr::kFreeFlowMask, 60 * 60 * 12);
      EXPECT_EQ(constrained, 40.);

      constrained = tile->GetSpeed(&e, baldr::kConstrainedFlowMask | baldr::kFreeFlowMask, 0);
      EXPECT_NE(constrained, 40);

      constrained = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0);
      EXPECT_NE(constrained, 40);

      constrained = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 0);
      EXPECT_NE(constrained, 40);
    }
  }
}

TEST_F(TrafficSmoothing, FreeflowTrafficSpeed) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  for (auto tile_id : reader->GetTileSet()) {
    auto tile = reader->GetGraphTile(tile_id);
    for (const auto& e : tile->GetDirectedEdges()) {
      freeflow = tile->GetSpeed(&e, baldr::kFreeFlowMask, 0);
      EXPECT_EQ(freeflow, 100);

      freeflow = tile->GetSpeed(&e, baldr::kConstrainedFlowMask, 0);
      EXPECT_NE(freeflow, 100);

      freeflow = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 0);
      EXPECT_NE(freeflow, 100);

      freeflow = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0);
      EXPECT_NE(freeflow, 100);
    }
  }
}

TEST_F(TrafficSmoothing, LiveTrafficOneEdge) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\",\"current\"";

  auto result =
      gurka::do_action(valhalla::Options::route, map, make_request("A", "B", speeds, 3, "current"));
  EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  gurka::assert::raw::expect_path_length(result, 50, 0.1);
  gurka::assert::raw::expect_eta(result, 3461, 10);
}

TEST_F(TrafficSmoothing, PredictedTrafficOneEdge) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";

  auto result = gurka::do_action(valhalla::Options::route, map,
                                 make_request("A", "B", speeds, 3, "2021-11-08T19:00"));
  EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  gurka::assert::raw::expect_path_length(result, 50, 0.1);
  gurka::assert::raw::expect_eta(result, 7050, 300);

  result = gurka::do_action(valhalla::Options::route, map,
                            make_request("A", "B", speeds, 3, "2021-11-08T00:00"));
  EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  gurka::assert::raw::expect_path_length(result, 50, 0.1);
  gurka::assert::raw::expect_eta(result, 5900, 200);
}

TEST_F(TrafficSmoothing, LiveMixedWithPredictive) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\",\"current\"";

  {
    auto result =
        gurka::do_action(valhalla::Options::route, map, make_request("A", "C", speeds, 3, "current"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
    gurka::assert::raw::expect_path_length(result, 110, 0.1);
    // one should have live traffic, the second mixes live and predictive.
    // The error margin is huge because of predictive traffic value range.
    // The value should not be close to only live-traffic time.
    gurka::assert::raw::expect_eta(result, 12650, 3500);
  }

  {
    // the same test but date_time_type is 0 (aka depart_at=now)
    auto result =
        gurka::do_action(valhalla::Options::route, map, make_request("A", "C", speeds, 0, "current"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
    gurka::assert::raw::expect_path_length(result, 110, 0.1);
    gurka::assert::raw::expect_eta(result, 12650, 3500);
  }
}

TEST_F(TrafficSmoothing, PredictiveTrafficChangesOverTime) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "C", speeds, 3, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
    gurka::assert::raw::expect_path_length(result, 110, 0.1);
    // one should have predictive speed 31, the second has predictive speed 30 km/h.
    gurka::assert::raw::expect_eta(result, 13006, 1);
  }

  {
    // the same test but date_time_type is 1 (aka depart_at=2021-11-08T00:00)
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "C", speeds, 1, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
    gurka::assert::raw::expect_path_length(result, 110, 0.1);
    gurka::assert::raw::expect_eta(result, 13006, 1);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, 3, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // one should have predictive speed 31, the second has predictive speed 30, next speed is 28.
    gurka::assert::raw::expect_eta(result, 20720, 1);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, 3, "2021-11-08T12:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    gurka::assert::raw::expect_eta(result, 28759, 30);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, 3, "2021-11-08T18:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    gurka::assert::raw::expect_eta(result, 22948, 30);
  }
}

TEST_F(TrafficSmoothing, RecostingWithPredictiveTraffic) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, 3, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // one should have predictive speed 31, the second has predictive speed 30, next speed is 28.
    gurka::assert::raw::expect_eta(result, 20720, 30);
  }

  {
    map.config.put("thor.invariant_recosting", true);
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, 3, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // all edges have current speed - 31
    gurka::assert::raw::expect_eta(result, 19741, 10);
    map.config.put("thor.invariant_recosting", false);
  }
}

TEST_F(TrafficSmoothing, LiveAndPredictiveMixBidirectionalAstar) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\",\"current\"";

  {
    auto result =
        gurka::do_action(valhalla::Options::route, map, make_request("A", "D", speeds, 3, "current"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // the first edge has live speed (52), second has a mix of live (52) and predictive
    // traffic (smth between 19 and 31), last has only predictive traffic (smth between 19 and 31).
    gurka::assert::raw::expect_eta(result, 21211, 5500);
  }

  {
    map.config.put("thor.invariant_recosting", true);
    auto result =
        gurka::do_action(valhalla::Options::route, map, make_request("A", "D", speeds, 3, "current"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // all edges have current live speed - 52
    gurka::assert::raw::expect_eta(result, 11769, 10);
    map.config.put("thor.invariant_recosting", false);
  }
}

TEST_F(TrafficSmoothing, PredictiveAndForwardAstar) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, 1, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // one should have predictive speed 31, the second has predictive speed 30, next speed is 28.
    gurka::assert::raw::expect_eta(result, 20720, 1);
  }
}

TEST_F(TrafficSmoothing, ConstrainedAndFreeflow) {
  std::string speeds = "\"freeflow\",\"constrained\"";
  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, 3, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // freeflow
    gurka::assert::raw::expect_eta(result, 6120, 1);
  }
  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, 3, "2021-11-08T12:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // constrained
    gurka::assert::raw::expect_eta(result, 15300, 1);
  }
  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, 3, "2021-11-08T06:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // mixed, 2 freeflow (night), 1 constrained (day)
    gurka::assert::raw::expect_eta(result, 9360, 1);
  }

  {
    map.config.put("thor.invariant_recosting", true);
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, 3, "2021-11-08T06:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // no recosting -> freeflow everywhere
    gurka::assert::raw::expect_eta(result, 6120, 1);
    map.config.put("thor.invariant_recosting", false);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, 3, "2021-11-08T18:30"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // mixed, 1 constrained (day), 2 freeflow (night)
    gurka::assert::raw::expect_eta(result, 8820, 1);
  }

  {
    map.config.put("thor.invariant_recosting", true);
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, 3, "2021-11-08T18:30"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // no recosting -> constrained everywhere
    gurka::assert::raw::expect_eta(result, 15300, 1);
    map.config.put("thor.invariant_recosting", false);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, 3, "2021-11-08T17:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // mixed, 2 constrained (day), 1 freeflow (night)
    gurka::assert::raw::expect_eta(result, 12060, 1);
  }

  {
    map.config.put("thor.invariant_recosting", true);
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, 3, "2021-11-08T17:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // no recosting -> constrained everywhere
    gurka::assert::raw::expect_eta(result, 15300, 1);
    map.config.put("thor.invariant_recosting", false);
  }
}
