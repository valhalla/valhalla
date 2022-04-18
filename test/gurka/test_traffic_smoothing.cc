#include "gurka.h"
#include "test.h"

using namespace valhalla;

void set_traffic(gurka::map& map) {
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

class SmallRouteSpeedTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
      A----B
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}, {"maxspeed", std::to_string(max_speed_)}}},
    };
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10000);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/speed_smoothing");
    map.config.put("mjolnir.traffic_extract", "test/data/speed_smoothing/traffic.tar");
    set_traffic(map);
  }

  static gurka::map map;
  static uint32_t current_, historical_max_, constrained_, freeflow_, max_speed_;
};

gurka::map SmallRouteSpeedTest::map = {};
uint32_t SmallRouteSpeedTest::current_ = 52, SmallRouteSpeedTest::historical_max_ = 31,
         SmallRouteSpeedTest::constrained_ = 40, SmallRouteSpeedTest::freeflow_ = 100,
         SmallRouteSpeedTest::max_speed_ = 70;

TEST_F(SmallRouteSpeedTest, CurrentTraffic) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  for (auto tile_id : reader->GetTileSet()) {
    auto tile = reader->GetGraphTile(tile_id);
    for (const auto& e : tile->GetDirectedEdges()) {
      {
        // default seconds_from_now value is 0, live traffic is used
        auto flow_mask = baldr::kCurrentFlowMask;
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0), current_);
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 10000), current_);
        // the same when it is set manually
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 0), current_);
      }
      {
        // mix of live traffic and default edge speed
        auto flow_mask = baldr::kCurrentFlowMask;
        // 6 minutes
        EXPECT_NEAR(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 360),
                    max_speed_ * 0.1 + current_ * 0.9, 1);
        // 40 minutes
        EXPECT_NEAR(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 2400),
                    max_speed_ * 0.666 + current_ * 0.333, 1);
        // 1 hour, no live traffic is used
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 3600), max_speed_);
      }
      {
        // mix of live traffic and predicted
        auto flow_mask = baldr::kCurrentFlowMask | baldr::kPredictedFlowMask;
        // live traffic only is used for current time
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 0), current_);
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 1000, false, nullptr, 0), current_);
        // 6 minutes
        EXPECT_NEAR(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 360),
                    historical_max_ * 0.1 + current_ * 0.9, 1);
        // 40 minutes
        EXPECT_NEAR(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 2400),
                    historical_max_ * 0.666 + current_ * 0.333 + 0.1, 1);
        // 1 hour
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 3600), historical_max_);
      }
      {
        // current and constrained mix
        auto flow_mask = baldr::kCurrentFlowMask | baldr::kConstrainedFlowMask;
        EXPECT_NEAR(tile->GetSpeed(&e, flow_mask, 60 * 60 * 12, false, nullptr, 360),
                    constrained_ * 0.1 + current_ * 0.9, 1);
        // current and freeflow mix
        flow_mask = baldr::kCurrentFlowMask | baldr::kFreeFlowMask;
        EXPECT_NEAR(tile->GetSpeed(&e, flow_mask, 60 * 60 * 19, false, nullptr, 360),
                    freeflow_ * 0.1 + current_ * 0.9, 1);
        // current flow is not allowed, predicted traffic is used for current time instead
        EXPECT_EQ(tile->GetSpeed(&e, baldr::kPredictedFlowMask, 0), historical_max_);
      }
    }
  }
}

TEST_F(SmallRouteSpeedTest, PredictedTraffic) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  for (auto tile_id : reader->GetTileSet()) {
    auto tile = reader->GetGraphTile(tile_id);
    for (const auto& e : tile->GetDirectedEdges()) {
      {
        // historical speed changes over time
        auto flow_mask = baldr::kPredictedFlowMask;
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 100), historical_max_);
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 3 * 24 * 60 * 60), historical_max_); // Wednesday 12pm
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 60 * 60 * 5), 26);                   // Monday 5am
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 60 * 60 * 7), 24);                   // Monday 7am
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 60 * 60 * 13), 20);                  // Monday 1pm
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 60 * 60 * 19), 26);                  // Monday 7pm
      }

      {
        // mix of predicted and live speeds
        int evening_time = 19 * 60 * 60, evening_speed = 26;
        auto flow_mask = baldr::kPredictedFlowMask | baldr::kCurrentFlowMask;
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, evening_time, false, nullptr, 0), current_);
        // 7 pm is in more than 1 hour
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, evening_time, false, nullptr, 3700), evening_speed);
        // 7 pm is in 6 minutes
        EXPECT_NEAR(tile->GetSpeed(&e, flow_mask, evening_time, false, nullptr, 360),
                    evening_speed * 0.1 + current_ * 0.9, 1);
      }
      {
        // predicted flow is not set, predicted speed is not used
        EXPECT_EQ(tile->GetSpeed(&e), constrained_);
        EXPECT_EQ(tile->GetSpeed(&e, baldr::kConstrainedFlowMask), constrained_);
        EXPECT_EQ(tile->GetSpeed(&e, baldr::kFreeFlowMask), freeflow_);
      }
    }
  }
}

TEST_F(SmallRouteSpeedTest, ConstrainedAndFreeFlowTraffic) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  for (auto tile_id : reader->GetTileSet()) {
    auto tile = reader->GetGraphTile(tile_id);
    for (const auto& e : tile->GetDirectedEdges()) {
      {
        // basic constrained
        auto flow_mask = baldr::kConstrainedFlowMask;
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 60 * 60 * 12), constrained_);
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0), max_speed_);

        // basic freeflow
        flow_mask = baldr::kFreeFlowMask;
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 60 * 60 * 12), max_speed_);
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0), freeflow_);
      }
      {
        // both constrained and freeflow allowed
        auto flow_mask = baldr::kConstrainedFlowMask | baldr::kFreeFlowMask;
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 60 * 60 * 12), constrained_);
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0), freeflow_);
      }
      {
        // constrained and freeflow are not set
        EXPECT_NE(tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0), constrained_);
        EXPECT_NE(tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0), freeflow_);
        EXPECT_NE(tile->GetSpeed(&e, baldr::kPredictedFlowMask, 0), constrained_);
        EXPECT_NE(tile->GetSpeed(&e, baldr::kPredictedFlowMask, 0), freeflow_);
      }
    }
  }
}

std::string date_time_in_N_minutes(const int minutes) {
  sc::system_clock::duration dtn((sc::duration<int, std::ratio<60>>(minutes)));
  int default_timezone_index = baldr::DateTime::get_tz_db().to_index("Etc/UTC");
  const auto* tz = dt::get_tz_db().from_index(default_timezone_index);
  auto now_date =
      date::make_zoned(tz, sc::time_point_cast<sc::seconds>(
                               sc::time_point_cast<sc::minutes>(sc::system_clock::now() + dtn)));
  std::ostringstream iso_dt;
  iso_dt << date::format("%FT%R", now_date);
  return iso_dt.str();
}

double calculate_eta(const valhalla::Api& result) {
  EXPECT_EQ(result.trip().routes_size(), 1);
  double eta_sec = 0;
  for (const auto& leg : result.directions().routes(0).legs()) {
    eta_sec += leg.summary().time();
  }
  return eta_sec;
}

std::string make_route_request(const gurka::map& map,
                               const std::string& from,
                               const std::string& to,
                               std::string speed_types,
                               bool prioritize_bidirectional,
                               int date_time_type,
                               std::string date_time_value) {
  const std::string query_pattern_with_speeds = R"({
      "locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],
      "costing": "auto",
      "costing_options":{"auto":{"speed_types":[%s]}},
      "prioritize_bidirectional":%s,
      "date_time": { "type": "%d", "value": "%s" }
    })";
  return (boost::format(query_pattern_with_speeds) % std::to_string(map.nodes.at(from).lat()) %
          std::to_string(map.nodes.at(from).lng()) % std::to_string(map.nodes.at(to).lat()) %
          std::to_string(map.nodes.at(to).lng()) % speed_types %
          std::to_string(prioritize_bidirectional) % date_time_type % date_time_value)
      .str();
}

class RouteWithTraffic : public ::testing::Test {
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
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/route_traffic_depends_on_time");
    map.config.put("mjolnir.traffic_extract", "test/data/route_traffic_depends_on_time/traffic.tar");
    set_traffic(map);
  }

  static gurka::map map;
};

gurka::map RouteWithTraffic::map = {};

TEST_F(RouteWithTraffic, OneEdgeCurrentTime) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\",\"current\"";

  std::vector<valhalla::Api> results;
  // invariant
  results.emplace_back(
      gurka::do_action(valhalla::Options::route, map,
                       make_route_request(map, "A", "B", speeds, false, 3, "current")));
  results.emplace_back(
      gurka::do_action(valhalla::Options::route, map,
                       make_route_request(map, "A", "B", speeds, true, 3, "current")));
  // depart_at = now
  results.emplace_back(
      gurka::do_action(valhalla::Options::route, map,
                       make_route_request(map, "A", "B", speeds, false, 0, "current")));
  results.emplace_back(
      gurka::do_action(valhalla::Options::route, map,
                       make_route_request(map, "A", "B", speeds, true, 0, "current")));
  // additional check, date_time_value should be ignored when type=0 (current)
  results.emplace_back(
      gurka::do_action(valhalla::Options::route, map,
                       make_route_request(map, "A", "B", speeds, false, 0, "2021-11-08T00:00")));
  // depart_at = current_date_time
  auto date_time = date_time_in_N_minutes(0);
  results.emplace_back(
      gurka::do_action(valhalla::Options::route, map,
                       make_route_request(map, "A", "B", speeds, false, 1, date_time)));
  results.emplace_back(
      gurka::do_action(valhalla::Options::route, map,
                       make_route_request(map, "A", "B", speeds, true, 1, date_time)));

  // arrive_by = current_date_time
  results.emplace_back(
      gurka::do_action(valhalla::Options::route, map,
                       make_route_request(map, "A", "B", speeds, false, 2, date_time)));
  results.emplace_back(
      gurka::do_action(valhalla::Options::route, map,
                       make_route_request(map, "A", "B", speeds, true, 2, date_time)));
  for (const auto& result : results) {
    gurka::assert::raw::expect_path_length(result, 50, 1);
    // live traffic is used whenever current time is specified
    // error margin is set to 70, because live speed can be mixed with predicted
    // when a new minute starts, in this case edge speed can become 51 instead of 52.
    gurka::assert::raw::expect_eta(result, 3461, 70);
  }
}

TEST_F(RouteWithTraffic, ReverseAstarHasTimeDependentTraffic) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\",\"current\"";

  for (bool prioritize_bidirectional : {true, false}) {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_route_request(map, "A", "D", speeds, prioritize_bidirectional,
                                                      2, "2021-11-08T00:00"));
    ASSERT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_reverse_a*");
    // the last edge has speed of 31, next has speed 28 km/h, the first edge - 26km/h
    gurka::assert::raw::expect_eta(result, 21605, 30);
  }
}

TEST_F(RouteWithTraffic, AlgoPrioritizationWithDepartAt) {
  std::string speeds = "\"predicted\",\"current\"";

  auto result =
      gurka::do_action(valhalla::Options::route, map,
                       make_route_request(map, "A", "D", speeds, false, 1, "2021-11-08T00:00"));
  ASSERT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  auto time_dependent_eta = calculate_eta(result);

  result = gurka::do_action(valhalla::Options::route, map,
                            make_route_request(map, "A", "D", speeds, true, 1, "2021-11-08T00:00"));
  ASSERT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
  auto bidirectional_eta = calculate_eta(result);

  // ETAs calculated by two algos should be equal
  EXPECT_NEAR(time_dependent_eta, bidirectional_eta, 0.1);

  // live is not used. One edge should have predicted speed 31, the second has predicted speed 30
  // km/h, the last edge's speed is 28.
  EXPECT_NEAR(bidirectional_eta, 20720, 1);
}

TEST_F(RouteWithTraffic, InvariantStaysInvariant) {
  std::string speeds = "\"predicted\"";

  {
    bool prioritize_bidirectional = true;
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_route_request(map, "A", "D", speeds, prioritize_bidirectional,
                                                      1, "2021-11-08T00:00"));
    ASSERT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    // one should have predicted speed 31, the second has predicted speed 30, next speed is 28.
    gurka::assert::raw::expect_eta(result, 20720, 30);
  }

  {
    // invariant time
    auto result =
        gurka::do_action(valhalla::Options::route, map,
                         make_route_request(map, "A", "D", speeds, true, 3, "2021-11-08T00:00"));
    ASSERT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    // all edges have current speed - 31
    gurka::assert::raw::expect_eta(result, 19741, 10);
  }
}

TEST_F(RouteWithTraffic, LiveOnButNotUsed) {
  std::vector<int> minutes_from_now{
      -60 * 24 * 7, // week before
      -60 * 7,      // 7 hours before
      70,           // 1 hr 10 minutes
      2 * 60,       // in 2 hours
      60 * 24 * 3   // in 3 days
  };

  for (int minute : minutes_from_now) {
    auto date_time = date_time_in_N_minutes(minute);
    std::vector<std::string> speed_sets = {"\"predicted\"", "\"predicted\",\"current\""};
    std::vector<double> etas;
    for (const auto& speeds : speed_sets) {
      for (bool prioritize_bidirectional : {true, false}) {
        auto result = gurka::do_action(valhalla::Options::route, map,
                                       make_route_request(map, "A", "D", speeds,
                                                          prioritize_bidirectional, 1, date_time));
        if (prioritize_bidirectional) {
          EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
        } else {
          EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
        }
        etas.emplace_back(calculate_eta(result));
      }
    }
    for (int i = 0; i < etas.size() - 1; ++i) {
      // Live traffic is not used, algos should return the same ETA.
      EXPECT_EQ(etas[i], etas[i + 1]);
    }
  }
}

TEST_F(RouteWithTraffic, InvariantLiveOnButNotUsed) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\",\"current\"";

  auto result =
      gurka::do_action(valhalla::Options::route, map,
                       make_route_request(map, "A", "D", speeds, false, 3, "2021-11-08T00:00"));
  EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
  gurka::assert::raw::expect_path_length(result, 170, 0.1);
  // live is not used. predicted speed 31 km/h everywhere.
  gurka::assert::raw::expect_eta(result, 19741, 1);
}

TEST_F(RouteWithTraffic, LiveTrafficUsedIntheNearestHour) {
  // live traffic is used, even when time is not exactly current
  std::vector<std::string> date_times = {
      date_time_in_N_minutes(6),  // 6 minutes from now, ~90% of the first edge has live speed.
      date_time_in_N_minutes(-90) // 90 minutes ago, second edge is affected by current speed.
  };

  for (const auto& date_time : date_times) {
    for (int date_time_type : {1, 3}) {
      for (bool prioritize_bidirectional : {true, false}) {
        std::string speeds = "\"predicted\"";
        auto predicted_result =
            gurka::do_action(valhalla::Options::route, map,
                             make_route_request(map, "A", "D", speeds, prioritize_bidirectional,
                                                date_time_type, date_time));
        const double predicted_eta = calculate_eta(predicted_result);

        speeds += ",\"current\"";
        auto live_result =
            gurka::do_action(valhalla::Options::route, map,
                             make_route_request(map, "A", "D", speeds, prioritize_bidirectional,
                                                date_time_type, date_time));
        const double live_eta = calculate_eta(live_result);

        // live traffic is used, it is much faster then predicted
        EXPECT_LE(live_eta, predicted_eta);
      }
    }
  }
}

TEST_F(RouteWithTraffic, PredictedTrafficChangesOverTime) {
  std::string speeds = "\"predicted\"";

  for (bool prioritize_bidirectional : {
           true, // bidirectional part, recosting
           false // time dependent algo
       }) {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_route_request(map, "A", "D", speeds, prioritize_bidirectional,
                                                      1, "2021-11-08T00:00"));
    // one should have predicted speed 31, the second has predicted speed 30, next speed is 28.
    gurka::assert::raw::expect_eta(result, 20720, 1);
  }

  std::vector<std::pair<std::string, int>> time_to_eta{
      {"2021-11-08T00:00", 20720},
      {"2021-11-08T12:00", 28759},
      {"2021-11-08T18:00", 22948},
  };
  for (const auto& date_time_eta : time_to_eta) {
    for (bool prioritize_bidirectional : {true, false}) {
      auto result =
          gurka::do_action(valhalla::Options::route, map,
                           make_route_request(map, "A", "D", speeds, prioritize_bidirectional, 1,
                                              date_time_eta.first));
      gurka::assert::raw::expect_eta(result, date_time_eta.second, 1);
    }
  }
}

TEST_F(RouteWithTraffic, LiveAndPredictedUsedAtCurrentTime) {
  std::string speeds = "\"predicted\",\"current\"";

  for (int date_time_type : {0, 1, 2}) {
    for (bool prioritize_bidirectional : {true, false}) {
      auto result =
          gurka::do_action(valhalla::Options::route, map,
                           make_route_request(map, "A", "D", speeds, prioritize_bidirectional,
                                              date_time_type, "current"));
      // the first edge has live speed (52), second has a mix of live (52) and predicted
      // traffic (smth between 19 and 31), last has only predicted traffic (smth between 19 and 31).
      gurka::assert::raw::expect_eta(result, 21211, 5500);
    }
  }

  {
    // invariant time
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_route_request(map, "A", "D", speeds, true, 3, "current"));
    // all edges have current live speed - 52
    gurka::assert::raw::expect_eta(result, 11769, 10);
  }
}

TEST_F(RouteWithTraffic, ConstrainedAndFreeflowTraffic) {
  std::string speeds = "\"freeflow\",\"constrained\"";

  std::vector<std::pair<std::string, int>> time_to_eta_with_recosting{
      {"2021-11-08T00:00", 6120},  // freeflow
      {"2021-11-08T12:00", 15300}, // constrained
      {"2021-11-08T06:00", 9360},  // mixed, 2 freeflow (night), 1 constrained (day)
      {"2021-11-08T18:30", 8820},  // mixed, 1 constrained (day), 2 freeflow (night)
      {"2021-11-08T17:00", 12060}, // mixed, 2 constrained (day), 1 freeflow (night)
  };

  for (const auto& date_time_eta : time_to_eta_with_recosting) {
    auto result =
        gurka::do_action(valhalla::Options::route, map,
                         make_route_request(map, "A", "D", speeds, true, 1, date_time_eta.first));
    gurka::assert::raw::expect_eta(result, date_time_eta.second, 1);
  }

  std::vector<std::pair<std::string, int>> time_to_eta_invariant{
      {"2021-11-08T06:00", 6120},  // no recosting -> freeflow everywhere
      {"2021-11-08T18:30", 15300}, // no recosting -> constrained everywhere
      {"2021-11-08T17:00", 15300}, // no recosting -> constrained everywhere
  };
  for (const auto& date_time_eta : time_to_eta_invariant) {
    auto result =
        gurka::do_action(valhalla::Options::route, map,
                         make_route_request(map, "A", "D", speeds, true, 3, date_time_eta.first));
    gurka::assert::raw::expect_eta(result, date_time_eta.second, 1);
  }
}

std::string make_mapmatch_request(const gurka::map& map,
                                  const std::string& from,
                                  const std::string& to,
                                  std::string speed_types,
                                  std::string date_time_value) {
  const std::string query_pattern_with_speeds = R"({
      "shape":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],
      "costing": "auto",
      "costing_options":{"auto":{"speed_types":[%s]}},
      "date_time":{"value":"%s","type":"1"}
    })";
  return (boost::format(query_pattern_with_speeds) % std::to_string(map.nodes.at(from).lat()) %
          std::to_string(map.nodes.at(from).lng()) % std::to_string(map.nodes.at(to).lat()) %
          std::to_string(map.nodes.at(to).lng()) % speed_types % date_time_value)
      .str();
}

class MapMatchWithTraffic : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
      A----B-----C-------D
    )";

    const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                              {"BC", {{"highway", "primary"}}},
                              {"CD", {{"highway", "primary"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/mapmatch_traffic_depends_on_time",
                            {
                                {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"},
                            });
    map.config.put("mjolnir.traffic_extract",
                   "test/data/mapmatch_traffic_depends_on_time/traffic.tar");
    test::build_live_traffic_data(map.config);
    test::customize_live_traffic_data(map.config, [&](baldr::GraphReader&, baldr::TrafficTile&, int,
                                                      valhalla::baldr::TrafficSpeed* traffic_speed) {
      traffic_speed->overall_encoded_speed = 2 >> 1;
      traffic_speed->encoded_speed1 = 2 >> 1;
      traffic_speed->breakpoint1 = 255;
    });

    test::customize_historical_traffic(map.config, [](DirectedEdge& e) {
      // speeds for every 5 min bucket of the week
      std::array<float, kBucketsPerWeek> historical;
      historical.fill(6);
      return historical;
    });
  }

  static gurka::map map;
};

gurka::map MapMatchWithTraffic::map = {};

TEST_F(MapMatchWithTraffic, OneEdgeLiveTrafficETA) {
  std::string speeds = "\"predicted\",\"current\"";
  auto result = gurka::do_action(valhalla::Options::trace_route, map,
                                 make_mapmatch_request(map, "A", "B", speeds, "current"));
  gurka::assert::raw::expect_path_length(result, 0.5, 0.1);
  // live traffic only is used (2km/h)
  gurka::assert::raw::expect_eta(result, 900, 0.01);
}

TEST_F(MapMatchWithTraffic, LiveMixedWithPredictedOnTheFirstEdge) {
  std::string speeds = "\"predicted\",\"current\"";
  auto date_time = date_time_in_N_minutes(40);
  auto result = gurka::do_action(valhalla::Options::trace_route, map,
                                 make_mapmatch_request(map, "A", "B", speeds, date_time));
  gurka::assert::raw::expect_path_length(result, 0.5, 0.1);
  // 66% of predicted, 33% of live traffic
  gurka::assert::raw::expect_eta(result, 450, 0.01);
}

TEST_F(MapMatchWithTraffic, PredictedUsedAfterALongTimeOfTheRoute) {
  std::string speeds = "\"predicted\",\"current\"";

  auto result = gurka::do_action(valhalla::Options::trace_route, map,
                                 make_mapmatch_request(map, "A", "D", speeds, "current"));
  gurka::assert::raw::expect_path_length(result, 1.9, 0.01);
  // live traffic is mixed with predicted on the last edges
  gurka::assert::raw::expect_eta(result, 2340, 1);
}

TEST_F(MapMatchWithTraffic, PredictedUsedIn10Hours) {
  std::string speeds = "\"predicted\",\"current\"";

  auto date_time = date_time_in_N_minutes(60 * 10);
  auto result = gurka::do_action(valhalla::Options::trace_route, map,
                                 make_mapmatch_request(map, "A", "D", speeds, date_time));
  // predicted only is used (6km/h)
  gurka::assert::raw::expect_eta(result, 1140, 1);
}

TEST_F(MapMatchWithTraffic, DateTimeSnap) {
  std::string speeds = "\"predicted\",\"current\"";
  // live traffic is used in different proportions on the edges, because of time
  {
    auto date_time = date_time_in_N_minutes(30);
    auto result = gurka::do_action(valhalla::Options::trace_route, map,
                                   make_mapmatch_request(map, "A", "D", speeds, date_time));
    gurka::assert::raw::expect_eta(result, 1566, 1);
  }

  {
    auto date_time = date_time_in_N_minutes(-30);
    auto result = gurka::do_action(valhalla::Options::trace_route, map,
                                   make_mapmatch_request(map, "A", "D", speeds, date_time));
    // live traffic is used more often (2 km/h instead of 6km/h for predicted)
    gurka::assert::raw::expect_eta(result, 2610, 1);
  }
}
