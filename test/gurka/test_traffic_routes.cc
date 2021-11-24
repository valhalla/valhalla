#include "gurka.h"
#include "test.h"

using namespace valhalla;

std::string date_time_in_N_minutes(const int minutes) {
  sc::system_clock::duration dtn((sc::duration<int, std::ratio<60>>(minutes)));
  int default_timezone_index = baldr::DateTime::get_tz_db().to_index("Etc/UTC");
  const auto* tz = dt::get_tz_db().from_index(default_timezone_index);
  const auto now_date =
      date::make_zoned(tz, sc::time_point_cast<sc::seconds>(
                               sc::time_point_cast<sc::minutes>(sc::system_clock::now() + dtn)));
  std::ostringstream iso_dt;
  iso_dt << date::format("%FT%R", now_date);
  return iso_dt.str();
}

float calculate_eta(const valhalla::Api& result) {
  EXPECT_EQ(result.trip().routes_size(), 1);
  double eta_sec = 0;
  for (const auto& leg : result.directions().routes(0).legs()) {
    eta_sec += leg.summary().time();
  }
  return eta_sec;
}

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

  std::string make_request(const std::string& from,
                           const std::string& to,
                           std::string speed_types,
                           bool prioritize_bidirectional,
                           int date_time_type,
                           std::string date_time_value) const {
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

  static gurka::map map;
};

gurka::map TrafficSmoothing::map = {};

TEST_F(TrafficSmoothing, OneEdgeCurrentTime) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\",\"current\"";

  std::vector<valhalla::Api> results;
  // invariant
  results.emplace_back(gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "B", speeds, false, 3, "current")));
  results.emplace_back(gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "B", speeds, true, 3, "current")));
  // depart_at = now
  results.emplace_back(gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "B", speeds, false, 0, "current")));
  results.emplace_back(gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "B", speeds, true, 0, "current")));
  // additional check, date_time_value should be ignored when type=0 (current)
  results.emplace_back(
      gurka::do_action(valhalla::Options::route, map,
                       make_request("A", "B", speeds, false, 0, "2021-11-08T00:00")));
  // depart_at = current_date_time
  auto date_time = date_time_in_N_minutes(0);
  results.emplace_back(gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "B", speeds, false, 1, date_time)));
  results.emplace_back(gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "B", speeds, true, 1, date_time)));
  for (auto result : results) {
    // time_dependent_forward_a* is used for one-edge routes.
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
    gurka::assert::raw::expect_path_length(result, 50, 0.1);
    // live traffic is used whenever current time is specified
    gurka::assert::raw::expect_eta(result, 3461, 10);
  }
}

TEST_F(TrafficSmoothing, ArriveByUsesReverseAlgo) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\",\"current\"";

  for (bool prioritize_bidirectional : {true, false}) {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, prioritize_bidirectional, 2,
                                                "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_reverse_a*");
  }
}

TEST_F(TrafficSmoothing, AlgoPrioritizationWithDepartAt) {
  std::string speeds = "\"predicted\",\"current\"";

  auto result = gurka::do_action(valhalla::Options::route, map,
                                 make_request("A", "D", speeds, false, 1, "2021-11-08T00:00"));
  EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  auto time_dependent_eta = calculate_eta(result);

  result = gurka::do_action(valhalla::Options::route, map,
                            make_request("A", "D", speeds, true, 1, "2021-11-08T00:00"));
  EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
  auto bidirectional_eta = calculate_eta(result);

  // ETA calculated by two algos should be equal
  EXPECT_EQ(time_dependent_eta, bidirectional_eta);

  // live is not used. One edge should have predictive speed 31, the second has predictive speed 30
  // km/h, the last edge's speed is 28.
  EXPECT_NEAR(bidirectional_eta, 20720, 1);
}

TEST_F(TrafficSmoothing, RecostingTrafficForBidirectionalAstar) {
  std::string speeds = "\"predicted\"";

  bool prioritize_bidirectional = true;
  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, prioritize_bidirectional, 1,
                                                "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    // one should have predictive speed 31, the second has predictive speed 30, next speed is 28.
    gurka::assert::raw::expect_eta(result, 20720, 30);
  }

  {
    // invariant time
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, prioritize_bidirectional, 3,
                                                "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    // all edges have current speed - 31
    gurka::assert::raw::expect_eta(result, 19741, 10);
  }
}

TEST_F(TrafficSmoothing, TimeOffsetIsSigned) {
  std::string speeds = "\"predicted\"";
  for (bool prioritize_bidirectional : {
           true, // bidirectional part, recosting
           false // time dependent algo
       }) {
    const float earlier_eta =
        calculate_eta(gurka::do_action(valhalla::Options::route, map,
                                       make_request("A", "D", speeds, prioritize_bidirectional, 1,
                                                    date_time_in_N_minutes(-90))));
    const float later_eta =
        calculate_eta(gurka::do_action(valhalla::Options::route, map,
                                       make_request("A", "D", speeds, prioritize_bidirectional, 1,
                                                    date_time_in_N_minutes(90))));
    EXPECT_NE(later_eta, earlier_eta);
  }
}

TEST_F(TrafficSmoothing, LiveOnButNotUsed) {
  std::vector<int> minutes_from_now{
      -60 * 24 * 7, // week before
      -60 * 7,      // 7 hours before
      -10, // 10 minutes before now. predicted traffic is used on the first edge, it will take ~1.36h
      70,  // 1 hr 10 minutes
      2 * 60,     // in 2 hours
      60 * 24 * 3 // in 3 days
  };

  for (int minute : minutes_from_now) {
    auto date_time = date_time_in_N_minutes(minute);
    std::vector<std::string> speed_sets = {"\"predicted\"", "\"predicted\",\"current\""};
    std::vector<float> etas;
    for (auto speeds : speed_sets) {
      for (bool prioritize_bidirectional : {true, false}) {
        auto result =
            gurka::do_action(valhalla::Options::route, map,
                             make_request("A", "D", speeds, prioritize_bidirectional, 1, date_time));
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

TEST_F(TrafficSmoothing, InvariantLiveOnButNotUsed) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\",\"current\"";

  auto result = gurka::do_action(valhalla::Options::route, map,
                                 make_request("A", "D", speeds, false, 3, "2021-11-08T00:00"));
  EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
  gurka::assert::raw::expect_path_length(result, 170, 0.1);
  // live is not used. predictive speed 31 km/h everywhere.
  gurka::assert::raw::expect_eta(result, 19741, 1);
}

TEST_F(TrafficSmoothing, LiveTrafficUsedIntheNearestHour) {
  // live traffic is used, even when time is not exactly current
  std::vector<std::string> date_times = {
      date_time_in_N_minutes(6),  // 6 minutes from now, ~90% of the first edge has live speed.
      date_time_in_N_minutes(-90) // 90 minutes ago, second edge is affected by current speed.
  };

  for (auto date_time : date_times) {
    for (int date_time_type : {1, 3}) {
      for (bool prioritize_bidirectional : {true, false}) {
        std::string speeds = "\"predicted\"";
        auto predicted_result =
            gurka::do_action(valhalla::Options::route, map,
                             make_request("A", "D", speeds, prioritize_bidirectional, date_time_type,
                                          date_time));
        const float predicted_eta = calculate_eta(predicted_result);

        speeds += ",\"current\"";
        auto live_result = gurka::do_action(valhalla::Options::route, map,
                                            make_request("A", "D", speeds, prioritize_bidirectional,
                                                         date_time_type, date_time));
        const float live_eta = calculate_eta(live_result);

        // live traffic is used, it is much faster then predicted
        EXPECT_LE(live_eta, predicted_eta);
      }
    }
  }
}

TEST_F(TrafficSmoothing, PredictiveTrafficChangesOverTime) {
  std::string speeds = "\"predicted\"";

  for (bool prioritize_bidirectional : {
           true, // bidirectional part, recosting
           false // time dependent algo
       }) {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, prioritize_bidirectional, 1,
                                                "2021-11-08T00:00"));
    // one should have predictive speed 31, the second has predictive speed 30, next speed is 28.
    gurka::assert::raw::expect_eta(result, 20720, 1);
  }

  std::vector<std::pair<std::string, int>> time_to_eta{
      {"2021-11-08T00:00", 20720},
      {"2021-11-08T12:00", 28759},
      {"2021-11-08T18:00", 22948},
  };
  for (auto [date_time, eta] : time_to_eta) {
    for (bool prioritize_bidirectional : {true, false}) {
      auto result =
          gurka::do_action(valhalla::Options::route, map,
                           make_request("A", "D", speeds, prioritize_bidirectional, 1, date_time));
      gurka::assert::raw::expect_eta(result, eta, 1);
    }
  }
}

TEST_F(TrafficSmoothing, LiveAndPredictiveUsedAtCurrentTime) {
  std::string speeds = "\"predicted\",\"current\"";

  for (int date_time_type : {0, 1}) {
    for (bool prioritize_bidirectional : {true, false}) {
      auto result = gurka::do_action(valhalla::Options::route, map,
                                     make_request("A", "D", speeds, prioritize_bidirectional,
                                                  date_time_type, "current"));
      // the first edge has live speed (52), second has a mix of live (52) and predictive
      // traffic (smth between 19 and 31), last has only predictive traffic (smth between 19 and 31).
      gurka::assert::raw::expect_eta(result, 21211, 5500);
    }
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, true, 3, "current"));
    // all edges have current live speed - 52
    gurka::assert::raw::expect_eta(result, 11769, 10);
  }
}

TEST_F(TrafficSmoothing, ConstrainedAndFreeflowTraffic) {
  std::string speeds = "\"freeflow\",\"constrained\"";

  std::vector<std::pair<std::string, int>> time_to_eta_with_recosting{
      {"2021-11-08T00:00", 6120},  // freeflow
      {"2021-11-08T12:00", 15300}, // constrained
      {"2021-11-08T06:00", 9360},  // mixed, 2 freeflow (night), 1 constrained (day)
      {"2021-11-08T18:30", 8820},  // mixed, 1 constrained (day), 2 freeflow (night)
      {"2021-11-08T17:00", 12060}, // mixed, 2 constrained (day), 1 freeflow (night)
  };

  for (auto [date_time, eta] : time_to_eta_with_recosting) {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, true, 1, date_time));
    gurka::assert::raw::expect_eta(result, eta, 1);
  }

  std::vector<std::pair<std::string, int>> time_to_eta_invariant{
      {"2021-11-08T06:00", 6120},  // no recosting -> freeflow everywhere
      {"2021-11-08T18:30", 15300}, // no recosting -> constrained everywhere
      {"2021-11-08T17:00", 15300}, // no recosting -> constrained everywhere
  };
  for (auto [date_time, eta] : time_to_eta_invariant) {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, true, 3, date_time));
    gurka::assert::raw::expect_eta(result, eta, 1);
  }
}
