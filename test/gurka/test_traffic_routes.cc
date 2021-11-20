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

const float calculate_eta(const valhalla::Api& result) {
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

  std::string make_request(std::string from,
                           std::string to,
                           std::string speed_types,
                           bool invariant_postprocess,
                           int date_time_type,
                           std::string date_time_value) const {
    const std::string query_pattern_with_speeds = R"({
      "locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],
      "costing": "auto",
      "costing_options":{"auto":{"speed_types":[%s]}},
      "invariant_postprocess":%s,
      "date_time": { "type": "%d", "value": "%s" }
    })";
    return (boost::format(query_pattern_with_speeds) % std::to_string(map.nodes.at(from).lat()) %
            std::to_string(map.nodes.at(from).lng()) % std::to_string(map.nodes.at(to).lat()) %
            std::to_string(map.nodes.at(to).lng()) % speed_types %
            std::to_string(invariant_postprocess) % date_time_type % date_time_value)
        .str();
  }

  static gurka::map map;
  static uint32_t current, historical, constrained, freeflow;
};

gurka::map TrafficSmoothing::map = {};
uint32_t TrafficSmoothing::current = 0, TrafficSmoothing::historical = 0,
         TrafficSmoothing::constrained = 0, TrafficSmoothing::freeflow = 0;

TEST_F(TrafficSmoothing, LiveTrafficOneEdge) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\",\"current\"";

  auto result = gurka::do_action(valhalla::Options::route, map,
                                 make_request("A", "B", speeds, false, 3, "current"));
  EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  gurka::assert::raw::expect_path_length(result, 50, 0.1);
  gurka::assert::raw::expect_eta(result, 3461, 10);
}

TEST_F(TrafficSmoothing, PredictedTrafficOneEdge) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";

  auto result = gurka::do_action(valhalla::Options::route, map,
                                 make_request("A", "B", speeds, false, 3, "2021-11-08T19:00"));
  EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  gurka::assert::raw::expect_path_length(result, 50, 0.1);
  gurka::assert::raw::expect_eta(result, 7050, 300);

  result = gurka::do_action(valhalla::Options::route, map,
                            make_request("A", "B", speeds, false, 3, "2021-11-08T00:00"));
  EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  gurka::assert::raw::expect_path_length(result, 50, 0.1);
  gurka::assert::raw::expect_eta(result, 5900, 200);
}

TEST_F(TrafficSmoothing, LiveMixedWithPredictive) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\",\"current\"";

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "C", speeds, false, 3, "current"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
    gurka::assert::raw::expect_path_length(result, 110, 0.1);
    // one should have live traffic, the second mixes live and predictive.
    // The error margin is huge because of predictive traffic value range.
    // The value should not be close to only live-traffic time.
    gurka::assert::raw::expect_eta(result, 12650, 3500);
  }

  {
    // the same test but date_time_type is 0 (aka depart_at=now)
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "C", speeds, false, 0, "current"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
    gurka::assert::raw::expect_path_length(result, 110, 0.1);
    gurka::assert::raw::expect_eta(result, 12650, 3500);
  }
}

TEST_F(TrafficSmoothing, LiveOnButTimeIsOld) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\",\"current\"";

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "C", speeds, false, 3, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
    gurka::assert::raw::expect_path_length(result, 110, 0.1);
    // one should have predictive speed 31, the second has predictive speed 30 km/h.
    gurka::assert::raw::expect_eta(result, 13006, 1);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "C", speeds, true, 3, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
    gurka::assert::raw::expect_path_length(result, 110, 0.1);
    // one should have predictive speed 31, the second has predictive speed 30 km/h.
    gurka::assert::raw::expect_eta(result, 13006, 1);
  }
}

TEST_F(TrafficSmoothing, LiveSmoothing10MinutesBefore) {
  auto date_time = date_time_in_N_minutes(-10);
  {
    std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";
    auto predicted_result = gurka::do_action(valhalla::Options::route, map,
                                             make_request("A", "C", speeds, false, 3, date_time));
    const float predicted_eta = calculate_eta(predicted_result);

    speeds += ",\"current\"";
    auto live_result = gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "C", speeds, false, 3, date_time));
    const float live_eta = calculate_eta(live_result);

    // predicted traffic is used on the first edge. Route through it will take ~1.36, live traffic
    // will not be used
    EXPECT_EQ(predicted_eta, live_eta);
  }
}

TEST_F(TrafficSmoothing, LiveSmoothing30MinutesBefore) {
  auto date_time = date_time_in_N_minutes(-30);
  {
    std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";
    auto predicted_result = gurka::do_action(valhalla::Options::route, map,
                                             make_request("A", "C", speeds, false, 3, date_time));
    const float predicted_eta = calculate_eta(predicted_result);
    speeds += ",\"current\"";
    auto live_result = gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "C", speeds, false, 3, date_time));
    const float live_eta = calculate_eta(live_result);

    // predicted traffic is used on the first edge. Route through it will take ~1.36, live traffic
    // will be used with 10% multiplier on the second edge.
    EXPECT_LE(live_eta, predicted_eta);
    EXPECT_NEAR(live_eta, predicted_eta, 1700);
  }
}

TEST_F(TrafficSmoothing, LiveSmoothingWeekBefore) {
  auto date_time = date_time_in_N_minutes(-60 * 24 * 7);
  {
    std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";
    auto predicted_result = gurka::do_action(valhalla::Options::route, map,
                                             make_request("A", "C", speeds, false, 3, date_time));
    const float predicted_eta = calculate_eta(predicted_result);

    speeds += ",\"current\"";
    auto live_result = gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "C", speeds, false, 3, date_time));
    const float live_eta = calculate_eta(live_result);

    // too long time ago, live traffic doesn't apply to the time.
    EXPECT_EQ(predicted_eta, live_eta);
  }
}

TEST_F(TrafficSmoothing, CurrentVsDateTime) {
  auto date_time = date_time_in_N_minutes(0);
  {
    std::string speeds = "\"freeflow\",\"constrained\",\"predicted\",\"current\"";
    auto date_time_result = gurka::do_action(valhalla::Options::route, map,
                                             make_request("A", "C", speeds, false, 3, date_time));
    const float date_time_eta = calculate_eta(date_time_result);

    auto current_result = gurka::do_action(valhalla::Options::route, map,
                                           make_request("A", "C", speeds, false, 3, "current"));
    const float current_eta = calculate_eta(current_result);

    // too long time ago, live traffic doesn't apply to the time.
    EXPECT_EQ(date_time_eta, current_eta);
  }
}

TEST_F(TrafficSmoothing, LiveSmoothingIn10Minutes) {
  auto date_time = date_time_in_N_minutes(10);
  {
    std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";
    auto predicted_result = gurka::do_action(valhalla::Options::route, map,
                                             make_request("A", "B", speeds, false, 3, date_time));
    const float predicted_eta = calculate_eta(predicted_result);

    speeds += ",\"current\"";
    auto live_result = gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "B", speeds, false, 3, date_time));
    const float live_eta = calculate_eta(live_result);

    // 90% of live-traffic is taken on the edge.
    EXPECT_LE(live_eta, predicted_eta);
    EXPECT_NEAR(predicted_eta / live_eta, 2.0, 0.5);
  }
}

TEST_F(TrafficSmoothing, LiveSmoothingIn2Hours) {
  auto date_time = date_time_in_N_minutes(2 * 60);
  {
    std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";
    auto predicted_result = gurka::do_action(valhalla::Options::route, map,
                                             make_request("A", "C", speeds, false, 3, date_time));
    const float predicted_eta = calculate_eta(predicted_result);

    speeds += ",\"current\"";
    auto live_result = gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "C", speeds, false, 3, date_time));
    const float live_eta = calculate_eta(live_result);

    // live traffic is not used in 2 hours
    EXPECT_EQ(predicted_eta, live_eta);
  }
}

TEST_F(TrafficSmoothing, LiveSmoothingIn3Days) {
  auto date_time = date_time_in_N_minutes(60 * 24 * 3);
  {
    std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";
    auto predicted_result = gurka::do_action(valhalla::Options::route, map,
                                             make_request("A", "C", speeds, false, 3, date_time));
    const float predicted_eta = calculate_eta(predicted_result);

    speeds += ",\"current\"";
    auto live_result = gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "C", speeds, false, 3, date_time));
    const float live_eta = calculate_eta(live_result);

    // live traffic is not used in 3 days
    EXPECT_EQ(predicted_eta, live_eta);
  }
}

TEST_F(TrafficSmoothing, LiveSmoothingIn3DaysBidir) {
  auto date_time = date_time_in_N_minutes(60 * 24 * 3);
  {
    std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";
    auto predicted_result = gurka::do_action(valhalla::Options::route, map,
                                             make_request("A", "D", speeds, false, 3, date_time));
    const float predicted_eta = calculate_eta(predicted_result);

    speeds += ",\"current\"";
    auto live_result = gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "D", speeds, false, 3, date_time));
    const float live_eta = calculate_eta(live_result);

    // live traffic is not used in 3 days
    EXPECT_EQ(predicted_eta, live_eta);
  }
}

TEST_F(TrafficSmoothing, LiveSmoothing6HoursBeforeBidir) {
  auto date_time = date_time_in_N_minutes(-60 * 60 * 6);
  {
    std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";
    auto predicted_result = gurka::do_action(valhalla::Options::route, map,
                                             make_request("A", "D", speeds, false, 3, date_time));
    const float predicted_eta = calculate_eta(predicted_result);

    speeds += ",\"current\"";
    auto live_result = gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "D", speeds, false, 3, date_time));
    const float live_eta = calculate_eta(live_result);

    EXPECT_EQ(predicted_eta, live_eta);
  }
}

TEST_F(TrafficSmoothing, LiveSmoothingIn90MinutesBidir) {
  auto date_time = date_time_in_N_minutes(90);
  {
    std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";
    auto predicted_result = gurka::do_action(valhalla::Options::route, map,
                                             make_request("A", "D", speeds, false, 3, date_time));
    const float predicted_eta = calculate_eta(predicted_result);

    speeds += ",\"current\"";
    auto live_result = gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "D", speeds, false, 3, date_time));
    const float live_eta = calculate_eta(live_result);

    EXPECT_EQ(predicted_eta, live_eta);
  }
}

TEST_F(TrafficSmoothing, LiveSmoothingBidir) {
  auto date_time = date_time_in_N_minutes(6);
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";
  auto predicted_result = gurka::do_action(valhalla::Options::route, map,
                                           make_request("A", "D", speeds, false, 3, date_time));
  const float predicted_eta = calculate_eta(predicted_result);

  speeds += ",\"current\"";
  auto live_result = gurka::do_action(valhalla::Options::route, map,
                                      make_request("A", "D", speeds, false, 3, date_time));
  const float live_eta = calculate_eta(live_result);

  auto earlier_date_time = date_time_in_N_minutes(-6);
  auto earlier_result = gurka::do_action(valhalla::Options::route, map,
                                         make_request("A", "D", speeds, false, 3, earlier_date_time));
  const float earlier_eta = calculate_eta(earlier_result);

  auto no_recosting_result = gurka::do_action(valhalla::Options::route, map,
                                              make_request("A", "D", speeds, true, 3, date_time));
  const float no_recosting_eta = calculate_eta(no_recosting_result);

  // live traffic is used on a vast majority of the first edge
  EXPECT_LE(live_eta, predicted_eta);
  // without recosting a huge live speed is used on the hole route
  EXPECT_LE(no_recosting_eta, live_eta);
  EXPECT_NE(live_eta, earlier_eta);
}

TEST_F(TrafficSmoothing, LiveSmoothing90MinutesBeforeBidir) {
  auto date_time = date_time_in_N_minutes(-90);
  {
    std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";
    auto predicted_result = gurka::do_action(valhalla::Options::route, map,
                                             make_request("A", "D", speeds, false, 3, date_time));
    const float predicted_eta = calculate_eta(predicted_result);

    speeds += ",\"current\"";
    auto live_result = gurka::do_action(valhalla::Options::route, map,
                                        make_request("A", "D", speeds, false, 3, date_time));
    const float live_eta = calculate_eta(live_result);

    auto later_date_time = date_time_in_N_minutes(90);
    auto later_result = gurka::do_action(valhalla::Options::route, map,
                                         make_request("A", "D", speeds, false, 3, later_date_time));
    const float later_eta = calculate_eta(later_result);

    auto no_recosting_result = gurka::do_action(valhalla::Options::route, map,
                                                make_request("A", "D", speeds, true, 3, date_time));
    const float no_recosting_eta = calculate_eta(no_recosting_result);

    // live traffic is used on a vast majority of the second edge
    EXPECT_LE(live_eta, predicted_eta);
    // without recosting a small predictive speed is used on the hole route
    EXPECT_LE(live_eta, no_recosting_eta);
    EXPECT_NE(later_eta, live_eta);
  }
}
TEST_F(TrafficSmoothing, PredictiveTrafficChangesOverTime) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "C", speeds, false, 3, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
    gurka::assert::raw::expect_path_length(result, 110, 0.1);
    // one should have predictive speed 31, the second has predictive speed 30 km/h.
    gurka::assert::raw::expect_eta(result, 13006, 1);
  }

  {
    // the same test but date_time_type is 1 (aka depart_at=2021-11-08T00:00)
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "C", speeds, false, 1, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
    gurka::assert::raw::expect_path_length(result, 110, 0.1);
    gurka::assert::raw::expect_eta(result, 13006, 1);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, false, 3, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // one should have predictive speed 31, the second has predictive speed 30, next speed is 28.
    gurka::assert::raw::expect_eta(result, 20720, 1);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, false, 3, "2021-11-08T12:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    gurka::assert::raw::expect_eta(result, 28759, 30);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, false, 3, "2021-11-08T18:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    gurka::assert::raw::expect_eta(result, 22948, 30);
  }
}

TEST_F(TrafficSmoothing, RecostingWithPredictiveTraffic) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, false, 3, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // one should have predictive speed 31, the second has predictive speed 30, next speed is 28.
    gurka::assert::raw::expect_eta(result, 20720, 30);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, true, 3, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // all edges have current speed - 31
    gurka::assert::raw::expect_eta(result, 19741, 10);
  }
}

TEST_F(TrafficSmoothing, LiveAndPredictiveMix) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\",\"current\"";

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, false, 3, "current"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // the first edge has live speed (52), second has a mix of live (52) and predictive
    // traffic (smth between 19 and 31), last has only predictive traffic (smth between 19 and 31).
    gurka::assert::raw::expect_eta(result, 21211, 5500);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, false, 1, "current"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // the first edge has live speed (52), second has a mix of live (52) and predictive
    // traffic (smth between 19 and 31), last has only predictive traffic (smth between 19 and 31).
    gurka::assert::raw::expect_eta(result, 21211, 5500);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, true, 3, "current"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // all edges have current live speed - 52
    gurka::assert::raw::expect_eta(result, 11769, 10);
  }
}

TEST_F(TrafficSmoothing, PredictiveAndForwardAstar) {
  std::string speeds = "\"freeflow\",\"constrained\",\"predicted\"";

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, false, 1, "2021-11-08T00:00"));
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
                                   make_request("A", "D", speeds, false, 3, "2021-11-08T00:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // freeflow
    gurka::assert::raw::expect_eta(result, 6120, 1);
  }
  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, false, 3, "2021-11-08T12:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // constrained
    gurka::assert::raw::expect_eta(result, 15300, 1);
  }
  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, false, 3, "2021-11-08T06:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // mixed, 2 freeflow (night), 1 constrained (day)
    gurka::assert::raw::expect_eta(result, 9360, 1);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, true, 3, "2021-11-08T06:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // no recosting -> freeflow everywhere
    gurka::assert::raw::expect_eta(result, 6120, 1);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, false, 3, "2021-11-08T18:30"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // mixed, 1 constrained (day), 2 freeflow (night)
    gurka::assert::raw::expect_eta(result, 8820, 1);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, true, 3, "2021-11-08T18:30"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // no recosting -> constrained everywhere
    gurka::assert::raw::expect_eta(result, 15300, 1);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, false, 3, "2021-11-08T17:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // mixed, 2 constrained (day), 1 freeflow (night)
    gurka::assert::raw::expect_eta(result, 12060, 1);
  }

  {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   make_request("A", "D", speeds, true, 3, "2021-11-08T17:00"));
    EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    gurka::assert::raw::expect_path_length(result, 170, 0.1);
    // no recosting -> constrained everywhere
    gurka::assert::raw::expect_eta(result, 15300, 1);
  }
}
