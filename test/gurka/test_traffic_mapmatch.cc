#include "gurka.h"
#include "test.h"

using namespace valhalla;

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

void set_traffic(gurka::map& map) {
  // add live traffic
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
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/mapmatch_uses_traffic",
                            {
                                {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"},
                            });
    map.config.put("mjolnir.traffic_extract", "test/data/mapmatch_uses_traffic/traffic.tar");
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
  // live traffic only is used
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
  // live traffic is mixed with predicted on the later edges
  gurka::assert::raw::expect_eta(result, 2340, 1);
}

TEST_F(MapMatchWithTraffic, PredictedUsedIn10Hours) {
  std::string speeds = "\"predicted\",\"current\"";

  auto date_time = date_time_in_N_minutes(60 * 10);
  auto result = gurka::do_action(valhalla::Options::trace_route, map,
                                 make_mapmatch_request(map, "A", "D", speeds, date_time));
  // predicted only is used. 6km/h
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
    // more path is passed with live traffic (2 km/h)
    gurka::assert::raw::expect_eta(result, 2610, 1);
  }
}
