#include "gurka.h"
#include "test.h"
#include <boost/format.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace valhalla;

void set_traffic(gurka::map& map, uint8_t traffic_edge_speed) {
  // add live traffic
  test::build_live_traffic_data(map.config);
  test::customize_live_traffic_data(map.config, [&](baldr::GraphReader&, baldr::TrafficTile&, int,
                                                    valhalla::baldr::TrafficSpeed* traffic_speed) {
    traffic_speed->overall_encoded_speed = traffic_edge_speed >> 1;
    traffic_speed->encoded_speed1 = traffic_edge_speed >> 1;
    traffic_speed->breakpoint1 = 255;
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
    // initialize Valhalla config singleton
    auto config = test::make_config("test/data/traffic_fading");
    std::stringstream ss;
    boost::property_tree::json_parser::write_json(ss, config);
    valhalla::config(ss.str());
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/traffic_fading");
    map.config.put("mjolnir.traffic_extract", "test/data/traffic_fading/traffic.tar");
    set_traffic(map, current_);
  }

  static gurka::map map;
  static uint32_t current_, max_speed_;
};

gurka::map SmallRouteSpeedTest::map = {};
uint32_t SmallRouteSpeedTest::current_ = 50, SmallRouteSpeedTest::max_speed_ = 100;

TEST_F(SmallRouteSpeedTest, FadingTraffic) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  auto flow_mask = baldr::kCurrentFlowMask;
  for (auto tile_id : reader->GetTileSet()) {
    auto tile = reader->GetGraphTile(tile_id);
    for (const auto& e : tile->GetDirectedEdges()) {
      {
        // default seconds_from_now value is 0, live traffic is used
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0), current_);
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 10000), current_);
        // the same when it is set manually
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 0), current_);
      }
      {
        // linear fading, start at 0m and fade over 1h
        // 6 minutes
        EXPECT_NEAR(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 360, 3600),
                    max_speed_ * 0.1 + current_ * 0.9, 1);
        // 40 minutes
        EXPECT_NEAR(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 2400, 3600),
                    max_speed_ * 2 / 3 + current_ * 1 / 3, 1);
        // 1 hour, no live traffic is used
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 3600, 3600), max_speed_);
      }
      {
        // linear fading, start at 30m and fade over 15m
        const uint64_t S = 30 * 60;
        const uint64_t D = 15 * 60;
        // 6 minutes
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 360, D, S), current_);
        // 40 minutes
        EXPECT_NEAR(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 2400, D, S),
                    max_speed_ * 0.666 + current_ * 0.333, 1);
        // 45 minutes, no live traffic is used
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 2700, D, S), max_speed_);
      }
      {
        // exponential fading, start at 1h and fade over 2h with exponent of 3
        const uint64_t S = 60 * 60;
        const uint64_t D = 2 * 60 * 60;
        const float E = 3;
        // 59 minutes
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 59 * 60, D, S, E), current_);
        // 2h
        EXPECT_NEAR(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 2 * 60 * 60, D, S, E),
                    max_speed_ * 0.125 + current_ * 0.875, 1);
        // 2h 30m
        EXPECT_NEAR(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 2.5 * 60 * 60, D, S, E),
                    max_speed_ * 0.421875 + current_ * 0.578125, 1);
        // 3h, no live traffic is used
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 3 * 60 * 60, D, S, E), max_speed_);
      }
      {
        // full traffic usage and then hard cut at 3h
        const uint64_t S = 3 * 60 * 60;
        const uint64_t D = 0;
        // 0 minutes
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 0, D, S), current_);
        // 1,5h
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 90 * 60, D, S), current_);
        // 3h, no live traffic is used
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, S, D, S), max_speed_);
      }
    }
  }
}