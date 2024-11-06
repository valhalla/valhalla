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
        EXPECT_NEAR(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 360, 3600),
                    max_speed_ * 0.1 + current_ * 0.9, 1);
        // 40 minutes
        EXPECT_NEAR(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 2400, 3600),
                    max_speed_ * 0.666 + current_ * 0.333, 1);
        // 1 hour, no live traffic is used
        EXPECT_EQ(tile->GetSpeed(&e, flow_mask, 0, false, nullptr, 3600, 3600), max_speed_);
      }
    }
  }
}