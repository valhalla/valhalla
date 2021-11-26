#include "gurka.h"
#include "test.h"

using namespace valhalla;

class SmallRouteSpeedTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
      A----B
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}}},
    };
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10000);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/small_route_speeds",
                            {
                                {"mjolnir.shortcuts", "false"},
                                {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"},
                            });
    map.config.put("mjolnir.traffic_extract", "test/data/small_route_speeds/traffic.tar");

    // add live traffic
    test::build_live_traffic_data(map.config);
    current_ = 52;
    freeflow_ = 100;
    constrained_ = 40;
    historical_max_ = 31;
    historical_min_ = 19;
    test::customize_live_traffic_data(map.config, [&](baldr::GraphReader&, baldr::TrafficTile&, int,
                                                      valhalla::baldr::TrafficSpeed* traffic_speed) {
      traffic_speed->overall_encoded_speed = current_ >> 1;
      traffic_speed->encoded_speed1 = current_ >> 1;
      traffic_speed->breakpoint1 = 255;
    });

    test::customize_historical_traffic(map.config, [&](DirectedEdge& e) {
      e.set_constrained_flow_speed(constrained_);
      e.set_free_flow_speed(freeflow_);

      // speeds for every 5 min bucket of the week
      std::array<float, kBucketsPerWeek> historical;
      for (size_t i = 0; i < historical.size(); ++i) {
        size_t min_timestamp = (i % (24 * 12)) / 12;
        if (min_timestamp < 12) {
          historical[i] = historical_max_ - min_timestamp;
        } else {
          historical[i] = historical_min_ + (min_timestamp - 12);
        }
      }
      return historical;
    });
  }

  static gurka::map map;
  static uint32_t current_, historical_max_, historical_min_, constrained_, freeflow_;
};

gurka::map SmallRouteSpeedTest::map = {};
uint32_t SmallRouteSpeedTest::current_ = 0, SmallRouteSpeedTest::historical_max_ = 0,
         SmallRouteSpeedTest::historical_min_ = 0, SmallRouteSpeedTest::constrained_ = 0,
         SmallRouteSpeedTest::freeflow_ = 0;

TEST_F(SmallRouteSpeedTest, LiveTrafficSpeed) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  for (auto tile_id : reader->GetTileSet()) {
    auto tile = reader->GetGraphTile(tile_id);
    for (const auto& e : tile->GetDirectedEdges()) {
      auto speed = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0);
      EXPECT_EQ(speed, current_);

      speed = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 10000);
      EXPECT_EQ(speed, current_);

      uint8_t* flow_sources = nullptr;
      speed = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0, false, flow_sources, 0);
      EXPECT_EQ(speed, current_);

      int highway_speed = 75;

      speed = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0, false, flow_sources, 360);
      // mix of live traffic and default highway speed
      EXPECT_EQ(speed, int(highway_speed * 0.1 + current_ * 0.9));

      speed = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0, false, flow_sources, 2400);
      // mix of live traffic and default highway speed
      EXPECT_EQ(speed, int(highway_speed * 0.666 + current_ * 0.333));

      speed = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0, false, flow_sources, 3600);
      // default highway speed
      EXPECT_EQ(speed, highway_speed);

      speed = tile->GetSpeed(&e, baldr::kCurrentFlowMask | baldr::kPredictedFlowMask, 1000, false,
                             flow_sources, 0);
      EXPECT_EQ(speed, current_);

      speed = tile->GetSpeed(&e, baldr::kCurrentFlowMask | baldr::kPredictedFlowMask, 0, false,
                             flow_sources, 360);
      EXPECT_EQ(speed, int(historical_max_ * 0.1 + current_ * 0.9));

      speed = tile->GetSpeed(&e, baldr::kCurrentFlowMask | baldr::kPredictedFlowMask, 0, false,
                             flow_sources, 2400);
      EXPECT_EQ(speed, int(historical_max_ * 0.666 + current_ * 0.333 + 0.1));

      speed = tile->GetSpeed(&e, baldr::kCurrentFlowMask | baldr::kPredictedFlowMask, 0, false,
                             flow_sources, 1000);
      EXPECT_EQ(speed, int(historical_max_ * 0.277 + current_ * 0.722));

      speed = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 0);
      EXPECT_EQ(speed, historical_max_);
    }
  }
}

TEST_F(SmallRouteSpeedTest, HistoricalTrafficSpeed) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  for (auto tile_id : reader->GetTileSet()) {
    auto tile = reader->GetGraphTile(tile_id);
    for (const auto& e : tile->GetDirectedEdges()) {
      auto historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 100);
      EXPECT_EQ(historical, historical_max_);

      historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 3 * 24 * 60 * 60);
      EXPECT_EQ(historical, historical_max_);

      historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 60 * 60 * 5);
      EXPECT_EQ(historical, 26);

      historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 60 * 60 * 7);
      EXPECT_EQ(historical, 24);

      historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 60 * 60 * 13);
      EXPECT_EQ(historical, 20);

      int evening_speed = 26;

      historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 60 * 60 * 19);
      EXPECT_EQ(historical, evening_speed);

      uint8_t* flow_sources = nullptr;
      historical = tile->GetSpeed(&e, baldr::kCurrentFlowMask | baldr::kPredictedFlowMask,
                                  60 * 60 * 19, false, flow_sources, 0);
      EXPECT_EQ(historical, current_);

      historical = tile->GetSpeed(&e, baldr::kCurrentFlowMask | baldr::kPredictedFlowMask,
                                  60 * 60 * 19, false, flow_sources, 3700);
      EXPECT_EQ(historical, evening_speed);

      historical = tile->GetSpeed(&e, baldr::kCurrentFlowMask | baldr::kPredictedFlowMask,
                                  60 * 60 * 19, false, flow_sources, 360);
      EXPECT_EQ(historical, int(evening_speed * 0.1 + current_ * 0.9));

      historical = tile->GetSpeed(&e, baldr::kCurrentFlowMask | baldr::kConstrainedFlowMask,
                                  60 * 60 * 12, false, flow_sources, 360);
      EXPECT_EQ(historical, int(constrained_ * 0.1 + current_ * 0.9));

      historical = tile->GetSpeed(&e, baldr::kCurrentFlowMask | baldr::kFreeFlowMask, 60 * 60 * 19,
                                  false, flow_sources, 360);
      EXPECT_EQ(historical, int(freeflow_ * 0.1 + current_ * 0.9));

      historical = tile->GetSpeed(&e, baldr::kFreeFlowMask, 60 * 60 * 19);
      EXPECT_NE(historical, evening_speed);
    }
  }
}

TEST_F(SmallRouteSpeedTest, ConstrainedTrafficSpeed) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  for (auto tile_id : reader->GetTileSet()) {
    auto tile = reader->GetGraphTile(tile_id);
    for (const auto& e : tile->GetDirectedEdges()) {
      auto constrained = tile->GetSpeed(&e, baldr::kConstrainedFlowMask, 60 * 60 * 12);
      EXPECT_EQ(constrained, constrained_);

      constrained =
          tile->GetSpeed(&e, baldr::kConstrainedFlowMask | baldr::kFreeFlowMask, 60 * 60 * 12);
      EXPECT_EQ(constrained, constrained_);

      constrained = tile->GetSpeed(&e, baldr::kConstrainedFlowMask | baldr::kFreeFlowMask, 0);
      EXPECT_NE(constrained, constrained_);

      constrained = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0);
      EXPECT_NE(constrained, constrained_);

      constrained = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 0);
      EXPECT_NE(constrained, constrained_);
    }
  }
}

TEST_F(SmallRouteSpeedTest, FreeflowTrafficSpeed) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  for (auto tile_id : reader->GetTileSet()) {
    auto tile = reader->GetGraphTile(tile_id);
    for (const auto& e : tile->GetDirectedEdges()) {
      auto freeflow = tile->GetSpeed(&e, baldr::kFreeFlowMask, 0);
      EXPECT_EQ(freeflow, freeflow_);

      freeflow = tile->GetSpeed(&e, baldr::kConstrainedFlowMask, 0);
      EXPECT_NE(freeflow, freeflow_);

      freeflow = tile->GetSpeed(&e, baldr::kPredictedFlowMask, 0);
      EXPECT_NE(freeflow, freeflow_);

      freeflow = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0);
      EXPECT_NE(freeflow, freeflow_);
    }
  }
}
