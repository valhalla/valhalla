#include <gtest/gtest.h>

#include <valhalla/baldr/traffictile.h>

TEST(Traffic, TileConstruction) {
  using namespace valhalla::baldr;

// Example of how data would be packed into an real tile on disk
#pragma pack(push, 1)
  struct TestTile {
    TrafficTileHeader header;
    TrafficSpeed speed1;
    TrafficSpeed speed2;
    TrafficSpeed speed3;
  };
#pragma pack(pop)

  TestTile testdata = {};

  testdata.header.directed_edge_count = 3;
  testdata.header.traffic_tile_version = TRAFFIC_TILE_VERSION;
  testdata.speed3.overall_speed = 98 >> 1;
  testdata.speed3.speed1 = 98 >> 1;
  testdata.speed3.speed2 = UNKNOWN_TRAFFIC_SPEED_RAW;
  testdata.speed3.speed3 = UNKNOWN_TRAFFIC_SPEED_RAW;
  testdata.speed3.breakpoint1 = 255;

  TrafficTile tile(reinterpret_cast<char*>(&testdata));

  auto const volatile& speed = tile.trafficspeed(2);
  EXPECT_TRUE(speed.valid());
  EXPECT_FALSE(speed.closed());
  EXPECT_EQ(speed.get_overall_speed(), 98);
  EXPECT_EQ(speed.get_speed(0), 98);
  EXPECT_EQ(speed.get_speed(1), UNKNOWN_TRAFFIC_SPEED_RAW << 1);

  // Verify the version (Expect this value to change when traffictile.h is updated)
  EXPECT_EQ(0xab6edf30, TRAFFIC_TILE_VERSION);
  // Test with an invalid version
  testdata.header.traffic_tile_version = 78;
  auto const volatile& invalid_speed = tile.trafficspeed(2);
  EXPECT_FALSE(invalid_speed.valid());
}

TEST(Traffic, NullTileConstruction) {
  using namespace valhalla::baldr;
  TrafficTile tile(nullptr); // Should not segfault

  auto volatile& speed = tile.trafficspeed(99);
  EXPECT_FALSE(speed.valid());
  EXPECT_FALSE(speed.closed());
}

TEST(Traffic, SpeedValid) {
  using namespace valhalla::baldr;
  TrafficSpeed speed = {};
  EXPECT_FALSE(speed.valid());

  speed.speed1 = 1;
  EXPECT_FALSE(speed.valid());
  EXPECT_FALSE(speed.closed());

  speed.speed1 = 0;
  speed.congestion1 = 1;
  EXPECT_FALSE(speed.valid());
  EXPECT_FALSE(speed.closed());

  speed.speed1 = 0;
  speed.congestion1 = 4;
  EXPECT_FALSE(speed.valid());
  EXPECT_FALSE(speed.closed());

  speed.speed1 = 0;
  speed.breakpoint1 = 255;
  EXPECT_TRUE(speed.valid());
  EXPECT_TRUE(speed.closed());

  // Test wraparound
  speed.speed1 = UNKNOWN_TRAFFIC_SPEED_RAW + 1;
  EXPECT_EQ(speed.speed1, 0);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
