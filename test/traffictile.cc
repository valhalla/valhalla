#include <gtest/gtest.h>

#include <valhalla/baldr/traffictile.h>

TEST(Traffic, TileConstruction) {
  using namespace valhalla::baldr;

// Example of how data would be packed into an real tile on disk
#pragma pack(push, 1)
  struct TestTile {
    traffic::TileHeader header;
    traffic::Speed speed1;
    traffic::Speed speed2;
    traffic::Speed speed3;
  };
#pragma pack(pop)

  TestTile testdata = {};

  testdata.header.directed_edge_count = 3;
  testdata.speed3.overall_speed = 98 >> 1;
  testdata.speed3.speed1 = 98 >> 1;
  testdata.speed3.speed2 = traffic::UNKNOWN_TRAFFIC_SPEED_VALUE;
  testdata.speed3.speed3 = traffic::UNKNOWN_TRAFFIC_SPEED_VALUE;
  testdata.speed3.is_valid = 1;

  traffic::Tile tile(reinterpret_cast<char*>(&testdata));

  auto const volatile& speed = tile.getTrafficForDirectedEdge(2);
  EXPECT_TRUE(speed.valid());
  EXPECT_FALSE(speed.closed());
  EXPECT_EQ(speed.get_overall_speed(), 98);
  EXPECT_EQ(speed.get_speed(0), 98);
  EXPECT_EQ(speed.get_speed(1), traffic::UNKNOWN_TRAFFIC_SPEED_VALUE << 1);
}

TEST(Traffic, NullTileConstruction) {
  using namespace valhalla::baldr;
  traffic::Tile tile(nullptr); // Should not segfault

  auto volatile& speed = tile.getTrafficForDirectedEdge(99);
  EXPECT_FALSE(speed.valid());
  EXPECT_FALSE(speed.closed());
}

TEST(Traffic, SpeedValid) {
  using namespace valhalla::baldr;
  traffic::Speed speed = {};
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
  speed.is_valid = 1;
  EXPECT_TRUE(speed.valid());
  EXPECT_TRUE(speed.closed());

  // Test wraparound
  speed.speed1 = traffic::UNKNOWN_TRAFFIC_SPEED_VALUE + 1;
  EXPECT_EQ(speed.speed1, 0);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
