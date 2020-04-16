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
  testdata.speed3.speed_kmh = 99;
  testdata.speed3.age_bucket = traffic::MAX_SPEED_AGE_BUCKET;

  traffic::Tile tile(reinterpret_cast<char*>(&testdata));

  auto const volatile& speed = tile.getTrafficForDirectedEdge(2);
  EXPECT_TRUE(speed.valid());
  EXPECT_FALSE(speed.closed());
  auto speed_val =
      speed.speed_kmh; // have to make a copy as gtest can't accept a reference to a bitfield
  EXPECT_EQ(speed_val, 99);
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

  speed.speed_kmh = 1;
  EXPECT_FALSE(speed.valid());
  EXPECT_FALSE(speed.closed());

  speed.speed_kmh = 0;
  speed.congestion_level = 1;
  EXPECT_FALSE(speed.valid());
  EXPECT_FALSE(speed.closed());

  speed.speed_kmh = 0;
  speed.congestion_level = 4;
  EXPECT_FALSE(speed.valid());
  EXPECT_FALSE(speed.closed());

  speed.speed_kmh = 0;
  speed.age_bucket = traffic::MAX_SPEED_AGE_BUCKET;
  EXPECT_TRUE(speed.valid());
  EXPECT_TRUE(speed.closed());

  // Test wraparound
  speed.speed_kmh = valhalla::baldr::traffic::MAX_TRAFFIC_SPEED_KPH + 1;
  EXPECT_EQ(speed.speed_kmh, 0);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
