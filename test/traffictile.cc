#include <gtest/gtest.h>

#include "baldr/traffictile.h"

namespace {
class UnmanagedGraphMemory : public valhalla::baldr::GraphMemory {
public:
  UnmanagedGraphMemory(char* const buf, const size_t len) {
    data = buf;
    size = len;
  }
};
} // namespace

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

  TestTile testdata{};
  testdata.header.directed_edge_count = 3;
  testdata.header.traffic_tile_version = TRAFFIC_TILE_VERSION;
  testdata.speed3.overall_encoded_speed = 98 >> 1;
  testdata.speed3.encoded_speed1 = 98 >> 1;
  testdata.speed3.encoded_speed2 = UNKNOWN_TRAFFIC_SPEED_RAW;
  testdata.speed3.encoded_speed3 = UNKNOWN_TRAFFIC_SPEED_RAW;
  testdata.speed3.breakpoint1 = 255;

  auto memory =
      std::make_unique<UnmanagedGraphMemory>(reinterpret_cast<char*>(&testdata), sizeof(TestTile));
  TrafficTile tile(std::move(memory));

  auto const volatile& speed = tile.trafficspeed(2);
  EXPECT_TRUE(speed.speed_valid());
  EXPECT_FALSE(speed.closed());
  EXPECT_EQ(speed.get_overall_speed(), 98);
  EXPECT_EQ(speed.get_speed(0), 98);
  EXPECT_EQ(speed.get_speed(1), UNKNOWN_TRAFFIC_SPEED_RAW << 1);

  // Verify the version
  EXPECT_EQ(3, TRAFFIC_TILE_VERSION);
  // Test with an invalid version
  testdata.header.traffic_tile_version = 78;
  auto const volatile& invalid_speed = tile.trafficspeed(2);
  EXPECT_FALSE(invalid_speed.speed_valid());
}

TEST(Traffic, NullTileConstruction) {
  using namespace valhalla::baldr;
  TrafficTile tile(nullptr); // Should not segfault

  auto volatile& speed = tile.trafficspeed(99);
  EXPECT_FALSE(speed.speed_valid());
  EXPECT_FALSE(speed.closed());
}

TEST(Traffic, Closed) {
  using namespace valhalla::baldr;
  TrafficSpeed speed = {};
  EXPECT_FALSE(speed.closed());

  speed.encoded_speed1 = 0;
  EXPECT_FALSE(speed.closed());
  EXPECT_FALSE(speed.closed(0));

  speed.breakpoint1 = 255;
  EXPECT_TRUE(speed.closed());
  EXPECT_TRUE(speed.closed(0));

  speed.overall_encoded_speed = 0;
  EXPECT_TRUE(speed.closed());
  EXPECT_TRUE(speed.closed(0));
}

TEST(Traffic, SpeedValid) {
  using namespace valhalla::baldr;
  TrafficSpeed speed = {};
  speed.overall_encoded_speed = UNKNOWN_TRAFFIC_SPEED_RAW;
  EXPECT_FALSE(speed.speed_valid());

  speed.encoded_speed1 = 1;
  EXPECT_FALSE(speed.speed_valid());
  EXPECT_FALSE(speed.closed());

  speed.encoded_speed1 = 0;
  speed.congestion1 = 1;
  EXPECT_FALSE(speed.speed_valid());
  EXPECT_FALSE(speed.closed());

  speed.encoded_speed1 = 0;
  speed.congestion1 = 4;
  EXPECT_FALSE(speed.speed_valid());
  EXPECT_FALSE(speed.closed());

  speed.encoded_speed1 = 0;
  speed.breakpoint1 = 255;
  EXPECT_FALSE(speed.speed_valid());
  EXPECT_FALSE(speed.closed());

  speed.encoded_speed1 = 0;
  speed.breakpoint1 = 255;
  speed.overall_encoded_speed = 0;
  EXPECT_TRUE(speed.speed_valid());
  EXPECT_TRUE(speed.closed());

  // Test wraparound
  speed.encoded_speed1 = UNKNOWN_TRAFFIC_SPEED_RAW + 1;
  EXPECT_EQ(speed.encoded_speed1, 0);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
