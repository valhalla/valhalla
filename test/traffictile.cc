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
    std::uint32_t incident_counts_0;
    std::uint32_t incident_counts_1;
    traffic::Incident incident_0_0;
    traffic::Incident incident_0_1;
    traffic::Incident incident_0_2;
    traffic::Incident incident_0_3;
    traffic::Incident incident_1_0;
    traffic::Incident incident_1_1;
    traffic::Incident incident_1_2;
    traffic::Incident incident_1_3;
  };
#pragma pack(pop)

  TestTile testdata = {};

  testdata.header.directed_edge_count = 3;
  testdata.header.incident_buffer_size = 4;
  testdata.header.active_incident_buffer = 1;
  testdata.speed3.has_incident = true;
  testdata.incident_1_0.edge_index = 2;
  testdata.incident_counts_1 = 1;
  testdata.speed3.speed_kmh = 99;

  traffic::Tile tile(reinterpret_cast<char*>(&testdata));

  auto incidents = tile.getIncidentsForDirectedEdge(2);
  ASSERT_EQ(incidents.size(), 1);
  EXPECT_EQ(incidents.front().edge_index, 2);

  auto speed = tile.getTrafficForDirectedEdge(2);
  EXPECT_EQ(speed.speed_kmh, 99);
}

TEST(Traffic, NullTileConstruction) {
  using namespace valhalla::baldr;
  traffic::Tile tile(nullptr); // Should not segfault

  auto speed = tile.getTrafficForDirectedEdge(99);
  EXPECT_EQ(speed.speed_kmh, 0);

  auto incidents = tile.getIncidentsForDirectedEdge(99);
  EXPECT_EQ(incidents.size(), 0);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}