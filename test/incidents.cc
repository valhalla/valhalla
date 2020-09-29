#include <fstream>
#include <iostream>

#include <gtest/gtest.h>

#include <valhalla/proto/incidents.pb.h>

TEST(Traffic, TileDeserialisation) {

  std::ifstream fin(VALHALLA_SOURCE_DIR "/test/data/incident_tiles/804.incidents");
  ASSERT_TRUE(fin.is_open());

  valhalla::incidents::IncidentsTile tile;
  tile.ParseFromIstream(&fin);

  ASSERT_EQ(tile.edge_to_incidents_size(), 72);
  ASSERT_EQ(tile.incidents_size(), 10);

  for (const auto& incident : tile.incidents()) {
    fprintf(stderr, "Desc: %s\n", incident.description().c_str());
  }
  // ASSERT_EQ(1, 0);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
