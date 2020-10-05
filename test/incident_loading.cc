#include "filesystem.h"
#include "src/baldr/incident_singleton.h"
#include "test.h"

const std::string scratch_dir = std::string("data") + filesystem::path::preferred_separator +
                                std::string("incident_loading") +
                                filesystem::path::preferred_separator;

class incident_loading : public testing::Test {
protected:
  void SetUp() override {
    if (!filesystem::create_directories(scratch_dir))
      throw std::runtime_error("Could not setup incident loading test dir");
  }
  void TearDown() override {
    if (!filesystem::remove_all(scratch_dir))
      throw std::runtime_error("Could not teardown incident loading test dir");
  }
};

TEST_F(incident_loading, read_tile) {
  std::string filename = scratch_dir + "foobar";

  // no file to read
  ASSERT_FALSE(read_tile(filename)) << " should fail to find the file to read";

  // failed to parse
  {
    std::ofstream f(filename);
    f << "bazqux";
  }
  ASSERT_FALSE(read_tile(filename)) << " should fail to parse the file";

  // empty but valid
  valhalla::IncidentsTile t;
  {
    std::ofstream f(filename, std::ofstream::out | std::ofstream::trunc);
    f << t.SerializeAsString();
  }
  ASSERT_FALSE(read_tile(filename)) << " should ignore empty tile";

  // actually something
  auto* loc = t.mutable_locations()->Add();
  loc->set_edge_index(0);
  loc->set_start_offset(0);
  loc->set_end_offset(1);
  loc->set_metadata_index(0);
  {
    std::ofstream f(filename, std::ofstream::out | std::ofstream::trunc);
    f << t.SerializeAsString();
  }
  ASSERT_TRUE(read_tile(filename)) << " should return valid tile";
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
