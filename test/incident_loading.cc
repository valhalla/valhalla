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

struct testable_singleton : public incident_singleton_t {
public:
  using incident_singleton_t::incident_singleton_t;
  using incident_singleton_t::read_tile;
  using incident_singleton_t::state_t;
  using incident_singleton_t::update_tile;
  using incident_singleton_t::watch;
};

TEST_F(incident_loading, constructor) {
  // TODO
}

TEST_F(incident_loading, read_tile) {
  std::string filename = scratch_dir + "foobar";

  // no file to read
  ASSERT_FALSE(testable_singleton::read_tile(filename)) << " should fail to find the file to read";

  // failed to parse
  {
    std::ofstream f(filename);
    f << "bazqux";
  }
  ASSERT_FALSE(testable_singleton::read_tile(filename)) << " should fail to parse the file";

  // empty but valid
  valhalla::IncidentsTile t;
  {
    std::ofstream f(filename, std::ofstream::out | std::ofstream::trunc);
    f << t.SerializeAsString();
  }
  ASSERT_FALSE(testable_singleton::read_tile(filename)) << " should ignore empty tile";

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
  ASSERT_TRUE(testable_singleton::read_tile(filename)) << " should return valid tile";
}

TEST_F(incident_loading, update_tile) {
  // no slot exists
  std::shared_ptr<testable_singleton::state_t> state{new testable_singleton::state_t{}};
  ASSERT_TRUE(testable_singleton::update_tile(state, valhalla::baldr::GraphId(0), {}))
      << " unable to update nonexistant tile";
  ASSERT_TRUE(state->cache.count(valhalla::baldr::GraphId(0))) << " cannot find new tile in cache";
  ASSERT_TRUE(state->cache.find(valhalla::baldr::GraphId(0))->second == nullptr)
      << " tile should be null";

  // slot exists already
  std::shared_ptr<const valhalla::IncidentsTile> tile{new valhalla::IncidentsTile()};
  ASSERT_TRUE(testable_singleton::update_tile(state, valhalla::baldr::GraphId(0), std::move(tile)))
      << " unable to update existing tile";
  ASSERT_TRUE(state->cache.count(valhalla::baldr::GraphId(0)))
      << " cannot find updated tile in cache";
  ASSERT_TRUE(state->cache.find(valhalla::baldr::GraphId(0))->second != nullptr)
      << " tile should be non null";

  // unexpected tile
  state->lock_free.store(true);
  ASSERT_FALSE(testable_singleton::update_tile(state, valhalla::baldr::GraphId(1), {}))
      << " should not be able to update this tile";
  ASSERT_FALSE(state->cache.count(valhalla::baldr::GraphId(1)))
      << " should not be able to find this tile";
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
