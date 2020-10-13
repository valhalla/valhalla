#include "filesystem.h"
#include "src/baldr/incident_singleton.h"
#include "test.h"

using namespace valhalla;

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
  IncidentsTile t;
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
  ASSERT_TRUE(testable_singleton::update_tile(state, baldr::GraphId(0), {}))
      << " unable to update nonexistant tile";
  ASSERT_TRUE(state->cache.count(baldr::GraphId(0))) << " cannot find new tile in cache";
  ASSERT_TRUE(state->cache.find(baldr::GraphId(0))->second == nullptr) << " tile should be null";

  // slot exists already
  std::shared_ptr<const IncidentsTile> tile{new IncidentsTile()};
  ASSERT_TRUE(testable_singleton::update_tile(state, baldr::GraphId(0), std::move(tile)))
      << " unable to update existing tile";
  ASSERT_TRUE(state->cache.count(baldr::GraphId(0))) << " cannot find updated tile in cache";
  ASSERT_TRUE(state->cache.find(baldr::GraphId(0))->second != nullptr) << " tile should be non null";

  // unexpected tile
  state->lock_free.store(true);
  ASSERT_FALSE(testable_singleton::update_tile(state, baldr::GraphId(1), {}))
      << " should not be able to update this tile";
  ASSERT_FALSE(state->cache.count(baldr::GraphId(1))) << " should not be able to find this tile";
}

TEST_F(incident_loading, disabled) {
  // check that it bails early
  boost::property_tree::ptree config;
  config.put("mjolnir.incident_log", "/foo/bar/baz/qux");
  config.put("mjolnir.incident_dir", "/foo/bar/baz/");
  std::shared_ptr<testable_singleton::state_t> state{new testable_singleton::state_t{}};
  testable_singleton::watch(config, {}, state, [](size_t) -> bool {
    throw std::logic_error("This code should never be reached");
  });
}

TEST_F(incident_loading, incident_dir) {
  return;
  // both configs
  auto log_path = scratch_dir + "log";
  for (const auto& conf :
       std::vector<std::tuple<std::string, std::string, std::unordered_set<baldr::GraphId>>>{
           {"mjolnir.incident_dir", scratch_dir, {}},
           {"mjolnir.incident_log", log_path, {}},
           {"mjolnir.incident_dir", scratch_dir, {baldr::GraphId{666, 2, 0}}},
           {"mjolnir.incident_log", log_path, {baldr::GraphId{666, 2, 0}}},
       }) {
    // build config
    boost::property_tree::ptree config;
    config.put(std::get<0>(conf), std::get<1>(conf));
    config.put("mjolnir.incident_max_loading_latency", 0);

    // map a log file
    midgard::mem_map<uint64_t> log(log_path, 2);

    // this is what the watch function will alter
    std::shared_ptr<testable_singleton::state_t> state{new testable_singleton::state_t{}};

    // actually test the watch function. the lambda here both checks its down the right things at the
    // right time and controls each iteration of the watch loop
    const auto& tileset = std::get<2>(conf);
    testable_singleton::watch(config, tileset, state, [&state](size_t i) -> bool {
      switch (i) {
        case 0: {
          // nothing loaded
          EXPECT_TRUE(state->cache.empty()) << " in the first iteration the cache should be empty";
          // load one
          IncidentsTile t;
          auto* loc = t.mutable_locations()->Add();
          loc->set_edge_index(0);
          loc->set_start_offset(0);
          loc->set_end_offset(1);
          loc->set_metadata_index(0);
          {
            auto filename =
                scratch_dir + baldr::GraphTile::FileSuffix(baldr::GraphId{666, 2, 0}, ".pbf");
            std::ofstream f(filename, std::ofstream::out | std::ofstream::trunc);
            f << t.SerializeAsString();
          }
          // TODO: update log
          return false;
        }
        case 1: {
          // TODO: one is loaded
          // TODO: update it
          return false;
        }
        case 2: {
          // TODO: one is updated
          // TODO: remove one
          return false;
        }
        case 3: {
          // TODO: one is removed
          // TODO: update two
          return false;
        }
        case 4: {
          // TODO: two are updated
          return false;
        }
        case 5: {
          // TODO: no updates
          // TODO: remove directory
          return true;
        }
        default:
          throw std::logic_error("This code should never be reached");
      }
    });
  }
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
