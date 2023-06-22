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
    filesystem::remove_all(scratch_dir);
    ASSERT_TRUE(!filesystem::exists(scratch_dir));
    ASSERT_TRUE(filesystem::create_directories(scratch_dir));
  }
  void TearDown() override {
    filesystem::remove_all(scratch_dir);
    ASSERT_TRUE(!filesystem::exists(scratch_dir));
  }
};

struct testable_singleton : public incident_singleton_t {
  // make an incident singleton and inject a watch function that either passes or fails initialization
  testable_singleton(const boost::property_tree::ptree& config, bool initialize)
      : incident_singleton_t(config,
                             {},
                             [initialize](boost::property_tree::ptree,
                                          std::unordered_set<valhalla::baldr::GraphId>,
                                          std::shared_ptr<state_t> state,
                                          std::function<bool(size_t)>) {
                               state->initialized.store(initialize);
                               state->signal.notify_one();
                             }) {
  }

  // this stuff is all static and protected here we make it public so we can test it
  using incident_singleton_t::read_tile;
  using incident_singleton_t::state_t;
  using incident_singleton_t::update_tile;
  using incident_singleton_t::watch;
};

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
      << " unable to update nonexistent tile";
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

  // tell it where to update the tile
  auto hint = state->cache.find(baldr::GraphId(0));
  ASSERT_TRUE(testable_singleton::update_tile(state, baldr::GraphId(0), {}, &hint))
      << " unable to update existing tile";
  ASSERT_TRUE(state->cache.count(baldr::GraphId(0))) << " cannot find new tile in cache";
  ASSERT_TRUE(state->cache.find(baldr::GraphId(0))->second == nullptr) << " tile should be null";
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

TEST_F(incident_loading, watch) {
  // both configs
  auto log_path = scratch_dir + "log";
  for (const auto& conf :
       std::vector<std::tuple<std::string, std::string, std::unordered_set<baldr::GraphId>>>{
           {"incident_dir", scratch_dir, {}},
           {"incident_log", log_path, {}},
           {"incident_dir", scratch_dir, {baldr::GraphId{11, 1, 0}, baldr::GraphId{66, 2, 0}}},
           {"incident_log", log_path, {baldr::GraphId{11, 1, 0}, baldr::GraphId{66, 2, 0}}},
       }) {
    // build config
    boost::property_tree::ptree config;
    config.put(std::get<0>(conf), std::get<1>(conf));
    config.put("incident_max_loading_latency", 0);

    // map a log file
    baldr::GraphId snake_eyes{11, 1, 0};
    auto snake_eyes_name = scratch_dir + baldr::GraphTile::FileSuffix(snake_eyes, ".pbf");
    baldr::GraphId box_cars{66, 2, 0};
    auto box_cars_name = scratch_dir + baldr::GraphTile::FileSuffix(box_cars, ".pbf");
    ASSERT_TRUE(filesystem::create_directories(filesystem::path(snake_eyes_name).parent_path()));
    ASSERT_TRUE(filesystem::create_directories(filesystem::path(box_cars_name).parent_path()));
    midgard::sequence<uint64_t> log(log_path, true, 1);

    // if its a static tileset we must preload the changelog before the watch function maps the file
    // this is because the sequence object doesnt listen for changes to the size of the mapped file
    const auto& tileset = std::get<2>(conf);
    if (!tileset.empty()) {
      log.push_back(0);
      log.push_back(0);
    }

    // this is what the watch function will alter
    std::shared_ptr<testable_singleton::state_t> state{new testable_singleton::state_t{}};

    // this is what we will track in our test
    IncidentsTile snake_eyes_tile, box_cars_tile;

    // actually test the watch function. the lambda here both checks its down the right things at the
    // right time and controls each iteration of the watch loop
    testable_singleton::watch(config, tileset, state, [&](size_t i) -> bool {
      // what iteration is this
      switch (i) {
        case 1: {
          // nothing loaded
          EXPECT_EQ(state->cache.size(), tileset.size())
              << " in the first iteration the cache should be the same size as the tileset";
          // load one
          snake_eyes_tile.Clear();
          auto* loc = snake_eyes_tile.mutable_locations()->Add();
          loc->set_edge_index(0);
          loc->set_start_offset(0);
          loc->set_end_offset(1);
          loc->set_metadata_index(0);
          {
            std::ofstream f(snake_eyes_name, std::ofstream::out | std::ofstream::binary);
            EXPECT_TRUE(f.is_open());
            f << snake_eyes_tile.SerializeAsString();
          }
          // update log
          while (log.size() < 1)
            log.push_back(0);
          log[0] = snake_eyes | (static_cast<uint64_t>(time(nullptr)) << 25);
          return false;
        }
        case 2: {
          // one is loaded
          EXPECT_EQ(state->cache.size(), tileset.empty() ? 1 : 2) << " wrong number of cache entries";
          EXPECT_EQ(state->cache.count(snake_eyes), 1) << " there should be one tile in here now";
          EXPECT_TRUE(state->cache[snake_eyes]) << " the tile pointer should be non null";
          EXPECT_TRUE(test::pbf_equals(snake_eyes_tile, *state->cache[snake_eyes]))
              << " the tile should be equal to the one written";
          // update it
          auto* loc = snake_eyes_tile.mutable_locations()->Add();
          loc->set_edge_index(25);
          loc->set_start_offset(0.2345);
          loc->set_end_offset(0.6789);
          loc->set_metadata_index(1);
          auto* meta = snake_eyes_tile.mutable_metadata()->Add();
          meta->set_id(2343324);
          meta->set_description("foo bar baz");
          meta->set_long_description("long foo bar baz quux");
          meta->set_start_time(123456);
          meta->set_type(IncidentsTile::Metadata::ACCIDENT);
          {
            std::ofstream f(snake_eyes_name,
                            std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);
            EXPECT_TRUE(f.is_open());
            f << snake_eyes_tile.SerializeAsString();
          }
          // update log
          log[0] = snake_eyes | (static_cast<uint64_t>(time(nullptr)) << 25);
          return false;
        }
        case 3: {
          // one is updated
          EXPECT_EQ(state->cache.size(), tileset.empty() ? 1 : 2) << " wrong number of cache entries";
          EXPECT_EQ(state->cache.count(snake_eyes), 1) << " should still be in there";
          EXPECT_TRUE(state->cache[snake_eyes]) << " should still be not null";
          EXPECT_TRUE(test::pbf_equals(snake_eyes_tile, *state->cache[snake_eyes]))
              << " should have all the changes that were made";
          // remove one
          EXPECT_TRUE(filesystem::remove(snake_eyes_name)) << " couldnt remove file";
          // update log
          log[0] = snake_eyes | (static_cast<uint64_t>(time(nullptr)) << 25);
          return false;
        }
        case 4: {
          // one is null
          EXPECT_EQ(state->cache.size(), tileset.empty() ? 1 : 2) << " wrong number of cache entries";
          EXPECT_EQ(state->cache.count(snake_eyes), 1) << " should still be in there";
          EXPECT_FALSE(state->cache[snake_eyes]) << " should be null now";
          // add two back
          {
            std::ofstream f(snake_eyes_name, std::ofstream::out | std::ofstream::binary);
            EXPECT_TRUE(f.is_open());
            f << snake_eyes_tile.SerializeAsString();
          }
          auto* loc = box_cars_tile.mutable_locations()->Add();
          loc->set_edge_index(12);
          loc->set_start_offset(0.31);
          loc->set_end_offset(0.321);
          loc->set_metadata_index(5);
          auto* meta = box_cars_tile.mutable_metadata()->Add();
          meta->set_id(1234);
          meta->set_description("fafeasef");
          meta->set_long_description("asefsaefsaefsaefasefsaef");
          meta->set_start_time(73532);
          meta->set_type(IncidentsTile::Metadata::WEATHER);
          {
            std::ofstream f(box_cars_name, std::ofstream::out | std::ofstream::binary);
            EXPECT_TRUE(f.is_open());
            f << box_cars_tile.SerializeAsString();
          }
          // update log
          log[0] = snake_eyes | (static_cast<uint64_t>(time(nullptr)) << 25);
          while (log.size() < 2)
            log.push_back(0);
          log[1] = box_cars | (static_cast<uint64_t>(time(nullptr)) << 25);
          return false;
        }
        case 5: {
          // two are updated
          EXPECT_EQ(state->cache.size(), 2) << " wrong number of cache entries";
          EXPECT_EQ(state->cache.count(snake_eyes), 1) << " both should be there";
          EXPECT_TRUE(state->cache[snake_eyes]) << " should be not null";
          EXPECT_TRUE(test::pbf_equals(snake_eyes_tile, *state->cache[snake_eyes]))
              << " should be equivalent";
          EXPECT_EQ(state->cache.count(box_cars), 1) << " both should be there";
          EXPECT_TRUE(state->cache[box_cars]) << " should be not null";
          EXPECT_TRUE(test::pbf_equals(box_cars_tile, *state->cache[box_cars]))
              << " should be equivalent";
          // remove one
          EXPECT_TRUE(filesystem::remove(snake_eyes_name)) << " couldnt remove file";
          // update log
          log[1] = box_cars | (static_cast<uint64_t>(time(nullptr)) << 25);
          return false;
        }
        case 6: {
          // one is null
          EXPECT_EQ(state->cache.size(), 2) << " wrong number of cache entries";
          EXPECT_EQ(state->cache.count(snake_eyes), 1) << " should still be in there";
          EXPECT_FALSE(state->cache[snake_eyes]) << " should be null now";
          EXPECT_EQ(state->cache.count(box_cars), 1) << " should also be this one";
          EXPECT_TRUE(state->cache[box_cars]) << " should be not null";
          EXPECT_TRUE(test::pbf_equals(box_cars_tile, *state->cache[box_cars]))
              << " should be equivalent";
          // remove the dir and quit before next update
          filesystem::remove_all(scratch_dir);
          EXPECT_TRUE(!filesystem::exists(scratch_dir))
              << " Could not teardown incident loading test dir";
          return true;
        }
        default:
          throw std::logic_error("This code should never be reached");
      }
    });

    // by the end of the dance above we should have 1 loaded and 1 null
    EXPECT_EQ(state->cache.size(), 2) << " wrong number of cache entries";
    EXPECT_EQ(state->cache.count(snake_eyes), 1) << " should still be in there";
    EXPECT_FALSE(state->cache[snake_eyes]) << " should be null now";
    EXPECT_EQ(state->cache.count(box_cars), 1) << " should also be this one";
    EXPECT_TRUE(state->cache[box_cars]) << " should be not null";
    EXPECT_TRUE(test::pbf_equals(box_cars_tile, *state->cache[box_cars])) << " should be equivalent";
  }
}

TEST_F(incident_loading, constructor) {
  boost::property_tree::ptree config;
  config.put("incident_max_loading_latency", 1);

  // this should not throw because the watcher function will say its initialized
  ASSERT_NO_THROW(testable_singleton(config, true));

  // this will throw because the watcher function will not initialize
  ASSERT_THROW(testable_singleton(config, false), std::runtime_error);
}

TEST_F(incident_loading, get) {
  // setup some data for it to get
  baldr::GraphId box_cars{66, 2, 0};
  auto box_cars_name = scratch_dir + baldr::GraphTile::FileSuffix(box_cars, ".pbf");
  ASSERT_TRUE(filesystem::create_directories(filesystem::path(box_cars_name).parent_path()));
  IncidentsTile box_cars_tile;
  auto* loc = box_cars_tile.mutable_locations()->Add();
  loc->set_edge_index(12);
  loc->set_start_offset(0.31);
  loc->set_end_offset(0.321);
  loc->set_metadata_index(5);
  auto* meta = box_cars_tile.mutable_metadata()->Add();
  meta->set_id(1234);
  meta->set_description("fafeasef");
  meta->set_long_description("asefsaefsaefsaefasefsaef");
  meta->set_start_time(73532);
  meta->set_type(IncidentsTile::Metadata::WEATHER);
  {
    std::ofstream f(box_cars_name, std::ofstream::out | std::ofstream::binary);
    EXPECT_TRUE(f.is_open());
    f << box_cars_tile.SerializeAsString();
  }

  // get the one that is there
  boost::property_tree::ptree config;
  config.put("incident_dir", scratch_dir);
  auto got = incident_singleton_t::get(box_cars, config, {});
  ASSERT_TRUE(got);
  ASSERT_TRUE(test::pbf_equals(box_cars_tile, *got));

  // get the one that isnt there
  got = incident_singleton_t::get({});
  ASSERT_FALSE(got);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
