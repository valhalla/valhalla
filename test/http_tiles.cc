#include "test.h"

#include "baldr/curl_tilegetter.h"
#include "baldr/graphtile.h"
#include "tyr/actor.h"
#include "valhalla/filesystem.h"
#include "valhalla/tile_server.h"

#include <prime_server/prime_server.hpp>

#include <ostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

using namespace valhalla;

zmq::context_t context;

std::string get_tile_url() {
  std::ostringstream oss;
  oss << test_tile_server_t::server_url << "/route-tile/v1/" << baldr::GraphTile::kTilePathPattern
      << "?version=%version&access_token=%token";

  return oss.str();
}

boost::property_tree::ptree
make_conf(const std::string& tile_dir, bool tile_url_gz, size_t curler_count) {
  // fake up config against pine grove traffic extract
  auto conf = test::json_to_pt(R"({
      "mjolnir":{
        "user_agent":"MapboxNavigationNative"
      },
      "loki":{
        "actions":["locate","route","sources_to_targets","optimized_route","isochrone","trace_route","trace_attributes","transit_available"],
        "logging":{"long_request": 100},
        "service_defaults":{"minimum_reachability": 50,"radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "street_side_max_distance": 1000, "heading_tolerance": 60}
      },
      "thor":{"logging":{"long_request": 110}},
      "skadi":{"actons":["height"],"logging":{"long_request": 5}},
      "meili":{"customizable": ["breakage_distance"],
               "mode":"auto","grid":{"cache_size":100240,"size":500},
               "default":{"beta":3,"breakage_distance":2000,"geometry":false,"gps_accuracy":5.0,"interpolation_distance":10,
               "max_route_distance_factor":3,"max_route_time_factor":3,"max_search_radius":100,"route":true,
               "search_radius":50,"sigma_z":4.07,"turn_penalty_factor":200}},
      "service_limits": {
        "auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
        "bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "taxi": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time": 120},
        "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
        "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
        "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
        "skadi": {"max_shape": 750000,"min_resample": 10.0},
        "trace": { "max_best_paths": 4, "max_best_paths_shape": 100, "max_distance": 200000.0, "max_gps_accuracy": 100.0, "max_search_radius": 100, "max_shape": 16000 },
        "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
        "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
      }
    })");

  conf.get_child("mjolnir").put("tile_url", get_tile_url());
  if (!tile_dir.empty()) {
    conf.get_child("mjolnir").put("tile_dir", tile_dir);
  }

  if (curler_count > 1) {
    conf.get_child("mjolnir").put("max_concurrent_reader_users", curler_count);
  }

  conf.get_child("mjolnir").put("tile_url_gz", tile_url_gz);
  conf.get_child("loki").put("use_connectivity", false);
  return conf;
}

void test_route(const std::string& tile_dir, bool tile_url_gz) {
  auto conf = make_conf(tile_dir, tile_url_gz, 1);
  tyr::actor_t actor(conf);

  auto route_json = actor.route(R"({"locations":[{"lat":52.09620,"lon": 5.11909,"type":"break"},
          {"lat":52.09585,"lon":5.11934,"type":"break"}],"costing":"auto"})");
  actor.cleanup();
  auto route = test::json_to_pt(route_json);

  // TODO: check result
  // didn't find the right street names in the route?
  EXPECT_NE(route_json.find("Wijckskade"), std::string::npos);
  EXPECT_NE(route_json.find("Lauwerstraat"), std::string::npos);
}

TEST(HttpTiles, test_no_cache_no_gz) {
  test_route("", false);
}

TEST(HttpTiles, test_no_cache_gz) {
  test_route("", true);
}

class HttpTilesWithCache : public ::testing::Test {
protected:
  void SetUp() override {
    filesystem::remove_all("url_tile_cache");
  }
  void TearDown() override {
    filesystem::remove_all("url_tile_cache");
  }
};

TEST_F(HttpTilesWithCache, test_cache_no_gz) {
  test_route("url_tile_cache", false);
}

TEST_F(HttpTilesWithCache, test_cache_gz) {
  test_route("url_tile_cache", true);
}

struct TestTileDownloadData {
  TestTileDownloadData() {
    test_tile_ids = {{3196, 0, 0},
                     {818660, 2, 0},
                     {51305, 1, 0},
                     // non-existent tile to exercise 404 errors
                     {200305, 2, 0}};
    for (const auto& id : test_tile_ids) {
      test_tile_names.emplace_back(
          baldr::GraphTile::FileSuffix(id, is_gzipped_tile ? baldr::SUFFIX_COMPRESSED
                                                           : baldr::SUFFIX_NON_COMPRESSED));
    }
  }

  baldr::GraphId get_nonexistent_tile_id() const {
    return test_tile_ids.back();
  }

  const std::string tile_url_base = "127.0.0.1:8004/route-tile/v1/";
  const std::string request_params = "?version=%version&access_token=%token";
  const std::string full_tile_url_pattern =
      tile_url_base + baldr::GraphTile::kTilePathPattern + request_params;

  const bool is_gzipped_tile = false;
  std::vector<baldr::GraphId> test_tile_ids;
  std::vector<std::string> test_tile_names;
};

void test_tile_download(size_t tile_count, size_t curler_count, size_t thread_count) {
  using namespace baldr;

  TestTileDownloadData params;

  const auto non_existent_tile_id = params.get_nonexistent_tile_id();

  curl_tile_getter_t tile_getter(curler_count, "", params.is_gzipped_tile);
  EXPECT_EQ(tile_getter.gzipped(), params.is_gzipped_tile);

  std::vector<std::thread> threads;
  threads.reserve(thread_count);
  for (int thread_i = 0; thread_i < thread_count; ++thread_i) {
    threads.emplace_back([&, thread_i]() {
      for (int tile_i = 0; tile_i < tile_count; ++tile_i) {
        bool is_for_this_thread = ((tile_i % thread_count) == thread_i);
        if (!is_for_this_thread) {
          continue;
        }

        auto test_tile_index = tile_i % params.test_tile_names.size();
        auto tile_name = params.test_tile_names[test_tile_index];
        auto expected_tile_id = params.test_tile_ids[test_tile_index];

        auto tile_uri = params.tile_url_base;
        tile_uri += tile_name;
        tile_uri += params.request_params;
        {
          auto result = tile_getter.get(tile_uri);

          if (result.status_ == tile_getter_t::status_code_t::SUCCESS) {
            GraphTile tile(GraphId(), std::move(result.bytes_));
            EXPECT_EQ(tile.id(), expected_tile_id);
          } else {
            EXPECT_EQ(expected_tile_id, non_existent_tile_id);
          }
        }
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

void test_graphreader_tile_download(size_t tile_count, size_t curler_count, size_t thread_count) {
  using namespace baldr;

  TestTileDownloadData params;

  const auto non_existent_tile_id = params.get_nonexistent_tile_id();

  curl_tile_getter_t tile_getter(curler_count, "", params.is_gzipped_tile);
  EXPECT_EQ(tile_getter.gzipped(), params.is_gzipped_tile);

  std::vector<std::thread> threads;
  threads.reserve(thread_count);
  for (int thread_i = 0; thread_i < thread_count; ++thread_i) {
    threads.emplace_back([&, thread_i]() {
      for (int tile_i = 0; tile_i < tile_count; ++tile_i) {
        bool is_for_this_thread = ((tile_i % thread_count) == thread_i);
        if (!is_for_this_thread) {
          continue;
        }

        auto test_tile_index = tile_i % params.test_tile_names.size();
        auto expected_tile_id = params.test_tile_ids[test_tile_index];
        auto tile =
            GraphTile::CacheTileURL(params.full_tile_url_pattern, expected_tile_id, &tile_getter, "");

        if (expected_tile_id != non_existent_tile_id) {
          EXPECT_EQ(tile.id(), expected_tile_id);
        } else {
          EXPECT_EQ(tile.header(), nullptr) << "Expected empty header";
        }
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

TEST(HttpTiles, test_curler_single_thread_download) {
  test_tile_download(5, 1, 1);
}

TEST(HttpTiles, test_curler_multiple_threads_without_contention) {
  test_tile_download(6, 3, 2);
}

TEST(HttpTiles, test_curler_multiple_threads_optimal) {
  test_tile_download(8, 4, 4);
}

TEST(HttpTiles, test_curler_multiple_threads_contention) {
  test_tile_download(6, 2, 5);
}

TEST(HttpTiles, test_graphreader_multiple_threads) {
  test_graphreader_tile_download(8, 2, 4);
}

TEST(HttpTiles, test_interrupt) {
  using namespace baldr;

  TestTileDownloadData params;
  const auto url_builder = [&params](size_t index) -> std::string {
    auto test_tile_index = index % params.test_tile_names.size();
    auto tile_name = params.test_tile_names[test_tile_index];
    auto tile_uri = params.tile_url_base;
    tile_uri += tile_name;
    tile_uri += params.request_params;
    return tile_uri;
  };

  const auto non_existent_tile_id = params.get_nonexistent_tile_id();
  std::unordered_set<std::string> canceled_uris{url_builder(0), url_builder(2)};

  curl_tile_getter_t tile_getter(2, "", params.is_gzipped_tile);
  std::string tile_uri;
  const curl_tile_getter_t::interrupt_t interrupt = [&tile_uri, &canceled_uris] {
    if (canceled_uris.find(tile_uri) != canceled_uris.end()) {
      throw std::runtime_error("Interrupt");
    }
  };

  tile_getter.set_interrupt(&interrupt);
  for (int tile_i = 0; tile_i < params.test_tile_ids.size(); ++tile_i) {
    auto test_tile_index = tile_i % params.test_tile_names.size();
    auto expected_tile_id = params.test_tile_ids[test_tile_index];

    tile_uri = url_builder(tile_i);
    if (canceled_uris.find(tile_uri) != canceled_uris.end()) {
      EXPECT_THROW(tile_getter.get(tile_uri), std::runtime_error);
      continue;
    }

    auto result = tile_getter.get(tile_uri);
    if (result.status_ == tile_getter_t::status_code_t::SUCCESS) {
      GraphTile tile(GraphId(), std::move(result.bytes_));
      EXPECT_EQ(tile.id(), expected_tile_id);
    } else {
      EXPECT_EQ(expected_tile_id, non_existent_tile_id);
    }
  }
}

class HttpTilesEnv : public ::testing::Environment {
public:
  void SetUp() override {
    // start a file server for utrecht tiles
    test_tile_server_t::start("test/data/utrecht_tiles", context);
  }

  void TearDown() override {
  }
};

int main(int argc, char* argv[]) {
  testing::AddGlobalTestEnvironment(new HttpTilesEnv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
