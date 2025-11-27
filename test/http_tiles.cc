#include "baldr/curl_tilegetter.h"
#include "baldr/graphtile.h"
#include "test.h"
#include "tile_server.h"
#include "tyr/actor.h"
#include "worker.h"

#include <boost/property_tree/ptree.hpp>
#include <prime_server/zmq_helpers.hpp>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

using namespace valhalla;

zmq::context_t context;
const std::string tile_remote_address{"127.0.0.1:48004"};
const std::string tile_source_dir = VALHALLA_BUILD_DIR "test/data/utrecht_tiles";
const auto tar_path = tile_source_dir + "/tiles.tar";
const std::string user_pw = "abcd:1234";

std::string get_tile_url(const bool is_tar) {
  std::ostringstream oss;
  if (is_tar) {
    oss << tile_remote_address << "/route-tar/v1";
  } else {
    oss << tile_remote_address << "/route-tile/v1/" << baldr::GraphTile::kTilePathPattern
        << "?version=%version&access_token=%token";
  }

  return oss.str();
}

boost::property_tree::ptree make_conf(const std::string& tile_dir,
                                      const bool tile_url_gz,
                                      const bool is_tar,
                                      const std::string& upw) {

  auto conf = test::make_config(tile_dir, {{"mjolnir.user_agent", "MapboxNavigationNative"}});

  conf.put("mjolnir.tile_url", get_tile_url(is_tar));
  if (tile_dir.empty()) {
    conf.erase("mjolnir.tile_dir");
  }
  if (!upw.empty()) {
    conf.put("mjolnir.tile_url_user_pw", upw);
  }

  conf.put("mjolnir.tile_url_gz", tile_url_gz);
  conf.put("loki.use_connectivity", false);
  return conf;
}

void test_route(const std::string& tile_dir,
                const bool tile_url_gz,
                const bool is_tar,
                const std::string& upw = "") {
  auto conf = make_conf(tile_dir, tile_url_gz, is_tar, upw);
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

class HttpTilesNoCache : public ::testing::Test {
protected:
  void TearDown() override {
    if (std::filesystem::exists(tile_source_dir + "/id.txt")) {
      std::filesystem::remove(tile_source_dir + "/id.txt");
    }
  }
};

TEST(HttpTilesNoCache, test_tar_tiles_no_cache) {
  test_route("", false, true);
}

TEST(HttpTilesNoCache, test_no_cache_no_gz) {
  test_route("", false, false);
}

TEST(HttpTilesNoCache, test_no_cache_gz) {
  test_route("", true, false);
}

class HttpTilesWithCache : public ::testing::Test {
protected:
  void SetUp() override {
    std::filesystem::remove_all("url_tile_cache");
  }
  void TearDown() override {
    std::filesystem::remove_all("url_tile_cache");
  }
};

TEST_F(HttpTilesWithCache, test_tar_cache) {
  test_route("url_tile_cache", false, true);
}

TEST_F(HttpTilesWithCache, test_tar_cache_outdated) {
  // create a fake id.txt with a bogus checksum and let GetGraphTile fail
  const std::string id_txt_path = "url_tile_cache/id.txt";
  std::filesystem::create_directories("url_tile_cache");

  std::ofstream id_txt_file(id_txt_path, std::ios::binary);
  if (id_txt_file) {
    id_txt_file << get_tile_url(true) << std::endl;
    id_txt_file << 1234 << std::endl;
  }
  id_txt_file.close();

  try {
    test_route("url_tile_cache", false, true);
    FAIL() << "Expected valhalla_exception_t";
  } catch (const valhalla_exception_t& e) {
    // a bit too generic IMO, at least there's an error log
    EXPECT_THAT(e.what(),
                ::testing::HasSubstr("Remote tar file has changed, service is unavailable"));
    EXPECT_EQ(e.code, 446);
  } catch (...) { FAIL() << "should've raised a valhalla_exception_t"; }
}

TEST_F(HttpTilesWithCache, test_tar_user_pw) {
  // happy path
  test_route("url_tile_cache", false, true, user_pw);

  // wrong user:password should return a 401
  try {
    test_route("url_tile_cache", false, true, "obviously:wrong");
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& e) {
    EXPECT_THAT(e.what(), ::testing::HasSubstr("HTTP status 401"));
  }
}

TEST_F(HttpTilesWithCache, test_cache_no_gz) {
  test_route("url_tile_cache", false, false);
}

TEST_F(HttpTilesWithCache, test_cache_gz) {
  test_route("url_tile_cache", true, false);
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

  const std::string tile_url_base = tile_remote_address + "/route-tile/v1/";
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

  curl_tile_getter_t tile_getter(curler_count, "", params.is_gzipped_tile, "");
  EXPECT_EQ(tile_getter.gzipped(), params.is_gzipped_tile);

  std::vector<std::thread> threads;
  threads.reserve(thread_count);
  for (size_t thread_i = 0; thread_i < thread_count; ++thread_i) {
    threads.emplace_back([&, thread_i]() {
      for (size_t tile_i = 0; tile_i < tile_count; ++tile_i) {
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
            auto tile = GraphTile::Create(GraphId(), std::move(result.bytes_));
            ASSERT_TRUE(tile);
            EXPECT_EQ(tile->id(), expected_tile_id);
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

  curl_tile_getter_t tile_getter(curler_count, "", params.is_gzipped_tile, "");
  EXPECT_EQ(tile_getter.gzipped(), params.is_gzipped_tile);

  std::vector<std::thread> threads;
  threads.reserve(thread_count);
  for (size_t thread_i = 0; thread_i < thread_count; ++thread_i) {
    threads.emplace_back([&, thread_i]() {
      for (size_t tile_i = 0; tile_i < tile_count; ++tile_i) {
        bool is_for_this_thread = ((tile_i % thread_count) == thread_i);
        if (!is_for_this_thread) {
          continue;
        }

        auto test_tile_index = tile_i % params.test_tile_names.size();
        auto expected_tile_id = params.test_tile_ids[test_tile_index];
        auto tile =
            GraphTile::CacheTileURL(params.full_tile_url_pattern, expected_tile_id, &tile_getter, "");

        if (expected_tile_id != non_existent_tile_id) {
          ASSERT_TRUE(tile);
          EXPECT_EQ(tile->id(), expected_tile_id);
        } else {
          EXPECT_FALSE(tile) << "Expected no tile";
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

  curl_tile_getter_t tile_getter(2, "", params.is_gzipped_tile, "");
  std::string tile_uri;
  const curl_tile_getter_t::interrupt_t interrupt = [&tile_uri, &canceled_uris] {
    if (canceled_uris.find(tile_uri) != canceled_uris.end()) {
      throw std::runtime_error("Interrupt");
    }
  };

  tile_getter.set_interrupt(&interrupt);
  for (size_t tile_i = 0; tile_i < params.test_tile_ids.size(); ++tile_i) {
    auto test_tile_index = tile_i % params.test_tile_names.size();
    auto expected_tile_id = params.test_tile_ids[test_tile_index];

    tile_uri = url_builder(tile_i);
    if (canceled_uris.find(tile_uri) != canceled_uris.end()) {
      EXPECT_THROW(tile_getter.get(tile_uri), std::runtime_error);
      continue;
    }

    auto result = tile_getter.get(tile_uri);
    if (result.status_ == tile_getter_t::status_code_t::SUCCESS) {
      auto tile = GraphTile::Create(GraphId(), std::move(result.bytes_));
      ASSERT_TRUE(tile);
      EXPECT_EQ(tile->id(), expected_tile_id);
    } else {
      EXPECT_EQ(expected_tile_id, non_existent_tile_id);
    }
  }
}

class HttpTilesEnv : public ::testing::Environment {
public:
  void SetUp() override {
    // start a file server for utrecht tiles
    valhalla::test_tile_server_t server;
    server.set_url(tile_remote_address);
    server.set_user_pw(user_pw);
    server.start(tile_source_dir, tar_path, context);
  }

  void TearDown() override {
  }
};

auto* const http_tiles_env_ = testing::AddGlobalTestEnvironment(new HttpTilesEnv);
