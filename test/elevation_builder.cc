#include "test.h"

#include <algorithm>
#include <cstdlib>
#include <string>
#include <thread>
#include <unordered_set>

#include <prime_server/prime_server.hpp>

#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/pointll.h"
#include "mjolnir/elevationbuilder.h"
#include "mjolnir/graphtilebuilder.h"
#include "skadi/sample.h"
#include "tile_server.h"

namespace {

template <typename T>
void parallel_call(const std::function<void(const std::string&, const T&)>& func =
                       std::function<void(const std::string&, const T&)>(),
                   std::vector<std::string> st = {},
                   const std::string& path = {},
                   const T& param = {},
                   std::uint32_t num_threads = 1);
std::vector<std::string> get_files(const std::string& root_dir, bool full_path = false);
// void clear(const std::string& path);
std::unordered_set<PointLL> get_coord(const std::string& tile_dir, const std::string& tile);
std::string remove_pattern(const std::string& root_dir, const std::string& filepath);
// bool save_file(const std::string& fpath, const std::string& data = {});

// meters to resample shape to.
// see elevationbuilder.cc for details
constexpr double POSTING_INTERVAL = 60;
const std::string src_dir{"test/data/"};
const std::string elevation_local_src{"elevation_src"};
const std::string src_path = src_dir + elevation_local_src;

zmq::context_t context;

struct ElevationDownloadTestData {
  ElevationDownloadTestData(const std::string& dir_dst) : m_dir_dst{dir_dst} {
    load_tiles();
  }

  void load_tiles() {
    std::unordered_set<std::string> seen;
    for (auto&& ftile : get_files(m_dir_dst)) {
      if (ftile.find("gph") != std::string::npos && !seen.count(ftile)) {
        seen.insert(ftile);
        m_test_tile_names.push_back(std::move(ftile));
      }
    }
  }

  std::vector<valhalla::baldr::GraphId> m_test_tile_ids;
  std::vector<std::string> m_test_tile_names;
  std::string m_dir_dst;
};

struct TestableSample : public valhalla::skadi::sample {
  static uint16_t get_tile_index(const PointLL& coord) {
    return valhalla::skadi::sample::get_tile_index(coord);
  }
  static std::string get_hgt_file_name(uint16_t index) {
    return valhalla::skadi::sample::get_hgt_file_name(index);
  }
};

std::string remove_pattern(const std::string& dir, const std::string& filepath) {
  if (dir.empty() || filepath.empty())
    return filepath;

  auto pos = filepath.find(dir);
  if (pos == std::string::npos)
    return filepath;
  return filepath.substr(pos + dir.size());
}

TEST(ElevationBuilder, remove_pattern_valid_input) {
  struct test_desc {
    std::string path;
    std::string remove_pattern;
    std::string res;
  };
  std::vector<test_desc> tests =
      {{"/valhalla-internal/build/test/data/utrecht_tiles/0/003/196.gp",
        "/valhalla-internal/build/test/data/utrecht_tiles", "/0/003/196.gp"},
       {"/valhalla-internal/build/test/data/utrecht_tiles/1/051/305.gph",
        "/valhalla-internal/build/test/data/utrecht_tiles", "/1/051/305.gph"},
       {"/valhalla-internal/build/test/data/utrecht_tiles/2/000/818/660.gph",
        "/valhalla-internal/build/test/data/utrecht_tiles", "/2/000/818/660.gph"},
       {"/valhalla-internal/build/test/data/utrecht_tiles/0/003/196.gp", "",
        "/valhalla-internal/build/test/data/utrecht_tiles/0/003/196.gp"},
       {"", "/valhalla-internal/build/test/data/utrecht_tiles", ""},
       {"/valhalla-internal/build/test/data/utrecht_tiles/2/000/818/660.gph", "/tmp",
        "/valhalla-internal/build/test/data/utrecht_tiles/2/000/818/660.gph"}};

  for (const auto& test : tests)
    EXPECT_EQ(remove_pattern(test.remove_pattern, test.path), test.res);
}

TEST(ElevationBuilder, remove_pattern_invalid_input) {
  struct test_desc {
    std::string path;
    std::string remove_pattern;
    std::string res;
  };

  std::vector<test_desc> tests =
      {{"/valhalla-internal/build/test/data/utrecht_tiles/0/003/196.gph", "1", "96.gph"},
       {"/valhalla-internal/build/test/data/utrecht_tiles/1/051/3o5.gph",
        "/valhalla-internal/build/test/data/utrecht_tiles", "/1/051/3o5.gph"},
       {"$", "/valhalla-internal/build/test/data/utrecht_tiles", "$"},
       {"/valhalla-internal/build/test/data/utrecht_tiles/0/003/196.gph",
        "/valhalla-internal/build/test/data/utrecht_tiles/0/003/196.gph", ""}};

  for (const auto& test : tests)
    EXPECT_EQ(remove_pattern(test.remove_pattern, test.path), test.res);
}

std::vector<std::string> get_files(const std::string& root_dir, bool full_path) {
  std::vector<std::string> files;
  for (filesystem::recursive_directory_iterator i(root_dir), end; i != end; ++i) {
    if (i->is_regular_file() || i->is_symlink()) {
      auto file_path = full_path ? i->path().string() : remove_pattern(root_dir, i->path().string());
      files.push_back(file_path);
    }
  }

  return files;
}

// bool save_file(const std::string& fpath, const std::string& data) {
//  auto dir = filesystem::path(fpath);
//  dir.replace_filename("");
//
//  if (!filesystem::create_directories(dir))
//    return false;
//
//  std::ofstream file(fpath, std::ios::out | std::ios::binary | std::ios::ate);
//  file << (data.empty() ? fpath : data) << std::endl;
//  file.close();
//  return true;
//}

std::unordered_set<PointLL> get_coord(const std::string& tile_dir, const std::string& tile) {
  valhalla::mjolnir::GraphTileBuilder tilebuilder(tile_dir, GraphTile::GetTileId(tile_dir + tile),
                                                  true);
  tilebuilder.header_builder().set_has_elevation(true);

  uint32_t count = tilebuilder.header()->directededgecount();
  std::unordered_set<uint32_t> cache;
  cache.reserve(2 * count);

  std::unordered_set<PointLL> result;
  for (uint32_t i = 0; i < count; ++i) {
    // Get a writeable reference to the directed edge
    const DirectedEdge& directededge = tilebuilder.directededge_builder(i);
    // Get the edge info offset
    uint32_t edge_info_offset = directededge.edgeinfo_offset();
    if (cache.count(edge_info_offset))
      continue;

    cache.insert(edge_info_offset);
    // Get the shape and length
    auto shape = tilebuilder.edgeinfo(&directededge).shape();
    auto length = directededge.length();
    if (!directededge.tunnel() && directededge.use() != Use::kFerry) {
      // Evenly sample the shape. If it is really short or a bridge just do both ends
      std::vector<PointLL> resampled;
      if (length < POSTING_INTERVAL * 3 || directededge.bridge()) {
        resampled = {shape.front(), shape.back()};
      } else {
        resampled = valhalla::midgard::resample_spherical_polyline(shape, POSTING_INTERVAL);
      }
      for (auto&& el : resampled)
        result.insert(std::move(el));
    }
  }
  return result;
}

template <typename T>
void parallel_call(const std::function<void(const std::string&, const T&)>& func,
                   std::vector<std::string> st,
                   const std::string& path,
                   const T& param,
                   std::uint32_t num_threads) {
  if (!func || st.empty())
    return;

  std::uint32_t size = std::max(1U, std::min(std::thread::hardware_concurrency(), num_threads));
  std::vector<std::thread> threads;
  threads.reserve(size);
  std::mutex m;
  for (std::size_t i = 0; i < size; ++i) {
    threads.emplace_back([&]() {
      while (!st.empty()) {
        m.lock();
        if (st.empty()) {
          m.unlock();
          return;
        }

        auto fname = st.back();
        st.pop_back();
        m.unlock();

        (void)func(std::move(path + fname), param);
      }
    });
  }

  std::for_each(std::begin(threads), std::end(threads), [](auto& thread) { thread.join(); });
}

TEST(ElevationBuilder, test_loaded_elevations) {
  const auto& config = test::
      make_config("test/data",
                  {{"additional_data.elevation_url",
                    "127.0.0.1:38004/route-tile/v1/{DataPath}?version=%version&access_token=%token"},
                   {"additional_data.elevation_url_gz", "false"},
                   {"mjolnir.tile_dir", "test/data/tile_src"},
                   {"additional_data.elevation_dir", elevation_local_src},
                   {"additional_data.elevation", "test/data/elevation_dst/"},
                   {"mjolnir.concurrency", "5"}});

  const auto& tile_dir = config.get<std::string>("mjolnir.tile_dir");
  ElevationDownloadTestData params{tile_dir};
  std::unordered_set<PointLL> coords_storage;
  for (const auto& tile : params.m_test_tile_names) {
    for (const auto& coord : get_coord(tile_dir, tile)) {
      coords_storage.insert(coord);
    }
  }

  std::vector<std::string> src_elevations;
  std::unordered_set<std::string> seen;
  for (const auto& coord : coords_storage) {
    auto elev = TestableSample::get_hgt_file_name(TestableSample::get_tile_index(coord));
    if (!seen.count(elev)) {
      seen.insert(elev);
      src_elevations.push_back(std::move(elev));
    }
  }

  ASSERT_FALSE(src_elevations.empty()) << "Fail to create any source elevations";

  parallel_call<std::string>(filesystem::save, src_elevations, src_path, {},
                             config.get<std::uint32_t>("mjolnir.concurrency", 1));

  const auto& dst_dir = config.get<std::string>("additional_data.elevation");
  std::unordered_set<std::string> dst_elevations;
  for (const auto& tile : params.m_test_tile_names) {
    (void)valhalla::mjolnir::ElevationBuilder::add_elevations(params.m_test_tile_names.front(),
                                                              config);

    ASSERT_TRUE(filesystem::exists(dst_dir));
    const auto& elev_paths = get_files(dst_dir, true);

    for (const auto& elev : elev_paths)
      dst_elevations.insert(elev);

    ASSERT_FALSE(elev_paths.empty())
        << "FAIL to load any elevations for tile " << params.m_test_tile_names.front();

    filesystem::clear(dst_dir);
  }

  for (const auto& elev : src_elevations) {
    EXPECT_TRUE(std::find_if(std::begin(dst_elevations), std::end(dst_elevations),
                             [&elev](const auto& file) {
                               return file.find(elev) != std::string::npos;
                             }) != std::end(dst_elevations))
        << elev << " NOT FOUND";
  }

  filesystem::clear(src_path);
}

} // namespace

class HttpElevationsEnv : public ::testing::Environment {
public:
  void SetUp() override {
    valhalla::test_tile_server_t server;
    server.set_url("127.0.0.1:38004");
    server.set_result_endpoint("ipc:///tmp/http_test_result_endpoint_elev");
    server.set_request_interrupt("ipc:///tmp/http_test_request_interrupt_elev");
    server.set_proxy_endpoint("ipc:///tmp/http_test_proxy_endpoint_elev");
    server.start(src_dir, context);
  }

  void TearDown() override {
  }
};

int main(int argc, char* argv[]) {
  testing::AddGlobalTestEnvironment(new HttpElevationsEnv);
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}