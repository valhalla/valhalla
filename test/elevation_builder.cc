#include "test.h"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <string>
#include <unordered_set>

#include <prime_server/prime_server.hpp>

#include "baldr/curl_tilegetter.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/pointll.h"
#include "midgard/sequence.h"
#include "mjolnir/elevationbuilder.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/util.h
#include "pixels.h"
#include "skadi/sample.h"
#include "tile_server.h"

namespace {
// meters to resample shape to.
// see elevationbuilder.cc for details
constexpr double POSTING_INTERVAL = 60;
const std::string src_dir{"test/data/"};
const std::string elevation_local_src{"elevation_src/"};
const std::string src_path = src_dir + elevation_local_src;
const std::string test_tile_dir{"test/data/elevationbuild"};
const std::string pbf_dir{"test/data/elevationbuild/pbf_data/"};

zmq::context_t context;

/**
 * @brief Generate coords from tile
 *
 * */
std::unordered_set<PointLL> get_coord(const std::string& tile_dir, const std::string& tile);
/**
 * @brief Contract full path to relative
 * */
std::vector<std::string> full_to_relative_path(const std::string& root_dir,
                                               std::vector<std::string>&& paths) {
  if (root_dir.empty() || paths.empty())
    return std::move(paths);

  std::vector<std::string> res;
  res.reserve(paths.size());
  for (auto&& path : paths) {
    auto pos = path.rfind(root_dir);
    if (pos == std::string::npos) {
      res.push_back(std::move(path));
      continue;
    }

    res.push_back(path.substr(pos + root_dir.size()));
  }

  return res;
}

TEST(Filesystem, full_to_relative_path_valid_input) {
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
       {"/utrecht_tiles/build/test/data/utrecht_tiles/1/051/305.gph", "utrecht_tiles",
        "/1/051/305.gph"},
       {"/utrecht_tiles/utrecht_tiles/utrecht_tiles/utrecht_tiles/utrecht_tiles/1/051/305.gph",
        "utrecht_tiles", "/1/051/305.gph"},
       {"/utrecht_tiles/build/test/data/utrecht_tiles/1/051/305.gph", "data",
        "/utrecht_tiles/1/051/305.gph"},
       {"/data/build/test/data/utrecht_tiles/1/051/305.gph", "data", "/utrecht_tiles/1/051/305.gph"},
       {"/valhalla-internal/build/test/data/utrecht_tiles/2/000/818/660.gph",
        "/valhalla-internal/build/test/data/utrecht_tiles", "/2/000/818/660.gph"},
       {"/valhalla-internal/build/test/data/utrecht_tiles/0/003/196.gp", "",
        "/valhalla-internal/build/test/data/utrecht_tiles/0/003/196.gp"},
       {"", "/valhalla-internal/build/test/data/utrecht_tiles", ""},
       {"/valhalla-internal/build/test/data/utrecht_tiles/2/000/818/660.gph", "/tmp",
        "/valhalla-internal/build/test/data/utrecht_tiles/2/000/818/660.gph"}};

  for (const auto& test : tests)
    EXPECT_EQ(full_to_relative_path(test.remove_pattern, {test.path}).front(), test.res);
}

TEST(Filesystem, full_to_relative_path_invalid_input) {
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
    EXPECT_EQ(full_to_relative_path(test.remove_pattern, {test.path}).front(), test.res);
}

/**
 * @brief Removes all the content of the directory.
 * */
inline bool clear(const std::string& dir) {
  if (!filesystem::exists(dir))
    return false;

  if (!filesystem::is_directory(dir))
    return filesystem::remove(dir);

  for (filesystem::recursive_directory_iterator i(dir), end; i != end; ++i) {
    if (filesystem::exists(i->path()))
      filesystem::remove_all(i->path());
  }

  return true;
}

TEST(Filesystem, clear_valid_input) {
  std::vector<std::string> tests{"/tmp/save_file_input/utrecht_tiles/0/003/196.gph",
                                 "/tmp/save_file_input/utrecht_tiles/1/051/305.gph",
                                 "/tmp/save_file_input/utrecht_tiles/2/000/818/660.gph"};

  for (const auto& test : tests)
    (void)filesystem::save<std::string>(test);

  EXPECT_FALSE(filesystem::get_files("/tmp/save_file_input/utrecht_tiles").empty());

  EXPECT_TRUE(clear("/tmp/save_file_input/utrecht_tiles/2/000/818/660.gph"));
  EXPECT_TRUE(clear("/tmp/save_file_input/"));

  EXPECT_TRUE(filesystem::get_files("/tmp/save_file_input/utrecht_tiles").empty());
}

TEST(Filesystem, clear_invalid_input) {
  EXPECT_FALSE(clear(".foobar"));
}

struct ElevationDownloadTestData {
  ElevationDownloadTestData(const std::string& dir_dst) : m_dir_dst{dir_dst} {
    load_tiles();
  }

  void load_tiles() {
    for (auto&& ftile : full_to_relative_path(m_dir_dst, filesystem::get_files(m_dir_dst))) {
      if (ftile.find("gph") != std::string::npos && !m_test_tile_names.count(ftile))
        m_test_tile_names.insert(std::move(ftile));
    }
  }

  std::vector<valhalla::baldr::GraphId> m_test_tile_ids;
  std::unordered_set<std::string> m_test_tile_names;
  std::string m_dir_dst;
};

struct TestableSample : public valhalla::skadi::sample {
  static uint16_t get_tile_index(const PointLL& coord) {
    return valhalla::skadi::sample::get_tile_index(coord);
  }
};

std::unordered_set<PointLL> get_coord(const std::string& tile_dir, const std::string& tile) {
  if (tile_dir.empty() || tile.empty())
    return {};

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

TEST(ElevationBuilder, get_coord) {
  EXPECT_TRUE(get_coord(test_tile_dir + "/tile_dir", "").empty());
  EXPECT_TRUE(get_coord("", "/0/003/196.gph").empty());
  EXPECT_FALSE(get_coord(test_tile_dir + "/tile_dir", "/0/003/196.gph").empty());
}

TEST(ElevationBuilder, get_tile_ids) {
  auto config = test::make_config("test/data", {}, {});

  EXPECT_TRUE(valhalla::mjolnir::get_tile_ids(config, {}).empty());

  config.erase("mjolnir.tile_dir");
  // check if config contains tile_dir
  EXPECT_TRUE(valhalla::mjolnir::get_tile_ids(config, {"/0/003/196.gph"}).empty());

  // TODO: I don't understand what the following lines have to do with anything?
  // check if config contains elevation dir
  config.erase("additional_data.elevation");
  EXPECT_TRUE(valhalla::mjolnir::get_tile_ids(config, {"/0/003/196.gph"}).empty());

  config.put<std::string>("additional_data.elevation", "test/data/elevation_dst/");
  config.put<std::string>("mjolnir.tile_dir", "test/data/parser_tiles");
  // check if tile is from the specified tile_dir (it is not)
  EXPECT_TRUE(valhalla::mjolnir::get_tile_ids(config, {"/0/003/196.gph"}).empty());

  config.put<std::string>("mjolnir.tile_dir", test_tile_dir + "/tile_dir");
  EXPECT_FALSE(valhalla::mjolnir::get_tile_ids(config, {"/0/003/196.gph"}).empty());
}

TEST(ElevationBuilder, test_loaded_elevations) {
  const auto& config = test::
      make_config("test/data",
                  {{"additional_data.elevation_url",
                    "127.0.0.1:38004/route-tile/v1/{tilePath}?version=%version&access_token=%token"},
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
    auto elev = valhalla::skadi::get_hgt_file_name(TestableSample::get_tile_index(coord));
    if (!seen.count(elev)) {
      seen.insert(elev);
      src_elevations.push_back(std::move(elev));
    }
  }

  ASSERT_FALSE(src_elevations.empty()) << "Fail to create any source elevations";

  for (const auto& elev : src_elevations)
    EXPECT_TRUE(filesystem::save<std::string>(src_path + elev));

  const auto& dst_dir = config.get<std::string>("additional_data.elevation");
  valhalla::mjolnir::ElevationBuilder::Build(config,
                                             valhalla::mjolnir::get_tile_ids(config,
                                                                             params
                                                                                 .m_test_tile_names));

  ASSERT_TRUE(filesystem::exists(dst_dir));

  const auto& elev_paths = filesystem::get_files(dst_dir);
  ASSERT_FALSE(elev_paths.empty());

  std::unordered_set<std::string> dst_elevations;
  for (const auto& elev : elev_paths)
    dst_elevations.insert(elev);

  clear(dst_dir);

  for (const auto& elev : src_elevations) {
    EXPECT_TRUE(
        std::find_if(std::begin(dst_elevations), std::end(dst_elevations), [&elev](const auto& file) {
          return file.find(elev) != std::string::npos;
        }) != std::end(dst_elevations));
  }

  clear(src_path);
}

/************************************************************************
 * Validate that downloaded elevation tiles were not changed during loading
 * and that they applied correctly. Compare hash-sum of the tiles
 * with applied offline generated elevation tiles and online loaded.
 * ************************************************************************/

/**
 *
 * @brief Extract file name from url.
 * */
std::string filename(const std::string& url) {
  auto it = url.rfind('/');
  if (it == std::string::npos)
    return {};
  return url.substr(it);
}

/**
 * @brief Download file from passed url and return full path to the loaded file.
 * @param[in] url url to follow to download file.
 * @param[in] path path where to save downloaded file.
 * @return - full path to the downloaded file.
 * */
std::string download(const std::string& url, const std::string& path) {
  curl_tile_getter_t tile_getter(3, "", false);
  std::int8_t repeat{3};
  auto result = tile_getter.get(url);
  while (--repeat > 0 && result.status_ != tile_getter_t::status_code_t::SUCCESS)
    result = tile_getter.get(url);

  if (result.status_ != tile_getter_t::status_code_t::SUCCESS)
    return {};

  auto full_path = path + filename(url);
  if (!filesystem::save(full_path, result.bytes_))
    return {};

  return full_path;
}

void generate_tiles(const std::string& dst_dir) {
  const std::string pbf_url{"https://download.geofabrik.de/europe/cyprus-latest.osm.pbf"};
  auto full_path = download(pbf_url, pbf_dir);

  ASSERT_TRUE(filesystem::is_regular_file(full_path));

  const valhalla::mjolnir::BuildStage start_stage{valhalla::mjolnir::BuildStage::kInitialize};
  const valhalla::mjolnir::BuildStage end_stage{valhalla::mjolnir::BuildStage::kCleanup};
  const auto& config = test::make_config("test/data", {{"mjolnir.tile_dir", dst_dir}}, {});

  ASSERT_TRUE(valhalla::mjolnir::build_tile_set(config, {full_path}, start_stage, end_stage));

  ASSERT_TRUE(filesystem::exists(dst_dir));
  ASSERT_FALSE(filesystem::is_empty(dst_dir));
}

bool are_files_equal(const std::string& file1, const std::string& file2) {
  if (!filesystem::exists(file1) || !filesystem::exists(file2))
    return false;

  std::ifstream f1(file1, std::ifstream::binary | std::ifstream::ate);
  std::ifstream f2(file2, std::ifstream::binary | std::ifstream::ate);

  if (f1.fail() || f2.fail()) {
    return false; // file problem
  }

  if (f1.tellg() != f2.tellg()) {
    return false; // size mismatch
  }

  f1.seekg(0, std::ifstream::beg);
  f2.seekg(0, std::ifstream::beg);
  return std::equal(std::istreambuf_iterator<char>(f1.rdbuf()), std::istreambuf_iterator<char>(),
                    std::istreambuf_iterator<char>(f2.rdbuf()));
}

bool are_dirs_equal(const std::string& dir1, const std::string& dir2) {
  for (const auto& file : full_to_relative_path(dir1, filesystem::get_files(dir1))) {
    if (!are_files_equal(dir1 + file, dir2 + file))
      return false;
  }
  return true;
}

std::string read_file(const std::string& file) {
  std::ifstream t(file);
  std::stringstream buffer;
  buffer << t.rdbuf();
  return buffer.str();
}

bool copy_files(const std::string& src,
                const std::string& dst,
                const std::vector<std::string>& files = {}) {
  if (files.empty() || !filesystem::exists(src) || !filesystem::exists(dst))
    return false;

  for (const auto& file : files) {
    if (!filesystem::is_regular_file(file))
      continue;

    (void)filesystem::save(dst + full_to_relative_path(src, {file}).front(), read_file(file));
  }

  return !filesystem::is_empty(dst);
}

/**
 * @brief Copy contents of src dir into dst dir.
 * */
bool copy(const std::string& src, const std::string& dst) {
  if (!filesystem::exists(src) || !filesystem::exists(dst))
    return false;

  if (filesystem::is_directory(src) && filesystem::is_empty(src))
    return false;
  return copy_files(src, dst, filesystem::get_files(src));
}

void store_elevation(const std::string& elev) {
  filesystem::save<std::string>(elev);
  valhalla::midgard::sequence<int16_t> s(elev, true);
  for (size_t i = 0; i < 3601 * 2; ++i)
    s.push_back(((i / 3601 + 1) & 0xFF) << 8);
  for (size_t i = 0; i < 3601 * (3601 - 2); ++i)
    s.push_back(((-32768 & 0xFF) << 8) | ((-32768 >> 8) & 0xFF));
}

bool build_elevations(const std::string& offline_dir, const std::string& src_path) {
  ElevationDownloadTestData params{offline_dir};
  std::unordered_set<PointLL> coords_storage;
  for (const auto& tile : params.m_test_tile_names) {
    for (const auto& coord : get_coord(offline_dir, tile)) {
      coords_storage.insert(coord);
    }
  }

  std::vector<std::string> src_elevations;
  std::unordered_set<std::string> seen;
  for (const auto& coord : coords_storage) {
    auto elev = valhalla::skadi::get_hgt_file_name(TestableSample::get_tile_index(coord));
    if (!seen.count(elev)) {
      seen.insert(elev);
      store_elevation(src_path + elev);
    }
  }

  return true;
}

void apply_elevations(const std::string& tile_dir,
                      const std::string& elevation_path,
                      const std::string& url = {},
                      const std::string& remote_path = {}) {
  ElevationDownloadTestData params{tile_dir};
  auto config = test::make_config("test/data", {{"mjolnir.tile_dir", tile_dir},
                                                {"additional_data.elevation", elevation_path},
                                                {"mjolnir.concurrency", "5"}});

  if (!url.empty()) {
    config.put("additional_data.elevation_url", url);
    config.put("additional_data.elevation_dir", remote_path);
  }

  valhalla::mjolnir::ElevationBuilder::Build(config,
                                             valhalla::mjolnir::get_tile_ids(config,
                                                                             params
                                                                                 .m_test_tile_names));
}

TEST(ElevationBuilder, compare_applied_elevations) {
  const std::string tile_dst{test_tile_dir + "/tiles_from_pbf"};
  ASSERT_TRUE(filesystem::create_directories(tile_dst));
  generate_tiles(tile_dst);

  for (const auto& tile_path : filesystem::get_files(tile_dst))
    ASSERT_TRUE(filesystem::exists(tile_path) && filesystem::is_regular_file(tile_path));

  const std::string offline_dir{test_tile_dir + "/offline_test_dir"};
  ASSERT_TRUE(filesystem::create_directories(offline_dir)) << "Failed to create " << offline_dir;

  const std::string online_dir{test_tile_dir + "/online_test_dir"};
  ASSERT_TRUE(filesystem::create_directories(online_dir)) << "Failed to create " << online_dir;

  ASSERT_TRUE(copy(tile_dst, offline_dir)) << "Failed to copy files to " << offline_dir;

  for (const auto& tile_path : filesystem::get_files(offline_dir))
    ASSERT_TRUE(filesystem::exists(tile_path) && filesystem::is_regular_file(tile_path));

  ASSERT_TRUE(are_dirs_equal(tile_dst, offline_dir));

  ASSERT_TRUE(copy(tile_dst, online_dir)) << "Failed to copy files to " << online_dir;
  for (const auto& tile_path : filesystem::get_files(online_dir))
    ASSERT_TRUE(filesystem::exists(tile_path) && filesystem::is_regular_file(tile_path));

  ASSERT_TRUE(are_dirs_equal(tile_dst, online_dir));

  ASSERT_TRUE(filesystem::create_directories(src_path)) << "Failed to create " << src_path;

  build_elevations(offline_dir, src_path);
  for (const auto& elev_path : filesystem::get_files(src_path))
    ASSERT_TRUE(filesystem::exists(elev_path) && filesystem::is_regular_file(elev_path));

  apply_elevations(offline_dir, src_path);
  ASSERT_FALSE(are_dirs_equal(tile_dst, offline_dir));

  auto url{"127.0.0.1:38004/route-tile/v1/{tilePath}?version=%version&access_token=%token"};
  auto elev_storage_dir{"test/data/elevation_dst/"};
  ASSERT_TRUE(filesystem::create_directories(elev_storage_dir)) << "Failed to create " << src_path;

  apply_elevations(online_dir, elev_storage_dir, url, elevation_local_src);
  ASSERT_FALSE(are_dirs_equal(tile_dst, online_dir));

  ASSERT_TRUE(are_dirs_equal(offline_dir, online_dir));

  clear(src_path);
  clear(elev_storage_dir);
  clear(online_dir);
  clear(offline_dir);
  clear(tile_dst);
  clear(pbf_dir);
}

} // namespace

class HttpElevationsEnv : public ::testing::Environment {
public:
  void SetUp() override {
    valhalla::test_tile_server_t server;
    server.set_url("127.0.0.1:38004");
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