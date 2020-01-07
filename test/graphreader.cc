#include "test.h"
#include <cstdint>

#include "baldr/connectivity_map.h"
#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"

#include <boost/filesystem.hpp>
#include <fcntl.h>

using namespace std;
using namespace valhalla::baldr;

namespace {

class test_cache : public SimpleTileCache {
public:
  using SimpleTileCache::cache_size_;
  using SimpleTileCache::max_cache_size_;
  using SimpleTileCache::SimpleTileCache;
};

test_cache make_cache(size_t cache_size) {
  return {cache_size};
}

void TestOutOfRangeLL() {
  boost::property_tree::ptree pt;
  pt.put("tile_dir", "test/gphrdr_test");
  GraphReader reader(pt);

  // Latitude out of range
  if (reader.GetGraphTile({60.0f, 100.0f}) != nullptr)
    throw std::runtime_error("Out of range LL should return nullptr");

  // Out of range longitude
  if (reader.GetGraphTile({460.0f, 60.0f}) != nullptr)
    throw std::runtime_error("Out of range LL should return nullptr");
}

void TestCacheLimits() {
  if (make_cache(0).OverCommitted())
    throw std::runtime_error("Cache should be over committed");

  if (make_cache(1).OverCommitted())
    throw std::runtime_error("Cache should be under committed");

  auto cache = make_cache(10);
  cache.cache_size_ = cache.max_cache_size_ + 1;
  if (!cache.OverCommitted())
    throw std::runtime_error("Cache should be over committed");
  cache.Clear();
  if (cache.OverCommitted())
    throw std::runtime_error("Cache should be under committed");

  cache.cache_size_ = 1;
  cache.max_cache_size_ = 0;
  if (!cache.OverCommitted())
    throw std::runtime_error("Cache should be over committed");
}

void touch_tile(const uint32_t tile_id, const std::string& tile_dir) {
  auto suffix = GraphTile::FileSuffix({tile_id, 2, 0});
  auto fullpath = tile_dir + '/' + suffix;
  boost::filesystem::create_directories(boost::filesystem::path(fullpath).parent_path());
  int fd = open(fullpath.c_str(), O_CREAT | O_WRONLY, 0644);
  if (fd >= 0)
    close(fd);
}

void TestConnectivityMap() {
  // get the property tree to create some tiles
  boost::property_tree::ptree pt;
  pt.put("tile_dir", "test/gphrdr_test");
  std::string tile_dir = pt.get<std::string>("tile_dir");
  const auto& level = TileHierarchy::levels().find(2)->second;
  boost::filesystem::remove_all(tile_dir);

  // looks like this (XX) means no tile there:
  /*
   *     XX d1 XX XX
   *     XX d0 XX XX
   *     a2 XX c0 XX
   *     a0 a1 XX b0
   *
   */

  // create some empty files with tile names
  uint32_t a0 = 0;
  touch_tile(a0, tile_dir);

  uint32_t a1 = level.tiles.RightNeighbor(a0);
  touch_tile(a1, tile_dir);

  uint32_t a2 = level.tiles.TopNeighbor(a0);
  touch_tile(a2, tile_dir);

  uint32_t b0 = level.tiles.RightNeighbor(level.tiles.RightNeighbor(a1));
  touch_tile(b0, tile_dir);

  uint32_t c0 = level.tiles.TopNeighbor(level.tiles.RightNeighbor(a1));
  touch_tile(c0, tile_dir);

  uint32_t d0 = level.tiles.TopNeighbor(level.tiles.RightNeighbor(a2));
  touch_tile(d0, tile_dir);

  uint32_t d1 = level.tiles.TopNeighbor(d0);
  touch_tile(d1, tile_dir);

  // check that it looks right
  connectivity_map_t conn(pt);
  if (conn.get_color({a0, 2, 0}) != conn.get_color({a1, 2, 0}))
    throw std::runtime_error("a's should be connected");
  if (conn.get_color({a0, 2, 0}) != conn.get_color({a2, 2, 0}))
    throw std::runtime_error("a's should be connected");
  if (conn.get_color({a1, 2, 0}) != conn.get_color({a2, 2, 0}))
    throw std::runtime_error("a's should be connected");
  if (conn.get_color({d0, 2, 0}) != conn.get_color({d1, 2, 0}))
    throw std::runtime_error("d's should be connected");
  if (conn.get_color({c0, 2, 0}) == conn.get_color({a1, 2, 0}))
    throw std::runtime_error("c is disjoint");
  if (conn.get_color({b0, 2, 0}) == conn.get_color({a0, 2, 0}))
    throw std::runtime_error("b is disjoint");
  if (conn.get_color({a2, 2, 0}) == conn.get_color({d0, 2, 0}))
    throw std::runtime_error("a is disjoint from d");

  boost::filesystem::remove_all(tile_dir);
}

struct TestGraphTile : public GraphTile {
  TestGraphTile(GraphId id, size_t size) {
    graphtile_ = std::make_shared<std::vector<char>>(sizeof(GraphTileHeader));
    header_ = reinterpret_cast<GraphTileHeader*>(graphtile_->data());
    header_->set_graphid(id);
    header_->set_end_offset(size);
  }
};

static void CheckGraphTile(const GraphTile* tile, const GraphId& expected_id, size_t expected_size) {
  test::assert_bool(tile != nullptr, "Wrong tile: nullptr");

  test::assert_bool(tile->header()->graphid().value == expected_id.value,
                    "Wrong tile id " + std::to_string(tile->header()->graphid().value));

  test::assert_bool(tile->header()->end_offset() == expected_size,
                    "Wrong tile size " + std::to_string(tile->header()->end_offset()));
}

void Test_SimpleTileCache_Clear() {
  SimpleTileCache cache(400);

  GraphId id1(100, 2, 0);
  TestGraphTile tile1(id1, 123);
  const GraphTile* inserted1 = cache.Put(id1, tile1, 123);
  CheckGraphTile(inserted1, id1, 123);

  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  GraphId id2(300, 1, 0);
  TestGraphTile tile2(id2, 200);
  const GraphTile* inserted2 = cache.Put(id2, tile2, 200);
  CheckGraphTile(inserted2, id2, 200);

  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  GraphId id3(1000, 0, 0);
  TestGraphTile tile3(id3, 500);
  const GraphTile* inserted3 = cache.Put(id3, tile3, 500);
  CheckGraphTile(inserted3, id3, 500);

  test::assert_bool(cache.OverCommitted(), "expecting overcommit");

  // Check if inserted values are correct

  const auto* returned2 = cache.Get({300, 1, 0});
  CheckGraphTile(returned2, id2, 200);
  const auto* returned1 = cache.Get({100, 2, 0});
  CheckGraphTile(returned1, id1, 123);
  const auto* returned3 = cache.Get({1000, 0, 0});
  CheckGraphTile(returned3, id3, 500);

  test::assert_bool(cache.Contains(id1), "tile3 not found");
  test::assert_bool(cache.Contains(id2), "tile1 not found");
  test::assert_bool(cache.Contains(id3), "tile2 not found");

  cache.Clear();

  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  test::assert_bool(!cache.Contains(id1), "tile3 not found");
  test::assert_bool(!cache.Contains(id2), "tile1 not found");
  test::assert_bool(!cache.Contains(id3), "tile2 not found");

  test::assert_bool(cache.Get(id1) == nullptr, "tile1 get should return null");
  test::assert_bool(cache.Get(id2) == nullptr, "tile3 get should return null");
  test::assert_bool(cache.Get(id3) == nullptr, "tile3 get should return null");
}

void Test_SimpleTileCache_Trim() {
  SimpleTileCache cache(400);

  GraphId id1(100, 2, 0);
  TestGraphTile tile1(id1, 123);
  const GraphTile* inserted1 = cache.Put(id1, tile1, 123);
  CheckGraphTile(inserted1, id1, 123);

  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  GraphId id2(300, 1, 0);
  TestGraphTile tile2(id2, 200);
  const GraphTile* inserted2 = cache.Put(id2, tile2, 200);
  CheckGraphTile(inserted2, id2, 200);

  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  GraphId id3(1000, 0, 0);
  TestGraphTile tile3(id3, 500);
  const GraphTile* inserted3 = cache.Put(id3, tile3, 500);
  CheckGraphTile(inserted3, id3, 500);

  test::assert_bool(cache.OverCommitted(), "expecting overcommit");

  // Check if inserted values are correct

  const auto* returned2 = cache.Get({300, 1, 0});
  CheckGraphTile(returned2, id2, 200);
  const auto* returned1 = cache.Get({100, 2, 0});
  CheckGraphTile(returned1, id1, 123);
  const auto* returned3 = cache.Get({1000, 0, 0});
  CheckGraphTile(returned3, id3, 500);

  test::assert_bool(cache.Contains(id1), "tile3 not found");
  test::assert_bool(cache.Contains(id2), "tile1 not found");
  test::assert_bool(cache.Contains(id3), "tile2 not found");

  cache.Trim();

  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  test::assert_bool(!cache.Contains(id1), "tile3 not found");
  test::assert_bool(!cache.Contains(id2), "tile1 not found");
  test::assert_bool(!cache.Contains(id3), "tile2 not found");

  test::assert_bool(cache.Get(id1) == nullptr, "tile1 get should return null");
  test::assert_bool(cache.Get(id2) == nullptr, "tile3 get should return null");
  test::assert_bool(cache.Get(id3) == nullptr, "tile3 get should return null");
}

void Test_TileCacheLRU_HARD_Creation() {
  TileCacheLRU zero_limit_cache(0, TileCacheLRU::MemoryLimitControl::HARD);
  TileCacheLRU cache(1023, TileCacheLRU::MemoryLimitControl::HARD);
  TileCacheLRU big_cache(1073741824, TileCacheLRU::MemoryLimitControl::HARD);
}

void Test_TileCacheLRU_HARD_InsertSingleItemBiggerThanCacheSize() {
  TileCacheLRU cache(1023, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId id1(100, 2, 0);
  TestGraphTile tile1(id1, 2000);

  test::assert_throw<std::runtime_error,
                     std::function<void()>>([&]() { cache.Put(id1, tile1, 2000); },
                                            "no exception triggered");

  test::assert_bool(cache.Get(id1) == nullptr, "not expecting to have tile1 inserted");
  test::assert_bool(!cache.Contains(id1), "tile1 existence check should fail");
}

void Test_TileCacheLRU_HARD_InsertCacheFullOneshot() {
  const size_t tile1_size = 1234;

  TileCacheLRU cache(tile1_size, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  TestGraphTile tile1(tile1_id, tile1_size);

  const auto* inserted1 = cache.Put(tile1_id, tile1, tile1_size);

  CheckGraphTile(inserted1, tile1_id, tile1_size);
  test::assert_bool(!cache.OverCommitted(), "should not be overcommited");

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
}

void Test_TileCacheLRU_HARD_InsertCacheFull() {
  TileCacheLRU cache(10000, TileCacheLRU::MemoryLimitControl::HARD);

  const size_t tile1_size = 4000;
  GraphId tile1_id(1000, 1, 0);
  TestGraphTile tile1(tile1_id, tile1_size);
  const auto* inserted1 = cache.Put(tile1_id, tile1, tile1_size);
  CheckGraphTile(inserted1, tile1_id, tile1_size);

  const size_t tile2_size = 6000;
  GraphId tile2_id(33, 2, 0);
  TestGraphTile tile2(tile2_id, tile2_size);
  const auto* inserted2 = cache.Put(tile2_id, tile2, tile2_size);
  CheckGraphTile(inserted2, tile2_id, tile2_size);

  test::assert_bool(!cache.OverCommitted(), "should not be overcommited");

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile1 not found");
}

void Test_TileCacheLRU_HARD_InsertNoEviction() {
  TileCacheLRU cache(1023, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId id1(100, 2, 0);
  TestGraphTile tile1(id1, 123);
  const GraphTile* inserted1 = cache.Put(id1, tile1, 123);
  CheckGraphTile(inserted1, id1, 123);

  GraphId id2(300, 1, 0);
  TestGraphTile tile2(id2, 200);
  const GraphTile* inserted2 = cache.Put(id2, tile2, 200);
  CheckGraphTile(inserted2, id2, 200);

  GraphId id3(1000, 0, 0);
  TestGraphTile tile3(id3, 500);
  const GraphTile* inserted3 = cache.Put(id3, tile3, 500);
  CheckGraphTile(inserted3, id3, 500);

  // Check if inserted values are correct

  const auto* returned2 = cache.Get({300, 1, 0});
  CheckGraphTile(returned2, id2, 200);

  const auto* returned1 = cache.Get({100, 2, 0});
  CheckGraphTile(returned1, id1, 123);

  const auto* returned3 = cache.Get({1000, 0, 0});
  CheckGraphTile(returned3, id3, 500);

  // Make sure tiles we have not cached are not found

  test::assert_bool(!(cache.Get({1345, 1, 0}) != nullptr || cache.Get({100, 1, 0}) != nullptr ||
                      cache.Get({0, 0, 0}) != nullptr),
                    "Cache returned entry that has never been inserted");
}

void Test_TileCacheLRU_HARD_InsertWithEvictionBasic() {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 45;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile3_size), tile3_size);

  test::assert_bool(cache.Contains(tile3_id), "tile3 not found");
  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");

  // Add an insertion which now requires the eviction
  // The first inserted items should be evicted here (id1)

  GraphId tile4_id(400, 2, 0);
  const size_t tile4_size = 20;
  cache.Put(tile4_id, TestGraphTile(tile4_id, tile4_size), tile4_size);

  test::assert_bool(!cache.Contains(tile1_id), "tile1 is not evicted");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");
  test::assert_bool(cache.Contains(tile3_id), "tile3 not found");
  test::assert_bool(cache.Contains(tile4_id), "tile4 not found");

  // Now we access an entry that would be evicted next
  // to promote its position in the LRU list and change eviction order

  const auto* returned2 = cache.Get(tile2_id);
  CheckGraphTile(returned2, tile2_id, tile2_size);

  GraphId tile5_id(999, 1, 0);
  const size_t tile5_size = 200;
  cache.Put(tile5_id, TestGraphTile(tile5_id, tile5_size), tile5_size);

  // Expecting the next eviction of tile3 since tile1 has been just accessed

  test::assert_bool(!cache.Contains(tile1_id), "tile1 is not evicted");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");
  test::assert_bool(!cache.Contains(tile3_id), "tile3 is not evicted");
  test::assert_bool(cache.Contains(tile4_id), "tile4 not found");
  test::assert_bool(cache.Contains(tile4_id), "tile5 not found");

  test::assert_bool(cache.Get(tile1_id) == nullptr, "tile1 get should return null");
  test::assert_bool(cache.Get(tile3_id) == nullptr, "tile3 get should return null");

  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  CheckGraphTile(cache.Get(tile4_id), tile4_id, tile4_size);
  CheckGraphTile(cache.Get(tile5_id), tile5_id, tile5_size);
}

void Test_TileCacheLRU_HARD_OverwriteSameSize() {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 45;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile3_size), tile3_size);

  // overwrite with a new object and same size
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile3_size);
  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");
  test::assert_bool(cache.Contains(tile3_id), "tile3 not found");
}

void Test_TileCacheLRU_HARD_OverwriteSmallerSize() {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 45;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile3_size), tile3_size);

  const size_t tile4_size = 8;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile4_size), tile4_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile4_size);
  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");
  test::assert_bool(cache.Contains(tile3_id), "tile3 not found");
}

void Test_TileCacheLRU_HARD_OverwriteBiggerSizeNoEviction() {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 20;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile3_size), tile3_size);

  const size_t tile4_size = 45;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile4_size), tile4_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile4_size);
  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");
  test::assert_bool(cache.Contains(tile3_id), "tile3 not found");
}

void Test_TileCacheLRU_HARD_OverwriteBiggerSizeEvictionOne() {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 45;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile3_size), tile3_size);

  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");
  test::assert_bool(cache.Contains(tile3_id), "tile3 not found");

  // Add an insertion which now requires the eviction
  // The first inserted items should be evicted here (id1)
  // Note, we are not inserting a new entry but updating the existing one (id2)

  const size_t tile4_size = 260;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile4_size), tile4_size);

  test::assert_bool(!cache.Contains(tile1_id), "tile1 is not evicted");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");
  test::assert_bool(cache.Contains(tile3_id), "tile3 not found");

  test::assert_bool(cache.Get(tile1_id) == nullptr, "tile1 should be evicted");
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile4_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile3_size);
}

void Test_TileCacheLRU_HARD_OverwriteBiggerSizeEvictionMultiple() {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 45;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile3_size), tile3_size);

  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");
  test::assert_bool(cache.Contains(tile3_id), "tile3 not found");

  // Add an insertion which now requires the eviction
  // The first inserted items should be evicted here (id1)
  // Note, we are not inserting a new entry but updating the existing one (id2)
  // The new size is picked such that it should be the only item remaining in the cache

  const size_t tile4_size = 480;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile4_size), tile4_size);

  test::assert_bool(!cache.Contains(tile1_id), "tile1 is not evicted");
  test::assert_bool(!cache.Contains(tile3_id), "tile3 is not evicted");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");

  test::assert_bool(cache.Get(tile1_id) == nullptr, "tile1 should be evicted");
  test::assert_bool(cache.Get(tile3_id) == nullptr, "tile3 should be evicted");
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile4_size);
}

void Test_TileCacheLRU_HARD_InsertWithEvictionEntireCache() {
  TileCacheLRU cache(1000, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 300;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 900;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile3_size), tile3_size);

  test::assert_bool(cache.Contains(tile3_id), "tile3 not found");
  test::assert_bool(!cache.Contains(tile1_id), "tile1 found");
  test::assert_bool(!cache.Contains(tile2_id), "tile2 found");

  const auto* returned3 = cache.Get(tile3_id);
  CheckGraphTile(returned3, tile3_id, tile3_size);

  test::assert_bool(cache.Get(tile1_id) == nullptr, "tile1 get should return null");
  test::assert_bool(cache.Get(tile2_id) == nullptr, "tile2 get should return null");
}

void Test_TileCacheLRU_HARD_MixedInsertOverwrite() {
  TileCacheLRU cache(4000, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 1000;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 300;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 900;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile3_size), tile3_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile3_size);

  const size_t tile4_size = 123;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile4_size), tile4_size);

  GraphId tile5_id(1234, 1, 0);
  const size_t tile5_size = 200;
  cache.Put(tile5_id, TestGraphTile(tile5_id, tile5_size), tile5_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile4_size);
  CheckGraphTile(cache.Get(tile5_id), tile5_id, tile5_size);
}

void Test_TileCacheLRU_HARD_MixedInsertOverwriteEvict() {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  // bump tile1 (with get)
  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 45;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile3_size), tile3_size);

  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");
  test::assert_bool(cache.Contains(tile3_id), "tile3 not found");

  // Add an insertion which now requires the eviction
  // Expecting id2 to get evicted
  // Note, we are not inserting a new entry but updating the existing one (id3)

  const size_t tile4_size = 255;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile4_size), tile4_size);

  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(!cache.Contains(tile2_id), "tile2 is not evicted");
  test::assert_bool(cache.Contains(tile3_id), "tile3 not found");

  test::assert_bool(cache.Get(tile2_id) == nullptr, "tile2 should be evicted");
  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile4_size);
}

void Test_TileCacheLRU_HARD_ClearBasic() {
  TileCacheLRU cache(2000, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(10, 1, 0);
  const size_t tile1_size = 500;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 123;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");

  cache.Clear();

  test::assert_bool(!cache.Contains(tile1_id), "tile1 found");
  test::assert_bool(!cache.Contains(tile2_id), "tile2 found");
  test::assert_bool(cache.Get(tile1_id) == nullptr, "not expecting ot get tile1");
  test::assert_bool(cache.Get(tile2_id) == nullptr, "not expecting ot get tile2");
}

void Test_TileCacheLRU_HARD_TrimBasic() {
  // Trim should not have any effect on the cache with hard memory limit policy
  TileCacheLRU cache(2000, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(10, 1, 0);
  const size_t tile1_size = 500;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 123;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");

  cache.Trim();

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");
}

void Test_TileCacheLRU_SOFT_InsertBecomeOvercommittedTrim() {
  TileCacheLRU cache(2000, TileCacheLRU::MemoryLimitControl::SOFT);

  GraphId tile1_id(10, 1, 0);
  const size_t tile1_size = 1500;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 2000;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  test::assert_bool(cache.OverCommitted(), "unexpected overcommit");

  GraphId tile3_id(500, 1, 0);
  const size_t tile3_size = 100;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile3_size), tile3_size);

  test::assert_bool(cache.OverCommitted(), "unexpected overcommit");

  // With soft memory limit strategy there should not be any evictions here
  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile3_size);
  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");
  test::assert_bool(cache.Contains(tile3_id), "tile3 not found");

  cache.Trim();

  test::assert_bool(cache.Get(tile1_id) == nullptr, "not expecting ot get tile1");
  test::assert_bool(cache.Get(tile2_id) == nullptr, "not expecting ot get tile2");
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile3_size);
  test::assert_bool(!cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(!cache.Contains(tile2_id), "tile2 found");
  test::assert_bool(cache.Contains(tile3_id), "tile3 found");
}

void Test_TileCacheLRU_SOFT_InsertBecomeOvercommittedClear() {
  TileCacheLRU cache(2000, TileCacheLRU::MemoryLimitControl::SOFT);

  GraphId tile1_id(10, 1, 0);
  const size_t tile1_size = 1500;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 2000;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  test::assert_bool(cache.OverCommitted(), "unexpected overcommit");

  GraphId tile3_id(500, 1, 0);
  const size_t tile3_size = 100;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile3_size), tile3_size);

  test::assert_bool(cache.OverCommitted(), "unexpected overcommit");

  // With soft memory limit strategy there should not be any evictions here
  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile3_size);
  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");
  test::assert_bool(cache.Contains(tile3_id), "tile3 not found");

  cache.Clear();

  // Expecting clear to remove everything
  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  test::assert_bool(cache.Get(tile1_id) == nullptr, "not expecting ot get tile1");
  test::assert_bool(cache.Get(tile2_id) == nullptr, "not expecting ot get tile2");
  test::assert_bool(cache.Get(tile3_id) == nullptr, "not expecting ot get tile1");
  test::assert_bool(!cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(!cache.Contains(tile2_id), "tile2 found");
  test::assert_bool(!cache.Contains(tile3_id), "tile3 found");
}

void Test_TileCacheLRU_SOFT_UndercommittedTrim() {
  TileCacheLRU cache(5000, TileCacheLRU::MemoryLimitControl::SOFT);

  GraphId tile1_id(10, 1, 0);
  const size_t tile1_size = 300;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 2000;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  // With soft memory limit strategy there should not be any evictions here
  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");

  cache.Trim();

  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");
}

void Test_TileCacheLRU_SOFT_InsertWithEvictionBasic() {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::SOFT);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 50;
  cache.Put(tile3_id, TestGraphTile(tile3_id, tile3_size), tile3_size);

  GraphId tile4_id(400, 2, 0);
  const size_t tile4_size = 270;
  cache.Put(tile4_id, TestGraphTile(tile4_id, tile4_size), tile4_size);

  test::assert_bool(cache.OverCommitted(), "should be overcommit");

  // Now we access an entry that would be evicted next
  // to promote its position in the LRU list and change eviction order
  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);

  // should evict tiles 2 and 3
  cache.Trim();

  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  test::assert_bool(cache.Contains(tile1_id), "tile1 is not evicted");
  test::assert_bool(!cache.Contains(tile2_id), "tile2 not found");
  test::assert_bool(!cache.Contains(tile3_id), "tile3 is not evicted");
  test::assert_bool(cache.Contains(tile4_id), "tile4 not found");

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile4_id), tile4_id, tile4_size);
  test::assert_bool(cache.Get(tile2_id) == nullptr, "tile1 get should return null");
  test::assert_bool(cache.Get(tile3_id) == nullptr, "tile3 get should return null");
}

void Test_TileCacheLRU_SOFT_TrimOnExactlyFullCache() {
  TileCacheLRU cache(100000, TileCacheLRU::MemoryLimitControl::SOFT);

  GraphId tile1_id(10, 1, 0);
  const size_t tile1_size = 60000;
  cache.Put(tile1_id, TestGraphTile(tile1_id, tile1_size), tile1_size);

  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 40000;
  cache.Put(tile2_id, TestGraphTile(tile2_id, tile2_size), tile2_size);

  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  test::assert_bool(cache.Contains(tile1_id), "tile1 not found");
  test::assert_bool(cache.Contains(tile2_id), "tile2 not found");

  cache.Trim();

  // Not expecting any evictions if the cache does not cross the memory limit
  test::assert_bool(!cache.OverCommitted(), "unexpected overcommit");
  test::assert_bool(cache.Contains(tile1_id), "tile1 evicted but shouldn't");
  test::assert_bool(cache.Contains(tile2_id), "tile2 evicted but shouldn't");
  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
}

} // namespace

int main() {
  test::suite suite("graphtile");

  suite.test(TEST_CASE(TestOutOfRangeLL));

  suite.test(TEST_CASE(TestConnectivityMap));

  // SimpleTileCahe unit tests
  suite.test(TEST_CASE(TestCacheLimits));
  suite.test(TEST_CASE(Test_SimpleTileCache_Clear));
  suite.test(TEST_CASE(Test_SimpleTileCache_Trim));

  // LRU tile cache unit tests
  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_Creation));
  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_InsertSingleItemBiggerThanCacheSize));
  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_InsertNoEviction));
  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_InsertCacheFullOneshot));
  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_InsertCacheFull));
  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_InsertWithEvictionBasic));
  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_InsertWithEvictionEntireCache));

  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_MixedInsertOverwrite));
  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_MixedInsertOverwriteEvict));

  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_OverwriteSameSize));
  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_OverwriteSmallerSize));
  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_OverwriteBiggerSizeNoEviction));
  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_OverwriteBiggerSizeEvictionOne));
  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_OverwriteBiggerSizeEvictionMultiple));

  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_ClearBasic));
  suite.test(TEST_CASE(Test_TileCacheLRU_HARD_TrimBasic));

  suite.test(TEST_CASE(Test_TileCacheLRU_SOFT_InsertBecomeOvercommittedTrim));
  suite.test(TEST_CASE(Test_TileCacheLRU_SOFT_InsertBecomeOvercommittedClear));
  suite.test(TEST_CASE(Test_TileCacheLRU_SOFT_UndercommittedTrim));
  suite.test(TEST_CASE(Test_TileCacheLRU_SOFT_InsertWithEvictionBasic));
  suite.test(TEST_CASE(Test_TileCacheLRU_SOFT_TrimOnExactlyFullCache));

  return suite.tear_down();
}
