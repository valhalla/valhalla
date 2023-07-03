#include <cstdint>

#include "baldr/connectivity_map.h"
#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"

#include <fcntl.h>

#include "test.h"

using namespace valhalla::baldr;

namespace {

class test_cache : public SimpleTileCache {
public:
  using SimpleTileCache::cache_size_;
  using SimpleTileCache::max_cache_size_;
  using SimpleTileCache::SimpleTileCache;
};

TEST(SimpleCache, QueryByPointOutOfRangeLL) {
  boost::property_tree::ptree pt;
  pt.put("tile_dir", "test/gphrdr_test");
  GraphReader reader(pt);

  // Latitude out of range
  EXPECT_TRUE(reader.GetGraphTile({60.0f, 100.0f}) == nullptr)
      << "Out of range LL should return nullptr";

  // Out of range longitude
  EXPECT_TRUE(reader.GetGraphTile({460.0f, 60.0f}) == nullptr)
      << "Out of range LL should return nullptr";
}

test_cache make_cache(size_t cache_size) {
  return {cache_size};
}

TEST(SimpleCache, CacheLimitsZeroSizeOvercommited) {
  EXPECT_FALSE(make_cache(0).OverCommitted());
}

TEST(SimpleCache, CacheLimitsMinSizeOvercommited) {
  EXPECT_FALSE(make_cache(1).OverCommitted());
}

TEST(SimpleCache, CacheLimitsOvercommitBasic) {
  auto cache = make_cache(10);
  cache.cache_size_ = 20;
  EXPECT_TRUE(cache.OverCommitted());

  cache.cache_size_ = 1;
  cache.max_cache_size_ = 0;
  EXPECT_TRUE(cache.OverCommitted());
}

TEST(SimpleCache, CacheLimitsNoOvercommitAfterClear) {
  auto cache = make_cache(10);
  cache.cache_size_ = cache.max_cache_size_ + 1;
  EXPECT_TRUE(cache.OverCommitted());
  cache.Clear();
  EXPECT_FALSE(cache.OverCommitted());
}

void touch_tile(const uint32_t tile_id, const std::string& tile_dir, uint8_t level) {
  auto suffix = GraphTile::FileSuffix({tile_id, level, 0});
  auto fullpath = tile_dir + filesystem::path::preferred_separator + suffix;
  filesystem::create_directories(filesystem::path(fullpath).parent_path());
  int fd = open(fullpath.c_str(), O_CREAT | O_WRONLY, 0644);
  if (fd >= 0)
    close(fd);
}

TEST(ConnectivityMap, Basic) {
  // get the property tree to create some tiles
  boost::property_tree::ptree pt;
  pt.put("tile_dir", "test/gphrdr_test");
  std::string tile_dir = pt.get<std::string>("tile_dir");
  for (const auto& level : {TileHierarchy::levels()[2], TileHierarchy::GetTransitLevel()}) {
    filesystem::remove_all(tile_dir);

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
    touch_tile(a0, tile_dir, level.level);

    uint32_t a1 = level.tiles.RightNeighbor(a0);
    touch_tile(a1, tile_dir, level.level);

    uint32_t a2 = level.tiles.TopNeighbor(a0);
    touch_tile(a2, tile_dir, level.level);

    uint32_t b0 = level.tiles.RightNeighbor(level.tiles.RightNeighbor(a1));
    touch_tile(b0, tile_dir, level.level);

    uint32_t c0 = level.tiles.TopNeighbor(level.tiles.RightNeighbor(a1));
    touch_tile(c0, tile_dir, level.level);

    uint32_t d0 = level.tiles.TopNeighbor(level.tiles.RightNeighbor(a2));
    touch_tile(d0, tile_dir, level.level);

    uint32_t d1 = level.tiles.TopNeighbor(d0);
    touch_tile(d1, tile_dir, level.level);

    // check that it looks right
    connectivity_map_t conn(pt);

    EXPECT_EQ(conn.get_color({a0, level.level, 0}), conn.get_color({a1, level.level, 0}))
        << "a's should be connected";
    EXPECT_EQ(conn.get_color({a0, level.level, 0}), conn.get_color({a2, level.level, 0}))
        << "a's should be connected";
    EXPECT_EQ(conn.get_color({a1, level.level, 0}), conn.get_color({a2, level.level, 0}))
        << "a's should be connected";
    EXPECT_EQ(conn.get_color({d0, level.level, 0}), conn.get_color({d1, level.level, 0}))
        << "d's should be connected";
    EXPECT_NE(conn.get_color({c0, level.level, 0}), conn.get_color({a1, level.level, 0}))
        << "c is disjoint";
    EXPECT_NE(conn.get_color({b0, level.level, 0}), conn.get_color({a0, level.level, 0}))
        << "b is disjoint";
    EXPECT_NE(conn.get_color({a2, level.level, 0}), conn.get_color({d0, level.level, 0}))
        << "a is disjoint from d";

    filesystem::remove_all(tile_dir);
  }
}

class TestGraphMemory final : public GraphMemory {
public:
  TestGraphMemory() : memory_(sizeof(GraphTileHeader)) {
    data = const_cast<char*>(memory_.data());
    size = memory_.size();
  }

private:
  const std::vector<char> memory_;
};

struct TestGraphTile : public GraphTile {
  TestGraphTile(GraphId id, size_t size) {
    memory_ = std::make_unique<const TestGraphMemory>();
    header_ = reinterpret_cast<GraphTileHeader*>(memory_->data);
    header_->set_graphid(id);
    header_->set_end_offset(size);
  }
};

static void
CheckGraphTile(const graph_tile_ptr& tile, const GraphId& expected_id, size_t expected_size) {
  ASSERT_NE(tile, nullptr);
  EXPECT_EQ(tile->header()->graphid().value, expected_id.value);
  EXPECT_EQ(tile->header()->end_offset(), expected_size);
}

TEST(SimpleCache, Clear) {
  SimpleTileCache cache(400);

  GraphId id1(100, 2, 0);
  auto tile1 = cache.Put(id1, graph_tile_ptr{new TestGraphTile(id1, 123)}, 123);
  EXPECT_EQ(cache.Get(id1), tile1);
  CheckGraphTile(tile1, id1, 123);

  EXPECT_FALSE(cache.OverCommitted());

  GraphId id2(300, 1, 0);
  graph_tile_ptr tile2 = cache.Put(id2, graph_tile_ptr{new TestGraphTile(id2, 200)}, 200);
  EXPECT_EQ(cache.Get(id2), tile2);
  CheckGraphTile(tile2, id2, 200);

  EXPECT_FALSE(cache.OverCommitted());

  GraphId id3(1000, 0, 0);
  auto tile3 = cache.Put(id3, graph_tile_ptr{new TestGraphTile(id3, 500)}, 500);
  EXPECT_EQ(cache.Get(id3), tile3);
  CheckGraphTile(tile3, id3, 500);

  EXPECT_TRUE(cache.OverCommitted());

  // Check if inserted values are correct

  auto returned2 = cache.Get({300, 1, 0});
  CheckGraphTile(returned2, id2, 200);
  auto returned1 = cache.Get({100, 2, 0});
  CheckGraphTile(returned1, id1, 123);
  auto returned3 = cache.Get({1000, 0, 0});
  CheckGraphTile(returned3, id3, 500);

  EXPECT_TRUE(cache.Contains(id1));
  EXPECT_TRUE(cache.Contains(id2));
  EXPECT_TRUE(cache.Contains(id3));

  cache.Clear();

  EXPECT_FALSE(cache.OverCommitted());

  EXPECT_FALSE(cache.Contains(id1));
  EXPECT_FALSE(cache.Contains(id2));
  EXPECT_FALSE(cache.Contains(id3));

  EXPECT_EQ(cache.Get(id1), nullptr);
  EXPECT_EQ(cache.Get(id2), nullptr);
  EXPECT_EQ(cache.Get(id3), nullptr);
}

TEST(SimpleCache, Trim) {
  SimpleTileCache cache(400);

  GraphId id1(100, 2, 0);
  auto tile1 = cache.Put(id1, graph_tile_ptr{new TestGraphTile(id1, 123)}, 123);
  EXPECT_EQ(cache.Get(id1), tile1);
  CheckGraphTile(tile1, id1, 123);

  EXPECT_FALSE(cache.OverCommitted());

  GraphId id2(300, 1, 0);
  auto tile2 = cache.Put(id2, graph_tile_ptr{new TestGraphTile(id2, 200)}, 200);
  EXPECT_EQ(cache.Get(id2), tile2);
  CheckGraphTile(tile2, id2, 200);

  EXPECT_FALSE(cache.OverCommitted());

  GraphId id3(1000, 0, 0);
  auto tile3 = cache.Put(id3, graph_tile_ptr{new TestGraphTile(id3, 500)}, 500);
  EXPECT_EQ(cache.Get(id3), tile3);
  CheckGraphTile(tile3, id3, 500);

  EXPECT_TRUE(cache.OverCommitted());

  // Check if inserted values are correct

  auto returned2 = cache.Get({300, 1, 0});
  CheckGraphTile(returned2, id2, 200);
  auto returned1 = cache.Get({100, 2, 0});
  CheckGraphTile(returned1, id1, 123);
  auto returned3 = cache.Get({1000, 0, 0});
  CheckGraphTile(returned3, id3, 500);

  EXPECT_TRUE(cache.Contains(id1));
  EXPECT_TRUE(cache.Contains(id2));
  EXPECT_TRUE(cache.Contains(id3));

  cache.Trim();

  EXPECT_FALSE(cache.OverCommitted());

  EXPECT_FALSE(cache.Contains(id1));
  EXPECT_FALSE(cache.Contains(id2));
  EXPECT_FALSE(cache.Contains(id3));

  EXPECT_EQ(cache.Get(id1), nullptr);
  EXPECT_EQ(cache.Get(id2), nullptr);
  EXPECT_EQ(cache.Get(id3), nullptr);
}

TEST(CacheLruHard, Creation) {
  TileCacheLRU zero_limit_cache(0, TileCacheLRU::MemoryLimitControl::HARD);
  TileCacheLRU cache(1023, TileCacheLRU::MemoryLimitControl::HARD);
  TileCacheLRU big_cache(1073741824, TileCacheLRU::MemoryLimitControl::HARD);
}

TEST(CacheLruHard, InsertSingleItemBiggerThanCacheSize) {
  TileCacheLRU cache(1023, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId id1(100, 2, 0);

  EXPECT_THROW(cache.Put(id1, graph_tile_ptr{new TestGraphTile(id1, 2000)}, 2000),
               std::runtime_error);
  EXPECT_EQ(cache.Get(id1), nullptr);
  EXPECT_FALSE(cache.Contains(id1));
}

TEST(CacheLruHard, InsertCacheFullOneshot) {
  const size_t tile1_size = 1234;

  TileCacheLRU cache(tile1_size, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  auto tile1 =
      cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);
  EXPECT_EQ(cache.Get(tile1_id), tile1);
  EXPECT_FALSE(cache.OverCommitted());

  CheckGraphTile(tile1, tile1_id, tile1_size);
  EXPECT_TRUE(cache.Contains(tile1_id));
}

TEST(CacheLruHard, InsertCacheFull) {
  TileCacheLRU cache(10000, TileCacheLRU::MemoryLimitControl::HARD);

  const size_t tile1_size = 4000;
  GraphId tile1_id(1000, 1, 0);
  auto tile1 =
      cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);
  EXPECT_EQ(cache.Get(tile1_id), tile1);
  CheckGraphTile(tile1, tile1_id, tile1_size);

  const size_t tile2_size = 6000;
  GraphId tile2_id(33, 2, 0);
  auto tile2 =
      cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);
  EXPECT_EQ(cache.Get(tile2_id), tile2);
  CheckGraphTile(tile2, tile2_id, tile2_size);

  EXPECT_FALSE(cache.OverCommitted());

  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
}

TEST(CacheLruHard, InsertNoEviction) {
  TileCacheLRU cache(1023, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId id1(100, 2, 0);
  auto tile1 = cache.Put(id1, graph_tile_ptr{new TestGraphTile(id1, 123)}, 123);
  EXPECT_EQ(cache.Get(id1), tile1);
  CheckGraphTile(tile1, id1, 123);

  GraphId id2(300, 1, 0);
  auto tile2 = cache.Put(id2, graph_tile_ptr{new TestGraphTile(id2, 200)}, 200);
  EXPECT_EQ(cache.Get(id2), tile2);
  CheckGraphTile(tile2, id2, 200);

  GraphId id3(1000, 0, 0);
  auto tile3 = cache.Put(id3, graph_tile_ptr{new TestGraphTile(id3, 500)}, 500);
  EXPECT_EQ(cache.Get(id3), tile3);
  CheckGraphTile(tile3, id3, 500);

  // Check if inserted values are correct

  auto returned2 = cache.Get({300, 1, 0});
  CheckGraphTile(returned2, id2, 200);

  auto returned1 = cache.Get({100, 2, 0});
  CheckGraphTile(returned1, id1, 123);

  auto returned3 = cache.Get({1000, 0, 0});
  CheckGraphTile(returned3, id3, 500);

  // Make sure tiles we have never cached are not found
  EXPECT_EQ(cache.Get({1345, 1, 0}), nullptr);
  EXPECT_EQ(cache.Get({100, 1, 0}), nullptr);
  EXPECT_EQ(cache.Get({0, 0, 0}), nullptr);
}

TEST(CacheLruHard, InsertWithEvictionBasic) {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 45;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile3_size)}, tile3_size);

  EXPECT_TRUE(cache.Contains(tile3_id));
  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));

  // Add an insertion which now requires the eviction
  // The first inserted items should be evicted here (id1)

  GraphId tile4_id(400, 2, 0);
  const size_t tile4_size = 20;
  cache.Put(tile4_id, graph_tile_ptr{new TestGraphTile(tile4_id, tile4_size)}, tile4_size);

  EXPECT_FALSE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
  EXPECT_TRUE(cache.Contains(tile3_id));
  EXPECT_TRUE(cache.Contains(tile4_id));

  // Now we access an entry that would be evicted next
  // to promote its position in the LRU list and change eviction order

  auto returned2 = cache.Get(tile2_id);
  CheckGraphTile(returned2, tile2_id, tile2_size);

  GraphId tile5_id(999, 1, 0);
  const size_t tile5_size = 200;
  cache.Put(tile5_id, graph_tile_ptr{new TestGraphTile(tile5_id, tile5_size)}, tile5_size);

  // Expecting the next eviction of tile3 since tile1 has been just accessed

  EXPECT_FALSE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
  EXPECT_FALSE(cache.Contains(tile3_id));
  EXPECT_TRUE(cache.Contains(tile4_id));
  EXPECT_TRUE(cache.Contains(tile5_id));

  EXPECT_EQ(cache.Get(tile1_id), nullptr);
  EXPECT_EQ(cache.Get(tile3_id), nullptr);

  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  CheckGraphTile(cache.Get(tile4_id), tile4_id, tile4_size);
  CheckGraphTile(cache.Get(tile5_id), tile5_id, tile5_size);
}

TEST(CacheLruHard, OverwriteSameSize) {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 45;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile3_size)}, tile3_size);

  // overwrite with a new object and same size
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile3_size);
  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
  EXPECT_TRUE(cache.Contains(tile3_id));
}

TEST(CacheLruHard, OverwriteSmallerSize) {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 45;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile3_size)}, tile3_size);

  const size_t tile4_size = 8;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile4_size)}, tile4_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile4_size);
  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
  EXPECT_TRUE(cache.Contains(tile3_id));
}

TEST(CacheLruHard, OverwriteBiggerSizeNoEviction) {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 20;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile3_size)}, tile3_size);

  const size_t tile4_size = 45;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile4_size)}, tile4_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile4_size);
  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
  EXPECT_TRUE(cache.Contains(tile3_id));
}

TEST(CacheLruHard, OverwriteBiggerSizeEvictionOne) {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 45;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile3_size)}, tile3_size);

  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
  EXPECT_TRUE(cache.Contains(tile3_id));

  // Add an insertion which now requires the eviction
  // The first inserted items should be evicted here (id1)
  // Note, we are not inserting a new entry but updating the existing one (id2)

  const size_t tile4_size = 260;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile4_size)}, tile4_size);

  EXPECT_FALSE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
  EXPECT_TRUE(cache.Contains(tile3_id));

  EXPECT_EQ(cache.Get(tile1_id), nullptr);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile4_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile3_size);
}

TEST(CacheLruHard, OverwriteBiggerSizeEvictionMultiple) {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 45;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile3_size)}, tile3_size);

  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
  EXPECT_TRUE(cache.Contains(tile3_id));

  // Add an insertion which now requires the eviction
  // The first inserted items should be evicted here (id1)
  // Note, we are not inserting a new entry but updating the existing one (id2)
  // The new size is picked such that it should be the only item remaining in the cache

  const size_t tile4_size = 480;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile4_size)}, tile4_size);

  EXPECT_FALSE(cache.Contains(tile1_id));
  EXPECT_FALSE(cache.Contains(tile3_id));
  EXPECT_TRUE(cache.Contains(tile2_id));

  EXPECT_EQ(cache.Get(tile1_id), nullptr);
  EXPECT_EQ(cache.Get(tile3_id), nullptr);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile4_size);
}

TEST(CacheLruHard, InsertWithEvictionEntireCache) {
  TileCacheLRU cache(1000, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 300;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 900;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile3_size)}, tile3_size);

  EXPECT_TRUE(cache.Contains(tile3_id));
  EXPECT_FALSE(cache.Contains(tile1_id));
  EXPECT_FALSE(cache.Contains(tile2_id));

  auto returned3 = cache.Get(tile3_id);
  CheckGraphTile(returned3, tile3_id, tile3_size);

  EXPECT_EQ(cache.Get(tile1_id), nullptr);
  EXPECT_EQ(cache.Get(tile2_id), nullptr);
}

TEST(CacheLruHard, MixedInsertOverwrite) {
  TileCacheLRU cache(4000, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 1000;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 300;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 900;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile3_size)}, tile3_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile3_size);

  const size_t tile4_size = 123;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile4_size)}, tile4_size);

  GraphId tile5_id(1234, 1, 0);
  const size_t tile5_size = 200;
  cache.Put(tile5_id, graph_tile_ptr{new TestGraphTile(tile5_id, tile5_size)}, tile5_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile4_size);
  CheckGraphTile(cache.Get(tile5_id), tile5_id, tile5_size);
}

TEST(CacheLruHard, MixedInsertOverwriteEvict) {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  // bump tile1 (with get)
  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 45;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile3_size)}, tile3_size);

  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
  EXPECT_TRUE(cache.Contains(tile3_id));

  // Add an insertion which now requires the eviction
  // Expecting id2 to get evicted
  // Note, we are not inserting a new entry but updating the existing one (id3)

  const size_t tile4_size = 255;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile4_size)}, tile4_size);

  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_FALSE(cache.Contains(tile2_id));
  EXPECT_TRUE(cache.Contains(tile3_id));

  EXPECT_EQ(cache.Get(tile2_id), nullptr);
  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile4_size);
}

TEST(CacheLruHard, ClearBasic) {
  TileCacheLRU cache(2000, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(10, 1, 0);
  const size_t tile1_size = 500;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 123;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));

  cache.Clear();

  EXPECT_FALSE(cache.Contains(tile1_id));
  EXPECT_FALSE(cache.Contains(tile2_id));
  EXPECT_EQ(cache.Get(tile1_id), nullptr);
  EXPECT_EQ(cache.Get(tile2_id), nullptr);
}

TEST(CacheLruHard, TrimBasic) {
  // Trim should not have any effect on the cache with hard memory limit policy
  TileCacheLRU cache(2000, TileCacheLRU::MemoryLimitControl::HARD);

  GraphId tile1_id(10, 1, 0);
  const size_t tile1_size = 500;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 123;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));

  cache.Trim();

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
}

TEST(CacheLruSoft, InsertBecomeOvercommittedTrim) {
  TileCacheLRU cache(2000, TileCacheLRU::MemoryLimitControl::SOFT);

  GraphId tile1_id(10, 1, 0);
  const size_t tile1_size = 1500;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  EXPECT_FALSE(cache.OverCommitted());

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 2000;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  EXPECT_TRUE(cache.OverCommitted());

  GraphId tile3_id(500, 1, 0);
  const size_t tile3_size = 100;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile3_size)}, tile3_size);

  EXPECT_TRUE(cache.OverCommitted());

  // With soft memory limit strategy there should not be any evictions here
  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile3_size);
  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
  EXPECT_TRUE(cache.Contains(tile3_id));

  cache.Trim();

  EXPECT_EQ(cache.Get(tile1_id), nullptr);
  EXPECT_EQ(cache.Get(tile2_id), nullptr);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile3_size);
  EXPECT_FALSE(cache.Contains(tile1_id));
  EXPECT_FALSE(cache.Contains(tile2_id));
  EXPECT_TRUE(cache.Contains(tile3_id));
}

TEST(CacheLruSoft, InsertBecomeOvercommittedClear) {
  TileCacheLRU cache(2000, TileCacheLRU::MemoryLimitControl::SOFT);

  GraphId tile1_id(10, 1, 0);
  const size_t tile1_size = 1500;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  EXPECT_FALSE(cache.OverCommitted());

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 2000;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  EXPECT_TRUE(cache.OverCommitted());

  GraphId tile3_id(500, 1, 0);
  const size_t tile3_size = 100;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile3_size)}, tile3_size);

  EXPECT_TRUE(cache.OverCommitted());

  // With soft memory limit strategy there should not be any evictions here
  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  CheckGraphTile(cache.Get(tile3_id), tile3_id, tile3_size);
  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
  EXPECT_TRUE(cache.Contains(tile3_id));

  cache.Clear();

  // Expecting clear to remove everything
  EXPECT_FALSE(cache.OverCommitted());

  EXPECT_EQ(cache.Get(tile1_id), nullptr);
  EXPECT_EQ(cache.Get(tile2_id), nullptr);
  EXPECT_EQ(cache.Get(tile3_id), nullptr);
  EXPECT_FALSE(cache.Contains(tile1_id));
  EXPECT_FALSE(cache.Contains(tile2_id));
  EXPECT_FALSE(cache.Contains(tile3_id));
}

TEST(CacheLruSoft, UndercommittedTrim) {
  TileCacheLRU cache(5000, TileCacheLRU::MemoryLimitControl::SOFT);

  GraphId tile1_id(10, 1, 0);
  const size_t tile1_size = 300;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  EXPECT_FALSE(cache.OverCommitted());

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 2000;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  EXPECT_FALSE(cache.OverCommitted());

  // With soft memory limit strategy there should not be any evictions here
  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));

  cache.Trim();

  EXPECT_FALSE(cache.OverCommitted());

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
}

TEST(CacheLruSoft, InsertWithEvictionBasic) {
  TileCacheLRU cache(500, TileCacheLRU::MemoryLimitControl::SOFT);

  GraphId tile1_id(1000, 1, 0);
  const size_t tile1_size = 200;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 250;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  GraphId tile3_id(1, 1, 0);
  const size_t tile3_size = 50;
  cache.Put(tile3_id, graph_tile_ptr{new TestGraphTile(tile3_id, tile3_size)}, tile3_size);

  GraphId tile4_id(400, 2, 0);
  const size_t tile4_size = 270;
  cache.Put(tile4_id, graph_tile_ptr{new TestGraphTile(tile4_id, tile4_size)}, tile4_size);

  EXPECT_TRUE(cache.OverCommitted());

  // Now we access an entry that would be evicted next
  // to promote its position in the LRU list and change eviction order
  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);

  // should evict tiles 2 and 3
  cache.Trim();

  EXPECT_FALSE(cache.OverCommitted());

  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_FALSE(cache.Contains(tile2_id));
  EXPECT_FALSE(cache.Contains(tile3_id));
  EXPECT_TRUE(cache.Contains(tile4_id));

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile4_id), tile4_id, tile4_size);
  EXPECT_EQ(cache.Get(tile2_id), nullptr);
  EXPECT_EQ(cache.Get(tile3_id), nullptr);
}

TEST(CacheLruSoft, TrimOnExactlyFullCache) {
  TileCacheLRU cache(100000, TileCacheLRU::MemoryLimitControl::SOFT);

  GraphId tile1_id(10, 1, 0);
  const size_t tile1_size = 60000;
  cache.Put(tile1_id, graph_tile_ptr{new TestGraphTile(tile1_id, tile1_size)}, tile1_size);

  EXPECT_FALSE(cache.OverCommitted());

  GraphId tile2_id(300, 2, 0);
  const size_t tile2_size = 40000;
  cache.Put(tile2_id, graph_tile_ptr{new TestGraphTile(tile2_id, tile2_size)}, tile2_size);

  EXPECT_FALSE(cache.OverCommitted());

  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));

  cache.Trim();

  // Not expecting any evictions if the cache does not cross the memory limit
  EXPECT_FALSE(cache.OverCommitted());
  EXPECT_TRUE(cache.Contains(tile1_id));
  EXPECT_TRUE(cache.Contains(tile2_id));
  CheckGraphTile(cache.Get(tile1_id), tile1_id, tile1_size);
  CheckGraphTile(cache.Get(tile2_id), tile2_id, tile2_size);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
