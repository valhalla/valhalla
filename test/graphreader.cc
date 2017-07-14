#include <cstdint>
#include "test.h"

#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "baldr/connectivity_map.h"

#include <fcntl.h>
#include <boost/filesystem.hpp>

using namespace std;
using namespace valhalla::baldr;

namespace {

class test_cache : public SimpleTileCache {
 public:
  using SimpleTileCache::SimpleTileCache;
  using SimpleTileCache::cache_size_;
  using SimpleTileCache::max_cache_size_;
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
  if(make_cache(0).OverCommitted())
    throw std::runtime_error("Cache should be over committed");

  if(make_cache(1).OverCommitted())
    throw std::runtime_error("Cache should be under committed");

  auto cache = make_cache(10);
  cache.cache_size_ = cache.max_cache_size_ + 1;
  if(!cache.OverCommitted())
    throw std::runtime_error("Cache should be over committed");
  cache.Clear();
  if(cache.OverCommitted())
    throw std::runtime_error("Cache should be under committed");

  cache.cache_size_ = 1;
  cache.max_cache_size_ = 0;
  if(!cache.OverCommitted())
    throw std::runtime_error("Cache should be over committed");
}

void touch_tile(const uint32_t tile_id, const std::string& tile_dir) {
  auto suffix = GraphTile::FileSuffix({tile_id, 2, 0});
  auto fullpath = tile_dir + '/' + suffix;
  boost::filesystem::create_directories(boost::filesystem::path(fullpath).parent_path());
  int fd = open(fullpath.c_str(), O_CREAT | O_WRONLY, 0644);
  if(fd >= 0)
    close(fd);
}

void TestConnectivityMap() {
  //get the property tree to create some tiles
  boost::property_tree::ptree pt;
  pt.put("tile_dir", "test/gphrdr_test");
  std::string tile_dir = pt.get<std::string>("tile_dir");
  const auto& level = TileHierarchy::levels().find(2)->second;
  boost::filesystem::remove_all(tile_dir);

  //looks like this (XX) means no tile there:
  /*
   *     XX d1 XX XX
   *     XX d0 XX XX
   *     a2 XX c0 XX
   *     a0 a1 XX b0
   *
   */

  //create some empty files with tile names
  uint32_t a0 = 0;
  touch_tile(a0, tile_dir);

  uint32_t a1 = level.tiles.RightNeighbor(a0);
  touch_tile(a1, tile_dir);

  uint32_t a2 = level.tiles.TopNeighbor(a0);
  touch_tile(a2, tile_dir);

  uint32_t b0 = level.tiles.RightNeighbor(level.tiles.RightNeighbor(a1));
  touch_tile(b0, tile_dir);

  uint32_t c0 =  level.tiles.TopNeighbor(level.tiles.RightNeighbor(a1));
  touch_tile(c0, tile_dir);

  uint32_t d0 = level.tiles.TopNeighbor(level.tiles.RightNeighbor(a2));
  touch_tile(d0, tile_dir);

  uint32_t d1 = level.tiles.TopNeighbor(d0);
  touch_tile(d1, tile_dir);

  //check that it looks right
  connectivity_map_t conn(pt);
  if(conn.get_color({a0, 2, 0}) != conn.get_color({a1, 2, 0}))
    throw std::runtime_error("a's should be connected");
  if(conn.get_color({a0, 2, 0}) != conn.get_color({a2, 2, 0}))
    throw std::runtime_error("a's should be connected");
  if(conn.get_color({a1, 2, 0}) != conn.get_color({a2, 2, 0}))
    throw std::runtime_error("a's should be connected");
  if(conn.get_color({d0, 2, 0}) != conn.get_color({d1, 2, 0}))
    throw std::runtime_error("d's should be connected");
  if(conn.get_color({c0, 2, 0}) == conn.get_color({a1, 2, 0}))
    throw std::runtime_error("c is disjoint");
  if(conn.get_color({b0, 2, 0}) == conn.get_color({a0, 2, 0}))
    throw std::runtime_error("b is disjoint");
  if(conn.get_color({a2, 2, 0}) == conn.get_color({d0, 2, 0}))
    throw std::runtime_error("a is disjoint from d");

  boost::filesystem::remove_all(tile_dir);
}

}

int main() {
  test::suite suite("graphtile");

  suite.test(TEST_CASE(TestOutOfRangeLL));

  suite.test(TEST_CASE(TestCacheLimits));

  suite.test(TEST_CASE(TestConnectivityMap));

  return suite.tear_down();
}
