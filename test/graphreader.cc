#include "test.h"

#include "baldr/graphreader.h"

#include <fcntl.h>
#include <boost/filesystem.hpp>

using namespace std;
using namespace valhalla::baldr;

namespace {

class test_reader : public GraphReader {
 public:
  using GraphReader::GraphReader;
  using GraphReader::cache_size_;
  using GraphReader::max_cache_size_;
};

test_reader make_cache(std::string cache_size) {
  std::stringstream json; json << "\
  {"
    + cache_size +
    "\"tile_dir\": \"/data/valhalla\",\
    \"levels\": [\
      {\"name\": \"local\", \"level\": 2, \"size\": 0.25},\
      {\"name\": \"highway\", \"level\": 0, \"size\": 4},\
      {\"name\": \"arterial\", \"level\": 1, \"size\": 1, \"importance_cutoff\": \"Trunk\"}\
    ]\
  }";

  boost::property_tree::ptree pt;
  boost::property_tree::read_json(json, pt);

  return {pt};
}

void TestCacheLimits() {
  if(make_cache("\"max_cache_size\": 0,").OverCommitted())
    throw std::runtime_error("Cache should be over committed");

  if(make_cache("\"max_cache_size\": 1,").OverCommitted())
    throw std::runtime_error("Cache should be under committed");

  if(make_cache("").OverCommitted())
    throw std::runtime_error("Cache should be under committed");

  auto cache = make_cache("");
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

void touch_tile(const uint32_t tile_id, const TileHierarchy& tile_hierarchy) {
  auto suffix = GraphTile::FileSuffix({tile_id, 2, 0}, tile_hierarchy);
  auto fullpath = tile_hierarchy.tile_dir() + '/' + suffix;
  boost::filesystem::create_directories(boost::filesystem::path(fullpath).parent_path());
  int fd = open(fullpath.c_str(), O_CREAT | O_WRONLY, 0644);
  if(fd >= 0)
    close(fd);
}

void TestConnectivityMap() {
  //get the hierarchy to create some tiles
  std::stringstream json; json << "\
  {\
    \"tile_dir\": \"test/tiles\",\
    \"levels\": [\
      {\"name\": \"local\", \"level\": 2, \"size\": 0.25},\
      {\"name\": \"arterial\", \"level\": 1, \"size\": 1, \"importance_cutoff\": \"Trunk\"},\
      {\"name\": \"highway\", \"level\": 0, \"size\": 4}\
    ]\
  }";
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(json, pt);
  TileHierarchy th(pt);
  const auto& level = th.levels().find(2)->second;
  boost::filesystem::remove_all(th.tile_dir());

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
  touch_tile(a0, th);

  uint32_t a1 = level.tiles.RightNeighbor(a0);
  touch_tile(a1, th);

  uint32_t a2 = level.tiles.TopNeighbor(a0);
  touch_tile(a2, th);

  uint32_t b0 = level.tiles.RightNeighbor(level.tiles.RightNeighbor(a1));
  touch_tile(b0, th);

  uint32_t c0 =  level.tiles.TopNeighbor(level.tiles.RightNeighbor(a1));
  touch_tile(c0, th);

  uint32_t d0 = level.tiles.TopNeighbor(level.tiles.RightNeighbor(a2));
  touch_tile(d0, th);

  uint32_t d1 = level.tiles.TopNeighbor(d0);
  touch_tile(d1, th);

  //check that it looks right
  GraphReader reader(pt);
  if(!reader.AreConnected({a0, 2, 0}, {a1, 2, 0}))
    throw std::runtime_error("a's should be connected");
  if(!reader.AreConnected({a0, 2, 0}, {a2, 2, 0}))
    throw std::runtime_error("a's should be connected");
  if(!reader.AreConnected({a1, 2, 0}, {a2, 2, 0}))
    throw std::runtime_error("a's should be connected");
  if(!reader.AreConnected({d0, 2, 0}, {d1, 2, 0}))
    throw std::runtime_error("d's should be connected");
  if(reader.AreConnected({c0, 2, 0}, {a1, 2, 0}))
    throw std::runtime_error("c is disjoint");
  if(reader.AreConnected({b0, 2, 0}, {a0, 2, 0}))
    throw std::runtime_error("b is disjoint");
  if(reader.AreConnected({a2, 2, 0}, {d0, 2, 0}))
    throw std::runtime_error("a is disjoint from d");

  boost::filesystem::remove_all(th.tile_dir());
}

}

int main() {
  test::suite suite("graphtile");

  suite.test(TEST_CASE(TestCacheLimits));

  suite.test(TEST_CASE(TestConnectivityMap));

  return suite.tear_down();
}
