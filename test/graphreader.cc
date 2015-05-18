#include "test.h"

#include "baldr/graphreader.h"

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

void TestConnectivityMap() {
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

}

}

int main() {
  test::suite suite("graphtile");

  suite.test(TEST_CASE(TestCacheLimits));

  suite.test(TEST_CASE(TestConnectivityMap));

  return suite.tear_down();
}
