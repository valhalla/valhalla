#include "test.h"
#include "loki/search.h"

#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/midgard/pointll.h>

namespace {

void node_search(valhalla::baldr::GraphReader& reader, const valhalla::baldr::Location& location, const valhalla::midgard::PointLL& expected){
  valhalla::baldr::PathLocation p = valhalla::loki::Search(location, reader, valhalla::loki::SearchStrategy::NODE);
  if(!p.IsNode())
    throw std::runtime_error("Node search should produce node results");
  if(!p.vertex().ApproximatelyEqual(expected))
    throw std::runtime_error("Found wrong node");
}

void TestNodeSearch() {
  std::stringstream json; json << "{ \
    \"output\": { \
      \"tile_dir\": \"test/tiles\", \
      \"levels\": [ \
        {\"name\": \"local\", \"level\": 2, \"size\": 0.25}, \
        {\"name\": \"arterial\", \"level\": 1, \"size\": 1, \"importance_cutoff\": \"TertiaryUnclassified\"}, \
        {\"name\": \"highway\", \"level\": 0, \"size\": 4, \"importance_cutoff\": \"Trunk\"} \
      ] \
    } \
  }";

  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(json, conf);

  valhalla::baldr::GraphReader reader(conf);
  node_search(reader, valhalla::baldr::Location({9.51585, 47.21232}), {9.516032, 47.212207});
}

}

int main() {
  test::suite suite("search");

  //suite.test(TEST_CASE(TestNodeSearch));

  return suite.tear_down();
}
