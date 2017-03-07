#include "test.h"

#include <stdexcept>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "baldr/graphreader.h"
#include "baldr/graphid.h"
#include "meili/traffic_segment_matcher.h"

using namespace valhalla;

namespace {

  void test_matcher() {
    baldr::TileHierarchy h("test/traffic_matcher_tiles");
    boost::property_tree::ptree conf;
    conf.put("tile_dir", h.tile_dir());
    baldr::GraphReader reader(conf);
    meili::TrafficSegmentMatcher matcher;

  }

}

int main() {
  test::suite suite("traffic matcher");

  suite.test(TEST_CASE(test_matcher));

  return suite.tear_down();
}
