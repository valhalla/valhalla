#include "test.h"
#include "include/config.h"

#include "include/geo/tilehierarchy.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace std;
using namespace valhalla::geo;

namespace {
  void test_parse() {
    std::stringstream json; json << "\
    {\
      \"output\": {\
        \"tile_dir\": \"/data/valhalla\",\
        \"levels\": [\
          {\"name\": \"local\", \"level\": 2, \"size\": 0.25},\
          {\"name\": \"highway\", \"level\": 0, \"size\": 4},\
          {\"name\": \"arterial\", \"level\": 1, \"size\": 1}\
        ]\
      }\
    }";

    boost::property_tree::ptree pt;
    boost::property_tree::read_json(json, pt);
    TileHierarchy h(pt);

    if(h.tile_dir_ != "/data/valhalla")
      throw runtime_error("The tile directory was not correctly parsed");
    if(h.levels_.size() != 3)
      throw runtime_error("Incorrect number of hierarchy levels");
    if((++h.levels_.begin())->name_ != "arterial")
      throw runtime_error("Middle hierarchy should be named arterial");
    if(h.levels_.begin()->level_ != 0)
      throw runtime_error("Top hierarchy should have level 0");
    if(h.levels_.rbegin()->tiles_.TileSize() != .25f)
      throw runtime_error("Bottom hierarchy should have tile size of .25f");
  }
}

int main(void)
{
  test::suite suite("tilehierarchy");

  suite.test(TEST_CASE(test_parse));
  //suite.test(TEST_CASE(test_failure));

  return suite.tear_down();
}
