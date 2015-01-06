#include "valhalla/baldr/tilehierarchy.h"
#include "valhalla/baldr/graphid.h"
#include <valhalla/midgard/pointll.h>

#include "test.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {
  void test_parse() {
    std::stringstream json; json << "\
    {\
      \"output\": {\
        \"tile_dir\": \"/data/valhalla\",\
        \"levels\": [\
          {\"name\": \"local\", \"level\": 2, \"size\": 0.25},\
          {\"name\": \"highway\", \"level\": 0, \"size\": 4},\
          {\"name\": \"arterial\", \"level\": 1, \"size\": 1, \"importance_cutoff\": \"Trunk\"}\
        ]\
      }\
    }";

    boost::property_tree::ptree pt;
    boost::property_tree::read_json(json, pt);
    TileHierarchy h(pt);

    if(h.tile_dir() != "/data/valhalla")
      throw runtime_error("The tile directory was not correctly parsed");
    if(h.levels().size() != 3)
      throw runtime_error("Incorrect number of hierarchy levels");
    if((++h.levels().begin())->second.name != "arterial")
      throw runtime_error("Middle hierarchy should be named arterial");
    if(h.levels().begin()->second.level != 0)
      throw runtime_error("Top hierarchy should have level 0");
    if(h.levels().rbegin()->second.tiles.TileSize() != .25f)
      throw runtime_error("Bottom hierarchy should have tile size of .25f");
    if(h.HasLevel(5))
      throw runtime_error("There should only be levels 0, 1, 2");
    if(!h.HasLevel(2))
      throw runtime_error("There should be a level 2");
    GraphId id = h.GetGraphId(PointLL(0,0), 34);
    if(id.Is_Valid())
      throw runtime_error("GraphId should be invalid as the level doesn't exist");
    //there are 1440 cols and 720 rows, this spot lands on col 414 and row 522
    id = h.GetGraphId(PointLL(-76.5, 40.5), 2);
    if(id.level() != 2 || id.tileid() != (522 * 1440) + 414 || id.id() != 0)
      throw runtime_error("Expected different graph id for this location");
    if(h.levels().begin()->second.importance != RoadClass::kOther)
      throw runtime_error("Importance should be set to other");
    if((++h.levels().begin())->second.importance != RoadClass::kTrunk)
      throw runtime_error("Importance should be set to trunk");
  }
}

int main(void)
{
  test::suite suite("tilehierarchy");

  suite.test(TEST_CASE(test_parse));

  return suite.tear_down();
}
