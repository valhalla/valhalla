#include "baldr/tilehierarchy.h"
#include "baldr/graphid.h"
#include <valhalla/midgard/pointll.h>

#include "test.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {
  void test_parse() {
    TileHierarchy h("/data/valhalla");

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
    if(h.levels().find(5) != h.levels().end())
      throw runtime_error("There should only be levels 0, 1, 2");
    if(h.levels().find(2) == h.levels().end())
      throw runtime_error("There should be a level 2");
    GraphId id = h.GetGraphId(PointLL(0,0), 34);
    if(id.Is_Valid())
      throw runtime_error("GraphId should be invalid as the level doesn't exist");
    //there are 1440 cols and 720 rows, this spot lands on col 414 and row 522
    id = h.GetGraphId(PointLL(-76.5, 40.5), 2);
    if(id.level() != 2 || id.tileid() != (522 * 1440) + 414 || id.id() != 0)
      throw runtime_error("Expected different graph id for this location");
    if(h.levels().begin()->second.importance != RoadClass::kPrimary)
      throw runtime_error("Importance should be set to primary");
    if((++h.levels().begin())->second.importance != RoadClass::kTertiary)
      throw runtime_error("Importance should be set to tertiary");
    if(h.levels().rbegin()->second.importance != RoadClass::kServiceOther)
      throw runtime_error("Importance should be set to service/other");
  }

  void test_tiles() {
    TileHierarchy h("/data/valhalla");

    //there are 1440 cols and 720 rows, this spot lands on col 414 and row 522
    AABB2<PointLL> bbox{{-76.49, 40.51}, {-76.48, 40.52}};
    auto ids = h.GetGraphIds(bbox, 2);
    if (ids.size() != 1) {
      throw runtime_error("Should have only found one result.");
    }
    auto id = ids[0];
    if (id.level() != 2 || id.tileid() != (522 * 1440) + 414 || id.id() != 0) {
      throw runtime_error("Didn't find correct tile ID.");
    }

    bbox = AABB2<PointLL>{{-76.51, 40.49}, {-76.49, 40.51}};
    ids = h.GetGraphIds(bbox, 2);
    if (ids.size() != 4) {
      throw runtime_error("Should have found 4 results.");
    }
  }
}

int main(void)
{
  test::suite suite("tilehierarchy");

  suite.test(TEST_CASE(test_parse));
  suite.test(TEST_CASE(test_tiles));

  return suite.tear_down();
}
