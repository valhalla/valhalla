#include "test.h"
#include "valhalla/midgard/tiles.h"
#include "valhalla/midgard/aabb2.h"
#include "valhalla/midgard/pointll.h"


using namespace std;
using namespace valhalla::midgard;

namespace {

void TestMaxId() {
  if(Tiles::MaxTileId(AABB2(PointLL(-180, -90), PointLL(180, 90)), .25) != 1036799)
    throw std::runtime_error("Unexpected maxid result");
  if(Tiles::MaxTileId(AABB2(PointLL(-180, -90), PointLL(180, 90)), 1) != 64799)
    throw std::runtime_error("Unexpected maxid result");
  if(Tiles::MaxTileId(AABB2(PointLL(-180, -90), PointLL(180, 90)), 4) != 4049)
    throw std::runtime_error("Unexpected maxid result");
  if(Tiles::MaxTileId(AABB2(PointLL(-180, -90), PointLL(180, 90)), .33) != 595685)
    throw std::runtime_error("Unexpected maxid result");
}

void TileList() {
  Tiles tiles(AABB2(PointLL(-180, -90), PointLL(180, 90)), 1);

  AABB2 bbox(PointLL(-99.5f, 30.5f), PointLL(-90.5f, 39.5f));
  std::vector<int32_t> tilelist = tiles.TileList(bbox);
  if (tilelist.size() != 100) {
    throw std::runtime_error("Wrong number of tiles " +
                             std::to_string(tilelist.size()) +
                             " found in TileList");
  }
}

}

int main() {
  test::suite suite("tiles");

  // Test max. tile Id
  suite.test(TEST_CASE(TestMaxId));

  // Test tile list
  suite.test(TEST_CASE(TileList));

  return suite.tear_down();
}
