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

}

int main() {
  test::suite suite("tiles");

  // Subtraction of a point from another point yields a vector
  suite.test(TEST_CASE(TestMaxId));

  return suite.tear_down();
}
