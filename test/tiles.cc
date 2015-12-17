#include "test.h"
#include "valhalla/midgard/tiles.h"
#include "valhalla/midgard/aabb2.h"
#include "valhalla/midgard/pointll.h"


using namespace std;
using namespace valhalla::midgard;

namespace {

void TestMaxId() {
  if(Tiles<PointLL>::MaxTileId(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), .25) != 1036799)
    throw std::runtime_error("Unexpected maxid result");
  if(Tiles<PointLL>::MaxTileId(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), 1) != 64799)
    throw std::runtime_error("Unexpected maxid result");
  if(Tiles<PointLL>::MaxTileId(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), 4) != 4049)
    throw std::runtime_error("Unexpected maxid result");
  if(Tiles<PointLL>::MaxTileId(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), .33) != 595685)
    throw std::runtime_error("Unexpected maxid result");
}

void TestBase() {
  Tiles<PointLL> tiles(AABB2<PointLL>(Point2(-180, -90), PointLL(180, 90)), 1);
  PointLL ll;
  // left bottom
  ll = tiles.Base(0);
  if (!(ll.lng() == -180 && ll.lat() == -90)) {
    throw std::runtime_error("Unexpected base result");
  }
  ll = tiles.Base(1);
  if (!(ll.lng() == -179 && ll.lat() == -90)) {
    throw std::runtime_error("Unexpected base result");
  }
  // right bottm
  ll = tiles.Base(179);
  if (!(ll.lng() == 180 && ll.lat() == -90)) {
    throw std::runtime_error("Unexpected base result");
  }
  ll = tiles.Base(180);
  if (!(ll.lng() == -180 && ll.lat() == -89)) {
    throw std::runtime_error("Unexpected base result");
  }
  // right top
  ll = tiles.Base(180 * 180 - 1);
  if (!(ll.lng() == 180 && ll.lat() == 90)) {
    throw std::runtime_error("Unexpected base result");
  }
}

void TestRowCol() {
  Tiles<PointLL> tiles(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), 1);

  int32_t tileid1 = tiles.TileId(-76.5f, 40.5f);
  auto rc = tiles.GetRowColumn(tileid1);
  int32_t tileid2 = tiles.TileId(rc.second, rc.first);
  if (tileid1 != tileid2) {
    throw std::runtime_error("TileId does not match using row,col");
  }
}

void TestNeighbors() {
  Tiles<PointLL> tiles(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), 1);

  // Get a tile
  int32_t tileid1 = tiles.TileId(-76.5f, 40.5f);
  auto rc1 = tiles.GetRowColumn(tileid1);

  // Test left neighbor
  int32_t tileid2 = tiles.LeftNeighbor(tileid1);
  auto rc2 = tiles.GetRowColumn(tileid2);
  if (!tiles.AreNeighbors(tileid1, tileid2)) {
    throw std::runtime_error("Left neighbor not identified as a neighbor");
  }
  if (rc1.first != rc2.first || (rc1.second - 1) != rc2.second) {
    throw std::runtime_error("Left neighbor row,col not correct");
  }

  // Test right neighbor
  tileid2 = tiles.RightNeighbor(tileid1);
  rc2 = tiles.GetRowColumn(tileid2);
  if (!tiles.AreNeighbors(tileid1, tileid2)) {
    throw std::runtime_error("Right neighbor not identified as a neighbor");
  }
  if (rc1.first != rc2.first || (rc1.second + 1) != rc2.second) {
    throw std::runtime_error("Right neighbor row,col not correct");
  }

  // Top neighbor
  tileid2 = tiles.TopNeighbor(tileid1);
  rc2 = tiles.GetRowColumn(tileid2);
  if (!tiles.AreNeighbors(tileid1, tileid2)) {
    throw std::runtime_error("Top neighbor not identified as a neighbor");
  }
  if ((rc1.first + 1) != rc2.first || rc1.second != rc2.second) {
    throw std::runtime_error("Top neighbor row,col not correct");
  }

  // Bottom neighbor
  tileid2 = tiles.BottomNeighbor(tileid1);
  rc2 = tiles.GetRowColumn(tileid2);
  if (!tiles.AreNeighbors(tileid1, tileid2)) {
    throw std::runtime_error("Bottom neighbor not identified as a neighbor");
  }
  if ((rc1.first - 1) != rc2.first || rc1.second != rc2.second) {
    throw std::runtime_error("Bottom neighbor row,col not correct");
  }
}

void TileList() {
  Tiles<PointLL> tiles(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), 1);

  AABB2<PointLL> bbox(PointLL(-99.5f, 30.5f), PointLL(-90.5f, 39.5f));
  std::vector<int32_t> tilelist = tiles.TileList(bbox);
  if (tilelist.size() != 100) {
    throw std::runtime_error("Wrong number of tiles " +
                             std::to_string(tilelist.size()) +
                             " found in TileList");
  }
}

using intersect_t = std::unordered_map<int32_t, std::unordered_set<unsigned short> >;
void assert_answer(const Tiles<Point2>& g, const std::list<Point2>& l, const intersect_t& expected) {
  auto answer = g.Intersect(l);
  //wrong number of tiles
  if(answer.size() != expected.size())
    throw std::logic_error("Expected " + std::to_string(expected.size()) + " intersected tiles but got " + std::to_string(answer.size()));
  for(const auto& t : expected) {
    //missing tile
    auto i = answer.find(t.first);
    if(i == answer.cend())
      throw std::logic_error("Expected tile " + std::to_string(t.first) + " to be intersected");
    //wrong number of subdivisions
    if(t.second.size() != i->second.size())
      throw std::logic_error("In tile " + std::to_string(t.first) + " expected " + std::to_string(t.second.size()) + " intersected subdivisions but got " + std::to_string(i->second.size()));
    //missing subdivision
    for(const auto& s : t.second)
      if(i->second.find(s) == i->second.cend())
        throw std::logic_error("In tile " + std::to_string(t.first) + " expected subdivision " + std::to_string(s) + " to be intersected");
  }
}

void test_intersect_linestring() {
  Tiles<Point2> t(AABB2<Point2>{-1,-1,1,1}, .25, 5);
  assert_answer(t, {}, intersect_t{});
 /* assert_answer(t, { {-.9,0}, {.9,0} }, {10,11,12,13,14});
  assert_answer(t, { {-2,0}, {2,0} }, {10,11,12,13,14});
  assert_answer(t, { {-.9,0}, {-2,0} }, {10});
  assert_answer(t, { {-.9,.9} }, {20});
  assert_answer(t, { {.9,-.9} }, {4});
  assert_answer(t, { {.9,-1.1}, {.9, .9} }, {4, 9, 14, 19, 24});
  assert_answer(t, { {0.1, 2}, {0.1, 1.5}, {0.1, 1.0} }, {22});*/
}
/*
void test_intersect_circle() {
  grid<Point2> g(AABB2<Point2>{-1,-1,1,1}, 5);
  //TODO:
}

void test_random_linestring() {
  grid<Point2> g(AABB2<Point2>{-1,-1,1,1}, 5);
  std::default_random_engine generator;
  std::uniform_real_distribution<> distribution(-10, 10);
  for(int i = 0; i < 10000; ++i) {
    std::vector<Point2> linestring;
    for(int j = 0; j < 100; ++j)
      linestring.emplace_back(PointLL(distribution(generator), distribution(generator)));
    bool leaves;
    auto answer = g.intersect(linestring, leaves);
    for(auto a : answer)
      if(a > 24)
        throw std::runtime_error("Non-existant cell!");
  }
}

void test_random_circle() {
  grid<Point2> g(AABB2<Point2>{-1,-1,1,1}, 5);
  //TODO:
}
*/
}

int main() {
  test::suite suite("tiles");

  // Test tile id to row, col and vice-versa
  suite.test(TEST_CASE(TestRowCol));

  // Test neighbors
  suite.test(TEST_CASE(TestNeighbors));

  // Test max. tile Id
  suite.test(TEST_CASE(TestMaxId));

  // Test tile list
  suite.test(TEST_CASE(TileList));

  suite.test(TEST_CASE(test_intersect_linestring));
  /*suite.test(TEST_CASE(test_intersect_circle));
  suite.test(TEST_CASE(test_random_linestring));
  suite.test(TEST_CASE(test_random_circle));*/

  return suite.tear_down();
}
