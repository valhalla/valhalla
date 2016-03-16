#include "test.h"
#include "midgard/tiles.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"
#include "midgard/util.h"

#include <random>
#include <set>

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
  if(answer.size() > expected.size())
    throw std::logic_error("Expected no more than" + std::to_string(expected.size()) + " intersected tiles but got " + std::to_string(answer.size()));
  for(const auto& t : answer) {
    //missing tile
    auto i = expected.find(t.first);
    if(i == expected.cend())
      throw std::logic_error("Unexpected intersected tile " + std::to_string(t.first));
    //wrong number of subdivisions
    if(t.second.size() > i->second.size())
      throw std::logic_error("in tile " + std::to_string(t.first) + " expected no more than " + std::to_string(i->second.size()) + " intersected subdivisions but got " + std::to_string(t.second.size()));
    //missing subdivision
    for(const auto& s : t.second)
      if(i->second.find(s) == i->second.cend())
        throw std::logic_error("In tile " + std::to_string(t.first) + " unexpected intersected subdivision " + std::to_string(s));
  }
}

void test_intersect_linestring() {
  Tiles<Point2> t(AABB2<Point2>{-5,-5,5,5}, 2.5, 5);

  //nothing
  assert_answer(t, {}, intersect_t{});
  assert_answer(t, { {-10,-10} }, intersect_t{});
  assert_answer(t, { {-10,-10}, {-10,-10} }, intersect_t{});

  //single
  assert_answer(t, { {-1,-1} }, intersect_t{{5,{18}}});
  assert_answer(t, { {-1,-1}, {-1,-1} }, intersect_t{{5,{18}}});

  //horizontal
  assert_answer(t, { {-4.9,-4.9}, {4.9,-4.9} }, intersect_t{{0,{0,1,2,3,4}},{1,{0,1,2,3,4}},{2,{0,1,2,3,4}},{3,{0,1,2,3,4}}});
  assert_answer(t, { {-5.9,-4.9}, {5.9,-4.9} }, intersect_t{{0,{0,1,2,3,4}},{1,{0,1,2,3,4}},{2,{0,1,2,3,4}},{3,{0,1,2,3,4}}});
  assert_answer(t, { {-4.9,4.9}, {4.9,4.9} }, intersect_t{{12,{20,21,22,23,24}},{13,{20,21,22,23,24}},{14,{20,21,22,23,24}},{15,{20,21,22,23,24}}});
  assert_answer(t, { {-5.9,4.9}, {5.9,4.9} }, intersect_t{{12,{20,21,22,23,24}},{13,{20,21,22,23,24}},{14,{20,21,22,23,24}},{15,{20,21,22,23,24}}});

  //vertical
  assert_answer(t, { {-4.9,4.9}, {-4.9,-4.9} }, intersect_t{{0,{0,5,10,15,20}},{4,{0,5,10,15,20}},{8,{0,5,10,15,20}},{12,{0,5,10,15,20}}});
  assert_answer(t, { {-4.9,5.9}, {-4.9,-5.9} }, intersect_t{{0,{0,5,10,15,20}},{4,{0,5,10,15,20}},{8,{0,5,10,15,20}},{12,{0,5,10,15,20}}});
  assert_answer(t, { {4.9,4.9}, {4.9,-4.9} }, intersect_t{{3,{4,9,14,19,24}},{7,{4,9,14,19,24}},{11,{4,9,14,19,24}},{15,{4,9,14,19,24}}});
  assert_answer(t, { {4.9,5.9}, {4.9,-5.9} }, intersect_t{{3,{4,9,14,19,24}},{7,{4,9,14,19,24}},{11,{4,9,14,19,24}},{15,{4,9,14,19,24}}});

  //diagonal
  assert_answer(t, { {-4.9,-4.9}, {4.9,4.9} }, intersect_t{ {0,{0,1,5,6,7,11,12,13,17,18,19,23,24}},{1,{20}},{4,{4}},
                                                            {5,{0,1,5,6,7,11,12,13,17,18,19,23,24}},{6,{20}},{9,{4}},
                                                            {10,{0,1,5,6,7,11,12,13,17,18,19,23,24}},{11,{20}},{14,{4}},
                                                            {15,{0,1,5,6,7,11,12,13,17,18,19,23,24}} });
  assert_answer(t, { {-5.9,-5.9}, {5.9,5.9} }, intersect_t{ {0,{0,1,5,6,7,11,12,13,17,18,19,23,24}},{1,{20}},{4,{4}},
                                                            {5,{0,1,5,6,7,11,12,13,17,18,19,23,24}},{6,{20}},{9,{4}},
                                                            {10,{0,1,5,6,7,11,12,13,17,18,19,23,24}},{11,{20}},{14,{4}},
                                                            {15,{0,1,5,6,7,11,12,13,17,18,19,23,24}} });
  assert_answer(t, { {-4.9,4.9}, {4.9,-4.9} }, intersect_t{ {2,{24}},{3,{3,4,9,7,8,13,11,12,17,15,16,21,20}},{7,{0}},
                                                            {5,{24}},{6,{3,4,9,7,8,13,11,12,17,15,16,21,20}},{10,{0}},
                                                            {8,{24}},{9,{3,4,9,7,8,13,11,12,17,15,16,21,20}},{15,{0}},
                                                            {12,{3,4,9,7,8,13,11,12,17,15,16,21,20}} });
  assert_answer(t, { {-5.9,5.9}, {5.9,-5.9} }, intersect_t{ {2,{24}},{3,{3,4,9,7,8,13,11,12,17,15,16,21,20}},{7,{0}},
                                                            {5,{24}},{6,{3,4,9,7,8,13,11,12,17,15,16,21,20}},{10,{0}},
                                                            {8,{24}},{9,{3,4,9,7,8,13,11,12,17,15,16,21,20}},{15,{0}},
                                                            {12,{3,4,9,7,8,13,11,12,17,15,16,21,20}} });

  //random slopes
  t = Tiles<Point2>(AABB2<Point2>{0,0,6,6}, 6, 6);
  assert_answer(t, { {0.5,0.5}, {5.5,4.5} }, intersect_t{{0,{0,1,7,8,14,15,21,22,28,29}}});
  assert_answer(t, { {5.5,4.5}, {0.5,0.5} }, intersect_t{{0,{0,1,7,8,14,15,21,22,28,29}}});
  assert_answer(t, { {5.5,0.5}, {0.5,2.5} }, intersect_t{{0,{4,5,7,8,9,10,12,13}}});
  assert_answer(t, { {0.5,2.5}, {5.5,0.5} }, intersect_t{{0,{4,5,7,8,9,10,12,13}}});
  assert_answer(t, { {-1,-2}, {4,8} }, intersect_t{{0,{0,6,7,12,13,19,20,25,26,32,33}}});
  assert_answer(t, { {4,8}, {-1,-2} }, intersect_t{{0,{0,6,7,12,13,19,20,25,26,32,33}}});
  assert_answer(t, { {1,2}, {2,4} }, intersect_t{{0,{6,7,12,13,19,20,25,26}}});
  assert_answer(t, { {2,4}, {1,2} }, intersect_t{{0,{6,7,12,13,19,20,25,26}}});

  //some real locations on earth (without polar coordinates accounted for)
  Tiles<PointLL> ll(AABB2<PointLL>{-180,-90,180,90}, .25, 5);
  std::vector<PointLL> shape{{9.5499754, 47.250248},{9.55031681, 47.2501144}};
  auto intersection = ll.Intersect(shape);
  for(const auto& i : intersection)
    if(i.first != 791318)
      throw std::logic_error("This tile shouldn't be intersected: " + std::to_string(i.first));
}

void test_random_linestring() {
  Tiles<Point2> t(AABB2<Point2>{-10,-10,10,10}, 1, 5);
  std::default_random_engine generator;
  std::uniform_real_distribution<> distribution(-10, 10);
  for(int i = 0; i < 500; ++i) {
    std::vector<Point2> linestring;
    for(int j = 0; j < 100; ++j)
      linestring.emplace_back(PointLL(distribution(generator), distribution(generator)));
    auto answer = t.Intersect(linestring);
    for(auto tile : answer)
      for(auto sub : tile.second)
        if(sub > 24)
          throw std::runtime_error("Non-existant bin!");
  }
}

template <class coord_t>
std::pair<int32_t, int32_t> to_xy(std::tuple<int32_t, unsigned short, float> tile, const Tiles<coord_t>& t) {
  auto ax = (std::get<0>(tile) % t.ncolumns()) * t.nsubdivisions() + (std::get<1>(tile) % t.nsubdivisions());
  auto ay = (std::get<0>(tile) / t.ncolumns()) * t.nsubdivisions() + (std::get<1>(tile) / t.nsubdivisions());
  return std::make_pair(ax, ay);
}

template <class coord_t>
int32_t to_global_sub(std::tuple<int32_t, unsigned short, float> tile, const Tiles<coord_t>& t) {
  auto xy = to_xy(tile,t);
  return xy.second * (t.ncolumns() * t.nsubdivisions()) + xy.first;
}

//brute force entire set of subdivisions at once
using sub_t = std::tuple<int32_t, unsigned short, float>;
template <class coord_t>
std::set<sub_t, std::function<bool (const sub_t&, const sub_t&)> > closest_first_answer(const Tiles<coord_t>& t, const coord_t& p){
  //place to keep the subdivisions
  auto sort = [&t](const sub_t& a, const sub_t& b) {
    //turn tile local subdivision into global subdivision for comparison
    auto as = to_global_sub(a, t);
    auto bs = to_global_sub(b, t);
    return std::get<2>(a) == std::get<2>(b) ? as < bs : std::get<2>(a) < std::get<2>(b);
  };
  std::set<sub_t, std::function<bool (const sub_t&, const sub_t&)> > answer(sort);
  //run over all tiles
  for(int32_t i = 0; i < t.nrows(); ++i) {
    for(int32_t j = 0; j < t.ncolumns(); ++j) {
      //run over all subdivisions
      for(unsigned short k = 0; k < t.nsubdivisions(); ++k) {
        for(unsigned short l = 0; l < t.nsubdivisions(); ++l) {
          auto x0 = t.TileBounds().minx() + (l + j * t.nsubdivisions()) * t.SubdivisionSize();
          auto x1 = t.TileBounds().minx() + (l + j * t.nsubdivisions() + 1) * t.SubdivisionSize();
          auto y0 = t.TileBounds().miny() + (k + i * t.nsubdivisions()) * t.SubdivisionSize();
          auto y1 = t.TileBounds().miny() + (k + i * t.nsubdivisions() + 1) * t.SubdivisionSize();
          int both = 0;
          auto distance = std::numeric_limits<float>::max();
          std::list<coord_t> corners{ {x0, y0}, {x1, y0}, {x0, y1}, {x1, y1} };
          if(x0 < p.first && x1 > p.first) { corners.emplace_back(p.first, y0); corners.emplace_back(p.first, y1); ++both; }
          if(y0 < p.second && y1 > p.second) { corners.emplace_back(x0, p.second); corners.emplace_back(x1, p.second); ++both; }
          if(both == 2) corners = { p };
          for(const auto& c : corners) {
            auto d = p.Distance(c);
            if(d < distance)
              distance = d;
          }

          auto tile = t.TileId(j, i);
          auto subdivision = k * t.nsubdivisions() + l;
          answer.emplace(std::make_tuple(tile, subdivision, distance));
        }
      }
    }
  }
  return answer;
}

void test_closest_first() {
  //test a simple 8x4 grid wrapping
  Tiles<PointLL> t(AABB2<PointLL>{-180,-90,180,90}, 90, 2);
  auto y = t.ClosestFirst({179.99, -16.825});
  if(to_global_sub(y(), t) != 15)
    throw std::logic_error("Should have been 15");
  if(to_global_sub(y(), t) != 8)
    throw std::logic_error("Should have been wrapped to 8");
  if(to_global_sub(y(), t) != 23)
    throw std::logic_error("Should have been above to 23");
  y = t.ClosestFirst({-179.99, -16.825});
  if(to_global_sub(y(), t) != 8)
    throw std::logic_error("Should have been 8");
  if(to_global_sub(y(), t) != 15)
    throw std::logic_error("Should have been wrapped to 15");
  if(to_global_sub(y(), t) != 16)
    throw std::logic_error("Should have been above to 16");

  //check antimeridian wrapping
  t = Tiles<PointLL>(AABB2<PointLL>{-180,-90,180,90}, .25, 5);
  PointLL p{179.99, -16.825};
  int px = (p.first - t.TileBounds().minx()) / t.TileBounds().Width() * t.ncolumns() * t.nsubdivisions();
  int py = (p.second - t.TileBounds().miny()) / t.TileBounds().Height() * t.nrows() * t.nsubdivisions();
  auto c = t.ClosestFirst(p);
  auto first = c();
  if(to_global_sub(first, t) != py * t.ncolumns() * t.nsubdivisions() + px)
    throw std::logic_error("Unexpected global subdivision");
  if(std::get<2>(first) != 0)
    throw std::logic_error("Unexpected distance");
  auto second  = c();
  if(to_global_sub(second, t) != py * t.ncolumns() * t.nsubdivisions())
    throw std::logic_error("Unexpected global subdivision");
  if(std::get<2>(second) != p.Distance({-180, -16.825}))
    throw std::logic_error("Unexpected distance");

  //try planar coordinate system
  Tiles<Point2> tp(AABB2<Point2>{-10,-10,10,10}, 1, 5);
  auto foo = tp.ClosestFirst({0,0});
  auto bar = closest_first_answer<Point2>(tp, {0,0});
  for(const auto& p : std::list<Point2>{ {0,0}, {-1.99,-1.99}, {-.03,1.21}, {7.23,-3.332}, {.04,8.76} }) {
    auto c = tp.ClosestFirst(p);
    auto a = closest_first_answer<Point2>(tp, p);
    size_t zeros = 0;
    for(const auto& s : a)
      if(std::get<2>(s) == 0)
        ++zeros;
    if(zeros != 1 && zeros != 2 && zeros != 4)
      throw std::logic_error("Only 1, 2 and 4 subdivisions can be 0 distance from the input point");
    zeros = 0;
    for(const auto& s : a) {
      auto r = c();
      if(s != r)
        throw std::logic_error("Unexpected tile, subdivision or distance");
      if(std::get<2>(r) == 0)
        ++zeros;
    }
    if(zeros != 1 && zeros != 2 && zeros != 4)
      throw std::logic_error("Only 1, 2 and 4 subdivisions can be 0 distance from the input point");
    try { c(); } catch (const std::runtime_error&) { continue; }
    throw std::logic_error("Closest first functor should have thrown");
  }

  //try spherical coordinate system
  t = Tiles<PointLL>(AABB2<PointLL>{-180,-90,180,90}, 4, 5);
  for(const auto& p : std::list<PointLL>{ {0,0}, {-76.5,40.5}, {47.31707,9.2827}, {11.92515,78.92409}, {-67.61196,-54.93575}, {179.99155,-16.80257} }) {
    auto c = t.ClosestFirst(p);
    auto a = closest_first_answer<PointLL>(t, p);
    size_t zeros = 0;
    for(const auto& s : a)
      if(std::get<2>(s) == 0)
        ++zeros;
    if(zeros != 1 && zeros != 2 && zeros != 4)
      throw std::logic_error("Only 1, 2 and 4 subdivisions can be 0 distance from the input point");
    zeros = 0;
    for(const auto& s : a) {
      auto r = c();
      if(s != r)
        throw std::logic_error("Unexpected tile, subdivision or distance");
      if(std::get<2>(r) == 0)
        ++zeros;
    }
    if(zeros != 1 && zeros != 2 && zeros != 4)
      throw std::logic_error("Only 1, 2 and 4 subdivisions can be 0 distance from the input point");
    try { c(); } catch (const std::runtime_error&) { continue; }
    throw std::logic_error("Closest first functor should have thrown");
  }

}

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

  suite.test(TEST_CASE(test_closest_first));

  suite.test(TEST_CASE(test_random_linestring));

  return suite.tear_down();
}
