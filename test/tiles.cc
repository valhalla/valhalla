#include "midgard/tiles.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"
#include "midgard/util.h"

#include <random>

#include "test.h"

using namespace valhalla::midgard;

namespace {

TEST(Tiles, TestMaxId) {
  EXPECT_EQ(Tiles<PointLL>::MaxTileId(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), .25),
            1036799);
  EXPECT_EQ(Tiles<PointLL>::MaxTileId(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), 1),
            64799);
  EXPECT_EQ(Tiles<PointLL>::MaxTileId(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), 4), 4049);
  EXPECT_EQ(Tiles<PointLL>::MaxTileId(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), .33),
            595685);
}

TEST(Tiles, TestBase) {
  Tiles<PointLL> tiles(AABB2<PointLL>(Point2(-180, -90), PointLL(180, 90)), 1);
  PointLL ll;
  // left bottom
  ll = tiles.Base(0);
  EXPECT_EQ(ll.lng(), -180);
  EXPECT_EQ(ll.lat(), -90);

  ll = tiles.Base(1);
  EXPECT_EQ(ll.lng(), -179);
  EXPECT_EQ(ll.lat(), -90);

  // right bottm
  ll = tiles.Base(359);
  EXPECT_EQ(ll.lng(), 179);
  EXPECT_EQ(ll.lat(), -90);

  ll = tiles.Base(360);
  EXPECT_EQ(ll.lng(), -180);
  EXPECT_EQ(ll.lat(), -89);

  // right top
  ll = tiles.Base(360 * 180 - 1);
  EXPECT_EQ(ll.lng(), 179);
  EXPECT_EQ(ll.lat(), 89);
}

TEST(Tiles, TestRowCol) {
  Tiles<PointLL> tiles(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), 1);

  int32_t tileid1 = tiles.TileId(-76.5f, 40.5f);
  auto rc = tiles.GetRowColumn(tileid1);
  int32_t tileid2 = tiles.TileId(rc.second, rc.first);
  EXPECT_EQ(tileid1, tileid2) << "TileId does not match using row,col";
}

TEST(Tiles, TestNeighbors) {
  Tiles<PointLL> tiles(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), 1);

  // Get a tile
  int32_t tileid1 = tiles.TileId(-76.5f, 40.5f);
  auto rc1 = tiles.GetRowColumn(tileid1);

  // Test left neighbor
  int32_t tileid2 = tiles.LeftNeighbor(tileid1);
  auto rc2 = tiles.GetRowColumn(tileid2);

  EXPECT_TRUE(tiles.AreNeighbors(tileid1, tileid2))
      << "Left neighbor not identified as a neighbor " << tileid1 << " - " << tileid2;

  EXPECT_EQ(rc1.first, rc2.first) << "Left neighbor row,col not correct";
  EXPECT_EQ((rc1.second - 1), rc2.second) << "Left neighbor row,col not correct";

  // Test right neighbor
  tileid2 = tiles.RightNeighbor(tileid1);
  rc2 = tiles.GetRowColumn(tileid2);
  EXPECT_TRUE(tiles.AreNeighbors(tileid1, tileid2))
      << "Right neighbor not identified as a neighbor " << tileid1 << " - " << tileid2;

  EXPECT_EQ(rc1.first, rc2.first) << "Right neighbor row,col not correct";
  EXPECT_EQ((rc1.second + 1), rc2.second) << "Right neighbor row,col not correct";

  // Top neighbor
  tileid2 = tiles.TopNeighbor(tileid1);
  rc2 = tiles.GetRowColumn(tileid2);
  EXPECT_TRUE(tiles.AreNeighbors(tileid1, tileid2))
      << "Top neighbor not identified as a neighbor " << tileid1 << " - " << tileid2;

  EXPECT_EQ((rc1.first + 1), rc2.first) << "Top neighbor row,col not correct";
  EXPECT_EQ(rc1.second, rc2.second) << "Top neighbor row,col not correct";

  // Bottom neighbor
  tileid2 = tiles.BottomNeighbor(tileid1);
  rc2 = tiles.GetRowColumn(tileid2);
  EXPECT_TRUE(tiles.AreNeighbors(tileid1, tileid2))
      << "Bottom neighbor not identified as a neighbor " << tileid1 << " - " << tileid2;

  EXPECT_EQ((rc1.first - 1), rc2.first) << "Bottom neighbor row,col not correct";
  EXPECT_EQ(rc1.second, rc2.second) << "Bottom neighbor row,col not correct";
}

TEST(Tiles, TileList) {
  Tiles<PointLL> tiles(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), 1);

  AABB2<PointLL> bbox(PointLL(-99.5f, 30.5f), PointLL(-90.5f, 39.5f));
  std::vector<int32_t> tilelist = tiles.TileList(bbox);
  EXPECT_EQ(tilelist.size(), 100) << "Wrong number of tiles in TileList";

  // Test crossing -180
  AABB2<PointLL> bbox2(PointLL(-183.5f, 30.5f), PointLL(-176.5f, 34.5f));
  tilelist = tiles.TileList(bbox2);
  EXPECT_EQ(tilelist.size(), 40) << "Wrong number of tiles in TileList crossing -180";

  // Test crossing 180
  AABB2<PointLL> bbox3(PointLL(176.5f, 30.5f), PointLL(183.5f, 34.5f));
  tilelist = tiles.TileList(bbox3);
  EXPECT_EQ(tilelist.size(), 40) << "Wrong number of tiles in TileList crossing 180";

  Tiles<PointLL> tiles2(AABB2<PointLL>(PointLL(-180, -90), PointLL(180, 90)), 0.25f);
  AABB2<PointLL> bbox4(PointLL(-76.489998f, 40.509998f), PointLL(-76.480003f, 40.520000f));
  tilelist = tiles.TileList(bbox4);
  EXPECT_EQ(tilelist.size(), 1) << "Wrong number of tiles found in TileList";
}

using intersect_t = std::unordered_map<int32_t, std::unordered_set<unsigned short>>;
void assert_answer(const Tiles<Point2>& g, const std::list<Point2>& l, const intersect_t& expected) {
  auto answer = g.Intersect(l);
  // wrong number of tiles
  EXPECT_LE(answer.size(), expected.size()) << "Incorrect number of inserted tiles";

  for (const auto& t : answer) {
    // missing tile
    auto i = expected.find(t.first);
    ASSERT_NE(i, expected.cend()) << "Unexpected intersected tile " + std::to_string(t.first);
    // wrong number of subdivisions
    ASSERT_LE(t.second.size(), i->second.size())
        << "in tile " + std::to_string(t.first) + " expected no more than " +
               std::to_string(i->second.size()) + " intersected subdivisions but got " +
               std::to_string(t.second.size());

    // missing subdivision
    for (const auto& s : t.second)
      ASSERT_NE(i->second.find(s), i->second.cend()) << "In tile " + std::to_string(t.first) +
                                                            " unexpected intersected subdivision " +
                                                            std::to_string(s);
  }
}

TEST(Tiles, test_intersect_linestring) {
  Tiles<Point2> t(AABB2<Point2>{-5, -5, 5, 5}, 2.5, 5);

  // nothing
  assert_answer(t, {}, intersect_t{});
  assert_answer(t, {{-10, -10}}, intersect_t{});
  assert_answer(t, {{-10, -10}, {-10, -10}}, intersect_t{});

  // single
  assert_answer(t, {{-1, -1}}, intersect_t{{5, {18}}});
  assert_answer(t, {{-1, -1}, {-1, -1}}, intersect_t{{5, {18}}});

  // horizontal
  assert_answer(t, {{-4.9, -4.9}, {4.9, -4.9}},
                intersect_t{{0, {0, 1, 2, 3, 4}},
                            {1, {0, 1, 2, 3, 4}},
                            {2, {0, 1, 2, 3, 4}},
                            {3, {0, 1, 2, 3, 4}}});
  assert_answer(t, {{-5.9, -4.9}, {5.9, -4.9}},
                intersect_t{{0, {0, 1, 2, 3, 4}},
                            {1, {0, 1, 2, 3, 4}},
                            {2, {0, 1, 2, 3, 4}},
                            {3, {0, 1, 2, 3, 4}}});
  assert_answer(t, {{-4.9, 4.9}, {4.9, 4.9}},
                intersect_t{{12, {20, 21, 22, 23, 24}},
                            {13, {20, 21, 22, 23, 24}},
                            {14, {20, 21, 22, 23, 24}},
                            {15, {20, 21, 22, 23, 24}}});
  assert_answer(t, {{-5.9, 4.9}, {5.9, 4.9}},
                intersect_t{{12, {20, 21, 22, 23, 24}},
                            {13, {20, 21, 22, 23, 24}},
                            {14, {20, 21, 22, 23, 24}},
                            {15, {20, 21, 22, 23, 24}}});

  // vertical
  assert_answer(t, {{-4.9, 4.9}, {-4.9, -4.9}},
                intersect_t{{0, {0, 5, 10, 15, 20}},
                            {4, {0, 5, 10, 15, 20}},
                            {8, {0, 5, 10, 15, 20}},
                            {12, {0, 5, 10, 15, 20}}});
  assert_answer(t, {{-4.9, 5.9}, {-4.9, -5.9}},
                intersect_t{{0, {0, 5, 10, 15, 20}},
                            {4, {0, 5, 10, 15, 20}},
                            {8, {0, 5, 10, 15, 20}},
                            {12, {0, 5, 10, 15, 20}}});
  assert_answer(t, {{4.9, 4.9}, {4.9, -4.9}},
                intersect_t{{3, {4, 9, 14, 19, 24}},
                            {7, {4, 9, 14, 19, 24}},
                            {11, {4, 9, 14, 19, 24}},
                            {15, {4, 9, 14, 19, 24}}});
  assert_answer(t, {{4.9, 5.9}, {4.9, -5.9}},
                intersect_t{{3, {4, 9, 14, 19, 24}},
                            {7, {4, 9, 14, 19, 24}},
                            {11, {4, 9, 14, 19, 24}},
                            {15, {4, 9, 14, 19, 24}}});

  // diagonal
  assert_answer(t, {{-4.9, -4.9}, {4.9, 4.9}},
                intersect_t{{0, {0, 1, 5, 6, 7, 11, 12, 13, 17, 18, 19, 23, 24}},
                            {1, {20}},
                            {4, {4}},
                            {5, {0, 1, 5, 6, 7, 11, 12, 13, 17, 18, 19, 23, 24}},
                            {6, {20}},
                            {9, {4}},
                            {10, {0, 1, 5, 6, 7, 11, 12, 13, 17, 18, 19, 23, 24}},
                            {11, {20}},
                            {14, {4}},
                            {15, {0, 1, 5, 6, 7, 11, 12, 13, 17, 18, 19, 23, 24}}});
  assert_answer(t, {{-5.9, -5.9}, {5.9, 5.9}},
                intersect_t{{0, {0, 1, 5, 6, 7, 11, 12, 13, 17, 18, 19, 23, 24}},
                            {1, {20}},
                            {4, {4}},
                            {5, {0, 1, 5, 6, 7, 11, 12, 13, 17, 18, 19, 23, 24}},
                            {6, {20}},
                            {9, {4}},
                            {10, {0, 1, 5, 6, 7, 11, 12, 13, 17, 18, 19, 23, 24}},
                            {11, {20}},
                            {14, {4}},
                            {15, {0, 1, 5, 6, 7, 11, 12, 13, 17, 18, 19, 23, 24}}});
  assert_answer(t, {{-4.9, 4.9}, {4.9, -4.9}},
                intersect_t{{2, {24}},
                            {3, {3, 4, 9, 7, 8, 13, 11, 12, 17, 15, 16, 21, 20}},
                            {7, {0}},
                            {5, {24}},
                            {6, {3, 4, 9, 7, 8, 13, 11, 12, 17, 15, 16, 21, 20}},
                            {10, {0}},
                            {8, {24}},
                            {9, {3, 4, 9, 7, 8, 13, 11, 12, 17, 15, 16, 21, 20}},
                            {15, {0}},
                            {12, {3, 4, 9, 7, 8, 13, 11, 12, 17, 15, 16, 21, 20}}});
  assert_answer(t, {{-5.9, 5.9}, {5.9, -5.9}},
                intersect_t{{2, {24}},
                            {3, {3, 4, 9, 7, 8, 13, 11, 12, 17, 15, 16, 21, 20}},
                            {7, {0}},
                            {5, {24}},
                            {6, {3, 4, 9, 7, 8, 13, 11, 12, 17, 15, 16, 21, 20}},
                            {10, {0}},
                            {8, {24}},
                            {9, {3, 4, 9, 7, 8, 13, 11, 12, 17, 15, 16, 21, 20}},
                            {15, {0}},
                            {12, {3, 4, 9, 7, 8, 13, 11, 12, 17, 15, 16, 21, 20}}});

  // random slopes
  t = Tiles<Point2>(AABB2<Point2>{0, 0, 6, 6}, 6, 6);
  assert_answer(t, {{0.5, 0.5}, {5.5, 4.5}}, intersect_t{{0, {0, 1, 7, 8, 14, 15, 21, 22, 28, 29}}});
  assert_answer(t, {{5.5, 4.5}, {0.5, 0.5}}, intersect_t{{0, {0, 1, 7, 8, 14, 15, 21, 22, 28, 29}}});
  assert_answer(t, {{5.5, 0.5}, {0.5, 2.5}}, intersect_t{{0, {4, 5, 7, 8, 9, 10, 12, 13}}});
  assert_answer(t, {{0.5, 2.5}, {5.5, 0.5}}, intersect_t{{0, {4, 5, 7, 8, 9, 10, 12, 13}}});
  assert_answer(t, {{-1, -2}, {4, 8}}, intersect_t{{0, {0, 6, 7, 12, 13, 19, 20, 25, 26, 32, 33}}});
  assert_answer(t, {{4, 8}, {-1, -2}}, intersect_t{{0, {0, 6, 7, 12, 13, 19, 20, 25, 26, 32, 33}}});
  assert_answer(t, {{1, 2}, {2, 4}}, intersect_t{{0, {6, 7, 12, 13, 19, 20, 25, 26}}});
  assert_answer(t, {{2, 4}, {1, 2}}, intersect_t{{0, {6, 7, 12, 13, 19, 20, 25, 26}}});

  // some real locations on earth (without polar coordinates accounted for)
  Tiles<PointLL> ll(AABB2<PointLL>{-180, -90, 180, 90}, .25, 5);
  std::vector<PointLL> shape{{9.5499754, 47.250248}, {9.55031681, 47.2501144}};
  auto intersection = ll.Intersect(shape);
  for (const auto& i : intersection)
    EXPECT_EQ(i.first, 791318) << "This tile shouldn't be intersected";

  shape = {{130.399643, 33.6005592}, {130.399994, 33.5999985}};
  intersection = ll.Intersect(shape);
  size_t count = 0;
  for (const auto& i : intersection)
    count += i.second.size();
  EXPECT_LE(count, 2) << "Unexpected number of intersections for this shape";
}

TEST(Tiles, test_random_linestring) {
  Tiles<Point2> t(AABB2<Point2>{-10, -10, 10, 10}, 1, 5);
  std::mt19937 generator;
  std::uniform_real_distribution<> distribution(-10, 10);
  for (int i = 0; i < 500; ++i) {
    std::vector<Point2> linestring;
    linestring.reserve(100);
    for (int j = 0; j < 100; ++j)
      linestring.emplace_back(PointLL(distribution(generator), distribution(generator)));
    auto answer = t.Intersect(linestring);
    for (const auto& tile : answer)
      for (auto sub : tile.second)
        ASSERT_LE(sub, 24) << "Non-existant bin!";
  }
}

template <class coord_t>
std::pair<int32_t, int32_t> to_xy(std::tuple<int32_t, unsigned short, float> tile,
                                  const Tiles<coord_t>& t) {
  auto ax = (std::get<0>(tile) % t.ncolumns()) * t.nsubdivisions() +
            (std::get<1>(tile) % t.nsubdivisions());
  auto ay = (std::get<0>(tile) / t.ncolumns()) * t.nsubdivisions() +
            (std::get<1>(tile) / t.nsubdivisions());
  return std::make_pair(ax, ay);
}

template <class coord_t>
int32_t to_global_sub(std::tuple<int32_t, unsigned short, float> tile, const Tiles<coord_t>& t) {
  auto xy = to_xy(tile, t);
  return xy.second * (t.ncolumns() * t.nsubdivisions()) + xy.first;
}

template <class coord_t> coord_t dist(int32_t sub, const Tiles<coord_t>& tiles, const coord_t& seed) {
  auto subcols = tiles.ncolumns() * tiles.nsubdivisions();
  auto x = sub % subcols;
  auto x0 = tiles.TileBounds().minx() + x * tiles.SubdivisionSize();
  auto x1 = tiles.TileBounds().minx() + (x + 1) * tiles.SubdivisionSize();
  auto y = sub / subcols;
  auto y0 = tiles.TileBounds().miny() + y * tiles.SubdivisionSize();
  auto y1 = tiles.TileBounds().miny() + (y + 1) * tiles.SubdivisionSize();
  auto distance = std::numeric_limits<float>::max();
  std::list<coord_t> corners{{x0, y0}, {x1, y0}, {x0, y1}, {x1, y1}};
  if (x0 < seed.first && x1 > seed.first) {
    corners.emplace_back(seed.first, y0);
    corners.emplace_back(seed.first, y1);
  }
  if (y0 < seed.second && y1 > seed.second) {
    corners.emplace_back(x0, seed.second);
    corners.emplace_back(x1, seed.second);
  }
  coord_t used;
  for (const auto& c : corners) {
    auto d = seed.Distance(c);
    if (d < distance) {
      distance = d;
      used = c;
    }
  }
  return used;
}

template <class coord_t> void test_point(const Tiles<coord_t>& t, const coord_t& p) {
  auto a = t.ClosestFirst(p);
  size_t size = 0;
  size_t zeros = 0;
  std::tuple<int32_t, unsigned short, float> last{-1, -1, 0};
  while (true) {
    try {
      // keep track of zero distsance subdivisions
      auto r = a();
      auto d = std::get<2>(r);
      if (d == 0)
        ++zeros;
      // if its out of order you fail
      if (d < std::get<2>(last)) {
        // auto l = dist(to_global_sub(last, t), t, p);
        // auto c = dist(to_global_sub(r, t), t, p);
        FAIL() << "Distances should be smallest first " << d << " " << std::get<2>(last);
      }
      // remember the last distance and how many we've seen
      last = r;
      ++size;
    } catch (const std::runtime_error& e) {
      EXPECT_EQ(std::string(e.what()), "Subdivisions were exhausted")
          << "Should have thrown only for running out of subdivisions";
      break;
    }
  }

  EXPECT_EQ(size, t.ncolumns() * t.nsubdivisions() * t.nrows() * t.nsubdivisions())
      << "Number of subdivisions didnt match";

  EXPECT_TRUE(zeros == 1 || zeros == 2 || zeros == 4)
      << "Got: " << zeros
      << " but only 1, 2 and 4 subdivisions can be 0 distance from the input point";
}

TEST(Tiles, test_closest_first) {

  auto m = PointLL(8.99546623, -78.2651062).Distance({-91.2, 90});
  auto n = PointLL(8.99546623, -78.2651062).Distance({-92, 90});

  // test a simple 8x4 grid for polar and meridian wrapping
  Tiles<PointLL> t(AABB2<PointLL>{-180, -90, 180, 90}, 90, 2);
  auto y = t.ClosestFirst({179.99, -16.825});
  EXPECT_EQ(to_global_sub(y(), t), 15) << "Should have been 15";
  EXPECT_EQ(to_global_sub(y(), t), 8) << "Should have been wrapped to 8";
  EXPECT_EQ(to_global_sub(y(), t), 23) << "Should have been above to 23";
  y = t.ClosestFirst({-179.99, -16.825});
  EXPECT_EQ(to_global_sub(y(), t), 8) << "Should have been 8";
  EXPECT_EQ(to_global_sub(y(), t), 15) << "Should have been wrapped to 15";
  EXPECT_EQ(to_global_sub(y(), t), 16) << "Should have been above to 16";

  // check realistic antimeridian wrapping
  t = Tiles<PointLL>(AABB2<PointLL>{-180, -90, 180, 90}, .25, 5);
  PointLL p{179.99, -16.825};
  int px =
      (p.first - t.TileBounds().minx()) / t.TileBounds().Width() * t.ncolumns() * t.nsubdivisions();
  int py =
      (p.second - t.TileBounds().miny()) / t.TileBounds().Height() * t.nrows() * t.nsubdivisions();
  auto c = t.ClosestFirst(p);
  auto first = c();
  EXPECT_EQ(to_global_sub(first, t), py * t.ncolumns() * t.nsubdivisions() + px)
      << "Unexpected global subdivision";

  EXPECT_EQ(std::get<2>(first), 0) << "Unexpected distance";

  auto second = c();
  EXPECT_EQ(to_global_sub(second, t), py * t.ncolumns() * t.nsubdivisions())
      << "Unexpected global subdivision";
  EXPECT_EQ(std::get<2>(second), p.Distance({-180, -16.825})) << "Unexpected distance";

  // try planar coordinate system
  Tiles<Point2> tp(AABB2<Point2>{-10, -10, 10, 10}, 1, 5);
  for (const auto& p : std::list<Point2>{{0, 0},
                                         {-1.99, -1.99},
                                         {-.03, 1.21},
                                         {7.23, -3.332},
                                         {.04, 8.76},
                                         {9.99, 9.99}})
    test_point(tp, p);

  // try spherical coordinate system
  t = Tiles<PointLL>(AABB2<PointLL>{-180, -90, 180, 90}, 4, 5);
  for (const auto& p : std::list<PointLL>{{0, 0},
                                          {-76.5, 40.5},
                                          {47.31707, 9.2827},
                                          {11.92515, 78.92409},
                                          {-67.61196, -54.93575},
                                          {179.99155, -16.80257},
                                          {179.99, 89.99}})
    test_point(t, p);

  // try some randos
  std::mt19937 generator;
  std::uniform_real_distribution<> distribution(0, 360);
  for (size_t i = 0; i < 25; ++i) {
    PointLL p{distribution(generator) - 180.f, distribution(generator) / 2 - 90.f};
    test_point(t, p);
  }
}

TEST(Tiles, test_intersect_bbox_world) {
  AABB2<PointLL> world_box{-180, -90, 180, 90};
  Tiles<PointLL> t(world_box, 90, 2);
  auto intersection = t.Intersect(world_box);
  EXPECT_EQ(intersection.size(), t.TileCount())
      << "Expected " + std::to_string(t.TileCount()) +
             " tiles returned from world-spanning intersection";

  auto nbins = t.nsubdivisions() * t.nsubdivisions();
  for (const auto& i : intersection) {
    const auto& bins = i.second;
    EXPECT_EQ(bins.size(), nbins) << "For tile " << i.first;
  }
}

TEST(Tiles, test_intersect_bbox_single) {
  AABB2<PointLL> world_box{-180, -90, 180, 90};
  Tiles<PointLL> t(world_box, 90, 2);

  AABB2<PointLL> single_box{1, 1, 2, 2};
  auto intersection = t.Intersect(single_box);
  EXPECT_EQ(intersection.size(), 1) << "Expected one tile returned from world-spanning intersection";

  auto tile_id = intersection.begin()->first;
  auto bins = intersection.begin()->second;
  // expect tile id to be 6 because the point just up and right from the origin
  // should be in the 3rd column, 2nd row, so thats (ncols(=4) * row(=1)) +
  // col(=2).
  EXPECT_EQ(tile_id, 6);

  // there should be a single result bin, which should be in the lower left
  // and therefore be bin 0.
  EXPECT_EQ(bins.size(), 1);

  auto bin_id = *bins.begin();
  EXPECT_EQ(bin_id, 0);
}

TEST(Tiles, test_intersect_bbox_rounding) {
  AABB2<PointLL> world_box{-180, -90, 180, 90};
  Tiles<PointLL> t(world_box, 0.25, 5);

  AABB2<PointLL> single_box{0.5, 0.5, 0.501, 0.501};
  auto intersection = t.Intersect(single_box);
  EXPECT_EQ(intersection.size(), 1) << "Expected one tile returned from intersection";

  auto bins = intersection.begin()->second;
  // expect only the lower left bin, 0
  EXPECT_EQ(bins.size(), 1);

  auto bin_id = *bins.begin();
  EXPECT_EQ(bin_id, 0);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
