// -*- mode: c++ -*-

#include "midgard/linesegment2.h"
#include "midgard/pointll.h"

#include "meili/grid_range_query.h"
#include "test.h"

using namespace valhalla;

using BoundingBox = midgard::AABB2<midgard::PointLL>;
using LineSegment = midgard::LineSegment2<midgard::PointLL>;

void TestAddLineSegment() {
  BoundingBox bbox(0, 0, 100, 100);
  meili::GridRangeQuery<int, midgard::PointLL> grid(bbox, 1.f, 1.f);

  grid.AddLineSegment(0, LineSegment({2.5, 3.5}, {10, 3.5}));
  const auto& items23 = grid.GetItemsInSquare(2, 3);
  test::assert_bool(items23.size() == 1, "should be added to Cell(2, 3)");

  const auto& items53 = grid.GetItemsInSquare(5, 3);
  test::assert_bool(items53.size() == 1, "should be added to Cell(5, 3)");

  const auto& items88 = grid.GetItemsInSquare(8, 8);
  test::assert_bool(items88.empty(), "nothing should be added to Cell(8, 8)");

  grid.AddLineSegment(1, LineSegment({10, 3.5}, {2.5, 3.5}));
  const auto& items33 = grid.GetItemsInSquare(2, 3);
  test::assert_bool(items33.size() == 2, "2 items should be added to Cell(2, 3)");

  grid.AddLineSegment(0, LineSegment({-10, -10}, {110, 110}));
  const auto& items50 = grid.GetItemsInSquare(50, 50);
  test::assert_bool(items50.size() == 1, "should be added to Cell(50, 50)");

  const auto& old_item00_size = grid.GetItemsInSquare(0, 0).size();
  grid.AddLineSegment(0, LineSegment({0.5, 0.5}, {0.5, 0.5}));
  test::assert_bool(grid.GetItemsInSquare(0, 0).size() == old_item00_size + 1,
                    "empty segment should be added");

  // A special case that failed
  {
    BoundingBox bbox(-78.5, 0, -78.25, 0.25);
    // 0.005 == 0.25 / 500 where 0.25 is the tile size and we divided it into 500x500 cells
    meili::GridRangeQuery<int, midgard::PointLL> grid(bbox, 0.005f, 0.005f);
    // Should not throw anything here
    grid.AddLineSegment(1, LineSegment({-78.4831, 0.002865}, {-78.4839, -0.001577}));
  }
}

void TestQuery() {
  const BoundingBox bbox(0, 0, 100, 100);
  meili::GridRangeQuery<int, midgard::PointLL> grid(bbox, 1.f, 1.f);

  grid.AddLineSegment(0, LineSegment({2.5, 3.5}, {10, 3.5}));

  auto items = grid.Query(BoundingBox(2, 2, 5, 5));
  test::assert_bool(items.size() == 1 && items.find(0) != items.end(), "query should get item 0");

  items = grid.Query(BoundingBox(10, 10, 20, 20));
  test::assert_bool(items.empty(), "query should get nothing");

  items = grid.Query(BoundingBox(2, 3, 2.5, 3.5));
  test::assert_bool(items.size() == 1 && items.find(0) != items.end(), "quewry should get item 0");
}

int main(int argc, char* argv[]) {
  test::suite suite("grid range query");

  suite.test(TEST_CASE(TestAddLineSegment));

  suite.test(TEST_CASE(TestQuery));

  return suite.tear_down();
}
