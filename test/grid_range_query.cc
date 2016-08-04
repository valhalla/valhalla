// -*- mode: c++ -*-

#include "test.h"
#include "meili/grid_range_query.h"

using namespace valhalla::meili;


void TestGridTools()
{
  const BoundingBox bbox(0, 0, 100, 100);
  GridRangeQuery<int> grid(bbox, 1.f, 1.f);

  {
    const auto c = grid.GridCoordinates({12.5, 13.7});
    test::assert_bool(c.first == 12 && c.second == 13,
                      "should get the right grid coodinate");
  }

  {
    const auto intersects = BoundingBoxLineSegmentIntersections(grid.CellBoundingBox(2, 3),
                                                                LineSegment({2.5, 3.5}, {10, 3.5}));
    test::assert_bool(intersects.size() == 1, "should be intersected");
    test::assert_bool(intersects[0].point.x() == 3 && intersects[0].point.y() == 3.5,
                      "intersected coordinate should be correct");
  }

  {
    const LineSegment segment({0, 0}, {100, 100});
    LineSegment interior;
    const auto ok = InteriorLineSegment(grid.bbox(), segment, interior);
    test::assert_bool(ok && interior.a() == Point(0, 0) && interior.b() == Point(100, 100),
                      "intersection should be LINESTRING(0 0, 100 100)");
  }

  {
    const LineSegment segment({0, 0}, {50, 50});
    LineSegment interior;
    const auto ok = InteriorLineSegment(grid.bbox(), segment, interior);
    test::assert_bool(ok && interior.a() == Point(0, 0) && interior.b() == Point(50, 50),
                      "intersection should be LINESTRING(0 0, 50 50)");
  }

  {
    const LineSegment segment({50, 50}, {150, 150});
    LineSegment interior;
    const auto ok = InteriorLineSegment(grid.bbox(), segment, interior);
    test::assert_bool(ok && interior.a() == Point(50, 50) && interior.b() == Point(100, 100),
                      "intersection should be LINESTRING(50 50, 100 100)");
  }

  {
    const LineSegment segment({-50, -50}, {150, 150});
    LineSegment interior;
    const auto ok = InteriorLineSegment(grid.bbox(), segment, interior);
    test::assert_bool(ok && interior.a() == Point(0, 0) && interior.b() == Point(100, 100),
                      "intersection should be LINESTRING(0 0, 100 100)");
  }

  {
    const LineSegment segment({100, 100}, {150, 150});
    LineSegment interior;
    const auto ok = InteriorLineSegment(grid.bbox(), segment, interior);
    test::assert_bool(!ok, "Should be empty intersection");
  }

  {
    const LineSegment segment({120, 120}, {150, 150});
    LineSegment interior;
    const auto ok = InteriorLineSegment(grid.bbox(), segment, interior);
    test::assert_bool(!ok, "Should be empty intersection");
  }

  {
    const LineSegment segment({20, 20}, {80, 80});
    LineSegment interior;
    const auto ok = InteriorLineSegment(grid.bbox(), segment, interior);
    test::assert_bool(ok && interior.a() == Point(20, 20) && interior.b() == Point(80, 80),
                      "intersection should be LINESTRING(20 20, 80 80)");
  }

  {
    const LineSegment segment({20, 20}, {20, 20});
    LineSegment interior;
    const auto ok = InteriorLineSegment(grid.bbox(), segment, interior);
    test::assert_bool(ok && interior.a() == Point(20, 20) && interior.b() == Point(20, 20),
                      "intersection should be LINESTRING(20 20, 20 20)");
  }

  {
    const LineSegment segment({200, 200}, {200, 200});
    LineSegment interior;
    const auto ok = InteriorLineSegment(grid.bbox(), segment, interior);
    test::assert_bool(!ok, "should be empty intersection");
  }
}


void TestAddLineSegment()
{
  BoundingBox bbox(0, 0, 100, 100);
  GridRangeQuery<int> grid(bbox, 1.f, 1.f);

  grid.AddLineSegment(0, LineSegment({2.5, 3.5}, {10, 3.5}));
  const auto& items23 = grid.GetItemsInCell(2, 3);
  test::assert_bool(items23.size() == 1, "should be added to Cell(2, 3)");

  const auto& items53 = grid.GetItemsInCell(5, 3);
  test::assert_bool(items53.size() == 1, "should be added to Cell(5, 3)");

  const auto& items88 = grid.GetItemsInCell(8, 8);
  test::assert_bool(items88.empty(), "nothing should be added to Cell(8, 8)");

  grid.AddLineSegment(1, LineSegment({10, 3.5}, {2.5, 3.5}));
  const auto& items33 = grid.GetItemsInCell(2, 3);
  test::assert_bool(items33.size() == 2, "2 items should be added to Cell(2, 3)");

  grid.AddLineSegment(0, LineSegment({-10, -10}, {110, 110}));
  const auto& items50 = grid.GetItemsInCell(50, 50);
  test::assert_bool(items50.size() == 1, "should be added to Cell(50, 50)");

  const auto& old_item00_size = grid.GetItemsInCell(0, 0).size();
  grid.AddLineSegment(0, LineSegment({0.5, 0.5}, {0.5, 0.5}));
  test::assert_bool(grid.GetItemsInCell(0, 0).size() == old_item00_size + 1,
                    "empty segment should be added");
}


void TestQuery()
{
  const BoundingBox bbox(0, 0, 100, 100);
  GridRangeQuery<int> grid(bbox, 1.f, 1.f);

  grid.AddLineSegment(0, LineSegment({2.5, 3.5}, {10, 3.5}));

  auto items = grid.Query(BoundingBox(2, 2, 5, 5));
  test::assert_bool(items.size() == 1 && items.find(0) != items.end(),
                    "query should get item 0");

  items = grid.Query(BoundingBox(10, 10, 20, 20));
  test::assert_bool(items.empty(), "query should get nothing");

  items = grid.Query(BoundingBox(2, 3, 2.5, 3.5));
  test::assert_bool(items.size() == 1 && items.find(0) != items.end(),
                    "quewry should get item 0");
}


int main(int argc, char *argv[])
{
  test::suite suite("grid range query");

  suite.test(TEST_CASE(TestGridTools));

  suite.test(TEST_CASE(TestAddLineSegment));

  suite.test(TEST_CASE(TestQuery));

  return suite.tear_down();
}
