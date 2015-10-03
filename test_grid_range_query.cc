#include <iostream>

#include "grid_range_query.h"


void TestGridTools()
{
  BoundingBox bbox(0, 0, 100, 100);
  GridRangeQuery<int> grid(bbox, 1.f, 1.f);

  auto c = grid.GridCoordinates({12.5, 13.7});
  assert(c.first == 12 && c.second == 13);

  auto intersects = grid.CellLineSegmentIntersections(2, 3, LineSegment({2.5, 3.5}, {10, 3.5}));
  assert(intersects.size() == 1);
  assert(intersects[0].point.x() == 3 && intersects[0].point.y() == 3.5);

  LineSegment interior;
  LineSegment segment;
  bool ok;

  segment = LineSegment({0, 0}, {100, 100});
  ok = grid.InteriorLineSegment(segment, interior);
  assert(ok && interior.a() == Point(0, 0) && interior.b() == Point(100, 100));

  segment = LineSegment({0, 0}, {50, 50});
  ok = grid.InteriorLineSegment(segment, interior);
  assert(ok && interior.a() == Point(0, 0) && interior.b() == Point(50, 50));

  segment = LineSegment({50, 50}, {150, 150});
  ok = grid.InteriorLineSegment(segment, interior);
  assert(ok && interior.a() == Point(50, 50) && interior.b() == Point(100, 100));

  segment = LineSegment({-50, -50}, {150, 150});
  ok = grid.InteriorLineSegment(segment, interior);
  assert(ok && interior.a() == Point(0, 0) && interior.b() == Point(100, 100));

  segment = LineSegment({100, 100}, {150, 150});
  ok = grid.InteriorLineSegment(segment, interior);
  assert(!ok);

  segment = LineSegment({120, 120}, {150, 150});
  ok = grid.InteriorLineSegment(segment, interior);
  assert(!ok);

  segment = LineSegment({20, 20}, {80, 80});
  ok = grid.InteriorLineSegment(segment, interior);
  assert(ok && interior.a() == Point(20, 20) && interior.b() == Point(80, 80));

  segment = LineSegment({20, 20}, {20, 20});
  ok = grid.InteriorLineSegment(segment, interior);
  assert(ok && interior.a() == Point(20, 20) && interior.b() == Point(20, 20));

  segment = LineSegment({200, 200}, {200, 200});
  ok = grid.InteriorLineSegment(segment, interior);
  assert(!ok);
}


void TestAddLineSegment()
{
  BoundingBox bbox(0, 0, 100, 100);
  GridRangeQuery<int> grid(bbox, 1.f, 1.f);

  grid.AddLineSegment(0, LineSegment({2.5, 3.5}, {10, 3.5}));
  auto items23 = grid.ItemsInCell(2, 3);
  assert(items23.size() == 1);
  auto items53 = grid.ItemsInCell(5, 3);
  assert(items53.size() == 1);
  auto items88 = grid.ItemsInCell(8, 8);
  assert(items88.empty());

  grid.AddLineSegment(1, LineSegment({10, 3.5}, {2.5, 3.5}));
  auto items33 = grid.ItemsInCell(2, 3);
  assert(items33.size() == 2);

  grid.AddLineSegment(0, LineSegment({-10, -10}, {110, 110}));
  auto items50 = grid.ItemsInCell(50, 50);
  assert(items50.size() == 1);

  auto old_item00_size = grid.ItemsInCell(0, 0).size();
  grid.AddLineSegment(0, LineSegment({0.5, 0.5}, {0.5, 0.5}));
  assert(grid.ItemsInCell(0, 0).size() == old_item00_size + 1);
}


void TestQuery()
{
  BoundingBox bbox(0, 0, 100, 100);
  GridRangeQuery<int> grid(bbox, 1.f, 1.f);

  grid.AddLineSegment(0, LineSegment({2.5, 3.5}, {10, 3.5}));

  auto items = grid.Query(BoundingBox(2, 2, 5, 5));
  assert(items.size() == 1 && items.find(0) != items.end());

  items = grid.Query(BoundingBox(10, 10, 20, 20));
  assert(items.empty());

  items = grid.Query(BoundingBox(2, 3, 2.5, 3.5));
  assert(items.size() == 1 && items.find(0) != items.end());
}


int main(int argc, char *argv[])
{
  TestGridTools();
  TestAddLineSegment();
  TestQuery();
  std::cout << "all tests passed" << std::endl;
  return 0;
}
