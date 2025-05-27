// -*- mode: c++ -*-

#include "meili/grid_range_query.h"
#include "midgard/linesegment2.h"
#include "midgard/pointll.h"
#include "test.h"

namespace {

using namespace valhalla;

using BoundingBox = midgard::AABB2<midgard::PointLL>;
using LineSegment = midgard::LineSegment2<midgard::PointLL>;

TEST(GridRangeQuery, TestAddLineSegment) {
  BoundingBox bbox(0, 0, 100, 100);
  meili::GridRangeQuery<int, midgard::PointLL> grid(bbox, 1.f, 1.f);

  grid.AddLineSegment(0, LineSegment({2.5, 3.5}, {10, 3.5}));
  const auto& items23 = grid.GetItemsInSquare(2, 3);
  EXPECT_EQ(items23.size(), 1) << "should be added to Cell(2, 3)";

  const auto& items53 = grid.GetItemsInSquare(5, 3);
  EXPECT_EQ(items53.size(), 1) << "should be added to Cell(5, 3)";

  const auto& items88 = grid.GetItemsInSquare(8, 8);
  EXPECT_TRUE(items88.empty()) << "nothing should be added to Cell(8, 8)";

  grid.AddLineSegment(1, LineSegment({10, 3.5}, {2.5, 3.5}));
  const auto& items33 = grid.GetItemsInSquare(2, 3);
  EXPECT_EQ(items33.size(), 2) << "2 items should be added to Cell(2, 3)";

  grid.AddLineSegment(0, LineSegment({-10, -10}, {110, 110}));
  const auto& items50 = grid.GetItemsInSquare(50, 50);
  EXPECT_EQ(items50.size(), 1) << "should be added to Cell(50, 50)";

  const auto& old_item00_size = grid.GetItemsInSquare(0, 0).size();
  grid.AddLineSegment(0, LineSegment({0.5, 0.5}, {0.5, 0.5}));
  EXPECT_EQ(grid.GetItemsInSquare(0, 0).size(), old_item00_size + 1)
      << "empty segment should be added";

  // A special case that failed
  {
    BoundingBox bbox(-78.5, 0, -78.25, 0.25);
    // 0.005 == 0.25 / 500 where 0.25 is the tile size and we divided it into 500x500 cells
    meili::GridRangeQuery<int, midgard::PointLL> grid(bbox, 0.005f, 0.005f);
    // Should not throw anything here
    grid.AddLineSegment(1, LineSegment({-78.4831, 0.002865}, {-78.4839, -0.001577}));
  }
}

TEST(GridRangeQuery, TestQuery) {
  const BoundingBox bbox(0, 0, 100, 100);
  meili::GridRangeQuery<int, midgard::PointLL> grid(bbox, 1.f, 1.f);

  grid.AddLineSegment(0, LineSegment({2.5, 3.5}, {10, 3.5}));

  auto items = grid.Query(BoundingBox(2, 2, 5, 5));
  EXPECT_EQ(items.size(), 1 && items.find(0) != items.end()) << "query should get item 0";

  items = grid.Query(BoundingBox(10, 10, 20, 20));
  EXPECT_TRUE(items.empty()) << "query should get nothing";

  items = grid.Query(BoundingBox(2, 3, 2.5, 3.5));
  EXPECT_EQ(items.size(), 1);
  EXPECT_NE(items.find(0), items.end()) << "query should get item 0";
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
