// -*- mode: c++ -*-
// compile: [c++ | g++] -Wall -std=c++11 grid_range_query.cc

#include <iostream>
#include <algorithm>
#include <tuple>
#include <utility>
#include <vector>
#include <set>
#include <cmath>
#include <cassert>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/linesegment2.h>


using GraphId = uint32_t;
using Point = valhalla::midgard::PointLL;
using LineSegment = valhalla::midgard::LineSegment2<Point>;
using BoundingBox = valhalla::midgard::AABB2<Point>;




class GridRangeQuery
{
 public:
  GridRangeQuery(const BoundingBox& bbox, float cell_width, float cell_height) {
    Init(bbox, cell_width, cell_height);
  }


  // Divide the grid into num_cols by num_rows cells
  GridRangeQuery(const BoundingBox& bbox, uint32_t num_cols, uint32_t num_rows) {
    Init(bbox, bbox.Width() / num_cols, bbox.Height() / num_rows);
  }


  void Init(const BoundingBox& bbox, float cell_width, float cell_height) {
    bbox_ = bbox;
    cell_width_ = cell_width;
    cell_height_ = cell_height;
    num_rows_ = ceil(bbox_.Width() / cell_width);
    num_cols_ = ceil(bbox_.Height() / cell_height);
    items_.resize(num_cols_ * num_rows_);
  }


  // Get bbox of the grid
  const BoundingBox bbox() const {
    return bbox_;
  }


  std::pair<int, int> GridCoordinates(const Point &p) const {
    float dx = p.x() - bbox_.minx();
    float dy = p.y() - bbox_.miny();
    return { int(dx / cell_width_), int(dy / cell_height_) };
  }


  BoundingBox CellBoundingBox(int i, int j) const {
    return BoundingBox(
        bbox_.minx() + i * cell_width_,
        bbox_.miny() + j * cell_height_,
        bbox_.minx() + (i + 1) * cell_width_,
        bbox_.miny() + (j + 1) * cell_height_
      );
  }


  const std::vector<GraphId> &ItemsInCell(int i, int j) const {
    return items_[i + j * num_cols_];
  }

  std::vector<GraphId> &ItemsInCell(int i, int j) {
    return items_[i + j * num_cols_];
  }


  bool InteriorLineSegment(const LineSegment &segment, LineSegment &interior) {
    Point a = segment.a();
    Point b = segment.b();

    auto intersects = BoundingBoxLineSegmentIntersections(bbox_, LineSegment(a, b));
    if (bbox_.Contains(a)) intersects.push_back(a);
    if (bbox_.Contains(b)) intersects.push_back(b);

    float mint = 1, maxt = 0;
    Point minp, maxp;
    for (const auto &p : intersects) {
      float t = Unlerp(a, b, p);
      if (t < mint) {
        mint = t;
        minp = p;
      }
      if (t > maxt) {
        maxt = t;
        maxp = p;
      }
    }

    if (mint < 1 && maxt > 0) {
      interior = LineSegment(minp, maxp);
      return true;
    } else {
      return false;
    }
  }


  // Index a line segment into the grid
  void AddLineSegment(const GraphId edgeid, const LineSegment& segment) {
    // For now assume the segment is entirely inside the box
    LineSegment interior;
    if (!InteriorLineSegment(segment, interior)) return;

    Point start = interior.a();
    Point end = interior.b();
    Point current_point = start;

    // Walk along start,end
    while (Unlerp(start, end, current_point) < 1.0) {
      int i, j;
      std::tie(i, j) = GridCoordinates(current_point);
      ItemsInCell(i, j).push_back(edgeid);

      auto intersects = CellLineSegmentIntersections(i, j, LineSegment(current_point, end));

      float bestt = 0;
      Point bestp;
      for (const auto &p : intersects) {
        float t = Unlerp(current_point, end, p);
        if (t > bestt) {
          bestt = t;
          bestp = p;
        }
      }
      if (bestt > 0) {
        current_point = bestp;
      } else {
        break;
      }
    }
  }


  // Query all edges that intersects with the range
  std::set<GraphId> Query(const BoundingBox& range) const {
    std::set<GraphId> results;

    int mini, minj, maxi, maxj;
    std::tie(mini, minj) = GridCoordinates({range.minx(), range.miny()});
    std::tie(maxi, maxj) = GridCoordinates({range.maxx(), range.maxy()});

    mini = std::max(0, std::min(mini, num_cols_));
    maxi = std::max(0, std::min(maxi, num_cols_));
    minj = std::max(0, std::min(minj, num_rows_));
    maxj = std::max(0, std::min(maxj, num_rows_));

    for (int i = mini; i < maxi; ++i) {
      for (int j = minj; j < maxj; ++j) {
        auto items = ItemsInCell(i, j);
        results.insert(items.begin(), items.end());
      }
    }

    return results;
  }


  // Return t such that p = a + t * (b - a)
  float Unlerp(const Point &a, const Point &b, const Point &p) const {
    if (std::abs(b.x() - a.x()) > std::abs(b.y() - a.y())) {
      return (p.x() - a.x()) / (b.x() - a.x());
    } else {
      return (p.y() - a.y()) / (b.y() - a.y());
    }
  }


  std::vector<Point>
  CellLineSegmentIntersections(int i, int j, const LineSegment &segment) const {
    BoundingBox box = CellBoundingBox(i, j);
    return BoundingBoxLineSegmentIntersections(box, segment);
  }


  std::vector<Point>
  BoundingBoxLineSegmentIntersections(const BoundingBox &box, const LineSegment &segment) const {
    std::vector<Point> intersects;

    LineSegment e1({box.minx(), box.miny()}, {box.maxx(), box.miny()});
    LineSegment e2({box.maxx(), box.miny()}, {box.maxx(), box.maxy()});
    LineSegment e3({box.maxx(), box.maxy()}, {box.minx(), box.maxy()});
    LineSegment e4({box.minx(), box.maxy()}, {box.minx(), box.miny()});

    Point intersect;
    if (segment.Intersect(e1, intersect)) intersects.push_back(intersect);
    if (segment.Intersect(e2, intersect)) intersects.push_back(intersect);
    if (segment.Intersect(e3, intersect)) intersects.push_back(intersect);
    if (segment.Intersect(e4, intersect)) intersects.push_back(intersect);

    return intersects;
  }


 private:
  BoundingBox bbox_;
  float cell_width_;
  float cell_height_;
  int num_rows_;
  int num_cols_;
  std::vector<std::vector<GraphId> > items_;
};


void TestGridTools()
{
  BoundingBox bbox(0, 0, 100, 100);
  GridRangeQuery grid(bbox, 100u, 100u);

  auto c = grid.GridCoordinates({12.5, 13.7});
  assert(c.first == 12 && c.second == 13);

  auto intersects = grid.CellLineSegmentIntersections(2, 3, LineSegment({2.5, 3.5}, {10, 3.5}));
  assert(intersects.size() == 1);
  assert(intersects[0].x() == 3 && intersects[0].y() == 3.5);
}


void TestAddLineSegment()
{
  BoundingBox bbox(0, 0, 100, 100);
  GridRangeQuery grid(bbox, 100u, 100u);

  grid.AddLineSegment(0, LineSegment({2.5, 3.5}, {10, 3.5}));
  auto items23 = grid.ItemsInCell(2, 3);
  assert(items23.size() == 1);
  auto items53 = grid.ItemsInCell(5, 3);
  assert(items53.size() == 1);
  auto items88 = grid.ItemsInCell(8, 8);
  assert(items88.empty());

  grid.AddLineSegment(0, LineSegment({-10, -10}, {110, 110}));
  auto items50 = grid.ItemsInCell(50, 50);
  assert(items50.size() == 1);
}


void TestQuery()
{
  BoundingBox bbox(0, 0, 100, 100);
  GridRangeQuery grid(bbox, 100u, 100u);

  grid.AddLineSegment(0, LineSegment({2.5, 3.5}, {10, 3.5}));

  auto items = grid.Query(BoundingBox(2, 2, 5, 5));
  assert(items.size() == 1 && items.find(0) != items.end());

  items = grid.Query(BoundingBox(10, 10, 20, 20));
  assert(items.empty());
}


int main(int argc, char *argv[])
{
  TestGridTools();
  TestAddLineSegment();
  TestQuery();
  return 0;
}
