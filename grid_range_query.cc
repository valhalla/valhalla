// -*- mode: c++ -*-
// compile: [c++ | g++] -Wall -std=c++11 grid_range_query.cc

#include <algorithm>
#include <tuple>
#include <utility>
#include <vector>
#include <cmath>
#include <cassert>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/linesegment2.h>


using GraphId = uint32_t;
using PointLL = valhalla::midgard::PointLL;
using LineSegment = valhalla::midgard::LineSegment2<PointLL>;
using BoundingBox = valhalla::midgard::AABB2<PointLL>;



std::vector<PointLL>
LineSegmentBoundingBoxIntersection(const LineSegment &s, const BoundingBox &b) {

  return {};
}


class GridRangeQuery
{
 public:
  GridRangeQuery(const BoundingBox& bbox, float cell_width, float cell_height)
      : bbox_(bbox), cell_width_(cell_width), cell_height_(cell_height)
  {
  }

  // Divide the grid into num_cols by num_rows cells
  GridRangeQuery(const BoundingBox& bbox, uint32_t num_cols, uint32_t num_rows)
      : bbox_(bbox)
  {
    cell_width_ = bbox.Width() / num_cols;
    cell_height_ = bbox.Height() / num_rows;
  }

  // Get bbox of the grid
  const BoundingBox bbox() const {
    return bbox_;
  }

  // TODO more getters here

  // Index a line segment into the grid
  void AddLineSegment(const GraphId edgeid, const LineSegment& segment) {}

  // Query all edges that intersects with the range
  std::vector<GraphId> Query(const BoundingBox& range) const {
    return {};
  }

 private:
  BoundingBox bbox_;
  float cell_width_;
  float cell_height_;
};


void TestGridRangeQuery()
{
  BoundingBox bbox(0, 0, 100, 100);
  // Divide the grid into 100x100 cells
  GridRangeQuery grid(bbox, 100u, 100u);

  grid.AddLineSegment(0, LineSegment({0, 0}, {0.5, 0.5}));

  auto edges = grid.Query(BoundingBox(0, 0, 0.5, 0.5));
  assert(edges.size() == 1 && edges[0] == 0);

  edges = grid.Query(BoundingBox(0.6, 0.6, 1, 1));
  assert(edges.empty());

  // TODO more tests
}


int main(int argc, char *argv[])
{
  TestGridRangeQuery();
  return 0;
}
