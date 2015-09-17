// -*- mode: c++ -*-
// compile: [clang++ | g++] -Wall -std=c++11 grid_range_query.cc

#include <algorithm>
#include <tuple>
#include <pair>


using GraphId = uint32_t;
using PointLL = std::pair<float, float>;  // longitude, latitude (i.e. x and y)
using LineSegment = std::pair<PointLL, PointLL>;  // start point, end point
using AABB2 = std::tuple<float, float, float, float>;  // left, bottom, right, top
enum {LEFT=0, BOTTOM, RIGHT, TOP};


class GridRangeQuery
{
 public:
  GridRangeQuery(const AABB2& bbox, float cell_width, float cell_height)
      : bbox_(bbox), cell_width_(cell_width), cell_height_(cell_height)
  {
  }

  // Divide the grid into num_cols by num_rows cells
  GridRangeQuery(const AABB2& bbox, uint32_t num_cols, uint32_t num_rows)
      : bbox_(bbox)
  {
    auto grid_width = std::abs(std::get<RIGHT>(bbox) - std::get<LEFT>(bbox)),
        grid_height = std::abs(std::get<TOP>(bbox) - std::get<BOTTOM>(bbox));
    cell_width_ = grid_width / num_cols;
    cell_height_ = grid_height / num_rows;
  }

  // Get bbox of the grid
  const AABB2 bbox() const {
    return bbox_;
  }

  // TODO more getters here

  // Index a line segment into the grid
  virtual void AddLineSegment(const GraphId edgeid, const LineSegment& segment);

  // Query all edges that intersects with the range
  virtual std::vector<GraphId> Query(const AABB2& range) const;

 private:
  AABB2 bbox_;
  float cell_width_;
  float cell_height_;
};


void TestGridRangeQuery()
{
  AABB2 bbox(0, 0, 100, 100);
  // Divide the grid into 100x100 cells
  GridRangeQuery grid(bbox, 100u, 100u);

  grid.AddLineSegment(0, LineSegment({0, 0}, {0.5, 0.5}));

  auto edges = grid.Query(AABB2(0, 0, 0.5, 0.5));
  assert(edges.size() == 1 && edges[0] == 0);

  edges = grid.Query(AABB2(0.6, 0.6, 1, 1));
  assert(edges.empty());

  // TODO more tests
}


int main(int argc, char *argv[])
{
  TestGridRangeQuery();
  return 0;
}
