// -*- mode: c++ -*-
#ifndef MMP_GRID_RANGE_QUERY_H_
#define MMP_GRID_RANGE_QUERY_H_

#include <algorithm>
#include <tuple>
#include <utility>
#include <vector>
#include <unordered_set>
#include <cmath>
#include <stdexcept>

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/linesegment2.h>
#include <valhalla/midgard/distanceapproximator.h>

#include <valhalla/meili/grid_traversal.h>


namespace valhalla{
namespace meili {

using Point = midgard::PointLL;
using LineSegment = midgard::LineSegment2<Point>;
using BoundingBox = midgard::AABB2<Point>;


template <typename key_t>
class GridRangeQuery
{
 public:
  GridRangeQuery(const BoundingBox& bbox, float square_width, float square_height):
      bbox_(bbox),
      square_width_(square_width),
      square_height_(square_height),
      ncols_((bbox.maxx() - bbox.minx()) / square_width),
      nrows_((bbox.maxy() - bbox.miny()) / square_height),
      traverser_(bbox.minx(), bbox.miny(), square_width, square_height, ncols_, nrows_),
      items_()
  { items_.resize(ncols_ * nrows_); }

  const BoundingBox& bbox() const
  { return bbox_; }

  int nrows() const
  { return nrows_; }

  int ncols() const
  { return ncols_; }

  float square_width() const
  { return square_width_; }

  float square_height() const
  { return square_height_; }

  const std::vector<key_t>& GetItemsInCell(int i, int j) const
  {
    if (!(0 <= i && i < nrows_) || !(0 <= j && j < ncols_)) {
      throw std::runtime_error("Cell index (" + std::to_string(i)  + ", " + std::to_string(j)
                               + ") is out of the grid bounds (" + std::to_string(nrows_) + "x"
                               + std::to_string(ncols_) + ")");
    }
    return items_[i + j * ncols_];
  }

  // Index a line segment into the grid
  void AddLineSegment(const key_t edgeid, const LineSegment& segment)
  {
    for (const auto& square: traverser_.Traverse(segment)) {
      ItemsInCell(square.first, square.second).push_back(edgeid);
    }
  }

  // Query all edges that intersects with the range
  std::unordered_set<key_t> Query(const BoundingBox& range) const {
    std::unordered_set<key_t> results;

    int mini, minj, maxi, maxj;
    std::tie(mini, minj) = traverser_.SquareAtPoint(range.minpt());
    std::tie(maxi, maxj) = traverser_.SquareAtPoint(range.maxpt());

    mini = std::max(0, std::min(mini, ncols_ - 1));
    maxi = std::max(0, std::min(maxi, ncols_ - 1));
    minj = std::max(0, std::min(minj, nrows_ - 1));
    maxj = std::max(0, std::min(maxj, nrows_ - 1));

    for (int i = mini; i <= maxi; ++i) {
      for (int j = minj; j <= maxj; ++j) {
        const auto& items = GetItemsInCell(i, j);
        results.insert(items.begin(), items.end());
      }
    }

    return results;
  }

 private:
  std::vector<key_t>& ItemsInCell(int i, int j)
  {
    if (!(0 <= i && i < nrows_) || !(0 <= j && j < ncols_)) {
      throw std::runtime_error("Cell index (" + std::to_string(i)  + ", " + std::to_string(j)
                               + ") is out of the grid bounds (" + std::to_string(nrows_) + "x"
                               + std::to_string(ncols_) + " cells)");
    }
    return items_[i + j * ncols_];
  }

  BoundingBox bbox_;
  float square_width_;
  float square_height_;
  int ncols_;
  int nrows_;
  GridTraversal<Point> traverser_;
  std::vector<std::vector<key_t> > items_;
};

}
}
#endif // MMP_GRID_RANGE_QUERY_H_
