// -*- mode: c++ -*-
#ifndef MMP_GRID_TRAVERSAL_H_
#define MMP_GRID_TRAVERSAL_H_

#include <vector>
#include <cmath>
#include <limits>

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/linesegment2.h>


namespace valhalla{
namespace meili {

template <typename coord_t>
class GridTraversal
{
 public:
  GridTraversal(float minx, float miny,
                float square_width, float square_height,
                int ncols, int nrows)
      : minx_(minx),
        miny_(miny),
        maxx_(minx + square_width * ncols),
        maxy_(miny + square_height * nrows),
        square_width_(square_width),
        square_height_(square_height),
        ncols_(ncols),
        nrows_(nrows) {}

  std::pair<int, int> SquareAtPoint(const coord_t &pt) const
  {
    const float dx = pt.x() - minx_,
                dy = pt.y() - miny_;
    return {static_cast<int>(floor(dx / square_width_)),
            static_cast<int>(floor(dy / square_height_))};
  }

  bool IsValidSquare(int col, int row) const
  { return 0 <= col && col < ncols_ && 0 <= row && row < nrows_; }

  std::vector<std::pair<int, int> >
  Traverse(const midgard::LineSegment2<coord_t>& segment) const
  {
    // Division by zero is undefined in C++, here we ensure it to be
    // infinity
    const float height = segment.b().y() - segment.a().y(),
                 width = segment.b().x() - segment.a().x(),
               tangent = width == 0.f? std::numeric_limits<float>::infinity() : (height / width),
             cotangent = height == 0.f? std::numeric_limits<float>::infinity() : (width / height);

    int col, row, dest_col, dest_row;
    std::tie(col, row) = StartSquare(segment, tangent, cotangent);
    std::tie(dest_col, dest_row) = SquareAtPoint(segment.b());

    // Append intersecting squares
    std::vector<std::pair<int, int>> squares;
    while (!(col == dest_col && row == dest_row) && IsValidSquare(col, row)) {
      squares.emplace_back(col, row);

      // Calculating next intersecting square

      // Intersect with the right border of the square
      if (col < dest_col && IntersectsRow(segment.a(), tangent, col + 1) == row) {
        col++;
        continue;
      }
      // Intersect with the left border of the square
      if (dest_col < col && IntersectsRow(segment.a(), tangent, col) == row) {
        col--;
        continue;
      }
      // Intersect with the top border of the square
      if (row < dest_row && IntersectsColumn(segment.a(), cotangent, row + 1) == col) {
        row++;
        continue;
      }
      // Intersect with the bottom border of the square
      if (dest_row < row && IntersectsColumn(segment.a(), cotangent, row) == col) {
        row--;
        continue;
      }

      // Move along a diagonal line if no intersecting squares found
      // among four neighbors
      if (col < dest_col) {
        col++;
      } else if (dest_col < col) {
        col--;
      }
      if (row < dest_row) {
        row++;
      } else if (dest_row < row) {
        row--;
      }
    }

    // Append the last square
    if (IsValidSquare(col, row)) {
      squares.emplace_back(col, row);
    }

    return squares;
  }

 private:
  int IntersectsColumn(const coord_t& origin, float cotangent, int row) const
  {
    // The origin and the cotangent define a ray.

    // If the ray is parallel with horizontal grid lines, return an
    // invalid column
    if (std::isinf(cotangent)) {
      return -1;
    }
    // (intersect_x, intersect_y) is the intersecting point of the ray
    // and the bottom horizontal grid line at the specified row
    const float intersect_y = miny_ + row * square_height_;
    // Since we have (intersect_x - origin.x()) / (intersect_y - origin.y()) == cotangent
    const float intersect_x = (intersect_y - origin.y()) * cotangent + origin.x();
    // Return the intersecting column
    return (intersect_x - minx_) / square_width_;
  }

  int IntersectsRow(const coord_t& origin, float tangent, int col) const
  {
    // The origin and the tangent define a ray.

    // If the ray is parallel with vertical grid lines, return an
    // invalid row
    if (std::isinf(tangent)) {
      return -1;
    }
    // (intersect_x, intersect_y) is the intersecting point of the ray
    // and the left vertical grid line at the specified column
    const float intersect_x = minx_ + col * square_width_;
    // Since we have (intersect_y - origin.y()) / (intersect_x - origin.x()) == tangent
    const float intersect_y = (intersect_x - origin.x()) * tangent + origin.y();
    // Return the intersecting row
    return (intersect_y - miny_) / square_height_;
  }

  std::pair<int, int>
  StartSquare(const midgard::LineSegment2<coord_t>& segment, float tangent, float cotangent) const
  {
    const auto &origin = segment.a(),
                 &dest = segment.b();

    // Return the square if origin point falls inside the grid
    int col, row;
    std::tie(col, row) = SquareAtPoint(origin);
    if (IsValidSquare(col, row)) {
      return {col, row};
    }

    // If it falls outside the grid, then we find the first
    // intersecting square along the segment

    // Left side of the grid
    if (origin.x() < minx_) {
      const auto left_row = IntersectsRow(origin, tangent, 0);
      if (minx_ <= dest.x() && IsValidSquare(0, left_row)) {
        return {0, left_row};
      }
    }
    // Right side of the grid
    else {
      // Assert maxx_ <= origin.x()
      const auto right_row = IntersectsRow(origin, tangent, ncols_ - 1);
      if (dest.x() < maxx_ && IsValidSquare(ncols_ - 1, right_row)) {
        return {ncols_ - 1, right_row};
      }
    }

    // Bottom side of the grid
    if (origin.y() < miny_) {
      const auto bottom_col = IntersectsColumn(origin, cotangent, nrows_ - 1);
      if (miny_ <= dest.y() && IsValidSquare(bottom_col, 0)) {
        return {bottom_col, 0};
      }
    }
    // Top side of the grid
    else {
      // Assert maxy_ <= origin.y()
      const auto top_col = IntersectsColumn(origin, cotangent, 0);
      if (dest.y() < maxy_ && IsValidSquare(top_col, nrows_ - 1)) {
        return {top_col, nrows_ - 1};
      }
    }

    // Not intersecting the grid
    return {-1, -1};
  }

  float minx_, miny_, maxx_, maxy_;
  float square_width_, square_height_;
  unsigned ncols_, nrows_;
};

}
}
#endif // MMP_GRID_TRAVERSAL_H_
