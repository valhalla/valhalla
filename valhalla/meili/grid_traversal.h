// -*- mode: c++ -*-
#ifndef MMP_GRID_TRAVERSAL_H_
#define MMP_GRID_TRAVERSAL_H_

#include <cmath>
#include <limits>
#include <tuple>
#include <vector>

namespace valhalla {
namespace meili {

template <typename coord_t> class GridTraversal {
public:
  GridTraversal(double minx,
                double miny,
                double square_width,
                double square_height,
                int ncols,
                int nrows)
      : minx_(minx), miny_(miny), maxx_(minx + square_width * ncols),
        maxy_(miny + square_height * nrows), square_width_(square_width),
        square_height_(square_height), ncols_(ncols), nrows_(nrows) {
  }

  std::pair<int, int> SquareAtPoint(const coord_t& pt) const {
    const double dx = pt.x() - minx_, dy = pt.y() - miny_;
    return {static_cast<int>(floor(dx / square_width_)),
            static_cast<int>(floor(dy / square_height_))};
  }

  bool IsValidSquare(int col, int row) const {
    return 0 <= col && col < ncols_ && 0 <= row && row < nrows_;
  }

  std::vector<std::pair<int, int>> Traverse(const coord_t& origin, const coord_t& dest) const {
    // Division by zero is undefined in C++, here we ensure it to be
    // infinity
    const double height = dest.y() - origin.y(), width = dest.x() - origin.x(),
                 tangent = width == 0.f ? std::numeric_limits<double>::infinity() : (height / width),
                 cotangent =
                     height == 0.f ? std::numeric_limits<double>::infinity() : (width / height);

    int col, row, dest_col, dest_row;
    std::tie(col, row) = StartSquare(origin, dest, tangent, cotangent);
    std::tie(dest_col, dest_row) = SquareAtPoint(dest);

    // Append intersecting squares
    std::vector<std::pair<int, int>> squares;
    while (!(col == dest_col && row == dest_row) && IsValidSquare(col, row)) {
      squares.emplace_back(col, row);

      // Calculating next intersecting square

      // Intersect with the right border of the square
      if (col < dest_col && IntersectsRow(origin, tangent, col + 1) == row) {
        col++;
        continue;
      }
      // Intersect with the left border of the square
      if (dest_col < col && IntersectsRow(origin, tangent, col) == row) {
        col--;
        continue;
      }
      // Intersect with the top border of the square
      if (row < dest_row && IntersectsColumn(origin, cotangent, row + 1) == col) {
        row++;
        continue;
      }
      // Intersect with the bottom border of the square
      if (dest_row < row && IntersectsColumn(origin, cotangent, row) == col) {
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
  int IntersectsColumn(const coord_t& origin, double cotangent, int row) const {
    // The origin and the cotangent define a ray.

    // If the ray is parallel with horizontal grid lines, return an
    // invalid column
    if (std::isinf(cotangent)) {
      return -1;
    }
    // (intersect_x, intersect_y) is the intersecting point of the ray
    // and the bottom horizontal grid line at the specified row
    const double intersect_y = miny_ + row * square_height_;
    // Since we have (intersect_x - origin.x()) / (intersect_y - origin.y()) == cotangent
    const double intersect_x = (intersect_y - origin.y()) * cotangent + origin.x();
    // Return the intersecting column
    return floor((intersect_x - minx_) / square_width_);
  }

  int IntersectsRow(const coord_t& origin, double tangent, int col) const {
    // The origin and the tangent define a ray.

    // If the ray is parallel with vertical grid lines, return an
    // invalid row
    if (std::isinf(tangent)) {
      return -1;
    }
    // (intersect_x, intersect_y) is the intersecting point of the ray
    // and the left vertical grid line at the specified column
    const double intersect_x = minx_ + col * square_width_;
    // Since we have (intersect_y - origin.y()) / (intersect_x - origin.x()) == tangent
    const double intersect_y = (intersect_x - origin.x()) * tangent + origin.y();
    // Return the intersecting row
    return floor((intersect_y - miny_) / square_height_);
  }

  std::pair<int, int>
  StartSquare(const coord_t& origin, const coord_t& dest, double tangent, double cotangent) const {
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
    else if (origin.x() >= maxx_) {
      const auto right_row = IntersectsRow(origin, tangent, ncols_ - 1);
      if (dest.x() < maxx_ && IsValidSquare(ncols_ - 1, right_row)) {
        return {ncols_ - 1, right_row};
      }
    }

    // Bottom side of the grid
    if (origin.y() < miny_) {
      const auto bottom_col = IntersectsColumn(origin, cotangent, 0);
      if (miny_ <= dest.y() && IsValidSquare(bottom_col, 0)) {
        return {bottom_col, 0};
      }
    }
    // Top side of the grid
    else if (origin.y() >= maxy_) {
      const auto top_col = IntersectsColumn(origin, cotangent, nrows_ - 1);
      if (dest.y() < maxy_ && IsValidSquare(top_col, nrows_ - 1)) {
        return {top_col, nrows_ - 1};
      }
    }

    // Not intersecting the grid
    return {-1, -1};
  }

  double minx_, miny_, maxx_, maxy_;
  double square_width_, square_height_;
  int ncols_, nrows_;
};

} // namespace meili
} // namespace valhalla
#endif // MMP_GRID_TRAVERSAL_H_
