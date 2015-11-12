#ifndef VALHALLA_MIDGARD_GRID_H_
#define VALHALLA_MIDGARD_GRID_H_

#include <vector>
#include <unordered_set>

#include <valhalla/midgard/aabb2.h>

namespace valhalla {
namespace midgard {

/**
 * A container for a bunch of bounding boxes contained within a larger box
 * Intersection methods give the index of the sub cell of the larger grid
 * Note: the grid is oriented such that its origin corresponds with the minimum
 * coordinate provided and that with increasing coordinates in both dimensions
 * the sub cell indices increase. In other words the 0th index cell contains
 * the minimum point of the grid and conversely the (divisions*divisions - 1)th
 * index cell contains the maximum point.
 */
template <class coord_t>
class grid {
 public:
  using x_t = typename coord_t::first_type;
  using y_t = typename coord_t::second_type;

  /**
   * Constructor for grid
   * @param extents    the extents of the grid in the form of an AABB2
   * @param divisions  the number of rows/columns within the grid
   */
  grid(const AABB2<coord_t>& extents, size_t divisions);

  /**
   * Constructor for grid
   * @param min        the minimum extreme point of the grid
   * @param max        the maximum extreme point of the grid
   * @param divisions  the number of rows/columns within the grid
   */
  grid(const coord_t& min, const coord_t& max, size_t divisions);

  /**
   * Intersect the linestring with the grid to see which cells it touches
   * @param line_string  the linestring to be tested against the cells
   * @param uncontained  a flag indicating that the linestring is not entirely
   *                     contained within the grid. ie. it intersects adjacent grids
   * @return             the set of cells indices which intersect the linestring
   */
  template <class container_t>
  std::unordered_set<size_t> intersect(const container_t& linestring, bool& uncontained) const;

  /**
   * Intersect a circle with the grid to see which cells it touches
   * @param center  the center of the circle
   * @param radius  the radius of the circle
   * @return        the set of cells indices which intersect the circle
   */
  std::unordered_set<size_t> intersect(const coord_t& center, const float radius) const;

  /**
   * @return  the number of cells within this grid
   */
  size_t size() const;

  /**
   * @return  the extent encompasing this grid
   */
  const AABB2<coord_t>& extent() const;

 protected:
  size_t divisions;
  AABB2<coord_t> super_cell;
  std::vector<AABB2<coord_t> > cells;
  x_t x_index_coef;
  y_t y_index_coef;
};

}
}

#endif  // VALHALLA_MIDGARD_GRID_H_
