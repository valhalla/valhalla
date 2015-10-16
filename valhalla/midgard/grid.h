#ifndef VALHALLA_MIDGARD_GRID_H_
#define VALHALLA_MIDGARD_GRID_H_

#include <vector>
#include <unordered_set>

#include <valhalla/midgard/aabb2.h>

namespace valhalla {
namespace midgard {

template <class coord_t>
class grid {
 public:
  using x_t = typename coord_t::first_type;
  using y_t = typename coord_t::second_type;

  /**
   * Constructor for grid
   * @param extents    the extents of the grid in the form of an AABB2
   * @param divisions  the number of divisions in the grid in both the x and y axis
   */
  grid(const AABB2<coord_t>& extents, size_t divisions);

  /**
   * Constructor for grid
   * @param min        the minimum extreme point of the grid
   * @param max        the maximum extreme point of the grid
   * @param divisions  the number of divisions in the grid in both the x and y axis
   */
  grid(const coord_t& min, const coord_t& max, size_t divisions);

  /**
   * Intersect the linestring with the grid to see which cells it touches
   * @param line_string  the linestring to be tested against the cells
   * @return             the set of cells indices which intersect the linestring
   */
  template <class container_t>
  std::unordered_set<size_t> intersect(const container_t& linestring) const;

  /**
   * Intersect a circle with the grid to see which cells it touches
   * @param center  the center of the circle
   * @param radius  the radius of the circle
   * @return        the set of cells indices which intersect the circle
   */
  std::unordered_set<size_t> intersect(const coord_t& center, const float radius) const;

 protected:
  size_t divisions;
  AABB2<coord_t> super_cell;
  std::vector<AABB2<coord_t> > cells;
  x_t cell_width_recip;
  y_t cell_height_recip;
};

}
}

#endif  // VALHALLA_MIDGARD_GRID_H_
