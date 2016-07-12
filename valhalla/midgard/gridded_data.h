#ifndef VALHALLA_MIDGARD_GRIDDEDDATA_H_
#define VALHALLA_MIDGARD_GRIDDEDDATA_H_

#include <valhalla/midgard/tiles.h>

namespace valhalla {
namespace midgard {

/**
 * Class to store data in a gridded/tiled data structure. Contains methods
 * to mark each tile with data using a compare operator.
 */
template <class coord_t>
class GriddedData : public Tiles<coord_t> {
 public:
  /**
   * Constructor.
   * @param   bounds    Bounding box
   * @param   tilesize  Tile size
   * @param   value     Value to initialize data with.
   */
  GriddedData(const AABB2<coord_t>& bounds, const float tilesize,
              const float value);

  /**
   * Set the value at a specified point. Verifies that the point is within the
   * tiles.
   * @param  pt     Coordinate to set within the tiles.
   * @param  value  Value to set at the tile/grid location.
   * @return whether or not the value was set
   */
  bool Set(const coord_t& pt, const float value);

  /**
   * Set the value at a specified point if the value is less than the current
   * value set at the grid location. Verifies that the point is within the
   * tiles.
   * @param  pt     Coordinate to set within the tiles.
   * @param  value  Value to set at the tile/grid location.
   * @return whether or not the value was set
   */
  bool SetIfLessThan(const coord_t& pt, const float value);

  /**
   * Get the array of data.
   * @return  Returns the data associated with the tiles.
   */
  const std::vector<float>& data() const;

  //TODO: a data structure to hold the contours
  //TODO: a function to turn said object into geojson

  /**
   * Generate contour lines from the gridded data.
   */
  void GenerateContourLines(const std::vector<float>& contours);

 protected:
  std::vector<float> data_;                  // Data value within each tile

  /**
   * Add a segment to a contour line.
   * @param  pt1  Segment start point
   * @param  pt2  Segment end point
   * @param  k    Contour level.
   */
  void AddToContourLine(const coord_t& pt1, const coord_t& pt2,
                        const int level);
};

}
}

#endif  // VALHALLA_MIDGARD_GRIDDEDDATA_H_
