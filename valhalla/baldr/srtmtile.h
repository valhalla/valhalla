#ifndef VALHALLA_BALDR_SRTMTILE_H
#define VALHALLA_BALDR_SRTMTILE_H

#include <string>
#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace baldr {

/**
 * Supports loading and retrieving values from a single SRTM 30m tile.
 * These are 1 degree, .hgt files that are a 2D array of heights (meters).
 * https://lpdaac.usgs.gov/products/measures_products_table/srtmgl1
 */
class SRTMTile {
 public:
  SRTMTile() = delete;

  /**
   * Constructor given the base directory and base lat,lng. Loads
   * a single SRTM tile into memory.
   * @param  dir     File directory where SRTM data resides
   * @param  baselat Base lat for the tile (lower left corner)
   * @param  baselng Base lng for the tile (lower left corner)
   */
  SRTMTile(const std::string& dir, const int32_t baselat,
           const int32_t baselng);

  /**
   * Destructor.
   */
  virtual ~SRTMTile();

  /**
   * Status function. Returns true if the tile loaded OK, false if not.
   * @return  Returns the status of the constructor.
   */
  bool loaded() const;

  /**
   * Get the base latitude of the tile
   * @return  Returns the base latitude
   */
  float baselat() const;

  /**
   * Get the base longitude of the tile
   * @return  Returns the base longitude
   */
  float baselng() const;

  /**
   * Gets the height at the specified lat,lng. If optional filtering is
   * specified, a box filter (weighted) using the 4 nearest height postings
   * is used.
   * @param   ll       Lat,lng
   * @param   filter   Filter between the 4 nearest height posts if true
   * @return  Returns the height in meters or kEmptyHeight if the
   *          data doesn't exist
   */
  float height(const midgard::PointLL& ll, const bool filter) const;

 protected:
  bool     loaded_;       // Is the tile loaded?
  float    baselat_;      // Base latitude of the tile
  float    baselng_;      // Base longitude of the tile
  int16_t* heights_;      // Height postings (meters)

  /**
   * Constructs the SRTM file name given the latitude, longitude of the
   * south west corner.
   * @param  basedir  Base directory of the SRTM data.
   * @param  lat      Base latitude (lower left) of the tile.
   * @param  lng      Base longitude (lower left) of the tile.
   * @return  Returns the filename including directory path.
   */
  std::string filename(const std::string& basedir, const int32_t lat,
                       const int32_t lng) const;
};

}
}

#endif // VALHALLA_BALDR_SRTMTILE_H
