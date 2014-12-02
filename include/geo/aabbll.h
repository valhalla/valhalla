#ifndef __aabbll_h__
#define __aabbll_h__

namespace valhalla{
namespace geo{

/**
 * Rectangular latitude, longitude region defined by its bounding box.
 * @author  David W. Nesbitt
 */
class AABBLL : public AABB2 {
 public:
  /**
   * Default constructor.  Sets the bounding box to be the entire earth.
   */
  AABBLL() {
    minx_ = 180.0f;      // Longitude
    miny_ = -90.0f;      // Latitude
    maxx_ = 180.0f;      // Longitude
    maxy_ = 90.0f;       // Latitude
  }

  /**
   * Constructor with specified bounds.
   * @param   minlat    Minimum latitude of the bounding box.
   * @param   minlng    Minimum longitude of the bounding box.
   * @param   maxlat    Maximum latitude of the bounding box.
   * @param   maxlng    Maximum longitude of the bounding box.
   */
  AABBLL(const float minlat, const float minlng,
         const float maxlat, const float maxlng) {
    minx_ = minlng;
    miny_ = minlat;
    maxx_ = maxlng;
    maxy_ = maxlat;
  }

  /**
   * Constructor given a list of lat,lng locations.
   * @param   pts   List of LatLng locations
   */
/**
TODO - figure this out (how to use base class?
   AABBLL(const std::vector<PointLL>& pts){
    Create(pts);
  } **/

  /**
   *  Destructor
   */
  ~AABBLL() { }

  /**
   * Get the minimum latitude of the bounding box.
   * @return  Minimum latitude of the bounding box.
   */
  float minlat() const {
    return miny_;
  }

  /**
  * Get the minimum longitude of the bounding box.
  * @return  Minimum longitude of the bounding box.
  */
  float minlng() const {
    return minx_;
  }

  /**
   * Get the maximum latitude of the bounding box.
   * @return  Maximum latitude of the bounding box.
   */
  float maxlat() const {
    return maxy_;
  }

  /**
   * Get the maximum longitude of the bounding box
   * @return  Maximum longitude of the bounding box.
   */
  float maxlng() const {
    return maxx_;
  }

  /**
   * Gets the upper left lat,lng of the bounding box
   * @return  Returns upper left lat,lng (maxlat, minlng)
   */
  PointLL GetUpperLeft() const {
  return PointLL(maxy_, minx_);
  }

  /**
   * Gets the upper right lat,lng of the bounding box
   * @return  Returns upper right lat,lng (maxlat, maxlng)
   */
  PointLL GetUpperRight() const {
    return PointLL(maxy_, maxx_);
  }

  /**
   * Gets the lower left lat,lng of the bounding box
   * @return  Returns lower left lat,lng (minlat, minlng)
   */
  PointLL GetLowerLeft() const {
    return PointLL(miny_, minx_);
  }

  /**
   * Gets the lower right lat,lng of the bounding box
   * @return  Returns lower right lat,lng (minlat, maxlng)
   */
  PointLL GetLowerRight() const {
    return PointLL(miny_, maxx_);
  }
};

}
}

#endif

