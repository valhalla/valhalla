#ifndef VALHALLA_MIDGARD_AABBLL_H_
#define VALHALLA_MIDGARD_AABBLL_H_

#include "midgard/aabb2.h"
#include "midgard/pointll.h"

namespace valhalla{
namespace midgard{

/**
 * Rectangular latitude, longitude region defined by its bounding box.
 * @author  David W. Nesbitt
 */
class AABBLL : public AABB2 {
 public:
  /**
   * Default constructor.  Sets the bounding box to be the entire earth.
   */
  AABBLL();

  /**
   * Constructor with specified bounds.
   * @param   minlat    Minimum latitude of the bounding box.
   * @param   minlng    Minimum longitude of the bounding box.
   * @param   maxlat    Maximum latitude of the bounding box.
   * @param   maxlng    Maximum longitude of the bounding box.
   */
  AABBLL(const float minlat, const float minlng,
         const float maxlat, const float maxlng);

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
  ~AABBLL();

  /**
   * Get the minimum latitude of the bounding box.
   * @return  Minimum latitude of the bounding box.
   */
  float minlat() const;

  /**
  * Get the minimum longitude of the bounding box.
  * @return  Minimum longitude of the bounding box.
  */
  float minlng() const;

  /**
   * Get the maximum latitude of the bounding box.
   * @return  Maximum latitude of the bounding box.
   */
  float maxlat() const;

  /**
   * Get the maximum longitude of the bounding box
   * @return  Maximum longitude of the bounding box.
   */
  float maxlng() const;

  /**
   * Gets the upper left lat,lng of the bounding box
   * @return  Returns upper left lat,lng (maxlat, minlng)
   */
  PointLL GetUpperLeft() const;

  /**
   * Gets the upper right lat,lng of the bounding box
   * @return  Returns upper right lat,lng (maxlat, maxlng)
   */
  PointLL GetUpperRight() const;

  /**
   * Gets the lower left lat,lng of the bounding box
   * @return  Returns lower left lat,lng (minlat, minlng)
   */
  PointLL GetLowerLeft() const;

  /**
   * Gets the lower right lat,lng of the bounding box
   * @return  Returns lower right lat,lng (minlat, maxlng)
   */
  PointLL GetLowerRight() const;
};

}
}

#endif  // VALHALLA_MIDGARD_AABBLL_H_

