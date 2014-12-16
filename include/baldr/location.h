#ifndef VALHALLA_BALDR_LOCATION_H_
#define VALHALLA_BALDR_LOCATION_H_

#include <string>
#include <cstdint>
#include "geo/pointll.h"

namespace valhalla{
namespace baldr{

/**
 * Input from the outside world to be used in determining where in the graph
 * the route needs to go. A start, middle, destination or via point
 * through which the route must pass
 *
 * @author  Kevin Kreiser
 */
struct Location {
 public:
  /**
   * What kind of location this, determines whether a route can double back or not
   * to find the most efficient path
   */
  enum class StopType : bool { BREAK, THROUGH };

  /**
   * Constructor.
   * @param  latlng  the polar coordinates of the location
   */
  Location(const geo::PointLL& latlng, const StopType& stoptype = StopType::BREAK);

  /**
   * Constructor.
   * @param  geojson  a geojson representation of the location
   */
  Location(const std::string geojson);

  //coordinates of the location as used for routing
  geo::PointLL latlng_;
  //type of location for routing
  StopType stoptype_;

  //TODO: fill these out in constructors and add getters and setters
/*
  //the spot where the feature is on the map
  geo::PointLL display_latlng_;
  //name of the location (useful for POIs)
  std::string name_;
  //address of the location, probably should be its own more broken up structure
  std::string address_;
  //id of the osm way that this location was on
  std::uint64_t wayid_;
*/

 private:
  /**
   * Default constructor
   */
  explicit Location();
};

}
}

#endif // VALHALLA_BALDR_LOCATION_H_
