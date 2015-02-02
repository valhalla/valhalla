#ifndef VALHALLA_BALDR_LOCATION_H_
#define VALHALLA_BALDR_LOCATION_H_

#include <string>
#include <cstdint>

#include <valhalla/midgard/pointll.h>

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
   * You have to initialize the location with something
   */
  Location() = delete;

  /**
   * Constructor.
   * @param  latlng  the polar coordinates of the location
   */
  Location(const midgard::PointLL& latlng, const StopType& stoptype = StopType::BREAK);

  /**
   * conversion.
   * @param  geojson  a geojson representation of the location
   */
  static Location FromGeoJson(const std::string& geojson);

  /**
   * conversion.
   * @param  csv  a csv representation of the location
   */
  static Location FromCsv(const std::string& csv);


  //coordinates of the location as used for searching the graph
  midgard::PointLL latlng_;
  //type of location for routing
  StopType stoptype_;

  //TODO: this will probably need refactored due to it being very US centered..
  //address of the location, probably should be its own more broken up structure
  std::string name_;
  std::string street_;
  std::string city_;
  std::string state_;
  std::string zip_;
  std::string country_;

  //TODO: fill these out in constructors and add getters and setters
/*
  //the spot where the feature is on the map
  midgard::PointLL display_latlng_;
  //name of the location (useful for POIs)
  std::string name_;
  //id of the osm way that this location was on
  std::uint64_t wayid_;
*/

 protected:

};

}
}

#endif // VALHALLA_BALDR_LOCATION_H_
