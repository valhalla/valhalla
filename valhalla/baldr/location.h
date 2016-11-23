#ifndef VALHALLA_BALDR_LOCATION_H_
#define VALHALLA_BALDR_LOCATION_H_

#include <string>
#include <cstdint>

#include <valhalla/midgard/pointll.h>

#include <boost/property_tree/ptree.hpp>

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
   * Serializes this object to ptree
   * @return ptree
   */
  boost::property_tree::ptree ToPtree() const;

  /**
   * conversion.
   * @param  pt  a property tree representation of the location
   */
  static Location FromPtree(const boost::property_tree::ptree& pt);

  /**
   * conversion.
   * @param  json  a json representation of the location
   */
  static Location FromJson(const std::string& json);

  /**
   * conversion.
   * @param  csv  a csv representation of the location
   */
  static Location FromCsv(const std::string& csv);

  /**
   * equality.
   *
   */
  bool operator==(const Location& o) const;


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

  boost::optional<std::string> date_time_;
  boost::optional<int> heading_;
  boost::optional<uint64_t> way_id_;

  //TODO: fill these out in constructors and add getters and setters
/*
  //the spot where the feature is on the map
  midgard::PointLL display_latlng_;
  //id of the osm way that this location was on
  std::uint64_t wayid_;
*/

 protected:

};

}
}

namespace std {
  template <> struct hash<valhalla::baldr::Location> {
    size_t operator()(const valhalla::baldr::Location& l) const {
      return std::hash<valhalla::midgard::PointLL>()(l.latlng_);
    }
  };
}

#endif // VALHALLA_BALDR_LOCATION_H_
