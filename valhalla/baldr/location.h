#ifndef VALHALLA_BALDR_LOCATION_H_
#define VALHALLA_BALDR_LOCATION_H_

#include <cstdint>
#include <string>

#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace baldr {

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
  enum class StopType : uint8_t { BREAK, THROUGH, VIA, BREAK_THROUGH };

  enum class PreferredSide : uint8_t { EITHER, SAME, OPPOSITE };

  /**
   * You have to initialize the location with something
   */
  Location() = delete;

  /**
   * Constructor.
   * @param  latlng  the polar coordinates of the location
   */
  Location(const midgard::PointLL& latlng,
           const StopType& stoptype = StopType::BREAK,
           unsigned int min_outbound_reach = 0,
           unsigned int min_inbound_reach = 0,
           unsigned long radius = 0,
           const PreferredSide& side = PreferredSide::EITHER);

  /**
   * equality.
   *
   */
  bool operator==(const Location& o) const;

  // coordinates of the location as used for searching the graph
  midgard::PointLL latlng_;
  // type of location for routing
  StopType stoptype_;

  // TODO: this will probably need refactored due to it being very US centered..
  // address of the location, probably should be its own more broken up structure
  std::string name_;
  std::string street_;
  std::string city_;
  std::string state_;
  std::string zip_;
  std::string country_;

  boost::optional<std::string> date_time_;
  boost::optional<float> heading_;
  boost::optional<uint64_t> way_id_;

  // try to find candidates who are reachable from/to this many or more nodes
  // if a given candidate edge is reachable to/from less than this number of nodes its considered to
  // be a disconnected island and we'll search for more candidates until we find at least one that
  // isnt considered a disconnected island
  unsigned int min_outbound_reach_;
  unsigned int min_inbound_reach_;
  // dont return results further away than this (meters) unless there is nothing this close
  unsigned long radius_;

  // which side of the street wrt your input location to leave/arrive from/at
  PreferredSide preferred_side_;
  float node_snap_tolerance_;
  float heading_tolerance_;
  float search_cutoff_;
  float street_side_tolerance_;

protected:
};

} // namespace baldr
} // namespace valhalla

namespace std {
template <> struct hash<valhalla::baldr::Location> {
  size_t operator()(const valhalla::baldr::Location& l) const {
    return std::hash<valhalla::midgard::PointLL>()(l.latlng_);
  }
};
} // namespace std

#endif // VALHALLA_BALDR_LOCATION_H_
