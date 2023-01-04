#ifndef VALHALLA_BALDR_LOCATION_H_
#define VALHALLA_BALDR_LOCATION_H_

#include <cstdint>
#include <optional>
#include <string>

#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/proto/common.pb.h>

namespace valhalla {
namespace baldr {

/**
 * Input from the outside world to be used in determining where in the graph
 * the route needs to go. A start, middle, destination or via point
 * through which the route must pass
 *
 * TODO: deprecate this struct in favor of protobuf valhalla::Location.
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
   * Optional filters supplied in the request.
   *
   * NOTE: this struct must be kept in sync with the protobuf defined
   * valhalla::Location::SearchFilter in tripcommon.proto.
   */
  struct SearchFilter {
  public:
    SearchFilter(valhalla::RoadClass min_road_class = valhalla::RoadClass::kServiceOther,
                 valhalla::RoadClass max_road_class = valhalla::RoadClass::kMotorway,
                 bool exclude_tunnel = false,
                 bool exclude_bridge = false,
                 bool exclude_ramp = false,
                 bool exclude_closures = true);

    valhalla::RoadClass min_road_class_;
    valhalla::RoadClass max_road_class_;
    bool exclude_tunnel_;
    bool exclude_bridge_;
    bool exclude_ramp_;
    bool exclude_closures_;

  protected:
  };

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
           const PreferredSide& side = PreferredSide::EITHER,
           const SearchFilter& search_filter = SearchFilter(),
           std::optional<int8_t> preferred_layer = {});

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

  std::optional<std::string> date_time_;
  std::optional<float> heading_;

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
  float street_side_max_distance_;
  SearchFilter search_filter_;

  // coordinates of the location as used for altering the side of street
  std::optional<midgard::PointLL> display_latlng_;

  std::optional<int8_t> preferred_layer_;

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
