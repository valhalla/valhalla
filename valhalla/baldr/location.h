#ifndef VALHALLA_BALDR_LOCATION_H_
#define VALHALLA_BALDR_LOCATION_H_

#include <valhalla/midgard/pointll.h>
#include <valhalla/proto/common.pb.h>

namespace valhalla {
inline bool operator==(const LatLng& a, const LatLng& b) {
  return a.has_lat_case() == b.has_lat_case() && a.lat() == b.lat() &&
         a.has_lng_case() == b.has_lng_case() && a.lng() == b.lng();
}

inline bool operator==(const SearchFilter& a, const SearchFilter& b) {
  return a.has_min_road_class_case() == b.has_min_road_class_case() &&
         a.min_road_class() == b.min_road_class() &&
         a.has_max_road_class_case() == b.has_max_road_class_case() &&
         a.max_road_class() == b.max_road_class() && a.exclude_tunnel() == b.exclude_tunnel() &&
         a.exclude_bridge() == b.exclude_bridge() && a.exclude_ramp() == b.exclude_ramp() &&
         a.has_exclude_closures_case() == b.has_exclude_closures_case() &&
         a.exclude_closures() == b.exclude_closures() && a.exclude_toll() == b.exclude_toll() &&
         a.exclude_ferry() == b.exclude_ferry() && a.has_level_case() == b.has_level_case() &&
         a.level() == b.level();
}
inline bool operator==(const Location& a, const Location& b) {
  return a.ll() == b.ll() && a.type() == b.type() && a.has_heading_case() == b.has_heading_case() &&
         a.heading() == b.heading() && a.name() == b.name() && a.street() == b.street() &&
         a.date_time() == b.date_time() && a.side_of_street() == b.side_of_street() &&
         a.has_heading_tolerance_case() == b.has_heading_tolerance_case() &&
         a.heading_tolerance() == b.heading_tolerance() &&
         a.has_node_snap_tolerance_case() == b.has_node_snap_tolerance_case() &&
         a.node_snap_tolerance() == b.node_snap_tolerance() &&
         a.has_minimum_reachability_case() == b.has_minimum_reachability_case() &&
         a.minimum_reachability() == b.minimum_reachability() &&
         a.has_radius_case() == b.has_radius_case() && a.radius() == b.radius() &&
         a.has_accuracy_case() == b.has_accuracy_case() && a.accuracy() == b.accuracy() &&
         a.has_time_case() == b.has_time_case() && a.time() == b.time() &&
         a.skip_ranking_candidates() == b.skip_ranking_candidates() &&
         a.preferred_side() == b.preferred_side() && a.display_ll().lat() == b.display_ll().lat() &&
         a.display_ll() == b.display_ll() &&
         a.has_search_cutoff_case() == b.has_search_cutoff_case() &&
         a.search_cutoff() == b.search_cutoff() &&
         a.has_street_side_tolerance_case() == b.has_street_side_tolerance_case() &&
         a.street_side_tolerance() == b.street_side_tolerance() &&
         a.search_filter() == b.search_filter() &&
         a.has_street_side_max_distance_case() == b.has_street_side_max_distance_case() &&
         a.street_side_max_distance() == b.street_side_max_distance() &&
         a.has_preferred_layer_case() == b.has_preferred_layer_case() &&
         a.preferred_layer() == b.preferred_layer() && a.waiting_secs() == b.waiting_secs() &&
         a.has_street_side_cutoff_case() == b.has_street_side_cutoff_case() &&
         a.street_side_cutoff() == b.street_side_cutoff() &&
         a.has_minimum_inbound_reachability_case() == b.has_minimum_inbound_reachability_case() &&
         a.minimum_inbound_reachability() == b.minimum_inbound_reachability() &&
         a.has_minimum_outbound_reachability_case() == b.has_minimum_outbound_reachability_case() &&
         a.minimum_outbound_reachability() == b.minimum_outbound_reachability();
}

} // namespace valhalla
namespace std {

template <> struct hash<valhalla::Location> {
  size_t operator()(const valhalla::Location& location) const {
    return std::hash<valhalla::midgard::PointLL>()({location.ll().lng(), location.ll().lat()});
  }
};
} // namespace std

#endif // VALHALLA_BALDR_LOCATION_H_
