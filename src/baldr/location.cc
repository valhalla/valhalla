#include "baldr/location.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"

namespace valhalla {
namespace baldr {

Location::SearchFilter::SearchFilter(valhalla::RoadClass min_road_class,
                                     valhalla::RoadClass max_road_class,
                                     bool exclude_tunnel,
                                     bool exclude_bridge,
                                     bool exclude_ramp,
                                     bool exclude_closures)
    : min_road_class_(min_road_class), max_road_class_(max_road_class),
      exclude_tunnel_(exclude_tunnel), exclude_bridge_(exclude_bridge), exclude_ramp_(exclude_ramp),
      exclude_closures_(exclude_closures) {
}

// TODO: get defaults from config singleton
Location::Location(const midgard::PointLL& latlng,
                   const StopType& stoptype,
                   unsigned int min_outbound_reach,
                   unsigned int min_inbound_reach,
                   unsigned long radius,
                   const PreferredSide& side,
                   valhalla::RoadClass street_side_cutoff,
                   const SearchFilter& search_filter,
                   std::optional<int8_t> preferred_layer)
    : latlng_(latlng), stoptype_(stoptype), min_outbound_reach_(min_outbound_reach),
      min_inbound_reach_(min_inbound_reach), radius_(radius), preferred_side_(side),
      node_snap_tolerance_(5), heading_tolerance_(60), search_cutoff_(35000),
      street_side_tolerance_(5), street_side_max_distance_(1000),
      street_side_cutoff_(street_side_cutoff), search_filter_(search_filter),
      preferred_layer_(std::move(preferred_layer)) {
}

bool Location::operator==(const Location& o) const {
  return latlng_ == o.latlng_ && stoptype_ == o.stoptype_ && name_ == o.name_ &&
         street_ == o.street_ && date_time_ == o.date_time_ && heading_ == o.heading_ &&
         heading_tolerance_ == o.heading_tolerance_ &&
         node_snap_tolerance_ == o.node_snap_tolerance_ &&
         street_side_tolerance_ == o.street_side_tolerance_ &&
         street_side_max_distance_ == o.street_side_max_distance_ &&
         street_side_cutoff_ == o.street_side_cutoff_ &&
         min_outbound_reach_ == o.min_outbound_reach_ && min_inbound_reach_ == o.min_inbound_reach_ &&
         radius_ == o.radius_ && preferred_side_ == o.preferred_side_ &&
         display_latlng_ == o.display_latlng_ && preferred_layer_ == o.preferred_layer_;
}

} // namespace baldr
} // namespace valhalla
