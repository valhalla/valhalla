#include <stdexcept>

#include "baldr/location.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"

namespace valhalla {
namespace baldr {

// TODO: get defaults from config singleton
Location::Location(const midgard::PointLL& latlng,
                   const StopType& stoptype,
                   unsigned int min_outbound_reach,
                   unsigned int min_inbound_reach,
                   unsigned long radius,
                   const PreferredSide& side)
    : latlng_(latlng), stoptype_(stoptype), min_outbound_reach_(min_outbound_reach),
      min_inbound_reach_(min_inbound_reach), radius_(radius), preferred_side_(side),
      node_snap_tolerance_(5), heading_tolerance_(60), search_cutoff_(35000),
      street_side_tolerance_(5) {
}

bool Location::operator==(const Location& o) const {
  return latlng_ == o.latlng_ && stoptype_ == o.stoptype_ && name_ == o.name_ &&
         street_ == o.street_ && city_ == o.city_ && state_ == o.state_ && zip_ == o.zip_ &&
         country_ == o.country_ && date_time_ == o.date_time_ && heading_ == o.heading_ &&
         heading_tolerance_ == o.heading_tolerance_ &&
         node_snap_tolerance_ == o.node_snap_tolerance_ && way_id_ == o.way_id_ &&
         min_outbound_reach_ == o.min_outbound_reach_ && min_inbound_reach_ == o.min_inbound_reach_ &&
         radius_ == o.radius_ && preferred_side_ == o.preferred_side_;
}

} // namespace baldr
} // namespace valhalla
