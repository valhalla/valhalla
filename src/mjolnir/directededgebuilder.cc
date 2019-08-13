#include "mjolnir/directededgebuilder.h"
#include "midgard/logging.h"

#include <algorithm>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

constexpr uint32_t kMinimumEdgeLength = 1;

// Constructor with parameters
DirectedEdgeBuilder::DirectedEdgeBuilder(const OSMWay& way,
                                         const GraphId& endnode,
                                         const bool forward,
                                         const uint32_t length,
                                         const uint32_t speed,
                                         const uint32_t truck_speed,
                                         const baldr::Use use,
                                         const RoadClass rc,
                                         const uint32_t localidx,
                                         const bool signal,
                                         const uint32_t restrictions,
                                         const uint32_t bike_network,
                                         const bool reclass_ferry)
    : DirectedEdge() {
  set_endnode(endnode);
  set_use(use);
  set_speed(speed);             // KPH
  set_truck_speed(truck_speed); // KPH

  // Protect against 0 length edges
  set_length(std::max(length, kMinimumEdgeLength));

  // Override use for ferries/rail ferries. TODO - set this in lua
  if (way.ferry()) {
    set_use(Use::kFerry);
  }
  if (way.rail()) {
    set_use(Use::kRailFerry);
  }
  set_toll(way.toll());

  // Set flag indicating this edge has a bike network
  if (bike_network) {
    set_bike_network(true);
  }

  set_truck_route(way.truck_route());

  // Set destination only to true if the reclass_ferry is set to false and either destination only or
  // no thru traffic is set. Adding the reclass_ferry check allows us to know if we should override
  // the destination only attribution
  set_dest_only(!reclass_ferry && (way.destination_only() || way.no_thru_traffic()));
  if (reclass_ferry && (way.destination_only() || way.no_thru_traffic()))
    LOG_DEBUG("Overriding dest_only attribution to false for ferry.");
  set_dismount(way.dismount());
  set_use_sidepath(way.use_sidepath());
  set_sac_scale(way.sac_scale());
  set_surface(way.surface());
  set_tunnel(way.tunnel());
  set_roundabout(way.roundabout());
  set_bridge(way.bridge());
  set_link(way.link());
  set_classification(rc);
  set_localedgeidx(localidx);
  set_restrictions(restrictions);
  set_traffic_signal(signal);

  set_sidewalk_left(way.sidewalk_left());
  set_sidewalk_right(way.sidewalk_right());

  set_speed_type(way.tagged_speed() ? SpeedType::kTagged : SpeedType::kClassified);

  // Set forward flag and access modes (based on direction)
  set_forward(forward);
  uint32_t forward_access = 0;
  uint32_t reverse_access = 0;
  if ((way.auto_forward() && forward) || (way.auto_backward() && !forward)) {
    forward_access |= kAutoAccess;
  }
  if ((way.auto_forward() && !forward) || (way.auto_backward() && forward)) {
    reverse_access |= kAutoAccess;
  }
  if ((way.truck_forward() && forward) || (way.truck_backward() && !forward)) {
    forward_access |= kTruckAccess;
  }
  if ((way.truck_forward() && !forward) || (way.truck_backward() && forward)) {
    reverse_access |= kTruckAccess;
  }
  if ((way.bus_forward() && forward) || (way.bus_backward() && !forward)) {
    forward_access |= kBusAccess;
  }
  if ((way.bus_forward() && !forward) || (way.bus_backward() && forward)) {
    reverse_access |= kBusAccess;
  }
  if ((way.bike_forward() && forward) || (way.bike_backward() && !forward)) {
    forward_access |= kBicycleAccess;
  }
  if ((way.bike_forward() && !forward) || (way.bike_backward() && forward)) {
    reverse_access |= kBicycleAccess;
  }
  if ((way.moped_forward() && forward) || (way.moped_backward() && !forward)) {
    forward_access |= kMopedAccess;
  }
  if ((way.moped_forward() && !forward) || (way.moped_backward() && forward)) {
    reverse_access |= kMopedAccess;
  }
  if ((way.motorcycle_forward() && forward) || (way.motorcycle_backward() && !forward)) {
    forward_access |= kMotorcycleAccess;
  }
  if ((way.motorcycle_forward() && !forward) || (way.motorcycle_backward() && forward)) {
    reverse_access |= kMotorcycleAccess;
  }
  if ((way.emergency_forward() && forward) || (way.emergency_backward() && !forward)) {
    forward_access |= kEmergencyAccess;
  }
  if ((way.emergency_forward() && !forward) || (way.emergency_backward() && forward)) {
    reverse_access |= kEmergencyAccess;
  }
  if ((way.hov_forward() && forward) || (way.hov_backward() && !forward)) {
    forward_access |= kHOVAccess;
  }
  if ((way.hov_forward() && !forward) || (way.hov_backward() && forward)) {
    reverse_access |= kHOVAccess;
  }
  if ((way.taxi_forward() && forward) || (way.taxi_backward() && !forward)) {
    forward_access |= kTaxiAccess;
  }
  if ((way.taxi_forward() && !forward) || (way.taxi_backward() && forward)) {
    reverse_access |= kTaxiAccess;
  }
  if (way.pedestrian()) {
    forward_access |= kPedestrianAccess;
    reverse_access |= kPedestrianAccess;
  }
  if (way.use() != Use::kSteps &&
      ((way.wheelchair_tag() && way.wheelchair()) || (!way.wheelchair_tag() && way.pedestrian()))) {
    forward_access |= kWheelchairAccess;
    reverse_access |= kWheelchairAccess;
  }

  // Set access modes
  set_forwardaccess(forward_access);
  set_reverseaccess(reverse_access);
}

} // namespace mjolnir
} // namespace valhalla
