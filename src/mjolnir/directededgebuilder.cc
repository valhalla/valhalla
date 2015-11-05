#include "mjolnir/directededgebuilder.h"
#include <valhalla/midgard/logging.h>

#include <algorithm>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Constructor with parameters
DirectedEdgeBuilder::DirectedEdgeBuilder(
                   const OSMWay& way, const GraphId& endnode,
                   const bool forward, const uint32_t length,
                   const uint32_t speed, const uint32_t truck_speed,
                   const baldr::Use use, const RoadClass rc,
                   const uint32_t localidx, const bool signal,
                   const uint32_t restrictions, const uint32_t bike_network)
     :  DirectedEdge() {
  set_endnode(endnode);
  set_length(length);
  set_use(use);
  set_speed(speed);    // KPH
  set_truck_speed(truck_speed); // KPH
  set_ferry(way.ferry());
  set_railferry(way.rail());
  set_toll(way.toll());
  set_dest_only(way.destination_only());

  if (bike_network)
    set_bike_network(way.bike_network() | bike_network);
  else
    set_bike_network(way.bike_network());

  set_truck_route(way.truck_route());

  if (!way.destination_only())
    set_dest_only(way.no_thru_traffic());

  set_surface(way.surface());
  set_cyclelane(way.cyclelane());
  set_tunnel(way.tunnel());
  set_roundabout(way.roundabout());
  set_bridge(way.bridge());
  set_link(way.link());
  set_classification(rc);
  set_localedgeidx(localidx);
  set_restrictions(restrictions);
  set_traffic_signal(signal);

  set_speed_type(way.tagged_speed() ?
        SpeedType::kTagged : SpeedType::kClassified);

  // Set forward flag and access (based on direction)
  set_forward(forward);

  // Set access flags in the forward direction
  uint32_t forward_access = 0;
  if (way.auto_forward())
    forward_access |= kAutoAccess;
  if (way.truck_forward())
    forward_access |= kTruckAccess;
  if (way.bus_forward())
    forward_access |= kBusAccess;
  if (way.bike_forward())
    forward_access |= kBicycleAccess;
  if (way.emergency_forward())
    forward_access |= kEmergencyAccess;
  if (way.pedestrian())
    forward_access |= kPedestrianAccess;
  set_forwardaccess(forward_access);

  // TODO: HOV, Taxi

  // Access for opposite direction
  uint32_t reverse_access = 0;
  if (way.auto_backward())
    reverse_access |= kAutoAccess;
  if (way.truck_backward())
    reverse_access |= kTruckAccess;
  if (way.bus_backward())
    reverse_access |= kBusAccess;
  if (way.bike_backward())
    reverse_access |= kBicycleAccess;
  if (way.emergency_backward())
    reverse_access |= kEmergencyAccess;
  if (way.pedestrian())
    reverse_access |= kPedestrianAccess;
  set_reverseaccess(reverse_access);
}

}
}
