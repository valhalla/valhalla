#include "midgard/encoded.h"
#include "proto_conversions.h"
#include "thor/triplegbuilder.h"

#include <ctime>
#include <random>

namespace valhalla {

// Adds incidents to a TripLeg_Edge
void thor::addIncidents(TripLeg_Edge& trip_edge,
                        baldr::GraphReader& graphreader,
                        const baldr::GraphId& edge_id) {
  // TODO
}

} // namespace valhalla
