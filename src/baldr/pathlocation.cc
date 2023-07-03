#include "baldr/pathlocation.h"
#include "midgard/util.h"

namespace valhalla {
namespace baldr {

PathLocation::PathEdge::PathEdge(const GraphId& id,
                                 const double percent_along,
                                 const midgard::PointLL& projected,
                                 const double score,
                                 const SideOfStreet sos,
                                 const unsigned int outbound_reach,
                                 const unsigned int inbound_reach,
                                 const float projected_heading)
    : id(id), percent_along(percent_along), projected(projected), sos(sos), distance(score),
      outbound_reach(outbound_reach), inbound_reach(inbound_reach),
      projected_heading(projected_heading) {
}
bool PathLocation::PathEdge::begin_node() const {
  return percent_along == 0.f;
}
bool PathLocation::PathEdge::end_node() const {
  return percent_along == 1.f;
}

PathLocation::PathLocation(const Location& location) : Location(location) {
  edges.reserve(16);
}

bool PathLocation::operator==(const PathLocation& other) const {
  // Check all of the scalar properties
  if (other.min_outbound_reach_ != min_outbound_reach_ ||
      other.min_inbound_reach_ != min_inbound_reach_ || other.radius_ != radius_ ||
      other.stoptype_ != stoptype_ || other.latlng_ != latlng_ || other.heading_ != heading_ ||
      other.heading_tolerance_ != heading_tolerance_ ||
      other.node_snap_tolerance_ != node_snap_tolerance_ || other.date_time_ != date_time_ ||
      other.name_ != name_ || other.street_ != street_ || other.edges.size() != edges.size()) {
    return false;
  }

  return shares_edges(other);
}

bool PathLocation::shares_edges(const PathLocation& other) const {
  // Check that the other PathLocation has all the edges we do
  for (const auto& edge : edges) {
    bool found = false;
    for (const auto& other_edge : other.edges) {
      if (edge.id == other_edge.id && edge.sos == other_edge.sos &&
          midgard::equal<double>(edge.percent_along, other_edge.percent_along) &&
          midgard::equal<double>(edge.distance, other_edge.distance, .1) &&
          edge.projected.ApproximatelyEqual(other_edge.projected)) {
        found = true;
        break;
      }
    }
    if (!found) {
      return false;
    }
  }
  return true;
}

} // namespace baldr
} // namespace valhalla
