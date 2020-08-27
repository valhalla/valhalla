#include <cmath>
#include <string>
#include <vector>

#include "baldr/graphconstants.h"
#include "baldr/openlr.h"
#include "midgard/encoded.h"
#include "midgard/pointll.h"

#include "proto/trip.pb.h"
#include "proto/tripcommon.pb.h"

namespace valhalla {
namespace midgard {

using FormOfWay = OpenLR::LocationReferencePoint::FormOfWay;

FormOfWay road_class_to_fow(const TripLeg::Node& node) {
  const bool is_missing_rc = !node.has_edge() || !node.edge().has_road_class();
  if (is_missing_rc || !node.edge().has_use()) {
    return FormOfWay::OTHER;
  }
  const TripLeg::Use use = node.edge().use();
  const RoadClass rc = node.edge().road_class();
  const bool one_way =
      node.edge().has_traversability() && node.edge().traversability() != TripLeg::kBoth;
  if (node.edge().roundabout()) {
    return FormOfWay::ROUNDABOUT;
  } else if (use == TripLeg::kRampUse || use == TripLeg::kTurnChannelUse) {
    return FormOfWay::SLIPROAD;
  } else if (rc == RoadClass::kMotorway && one_way) {
    return FormOfWay::MOTORWAY;
  } else if (rc <= RoadClass::kTertiary && one_way) {
    return FormOfWay::MULTIPLE_CARRIAGEWAY;
  } else if (rc <= RoadClass::kTertiary) {
    return FormOfWay::SINGLE_CARRIAGEWAY;
  } else {
    return FormOfWay::OTHER;
  }
}

std::vector<std::string> openlr_edges(const TripLeg& leg) {
  const std::vector<PointLL>& points = decode<std::vector<PointLL>>(leg.shape());
  std::vector<std::string> openlrs;
  openlrs.reserve(leg.node_size());
  for (const TripLeg::Node& node : leg.node()) {
    // OpenLR for an edge is composed of two points with the following semantics:
    // - FRC = relative importance of the road, so use the rank order of the FormOfWay enum.
    // - Bearing of last point in the segment must point inward (bearing rotated by 180deg),
    //   in-order to follow OpenLR conventions.
    const FormOfWay fow = road_class_to_fow(node);
    const unsigned char frc = static_cast<unsigned char>(fow);
    unsigned char lfrcnp = frc;
    const PointLL& start = points.at(node.edge().begin_shape_index());
    const PointLL& end = points.at(node.edge().end_shape_index());
    float bearing_deg = start.Heading(end);
    const float distance_meters = start.Distance(end);
    std::vector<OpenLR::LocationReferencePoint> lrps;
    lrps.emplace_back(start.lng(), start.lat(), bearing_deg, frc, fow, nullptr, distance_meters,
                      lfrcnp);
    bearing_deg = fmod(bearing_deg + 180., 360.);
    lrps.emplace_back(end.lng(), end.lat(), bearing_deg, frc, fow, &lrps.back());
    openlrs.emplace_back(OpenLR::LineLocation{lrps, 0, 0}.toBase64());
  }
  return openlrs;
}

std::vector<std::string> openlr_legs(const TripLeg& leg) {
  const std::vector<PointLL>& points = decode<std::vector<PointLL>>(leg.shape());
  float bearing_deg = 0;
  float distance_meters = 0;
  unsigned char lfrcnp = static_cast<unsigned char>(FormOfWay::OTHER);
  std::vector<OpenLR::LocationReferencePoint> lrps;
  for (std::size_t i = 0, num_nodes = leg.node_size(); i < num_nodes; ++i) {
    // See comments above for openlr_edges
    const TripLeg::Node& current = leg.node(i);
    const PointLL& location = points.at(current.edge().end_shape_index());
    const FormOfWay fow = road_class_to_fow(current);
    const unsigned char frc = static_cast<unsigned char>(fow);
    lfrcnp = std::min(lfrcnp, frc);
    if (i == num_nodes - 1) {
      bearing_deg = fmod(bearing_deg + 180., 360.);
      lrps.emplace_back(location.lng(), location.lat(), bearing_deg, frc, fow,
                        lrps.empty() ? nullptr : &lrps.back());
    } else {
      const PointLL& next_location = points.at(leg.node(i + 1).edge().end_shape_index());
      bearing_deg = location.Heading(next_location);
      distance_meters = location.Distance(next_location);
    }
    lrps.emplace_back(location.lng(), location.lat(), bearing_deg, frc, fow,
                      lrps.empty() ? nullptr : &lrps.back(), distance_meters, lfrcnp);
  }
  return {OpenLR::LineLocation{lrps, 0, 0}.toBase64()};
}

} // namespace midgard
} // namespace valhalla
