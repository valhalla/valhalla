#ifndef VALHALLA_THOR_LOOPTREE_H_
#define VALHALLA_THOR_LOOPTREE_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/proto/common.pb.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/thor/dijkstras.h>

#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

namespace valhalla {
namespace thor {

// The end of a directed edge where the next round trip leg can start
struct IntermediateCandidate {
  uint32_t label;
  // the edge in driving direction: it ends at the turning point in a forward expansion and leaves
  // it in a reverse expansion
  baldr::GraphId edge_id;
  midgard::PointLL ll;
  // distance to the end of the edge
  double distance;
  double bearing;
  baldr::IntersectionType intersection;
};

/**
 * An expansion from/to the start location to collect possible intermediate
 * points that the final loop will use.
 */
class LoopTree : public Dijkstras {
public:
  explicit LoopTree(const boost::property_tree::ptree& config = {}) : Dijkstras(config) {
  }

  void Clear() override {
    Dijkstras::Clear();
    candidates_.clear();
  }

  void Run(ExpansionType type,
           google::protobuf::RepeatedPtrField<Location>& locations,
           baldr::GraphReader& reader,
           const sif::mode_costing_t& costings,
           sif::travel_mode_t mode,
           double min_distance,
           double max_distance,
           double cone_center,
           double cone_half_width) {
    reverse_ = type == ExpansionType::reverse;
    use_hierarchy_limits_ = costings[static_cast<uint32_t>(mode)]->UseHierarchyLimits();
    start_ = midgard::PointLL(locations.Get(0).ll().lng(), locations.Get(0).ll().lat());
    min_distance_ = min_distance;
    max_distance_ = max_distance;
    cone_center_ = cone_center;
    cone_half_width_ = cone_half_width;
    motor_vehicle_ = mode == sif::travel_mode_t::kDrive;
    if (reverse_) {
      Compute<ExpansionType::reverse>(locations, reader, costings, mode);
    } else {
      Compute<ExpansionType::forward>(locations, reader, costings, mode);
    }
  }

  std::vector<IntermediateCandidate> Candidates() const {
    std::vector<IntermediateCandidate> candidates;
    candidates.reserve(candidates_.size());
    for (const auto& [cell, candidate] : candidates_) {
      candidates.push_back(candidate);
    }
    return candidates;
  }

  // the edges between the start and the candidate's edge, in driving order
  std::vector<baldr::GraphId> Path(uint32_t label) const {
    std::vector<baldr::GraphId> edges;
    for (auto l = label; l != baldr::kInvalidLabel; l = bdedgelabels_[l].predecessor()) {
      edges.push_back(TraversedEdge(l));
    }
    if (!reverse_) {
      std::reverse(edges.begin(), edges.end());
    }
    return edges;
  }

  // reverse labels hold the edge in expansion direction, the traversed one is its opposing edge
  baldr::GraphId TraversedEdge(uint32_t label) const {
    return reverse_ ? bdedgelabels_[label].opp_edgeid() : bdedgelabels_[label].edgeid();
  }

protected:
  // near the start the bearing of a node says little about where the path goes, so the expansion
  // is only kept inside its cone beyond this distance, and with some slack to bend around obstacles
  static constexpr double kConeStartDistance = 3000.;
  static constexpr double kConeMargin = 25.;
  // candidates are kept on a grid of distance and bearing, one per cell
  static constexpr int kDistanceBins = 10;
  static constexpr int kBearingBins = 4;

  void ExpandingNode(baldr::GraphReader&,
                     baldr::graph_tile_ptr,
                     const baldr::NodeInfo*,
                     const sif::EdgeLabel&,
                     const sif::EdgeLabel*) override {
  }

  void GetExpansionHints(uint32_t& bucket_count, uint32_t& edge_label_reservation) const override {
    bucket_count = 20000;
    edge_label_reservation = 200000;
  }

  ExpansionRecommendation
  ShouldExpand(baldr::GraphReader& reader, const sif::EdgeLabel& pred, const ExpansionType) override {
    const double distance = pred.path_distance();

    // stop if we're past the max distance
    if (distance > max_distance_) {
      return ExpansionRecommendation::prune_expansion;
    }

    baldr::graph_tile_ptr tile;
    const auto* node = reader.nodeinfo(pred.endnode(), tile);

    // stop if the node isn't available
    if (!node) {
      return ExpansionRecommendation::prune_expansion;
    }

    // get the bearing between the start location and the end node
    const midgard::PointLL ll = node->latlng(tile->header()->base_ll());
    const double bearing = start_.Heading(ll);
    const double off_center = AngleDiff(bearing, cone_center_);

    // stop if we're outside the cone and outside of the cone start distance
    if (distance > kConeStartDistance && off_center > cone_half_width_ + kConeMargin) {
      return ExpansionRecommendation::prune_expansion;
    }

    // prune deadends
    if (node->intersection() == baldr::IntersectionType::kDeadEnd) {
      return ExpansionRecommendation::prune_expansion;
    }

    // continue but don't mind this edge as a candidate
    if (distance < min_distance_ || off_center > cone_half_width_) {
      return ExpansionRecommendation::continue_expansion;
    }

    const uint32_t label = edgestatus_.Get(pred.edgeid(), pred.path_id()).index();
    const baldr::GraphId edge_id = TraversedEdge(label);
    baldr::graph_tile_ptr edge_tile;
    const auto* edge = reader.directededge(edge_id, edge_tile);

    if (!edge || !PossibleCandidate(edge)) {
      return ExpansionRecommendation::continue_expansion;
    }

    const int distance_bin =
        std::clamp(static_cast<int>((distance - min_distance_) / (max_distance_ - min_distance_) *
                                    kDistanceBins),
                   0, kDistanceBins - 1);
    const int bearing_bin =
        std::clamp(static_cast<int>((SignedAngle(cone_center_, bearing) + cone_half_width_) /
                                    (2. * cone_half_width_) * kBearingBins),
                   0, kBearingBins - 1);
    const IntermediateCandidate candidate{label,    edge_id, ll,
                                          distance, bearing, node->intersection()};
    // keep the first candidate per cell, unless only a later one is at a real intersection
    auto [cell, inserted] =
        candidates_.try_emplace(distance_bin * kBearingBins + bearing_bin, candidate);
    if (!inserted && cell->second.intersection == baldr::IntersectionType::kFalse &&
        candidate.intersection != baldr::IntersectionType::kFalse) {
      cell->second = candidate;
    }
    return ExpansionRecommendation::continue_expansion;
  }

  // whether the edge is a proper road for a round trip to turn at
  bool PossibleCandidate(const baldr::DirectedEdge* edge) const {
    if (edge->is_shortcut() || edge->link() || edge->internal() || edge->roundabout() ||
        edge->deadend() || edge->not_thru()) {
      return false;
    }
    switch (edge->use()) {
      case baldr::Use::kRamp:
      case baldr::Use::kTurnChannel:
      case baldr::Use::kDriveway:
      case baldr::Use::kAlley:
      case baldr::Use::kParkingAisle:
      case baldr::Use::kEmergencyAccess:
      case baldr::Use::kDriveThru:
      case baldr::Use::kCuldesac:
      case baldr::Use::kServiceRoad:
      case baldr::Use::kRestArea:
      case baldr::Use::kServiceArea:
      case baldr::Use::kSteps:
      case baldr::Use::kElevator:
      case baldr::Use::kEscalator:
      case baldr::Use::kLadder:
      case baldr::Use::kPlatform:
      case baldr::Use::kFerry:
      case baldr::Use::kRailFerry:
      case baldr::Use::kConstruction:
        return false;
      default:
        break;
    }
    // away from the start the hierarchy limits keep motor vehicles on higher road classes anyway
    return !motor_vehicle_ || edge->classification() <= baldr::RoadClass::kResidential;
  }

  static double AngleDiff(double a, double b) {
    const double d = std::fmod(std::abs(a - b), 360.);
    return d > 180. ? 360. - d : d;
  }

  // angle from `from` to `to` in (-180, 180], positive is clockwise
  static double SignedAngle(double from, double to) {
    return std::fmod(to - from + 540., 360.) - 180.;
  }

  midgard::PointLL start_;
  double min_distance_ = 0.;
  double max_distance_ = 0.;
  double cone_center_ = 0.;
  double cone_half_width_ = 0.;
  bool motor_vehicle_ = false;
  bool reverse_ = false;
  std::map<int, IntermediateCandidate> candidates_;
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_LOOPTREE_H_
