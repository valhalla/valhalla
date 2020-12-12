#include "thor/centroid.h"

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

// constructor
PathIntersection::PathIntersection(uint64_t edge_id, uint64_t opp_id, uint8_t location_count)
    : edge_id_(edge_id), opp_id_(opp_id) {
  assert(location_count < 128);
  // the smaller edge goes first for determinisms sake
  if (opp_id_ < edge_id_)
    std::swap(edge_id_, opp_id_);
  // we pre-flip the bits of the paths we arent tracking so its as if they are already done
  // that way its easy to tell when we are done later on
  uint64_t low_bits = std::min(location_count, static_cast<uint8_t>(64));
  lower_mask_ = ~((1 << low_bits) - 1);
  if (location_count > 64) {
    uint64_t high_bits = static_cast<uint64_t>(location_count - 64);
    upper_mask_ = ~((1 << high_bits) - 1);
  }
}

// add an path has having connected at this intersection
bool PathIntersection::AddPath(uint8_t path_id) const {
  assert(path_id < 128);
  if (path_id < 64) {
    lower_mask_ |= 1 << static_cast<uint64_t>(path_id);
  } else {
    upper_mask_ |= 1 << static_cast<uint64_t>(path_id - 64);
  }
  // this will only be true once all the bits are flipped to true
  return lower_mask_ & upper_mask_;
}

// has this path converged on this intersection (edge pair)
bool PathIntersection::HasConverged(uint8_t path_id) const {
  assert(path_id < 128);
  if (path_id < 64) {
    return lower_mask_ & (1 >> static_cast<uint64_t>(path_id));
  } else {
    return upper_mask_ & (1 << static_cast<uint64_t>(path_id - 64));
  }
}

// we only care about the edge id because that is what we hash on
bool PathIntersection::operator==(const PathIntersection& i) const {
  return edge_id_ == i.edge_id_;
}

// main entry point to the functionality
std::vector<std::vector<PathInfo>>
Centroid::forward(google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                  baldr::GraphReader& reader,
                  const sif::mode_costing_t& costings,
                  const sif::TravelMode mode) {
  // preflight check
  if (locations.size() > baldr::kMaxMultiPathId)
    throw std::runtime_error("Max number of locations exceeded");

  // initialize state
  best_intersection_ = PathIntersection{baldr::kInvalidGraphId, baldr::kInvalidGraphId, 0};
  location_count_ = locations.size();
  // tell dijkstras we want to track the origin paths separately/concurrently
  multipath_ = true;
  // compute the expansion
  Dijkstras::Compute(locations, reader, costings, mode);
  // create the paths from the labelset
  return FormPaths(locations, bdedgelabels_);
}

// this is fired when the edge in the label has been settled (shortest path found) so we need to check
// our intersections and add or update them
thor::ExpansionRecommendation Centroid::ShouldExpand(baldr::GraphReader& reader,
                                                     const sif::EdgeLabel& label,
                                                     const thor::InfoRoutingType) {
  // TODO: we should quit earlier if finding a centroid isnt working out

  // TODO: refactor dijkstras a bit to get the tile and send it to us so we dont have to

  // grab the opposing edge if you can
  const baldr::GraphTile* tile = nullptr;
  baldr::GraphId opp_id;
  if (const auto* node = reader.nodeinfo(label.endnode(), tile)) {
    opp_id = tile->header()->graphid();
    opp_id.set_id(node->edge_index() + label.opp_index());
  }

  // see if we have seen this edge before
  PathIntersection intersection(label.edgeid(), opp_id, location_count_);
  auto found = intersections_.find(intersection);

  // if not we create the record
  if (found == intersections_.end()) {
    found = intersections_.insert(std::move(intersection)).first;
  }

  // update the record to include this path
  bool is_centroid = found->AddPath(label.path_id());

  // TODO: we should probably reject certain road classes as a centroid if desired, if you wanted to
  // actually drive these paths it doesnt make sense to meet other drivers on a limited access road

  // quit as soon as we find a centroid, otherwise always continue looking
  if (is_centroid) {
    best_intersection_ = *found;
    return thor::ExpansionRecommendation::stop_expansion;
  }
  return thor::ExpansionRecommendation::continue_expansion;
}

// tell the expansion how many labels to expect and how many buckets to use
void Centroid::GetExpansionHints(uint32_t& bucket_count, uint32_t& edge_label_reservation) const {
  // TODO: come up with a heuristic based on the expansion we expect to have to do (input locations)
  bucket_count = 20000;
  edge_label_reservation = 500000;
}

// deallocate and prepare for next request
void Centroid::Clear() {
  intersections_.clear();
  Dijkstras::Clear();
}

// walk edge labels to form paths for each location to the centroid
template <typename label_container_t>
std::vector<std::vector<PathInfo>>
Centroid::FormPaths(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                    const label_container_t& labels) const {
  // a place to store the results
  std::vector<std::vector<PathInfo>> paths;
  paths.reserve(locations.size());

  // for each path
  size_t path_reservation = 100;
  for (uint8_t path_id = 0; path_id < location_count_; ++path_id) {
    // skip this one if no path was found otherwise prepare to build the path
    path_reservation = std::max(path_reservation, paths.size() ? paths.back().size() : 0);
    paths.emplace_back();
    auto& path = paths.back();
    if (!best_intersection_.HasConverged(path_id))
      continue;
    path.reserve(path_reservation);

    // grab the edge statuses for both potential paths to two edges at the centroid
    auto status = edgestatus_.Get(baldr::GraphId(best_intersection_.edge_id_), path_id);
    auto opp_status = edgestatus_.Get(baldr::GraphId(best_intersection_.opp_id_), path_id);

    // check the edge status for both edges and find the label that was on the cheapest path
    // if the first status either wasnt settled (or even reached) or it was but it wasnt cheapest
    // then we switch to using the opposing label as its a better path
    auto label_index = status.index();
    if (status.set() != EdgeSet::kPermanent ||
        (opp_status.set() == EdgeSet::kPermanent &&
         labels[opp_status.index()].cost().cost < labels[status.index()].cost().cost)) {
      label_index = opp_status.index();
    }

    // recover the path from the centroid back to the origin edge
    for (auto l = label_index; l != baldr::kInvalidLabel; l = labels[l].predecessor()) {
      const auto& label = labels[l];
      path.emplace_back(label.mode(), label.cost(), label.edgeid(), 0, label.restriction_idx(),
                        label.transition_cost());
    }
  }

  // TODO: the final edge in each path could be a long one we should probably pick the optimal spot
  // along it to make all paths to it the most happy

  return paths;
}

template std::vector<std::vector<PathInfo>> Centroid::FormPaths<decltype(Dijkstras::bdedgelabels_)>(
    const google::protobuf::RepeatedPtrField<valhalla::Location>&,
    const decltype(Dijkstras::bdedgelabels_)&) const;

template std::vector<std::vector<PathInfo>> Centroid::FormPaths<decltype(Dijkstras::mmedgelabels_)>(
    const google::protobuf::RepeatedPtrField<valhalla::Location>&,
    const decltype(Dijkstras::mmedgelabels_)&) const;

} // namespace thor
} // namespace valhalla
