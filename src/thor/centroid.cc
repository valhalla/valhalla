#include "thor/centroid.h"

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {

/**
 * Constructs a path location as the mid point of an edge
 *
 * @param edge_id  the id of the edge from which to construct the location
 * @param reader   provides access to graph primitives
 * @return the constructed location
 */
valhalla::Location make_centroid(const valhalla::baldr::GraphId& edge_id,
                                 valhalla::baldr::GraphReader& reader) {
  using namespace valhalla;
  valhalla::baldr::graph_tile_ptr tile;
  const auto* edge = reader.directededge(baldr::GraphId(edge_id), tile);
  auto info = tile->edgeinfo(edge);
  const auto& shape = info.shape();
  auto length = midgard::length(shape);
  auto mid_point = midgard::resample_spherical_polyline(shape, length / 2.)[1];
  auto names = info.GetNames();

  valhalla::Location location;
  location.Clear();
  location.mutable_ll()->set_lng(mid_point.first);
  location.mutable_ll()->set_lat(mid_point.second);

  auto* path_edge = location.mutable_correlation()->mutable_edges()->Add();
  std::for_each(names.begin(), names.end(),
                [path_edge](const std::string& n) { path_edge->mutable_names()->Add()->assign(n); });
  path_edge->set_begin_node(false);
  path_edge->set_end_node(false);
  path_edge->set_distance(0);
  path_edge->set_percent_along(.5);
  path_edge->set_inbound_reach(0);
  path_edge->set_outbound_reach(0);
  path_edge->set_side_of_street(valhalla::Location::kNone);
  path_edge->mutable_ll()->CopyFrom(location.ll());
  return location;
}

} // namespace

namespace valhalla {
namespace thor {

// constructor
PathIntersection::PathIntersection(uint64_t edge_id, uint64_t opp_id, uint8_t location_count)
    : edge_id_(edge_id) {
  assert(location_count < 128);
  // the smaller edge goes first for determinisms sake
  if (opp_id < edge_id_)
    edge_id_ = opp_id;
  // we pre-flip the bits of the paths we arent tracking so its as if they are already done
  // that way its easy to tell when we are done later on
  if (location_count < 64) {
    lower_mask_ = ~((1ull << static_cast<uint64_t>(location_count)) - 1ull);
    upper_mask_ = 0xffffffffffffffff;
  } else {
    lower_mask_ = 0;
    upper_mask_ = ~((1ull << static_cast<uint64_t>(location_count - 64)) - 1ull);
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
  return (lower_mask_ & upper_mask_) == 0xffffffffffffffff;
}

// has this path converged on this intersection (edge pair)
bool PathIntersection::HasConverged(uint8_t path_id) const {
  assert(path_id < 128);
  if (path_id < 64) {
    return lower_mask_ & (1 << static_cast<uint64_t>(path_id));
  } else {
    return upper_mask_ & (1 << static_cast<uint64_t>(path_id - 64));
  }
}

// we only care about the edge id because that is what we hash on
bool PathIntersection::operator==(const PathIntersection& i) const {
  return edge_id_ == i.edge_id_;
}

// main entry point to the functionality
std::vector<std::vector<PathInfo>> Centroid::Expand(const ExpansionType& expansion_type,
                                                    valhalla::Api& api,
                                                    baldr::GraphReader& reader,
                                                    const sif::mode_costing_t& costings,
                                                    const sif::TravelMode mode,
                                                    valhalla::Location& centroid) {
  // preflight check
  if (api.options().locations_size() > baldr::kMaxMultiPathId)
    throw std::runtime_error("Max number of locations exceeded");

  // initialize state
  location_count_ = api.options().locations_size();
  best_intersection_ =
      PathIntersection{baldr::kInvalidGraphId, baldr::kInvalidGraphId, location_count_};

  // tell dijkstras we want to track the locations' paths separately/concurrently
  multipath_ = true;

  // compute the expansion
  Dijkstras::Expand(expansion_type, api, reader, costings, mode);

  // create the paths from the labelset
  return FormPaths(expansion_type, api.options().locations(), bdedgelabels_, reader, centroid);
}

// this is fired when the edge in the label has been settled (shortest path found) so we need to check
// our intersections and add or update them
thor::ExpansionRecommendation Centroid::ShouldExpand(baldr::GraphReader& reader,
                                                     const sif::EdgeLabel& label,
                                                     const thor::ExpansionType) {
  // TODO: we should quit earlier if finding a centroid isnt working out

  // TODO: refactor dijkstras a bit to get the tile and send it to us so we dont have to

  // grab the opposing edge if you can
  baldr::graph_tile_ptr tile;
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

  // TODO: prune these when they are outside of a reasonable bounding box, if that leads to failure
  // drop the bounding box and dont prune

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
  edge_label_reservation = kInitialEdgeLabelCountDijkstras;
}

// deallocate and prepare for next request
void Centroid::Clear() {
  intersections_.clear();
  Dijkstras::Clear();
}

// walk edge labels to form paths for each location to the centroid
template <typename label_container_t>
std::vector<std::vector<PathInfo>>
Centroid::FormPaths(const ExpansionType& expansion_type,
                    const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                    const label_container_t& labels,
                    baldr::GraphReader& reader,
                    valhalla::Location& centroid) const {
  // construct a centroid where all the paths meet
  auto edge_id = baldr::GraphId(best_intersection_.edge_id_);
  centroid = make_centroid(edge_id, reader);

  // keep the opposing edge in case its a better path for some locations
  graph_tile_ptr tile;
  auto opp_id = reader.GetOpposingEdgeId(edge_id, tile);

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
    auto status = edgestatus_.Get(edge_id, path_id);
    auto opp_status = edgestatus_.Get(opp_id, path_id);

    // check the edge status for both edges and find the label that was on the cheapest path
    // if the first status either wasnt settled (or even reached) or it was but it wasnt cheapest
    // then we switch to using the opposing label as its a better path
    auto label_index = status.index();
    if (status.set() != EdgeSet::kPermanent ||
        (opp_status.set() == EdgeSet::kPermanent &&
         labels[opp_status.index()].cost().cost < labels[status.index()].cost().cost)) {
      label_index = opp_status.index();
    }

    // recover the path from the centroid back to the locations edge candidate
    for (auto l = label_index; l != baldr::kInvalidLabel; l = labels[l].predecessor()) {
      const auto& label = labels[l];
      auto path_edge_id = expansion_type == ExpansionType::reverse
                              ? reader.GetOpposingEdgeId(label.edgeid(), tile)
                              : label.edgeid();
      path.emplace_back(label.mode(), label.cost(), path_edge_id, 0, label.path_distance(),
                        label.restriction_idx(), label.transition_cost());
    }

    // reverse the path since we recovered it starting at the beginning
    if (expansion_type != ExpansionType::reverse)
      std::reverse(path.begin(), path.end());

    // TODO: the final edge in each path could be a long one we should probably pick the optimal spot
    // along it to make all paths to it the most happy. for now we'll take the mid point
    auto edge_cost = path.back().elapsed_cost - path.back().transition_cost;
    path.back().elapsed_cost -= edge_cost * .5;
  }

  return paths;
}

template std::vector<std::vector<PathInfo>> Centroid::FormPaths<decltype(Dijkstras::bdedgelabels_)>(
    const ExpansionType&,
    const google::protobuf::RepeatedPtrField<valhalla::Location>&,
    const decltype(Dijkstras::bdedgelabels_)&,
    baldr::GraphReader&,
    valhalla::Location&) const;

template std::vector<std::vector<PathInfo>> Centroid::FormPaths<decltype(Dijkstras::mmedgelabels_)>(
    const ExpansionType&,
    const google::protobuf::RepeatedPtrField<valhalla::Location>&,
    const decltype(Dijkstras::mmedgelabels_)&,
    baldr::GraphReader&,
    valhalla::Location&) const;

} // namespace thor
} // namespace valhalla
