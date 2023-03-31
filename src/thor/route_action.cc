#include "thor/worker.h"
#include <cstdint>

#include "baldr/attributes_controller.h"
#include "baldr/json.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"

#include "proto/common.pb.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {
// Threshold for running a second pass pedestrian route with adjusted A*. The first
// pass for pedestrian routes is run with an aggressive A* threshold based on walking
// speed. If ferries are included in the path the A* heuristic rules can be violated
// which can lead to irregular paths. Running a second pass with less aggressive
// A* can take excessive time for longer paths - so exclude them to protect the service.
constexpr float kPedestrianMultipassThreshold = 50000.0f; // 50km

/**
 * Check if the paths meet at opposing edges (but not at a node). If so, add an intermediate location
 * so that the shape / distance along the path is adjusted at the location.
 */
bool intermediate_loc_edge_trimming(
    valhalla::Location& loc,
    const GraphId& in,
    const GraphId& out,
    std::unordered_map<size_t, std::pair<EdgeTrimmingInfo, EdgeTrimmingInfo>>& edge_trimming,
    const size_t path_index,
    const bool arrive_by) {
  // Find the path edges within the locations.
  auto in_pe = std::find_if(loc.correlation().edges().begin(), loc.correlation().edges().end(),
                            [&in](const valhalla::PathEdge& e) { return e.graph_id() == in; });
  auto out_pe = std::find_if(loc.correlation().edges().begin(), loc.correlation().edges().end(),
                             [&out](const valhalla::PathEdge& e) { return e.graph_id() == out; });

  // Could not find the edges. This seems like it should not happen. Log a warning
  // and do not insert an intermediate location.
  if (in_pe == loc.correlation().edges().end() || out_pe == loc.correlation().edges().end()) {
    LOG_WARN("Could not find connecting edges within the intermediate loc edge trimming");
    return false;
  }

  // Just set the edge index on the location for now then in triplegbuilder set to the shape index
  loc.mutable_correlation()->set_leg_shape_index(path_index + (arrive_by ? 1 : 0));
  // If the intermediate point is at a node we dont need to trim the edge we just set the edge index
  if (in_pe->begin_node() || in_pe->end_node() || out_pe->begin_node() || out_pe->end_node()) {
    // In this case we won't add a duplicate edge so we cant increment for arrive by
    loc.mutable_correlation()->set_leg_shape_index(path_index);
    return true;
  }

  // So what we are doing below is we are telling tripleg builder how to trim the two edges that come
  // together at an intermediate location. There are two cases, one where the route keeps going on the
  // same edge and one where the route does a uturn onto to opposing edge. In both cases we get two
  // edges in the final route just for simplicities sake. We could technically only do two when its
  // the uturn case. The way we do this is we specify a way to trim the shape of each edge. We use the
  // lat lon of the intermediate location to fix the geometry and we use the distance along to tell
  // trip leg builder how to cut the shape. We have to have at least one cut on each edge on either
  // side of the intermediate location. If we have multiple intermediate locations on a single edge we
  // will need to cut the edge based on the previous intermediate location we processed.

  PointLL snap_ll(in_pe->ll().lng(), in_pe->ll().lat());
  double dist_along = in_pe->percent_along();

  // Cut the first edges end off back to where the location lands along it
  auto inserted = edge_trimming.insert(
      {path_index + (arrive_by ? 1 : 0), {{false, PointLL(), 0.0}, {true, snap_ll, dist_along}}});
  // If it was already there we need to update it, should only happen for depart at (left to right)
  if (!inserted.second) {
    inserted.first->second.second = EdgeTrimmingInfo{true, snap_ll, dist_along};
  }

  // We need to use the distance along from the edge exiting the intermediate location to handle the
  // case when its a uturn at a via using the opposing edge that came into the via. Basically we need
  // to invert the distance along because we inverted the direction we are traveling along the edge.
  // This is handled automatically by using the correct exiting edge
  dist_along = out_pe->percent_along();

  // Cut the second edges beginning off up to where the location lands along it
  inserted = edge_trimming.insert(
      {path_index + (arrive_by ? 0 : 1), {{true, snap_ll, dist_along}, {false, PointLL(), 1.0}}});
  // If it was already there we need to update it, should only happen for arrive by (right to left)
  if (!inserted.second) {
    inserted.first->second.first = EdgeTrimmingInfo{true, snap_ll, dist_along};
  }

  return false;
}

inline bool is_through_point(const valhalla::Location& l) {
  return l.type() == valhalla::Location::kThrough || l.type() == valhalla::Location::kBreakThrough;
}

inline bool is_break_point(const valhalla::Location& l) {
  return l.type() == valhalla::Location::kBreak || l.type() == valhalla::Location::kBreakThrough;
}

inline bool is_highly_reachable(const valhalla::Location& loc, const valhalla::PathEdge& edge) {
  return edge.inbound_reach() >= loc.minimum_reachability() &&
         edge.outbound_reach() >= loc.minimum_reachability();
}

template <typename Predicate> inline void remove_path_edges(valhalla::Location& loc, Predicate pred) {
  auto new_end = std::remove_if(loc.mutable_correlation()->mutable_edges()->begin(),
                                loc.mutable_correlation()->mutable_edges()->end(), pred);
  int start_idx = std::distance(loc.mutable_correlation()->mutable_edges()->begin(), new_end);
  loc.mutable_correlation()->mutable_edges()->DeleteSubrange(start_idx,
                                                             loc.correlation().edges_size() -
                                                                 start_idx);

  new_end = std::remove_if(loc.mutable_correlation()->mutable_filtered_edges()->begin(),
                           loc.mutable_correlation()->mutable_filtered_edges()->end(), pred);
  start_idx = std::distance(loc.mutable_correlation()->mutable_filtered_edges()->begin(), new_end);
  loc.mutable_correlation()
      ->mutable_filtered_edges()
      ->DeleteSubrange(start_idx, loc.correlation().filtered_edges_size() - start_idx);
}

/**
// removes any edges from the location that aren't connected to it (because of radius)
void remove_edges(const GraphId& edge_id, valhalla::Location& loc, GraphReader& reader) {
  // find the path edge at this point
  auto pe =
      std::find_if(loc.correlation().edges().begin(), loc.correlation().edges().end(),
                   [&edge_id](const valhalla::PathEdge& e) { return e.graph_id() == edge_id;
});
  // if its in the middle of the edge it can only be this edge or the opposing depending on type
  if (!pe->begin_node() && !pe->end_node()) {
    GraphId opposing;
    if (loc.type() == valhalla::Location::kBreak || loc.type() == valhalla::Location::kVia)
      opposing = reader.GetOpposingEdgeId(edge_id);
    // remove anything that isnt one of these two edges
    for (int i = 0; i < loc.path_edges_size(); ++i) {
      if (loc.correlation().edges(i).graph_id() != edge_id && loc.correlation().edges(i).graph_id() !=
opposing) { loc.mutable_correlation()->mutable_edges()->SwapElements(i, loc.path_edges_size() - 1);
        loc.mutable_correlation()->mutable_edges()->RemoveLast();
      }
    }
    return;
  }

  // if its at the begin node lets center our sights on that
 graph_tile_ptr tile = reader.GetGraphTile(edge_id);
  const auto* edge = tile->directededge(edge_id);
  const auto* node = reader.GetEndNode(edge, tile);
  if (pe->begin_node()) {
    const auto* opp_edge = tile->directededge(node->edge_index() + edge->opp_index());
    node = reader.GetEndNode(opp_edge, tile);
  }

  // TODO: nuke any edges that aren't connected to this node
  GraphId start_edge = tile->header()->graphid();
  start_edge.set_id(node->edge_index);
  GraphId end_edge = start_edge + node->edge_count;
  for (int i = 0; i < loc.path_edges_size(); ++i) {
    if (loc.correlation().edges(i).graph_id() < start_edge || loc.correlation().edges(i) >= end_edge)
{ loc.mutable_correlation()->mutable_edges()->SwapElements(i, loc.path_edges_size() - 1);
      loc.mutable_correlation()->mutable_edges()->RemoveLast();
    }
  }
}*/

} // namespace

namespace valhalla {
namespace thor {

void thor_worker_t::centroid(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  auto& options = *request.mutable_options();
  adjust_scores(options);
  controller = AttributesController(request.options());
  auto costing = parse_costing(request);
  auto& locations = *options.mutable_locations();
  valhalla::Location destination;

  // get all the routes
  auto paths =
      centroid_gen.Expand(ExpansionType::forward, request, *reader, mode_costing, mode, destination);

  // serialize path information of each route into protobuf route objects
  auto origin = locations.begin();
  for (const auto& path : paths) {
    // the centroid could be either direction of the edge so here we set which it was by id
    auto dest = destination;
    dest.mutable_correlation()->mutable_edges(0)->set_graph_id(path.back().edgeid);

    // actually build the route object
    auto* route = request.mutable_trip()->mutable_routes()->Add();
    auto& leg = *route->mutable_legs()->Add();
    thor::TripLegBuilder::Build(options, controller, *reader, mode_costing, path.begin(), path.end(),
                                *origin, dest, leg, {"centroid"}, interrupt);

    // TODO: set the time at the destination if time dependent

    // next route
    ++origin;
  }
}

void thor_worker_t::route(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  auto& options = *request.mutable_options();
  adjust_scores(options);
  controller = AttributesController(options);
  auto costing = parse_costing(request);

  // get all the legs
  if (options.date_time_type() == Options::arrive_by) {
    path_arrive_by(request, costing);
  } else {
    path_depart_at(request, costing);
  }
}

thor::PathAlgorithm* thor_worker_t::get_path_algorithm(const std::string& routetype,
                                                       const valhalla::Location& origin,
                                                       const valhalla::Location& destination,
                                                       const Options& options) {
  // make sure they are all cancelable
  for (auto* alg : std::vector<PathAlgorithm*>{
           &multi_modal_astar,
           &timedep_forward,
           &timedep_reverse,
           &bidir_astar,
           &bss_astar,
       }) {
    alg->set_interrupt(interrupt);
  }

  // Have to use multimodal for transit based routing
  if (routetype == "multimodal" || routetype == "transit") {
    return &multi_modal_astar;
  }

  // Have to use bike share station algorithm
  if (routetype == "bikeshare") {
    return &bss_astar;
  }

  // If the origin has date_time set use timedep_forward method if the distance
  // between location is below some maximum distance (TBD).
  if (!origin.date_time().empty() && options.date_time_type() != Options::invariant &&
      !options.prioritize_bidirectional()) {
    PointLL ll1(origin.ll().lng(), origin.ll().lat());
    PointLL ll2(destination.ll().lng(), destination.ll().lat());
    if (ll1.Distance(ll2) < max_timedep_distance) {
      return &timedep_forward;
    }
  }

  // If the destination has date_time set use timedep_reverse method if the distance
  // between location is below some maximum distance (TBD).
  if (!destination.date_time().empty() && options.date_time_type() != Options::invariant) {
    PointLL ll1(origin.ll().lng(), origin.ll().lat());
    PointLL ll2(destination.ll().lng(), destination.ll().lat());
    if (ll1.Distance(ll2) < max_timedep_distance) {
      return &timedep_reverse;
    }
  }

  // Use A* if any origin and destination edges are the same or are connected - otherwise
  // use bidirectional A*. Bidirectional A* does not handle trivial cases with oneways and
  // has issues when cost of origin or destination edge is high (needs a high threshold to
  // find the proper connection).
  for (auto& edge1 : origin.correlation().edges()) {
    for (auto& edge2 : destination.correlation().edges()) {
      bool same_graph_id = edge1.graph_id() == edge2.graph_id();
      bool are_connected =
          reader->AreEdgesConnected(GraphId(edge1.graph_id()), GraphId(edge2.graph_id()));
      if (same_graph_id || are_connected) {
        return &timedep_forward;
      }
    }
  }

  // No other special cases we land on bidirectional a*
  return &bidir_astar;
}

std::vector<std::vector<thor::PathInfo>> thor_worker_t::get_path(PathAlgorithm* path_algorithm,
                                                                 valhalla::Location& origin,
                                                                 valhalla::Location& destination,
                                                                 const std::string& costing,
                                                                 const Options& options) {
  // Find the path.
  valhalla::sif::cost_ptr_t cost = mode_costing[static_cast<uint32_t>(mode)];

  // If bidirectional A* disable use of destination-only edges on the
  // first pass. If there is a failure, we allow them on the second pass.
  // Other path algorithms can use destination-only edges on the first pass.
  cost->set_allow_destination_only(path_algorithm == &bidir_astar ? false : true);

  cost->set_pass(0);
  auto paths = path_algorithm->GetBestPath(origin, destination, *reader, mode_costing, mode, options);

  // Check if we should run a second pass pedestrian route with different A*
  // (to look for better routes where a ferry is taken)
  bool ped_second_pass = false;
  if (!paths.empty() && (costing == "pedestrian" && path_algorithm->has_ferry())) {
    // DO NOT run a second pass on long routes due to performance issues
    float d = PointLL(origin.ll().lng(), origin.ll().lat())
                  .Distance(PointLL(destination.ll().lng(), destination.ll().lat()));
    if (d < kPedestrianMultipassThreshold) {
      ped_second_pass = true;
    }
  }

  // If path is not found try again with relaxed limits (if allowed). Use less aggressive
  // hierarchy transition limits, and retry with more candidate edges (add those filtered
  // by heading on first pass).
  if ((paths.empty() || ped_second_pass) && cost->AllowMultiPass()) {
    // add filtered edges to candidate edges for origin and destination
    origin.mutable_correlation()->mutable_edges()->MergeFrom(origin.correlation().filtered_edges());
    destination.mutable_correlation()->mutable_edges()->MergeFrom(
        destination.correlation().filtered_edges());

    path_algorithm->Clear();
    cost->set_pass(1);
    const bool using_bd = path_algorithm == &bidir_astar;
    cost->RelaxHierarchyLimits(using_bd);
    cost->set_allow_destination_only(true);
    cost->set_allow_conditional_destination(true);
    path_algorithm->set_not_thru_pruning(false);
    // Get the best path. Return if not empty (else return the original path)
    auto relaxed_paths =
        path_algorithm->GetBestPath(origin, destination, *reader, mode_costing, mode, options);
    if (!relaxed_paths.empty()) {
      return relaxed_paths;
    }
  }

  return paths;
}

void thor_worker_t::path_arrive_by(Api& api, const std::string& costing) {
  // Things we'll need
  TripRoute* route = nullptr;
  GraphId first_edge;
  std::unordered_map<size_t, std::pair<EdgeTrimmingInfo, EdgeTrimmingInfo>> edge_trimming;
  std::vector<thor::PathInfo> path;
  std::vector<std::string> algorithms;
  const Options& options = api.options();
  valhalla::Trip& trip = *api.mutable_trip();
  trip.mutable_routes()->Reserve(options.alternates() + 1);

  auto route_two_locations = [&](auto& origin, auto& destination) -> bool {
    // Get the algorithm type for this location pair
    thor::PathAlgorithm* path_algorithm =
        this->get_path_algorithm(costing, *origin, *destination, options);
    path_algorithm->Clear();
    algorithms.push_back(path_algorithm->name());
    LOG_INFO(std::string("algorithm::") + path_algorithm->name());

    // If we are continuing through a location we need to make sure we
    // only allow the edge that was used previously (avoid u-turns)
    if (is_through_point(*destination) && first_edge.Is_Valid()) {
      remove_path_edges(*destination,
                        [&first_edge](const auto& edge) { return edge.graph_id() != first_edge; });
    }

    // Get best path and keep it
    auto temp_paths = this->get_path(path_algorithm, *origin, *destination, costing, options);
    if (temp_paths.empty())
      return false;

    for (auto& temp_path : temp_paths) {
      // back propagate time information
      if (!destination->date_time().empty() &&
          options.date_time_type() != valhalla::Options::invariant) {
        auto origin_dt = offset_date(*reader, destination->date_time(), temp_path.back().edgeid,
                                     -temp_path.back().elapsed_cost.secs, temp_path.front().edgeid);
        origin->set_date_time(origin_dt);
      }

      // add waiting_secs again from the final destination's datetime, so we output the departing time
      // at intermediate locations, not the arrival time
      if (destination->waiting_secs() && !destination->date_time().empty()) {
        auto dest_dt = offset_date(*reader, destination->date_time(), temp_path.back().edgeid,
                                   destination->waiting_secs(), temp_path.back().edgeid);
        destination->set_date_time(dest_dt);
      }

      first_edge = temp_path.front().edgeid;
      temp_path.swap(path); // so we can append to path instead of prepend

      // Merge through legs by updating the time and splicing the lists
      if (!temp_path.empty()) {
        auto offset = path.back().elapsed_cost;
        auto distance_offset = path.back().path_distance;
        // NOTE: This will not work for all algorithms.  At this point, path_distance
        // is not correct across the board so do not rely on it downstream.
        std::for_each(temp_path.begin(), temp_path.end(), [offset, distance_offset](PathInfo& i) {
          i.elapsed_cost += offset;
          i.path_distance += distance_offset;
        });

        // When stitching routes at an intermediate location we need to store information about where
        // along the edge it happened so triplegbuilder can properly cut the shape where the location
        // was and store that info to be serialized int he output
        auto at_node =
            intermediate_loc_edge_trimming(*destination, path.back().edgeid, temp_path.front().edgeid,
                                           edge_trimming, temp_path.size(), true);

        // Connects via the same edge so we only need it once
        if (path.back().edgeid == temp_path.front().edgeid && at_node) {
          path.pop_back();
        }

        path.insert(path.end(), temp_path.begin(), temp_path.end());
      }

      // Build trip path for this leg and add to the result if this
      // location is a BREAK or if this is the last location
      if (is_break_point(*origin)) {
        // Move destination back to the last break
        std::vector<valhalla::Location> intermediates;
        while (!is_break_point(*destination)) {
          destination->mutable_correlation()->set_leg_shape_index(
              path.size() - destination->correlation().leg_shape_index());
          intermediates.push_back(*destination);
          --destination;
        }

        // We have to flip the intermediate loc indices because we built them in backwards order
        std::remove_reference<decltype(edge_trimming)>::type flipped;
        flipped.reserve(edge_trimming.size());
        for (const auto& kv : edge_trimming) {
          flipped.emplace(path.size() - kv.first, kv.second);
        }
        edge_trimming.swap(flipped);

        // Form output information based on path edges
        if (trip.routes_size() == 0 || options.alternates() > 0) {
          route = trip.mutable_routes()->Add();
          route->mutable_legs()->Reserve(options.locations_size());
        }
        auto& leg = *route->mutable_legs()->Add();
        TripLegBuilder::Build(options, controller, *reader, mode_costing, path.begin(), path.end(),
                              *origin, *destination, leg, algorithms, interrupt, edge_trimming,
                              intermediates);

        // advance the time for the next destination (i.e. algo origin) by the waiting_secs
        // of this origin (i.e. algo destination)
        if (origin->waiting_secs()) {
          auto origin_dt = offset_date(*reader, origin->date_time(), path.front().edgeid,
                                       -origin->waiting_secs(), path.front().edgeid);
          origin->set_date_time(origin_dt);
        }
        path.clear();
        edge_trimming.clear();
      }
    }

    // if we just made a leg that means we are done recording which algorithms were used
    if (path.empty())
      algorithms.clear();

    return true;
  };

  auto correlated = options.locations();
  bool allow_retry = true;

  // For each pair of locations
  auto origin = ++correlated.rbegin();
  while (origin != correlated.rend()) {
    auto destination = std::prev(origin);
    if (!route_two_locations(origin, destination)) {
      // if routing failed because an intermediate waypoint was snapped to the low reachability road
      // (such road lies in a small connectivity component that is not connected to other locations)
      // we should leave only high reachability candidates and try to route again
      if (allow_retry && destination != correlated.rbegin() && is_through_point(*destination) &&
          destination->correlation().edges_size() > 0 &&
          !is_highly_reachable(*destination, destination->correlation().edges(0))) {
        allow_retry = false;
        // for each intermediate waypoint remove candidates with low reachability
        correlated = options.locations();
        for (auto loc = std::next(correlated.begin()); loc != std::prev(correlated.end()); ++loc) {
          remove_path_edges(*loc,
                            [&loc](const auto& edge) { return !is_highly_reachable(*loc, edge); });
          // it doesn't make sense to continue if there are no more path edges
          if (loc->correlation().edges_size() == 0)
            // no route found
            throw valhalla_exception_t{442};
        }
        // resets the entire state of all the legs of the route and starts completely
        // over from the beginning doing all the legs over
        route = nullptr;
        first_edge = {};
        edge_trimming.clear();
        path.clear();
        algorithms.clear();
        trip.mutable_routes()->Clear();
        origin = ++correlated.rbegin();
        continue;
      }
      // no route found
      throw valhalla_exception_t{442};
    }
    ++origin;
  }
  // Reverse the legs because protobuf only has adding to the end
  std::reverse(route->mutable_legs()->begin(), route->mutable_legs()->end());
  // assign changed locations
  *api.mutable_options()->mutable_locations() = std::move(correlated);
}

void thor_worker_t::path_depart_at(Api& api, const std::string& costing) {
  // Things we'll need
  TripRoute* route = nullptr;
  GraphId last_edge;
  std::unordered_map<size_t, std::pair<EdgeTrimmingInfo, EdgeTrimmingInfo>> edge_trimming;
  std::vector<thor::PathInfo> path;
  std::vector<std::string> algorithms;
  const Options& options = api.options();
  valhalla::Trip& trip = *api.mutable_trip();
  trip.mutable_routes()->Reserve(options.alternates() + 1);

  auto route_two_locations = [&, this](auto& origin, auto& destination) -> bool {
    // Get the algorithm type for this location pair
    thor::PathAlgorithm* path_algorithm =
        this->get_path_algorithm(costing, *origin, *destination, options);
    path_algorithm->Clear();
    algorithms.push_back(path_algorithm->name());
    LOG_INFO(std::string("algorithm::") + path_algorithm->name());

    // If we are continuing through a location we need to make sure we
    // only allow the edge that was used previously (avoid u-turns)
    if (is_through_point(*origin) && last_edge.Is_Valid()) {
      remove_path_edges(*origin,
                        [&last_edge](const auto& edge) { return edge.graph_id() != last_edge; });
    }
    // Get best path and keep it
    auto temp_paths = this->get_path(path_algorithm, *origin, *destination, costing, options);
    if (temp_paths.empty())
      return false;

    for (auto& temp_path : temp_paths) {
      // forward propagate time information
      if (!origin->date_time().empty() && options.date_time_type() != valhalla::Options::invariant) {
        auto destination_dt =
            offset_date(*reader, origin->date_time(), temp_path.front().edgeid,
                        temp_path.back().elapsed_cost.secs + destination->waiting_secs(),
                        temp_path.back().edgeid);
        destination->set_date_time(destination_dt);
      }

      last_edge = temp_path.back().edgeid;

      // Merge through legs by updating the time and splicing the lists
      if (!path.empty()) {
        auto offset = path.back().elapsed_cost;
        auto distance_offset = path.back().path_distance;
        std::for_each(temp_path.begin(), temp_path.end(), [offset, distance_offset](PathInfo& i) {
          i.elapsed_cost += offset;
          i.path_distance += distance_offset;
        });

        // When stitching routes at an intermediate location we need to store information about where
        // along the edge it happened so triplegbuilder can properly cut the shape where the location
        // was and store that info to be serialized int he output
        auto at_node =
            intermediate_loc_edge_trimming(*origin, path.back().edgeid, temp_path.front().edgeid,
                                           edge_trimming, path.size() - 1, false);

        // Connects via the same edge so we only need it once
        if (path.back().edgeid == temp_path.front().edgeid && at_node) {
          path.pop_back();
        }

        path.insert(path.end(), temp_path.begin(), temp_path.end());
      } // Didnt need to merge
      else {
        path.swap(temp_path);
      }

      // Build trip path for this leg and add to the result if this
      // location is a BREAK or if this is the last location
      if (is_break_point(*destination)) {
        // Move origin back to the last break
        while (!is_break_point(*origin)) {
          --origin;
        }

        // Form output information based on path edges.
        if (trip.routes_size() == 0 || options.alternates() > 0) {
          route = trip.mutable_routes()->Add();
          route->mutable_legs()->Reserve(options.locations_size());
        }
        auto& leg = *route->mutable_legs()->Add();
        thor::TripLegBuilder::Build(options, controller, *reader, mode_costing, path.begin(),
                                    path.end(), *origin, *destination, leg, algorithms, interrupt,
                                    edge_trimming, {std::next(origin), destination});

        path.clear();
        edge_trimming.clear();
      }
    }

    // if we just made a leg that means we are done recording which algorithms were used
    if (path.empty())
      algorithms.clear();

    return true;
  };

  auto correlated = options.locations();
  bool allow_retry = true;

  // For each pair of locations
  auto destination = ++correlated.begin();
  while (destination != correlated.end()) {
    auto origin = std::prev(destination);
    if (!route_two_locations(origin, destination)) {
      // if routing failed because an intermediate waypoint was snapped to the low reachability road
      // (such road lies in a small connectivity component that is not connected to other locations)
      // we should leave only high reachability candidates and try to route again
      if (allow_retry && origin != correlated.begin() && is_through_point(*origin) &&
          origin->correlation().edges_size() > 0 &&
          !is_highly_reachable(*origin, origin->correlation().edges(0))) {
        allow_retry = false;
        // for each intermediate waypoint remove candidates with low reachability
        correlated = options.locations();
        for (auto loc = std::next(correlated.begin()); loc != std::prev(correlated.end()); ++loc) {
          remove_path_edges(*loc,
                            [&loc](const auto& edge) { return !is_highly_reachable(*loc, edge); });
          // it doesn't make sense to continue if there are no more path edges
          if (loc->correlation().edges_size() == 0)
            // no route found
            throw valhalla_exception_t{442};
        }
        // resets the entire state of all the legs of the route and starts completely
        // over from the beginning doing all the legs over
        route = nullptr;
        last_edge = {};
        edge_trimming.clear();
        path.clear();
        algorithms.clear();
        trip.mutable_routes()->Clear();
        destination = ++correlated.begin();
        continue;
      }
      // no route found
      throw valhalla_exception_t{442};
    }
    ++destination;
  }
  // assign changed locations
  *api.mutable_options()->mutable_locations() = std::move(correlated);
}

/**
 * Offset a time by some number of seconds, optionally taking into account timezones at the origin &
 * destination.
 *
 * @param reader   graphreader for tile/edge/node access
 * @param in_dt    the input date time string
 * @param in_edge  the input edgeid (used for timezone lookup)
 * @param offset   the offset in seconds from the input date time string
 * @param out_edge the output edgeid (used for timezone lookup)
 * @return out_dt  the time at the out_edge in local time after the offset is applied to the in_dt
 */
std::string thor_worker_t::offset_date(GraphReader& reader,
                                       const std::string& in_dt,
                                       const GraphId& in_edge,
                                       float offset,
                                       const GraphId& out_edge) {
  uint32_t in_tz = 0;
  uint32_t out_tz = 0;
  // get the timezone of the input location
  graph_tile_ptr tile = nullptr;
  auto in_nodes = reader.GetDirectedEdgeNodes(in_edge, tile);
  if (const auto* node = reader.nodeinfo(in_nodes.first, tile))
    in_tz = node->timezone();
  else if (const auto* node = reader.nodeinfo(in_nodes.second, tile))
    in_tz = node->timezone();

  // get the timezone of the output location
  auto out_nodes = reader.GetDirectedEdgeNodes(out_edge, tile);
  if (const auto* node = reader.nodeinfo(out_nodes.first, tile))
    out_tz = node->timezone();
  else if (const auto* node = reader.nodeinfo(out_nodes.second, tile))
    out_tz = node->timezone();

  // offset the time
  uint64_t in_epoch = DateTime::seconds_since_epoch(in_dt, DateTime::get_tz_db().from_index(in_tz));
  double out_epoch = static_cast<double>(in_epoch) + offset;
  auto out_dt = DateTime::seconds_to_date(static_cast<uint64_t>(out_epoch + .5),
                                          DateTime::get_tz_db().from_index(out_tz), false);
  return out_dt;
}
} // namespace thor
} // namespace valhalla
