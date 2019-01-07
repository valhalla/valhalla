#include "thor/worker.h"
#include <cstdint>

#include "baldr/json.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "thor/attributes_controller.h"

#include <valhalla/proto/tripcommon.pb.h>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace valhalla {
namespace thor {

// Threshold for running a second pass pedestrian route with adjusted A*. The first
// pass for pedestrian routes is run with an aggressive A* threshold based on walking
// speed. If ferries are included in the path the A* heuristic rules can be violated
// which can lead to irregular paths. Running a second pass with less aggressive
// A* can take excessive time for longer paths - so exclude them to protect the service.
constexpr float kPedestrianMultipassThreshold = 50000.0f; // 50km

std::list<valhalla::odin::TripPath> thor_worker_t::route(valhalla_request_t& request) {
  parse_locations(request);
  auto costing = parse_costing(request);

  auto trippaths = (request.options.has_date_time_type() &&
                    request.options.date_time_type() == odin::DirectionsOptions::arrive_by)
                       ? path_arrive_by(*request.options.mutable_locations(), costing)
                       : path_depart_at(*request.options.mutable_locations(), costing);

  if (!request.options.do_not_track()) {
    for (const auto& tp : trippaths) {
      log_admin(tp);
    }
  }
  return trippaths;
}

thor::PathAlgorithm* thor_worker_t::get_path_algorithm(const std::string& routetype,
                                                       const odin::Location& origin,
                                                       const odin::Location& destination) {
  if (routetype == "multimodal" || routetype == "transit") {
    multi_modal_astar.set_interrupt(interrupt);
    return &multi_modal_astar;
  }

  // If the origin has date_time set use timedep_forward method if the distance
  // between location is below some maximum distance (TBD).
  if (origin.has_date_time()) {
    PointLL ll1(origin.ll().lng(), origin.ll().lat());
    PointLL ll2(destination.ll().lng(), destination.ll().lat());
    if (ll1.Distance(ll2) < max_timedep_distance) {
      timedep_forward.set_interrupt(interrupt);
      return &timedep_forward;
    }
  }

  // If the destination has date_time set use timedep_reverse method if the distance
  // between location is below some maximum distance (TBD).
  if (destination.has_date_time()) {
    PointLL ll1(origin.ll().lng(), origin.ll().lat());
    PointLL ll2(destination.ll().lng(), destination.ll().lat());
    if (ll1.Distance(ll2) < max_timedep_distance) {
      timedep_reverse.set_interrupt(interrupt);
      return &timedep_reverse;
    }
  }

  // Use A* if any origin and destination edges are the same or are connected - otherwise
  // use bidirectional A*. Bidirectional A* does not handle trivial cases with oneways and
  // has issues when cost of origin or destination edge is high (needs a high threshold to
  // find the proper connection).
  for (auto& edge1 : origin.path_edges()) {
    for (auto& edge2 : destination.path_edges()) {
      if (edge1.graph_id() == edge2.graph_id() ||
          reader->AreEdgesConnected(GraphId(edge1.graph_id()), GraphId(edge2.graph_id()))) {
        astar.set_interrupt(interrupt);
        return &astar;
      }
    }
  }
  bidir_astar.set_interrupt(interrupt);
  return &bidir_astar;
}

std::vector<thor::PathInfo> thor_worker_t::get_path(PathAlgorithm* path_algorithm,
                                                    odin::Location& origin,
                                                    odin::Location& destination,
                                                    const std::string& costing) {
  // Find the path. If bidirectional A* disable use of destination only edges on the
  // first pass. If there is a failure, we allow them on the second pass.
  valhalla::sif::cost_ptr_t cost = mode_costing[static_cast<uint32_t>(mode)];
  if (path_algorithm == &bidir_astar) {
    cost->set_allow_destination_only(false);
  }
  cost->set_pass(0);
  auto path = path_algorithm->GetBestPath(origin, destination, *reader, mode_costing, mode);

  // Check if we should run a second pass pedestrian route with different A*
  // (to look for better routes where a ferry is taken)
  bool ped_second_pass = false;
  if (!path.empty() && (costing == "pedestrian" && path_algorithm->has_ferry())) {
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
  if ((path.empty() || ped_second_pass) && cost->AllowMultiPass()) {
    // add filtered edges to candidate edges for origin and destination
    origin.mutable_path_edges()->MergeFrom(origin.filtered_edges());
    destination.mutable_path_edges()->MergeFrom(destination.filtered_edges());

    path_algorithm->Clear();
    cost->set_pass(1);
    bool using_astar = (path_algorithm == &astar);
    float relax_factor = using_astar ? 16.0f : 8.0f;
    float expansion_within_factor = using_astar ? 4.0f : 2.0f;
    cost->RelaxHierarchyLimits(relax_factor, expansion_within_factor);
    cost->set_allow_destination_only(true);

    // Get the best path. Return if not empty (else return the original path)
    auto path2 = path_algorithm->GetBestPath(origin, destination, *reader, mode_costing, mode);
    if (!path2.empty()) {
      return path2;
    }
  }

  // All or nothing
  if (path.empty()) {
    throw valhalla_exception_t{442};
  }
  return path;
}

std::list<valhalla::odin::TripPath> thor_worker_t::path_arrive_by(
    google::protobuf::RepeatedPtrField<valhalla::odin::Location>& correlated,
    const std::string& costing) {
  // Things we'll need
  std::vector<thor::PathInfo> path;
  std::list<valhalla::odin::TripPath> trip_paths;
  correlated.begin()->set_type(odin::Location::kBreak);
  correlated.rbegin()->set_type(odin::Location::kBreak);

  // For each pair of locations
  for (auto origin = ++correlated.rbegin(); origin != correlated.rend(); ++origin) {
    // Get the algorithm type for this location pair
    auto destination = std::prev(origin);
    thor::PathAlgorithm* path_algorithm = get_path_algorithm(costing, *origin, *destination);
    path_algorithm->Clear();

    // If we are continuing through a location we need to make sure we
    // only allow the edge that was used previously (avoid u-turns)
    while (!path.empty() && destination->path_edges_size() > 1) {
      if (destination->path_edges().rbegin()->graph_id() == path.front().edgeid) {
        destination->mutable_path_edges()->SwapElements(0, destination->path_edges_size() - 1);
      }
      destination->mutable_path_edges()->RemoveLast();
    }

    // Get best path and keep it
    auto temp_path = get_path(path_algorithm, *origin, *destination, costing);
    temp_path.swap(path);

    // Merge through legs by updating the time and splicing the lists
    if (!temp_path.empty()) {
      auto offset = path.back().elapsed_time;
      std::for_each(temp_path.begin(), temp_path.end(),
                    [offset](PathInfo& i) { i.elapsed_time += offset; });
      if (path.back().edgeid == temp_path.front().edgeid) {
        path.pop_back();
      }
      path.insert(path.end(), temp_path.begin(), temp_path.end());
    }

    // Build trip path for this leg and add to the result if this
    // location is a BREAK or if this is the last location
    if (origin->type() == odin::Location::kBreak) {
      // Move destination back to the last break and collect the throughs
      std::list<odin::Location> throughs;
      while (destination->type() != odin::Location::kBreak) {
        throughs.push_back(*destination);
        --destination;
      }

      // Create controller for default route attributes
      AttributesController controller;

      // Form output information based on path edges
      auto trip_path = thor::TripPathBuilder::Build(controller, *reader, mode_costing, path, *origin,
                                                    *destination, throughs, interrupt);
      path.clear();

      // Keep the protobuf path
      trip_paths.emplace_back(std::move(trip_path));
    }
  }

  // return the trip paths
  return trip_paths;
}

std::list<valhalla::odin::TripPath> thor_worker_t::path_depart_at(
    google::protobuf::RepeatedPtrField<valhalla::odin::Location>& correlated,
    const std::string& costing) {
  // Things we'll need
  std::vector<thor::PathInfo> path;
  std::list<valhalla::odin::TripPath> trip_paths;
  correlated.begin()->set_type(odin::Location::kBreak);
  correlated.rbegin()->set_type(odin::Location::kBreak);

  // For each pair of locations
  for (auto destination = ++correlated.begin(); destination != correlated.end(); ++destination) {
    // Get the algorithm type for this location pair
    auto origin = std::prev(destination);
    thor::PathAlgorithm* path_algorithm = get_path_algorithm(costing, *origin, *destination);
    path_algorithm->Clear();

    // If we are continuing through a location we need to make sure we
    // only allow the edge that was used previously (avoid u-turns)
    while (!path.empty() && origin->path_edges_size() > 1) {
      if (origin->path_edges().rbegin()->graph_id() == path.back().edgeid) {
        origin->mutable_path_edges()->SwapElements(0, origin->path_edges_size() - 1);
      }
      origin->mutable_path_edges()->RemoveLast();
    }

    // Get best path and keep it
    auto temp_path = get_path(path_algorithm, *origin, *destination, costing);

    // Merge through legs by updating the time and splicing the lists
    if (!path.empty()) {
      auto offset = path.back().elapsed_time;
      std::for_each(temp_path.begin(), temp_path.end(),
                    [offset](PathInfo& i) { i.elapsed_time += offset; });
      if (path.back().edgeid == temp_path.front().edgeid) {
        path.pop_back();
      }
      path.insert(path.end(), temp_path.begin(), temp_path.end());
    } // Didnt need to merge
    else {
      path.swap(temp_path);
    }

    // Build trip path for this leg and add to the result if this
    // location is a BREAK or if this is the last location
    if (destination->type() == odin::Location::kBreak) {
      // Move origin back to the last break and collect the throughs
      std::list<odin::Location> throughs;
      while (origin->type() != odin::Location::kBreak) {
        throughs.push_front(*origin);
        --origin;
      }

      // Create controller for default route attributes
      AttributesController controller;

      // Form output information based on path edges
      auto trip_path = thor::TripPathBuilder::Build(controller, *reader, mode_costing, path, *origin,
                                                    *destination, throughs, interrupt);
      path.clear();

      // Keep the protobuf path
      trip_paths.emplace_back(std::move(trip_path));
    }
  }

  // return the trip paths
  return trip_paths;
}

} // namespace thor
} // namespace valhalla
