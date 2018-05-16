#include "thor/worker.h"
#include <cstdint>

#include "baldr/json.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "proto/tripcommon.pb.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "thor/attributes_controller.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace valhalla {
namespace thor {

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

  // Use A* if any origin and destination edges are the same - otherwise
  // use bidirectional A*. Bidirectional A* does not handle trivial cases
  // with oneways.
  for (auto& edge1 : origin.path_edges()) {
    for (auto& edge2 : destination.path_edges()) {
      if (edge1.graph_id() == edge2.graph_id()) {
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
  // Find the path. If bidirectional A* disable use of destination only
  // edges on the first pass. If there is a failure, we allow them on the
  // second pass.
  valhalla::sif::cost_ptr_t cost = mode_costing[static_cast<uint32_t>(mode)];
  if (path_algorithm == &bidir_astar) {
    cost->set_allow_destination_only(false);
  }
  cost->set_pass(0);
  auto path = path_algorithm->GetBestPath(origin, destination, reader, mode_costing, mode);

  // If path is not found try again with relaxed limits (if allowed)
  if (path.empty() || (costing == "pedestrian" && path_algorithm->has_ferry())) {
    if (cost->AllowMultiPass()) {
      // 2nd pass. Less aggressive hierarchy transitioning, and retry with more candidate
      // edges(filterd by heading in loki).

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
      path = path_algorithm->GetBestPath(origin, destination, reader, mode_costing, mode);
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
      auto trip_path = thor::TripPathBuilder::Build(controller, reader, mode_costing, path, *origin,
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
      auto trip_path = thor::TripPathBuilder::Build(controller, reader, mode_costing, path, *origin,
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
