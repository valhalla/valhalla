#include "thor/worker.h"
#include <cstdint>

#include "midgard/logging.h"
#include "midgard/constants.h"
#include "baldr/json.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "proto/trippath.pb.h"
#include "thor/attributes_controller.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace valhalla {
  namespace thor {

  std::list<valhalla::odin::TripPath> thor_worker_t::route(const boost::property_tree::ptree& request, const boost::optional<int> &date_time_type){
    parse_locations(request);
    auto costing = parse_costing(request);

    auto trippaths = (date_time_type && *date_time_type == 2) ?
        path_arrive_by(correlated, costing) :
        path_depart_at(correlated, costing, date_time_type);

    return trippaths;
  }

  thor::PathAlgorithm* thor_worker_t::get_path_algorithm(const std::string& routetype,
        const baldr::PathLocation& origin, const baldr::PathLocation& destination) {
    if (routetype == "multimodal" || routetype == "transit") {
      multi_modal_astar.set_interrupt(interrupt);
      return &multi_modal_astar;
    }

    // Use A* if any origin and destination edges are the same - otherwise
    // use bidirectional A*. Bidirectional A* does not handle trivial cases
    // with oneways.
    for (auto& edge1 : origin.edges) {
      for (auto& edge2 : destination.edges) {
        if (edge1.id == edge2.id) {
          astar.set_interrupt(interrupt);
          return &astar;
        }
      }
    }
    bidir_astar.set_interrupt(interrupt);
    return &bidir_astar;
  }

  std::vector<thor::PathInfo> thor_worker_t::get_path(PathAlgorithm* path_algorithm, baldr::PathLocation& origin,
      baldr::PathLocation& destination) {
    // Find the path. If bidirectional A* disable use of destination only
    // edges on the first pass. If there is a failure, we allow them on the
    // second pass.
    valhalla::sif::cost_ptr_t cost = mode_costing[static_cast<uint32_t>(mode)];
    if (path_algorithm == &bidir_astar) {
      cost->set_allow_destination_only(false);
    }
    auto path = path_algorithm->GetBestPath(origin, destination, reader,
                                             mode_costing, mode);
    // If path is not found try again with relaxed limits (if allowed)
    if (path.empty()) {
      if (cost->AllowMultiPass()) {
        // 2nd pass. Less aggressive hierarchy transitioning.
        path_algorithm->Clear();
        bool using_astar = (path_algorithm == &astar);
        float relax_factor = using_astar ? 16.0f : 8.0f;
        float expansion_within_factor = using_astar ? 4.0f : 2.0f;
        cost->RelaxHierarchyLimits(relax_factor, expansion_within_factor);
        cost->set_allow_destination_only(true);
        path = path_algorithm->GetBestPath(origin, destination,
                                  reader, mode_costing, mode);
      }
    }

    // All or nothing
    if(path.empty())
      throw valhalla_exception_t{442};
    return path;
  }

  std::list<valhalla::odin::TripPath> thor_worker_t::path_arrive_by(std::vector<PathLocation>& correlated, const std::string &costing) {
    // Things we'll need
    std::vector<thor::PathInfo> path;
    std::list<valhalla::odin::TripPath> trip_paths;
    correlated.front().stoptype_ = correlated.back().stoptype_ = Location::StopType::BREAK;

    // For each pair of locations
    for(auto origin = ++correlated.rbegin(); origin != correlated.rend(); ++origin) {
      // Get the algorithm type for this location pair
      auto destination = std::prev(origin);
      thor::PathAlgorithm* path_algorithm = get_path_algorithm(costing, *origin, *destination);
      path_algorithm->Clear();

      // If we are continuing through a location we need to make sure we
      // only allow the edge that was used previously (avoid u-turns)
      if(!path.empty()) {
        auto erasure_position = std::remove_if(destination->edges.begin(), destination->edges.end(),
          [&path](const PathLocation::PathEdge& e){
            return e.id != path.front().edgeid;
        });
        destination->edges.erase(erasure_position, destination->edges.end());
      }

      // Get best path and keep it
      auto temp_path = get_path(path_algorithm, *origin, *destination);
      temp_path.swap(path);

      // Merge through legs by updating the time and splicing the lists
      if(!temp_path.empty()) {
        auto offset = path.back().elapsed_time;
        std::for_each(temp_path.begin(), temp_path.end(), [offset](PathInfo& i) { i.elapsed_time += offset; });
        if(path.back().edgeid == temp_path.front().edgeid) path.pop_back();
        path.insert(path.end(), temp_path.begin(), temp_path.end());
      }

      // Build trip path for this leg and add to the result if this
      // location is a BREAK or if this is the last location
      if (origin->stoptype_ == Location::StopType::BREAK) {
        // Move destination back to the last break and collect the throughs
        std::list<PathLocation> throughs;
        while(destination->stoptype_ != Location::StopType::BREAK) {
          throughs.push_back(*destination);
          --destination;
        }

        // Create controller for default route attributes
        AttributesController controller;

        // Form output information based on path edges
        auto trip_path = thor::TripPathBuilder::Build(controller, reader, mode_costing, path,
                                                      *origin, *destination, throughs, interrupt);
        path.clear();

        // Keep the protobuf path
        trip_paths.emplace_back(std::move(trip_path));

        // Some logging
        log_admin(trip_paths.back());
      }
    }

    //return the trip paths
    return trip_paths;
  }

  std::list<valhalla::odin::TripPath> thor_worker_t::path_depart_at(std::vector<PathLocation>& correlated, const std::string &costing, const boost::optional<int> &date_time_type) {
    // Things we'll need
    std::vector<thor::PathInfo> path;
    std::list<valhalla::odin::TripPath> trip_paths;
    correlated.front().stoptype_ = correlated.back().stoptype_ = Location::StopType::BREAK;

    // For each pair of locations
    for(auto destination = ++correlated.begin(); destination != correlated.end(); ++destination) {
      // Get the algorithm type for this location pair
      auto origin = std::prev(destination);
      thor::PathAlgorithm* path_algorithm = get_path_algorithm(costing, *origin, *destination);
      path_algorithm->Clear();

      // If we are continuing through a location we need to make sure we
      // only allow the edge that was used previously (avoid u-turns)
      if(!path.empty()) {
        auto erasure_position = std::remove_if(origin->edges.begin(), origin->edges.end(),
          [&path](const PathLocation::PathEdge& e){
            return e.id != path.back().edgeid;
        });
        origin->edges.erase(erasure_position, origin->edges.end());
      }

      // Get best path and keep it
      auto temp_path = get_path(path_algorithm, *origin, *destination);

      // Merge through legs by updating the time and splicing the lists
      if(!path.empty()) {
        auto offset = path.back().elapsed_time;
        std::for_each(temp_path.begin(), temp_path.end(), [offset](PathInfo& i) { i.elapsed_time += offset; });
        if(path.back().edgeid == temp_path.front().edgeid) path.pop_back();
        path.insert(path.end(), temp_path.begin(), temp_path.end());
      }// Didnt need to merge
      else
        path.swap(temp_path);

      // Build trip path for this leg and add to the result if this
      // location is a BREAK or if this is the last location
      if (destination->stoptype_ == Location::StopType::BREAK) {
        // Move origin back to the last break and collect the throughs
        std::list<PathLocation> throughs;
        while(origin->stoptype_ != Location::StopType::BREAK) {
          throughs.push_front(*origin);
          --origin;
        }

        // Create controller for default route attributes
        AttributesController controller;

        // Form output information based on path edges
        auto trip_path = thor::TripPathBuilder::Build(controller, reader, mode_costing, path,
                                                      *origin, *destination, throughs, interrupt);
        path.clear();

        // Keep the protobuf path
        trip_paths.emplace_back(std::move(trip_path));

        // Some logging
        log_admin(trip_paths.back());
      }
    }

    //return the trip paths
    return trip_paths;
  }

  }
}
