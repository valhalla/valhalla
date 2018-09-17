#include "thor/worker.h"

#include "midgard/constants.h"
#include "midgard/logging.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "thor/costmatrix.h"
#include "thor/optimizer.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace valhalla {
namespace thor {

std::list<valhalla::odin::TripPath> thor_worker_t::optimized_route(valhalla_request_t& request) {
  parse_locations(request);
  auto costing = parse_costing(request);

  if (!request.options.do_not_track()) {
    valhalla::midgard::logging::Log("matrix_type::optimized_route", " [ANALYTICS] ");
  }

  // Use CostMatrix to find costs from each location to every other location
  CostMatrix costmatrix;
  std::vector<thor::TimeDistance> td =
      costmatrix.SourceToTarget(request.options.sources(), request.options.targets(), *reader,
                                mode_costing, mode, max_matrix_distance.find(costing)->second);

  // Return an error if any locations are totally unreachable
  const auto& correlated =
      (request.options.sources_size() > request.options.targets_size() ? request.options.sources()
                                                                       : request.options.targets());

  // Set time costs to send to Optimizer.
  std::vector<float> time_costs;
  bool reachable = true;
  for (size_t i = 0; i < td.size(); ++i) {
    // If any location is completely unreachable then we cant have a connected path
    if (i % correlated.size() == 0) {
      if (!reachable) {
        throw valhalla_exception_t{441, " at index " + std::to_string(i / correlated.size())};
      };
      reachable = false;
    }
    reachable = reachable || td[i].time != kMaxCost;
    // Keep the times for the reordering
    time_costs.emplace_back(static_cast<float>(td[i].time));
  }

  Optimizer optimizer;
  // returns the optimal order of the path_locations
  auto optimal_order = optimizer.Solve(correlated.size(), time_costs);
  // put the optimal order into the locations array
  request.options.mutable_locations()->Clear();
  for (size_t i = 0; i < optimal_order.size(); i++) {
    request.options.mutable_locations()->Add()->CopyFrom(correlated.Get(optimal_order[i]));
  }

  return path_depart_at(*request.options.mutable_locations(), costing);
}

} // namespace thor
} // namespace valhalla
