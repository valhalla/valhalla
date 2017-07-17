#include "thor/worker.h"

#include "midgard/logging.h"
#include "midgard/constants.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "thor/optimizer.h"
#include "thor/costmatrix.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace valhalla {
  namespace thor {

  std::list<valhalla::odin::TripPath> thor_worker_t::optimized_route(const boost::property_tree::ptree& request) {
    parse_locations(request);
    auto costing = parse_costing(request);

    if (!healthcheck)
      valhalla::midgard::logging::Log("matrix_type::optimized_route", " [ANALYTICS] ");

    // Use CostMatrix to find costs from each location to every other location
    CostMatrix costmatrix;
    std::vector<thor::TimeDistance> td = costmatrix.SourceToTarget(correlated_s, correlated_t, reader,
                                                                  mode_costing, mode,
                                                                  max_matrix_distance.find(costing)->second);

    // Return an error if any locations are totally unreachable
    std::vector<baldr::PathLocation> correlated =  (correlated_s.size() > correlated_t.size() ? correlated_s : correlated_t);

    // Set time costs to send to Optimizer.
    std::vector<float> time_costs;
    bool reachable = true;
    for(size_t i = 0; i < td.size(); ++i) {
      // If any location is completely unreachable then we cant have a connected path
      if(i % correlated.size() == 0) {
        if(!reachable)
          throw valhalla_exception_t{441, " at index " + std::to_string(i / correlated.size())};
        reachable = false;
      }
      reachable = reachable || td[i].time != kMaxCost;
      // Keep the times for the reordering
      time_costs.emplace_back(static_cast<float>(td[i].time));
    }

    Optimizer optimizer;
    //returns the optimal order of the path_locations
    optimal_order = optimizer.Solve(correlated.size(), time_costs);
    std::vector<PathLocation> best_order;
    for (size_t i = 0; i< optimal_order.size(); i++)
      best_order.emplace_back(correlated[optimal_order[i]]);

    auto trippaths = path_depart_at(best_order, costing, date_time_type);

    return trippaths;
  }

  }
}
