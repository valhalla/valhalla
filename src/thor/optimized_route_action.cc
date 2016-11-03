#include <prime_server/prime_server.hpp>

using namespace prime_server;

#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/baldr/json.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/pedestriancost.h>

#include "thor/service.h"
#include "thor/optimizer.h"
#include "thor/costmatrix.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

}

namespace valhalla {
  namespace thor {

  worker_t::result_t  thor_worker_t::optimized_route(const boost::property_tree::ptree& request, const std::string &request_str, const bool header_dnt) {
    parse_locations(request);
    auto costing = parse_costing(request);

    valhalla::midgard::logging::Log("matrix_type::optimized_route", " [ANALYTICS] ");
    worker_t::result_t result{true};
    //get time for start of request
    auto s = std::chrono::system_clock::now();

    // Forward the original request
    result.messages.emplace_back(std::move(request_str));

    // Use CostMatrix to find costs from each location to every other location
    CostMatrix costmatrix;
    std::vector<thor::TimeDistance> td = costmatrix.SourceToTarget(correlated_s, correlated_t, reader, mode_costing, mode);

    // Return an error if any locations are totally unreachable
    std::vector<baldr::PathLocation> correlated =  (correlated_s.size() > correlated_t.size() ? correlated_s : correlated_t);

    // Set time costs to send to Optimizer.
    std::vector<float> time_costs;
    bool reachable = true;
    for(size_t i = 0; i < td.size(); ++i) {
      // If any location is completely unreachable then we cant have a connected path
      if(i % correlated.size() == 0) {
        if(!reachable)
          throw valhalla_exception_t{400, 441, " at index " + std::to_string(i / correlated.size())};
        reachable = false;
      }
      reachable = reachable || td[i].time != kMaxCost;
      // Keep the times for the reordering
      time_costs.emplace_back(static_cast<float>(td[i].time));
    }

    Optimizer optimizer;
    //returns the optimal order of the path_locations
    auto order = optimizer.Solve(correlated.size(), time_costs);
    std::vector<PathLocation> best_order;
    for (size_t i = 0; i< order.size(); i++)
      best_order.emplace_back(correlated[order[i]]);

    auto trippaths = path_depart_at(best_order, costing, date_time_type, request_str);
    for (const auto &trippath: trippaths)
      result.messages.emplace_back(trippath.SerializeAsString());

    //get processing time for thor
    auto e = std::chrono::system_clock::now();
    std::chrono::duration<float, std::milli> elapsed_time = e - s;
    //log request if greater than X (ms)
    if (!header_dnt && ((elapsed_time.count() / correlated_s.size()) || elapsed_time.count() / correlated_t.size()) > long_request) {
      LOG_WARN("thor::optimized_route elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
      LOG_WARN("thor::optimized_route exceeded threshold::"+ request_str);
      midgard::logging::Log("valhalla_thor_long_request_manytomany", " [ANALYTICS] ");
    }
    return result;
  }

  }
}
