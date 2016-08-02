#include <prime_server/prime_server.hpp>

using namespace prime_server;

#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/baldr/json.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/pedestriancost.h>
#include <valhalla/thor/optimizer.h>
#include <valhalla/thor/costmatrix.h>

#include "thor/service.h"

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

    worker_t::result_t  thor_worker_t::optimized_path(const std::vector<PathLocation>& correlated, const std::string &costing, const std::string &request_str, const bool header_dnt) {
      worker_t::result_t result{true};
      //get time for start of request
      auto s = std::chrono::system_clock::now();

      // Forward the original request
      result.messages.emplace_back(std::move(request_str));

      // Use CostMatrix to find costs from each location to every other location
      CostMatrix costmatrix;
      std::vector<thor::TimeDistance> td = costmatrix.SourceToTarget(correlated, correlated, reader, mode_costing, mode);

      // Return an error if any locations are totally unreachable
      uint32_t idx = 0;
      uint32_t n = correlated.size();
      for (uint32_t i = 0; i < n; i++) {
        bool reachable = false;
        for (uint32_t j = 0; j < n; j++) {
          if (td[idx].time != kMaxCost && i != j) {
            reachable = true;
          }
          idx++;
        }
        if (!reachable) {
          throw std::runtime_error("Location at index " + std::to_string(i) + " is unreachable");
        }
      }

      // Set time costs to send to Optimizer.
      std::vector<float> time_costs;
      for (auto& itr : td) {
        time_costs.emplace_back(static_cast<float>(itr.time));
      }

      for (size_t i = 0; i < correlated.size(); i++)
        LOG_INFO("BEFORE reorder of locations:: " + std::to_string(correlated[i].latlng_.lat()) + ", "+ std::to_string(correlated[i].latlng_.lng()));

      Optimizer optimizer;
      //returns the optimal order of the path_locations
      auto order = optimizer.Solve(correlated.size(), time_costs);
      std::vector<PathLocation> best_order;
      for (size_t i = 0; i< order.size(); i++) {
        best_order.emplace_back(correlated[order[i]]);
        LOG_INFO("reordered locations:: " + std::to_string(best_order[i].latlng_.lat()) + ", "+ std::to_string(best_order[i].latlng_.lng()));
      }

      auto trippaths = path_depart_from(best_order, costing, date_time_type, request_str);
      for (const auto &trippath: trippaths)
        result.messages.emplace_back(trippath.SerializeAsString());

      //get processing time for thor
      auto e = std::chrono::system_clock::now();
      std::chrono::duration<float, std::milli> elapsed_time = e - s;
      //log request if greater than X (ms)
      if (!header_dnt && (elapsed_time.count() / correlated.size()) > long_request) {
        LOG_WARN("thor::optimized_route elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
        LOG_WARN("thor::optimized_route exceeded threshold::"+ request_str);
        midgard::logging::Log("valhalla_thor_long_request_manytomany", " [ANALYTICS] ");
      }
      return result;
    }

  }
}
