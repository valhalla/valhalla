#include <prime_server/prime_server.hpp>

using namespace prime_server;

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

  json::ArrayPtr locations(const std::vector<baldr::PathLocation>& correlated) {
    auto input_locs = json::array({});
    for(size_t i = 0; i < correlated.size(); i++) {
      input_locs->emplace_back(
        json::map({
          {"lat", json::fp_t{correlated[i].latlng_.lat(), 6}},
          {"lon", json::fp_t{correlated[i].latlng_.lng(), 6}}
        })
      );
    }
    return input_locs;
  }

}

namespace valhalla {
  namespace thor {

    worker_t::result_t  thor_worker_t::optimized_path(const std::vector<PathLocation> correlated, const std::string &costing, const std::string &request_str) {
      worker_t::result_t result{true};
      //get time for start of request
      auto s = std::chrono::system_clock::now();

      // Forward the original request
      result.messages.emplace_back(std::move(request_str));

      TimeDistanceMatrix tdmatrix;
      std::vector<TimeDistance> td = tdmatrix.ManyToMany(correlated, reader, mode_costing, mode);
      std::vector<float> time_costs;
      for (size_t i = 0; i < td.size(); i++) {
        time_costs.emplace_back(static_cast<float>(td[i].time));
      }
      for (size_t i = 0; i < correlated.size(); i++) {
        LOG_DEBUG("BEFORE reorder of locations:: " + std::to_string(correlated[i].latlng_.lat()) + ", "+ std::to_string(correlated[i].latlng_.lng()));
      }
      Optimizer optimizer;
      //returns the optimal order of the path_locations
      auto order = optimizer.Solve(correlated.size(), time_costs);
      std::vector<PathLocation> best_order;
      for (size_t i = 0; i< order.size(); i++) {
        LOG_INFO("optimizer return:: " + std::to_string(order[i]));
        best_order.emplace_back(correlated[order[i]]);
        LOG_DEBUG("reordered locations:: " + std::to_string(best_order[i].latlng_.lat()) + ", "+ std::to_string(best_order[i].latlng_.lng()));
      }

      return thor_worker_t::getPathDepartFrom(best_order, costing, date_time_type, request_str, result);
    }

  }
}
