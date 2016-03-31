#include <prime_server/prime_server.hpp>

using namespace prime_server;

#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/baldr/json.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/pedestriancost.h>
#include <valhalla/thor/optimizer.h>
#include <valhalla/thor/timedistancematrix.h>

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

      TimeDistanceMatrix tdmatrix;
      std::vector<TimeDistance> td = tdmatrix.ManyToMany(correlated, reader, mode_costing, mode);
      std::vector<float> time_costs;
      for (size_t i = 0; i < td.size(); i++)
        time_costs.emplace_back(static_cast<float>(td[i].time));

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
      if (!header_dnt && (elapsed_time.count() / correlated.size()) > long_request_manytomany) {
        LOG_WARN("thor::route optimized_path elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
        LOG_WARN("thor::route optimized_path exceeded threshold::"+ request_str);
        midgard::logging::Log("valhalla_thor_long_request_manytomany", " [ANALYTICS] ");
      }
      return result;
    }

  }
}
