#include <cstdint>
#include <prime_server/prime_server.hpp>

using namespace prime_server;

#include "midgard/logging.h"
#include "midgard/constants.h"
#include "baldr/json.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"

#include "thor/service.h"
#include "thor/costmatrix.h"
#include "thor/timedistancematrix.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace std {
  template <>
  struct hash<thor_worker_t::ACTION_TYPE>
  {
    std::size_t operator()(const thor_worker_t::ACTION_TYPE& a) const {
      return std::hash<int>()(a);
    }
  };
}

namespace {

  // TODO: Maybe find a better spot to put this or reconsider if it is necessary.
  using ignore_list_t = std::unordered_map<uint32_t, std::unordered_set<uint32_t>>;

  const std::unordered_map<thor_worker_t::ACTION_TYPE, std::string> ACTION_TO_STRING {
     {thor_worker_t::ONE_TO_MANY, "one_to_many"},
     {thor_worker_t::MANY_TO_ONE, "many_to_one"},
     {thor_worker_t::MANY_TO_MANY, "many_to_many"},
     {thor_worker_t::SOURCES_TO_TARGETS, "sources_to_targets"},
     {thor_worker_t::OPTIMIZED_ROUTE, "optimized_route"}
   };


  constexpr double kMilePerMeter = 0.000621371;
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

  json::ArrayPtr serialize_row(const std::vector<TimeDistance>& tds,
      size_t start_td, const size_t td_count, const size_t source_index, const size_t target_index, double distance_scale) {
    auto row = json::array({});
    for(size_t i = start_td; i < start_td + td_count; ++i) {
      //check to make sure a route was found; if not, return null for distance & time in matrix result
      if (tds[i].time != kMaxCost) {
        row->emplace_back(json::map({
          {"from_index", static_cast<uint64_t>(source_index)},
          {"to_index", static_cast<uint64_t>(target_index + (i - start_td))},
          {"time", static_cast<uint64_t>(tds[i].time)},
          {"distance", json::fp_t{tds[i].dist * distance_scale, 3}}
        }));
      } else {
        row->emplace_back(json::map({
          {"from_index", static_cast<uint64_t>(source_index)},
          {"to_index", static_cast<uint64_t>(target_index + (i - start_td))},
          {"time", static_cast<std::nullptr_t>(nullptr)},
          {"distance", static_cast<std::nullptr_t>(nullptr)}
        }));
      }
    }
    return row;
  }

  json::MapPtr serialize(const std::string action, const boost::optional<std::string>& id, const std::vector<PathLocation>& correlated_s, const std::vector<PathLocation>& correlated_t, const std::vector<TimeDistance>& tds, std::string& units, double distance_scale) {
    json::ArrayPtr matrix = json::array({});
    for(size_t source_index = 0; source_index < correlated_s.size(); ++source_index) {
        matrix->emplace_back(
          serialize_row(tds, source_index * correlated_t.size(), correlated_t.size(),
                        source_index, action == "many_to_one" ? correlated_s.size()-1 : 0, distance_scale));
    }
    auto json = json::map({
      {action, matrix},
      {"units", units},
    });
    if (action == "sources_to_targets") {
      json->emplace("targets", json::array({locations(correlated_t)}));
      json->emplace("sources", json::array({locations(correlated_s)}));
    } else {
      json->emplace("locations", json::array({locations(correlated_s.size() > correlated_t.size() ? correlated_s : correlated_t)}));
    }
    if (id)
      json->emplace("id", *id);
    return json;
  }

  ignore_list_t parse_ignore_list (const boost::property_tree::ptree& request) {
    ignore_list_t ignore_list {};
    auto ignore_list_json = request.get_child_optional("ignore_list");

    if (ignore_list_json) {
      for (const auto& pair : *ignore_list_json) {
        uint32_t source = pair.second.get<uint32_t>("source");
        uint32_t target = pair.second.get<uint32_t>("target");
        ignore_list[source].emplace(target);
      }
    }
    // TODO: Do we need to check this?
    else {
      throw valhalla_exception_t {500, 159};
    }

    return ignore_list;
  }

}

namespace valhalla {
  namespace thor {

    worker_t::result_t  thor_worker_t::matrix(ACTION_TYPE action, const boost::property_tree::ptree &request, http_request_info_t& request_info) {
      //get time for start of request
      auto s = std::chrono::system_clock::now();

      parse_locations(request);
      auto costing = parse_costing(request);
      auto ignore_list = parse_ignore_list (request);

      const auto& matrix_type = ACTION_TO_STRING.find(action)->second;
      if (!healthcheck)
        valhalla::midgard::logging::Log("matrix_type::" + matrix_type, " [ANALYTICS] ");

      // Parse out units; if none specified, use kilometers
      double distance_scale = kKmPerMeter;
      auto units = request.get<std::string>("units", "km");
      if (units == "mi")
        distance_scale = kMilePerMeter;

      json::MapPtr json;
      //do the real work
      std::vector<TimeDistance> time_distances;
      auto costmatrix = [&]() {
        thor::CostMatrix matrix;
        return matrix.SourceToTarget(correlated_s, correlated_t, reader, mode_costing, mode,
                                     max_matrix_distance.find(costing)->second, ignore_list);
      };
      auto timedistancematrix = [&]() {
        thor::TimeDistanceMatrix matrix;
        return matrix.SourceToTarget(correlated_s, correlated_t, reader, mode_costing,
                                    mode, max_matrix_distance.find(costing)->second, ignore_list);
      };
      switch (source_to_target_algorithm) {
        case SELECT_OPTIMAL:
          //TODO - Do further performance testing to pick the best algorithm for the job
          switch (mode) {
            case TravelMode::kPedestrian:
            case TravelMode::kBicycle:
            case TravelMode::kPublicTransit:
              time_distances = timedistancematrix();
              break;
            default:
              time_distances = costmatrix();
          }
          break;
        case COST_MATRIX:
          time_distances = costmatrix();
          break;
        case TIME_DISTANCE_MATRIX:
          time_distances = timedistancematrix();
          break;
      }
      json = serialize(matrix_type, request.get_optional<std::string>("id"), correlated_s, correlated_t,
        time_distances, units, distance_scale);

      //jsonp callback if need be
      std::ostringstream stream;
      auto jsonp = request.get_optional<std::string>("jsonp");
      if(jsonp)
        stream << *jsonp << '(';
      stream << *json;
      if(jsonp)
        stream << ')';

      //get processing time for thor
      auto e = std::chrono::system_clock::now();
      std::chrono::duration<float, std::milli> elapsed_time = e - s;
      //log request if greater than X (ms)
      if (!healthcheck && !request_info.spare && elapsed_time.count() / (correlated_s.size() * correlated_t.size()) > long_request) {
        std::stringstream ss;
        boost::property_tree::json_parser::write_json(ss, request, false);
        LOG_WARN("thor::" + matrix_type + " matrix request elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
        LOG_WARN("thor::" + matrix_type + " matrix request exceeded threshold::"+ ss.str());
        midgard::logging::Log("valhalla_thor_long_request_matrix", " [ANALYTICS] ");
      }
      http_response_t response(200, "OK", stream.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
      response.from_info(request_info);
      worker_t::result_t result{false};
      result.messages.emplace_back(response.to_string());
      return result;
    }
  }
}
