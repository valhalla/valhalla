#include <prime_server/prime_server.hpp>

using namespace prime_server;

#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/baldr/json.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/pedestriancost.h>
#include <valhalla/thor/costmatrix.h>

#include "thor/service.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;


namespace {

  const std::unordered_map<std::string, thor_worker_t::MATRIX_TYPE> MATRIX {
    {"one_to_many", thor_worker_t::ONE_TO_MANY},
    {"many_to_one", thor_worker_t::MANY_TO_ONE},
    {"many_to_many", thor_worker_t::MANY_TO_MANY}
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

  json::ArrayPtr serialize_row(const std::vector<PathLocation>& correlated, const std::vector<TimeDistance>& tds,
      const size_t origin, const size_t destination, const size_t start, const size_t end, double distance_scale) {
    auto row = json::array({});
    for(size_t i = start; i < end; i++) {
      //check to make sure a route was found; if not, return null for distance & time in matrix result
      if (tds[i].time != kMaxCost) {
        row->emplace_back(json::map({
          {"from_index", static_cast<uint64_t>(origin)},
          {"to_index", static_cast<uint64_t>(destination + (i - start))},
          {"time", static_cast<uint64_t>(tds[i].time)},
          {"distance", json::fp_t{tds[i].dist * distance_scale, 3}}
        }));
      } else {
        row->emplace_back(json::map({
          {"from_index", static_cast<uint64_t>(origin)},
          {"to_index", static_cast<uint64_t>(destination + (i - start))},
          {"time", static_cast<std::nullptr_t>(nullptr)},
          {"distance", static_cast<std::nullptr_t>(nullptr)}
        }));
      }
    }
    return row;
  }

  //Returns a row vector of computed time and distance from the first (origin) location to each additional location provided.
  // {
  //   input_locations: [{},{},{}],
  //   one_to_many:
  //   [
  //     [{origin0,dest0,0,0},{origin0,dest1,x,x},{origin0,dest2,x,x},{origin0,dest3,x,x}]
  //   ]
  // }
  json::MapPtr serialize_one_to_many(const boost::optional<std::string>& id, const std::vector<PathLocation>& correlated, const std::vector<TimeDistance>& tds, std::string& units, double distance_scale) {
     auto json = json::map({
      {"one_to_many", json::array({serialize_row(correlated, tds, 0, 0, 0, tds.size(), distance_scale)})},
      {"locations", json::array({locations(correlated)})},
      {"units", units},
    });
    if (id)
      json->emplace("id", *id);
    return json;
  }

  //Returns a column vector of computed time and distance from each location to the last (destination) location provided.
  // {
  //   input_locations: [{},{},{}],
  //   many_to_one:
  //   [
  //     [{origin0,dest0,x,x}],
  //     [{origin1,dest0,x,x}],
  //     [{origin2,dest0,x,x}],
  //     [{origin3,dest0,0,0}]
  //   ]
  // }
  json::MapPtr serialize_many_to_one(const boost::optional<std::string>& id, const std::vector<PathLocation>& correlated, const std::vector<TimeDistance>& tds, std::string& units, double distance_scale) {
    json::ArrayPtr column_matrix = json::array({});
    for(size_t i = 0; i < correlated.size(); ++i)
      column_matrix->emplace_back(serialize_row(correlated, tds, i, correlated.size() - 1, i, i + 1, distance_scale));
    auto json = json::map({
      {"many_to_one", column_matrix},
      {"locations", json::array({locations(correlated)})},
      {"units", units},
    });
    if (id)
      json->emplace("id", *id);
    return json;
  }

  //Returns a square matrix of computed time and distance from each location to every other location.
  // {
  //   input_locations: [{},{},{}],
  //   many_to_many:
  //   [
  //     [{origin0,dest0,0,0},{origin0,dest1,x,x},{origin0,dest2,x,x},{origin0,dest3,x,x}],
  //     [{origin1,dest0,x,x},{origin1,dest1,0,0},{origin1,dest2,x,x},{origin1,dest3,x,x}],
  //     [{origin2,dest0,x,x},{origin2,dest1,x,x},{origin2,dest2,0,0},{origin2,dest3,x,x}],
  //     [{origin3,dest0,x,x},{origin3,dest1,x,x},{origin3,dest2,x,x},{origin3,dest3,0,0}]
  //   ]
  // }
  json::MapPtr serialize_many_to_many(const boost::optional<std::string>& id, const std::vector<PathLocation>& correlated, const std::vector<TimeDistance>& tds, std::string& units, double distance_scale) {
    json::ArrayPtr square_matrix = json::array({});
    for(size_t i = 0; i < correlated.size(); ++i)
      square_matrix->emplace_back(serialize_row(correlated, tds, i, 0, correlated.size() * i, correlated.size() * (i + 1), distance_scale));
    auto json = json::map({
      {"many_to_many", square_matrix},
      {"locations", json::array({locations(correlated)})},
      {"units", units},
    });
    if (id)
      json->emplace("id", *id);
    return json;
  }

}

namespace valhalla {
  namespace thor {

    worker_t::result_t  thor_worker_t::matrix(const MATRIX_TYPE matrix_type, const std::string &costing, const boost::property_tree::ptree &request, http_request_t::info_t& request_info) {
      //get time for start of request
      auto s = std::chrono::system_clock::now();
      // Parse out units; if none specified, use kilometers
      double distance_scale = kKmPerMeter;
      auto matrix_action_type = request.get_optional<std::string>("matrix_type");
      auto units = request.get<std::string>("units", "km");
      if (units == "mi")
        distance_scale = kMilePerMeter;


      //do the real work
      json::MapPtr json;
      thor::CostMatrix costmatrix;
      switch ( matrix_type) {
        case MATRIX_TYPE::ONE_TO_MANY:
          json = serialize_one_to_many(request.get_optional<std::string>("id"), correlated, costmatrix.SourceToTarget({correlated.front()}, correlated, reader, mode_costing, mode), units, distance_scale);
          matrix_action_type = "one-to-many";
          break;
        case MATRIX_TYPE::MANY_TO_ONE:
          json = serialize_many_to_one(request.get_optional<std::string>("id"), correlated, costmatrix.SourceToTarget(correlated, {correlated.back()}, reader, mode_costing, mode), units, distance_scale);
          matrix_action_type = "many-to-one";
          break;
        case MATRIX_TYPE::MANY_TO_MANY:
          json = serialize_many_to_many(request.get_optional<std::string>("id"), correlated, costmatrix.SourceToTarget(correlated, correlated, reader, mode_costing, mode), units, distance_scale);
          matrix_action_type = "many-to-many";
          break;
        }

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
      if (!request_info.do_not_track && (elapsed_time.count() / correlated.size()) > long_request) {
        std::stringstream ss;
        boost::property_tree::json_parser::write_json(ss, request, false);
        LOG_WARN("thor::" + *matrix_action_type + " matrix request elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
        LOG_WARN("thor::" + *matrix_action_type + " matrix request exceeded threshold::"+ ss.str());
        (matrix_type!=MATRIX_TYPE::MANY_TO_MANY) ? midgard::logging::Log("valhalla_thor_long_request_route", " [ANALYTICS] ") : midgard::logging::Log("valhalla_thor_long_request_manytomany", " [ANALYTICS] ");
      }

      http_response_t response(200, "OK", stream.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
      response.from_info(request_info);
      worker_t::result_t result{false};
      result.messages.emplace_back(response.to_string());
      return result;
    }

  }
}
