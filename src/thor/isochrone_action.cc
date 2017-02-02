#include <prime_server/prime_server.hpp>

using namespace prime_server;

#include "baldr/geojson.h"
#include "midgard/logging.h"

#include "thor/service.h"

using namespace valhalla::midgard;

namespace {
  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};
}

namespace valhalla {
  namespace thor {

    worker_t::result_t  thor_worker_t::isochrone(const boost::property_tree::ptree &request, prime_server::http_request_info_t& request_info) {
      //get time for start of request
      auto s = std::chrono::system_clock::now();

      parse_locations(request);
      auto costing = parse_costing(request);

      std::vector<float> contours;
      std::vector<std::string> colors;
      for(const auto& contour : request.get_child("contours")) {
        contours.push_back(contour.second.get<float>("time"));
        colors.push_back(contour.second.get<std::string>("color", ""));
      }
      auto polygons = request.get<bool>("polygons", false);
      auto denoise = std::max(std::min(request.get<float>("denoise", 1.f), 1.f), 0.f);
      auto generalize = request.get<float>("generalize", .2f);

      //get the raster
      //Extend the times in the 2-D grid to be 10 minutes beyond the highest contour time.
      //Cost (including penalties) is used when adding to the adjacency list but the elapsed
      //time in seconds is used when terminating the search. The + 10 minutes adds a buffer for edges
      //where there has been a higher cost that might still be marked in the isochrone
      auto grid = (costing == "multimodal" || costing == "transit") ?
        isochrone_gen.ComputeMultiModal(correlated, contours.back()+10, reader, mode_costing, mode) :
        isochrone_gen.Compute(correlated, contours.back()+10, reader, mode_costing, mode);

      //turn it into geojson
      auto isolines = grid->GenerateContours(contours, polygons, denoise, generalize);
      auto geojson = baldr::json::to_geojson<PointLL>(isolines, polygons, colors);
      auto id = request.get_optional<std::string>("id");
      if(id)
        geojson->emplace("id", *id);
      std::stringstream stream; stream << *geojson;

      //get processing time for thor
       auto e = std::chrono::system_clock::now();
       std::chrono::duration<float, std::milli> elapsed_time = e - s;
       //log request if greater than X (ms)
       if (!healthcheck && !request_info.spare && elapsed_time.count() / (correlated_s.size() * correlated_t.size()) > long_request) {
         std::stringstream ss;
         boost::property_tree::json_parser::write_json(ss, request, false);
         LOG_WARN("thor::isochrone elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
         LOG_WARN("thor::isochrone exceeded threshold::"+ ss.str());
         midgard::logging::Log("valhalla_thor_long_request_isochrone", " [ANALYTICS] ");
       }
      //return the geojson
      worker_t::result_t result{false};
      http_response_t response(200, "OK", stream.str(), headers_t{CORS, JSON_MIME});
      response.from_info(request_info);
      result.messages.emplace_back(response.to_string());
      return result;
    }

  }
}
