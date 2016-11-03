#include <prime_server/prime_server.hpp>

using namespace prime_server;

#include <valhalla/baldr/geojson.h>

#include "thor/service.h"

namespace {
  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};
}

namespace valhalla {
  namespace thor {

    worker_t::result_t  thor_worker_t::trace_attributes(const boost::property_tree::ptree &request, prime_server::http_request_t::info_t& request_info) {
      parse_costing(request);
      parse_shape(request);

      //TODO: call Meili for map matching to get a collection of pathLocation Edges
      //TODO: serialize the trip path and return in Thor

      //turn it into json
      auto json = baldr::json::map({});
      auto id = request.get_optional<std::string>("id");
      if(id)
        json->emplace("id", *id);
      std::stringstream stream; stream << *json;

      //return the geojson
      worker_t::result_t result{false};
      http_response_t response(200, "OK", stream.str(), headers_t{CORS, JSON_MIME});
      response.from_info(request_info);
      result.messages.emplace_back(response.to_string());
      return result;
    }

  }
}
