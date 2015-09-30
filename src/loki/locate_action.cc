#include "loki/service.h"
#include "loki/search.h"

#include <valhalla/baldr/json.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/midgard/logging.h>

using namespace prime_server;
using namespace valhalla::baldr;


namespace {
  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

  //TODO: move json header to baldr
  //TODO: make objects serialize themselves

  json::ArrayPtr serialize_edges(const PathLocation& location, GraphReader& reader, bool verbose) {
    auto array = json::array({});
    std::unordered_multimap<uint64_t, PointLL> ids;
    for(const auto& edge : location.edges()) {
      try {
        //get the osm way id
        auto tile = reader.GetGraphTile(edge.id);
        auto* directed_edge = tile->directededge(edge.id);
        auto edge_info = tile->edgeinfo(directed_edge->edgeinfo_offset());
        //check if we did this one before
        auto range = ids.equal_range(edge_info->wayid());
        bool duplicate = false;
        for(auto id = range.first; id != range.second; ++id) {
          if(id->second == location.vertex()) {
            duplicate = true;
            break;
          }
        }
        //only serialize it if we didnt do it before
        if(!duplicate) {
          ids.emplace(edge_info->wayid(), location.vertex());
          //they want MOAR!
          if(verbose) {
            array->emplace_back(
              json::map({
                {"way_id", edge_info->wayid()},
                {"correlated_lat", json::fp_t{location.vertex().lat(), 6}},
                {"correlated_lon", json::fp_t{location.vertex().lng(), 6}}
              })
            );
          }//spare the details
          else {
            array->emplace_back(
              json::map({
                {"way_id", edge_info->wayid()},
                {"correlated_lat", json::fp_t{location.vertex().lat(), 6}},
                {"correlated_lon", json::fp_t{location.vertex().lng(), 6}}
              })
            );
          }
        }
      }
      catch(...) {
        //this really shouldnt ever get hit
        LOG_WARN("Expected edge not found in graph but found by loki::search!");
      }
    }
    return array;
  }

  json::MapPtr serialize(const PathLocation& location, GraphReader& reader, bool verbose) {
    return json::map({
      {"ways", serialize_edges(location, reader, verbose)},
      {"input_lat", json::fp_t{location.latlng_.lat(), 6}},
      {"input_lon", json::fp_t{location.latlng_.lng(), 6}}
    });
  }

  json::MapPtr serialize(const PointLL& ll, const std::string& reason) {
    return json::map({
      {"ways", static_cast<std::nullptr_t>(nullptr)},
      {"input_lat", json::fp_t{ll.lat(), 6}},
      {"input_lon", json::fp_t{ll.lng(), 6}},
      {"reason", reason}
    });
  }
}

namespace valhalla {
  namespace loki {

    worker_t::result_t loki_worker_t::locate(const boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
      //correlate the various locations to the underlying graph
      auto json = json::array({});
      for(const auto& location : locations) {
        try {
          auto correlated = loki::Search(location, reader, costing_filter);
          json->emplace_back(serialize(correlated, reader, request.get<bool>("verbose", false)));
        }
        catch(const std::exception& e) {
          json->emplace_back(serialize(location.latlng_, e.what()));
        }
      }

      //jsonp callback if need be
      std::ostringstream stream;
      auto jsonp = request.get_optional<std::string>("jsonp");
      if(jsonp)
        stream << *jsonp << '(';
      stream << *json;
      if(jsonp)
        stream << ')';

      worker_t::result_t result{false};
      http_response_t response(200, "OK", stream.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
      response.from_info(request_info);
      result.messages.emplace_back(response.to_string());
      return result;
    }
  }
}
