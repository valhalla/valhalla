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

  json::ArrayPtr serialize_edges(const PathLocation& location, GraphReader& reader, bool verbose) {
    auto array = json::array({});
    for(const auto& edge : location.edges()) {
      try {
        //get the osm way id
        auto tile = reader.GetGraphTile(edge.id);
        auto* directed_edge = tile->directededge(edge.id);
        auto edge_info = tile->edgeinfo(directed_edge->edgeinfo_offset());
        //they want MOAR!
        if(verbose) {
          array->emplace_back(
            json::map({
              {"correlated_lat", json::fp_t{location.vertex().lat(), 6}},
              {"correlated_lon", json::fp_t{location.vertex().lng(), 6}},
              {"side_of_street",
                edge.sos == PathLocation::LEFT ? std::string("left") :
                  (edge.sos == PathLocation::RIGHT ? std::string("right") : std::string("neither"))
              },
              {"percent_along", json::fp_t{edge.dist, 5} },
              {"edge_id", edge.id.json()},
              {"edge", directed_edge->json()},
              {"edge_info", edge_info->json()},
            })
          );
        }//they want it lean and mean
        else {
          array->emplace_back(
            json::map({
              {"way_id", edge_info->wayid()},
              {"correlated_lat", json::fp_t{location.vertex().lat(), 6}},
              {"correlated_lon", json::fp_t{location.vertex().lng(), 6}},
              {"side_of_street",
                edge.sos == PathLocation::LEFT ? std::string("left") :
                  (edge.sos == PathLocation::RIGHT ? std::string("right") : std::string("neither"))
              },
              {"percent_along", json::fp_t{edge.dist, 5} },
            })
          );
        }
      }
      catch(...) {
        //this really shouldnt ever get hit
        LOG_WARN("Expected edge not found in graph but found by loki::search!");
      }
    }
    return array;
  }

  json::MapPtr serialize(const boost::optional<std::string>& id, const PathLocation& location, GraphReader& reader, bool verbose) {
    //serialze all the edges
    auto m = json::map
    ({
      {"edges", serialize_edges(location, reader, verbose)},
      {"input_lat", json::fp_t{location.latlng_.lat(), 6}},
      {"input_lon", json::fp_t{location.latlng_.lng(), 6}},
    });

    //serialize the node
    GraphId node;
    if(location.IsNode()) {
      for(const auto& e : location.edges()) {
        if(e.dist == 1.f) {
          node = reader.GetGraphTile(e.id)->directededge(e.id)->endnode();
          const GraphTile* tile = reader.GetGraphTile(node);
          auto* node_info = tile->node(node);
          if(verbose)
            m->emplace("node", node_info->json(tile));
          else {
            m->emplace("node", json::map({
              {"lon", json::fp_t{node_info->latlng().first, 6}},
              {"lat", json::fp_t{node_info->latlng().second, 6}},
              //TODO: osm_id
            }));
          }
          break;
        }
      }
    }//no node
    else
      m->emplace("node", static_cast<nullptr_t>(nullptr));
    if(verbose)
      m->emplace("node_id", node.json());
    /*if (id)
      m->emplace("id", *id);*/

    return m;
  }

  json::MapPtr serialize(const boost::optional<std::string>& id, const PointLL& ll, const std::string& reason, bool verbose) {
    auto m = json::map({
      {"edges", static_cast<std::nullptr_t>(nullptr)},
      {"node", static_cast<std::nullptr_t>(nullptr)},
      {"input_lat", json::fp_t{ll.lat(), 6}},
      {"input_lon", json::fp_t{ll.lng(), 6}},
    });
    if(verbose)
      m->emplace("reason", reason);
    /*if(id)
      m->emplace("id", *id);*/

    return m;
  }
}

namespace valhalla {
  namespace loki {

    worker_t::result_t loki_worker_t::locate(const boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
      //correlate the various locations to the underlying graph
      auto json = json::array({});
      auto verbose = request.get<bool>("verbose", false);

      for(const auto& location : locations) {
        try {
          auto correlated = loki::Search(location, reader, costing_filter);
          json->emplace_back(serialize(request.get_optional<std::string>("id"), correlated, reader, verbose));
        }
        catch(const std::exception& e) {
          json->emplace_back(serialize(request.get_optional<std::string>("id"), location.latlng_, e.what(), verbose));
        }
      }

      std::ostringstream stream;
      //jsonp callback if need be
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
