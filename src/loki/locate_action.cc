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
    for(const auto& edge : location.edges) {
      try {
        //get the osm way id
        auto tile = reader.GetGraphTile(edge.id);
        auto* directed_edge = tile->directededge(edge.id);
        auto edge_info = tile->edgeinfo(directed_edge->edgeinfo_offset());
        //they want MOAR!
        if(verbose) {
          array->emplace_back(
            json::map({
              {"correlated_lat", json::fp_t{edge.projected.lat(), 6}},
              {"correlated_lon", json::fp_t{edge.projected.lng(), 6}},
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
              {"correlated_lat", json::fp_t{edge.projected.lat(), 6}},
              {"correlated_lon", json::fp_t{edge.projected.lng(), 6}},
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

  json::ArrayPtr serialize_nodes(const PathLocation& location, GraphReader& reader, bool verbose) {
    //get the nodes we need
    std::unordered_set<uint64_t> nodes;
    for(const auto& e : location.edges)
      if(e.end_node())
        nodes.emplace(reader.GetGraphTile(e.id)->directededge(e.id)->endnode());
    //ad them into an array of json
    auto array = json::array({});
    for(auto node_id : nodes) {
      GraphId n(node_id);
      const GraphTile* tile = reader.GetGraphTile(n);
      auto* node_info = tile->node(n);
      json::MapPtr node;
      if(verbose) {
        node = node_info->json(tile);
        node->emplace("node_id", n.json());
      }
      else {
        node = json::map({
          {"lon", json::fp_t{node_info->latlng().first, 6}},
          {"lat", json::fp_t{node_info->latlng().second, 6}},
          //TODO: osm_id
        });
      }
      array->emplace_back(node);
    }
    //give them back
    return array;
  }

  json::MapPtr serialize(const boost::optional<std::string>& id, const PathLocation& location, GraphReader& reader, bool verbose) {
    //serialze all the edges
    auto m = json::map
    ({
      {"edges", serialize_edges(location, reader, verbose)},
      {"nodes", serialize_nodes(location, reader, verbose)},
      {"input_lat", json::fp_t{location.latlng_.lat(), 6}},
      {"input_lon", json::fp_t{location.latlng_.lng(), 6}},
    });
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

    void loki_worker_t::init_locate(const ACTION_TYPE& action,  boost::property_tree::ptree& request) {
      //we require locations
      auto request_locations = request.get_child_optional("locations");

      if (!request_locations)
        throw std::runtime_error("Insufficiently specified required parameter 'locations'");

      for(const auto& location : *request_locations) {
        try{
          locations.push_back(baldr::Location::FromPtree(location.second));
        }
        catch (...) {
          throw std::runtime_error("Failed to parse location");
        }
      }
      if(locations.size() < 1)
        throw std::runtime_error("Insufficient number of locations provided");

      valhalla::midgard::logging::Log("location_count::" + std::to_string(request_locations->size()), " [ANALYTICS] ");

      //using the costing we can determine what type of edge filtering to use
      auto costing = request.get_optional<std::string>("costing");
      if (costing)
        valhalla::midgard::logging::Log("costing_type::" + *costing, " [ANALYTICS] ");
      else {
        //locate doesnt require a filter
        edge_filter = loki::PassThroughEdgeFilter;
        node_filter = loki::PassThroughNodeFilter;
        return;
      }

      // TODO - have a way of specifying mode at the location
      if(*costing == "multimodal")
        *costing = "pedestrian";

      // Get the costing options. Get the base options from the config and the
      // options for the specified costing method
      std::string method_options = "costing_options." + *costing;
      auto config_costing = config.get_child_optional(method_options);
      if(!config_costing)
        throw std::runtime_error("No costing method found for '" + *costing + "'");
      auto request_costing = request.get_child_optional(method_options);
      if(request_costing) {
        // If the request has any options for this costing type, merge the 2
        // costing options - override any config options that are in the request.
        // and add any request options not in the config.
        // TODO: suboptions are probably getting smashed when we do this, preserve them
        boost::property_tree::ptree overridden = *config_costing;
        for(const auto& r : *request_costing)
          overridden.put_child(r.first, r.second);
        auto c = factory.Create(*costing, overridden);
        edge_filter = c->GetEdgeFilter();
        node_filter = c->GetNodeFilter();
      }// No options to override so use the config options verbatim
      else {
        auto c = factory.Create(*costing, *config_costing);
        edge_filter = c->GetEdgeFilter();
        node_filter = c->GetNodeFilter();
      }
    }

    worker_t::result_t loki_worker_t::locate(const boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
      //correlate the various locations to the underlying graph
      auto json = json::array({});
      auto verbose = request.get<bool>("verbose", false);

      for(const auto& location : locations) {
        try {
          auto correlated = loki::Search(location, reader, edge_filter, node_filter);
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
