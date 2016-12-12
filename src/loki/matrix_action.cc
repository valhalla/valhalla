#include "loki/service.h"
#include "loki/search.h"

#include <boost/property_tree/info_parser.hpp>
#include <unordered_map>

#include <valhalla/baldr/datetime.h>
#include <valhalla/midgard/logging.h>

using namespace prime_server;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace std {
  template <>
  struct hash<loki_worker_t::ACTION_TYPE>
  {
    std::size_t operator()(const loki_worker_t::ACTION_TYPE& a) const {
      return std::hash<int>()(a);
    }
  };
}

namespace {

  // TODO: Separate matrix actions to be deprecated and replaced by sources_to_targets action
  const std::unordered_map<loki_worker_t::ACTION_TYPE, std::string> ACTION_TO_STRING {
     {loki_worker_t::ONE_TO_MANY, "one_to_many"},
     {loki_worker_t::MANY_TO_ONE, "many_to_one"},
     {loki_worker_t::MANY_TO_MANY, "many_to_many"},
     {loki_worker_t::SOURCES_TO_TARGETS, "sources_to_targets"},
     {loki_worker_t::OPTIMIZED_ROUTE, "optimized_route"}
   };

  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

  void check_distance(const std::vector<Location>& sources, const std::vector<Location>& targets, float matrix_max_distance, float& max_location_distance) {

    //see if any locations pairs are unreachable or too far apart
    for(const auto& source : sources){
      for(const auto& target : targets) {
        //check if distance between latlngs exceed max distance limit
        auto path_distance = source.latlng_.Distance(target.latlng_);

        //only want to log the maximum distance between 2 locations for matrix
        LOG_DEBUG("path_distance -> " + std::to_string(path_distance));
        if (path_distance >= max_location_distance) {
          max_location_distance = path_distance;
          LOG_DEBUG("max_location_distance -> " + std::to_string(max_location_distance));
        }

        if (path_distance > matrix_max_distance)
          throw valhalla_exception_t{400, 154};
        }
     }
  }
}

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_matrix(ACTION_TYPE action, boost::property_tree::ptree& request) {
      auto request_locations = request.get_child_optional("locations");
      auto request_sources = request.get_child_optional("sources");
      auto request_targets = request.get_child_optional("targets");

      //we require locations
      if (!request_locations) {
        if (!request_sources || !request_targets) {
          throw valhalla_exception_t{400, 112};
        }
      }

      //if MATRIX OR OPTIMIZED and not using sources & targets parameters
      //deprecated way of specifying
      if (!request_sources && !request_targets) {
        if (request_locations->size() < 2)
          throw valhalla_exception_t{400, 120};

        //create new sources and targets ptree from locations
        boost::property_tree::ptree sources_child, targets_child;
        switch (action) {
          case ONE_TO_MANY:
            sources_child.push_back(request_locations->front());
            for(const auto& reqloc : *request_locations)
              targets_child.push_back(reqloc);

            break;
          case MANY_TO_ONE:
            for(const auto& reqloc : *request_locations)
              sources_child.push_back(reqloc);

            targets_child.push_back(request_locations->back());
            break;
          case MANY_TO_MANY:
          case OPTIMIZED_ROUTE:
            for(const auto& reqloc : *request_locations) {
              sources_child.push_back(reqloc);
              targets_child.push_back(reqloc);
            }
            break;
        }
        //add these back in the original request (in addition to locations while being deprecated
        request.add_child("sources", sources_child);
        request.add_child("targets", targets_child);
        request_sources = request.get_child("sources");
        request_targets = request.get_child("targets");

      }
      for(const auto& source : *request_sources) {
        try{
          sources.push_back(baldr::Location::FromPtree(source.second));
          sources.back().heading_.reset();
        }
        catch (...) {
          throw valhalla_exception_t{400, 131};
        }
      }
      for(const auto& target : *request_targets) {
        try{
          targets.push_back(baldr::Location::FromPtree(target.second));
          targets.back().heading_.reset();
        }
        catch (...) {
          throw valhalla_exception_t{400, 132};
        }
      }
      if(sources.size() < 1)
        throw valhalla_exception_t{400, 121};
      valhalla::midgard::logging::Log("source_count::" + std::to_string(request_sources->size()), " [ANALYTICS] ");

      if(targets.size() < 1)
        throw valhalla_exception_t{400, 122};
      valhalla::midgard::logging::Log("target_count::" + std::to_string(request_targets->size()), " [ANALYTICS] ");

      //no locations!
      request.erase("locations");

      parse_costing(request);
    }

    worker_t::result_t loki_worker_t::matrix(ACTION_TYPE action,boost::property_tree::ptree& request, http_request_info_t& request_info) {
      init_matrix(action, request);
      auto costing = request.get<std::string>("costing");
      if (costing == "multimodal")
        return jsonify_error({400, 140, ACTION_TO_STRING.find(action)->second}, request_info);

      //check that location size does not exceed max.
      auto max = max_locations.find("sources_to_targets")->second;
      if (sources.size() > max || targets.size() > max)
        throw valhalla_exception_t{400, 150, std::to_string(max)};

      //check the distances
      auto max_location_distance = std::numeric_limits<float>::min();
      check_distance(sources, targets, max_distance.find("sources_to_targets")->second, max_location_distance);

      //correlate the various locations to the underlying graph
      std::vector<baldr::Location> sources_targets;
      std::move(sources.begin(), sources.end(), std::back_inserter(sources_targets));
      std::move(targets.begin(), targets.end(), std::back_inserter(sources_targets));

      //correlate the various locations to the underlying graph
      std::unordered_map<size_t, size_t> color_counts;
      try{
        const auto searched = loki::Search(sources_targets, reader, edge_filter, node_filter);
        for(size_t i = 0; i < sources_targets.size(); ++i) {
          const auto& l = sources_targets[i];
          const auto& projection = searched.at(l);
          request.put_child("correlated_" + std::to_string(i), projection.ToPtree(i));

          //TODO: get transit level for transit costing
          //TODO: if transit send a non zero radius
          auto colors = connectivity_map.get_colors(reader.GetTileHierarchy().levels().rbegin()->first, projection, 0);
          for(auto& color : colors){
            auto itr = color_counts.find(color);
            if(itr == color_counts.cend())
              color_counts[color] = 1;
            else
              ++itr->second;
          }
        }
      }
      catch(const std::exception&) {
        throw valhalla_exception_t{400, 171};
      }


      //are all the locations in the same color regions
      bool connected = false;
      for(const auto& c : color_counts) {
        if(c.second == sources_targets.size()) {
          connected = true;
          break;
        }
      }
      if(!connected)
        throw valhalla_exception_t{400, 170};
      valhalla::midgard::logging::Log("max_location_distance::" + std::to_string(max_location_distance * kKmPerMeter) + "km", " [ANALYTICS] ");

      std::stringstream stream;
      boost::property_tree::write_json(stream, request, false);
      worker_t::result_t result{true};
      result.messages.emplace_back(stream.str());

      return result;
    }
  }
}
