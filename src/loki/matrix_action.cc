#include "loki/worker.h"
#include "loki/search.h"

#include <boost/property_tree/info_parser.hpp>
#include <unordered_map>

#include "baldr/tilehierarchy.h"
#include "baldr/datetime.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "tyr/actor.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace {
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
          throw valhalla_exception_t{154};
      }
    }
  }
}

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_matrix(valhalla_request_t& request) {
      //we require sources and targets
      if(request.options.action() == odin::DirectionsOptions::sources_to_targets) {
        sources = parse_locations(request, "sources", 131, valhalla_exception_t{112});
        targets = parse_locations(request, "targets", 132, valhalla_exception_t{112});
      }//optimized route uses locations but needs to do a matrix
      else {
        locations = parse_locations(request, "locations", 130, valhalla_exception_t{112});
        if (locations.size() < 2)
          throw valhalla_exception_t{120};

        //create new sources and targets ptree from locations
        rapidjson::Value sources_child{rapidjson::kArrayType}, targets_child{rapidjson::kArrayType};
        auto request_locations = rapidjson::get_optional<rapidjson::Value::Array>(request.document, "/locations");
        auto& allocator = request.document.GetAllocator();
        request.document.AddMember("targets", rapidjson::Value{request.document["locations"], allocator}, allocator);
        request.document.AddMember("sources", *request_locations, allocator);
        targets = locations;
        sources.swap(locations);
      }

      //sanitize
      if(sources.size() < 1) throw valhalla_exception_t{121};
      for(auto& s : sources) s.heading_.reset();
      if(targets.size() < 1) throw valhalla_exception_t{122};
      for(auto& t : targets) t.heading_.reset();

      //no locations!
      request.document.RemoveMember("locations");

      //need costing
      parse_costing(request);
    }

    void loki_worker_t::matrix(valhalla_request_t& request) {
      init_matrix(request);
      std::string costing = request.document["costing"].GetString();
      if (costing == "multimodal")
        throw valhalla_exception_t{140, odin::DirectionsOptions::Action_Name(request.options.action())};

      //check that location size does not exceed max.
      auto max = max_matrix_locations.find(costing)->second;
      if (sources.size() > max || targets.size() > max)
        throw valhalla_exception_t{150, std::to_string(max)};

      //check the distances
      auto max_location_distance = std::numeric_limits<float>::min();
      check_distance(sources, targets, max_matrix_distance.find(costing)->second, max_location_distance);

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
          //TODO: remove this when using pbf everywhere
          rapidjson::Pointer("/correlated_" + std::to_string(i)).
              Set(request.document, projection.ToRapidJson(i, request.document.GetAllocator()));
          toPBF(projection, i < sources.size() ?
              request.options.mutable_sources()->Add() :
              request.options.mutable_targets()->Add());
          //TODO: get transit level for transit costing
          //TODO: if transit send a non zero radius
          if (!connectivity_map)
            continue;
          auto colors = connectivity_map->get_colors(TileHierarchy::levels().rbegin()->first, projection, 0);
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
        throw valhalla_exception_t{171};
      }

      //are all the locations in the same color regions
      if (!connectivity_map)
        return;
      bool connected = false;
      for(const auto& c : color_counts) {
        if(c.second == sources_targets.size()) {
          connected = true;
          break;
        }
      }
      if(!connected)
        throw valhalla_exception_t{170};
      if (!request.options.do_not_track())
        valhalla::midgard::logging::Log("max_location_distance::" + std::to_string(max_location_distance * kKmPerMeter) + "km", " [ANALYTICS] ");
    }
  }
}
