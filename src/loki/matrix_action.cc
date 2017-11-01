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

    void loki_worker_t::init_matrix(ACTION_TYPE action, rapidjson::Document& request) {
      //we require sources and targets
      try {
        sources = parse_locations(request, "sources", 131, valhalla_exception_t{112});
        targets = parse_locations(request, "targets", 132, valhalla_exception_t{112});
      }//deprecated using locations
      catch(const valhalla_exception_t& e) {
        if(request.HasMember("locations"))
          locations = parse_locations(request, "locations", 130, valhalla_exception_t{112});
        else
          throw e;
        if (locations.size() < 2)
          throw valhalla_exception_t{120};
        //create new sources and targets ptree from locations
        rapidjson::Value sources_child{rapidjson::kArrayType}, targets_child{rapidjson::kArrayType};
        auto request_locations = GetOptionalFromRapidJson<rapidjson::Value::Array>(request, "/locations");
        auto& allocator = request.GetAllocator();
        switch (action) {
          case ONE_TO_MANY:
            sources_child.PushBack(rapidjson::Value{*request_locations->Begin(), allocator}, allocator);
            request.AddMember("sources", sources_child, allocator);
            request.AddMember("targets", *request_locations, allocator);
            sources = { locations.front() };
            targets.swap(locations);
            break;
          case MANY_TO_ONE:
            targets_child.PushBack(rapidjson::Value{*(request_locations->End() - 1), allocator},allocator);
            request.AddMember("targets", targets_child, allocator);
            request.AddMember("sources", *request_locations, allocator);
            targets = { locations.back() };
            sources.swap(locations);
            break;
          case MANY_TO_MANY:
          case OPTIMIZED_ROUTE:
            request.AddMember("targets", rapidjson::Value{request["locations"], allocator}, allocator);
            request.AddMember("sources", *request_locations, allocator);
            targets = locations;
            sources.swap(locations);
            break;
        }
      }

      //sanitize
      if(sources.size() < 1) throw valhalla_exception_t{121};
      for(auto& s : sources) s.heading_.reset();
      if(targets.size() < 1) throw valhalla_exception_t{122};
      for(auto& t : targets) t.heading_.reset();

      //no locations!
      request.RemoveMember("locations");

      //need costing
      parse_costing(request);
    }

    void loki_worker_t::matrix(ACTION_TYPE action, rapidjson::Document& request) {
      init_matrix(action, request);
      std::string costing = request["costing"].GetString();
      if (costing == "multimodal")
        throw valhalla_exception_t{140, ACTION_TO_STRING.find(action)->second};

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
          rapidjson::Pointer("/correlated_" + std::to_string(i)).Set(request, projection.ToRapidJson(i, request.GetAllocator()));
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
      if (!healthcheck)
        valhalla::midgard::logging::Log("max_location_distance::" + std::to_string(max_location_distance * kKmPerMeter) + "km", " [ANALYTICS] ");
    }
  }
}
