#include <cstdint>

#include "baldr/json.h"
#include "thor/costmatrix.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::thor;


namespace {

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

  json::MapPtr serialize(const odin::DirectionsOptions& options, const std::vector<PathLocation>& sources,
      const std::vector<PathLocation>& targets, const std::vector<TimeDistance>& time_distances, double distance_scale) {
    json::ArrayPtr matrix = json::array({});
    for(size_t source_index = 0; source_index < sources.size(); ++source_index) {
        matrix->emplace_back(
          serialize_row(time_distances, source_index * targets.size(), targets.size(),
                        source_index, 0, distance_scale));
    }
    auto json = json::map({
      {"sources_to_targets", matrix},
      {"units", odin::DirectionsOptions::Units_Name(options.units())},
    });
    json->emplace("targets", json::array({locations(targets)}));
    json->emplace("sources", json::array({locations(sources)}));

    if (options.has_id())
      json->emplace("id", options.id());
    return json;
  }

  json::MapPtr serializeOSRM(const odin::DirectionsOptions& options, const std::vector<PathLocation>& sources,
      const std::vector<PathLocation>& targets, const std::vector<TimeDistance>& time_distances, double distance_scale) {
    //TODO:
    return {};
  }

}

namespace valhalla {
  namespace tyr {

    std::string serializeMatrix(const valhalla_request_t& request, const std::vector<PathLocation>& sources,
        const std::vector<PathLocation>& targets, const std::vector<TimeDistance>& time_distances, double distance_scale) {

      auto json = request.options.format() == odin::DirectionsOptions::osrm ?
          serializeOSRM(request.options, sources, targets, time_distances, distance_scale) :
          serialize(request.options, sources, targets, time_distances, distance_scale);

      std::stringstream ss;
      ss << *json;
      return ss.str();
    }

  }
}
