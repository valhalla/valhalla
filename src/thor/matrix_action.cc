#include "thor/worker.h"
#include <cstdint>

#include "midgard/logging.h"
#include "midgard/constants.h"
#include "baldr/json.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "thor/costmatrix.h"
#include "thor/timedistancematrix.h"
#include "tyr/actor.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

  constexpr double kMilePerMeter = 0.000621371;
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

  json::MapPtr serialize(const std::string action, const boost::optional<std::string>& id, const std::vector<PathLocation>& correlated_s, const std::vector<PathLocation>& correlated_t, const std::vector<TimeDistance>& tds, std::string& units, double distance_scale) {
    json::ArrayPtr matrix = json::array({});
    for(size_t source_index = 0; source_index < correlated_s.size(); ++source_index) {
        matrix->emplace_back(
          serialize_row(tds, source_index * correlated_t.size(), correlated_t.size(),
                        source_index, action == "many_to_one" ? correlated_s.size()-1 : 0, distance_scale));
    }
    auto json = json::map({
      {action, matrix},
      {"units", units},
    });
    if (action == "sources_to_targets") {
      json->emplace("targets", json::array({locations(correlated_t)}));
      json->emplace("sources", json::array({locations(correlated_s)}));
    } else {
      json->emplace("locations", json::array({locations(correlated_s.size() > correlated_t.size() ? correlated_s : correlated_t)}));
    }
    if (id)
      json->emplace("id", *id);
    return json;
  }

}

namespace valhalla {
  namespace thor {

    json::MapPtr thor_worker_t::matrix(ACTION_TYPE action, const boost::property_tree::ptree &request) {
      parse_locations(request);
      auto costing = parse_costing(request);

      const auto& matrix_type = ACTION_TO_STRING.find(action)->second;
      if (!healthcheck)
        valhalla::midgard::logging::Log("matrix_type::" + matrix_type, " [ANALYTICS] ");

      // Parse out units; if none specified, use kilometers
      double distance_scale = kKmPerMeter;
      auto units = request.get<std::string>("units", "km");
      if (units == "mi")
        distance_scale = kMilePerMeter;

      json::MapPtr json;
      //do the real work
      std::vector<TimeDistance> time_distances;
      auto costmatrix = [&]() {
        thor::CostMatrix matrix;
        return matrix.SourceToTarget(correlated_s, correlated_t, reader, mode_costing,
                                    mode, max_matrix_distance.find(costing)->second);
      };
      auto timedistancematrix = [&]() {
        thor::TimeDistanceMatrix matrix;
        return matrix.SourceToTarget(correlated_s, correlated_t, reader, mode_costing,
                                    mode, max_matrix_distance.find(costing)->second);
      };
      switch (source_to_target_algorithm) {
        case SELECT_OPTIMAL:
          //TODO - Do further performance testing to pick the best algorithm for the job
          switch (mode) {
            case TravelMode::kPedestrian:
            case TravelMode::kBicycle:
            case TravelMode::kPublicTransit:
              time_distances = timedistancematrix();
              break;
            default:
              time_distances = costmatrix();
          }
          break;
        case COST_MATRIX:
          time_distances = costmatrix();
          break;
        case TIME_DISTANCE_MATRIX:
          time_distances = timedistancematrix();
          break;
      }
      json = serialize(matrix_type, request.get_optional<std::string>("id"), correlated_s, correlated_t,
        time_distances, units, distance_scale);

      return json;
    }
  }
}
