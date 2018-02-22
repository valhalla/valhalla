#include "thor/worker.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "thor/costmatrix.h"
#include "thor/timedistancematrix.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

  constexpr double kMilePerMeter = 0.000621371;

}

namespace valhalla {
  namespace thor {

    std::string thor_worker_t::matrix(ACTION_TYPE action, const rapidjson::Document& request) {
      parse_locations(request);
      auto costing = parse_costing(request);

      const auto& matrix_type = ACTION_TO_STRING.find(action)->second;
      if (!healthcheck)
        valhalla::midgard::logging::Log("matrix_type::" + matrix_type, " [ANALYTICS] ");

      // Parse out units; if none specified, use kilometers
      double distance_scale = kKmPerMeter;
      if (options.units() == odin::DirectionsOptions::miles)
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
      return tyr::serializeMatrix(options, correlated_s, correlated_t, time_distances, distance_scale);
    }
  }
}
