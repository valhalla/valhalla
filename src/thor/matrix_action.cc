#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "thor/costmatrix.h"
#include "thor/timedistancematrix.h"
#include "thor/worker.h"
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

constexpr uint32_t kCostMatrixThreshold = 5;

std::string thor_worker_t::matrix(Api& request) {
  parse_locations(request);
  auto costing = parse_costing(request);
  const auto& options = request.options();

  if (!options.do_not_track()) {
    valhalla::midgard::logging::Log("matrix_type::" + Options_Action_Enum_Name(options.action()),
                                    " [ANALYTICS] ");
  }

  // Parse out units; if none specified, use kilometers
  double distance_scale = kKmPerMeter;
  if (options.units() == Options::miles) {
    distance_scale = kMilePerMeter;
  }

  json::MapPtr json;
  // do the real work
  std::vector<TimeDistance> time_distances;
  auto costmatrix = [&]() {
    thor::CostMatrix matrix;
    return matrix.SourceToTarget(options.sources(), options.targets(), *reader, mode_costing, mode,
                                 max_matrix_distance.find(costing)->second);
  };
  auto timedistancematrix = [&]() {
    thor::TimeDistanceMatrix matrix;
    return matrix.SourceToTarget(options.sources(), options.targets(), *reader, mode_costing, mode,
                                 max_matrix_distance.find(costing)->second);
  };
  switch (source_to_target_algorithm) {
    case SELECT_OPTIMAL:
      // TODO - Do further performance testing to pick the best algorithm for the job
      switch (mode) {
        case TravelMode::kPedestrian:
        case TravelMode::kBicycle:
          // Use CostMatrix if number of sources and number of targets
          // exceeds some threshold
          if (options.sources().size() > kCostMatrixThreshold &&
              options.targets().size() > kCostMatrixThreshold) {
            time_distances = costmatrix();
          } else {
            time_distances = timedistancematrix();
          }
          break;
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
  return tyr::serializeMatrix(request, time_distances, distance_scale);
}
} // namespace thor
} // namespace valhalla
