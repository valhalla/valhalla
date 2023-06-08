#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "thor/costmatrix.h"
#include "thor/timedistancebssmatrix.h"
#include "thor/timedistancematrix.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace valhalla {
namespace thor {

constexpr uint32_t kCostMatrixThreshold = 5;

std::string thor_worker_t::matrix(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  auto& options = *request.mutable_options();
  adjust_scores(options);
  auto costing = parse_costing(request);

  // Distance scaling (miles or km)
  double distance_scale = (options.units() == Options::miles) ? kMilePerMeter : kKmPerMeter;

  // lambdas to do the real work
  auto costmatrix = [&](const bool has_time) {
    return costmatrix_.SourceToTarget(*options.mutable_sources(), *options.mutable_targets(), *reader,
                                      mode_costing, mode, max_matrix_distance.find(costing)->second,
                                      has_time, options.date_time_type() == Options::invariant);
  };
  auto timedistancematrix = [&]() {
    return time_distance_matrix_.SourceToTarget(*options.mutable_sources(),
                                                *options.mutable_targets(), *reader, mode_costing,
                                                mode, max_matrix_distance.find(costing)->second,
                                                options.matrix_locations(),
                                                options.date_time_type() == Options::invariant);
  };

  if (costing == "bikeshare") {
    const auto& time_distances =
        time_distance_bss_matrix_.SourceToTarget(options.sources(), options.targets(), *reader,
                                                 mode_costing, mode,
                                                 max_matrix_distance.find(costing)->second,
                                                 options.matrix_locations());
    return tyr::serializeMatrix(request, time_distances, distance_scale, MatrixType::TimeDist);
  }

  MatrixType matrix_type = MatrixType::Cost;
  switch (source_to_target_algorithm) {
    case SELECT_OPTIMAL:
      // TODO - Do further performance testing to pick the best algorithm for the job
      switch (mode) {
        case travel_mode_t::kPedestrian:
        case travel_mode_t::kBicycle:
          // Use CostMatrix if number of sources and number of targets
          // exceeds some threshold
          if (options.sources().size() <= kCostMatrixThreshold ||
              options.targets().size() <= kCostMatrixThreshold) {
            matrix_type = MatrixType::TimeDist;
          }
          break;
        case travel_mode_t::kPublicTransit:
          matrix_type = MatrixType::TimeDist;
          break;
        default:
          break;
      }
      break;
    case COST_MATRIX:
      break;
    case TIME_DISTANCE_MATRIX:
      matrix_type = MatrixType::TimeDist;
      break;
  }

  // similar to routing: prefer the exact unidirectional algo if not requested otherwise
  // don't use matrix_type, we only need it to set the right warnings for what will be used
  bool has_time =
      check_matrix_time(request,
                        options.prioritize_bidirectional() ? MatrixType::Cost : MatrixType::TimeDist);
  if (has_time && !options.prioritize_bidirectional() && source_to_target_algorithm != COST_MATRIX) {
    return tyr::serializeMatrix(request, timedistancematrix(), distance_scale, MatrixType::TimeDist);
  } else if (has_time && options.prioritize_bidirectional() &&
             source_to_target_algorithm != TIME_DISTANCE_MATRIX) {
    return tyr::serializeMatrix(request, costmatrix(has_time), distance_scale, MatrixType::Cost);
  } else if (matrix_type == MatrixType::Cost) {
    // if this happens, the server config only allows for timedist matrix
    if (has_time && !options.prioritize_bidirectional()) {
      add_warning(request, 301);
    }
    return tyr::serializeMatrix(request, costmatrix(has_time), distance_scale, MatrixType::Cost);
  } else {
    if (has_time && options.prioritize_bidirectional()) {
      add_warning(request, 300);
    }
    return tyr::serializeMatrix(request, timedistancematrix(), distance_scale, MatrixType::TimeDist);
  }
}
} // namespace thor
} // namespace valhalla
