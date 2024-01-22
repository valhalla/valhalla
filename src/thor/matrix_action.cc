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

namespace {
constexpr uint32_t kCostMatrixThreshold = 5;
}

namespace valhalla {
namespace thor {

std::string thor_worker_t::matrix(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  auto& options = *request.mutable_options();
  adjust_scores(options);
  auto costing = parse_costing(request);

  // TODO: do this for others as well
  costmatrix_.set_interrupt(interrupt);

  // lambdas to do the real work
  auto costmatrix = [&](const bool has_time) {
    return costmatrix_.SourceToTarget(request, *reader, mode_costing, mode,
                                      max_matrix_distance.find(costing)->second, has_time,
                                      options.date_time_type() == Options::invariant,
                                      options.shape_format());
  };
  auto timedistancematrix = [&]() {
    if (options.shape_format() != no_shape)
      add_warning(request, 207);
    return time_distance_matrix_.SourceToTarget(request, *reader, mode_costing, mode,
                                                max_matrix_distance.find(costing)->second,
                                                options.matrix_locations(),
                                                options.date_time_type() == Options::invariant);
  };

  if (costing == "bikeshare") {
    if (options.shape_format() != no_shape)
      add_warning(request, 207);
    time_distance_bss_matrix_.SourceToTarget(request, *reader, mode_costing, mode,
                                             max_matrix_distance.find(costing)->second,
                                             options.matrix_locations());
    return tyr::serializeMatrix(request);
  }

  Matrix::Algorithm matrix_algo = Matrix::CostMatrix;
  switch (source_to_target_algorithm) {
    case SELECT_OPTIMAL:
      // TODO - Do further performance testing to pick the best algorithm for the job
      switch (mode) {
        case travel_mode_t::kPedestrian:
        case travel_mode_t::kBicycle:
          // Use CostMatrix if number of sources and number of targets
          // exceeds some threshold
          if (static_cast<uint32_t>(options.sources().size()) <= kCostMatrixThreshold ||
              static_cast<uint32_t>(options.targets().size()) <= kCostMatrixThreshold) {
            matrix_algo = Matrix::TimeDistanceMatrix;
          }
          break;
        case travel_mode_t::kPublicTransit:
          matrix_algo = Matrix::TimeDistanceMatrix;
          break;
        default:
          break;
      }
      break;
    case COST_MATRIX:
      break;
    case TIME_DISTANCE_MATRIX:
      matrix_algo = Matrix::TimeDistanceMatrix;
      break;
  }

  // similar to routing: prefer the exact unidirectional algo if not requested otherwise
  // don't use matrix_type, we only need it to set the right warnings for what will be used
  bool has_time =
      check_matrix_time(request, options.prioritize_bidirectional() ? Matrix::CostMatrix
                                                                    : Matrix::TimeDistanceMatrix);
  if (has_time && !options.prioritize_bidirectional() && source_to_target_algorithm != COST_MATRIX) {
    timedistancematrix();
  } else if (has_time && options.prioritize_bidirectional() &&
             source_to_target_algorithm != TIME_DISTANCE_MATRIX) {
    costmatrix(has_time);
  } else if (matrix_algo == Matrix::CostMatrix) {
    // if this happens, the server config only allows for timedist matrix
    if (has_time && !options.prioritize_bidirectional()) {
      add_warning(request, 301);
    }
    costmatrix(has_time);
  } else {
    if (has_time && options.prioritize_bidirectional()) {
      add_warning(request, 300);
    }
    timedistancematrix();
  }
  return tyr::serializeMatrix(request);
}
} // namespace thor
} // namespace valhalla
