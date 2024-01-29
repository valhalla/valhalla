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

MatrixAlgorithm* thor_worker_t::get_matrix_algorithm(Api& request, const bool has_time) {

  Matrix::Algorithm config_algo = Matrix::CostMatrix;
  switch (source_to_target_algorithm) {
    case SELECT_OPTIMAL:
      // TODO - Do further performance testing to pick the best algorithm for the job
      switch (mode) {
        case travel_mode_t::kPedestrian:
        case travel_mode_t::kBicycle:
          // Use CostMatrix if number of sources and number of targets
          // exceeds some threshold
          if (static_cast<uint32_t>(request.options().sources().size()) <= kCostMatrixThreshold ||
              static_cast<uint32_t>(request.options().targets().size()) <= kCostMatrixThreshold) {
            config_algo = Matrix::TimeDistanceMatrix;
          }
          break;
        case travel_mode_t::kPublicTransit:
          config_algo = Matrix::TimeDistanceMatrix;
          break;
        default:
          break;
      }
      break;
    case COST_MATRIX:
      break;
    case TIME_DISTANCE_MATRIX:
      config_algo = Matrix::TimeDistanceMatrix;
      break;
  }

  // similar to routing: prefer the exact unidirectional algo if not requested otherwise
  // don't use matrix_type, we only need it to set the right warnings for what will be used
  if (has_time && !request.options().prioritize_bidirectional() &&
      source_to_target_algorithm != COST_MATRIX) {
    return &time_distance_matrix_;
  } else if (has_time && request.options().prioritize_bidirectional() &&
             source_to_target_algorithm != TIME_DISTANCE_MATRIX) {
    return &costmatrix_;
  } else if (config_algo == Matrix::CostMatrix) {
    if (has_time && !request.options().prioritize_bidirectional()) {
      add_warning(request, 301);
    }
    return &costmatrix_;
  } else {
    // if this happens, the server config only allows for timedist matrix
    if (has_time && request.options().prioritize_bidirectional()) {
      add_warning(request, 300);
    }
    return &time_distance_matrix_;
  }
}

std::string thor_worker_t::matrix(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  auto& options = *request.mutable_options();
  adjust_scores(options);
  auto costing = parse_costing(request);

  bool has_time =
      check_matrix_time(request, options.prioritize_bidirectional() ? Matrix::CostMatrix
                                                                    : Matrix::TimeDistanceMatrix);

  // allow all algos to be cancelled
  for (auto* alg : std::vector<MatrixAlgorithm*>{
           &costmatrix_,
           &time_distance_matrix_,
           &time_distance_bss_matrix_,
       }) {
    alg->set_interrupt(interrupt);
    alg->set_has_time(has_time);
  }

  auto* algo =
      costing == "bikeshare" ? &time_distance_bss_matrix_ : get_matrix_algorithm(request, has_time);

  valhalla::sif::cost_ptr_t cost = mode_costing[static_cast<uint32_t>(mode)];
  cost->set_allow_destination_only(false);
  cost->set_pass(0);

  if (!algo->SourceToTarget(request, *reader, mode_costing, mode,
                            max_matrix_distance.find(costing)->second) &&
      cost->AllowMultiPass()) {
    // TODO(nils): probably add filtered edges here too?
    algo->Clear();
    cost->set_pass(1);
    cost->RelaxHierarchyLimits(true);
    cost->set_allow_destination_only(true);
    cost->set_allow_conditional_destination(true);
    algo->set_not_thru_pruning(false);
    // expand a second time with only the ones that matter
    algo->SourceToTarget(request, *reader, mode_costing, mode,
                         max_matrix_distance.find(costing)->second);
  };

  return tyr::serializeMatrix(request);
}
} // namespace thor
} // namespace valhalla
