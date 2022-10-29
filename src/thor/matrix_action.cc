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

// return true if any location had a time set
// also disable time if it doesn't make sense computationally
bool has_time(Api& request) {
  auto& options = request.options();
  bool less_sources = options.sources().size() <= options.targets().size();
  bool had_valid_time = false;
  for (const auto& loc : options.sources()) {
    if (!loc.date_time().empty()) {
      if (!less_sources) {
        add_warning(request, 401);
        return false;
      }
      had_valid_time = true;
      break;
    }
  }
  for (const auto& loc : options.targets()) {
    if (!loc.date_time().empty()) {
      if (less_sources) {
        add_warning(request, 402);
        return false;
      }
      had_valid_time = true;
      break;
    }
  }

  return had_valid_time;
}
} // namespace

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
  std::vector<TimeDistance> time_distances;
  auto costmatrix = [&]() {
    return costmatrix_.SourceToTarget(options.sources(), options.targets(), *reader, mode_costing,
                                      mode, max_matrix_distance.find(costing)->second);
  };
  auto timedistancematrix = [&]() {
    return time_distance_matrix_.SourceToTarget(*options.mutable_sources(),
                                                *options.mutable_targets(), *reader, mode_costing,
                                                mode, max_matrix_distance.find(costing)->second,
                                                options.matrix_locations(),
                                                options.date_time_type() == Options::invariant);
  };

  if (costing == "bikeshare") {
    time_distances =
        time_distance_bss_matrix_.SourceToTarget(options.sources(), options.targets(), *reader,
                                                 mode_costing, mode,
                                                 max_matrix_distance.find(costing)->second,
                                                 options.matrix_locations());
    return tyr::serializeMatrix(request, time_distances, distance_scale);
  }
  switch (source_to_target_algorithm) {
    case SELECT_OPTIMAL:
      // TODO - Do further performance testing to pick the best algorithm for the job
      switch (mode) {
        case travel_mode_t::kPedestrian:
        case travel_mode_t::kBicycle:
          // Use CostMatrix if number of sources and number of targets
          // exceeds some threshold
          if (options.sources().size() > kCostMatrixThreshold &&
              options.targets().size() > kCostMatrixThreshold) {
            time_distances = costmatrix();
          } else {
            time_distances = timedistancematrix();
          }
          break;
        case travel_mode_t::kPublicTransit:
          time_distances = timedistancematrix();
          break;
        default:
          // force timedistance if traffic is desired and allowed
          if (has_time(request)) {
            time_distances = timedistancematrix();
          } else {
            time_distances = costmatrix();
          }
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
