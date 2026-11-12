#include "loki/search.h"
#include "loki/worker.h"

#include <algorithm>
#include <cmath>
#include <format>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace {

void parse_round_trip_options(Api& request, double max_distance) {
  auto* options = request.mutable_options()->mutable_round_trip_options();
  request.mutable_options()->mutable_locations(0)->set_round_trip_group_index(
      0); // overwrite if the user tried to set it

  // nothing or negative value set, throw
  if (options->distance() <= 0.0) {
    throw valhalla_exception_t{178};
  }

  // clamp distance
  if (options->distance() > max_distance) {
    options->set_distance(max_distance);
    add_warning(request, 304, std::format("{:.1f}", max_distance));
  }

  if (!options->has_heading()) {
    options->set_heading(0.0); // todo: randomly using seed
  } else if (options->heading() >= 360. || options->heading() < 0.0) {
    auto wrapped = std::fmod(options->heading(), 360.0);
    options->set_heading(wrapped < 0.0 ? wrapped + 360.0 : wrapped);
  }

  constexpr double kMinHeadingTolerance = 10.;
  constexpr double kMaxHeadingTolerance = 180.;

  if (!options->has_heading_tolerance()) {
    options->set_heading_tolerance(45.0);
  } else if (options->heading_tolerance() < kMinHeadingTolerance ||
             options->heading_tolerance() > kMaxHeadingTolerance) {
    options->set_heading_tolerance(
        std::clamp(options->heading_tolerance(), kMinHeadingTolerance, kMaxHeadingTolerance));
    add_warning(request, 305,
                std::format("[{:.1f},{:.1f}]", kMinHeadingTolerance, kMaxHeadingTolerance));
  }

  constexpr double kDefaultOverlapFactor = 1.;
  constexpr double kMaxOverlapFactor = 100.;

  if (!options->has_overlap_factor()) {
    options->set_overlap_factor(kDefaultOverlapFactor);
  } else if (options->overlap_factor() < 0. || options->overlap_factor() > kMaxOverlapFactor) {
    options->set_overlap_factor(std::clamp(options->overlap_factor(), 0., kMaxOverlapFactor));
    add_warning(request, 306, std::format("[{:.1f},{:.1f}]", 0., kMaxOverlapFactor));
  }

  constexpr double kDefaultDistanceTolerance = 0.1;
  constexpr double kMinDistanceTolerance = 0.02;
  constexpr double kMaxDistanceTolerance = 0.5;

  if (!options->has_distance_tolerance()) {
    options->set_distance_tolerance(kDefaultDistanceTolerance);
  } else if (options->distance_tolerance() < kMinDistanceTolerance ||
             options->distance_tolerance() > kMaxDistanceTolerance) {
    options->set_distance_tolerance(
        std::clamp(options->distance_tolerance(), kMinDistanceTolerance, kMaxDistanceTolerance));
    add_warning(request, 307,
                std::format("[{:.2f},{:.2f}]", kMinDistanceTolerance, kMaxDistanceTolerance));
  }
}
} // namespace

namespace valhalla {
namespace loki {
void loki_worker_t::round_trip(Api& request) {

  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  parse_costing(request);

  // only allow a single location
  if (request.options().locations_size() == 0) {
    throw valhalla_exception_t{120};
  } else if (request.options().locations_size() > 1) {
    throw valhalla_exception_t{150, "1"};
  }

  auto& options = *request.mutable_options();
  const auto& costing_name = Costing_Enum_Name(options.costing_type());
  auto it = max_roundtrip_distances.find(costing_name);
  if (it == max_roundtrip_distances.end()) {
    throw valhalla_exception_t{125};
  }
  try {
    parse_round_trip_options(request, max_roundtrip_distances.at(costing_name));
  } catch (const valhalla_exception_t&) { throw; } catch (const std::exception& e) {
    LOG_ERROR("{}", e.what());
    throw valhalla_exception_t{100};
  }

  // thor finds the turning points of the round trip in its own expansions, so only the start is
  // correlated here
  auto* locations = options.mutable_locations();
  parse_locations(locations, request, valhalla_exception_t{112});
  try {
    search_.search(*locations, mode_costing[static_cast<size_t>(mode)]);
  } catch (const valhalla_exception_t&) { throw; } catch (const std::exception& e) {
    LOG_ERROR(e.what());
    throw valhalla_exception_t{171};
  }

  auto* correlation = locations->Mutable(0)->mutable_correlation();
  if (correlation->edges_size() == 0 && correlation->filtered_edges_size() == 0) {
    throw valhalla_exception_t{171};
  } else if (correlation->edges_size() == 0) {
    // if it only had filtered edges, promote those
    correlation->mutable_edges()->Swap(correlation->mutable_filtered_edges());
  }
}
} // namespace loki
} // namespace valhalla
