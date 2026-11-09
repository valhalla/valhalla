
#include "loki/search.h"
#include "loki/worker.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace {
std::uint64_t mix(std::uint64_t x) {
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}

std::uint64_t round_trip_seed(const PointLL& ll) {
  const auto lat = static_cast<std::int64_t>(std::llround(ll.lat() * 1e6));
  const auto lng = static_cast<std::int64_t>(std::llround(ll.lng() * 1e6));
  return mix(static_cast<std::uint64_t>(lat) ^ mix(static_cast<std::uint64_t>(lng)));
}

std::vector<std::vector<PointLL>> seed_points(const Location& start,
                                              double heading,
                                              double heading_tolerance,
                                              double radius_meters,
                                              double radius_tolerance_meters,
                                              size_t num_groups,
                                              size_t points_per_group) {

  assert(num_groups > 0);
  assert(points_per_group > 0);

  PointLL startll{start.ll().lng(), start.ll().lat()};
  midgard::AzimuthalEquidistant aed(startll);
  Point2d start2d = aed.project(startll);

  auto heading_lo = heading - heading_tolerance;
  auto heading_hi = heading + heading_tolerance;

  radius_tolerance_meters = std::clamp(radius_tolerance_meters, 0.0, radius_meters);
  auto radius_lo = radius_meters - radius_tolerance_meters;
  auto radius_hi = radius_meters + radius_tolerance_meters;

  std::mt19937 engine{round_trip_seed(startll)};
  std::uniform_real_distribution radius_distribution(midgard::sqr(radius_lo),
                                                     midgard::sqr(radius_hi));

  std::vector<std::vector<PointLL>> seeded_points;
  seeded_points.resize(num_groups);

  auto step = (heading_hi - heading_lo) / static_cast<double>(num_groups);
  for (size_t i = 0; i < num_groups; ++i) {
    auto lo = heading_lo + i * step;
    auto hi = lo + step;
    std::uniform_real_distribution heading_distribution(lo, hi);
    seeded_points[i].resize(points_per_group);
    for (size_t j = 0; j < points_per_group; ++j) {
      // generate a random point in annular sector
      seeded_points[i][j] =
          aed.project_inverse(start2d.PointAlongHeading(heading_distribution(engine),
                                                        std::sqrt(radius_distribution(engine))));
    }
  }

  return seeded_points;
}
} // namespace

namespace valhalla {
namespace loki {
void loki_worker_t::round_trip(Api& request) {

  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  parse_locations(request.mutable_options()->mutable_locations(), request, valhalla_exception_t{112});

  // TODO: apply sensible defaults

  // only allow a single location for now
  if (request.options().locations_size() == 0) {
    throw valhalla_exception_t{120};
  } else if (request.options().locations_size() > 1) {
    throw valhalla_exception_t{150, "1"};
  }

  auto& options = *request.mutable_options();
  const auto& costing_name = Costing_Enum_Name(options.costing_type());

  // check that location size does not exceed max.
  auto max = max_matrix_locations.find(costing_name)->second;
  if (options.sources_size() * options.targets_size() > max) {
    throw valhalla_exception_t{150, std::to_string(max)};
  };

  // TODO: seed the points
  // add seed points to the back of the locations list

  // correlate the various locations to the underlying graph
  std::unordered_map<size_t, size_t> color_counts;
  try {
    search_.search(*options.mutable_locations(), mode_costing[static_cast<size_t>(mode)]);
    for (int i = 0; i < options.locations_size(); ++i) {
      const auto& l = options.locations().at(i);
      if (i < options.sources_size()) {
        options.mutable_sources(i)->CopyFrom(l);
      } else {
        options.mutable_targets(i - options.sources_size())->CopyFrom(l);
      }
      // TODO: get transit level for transit costing
      // TODO: if transit send a non zero radius
      if (!connectivity_map) {
        continue;
      }
      auto colors = connectivity_map->get_colors(TileHierarchy::levels().back(), l, 0);
      for (auto& color : colors) {
        auto itr = color_counts.find(color);
        if (itr == color_counts.cend()) {
          color_counts[color] = 1;
        } else {
          ++itr->second;
        }
      }
    }
  } catch (const valhalla_exception_t& e) { throw e; } catch (const std::exception&) {
    throw valhalla_exception_t{171};
  }

  // are all the locations in the same color regions
  if (!connectivity_map) {
    return;
  }
  bool connected = false;
  for (const auto& c : color_counts) {
    // one color must contain all locations
    if (static_cast<int>(c.second) == options.locations_size()) {
      connected = true;
      break;
    }
  }
  if (!connected) {
    throw valhalla_exception_t{170};
  };
}
} // namespace loki
} // namespace valhalla
