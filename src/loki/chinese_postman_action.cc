#include "baldr/datetime.h"
#include "baldr/rapidjson_utils.h"
#include "loki/search.h"
#include "loki/worker.h"
#include "midgard/logging.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace valhalla {
namespace loki {

void loki_worker_t::init_chinese_postman(Api& request) {
  auto& options = *request.mutable_options();

  // strip off unused information
  parse_locations(options.mutable_locations());
  if (options.locations_size() != 2) {
    throw valhalla_exception_t{120};
  };
  for (auto& l : *options.mutable_locations()) {
    l.clear_heading();
  }

  parse_costing(request);
}
void loki_worker_t::chinese_postman(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request, "loki_worker_t::chinese_postman");

  init_chinese_postman(request);
  auto& options = *request.mutable_options();

  try {
    // correlate the various locations to the underlying graph
    auto locations = PathLocation::fromPBF(options.locations());
    const auto projections = loki::Search(locations, *reader, costing);
    for (size_t i = 0; i < locations.size(); ++i) {
      const auto& projection = projections.at(locations[i]);
      PathLocation::toPBF(projection, options.mutable_locations(i), *reader);
    }
  } catch (const std::exception&) { throw valhalla_exception_t{171}; }

  // Taken from loki route_action
  // correlate the various locations to the underlying graph
  std::unordered_map<size_t, size_t> color_counts;
  try {
    auto locations = PathLocation::fromPBF(options.locations(), true);
    const auto projections = loki::Search(locations, *reader, costing);
    for (size_t i = 0; i < locations.size(); ++i) {
      const auto& correlated = projections.at(locations[i]);
      PathLocation::toPBF(correlated, options.mutable_locations(i), *reader);
      // TODO: get transit level for transit costing
      // TODO: if transit send a non zero radius
      if (!connectivity_map) {
        continue;
      }
      auto colors = connectivity_map->get_colors(TileHierarchy::levels().back().level, correlated, 0);
      for (auto color : colors) {
        auto itr = color_counts.find(color);
        if (itr == color_counts.cend()) {
          color_counts[color] = 1;
        } else {
          ++itr->second;
        }
      }
    }
  } catch (const std::exception&) { throw valhalla_exception_t{171}; }

  // are all the locations in the same color regions
  if (!connectivity_map) {
    return;
  }
  bool connected = false;
  for (const auto& c : color_counts) {
    if (c.second == options.locations_size()) {
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
