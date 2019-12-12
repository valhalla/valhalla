#include "loki/search.h"
#include "loki/worker.h"

#include <unordered_map>

#include "baldr/datetime.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "midgard/logging.h"
#include "tyr/actor.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace {
midgard::PointLL to_ll(const valhalla::Location& l) {
  return midgard::PointLL{l.ll().lng(), l.ll().lat()};
}

void check_distance(const google::protobuf::RepeatedPtrField<valhalla::Location>& sources,
                    const google::protobuf::RepeatedPtrField<valhalla::Location>& targets,
                    float matrix_max_distance,
                    float& max_location_distance) {
  // see if any locations pairs are unreachable or too far apart
  for (const auto& source : sources) {
    for (const auto& target : targets) {
      // check if distance between latlngs exceed max distance limit
      auto path_distance = to_ll(source).Distance(to_ll(target));

      // only want to log the maximum distance between 2 locations for matrix
      LOG_DEBUG("path_distance -> " + std::to_string(path_distance));
      if (path_distance >= max_location_distance) {
        max_location_distance = path_distance;
        LOG_DEBUG("max_location_distance -> " + std::to_string(max_location_distance));
      }

      if (path_distance > matrix_max_distance) {
        throw valhalla_exception_t{154};
      };
    }
  }
}
} // namespace

namespace valhalla {
namespace loki {

void loki_worker_t::init_matrix(Api& request) {
  // we require sources and targets
  auto& options = *request.mutable_options();
  if (options.action() == Options::sources_to_targets) {
    parse_locations(options.mutable_sources(), valhalla_exception_t{112});
    parse_locations(options.mutable_targets(), valhalla_exception_t{112});
  } // optimized route uses locations but needs to do a matrix
  else {
    parse_locations(options.mutable_locations(), valhalla_exception_t{112});
    if (options.locations_size() < 2) {
      throw valhalla_exception_t{120};
    };

    // create new sources and targets from locations
    options.mutable_targets()->CopyFrom(options.locations());
    options.mutable_sources()->CopyFrom(options.locations());
  }

  // sanitize
  if (options.sources_size() < 1) {
    throw valhalla_exception_t{121};
  };
  for (auto& s : *options.mutable_sources()) {
    s.clear_heading();
  }
  if (options.targets_size() < 1) {
    throw valhalla_exception_t{122};
  };
  for (auto& t : *options.mutable_targets()) {
    t.clear_heading();
  }

  // no locations!
  options.clear_locations();

  // need costing
  parse_costing(request);
}

void loki_worker_t::matrix(Api& request) {
  init_matrix(request);
  auto& options = *request.mutable_options();
  auto costing_name = Costing_Enum_Name(options.costing());

  if (costing_name == "multimodal") {
    throw valhalla_exception_t{140, Options_Action_Enum_Name(options.action())};
  };

  // check that location size does not exceed max.
  auto max = max_matrix_locations.find(costing_name)->second;
  if (options.sources_size() > max || options.targets_size() > max) {
    throw valhalla_exception_t{150, std::to_string(max)};
  };

  // check the distances
  auto max_location_distance = std::numeric_limits<float>::min();
  check_distance(options.sources(), options.targets(), max_matrix_distance.find(costing_name)->second,
                 max_location_distance);

  // correlate the various locations to the underlying graph
  auto sources_targets = PathLocation::fromPBF(options.sources());
  auto st = PathLocation::fromPBF(options.targets());
  sources_targets.insert(sources_targets.end(), std::make_move_iterator(st.begin()),
                         std::make_move_iterator(st.end()));

  // correlate the various locations to the underlying graph
  std::unordered_map<size_t, size_t> color_counts;
  try {
    const auto searched = loki::Search(sources_targets, *reader, costing.get());
    for (size_t i = 0; i < sources_targets.size(); ++i) {
      const auto& l = sources_targets[i];
      const auto& projection = searched.at(l);
      PathLocation::toPBF(projection,
                          i < options.sources_size()
                              ? options.mutable_sources(i)
                              : options.mutable_targets(i - options.sources_size()),
                          *reader);
      // TODO: get transit level for transit costing
      // TODO: if transit send a non zero radius
      if (!connectivity_map) {
        continue;
      }
      auto colors =
          connectivity_map->get_colors(TileHierarchy::levels().rbegin()->first, projection, 0);
      for (auto& color : colors) {
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
    if (c.second == sources_targets.size()) {
      connected = true;
      break;
    }
  }
  if (!connected) {
    throw valhalla_exception_t{170};
  };
  if (!options.do_not_track()) {
    valhalla::midgard::logging::Log("max_location_distance::" +
                                        std::to_string(max_location_distance * midgard::kKmPerMeter) +
                                        "km",
                                    " [ANALYTICS] ");
  }
}
} // namespace loki
} // namespace valhalla
