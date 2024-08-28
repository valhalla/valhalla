#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/costconstants.h"
#include "sif/motorscootercost.h"
#include "sif/pedestriancost.h"

#include "meili/candidate_search.h"
#include "meili/map_matcher.h"

#include "meili/map_matcher_factory.h"

namespace {

inline float local_tile_size() {
  const auto& tiles = valhalla::baldr::TileHierarchy::levels().back().tiles;
  return tiles.TileSize();
}

} // namespace

namespace valhalla {
namespace meili {

MapMatcherFactory::MapMatcherFactory(const boost::property_tree::ptree& root,
                                     const std::shared_ptr<baldr::GraphReader>& graph_reader)
    : config_(root.get_child("meili")), graphreader_(graph_reader) {
  if (!graphreader_)
    graphreader_.reset(new baldr::GraphReader(root.get_child("mjolnir")));
  candidatequery_.reset(
      new CandidateGridQuery(*graphreader_, local_tile_size() / config_.candidate_search.grid_size,
                             local_tile_size() / config_.candidate_search.grid_size));
}

MapMatcherFactory::~MapMatcherFactory() {
}

MapMatcher* MapMatcherFactory::Create(const Options& options) {
  // Merge any customizable options with the config defaults
  const auto& config = MergeConfig(options);

  valhalla::sif::cost_ptr_t cost = cost_factory_.Create(options);
  valhalla::sif::TravelMode mode = cost->travel_mode();

  mode_costing_[static_cast<uint32_t>(mode)] = cost;

  // TODO investigate exception safety
  return new MapMatcher(config, *graphreader_, *candidatequery_, mode_costing_, mode);
}

Config MapMatcherFactory::MergeConfig(const Options& options) const {
  // Copy the default config
  auto config = config_;

  // Check for overrides of matcher related directions options. Override these values in config.
  if (options.has_search_radius_case() && config.candidate_search.is_search_radius_customizable) {
    config.candidate_search.search_radius_meters = options.search_radius();
  }
  if (options.has_turn_penalty_factor_case() &&
      config.transition_cost.is_turn_penalty_factor_customizable) {
    config.transition_cost.turn_penalty_factor = options.turn_penalty_factor();
  }
  if (options.has_gps_accuracy_case() && config.emission_cost.is_gps_accuracy_customizable) {
    config.emission_cost.gps_accuracy_meters = options.gps_accuracy();
  }
  if (options.has_breakage_distance_case() &&
      config.transition_cost.is_breakage_distance_customizable) {
    config.transition_cost.breakage_distance_meters = options.breakage_distance();
  }
  if (options.has_interpolation_distance_case() &&
      config.routing.is_interpolation_distance_customizable) {
    config.routing.interpolation_distance_meters = options.interpolation_distance();
  }

  // Give it back
  return config;
}

void MapMatcherFactory::ClearFullCache() {
  if (graphreader_->OverCommitted()) {
    graphreader_->Trim();
  }

  if (candidatequery_->size() > config_.candidate_search.cache_size) {
    candidatequery_->Clear();
  }
}

void MapMatcherFactory::ClearCache() {
  graphreader_->Clear();
  candidatequery_->Clear();
}

} // namespace meili
} // namespace valhalla
