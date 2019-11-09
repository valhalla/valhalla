#include <string>

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
  const auto& tiles = valhalla::baldr::TileHierarchy::levels().rbegin()->second.tiles;
  return tiles.TileSize();
}

} // namespace

namespace valhalla {
namespace meili {

MapMatcherFactory::MapMatcherFactory(const boost::property_tree::ptree& root,
                                     const std::shared_ptr<baldr::GraphReader>& graph_reader)
    : config_(root.get_child("meili")), graphreader_(graph_reader),
      max_grid_cache_size_(root.get<float>("meili.grid.cache_size")) {
  if (!graphreader_)
    graphreader_.reset(new baldr::GraphReader(root.get_child("mjolnir")));
  candidatequery_.reset(
      new CandidateGridQuery(*graphreader_, local_tile_size() / root.get<size_t>("meili.grid.size"),
                             local_tile_size() / root.get<size_t>("meili.grid.size")));
  cost_factory_.RegisterStandardCostingModels();
}

MapMatcherFactory::~MapMatcherFactory() {
}

MapMatcher* MapMatcherFactory::Create(const Costing costing, const Options& options) {
  // Merge any customizable options with the config defaults
  const auto& config = MergeConfig(options);

  valhalla::sif::cost_ptr_t cost = cost_factory_.Create(costing, options);
  valhalla::sif::TravelMode mode = cost->travel_mode();

  mode_costing_[static_cast<uint32_t>(mode)] = cost;

  // TODO investigate exception safety
  return new MapMatcher(config, *graphreader_, *candidatequery_, mode_costing_, mode);
}

MapMatcher* MapMatcherFactory::Create(const Options& options) {
  return Create(options.costing(), options);
}

boost::property_tree::ptree MapMatcherFactory::MergeConfig(const Options& options) {
  // Copy the default child config
  auto config = config_.get_child("default");

  // Get a list of customizable options
  std::unordered_set<std::string> customizable;
  for (const auto& item : config_.get_child("customizable")) {
    customizable.insert(item.second.get_value<std::string>());
  }

  // Check for overrides of matcher related directions options. Override these values in config.
  if (options.search_radius() && customizable.find("search_radius") != customizable.end()) {
    config.put<float>("search_radius", options.search_radius());
  }
  if (options.turn_penalty_factor() &&
      customizable.find("turn_penalty_factor") != customizable.end()) {
    config.put<float>("turn_penalty_factor", options.turn_penalty_factor());
  }
  if (options.gps_accuracy() && customizable.find("gps_accuracy") != customizable.end()) {
    config.put<float>("gps_accuracy", options.gps_accuracy());
  }
  if (options.breakage_distance() && customizable.find("breakage_distance") != customizable.end()) {
    config.put<float>("breakage_distance", options.breakage_distance());
  }
  if (options.has_interpolation_distance() &&
      customizable.find("interpolation_distance") != customizable.end()) {
    config.put<float>("interpolation_distance", options.interpolation_distance());
  }

  // Give it back
  return config;
}

void MapMatcherFactory::ClearFullCache() {
  if (graphreader_->OverCommitted()) {
    graphreader_->Trim();
  }

  if (candidatequery_->size() > max_grid_cache_size_) {
    candidatequery_->Clear();
  }
}

void MapMatcherFactory::ClearCache() {
  graphreader_->Clear();
  candidatequery_->Clear();
}

} // namespace meili
} // namespace valhalla
