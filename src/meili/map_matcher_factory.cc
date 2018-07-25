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

MapMatcherFactory::MapMatcherFactory(const boost::property_tree::ptree& root)
    : config_(root.get_child("meili")), graphreader_(root.get_child("mjolnir")),
      candidatequery_(graphreader_,
                      local_tile_size() / root.get<size_t>("meili.grid.size"),
                      local_tile_size() / root.get<size_t>("meili.grid.size")),
      max_grid_cache_size_(root.get<float>("meili.grid.cache_size")) {
  cost_factory_.RegisterStandardCostingModels();
}

MapMatcherFactory::~MapMatcherFactory() {
}

MapMatcher* MapMatcherFactory::Create(const odin::Costing costing,
                                      const odin::DirectionsOptions& options) {
  // TODO figure out how we want to handle
  //  const auto& config = MergeConfig(costing, preferences);

  valhalla::sif::cost_ptr_t cost = cost_factory_.Create(costing, options);
  valhalla::sif::TravelMode mode = cost->travel_mode();

  mode_costing_[static_cast<uint32_t>(mode)] = cost;

  // TODO investigate exception safety
  //  return new MapMatcher(config, graphreader_, candidatequery_, mode_costing_, mode);
  return new MapMatcher(config_.get_child("default"), graphreader_, candidatequery_, mode_costing_,
                        mode);
}

MapMatcher* MapMatcherFactory::Create(const odin::DirectionsOptions& options) {
  return Create(options.costing(), options);
}

boost::property_tree::ptree
MapMatcherFactory::MergeConfig(const std::string& name,
                               const boost::property_tree::ptree& preferences) {
  // Copy the default child config
  auto config = config_.get_child("default");

  // The mode-specific config overwrites defaults
  const auto mode_config = config_.get_child_optional(name);
  if (mode_config) {
    for (const auto& child : *mode_config) {
      config.put_child(child.first, child.second);
    }
  }

  std::unordered_set<std::string> customizable;
  for (const auto& item : config_.get_child("customizable")) {
    customizable.insert(item.second.get_value<std::string>());
  }

  // Preferences overwrites defaults
  const auto trace_options = preferences.get_child_optional("trace_options");
  if (trace_options) {
    for (const auto& child : *trace_options) {
      const auto& name = child.first;
      const auto& values = child.second.data();
      if (customizable.find(name) != customizable.end() && !values.empty()) {
        try {
          // Possibly throw std::invalid_argument or std::out_of_range
          config.put<float>(name, std::stof(values));
        } catch (const std::invalid_argument& ex) {
          throw std::invalid_argument("Invalid argument: unable to parse " + name + " to float");
        } catch (const std::out_of_range& ex) {
          throw std::out_of_range("Invalid argument: " + name + " is out of float range");
        }
      }
    }
  }

  // Give it back
  return config;
}

void MapMatcherFactory::ClearFullCache() {
  if (graphreader_.OverCommitted()) {
    graphreader_.Clear();
  }

  if (candidatequery_.size() > max_grid_cache_size_) {
    candidatequery_.Clear();
  }
}

void MapMatcherFactory::ClearCache() {
  graphreader_.Clear();
  candidatequery_.Clear();
}

} // namespace meili
} // namespace valhalla
