#include <string>

#include <valhalla/sif/costconstants.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/pedestriancost.h>
#include <valhalla/baldr/graphreader.h>

#include <valhalla/meili/candidate_search.h>
#include <valhalla/meili/map_matcher.h>

#include "meili/universal_cost.h"
#include "meili/map_matcher_factory.h"


namespace {

inline float
local_tile_size(const valhalla::baldr::GraphReader& graphreader)
{
  const auto& tile_hierarchy = graphreader.GetTileHierarchy();
  const auto& tiles = tile_hierarchy.levels().rbegin()->second.tiles;
  return tiles.TileSize();
}

}


namespace valhalla {
namespace meili {

MapMatcherFactory::MapMatcherFactory(const boost::property_tree::ptree& root)
    : config_(root.get_child("meili")),
      graphreader_(root.get_child("mjolnir")),
      mode_costing_{nullptr},
      mode_name_(),
      candidatequery_(graphreader_,
                      local_tile_size(graphreader_)/root.get<size_t>("meili.grid.size"),
                      local_tile_size(graphreader_)/root.get<size_t>("meili.grid.size")),
      max_grid_cache_size_(root.get<float>("meili.grid.cache_size"))
      { init_costings(root); }


MapMatcherFactory::~MapMatcherFactory() {}


sif::TravelMode
MapMatcherFactory::NameToTravelMode(const std::string& name)
{
  for (size_t idx = 0; idx < kModeCostingCount; idx++) {
    if (!name.empty() && mode_name_[idx] == name) {
      return static_cast<sif::TravelMode>(idx);
    }
  }
  throw std::invalid_argument("Invalid costing name: " + name);
}


const std::string&
MapMatcherFactory::TravelModeToName(sif::TravelMode travelmode)
{
  const auto index = static_cast<size_t>(travelmode);
  if (index < kModeCostingCount) {
    if (!mode_name_[index].empty()) {
      return mode_name_[index];
    }
  }
  throw std::invalid_argument("Invalid travelmode code " + std::to_string(index));
}


MapMatcher*
MapMatcherFactory::Create(const boost::property_tree::ptree& preferences)
{
  const auto& name = preferences.get<std::string>("mode", config_.get<std::string>("mode"));
  const auto travelmode = NameToTravelMode(name);
  return Create(travelmode, preferences);
}


MapMatcher*
MapMatcherFactory::Create(sif::TravelMode travelmode, const boost::property_tree::ptree& preferences)
{
  const auto& config = MergeConfig(TravelModeToName(travelmode), preferences);
  // TODO investigate exception safety
  return new MapMatcher(config, graphreader_, candidatequery_, mode_costing_, travelmode);
}


boost::property_tree::ptree
MapMatcherFactory::MergeConfig(const std::string& name,
                               const boost::property_tree::ptree& preferences)
{
  // Copy the default child config
  auto config = config_.get_child("default");

  // The mode-specific config overwrites defaults
  const auto mode_config = config_.get_child_optional(name);
  if (mode_config) {
    for (const auto& child : *mode_config) {
      config.put_child(child.first, child.second);
    }
  }

  // Preferences overwrites defaults
  for (const auto& child : preferences) {
    config.put_child(child.first, child.second);
  }

  // Give it back
  return config;
}


boost::property_tree::ptree&
MapMatcherFactory::MergeConfig(const std::string& name,
                               boost::property_tree::ptree& preferences)
{
  const auto mode_config = config_.get_child_optional(name);
  if (mode_config) {
    for (const auto& child : *mode_config) {
      auto pchild = preferences.get_child_optional(child.first);
      if (!pchild) {
        preferences.put_child(child.first, child.second);
      }
    }
  }

  for (const auto& child : config_.get_child("default")) {
    auto pchild = preferences.get_child_optional(child.first);
    if (!pchild) {
      preferences.put_child(child.first, child.second);
    }
  }

  return preferences;
}


size_t
MapMatcherFactory::register_costing(const std::string& mode_name,
                                    factory_function_t factory,
                                    const boost::property_tree::ptree& config)
{
  auto costing = factory(config);
  auto index = static_cast<size_t>(costing->travelmode());
  if (!(index < kModeCostingCount)) {
    throw std::out_of_range("Configuration error: out of bounds");
  }
  if (mode_costing_[index]) {
    throw std::runtime_error("Configuration error: found duplicate travel mode");
  }
  mode_costing_[index] = costing;
  mode_name_[index] = mode_name;
  return index;
}


sif::cost_ptr_t*
MapMatcherFactory::init_costings(const boost::property_tree::ptree& root)
{
  register_costing("auto", sif::CreateAutoCost, root.get_child("costing_options.auto"));
  register_costing("bicycle", sif::CreateBicycleCost, root.get_child("costing_options.bicycle"));
  register_costing("pedestrian", sif::CreatePedestrianCost, root.get_child("costing_options.pedestrian"));
  register_costing("multimodal", CreateUniversalCost, root.get_child("costing_options.multimodal"));

  return mode_costing_;
}


void MapMatcherFactory::ClearFullCache()
{
  if(graphreader_.OverCommitted()) {
    graphreader_.Clear();
  }

  if (candidatequery_.size() > max_grid_cache_size_) {
    candidatequery_.Clear();
  }
}


void MapMatcherFactory::ClearCache()
{
  graphreader_.Clear();
  candidatequery_.Clear();
}

}
}
