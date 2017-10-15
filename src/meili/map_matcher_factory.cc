#include <string>

#include "sif/costconstants.h"
#include "sif/autocost.h"
#include "sif/motorscootercost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"

#include "meili/candidate_search.h"
#include "meili/map_matcher.h"

#include "meili/universal_cost.h"
#include "meili/map_matcher_factory.h"


namespace {

inline float
local_tile_size()
{
  const auto& tiles = valhalla::baldr::TileHierarchy::levels().rbegin()->second.tiles;
  return tiles.TileSize();
}

}


namespace valhalla {
namespace meili {

MapMatcherFactory::MapMatcherFactory(const boost::property_tree::ptree& root)
    : config_(root.get_child("meili")),
      graphreader_(root.get_child("mjolnir")),
      candidatequery_(graphreader_,
                      local_tile_size()/root.get<size_t>("meili.grid.size"),
                      local_tile_size()/root.get<size_t>("meili.grid.size")),
      max_grid_cache_size_(root.get<float>("meili.grid.cache_size"))
      { 
  cost_factory_.Register("auto", sif::CreateAutoCost);
  cost_factory_.Register("bicycle", sif::CreateBicycleCost);
  cost_factory_.Register("motor_scooter", sif::CreateMotorScooterCost);
  cost_factory_.Register("pedestrian", sif::CreatePedestrianCost);
  cost_factory_.Register("multimodal", CreateUniversalCost);
}


MapMatcherFactory::~MapMatcherFactory() {}


MapMatcher*
MapMatcherFactory::Create(const boost::property_tree::ptree& preferences)
{
  const auto& name = preferences.get<std::string>("mode", config_.get<std::string>("mode"));
  return Create(name, preferences);
}


MapMatcher*
MapMatcherFactory::Create(const std::string& costing, const boost::property_tree::ptree& preferences)
{
  const auto& config = MergeConfig(costing, preferences);

  valhalla::sif::cost_ptr_t cost = get_costing(config, costing);
  valhalla::sif::TravelMode mode = cost->travel_mode();

  mode_costing_[static_cast<uint32_t>(mode)] = cost;

  // TODO investigate exception safety
  return new MapMatcher(config, graphreader_, candidatequery_, mode_costing_, mode);
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

sif::cost_ptr_t MapMatcherFactory::get_costing(const boost::property_tree::ptree& request,
                                          const std::string& costing) {
  std::string method_options = "costing_options." + costing;
  auto costing_options = request.get_child(method_options, {});
  return cost_factory_.Create(costing, costing_options);
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
