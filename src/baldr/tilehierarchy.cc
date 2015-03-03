#include "valhalla/baldr/tilehierarchy.h"

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {
TileHierarchy::TileLevel::TileLevel(const boost::property_tree::ptree& pt):
  tiles({{-180, -90}, {180, 90}}, pt.get<float>("size")){
  level = pt.get<uint8_t>("level");
  name = pt.get<std::string>("name");
  //if not provided default to everything
  std::string cutoff = pt.get<std::string>("importance_cutoff", "ServiceOther");
  std::unordered_map<std::string, RoadClass>::const_iterator kv = stringToRoadClass.find(cutoff);
  if(kv == stringToRoadClass.end())
    throw std::runtime_error(cutoff + ": is not a valid value for importance_cutoff");
  importance = kv->second;
}

TileHierarchy::TileLevel::TileLevel(const unsigned char level, const std::string& name, const RoadClass importance, const Tiles& tiles)
  :level(level), importance(importance), name(name), tiles(tiles) {
}

TileHierarchy::TileHierarchy(const boost::property_tree::ptree& pt) {
  //grab out other config information
  tile_dir_ = pt.get<std::string>("tile_dir");

  //grab out each tile level
  for(const auto& tile_level : pt.get_child("levels")) {
    TileLevel tl(tile_level.second);
    auto success = levels_.emplace(tl.level, tl);
    if(!success.second)
      throw std::runtime_error("Multiple tile sets at the same level are not yet supported");
  }

  //if we didn't have any levels that is just not usable
  if(levels_.empty())
    throw std::runtime_error("Expected 1 or more levels in the tile hierarchy");
}

TileHierarchy::TileHierarchy(){}

const std::map<unsigned char, TileHierarchy::TileLevel>& TileHierarchy::levels() const {
  return levels_;
}

const std::string& TileHierarchy::tile_dir() const {
  return tile_dir_;
}

bool TileHierarchy::HasLevel(const unsigned char level) const {
  return levels_.find(level) != levels_.end();
}

GraphId TileHierarchy::GetGraphId(const midgard::PointLL& pointll, const unsigned char level) const {
  GraphId id;
  const auto& tl = levels_.find(level);
  if(tl != levels_.end()) {
    id.Set(static_cast<int32_t>(tl->second.tiles.TileId(pointll)), level, 0);
  }
  return id;
}

}
}
