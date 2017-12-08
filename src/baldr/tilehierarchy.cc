#include "baldr/tilehierarchy.h"
#include "baldr/graphtileheader.h"

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

// Static tile levels
std::map<uint8_t, TileLevel> TileHierarchy::levels_  = {
  {2, TileLevel{2, stringToRoadClass.find("ServiceOther")->second, "local",
    midgard::Tiles<midgard::PointLL>{{{-180, -90}, {180, 90}}, .25, static_cast<unsigned short>(kBinsDim)}}},
  {1, TileLevel{1, stringToRoadClass.find("Tertiary")->second, "arterial",
    midgard::Tiles<midgard::PointLL>{{{-180, -90}, {180, 90}}, 1, static_cast<unsigned short>(kBinsDim)}}},
  {0, TileLevel{0, stringToRoadClass.find("Primary")->second, "highway",
    midgard::Tiles<midgard::PointLL>{{{-180, -90}, {180, 90}}, 4, static_cast<unsigned short>(kBinsDim)}}}
};

//Should we make a class lower than service other for transit?
TileLevel TileHierarchy::transit_level_ = {3, stringToRoadClass.find("ServiceOther")->second, "transit",
  midgard::Tiles<midgard::PointLL>{{{-180, -90}, {180, 90}}, .25, static_cast<unsigned short>(kBinsDim)}};

// Returns the GraphId of the requested tile based on a lat,lng and a level.
// If the level is not supported an invalid id will be returned.
GraphId TileHierarchy::GetGraphId(const midgard::PointLL& pointll, const uint8_t level) {
  GraphId id;
  const auto& tl = levels().find(level);
  if(tl != levels_.end()) {
    auto tile_id = tl->second.tiles.TileId(pointll);
    if (tile_id >= 0) {
      id = { static_cast<uint32_t>(tile_id), level, 0 };
    }
  }
  return id;
}

// Gets the hierarchy level given the road class.
uint8_t TileHierarchy::get_level(const RoadClass roadclass) {
  if (roadclass <= levels().find(0)->second.importance) {
    return 0;
  } else if (roadclass <= levels().find(1)->second.importance) {
    return 1;
  } else {
    return 2;
  }
}

// Get the max hierarchy level.
uint8_t TileHierarchy::get_max_level() {
  return transit_level_.level;
}

// Returns all the GraphIds of the tiles which intersect the given bounding
// box at that level.
std::vector<GraphId> TileHierarchy::GetGraphIds(
        const midgard::AABB2<midgard::PointLL> &bbox,
        const uint8_t level) {
  std::vector<GraphId> ids;
  auto itr = levels().find(level);
  if (itr != levels().end()) {
    auto tile_ids = itr->second.tiles.TileList(bbox);
    ids.reserve(tile_ids.size());

    for (auto tile_id : tile_ids) {
      ids.emplace_back(tile_id, level, 0);
    }
  }
  return ids;
}

// Returns all the GraphIds of the tiles which intersect the given bounding
// box at any level.
std::vector<GraphId> TileHierarchy::GetGraphIds(
        const midgard::AABB2<midgard::PointLL> &bbox) {
  std::vector<GraphId> ids;
  for (const auto &entry : levels()) {
    auto level_ids = GetGraphIds(bbox, entry.first);
    ids.reserve(ids.size() + level_ids.size());
    ids.insert(ids.end(), level_ids.begin(), level_ids.end());
  }
  return ids;
}

}
}
