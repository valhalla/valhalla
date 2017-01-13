#include "baldr/tilehierarchy.h"
#include "baldr/graphtileheader.h"

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

TileHierarchy::TileHierarchy(const std::string& tile_dir):tile_dir_(tile_dir) {
  levels_ = {
    {2, TileLevel{2, stringToRoadClass.find("ServiceOther")->second, "local", Tiles<PointLL>{{{-180, -90}, {180, 90}}, .25, kBinsDim}}},
    {1, TileLevel{1, stringToRoadClass.find("Tertiary")->second, "arterial", Tiles<PointLL>{{{-180, -90}, {180, 90}}, 1, kBinsDim}}},
    {0, TileLevel{0, stringToRoadClass.find("Primary")->second, "highway", Tiles<PointLL>{{{-180, -90}, {180, 90}}, 4, kBinsDim}}}
  };
}

TileHierarchy::TileHierarchy(){}

const std::map<unsigned char, TileHierarchy::TileLevel>& TileHierarchy::levels() const {
  return levels_;
}

const std::string& TileHierarchy::tile_dir() const {
  return tile_dir_;
}

GraphId TileHierarchy::GetGraphId(const midgard::PointLL& pointll, const unsigned char level) const {
  GraphId id;
  const auto& tl = levels_.find(level);
  if(tl != levels_.end()) {
    auto tile_id = tl->second.tiles.TileId(pointll);
    if (tile_id >= 0) {
      id.Set(tile_id, level, 0);
    }
  }
  return id;
}

// Gets the hierarchy level given the road class.
uint8_t TileHierarchy::get_level(const RoadClass roadclass) const {
  if (roadclass <= levels_.find(0)->second.importance) {
    return 0;
  } else if (roadclass <= levels_.find(1)->second.importance) {
    return 1;
  } else {
    return 2;
  }
}

std::vector<GraphId> TileHierarchy::GetGraphIds(
  const midgard::AABB2<midgard::PointLL> &bbox,
  uint8_t level) const {

  std::vector<GraphId> ids;

  auto itr = levels_.find(level);
  if (itr != levels_.end()) {
    auto tile_ids = itr->second.tiles.TileList(bbox);
    ids.reserve(tile_ids.size());

    for (auto tile_id : tile_ids) {
      ids.emplace_back(tile_id, level, 0);
    }
  }

  return ids;
}

std::vector<GraphId> TileHierarchy::GetGraphIds(
  const midgard::AABB2<midgard::PointLL> &bbox) const {

  std::vector<GraphId> ids;

  for (const auto &entry : levels_) {
    auto level_ids = GetGraphIds(bbox, entry.first);
    ids.reserve(ids.size() + level_ids.size());
    ids.insert(ids.end(), level_ids.begin(), level_ids.end());
  }

  return ids;
}

}
}
