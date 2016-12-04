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
    id.Set(static_cast<int32_t>(tl->second.tiles.TileId(pointll)), level, 0);
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

}
}
