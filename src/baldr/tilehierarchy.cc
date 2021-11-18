#include "baldr/tilehierarchy.h"
#include "baldr/graphtileheader.h"

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

// # level 0:  180
// # level 1:  90
// # level 2:  45
// # level 3:  22.5
// # level 4:  11.25
// # level 5:  5.625
// # level 6:  2.8125
// # level 7:  1.40625
// # level 8:  0.703125
// # level 9:  0.3515625
// # level 10: 0.17578125
// # level 11: 0.087890625
// # level 12: 0.0439453125
// # level 13: 0.02197265625

const std::vector<TileLevel>& TileHierarchy::levels() {
  // Static tile levels
  static const std::vector<TileLevel> levels_ = {

      TileLevel{0, stringToRoadClass("Primary"), "highway",
                midgard::Tiles<midgard::PointLL>{{{-180, -90}, {180, 90}},
                                                 4,
                                                 //  5.625, //  NDS Level 5
                                                 static_cast<unsigned short>(kBinsDim)}},

      TileLevel{1, stringToRoadClass("Tertiary"), "arterial",
                midgard::Tiles<midgard::PointLL>{{{-180, -90}, {180, 90}},
                                                 1,
                                                 //  1.40625, // NDS Level 7
                                                 static_cast<unsigned short>(kBinsDim)}},

      TileLevel{2, stringToRoadClass("ServiceOther"), "local",
                midgard::Tiles<midgard::PointLL>{{{-180, -90}, {180, 90}},
                                                 .25,
                                                 //  0.02197265625, // NDS Level 13
                                                 static_cast<unsigned short>(kBinsDim)}},
  };

  return levels_;
}

const TileLevel& TileHierarchy::GetTransitLevel() {
  // Should we make a class lower than service other for transit?
  static const TileLevel transit_level_ =
      {3, stringToRoadClass("ServiceOther"), "transit",
       midgard::Tiles<midgard::PointLL>{{{-180, -90}, {180, 90}},
                                        .25,
                                        // 0.02197265625,
                                        static_cast<unsigned short>(kBinsDim)}};

  return transit_level_;
}

midgard::AABB2<midgard::PointLL> TileHierarchy::GetGraphIdBoundingBox(const GraphId& id) {
  assert(id.Is_Valid());
  auto const& tileLevel = levels().at(id.level());
  return tileLevel.tiles.TileBounds(id.tileid());
}

// Returns the GraphId of the requested tile based on a lat,lng and a level.
// If the level is not supported an invalid id will be returned.
GraphId TileHierarchy::GetGraphId(const midgard::PointLL& pointll, const uint8_t level) {
  GraphId id;
  if (level < levels().size()) {
    auto tile_id = levels()[level].tiles.TileId(pointll);
    if (tile_id >= 0) {
      id = {static_cast<uint32_t>(tile_id), level, 0};
    }
  }
  return id;
}

// Gets the hierarchy level given the road class.
uint8_t TileHierarchy::get_level(const RoadClass roadclass) {
  if (roadclass <= levels()[0].importance) {
    return 0;
  } else if (roadclass <= levels()[1].importance) {
    return 1;
  } else {
    return 2;
  }
}

// Get the max hierarchy level.
uint8_t TileHierarchy::get_max_level() {
  return GetTransitLevel().level;
}

// Returns all the GraphIds of the tiles which intersect the given bounding
// box at that level.
std::vector<GraphId> TileHierarchy::GetGraphIds(const midgard::AABB2<midgard::PointLL>& bbox,
                                                const uint8_t level) {
  std::vector<GraphId> ids;
  if (level < levels().size()) {
    auto tile_ids = levels()[level].tiles.TileList(bbox);
    ids.reserve(tile_ids.size());

    for (auto tile_id : tile_ids) {
      ids.emplace_back(tile_id, level, 0);
    }
  }
  return ids;
}

// Returns all the GraphIds of the tiles which intersect the given bounding
// box at any level.
std::vector<GraphId> TileHierarchy::GetGraphIds(const midgard::AABB2<midgard::PointLL>& bbox) {
  std::vector<GraphId> ids;
  for (const auto& entry : levels()) {
    auto level_ids = GetGraphIds(bbox, entry.level);
    ids.reserve(ids.size() + level_ids.size());
    ids.insert(ids.end(), level_ids.begin(), level_ids.end());
  }
  return ids;
}

/**
 * Get the tiling system for a specified level.
 * @param level  Level Id.
 * @return Returns a const reference to the tiling system for this level.
 */
const midgard::Tiles<midgard::PointLL>& TileHierarchy::get_tiling(const uint8_t level) {
  if (level < levels().size()) {
    return levels()[level].tiles;
  }
  throw std::runtime_error("Invalid level Id for TileHierarchy::get_tiling");
}
} // namespace baldr
} // namespace valhalla
