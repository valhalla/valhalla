#pragma once

#include <deque>
#include <string>
#include <unordered_set>

#include <boost/property_tree/ptree.hpp>

#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "filesystem.h"
#include "midgard/logging.h"

namespace valhalla {
namespace mjolnir {

std::deque<baldr::GraphId> get_tile_ids(const boost::property_tree::ptree& pt,
                                        const std::unordered_set<std::string>& tiles) {
  if (tiles.empty())
    return {};

  auto tile_dir = pt.get_optional<std::string>("mjolnir.tile_dir");
  if (!tile_dir || !filesystem::exists(*tile_dir)) {
    LOG_WARN("Tile storage directory does not exist");
    return {};
  }

  std::deque<baldr::GraphId> tilequeue;
  baldr::GraphReader reader(pt.get_child("mjolnir"));
  std::for_each(std::begin(tiles), std::end(tiles), [&](const auto& tile) {
    auto tile_id = baldr::GraphTile::GetTileId(*tile_dir + tile);
    baldr::GraphId local_tile_id(tile_id.tileid(), tile_id.level(), tile_id.id());
    if (!reader.DoesTileExist(local_tile_id)) {
      LOG_WARN("Provided tile doesn't belong to the tile directory from config file");
      return;
    }

    tilequeue.push_back(tile_id);
  });

  return tilequeue;
}

} // namespace mjolnir
} // namespace valhalla
