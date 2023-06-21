#pragma once

#include <deque>
#include <string>
#include <unordered_set>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/filesystem.h>
#include <valhalla/midgard/logging.h>

namespace valhalla {
namespace baldr {

std::deque<baldr::GraphId> get_tile_ids(const boost::property_tree::ptree& pt,
                                        const std::unordered_set<std::string>& tiles) {
  if (tiles.empty())
    return {};

  auto tile_dir = pt.get_optional<std::string>("mjolnir.tile_dir");
  if (!tile_dir || !filesystem::exists(*tile_dir)) {
    LOG_WARN("Tile storage directory does not exist");
    return {};
  }

  std::unordered_set<std::string> tiles_set{tiles.begin(), tiles.end()};

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

} // namespace baldr
} // namespace valhalla
