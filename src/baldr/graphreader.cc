#include "baldr/graphreader.h"

#include <string>
#include <iostream>
#include <fstream>

namespace valhalla {
namespace baldr {

//this constructor delegates to the other
GraphReader::GraphReader(const boost::property_tree::ptree& pt):GraphReader(TileHierarchy(pt)) {
}

GraphReader::GraphReader(const TileHierarchy& th):tile_hierarchy_(th) {
}

// Get a pointer to a graph tile object given a GraphId.
GraphTile* GraphReader::GetGraphTile(const GraphId& graphid) {
  // Check if the level/tileid combination is in the cache
  GraphTile* cacheptr = GetTileFromCache(graphid);
  if (cacheptr != nullptr) {
    return cacheptr;
  }

  // Create a GraphTile object. This reads the tile. Check that the
  // size != 0 (0 indicates tile not found or other error)
  GraphTile tile(tile_hierarchy_.tile_dir(), graphid);
  if (tile.size() == 0) {
    return nullptr;
  }

  // Add to the cache and return the pointer to the GraphTile
  auto success = tilecache_.emplace(graphid.Tile_Base(), tile);
  return &success.first->second;
}

GraphTile* GraphReader::GetGraphTile(const PointLL& pointll, const uint8_t level){
  return GetGraphTile(tile_hierarchy_.GetGraphId(pointll, level));
}

GraphTile* GraphReader::GetGraphTile(const PointLL& pointll){
  return GetGraphTile(pointll, tile_hierarchy_.levels().rbegin()->second.level);
}

// Get a tile object from cache. Checks if the tile given by the tileid
// and level from the graphid is already in the cache.
GraphTile* GraphReader::GetTileFromCache(const GraphId& graphid) {
  auto it = tilecache_.find(graphid.Tile_Base());
  return (it == tilecache_.end()) ? nullptr : &it->second;
}

}
}
