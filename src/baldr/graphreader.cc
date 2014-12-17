#include "baldr/graphreader.h"

#include <string>
#include <iostream>
#include <fstream>

namespace valhalla {
namespace baldr {

// Default constructor
GraphReader::GraphReader(const std::string& datadirectory) {
  datadir_ = datadirectory;
}

// TODO - need a const for tileID bit size
unsigned int GraphReader::GetKey(const unsigned int level,
              const unsigned int tileid) {
  return tileid + (level << 24);
}

// Get a pointer to a graph tile object given a GraphId.
GraphTile* GraphReader::GetGraphTile(const GraphId& graphid) {
  // Check if the level/tileid combination is in the cache
  GraphTile* cacheptr = GetTileFromCache(graphid);
  if (cacheptr != nullptr) {
    return cacheptr;
  }

  // Tile is not in the cache, read it from disk.
  return ReadTile(graphid);
}

// Get a tile object from cache. Checks if the tile given by the tileid
// and level from the graphid is already in the cache.
GraphTile* GraphReader::GetTileFromCache(const GraphId& graphid) {
  auto it = tilecache_.find(GetKey(graphid.level(), graphid.tileid()));
  return (it == tilecache_.end()) ? nullptr : it->second;
}

// Read a tile object from disk and adds it to the cache.
GraphTile* GraphReader::ReadTile(const GraphId& graphid) {
  // Create a GraphTile object. This reads the tile. Check that the
  // size != -1 (-1 indicates tile not found or other error)
  GraphTile* tile = new GraphTile(datadir_, graphid);
  if (tile->size() == -1) {
    return nullptr;
  }

  // Add to the cache and return the pointer to the GraphTile
  tilecache_[GetKey(graphid.level(), graphid.tileid())] = tile;
  return tile;
}

}
}
