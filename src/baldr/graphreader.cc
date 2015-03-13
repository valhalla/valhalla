#include "baldr/graphreader.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sys/stat.h>

namespace {
  constexpr size_t DEFAULT_MAX_CACHE_SIZE = 1073741824; //1 gig
  constexpr size_t AVERAGE_TILE_SIZE = 2097152; //2 megs
}

namespace valhalla {
namespace baldr {

//this constructor delegates to the other
GraphReader::GraphReader(const boost::property_tree::ptree& pt):tile_hierarchy_(pt), cache_size_(0) {
  max_cache_size_ = pt.get<size_t>("max_cache_size", DEFAULT_MAX_CACHE_SIZE);

  //assume avg of 10 megs per tile
  cache_.reserve(max_cache_size_/AVERAGE_TILE_SIZE);
}

// Method to test if tile exists
bool GraphReader::DoesTileExist(const GraphId& graphid) const {
  return DoesTileExist(tile_hierarchy_, graphid);
}
bool GraphReader::DoesTileExist(const TileHierarchy& tile_hierarchy, const GraphId& graphid) {
  std::string file_location = tile_hierarchy.tile_dir() + "/" +
    GraphTile::FileSuffix(graphid.Tile_Base(), tile_hierarchy);
  struct stat buffer;
  return stat(file_location.c_str(), &buffer) == 0;
}

// Get a pointer to a graph tile object given a GraphId.
const GraphTile* GraphReader::GetGraphTile(const GraphId& graphid) {
  //TODO: clear the cache automatically once we become overcommitted by a certain amount

  // Check if the level/tileid combination is in the cache
  auto cached = cache_.find(graphid.Tile_Base());
  if(cached != cache_.end())
    return &cached->second;

  // It wasn't in cache so create a GraphTile object. This reads the tile from disk
  GraphTile tile(tile_hierarchy_, graphid);
  // Need to check that the tile could be loaded, if it has no size it wasn't loaded
  if(tile.size() == 0)
    return nullptr;
  // Keep a copy in the cache and return it
  cache_size_ += tile.size();
  auto inserted = cache_.emplace(graphid.Tile_Base(), std::move(tile));
  return &inserted.first->second;
}

const GraphTile* GraphReader::GetGraphTile(const PointLL& pointll, const uint8_t level){
  return GetGraphTile(tile_hierarchy_.GetGraphId(pointll, level));
}

const GraphTile* GraphReader::GetGraphTile(const PointLL& pointll){
  return GetGraphTile(pointll, tile_hierarchy_.levels().rbegin()->second.level);
}

const TileHierarchy& GraphReader::GetTileHierarchy() const {
  return tile_hierarchy_;
}

/**
 * Clears the cache
 */
void GraphReader::Clear() {
  cache_size_ = 0;
  cache_.clear();
}

/** Returns true if the cache is over committed with respect to the limit
 * @return  true
 */
bool GraphReader::OverCommitted() const {
  return max_cache_size_ < cache_size_;
}


}
}
