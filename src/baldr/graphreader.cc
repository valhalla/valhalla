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
  auto cached = tilecache_.find(graphid.Tile_Base());
  if(cached != tilecache_.end())
    return &cached->second;

  // It wasn't in cache so create a GraphTile object. This reads the tile from disk
  GraphTile tile(tile_hierarchy_.tile_dir(), graphid);
  // Need to check that the tile could be loaded, if it has no size it wasn't loaded
  if(tile.size() == 0)
    return nullptr;
  // Keep a copy in the cache and return it
  auto inserted = tilecache_.emplace(graphid.Tile_Base(), tile);
  return &inserted.first->second;
}

GraphTile* GraphReader::GetGraphTile(const PointLL& pointll, const uint8_t level){
  return GetGraphTile(tile_hierarchy_.GetGraphId(pointll, level));
}

GraphTile* GraphReader::GetGraphTile(const PointLL& pointll){
  return GetGraphTile(pointll, tile_hierarchy_.levels().rbegin()->second.level);
}

}
}
