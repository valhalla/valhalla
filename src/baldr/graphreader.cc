#include "baldr/graphreader.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <boost/filesystem.hpp>

#include <valhalla/midgard/logging.h>

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

// Get a tile "map" - bool vector identifying which tiles exist.
std::vector<bool> GraphReader::TileMap(const uint32_t level) {
  for (auto tile_level : tile_hierarchy_.levels()) {
    if (tile_level.second.level == level) {
      Tiles tiles = tile_level.second.tiles;
      std::vector<bool> tilemap(tiles.TileCount(), false);

      // Get the base directory of this level
      std::string root_dir = tile_hierarchy_.tile_dir() + "/" +
                    std::to_string(level);
      for (boost::filesystem::recursive_directory_iterator i(root_dir),
                    end; i != end; ++i) {
        if (!is_directory(i->path())) {
          GraphId tileid = GraphTile::GetTileId(i->path().string());
          tilemap[tileid.tileid()] = true;
        }
      }
      return tilemap;
    }
  }
  LOG_ERROR("GraphReader::TileMap invalid level: " + std::to_string(level));
  return std::vector<bool>(0);
}

// Get a tile connectivity map. This is a vector where each tile Id is
std::vector<uint32_t> GraphReader::ConnectivityMap(const uint32_t level) {
  std::vector<bool> tilemap = TileMap(level);
  if (tilemap.size() > 0) {
    for (auto tile_level : tile_hierarchy_.levels()) {
      if (tile_level.second.level == level) {
        Tiles tiles = tile_level.second.tiles;
        return tiles.ConnectivityMap(tilemap);
      }
    }
  }
  return std::vector<uint32_t>(0);
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

// Convenience method to get an opposing directed edge.
GraphId GraphReader::GetOpposingEdgeId(const GraphId& edgeid) {
  auto* directededge = GetGraphTile(edgeid)->directededge(edgeid);
  GraphId endnodeid = directededge->endnode();
  auto* endnode = GetGraphTile(endnodeid)->node(endnodeid);
  GraphId opposing_edge_id;
  opposing_edge_id.Set(endnodeid.tileid(), endnodeid.level(),
                       endnode->edge_index() + directededge->opp_index());
  return opposing_edge_id;
}


}
}
