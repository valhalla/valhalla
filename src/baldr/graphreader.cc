#include "baldr/graphreader.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <boost/filesystem.hpp>

#include <valhalla/midgard/logging.h>
using namespace valhalla::baldr;

namespace {
  constexpr size_t DEFAULT_MAX_CACHE_SIZE = 1073741824; //1 gig
  constexpr size_t AVERAGE_TILE_SIZE = 2097152; //2 megs

  struct connectivity_map_t {
    connectivity_map_t(const TileHierarchy& tile_hierarchy) {
      // Populate a map for each level of the tiles that exist
      for (const auto& tile_level : tile_hierarchy.levels()) {
        auto level_colors = colors.insert({tile_level.first, std::unordered_map<uint32_t, size_t>{}}).first->second;
        std::string root_dir = tile_hierarchy.tile_dir() + "/" + std::to_string(tile_level.first);
        for (boost::filesystem::recursive_directory_iterator i(root_dir), end; i != end; ++i) {
          if (!is_directory(i->path())) {
            GraphId id = GraphTile::GetTileId(i->path().string(), tile_hierarchy);
            level_colors.insert({id.tileid(), 0});
          }
        }
      }

      // All tiles have color 0 (not connected), go through each level and connect them
      for(auto& level_colors : colors) {
        auto level = tile_hierarchy.levels().find(level_colors.first);
        level->second.tiles.ColorMap(level_colors.second);
      }
    }
    size_t get_color(const GraphId& id) const {
      auto level = colors.find(id.level());
      if(level == colors.cend())
        return 0;
      auto color = level->second.find(id.tileid());
      if(color == level->second.cend())
        return 0;
      return color->second;
    }
    std::unordered_map<uint32_t, std::unordered_map<uint32_t, size_t> > colors;
  };
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

bool GraphReader::AreConnected(const GraphId& first, const GraphId& second) const {
  //singleton is efficient here but does mean we cant reconfigure the tiles on the fly
  static const connectivity_map_t connectivity_map(this->tile_hierarchy_);

  //both must be the same color but also neither must be 0
  auto first_color = connectivity_map.get_color(first.Tile_Base());
  auto second_color = connectivity_map.get_color(second.Tile_Base());
  return first_color == second_color && first_color != 0;
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
