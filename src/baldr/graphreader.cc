#include "baldr/graphreader.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <boost/filesystem.hpp>

#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/sequence.h>

#include "baldr/connectivity_map.h"
using namespace valhalla::baldr;

namespace {
  constexpr size_t DEFAULT_MAX_CACHE_SIZE = 1073741824; //1 gig
  constexpr size_t AVERAGE_TILE_SIZE = 2097152; //2 megs
  constexpr size_t AVERAGE_MM_TILE_SIZE = 1024; //1k
}

namespace valhalla {
namespace baldr {

struct GraphReader::tile_extract_t : public midgard::tar {
  tile_extract_t(const boost::property_tree::ptree& pt):tar(pt.get<std::string>("tile_extract","")) {
    //if you really meant to load it
    if(pt.get_optional<std::string>("tile_extract")) {
      //map files to graph ids
      for(auto& c : contents) {
        try {
          auto id = GraphTile::GetTileId(c.first);
          tiles[id] = std::make_pair(const_cast<char*>(c.second.first), c.second.second);
        }
        catch(...){}
      }
      //couldn't load it
      if(tiles.empty()) {
        LOG_WARN("Tile extract could not be loaded");
      }//loaded ok but with possibly bad blocks
      else {
        LOG_INFO("Tile extract successfully loaded");
        if(corrupt_blocks)
          LOG_WARN("Tile extract had " + std::to_string(corrupt_blocks) + " corrupt blocks");
      }
    }
  }
  // TODO: dont remove constness, and actually make graphtile read only?
  std::unordered_map<uint64_t, std::pair<char*, size_t> > tiles;
};

std::shared_ptr<const GraphReader::tile_extract_t> GraphReader::get_extract_instance(const boost::property_tree::ptree& pt) {
  static std::shared_ptr<const GraphReader::tile_extract_t> tile_extract(new GraphReader::tile_extract_t(pt));
  return tile_extract;
}

// Constructor using separate tile files
GraphReader::GraphReader(const boost::property_tree::ptree& pt)
    : tile_hierarchy_(pt.get<std::string>("tile_dir")),
      cache_size_(0),
      tile_extract_(get_extract_instance(pt)) {
  max_cache_size_ = pt.get<size_t>("max_cache_size", DEFAULT_MAX_CACHE_SIZE);

  // Reserve cache (based on whether using individual tile files or shared,
  // mmap'd file
  if (!tile_extract_->tiles.empty()) {
    cache_.reserve(max_cache_size_/AVERAGE_MM_TILE_SIZE);
  } else {
    // Assume avg of 2 megs per tile
    cache_.reserve(max_cache_size_/AVERAGE_TILE_SIZE);
  }
}

// Method to test if tile exists
bool GraphReader::DoesTileExist(const GraphId& graphid) const {
  if(tile_extract_->tiles.find(graphid) != tile_extract_->tiles.cend())
    return true;
  if(cache_.find(graphid) != cache_.end())
    return true;
  std::string file_location = tile_hierarchy_.tile_dir() + "/" +
    GraphTile::FileSuffix(graphid.Tile_Base(), tile_hierarchy_);
  struct stat buffer;
  return stat(file_location.c_str(), &buffer) == 0;
}
bool GraphReader::DoesTileExist(const boost::property_tree::ptree& pt, const GraphId& graphid) {
  auto extract = get_extract_instance(pt);
  if(extract->tiles.find(graphid) != extract->tiles.cend())
    return true;
  TileHierarchy tile_hierarchy(pt.get<std::string>("tile_dir"));
  std::string file_location = tile_hierarchy.tile_dir() + "/" +
    GraphTile::FileSuffix(graphid.Tile_Base(), tile_hierarchy);
  struct stat buffer;
  return stat(file_location.c_str(), &buffer) == 0;
}

// Get a pointer to a graph tile object given a GraphId. Return nullptr
// if the tile is not found/empty
const GraphTile* GraphReader::GetGraphTile(const GraphId& graphid) {
  //TODO: clear the cache automatically once we become overcommitted by a certain amount

  // Return nullptr if not a valid tile
  if (!graphid.Is_Valid()) {
    return nullptr;
  }

  // Check if the level/tileid combination is in the cache
  auto base = graphid.Tile_Base();
  auto cached = cache_.find(base);
  if(cached != cache_.end()) {
    return &cached->second;
  }

  // Try getting it from the memmapped tar extract
  if (!tile_extract_->tiles.empty()) {
    // Do we have this tile
    auto t = tile_extract_->tiles.find(base);
    if(t == tile_extract_->tiles.cend())
      return nullptr;

    // This initializes the tile from mmap
    GraphTile tile(base, t->second.first, t->second.second);
    if (!tile.header())
      return nullptr;

    // Keep a copy in the cache and return it
    cache_size_ += AVERAGE_MM_TILE_SIZE; // tile.end_offset();  // TODO what size??
    auto inserted = cache_.emplace(base, std::move(tile));
    return &inserted.first->second;
  }// Try getting it from flat file
  else {
    // This reads the tile from disk
    GraphTile tile(tile_hierarchy_, base);
    if (!tile.header())
      return nullptr;

    // Keep a copy in the cache and return it
    cache_size_ += tile.header()->end_offset();
    auto inserted = cache_.emplace(base, std::move(tile));
    return &inserted.first->second;
  }
}

const GraphTile* GraphReader::GetGraphTile(const PointLL& pointll, const uint8_t level){
  GraphId id = tile_hierarchy_.GetGraphId(pointll, level);
  return (id.Is_Valid()) ? GetGraphTile(tile_hierarchy_.GetGraphId(pointll, level)) : nullptr;
}

const GraphTile* GraphReader::GetGraphTile(const PointLL& pointll){
  return GetGraphTile(pointll, tile_hierarchy_.levels().rbegin()->second.level);
}

const TileHierarchy& GraphReader::GetTileHierarchy() const {
  return tile_hierarchy_;
}

// Clears the cache
void GraphReader::Clear() {
  cache_size_ = 0;
  cache_.clear();
}

// Returns true if the cache is over committed with respect to the limit
bool GraphReader::OverCommitted() const {
  return max_cache_size_ < cache_size_;
}

// Convenience method to get an opposing directed edge graph Id.
GraphId GraphReader::GetOpposingEdgeId(const GraphId& edgeid) {
  const GraphTile* NO_TILE = nullptr;
  return GetOpposingEdgeId(edgeid, NO_TILE);
}
GraphId GraphReader::GetOpposingEdgeId(const GraphId& edgeid, const GraphTile*& tile) {
  tile = GetGraphTile(edgeid);
  if(!tile)
    return {};
  const auto* directededge = tile->directededge(edgeid);

  // For now return an invalid Id if this is a transit edge
  if (directededge->IsTransitLine()) {
    return {};
  }

  // Get the opposing edge, if edge leaves the tile get the end node's tile
  GraphId id = directededge->endnode();

  if (directededge->leaves_tile()) {
    // Get tile at the end node
    tile = GetGraphTile(id);
  }

  if (tile != nullptr) {
    id.fields.id = tile->node(id)->edge_index() + directededge->opp_index();
    return id;
  } else {
    LOG_ERROR("Invalid tile for opposing edge: tile ID= " + std::to_string(id.tileid()) + " level= " + std::to_string(id.level()));
    if (directededge->trans_up() || directededge->trans_down()) {
      LOG_ERROR("transition edge being checked?");
    }
  }
  return {};
}

// Convenience method to get an opposing directed edge.
const DirectedEdge* GraphReader::GetOpposingEdge(const GraphId& edgeid) {
  const GraphTile* NO_TILE = nullptr;
  return GetOpposingEdge(edgeid, NO_TILE);
}
const DirectedEdge* GraphReader::GetOpposingEdge(const GraphId& edgeid, const GraphTile*& tile) {
  GraphId oppedgeid = GetOpposingEdgeId(edgeid, tile);
  return oppedgeid.Is_Valid() ? tile->directededge(oppedgeid) : nullptr;
}

// Convenience method to determine if 2 directed edges are connected.
bool GraphReader::AreEdgesConnected(const GraphId& edge1, const GraphId& edge2) {
  // Get both directed edges
  const GraphTile* t1 = GetGraphTile(edge1);
  const DirectedEdge* de1 = t1->directededge(edge1);
  const GraphTile* t2 = (edge2.Tile_Base() == edge1.Tile_Base()) ?
                          t1 : GetGraphTile(edge2);
  const DirectedEdge* de2 = t2->directededge(edge2);
  if (de1->endnode() == de2->endnode()) {
    return true;
  }

  // Get opposing edge to de1
  const DirectedEdge* de1_opp = GetOpposingEdge(edge1, t1);
  if (de1_opp->endnode() == de2->endnode()) {
    return true;
  }

  // Get opposing edge to de2 and compare to both edge1 endnodes
  const DirectedEdge* de2_opp = GetOpposingEdge(edge2, t2);
  if (de2_opp->endnode() == de1->endnode() ||
      de2_opp->endnode() == de1_opp->endnode()) {
    return true;
  }
  return false;
}

// Convenience method to get the relative edge density (from the
// begin node of an edge).
uint32_t GraphReader::GetEdgeDensity(const GraphId& edgeid) {
  // Get the end node of the opposing directed edge
  const DirectedEdge* opp_edge = GetOpposingEdge(edgeid);
  GraphId id = opp_edge->endnode();
  const GraphTile* tile = GetGraphTile(id);
  return (tile != nullptr) ? tile->node(id)->density() : 0;
}


std::unordered_set<GraphId> GraphReader::GetTileSet() const {
  //either mmap'd tiles
  std::unordered_set<GraphId> tiles;
  if(tile_extract_->tiles.size()) {
    for(const auto& t : tile_extract_->tiles)
      tiles.emplace(t.first);
  }//or individually on disk
  else {
    //for each level
    for(uint8_t level = 0; level < tile_hierarchy_.levels().rbegin()->first + 1; ++level) {
      //crack open this level of tiles directory
      boost::filesystem::path root_dir(tile_hierarchy_.tile_dir() + '/' + std::to_string(level) + '/');
      if(boost::filesystem::exists(root_dir) && boost::filesystem::is_directory(root_dir)) {
        //iterate over all the files in there
        for (boost::filesystem::recursive_directory_iterator i(root_dir), end; i != end; ++i) {
          if (!boost::filesystem::is_directory(i->path())) {
            //add it if it can be parsed as a valid tile file name
            try { tiles.emplace(GraphTile::GetTileId(i->path().string())); }
            catch (...) { }
          }
        }
      }
    }
  }

  //give them back
  return tiles;
}

}
}
