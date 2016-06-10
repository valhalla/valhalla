#include "baldr/graphreader.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <boost/filesystem.hpp>

#include <valhalla/midgard/logging.h>
#include "baldr/connectivity_map.h"
using namespace valhalla::baldr;

namespace {
  constexpr size_t DEFAULT_MAX_CACHE_SIZE = 1073741824; //1 gig
  constexpr size_t AVERAGE_TILE_SIZE = 2097152; //2 megs
}

namespace valhalla {
namespace baldr {

//this constructor delegates to the other
GraphReader::GraphReader(const boost::property_tree::ptree& pt):tile_hierarchy_(pt.get<std::string>("tile_dir")), cache_size_(0) {
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

// Convenience method to get an opposing directed edge graph Id.
GraphId GraphReader::GetOpposingEdgeId(const GraphId& edgeid) {
  const GraphTile* NO_TILE = nullptr;
  return GetOpposingEdgeId(edgeid, NO_TILE);
}
GraphId GraphReader::GetOpposingEdgeId(const GraphId& edgeid, const GraphTile*& tile) {
  tile = GetGraphTile(edgeid);
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


}
}
