#include "baldr/graphreader.h"

#include <string>
#include <iostream>

#include "midgard/logging.h"

#include "baldr/connectivity_map.h"

using namespace valhalla::baldr;

namespace {
  constexpr size_t DEFAULT_MAX_CACHE_SIZE = 1073741824; //1 gig
  constexpr size_t AVERAGE_TILE_SIZE = 2097152; //2 megs
  constexpr size_t AVERAGE_MM_TILE_SIZE = 1024; //1k
}

namespace valhalla {
namespace baldr {

// Constructor using separate tile files
GraphReader::GraphReader(const std::shared_ptr<GraphTileStorage>& tile_storage, const boost::property_tree::ptree& pt)
    : tile_hierarchy_(tile_storage),
      cache_size_(0) {
  max_cache_size_ = pt.get<size_t>("max_cache_size", DEFAULT_MAX_CACHE_SIZE);

  // Assume avg of 2 megs per tile
  // TODO: for mmapped tiles, should assume 4KB per tile
  cache_.reserve(max_cache_size_/AVERAGE_TILE_SIZE);
}

// Method to test if tile exists
bool GraphReader::DoesTileExist(const GraphId& graphid) const {
  if(cache_.find(graphid) != cache_.end())
    return true;
  return DoesTileExist(tile_hierarchy_, graphid);
}

bool GraphReader::DoesTileExist(const TileHierarchy& tile_hierarchy, const GraphId& graphid) {
  return tile_hierarchy.tile_storage()->DoesTileExist(graphid, tile_hierarchy);
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

  // This reads the tile from disk
  GraphTile tile(tile_hierarchy_, base);
  if (!tile.header())
    return nullptr;

  // Keep a copy in the cache and return it
  cache_size_ += tile.header()->end_offset();
  auto inserted = cache_.emplace(base, std::move(tile));
  return &inserted.first->second;
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

// Get the shortcut edge that includes this edge.
GraphId GraphReader::GetShortcut(const GraphId& id) {
  // Lambda to get continuing edge at a node. Skips the specified edge Id
  // transition edges, shortcut edges, and transit connections. Returns
  // nullptr if more than one edge remains or no continuing edge is found.
  auto continuing_edge = [](const GraphTile* tile, const GraphId& edgeid,
                            const NodeInfo* nodeinfo) {
    uint32_t idx = nodeinfo->edge_index();
    const DirectedEdge* continuing_edge = static_cast<const DirectedEdge*>(nullptr);
    const DirectedEdge* directededge = tile->directededge(idx);
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, idx++) {
      if (directededge->trans_up() || directededge->trans_down() ||
          idx == edgeid.id() || directededge->is_shortcut() ||
          directededge->use() == Use::kTransitConnection) {
        continue;
      }
      if (continuing_edge != nullptr) {
        return static_cast<const DirectedEdge*>(nullptr);
      }
      continuing_edge = directededge;
    }
    return continuing_edge;
  };

  // No shortcuts on the local level or transit level.
  if (id.level() >= tile_hierarchy_.levels().rbegin()->second.level) {
    return { };
  }

  // If this edge is a shortcut return this edge Id
  const GraphTile* tile = GetGraphTile(id);
  const DirectedEdge* directededge = tile->directededge(id);
  if (directededge->is_shortcut()) {
    return id;
  }

  // Walk backwards along the opposing directed edge until a shortcut
  // beginning is found or to get the continuing edge until a node that starts
  // the shortcut is found or there are 2 or more other regular edges at the
  // node.
  GraphId edgeid = id;
  const NodeInfo* node = nullptr;
  const DirectedEdge* cont_de = nullptr;
  while (true) {
    // Get the continuing directed edge. Initial case is to use the opposing
    // directed edge.
    cont_de = (node == nullptr) ? GetOpposingEdge(id) :
                continuing_edge(tile, edgeid, node);
    if (cont_de == nullptr) {
      return { };
    }

    // Get the end node and end node tile
    GraphId endnode = cont_de->endnode();
    if (cont_de->leaves_tile()) {
      tile = GetGraphTile(endnode.Tile_Base());
    }
    node = tile->node(endnode);

    // Get the opposing edge Id and its directed edge
    uint32_t idx = node->edge_index() + cont_de->opp_index();
    edgeid = { endnode.tileid(), endnode.level(), idx };
    directededge = tile->directededge(edgeid);
    if (directededge->superseded()) {
      // Get the shortcut edge Id that supersedes this edge
      uint32_t idx = node->edge_index() + (directededge->superseded() - 1);
      return GraphId(endnode.tileid(), endnode.level(), idx);
    }
  }
  return { };
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
  return tile_hierarchy_.tile_storage()->FindTiles(tile_hierarchy_);
}

}
}
