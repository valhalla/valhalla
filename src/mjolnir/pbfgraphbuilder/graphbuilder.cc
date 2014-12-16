#include <unordered_map>
#include <boost/functional/hash.hpp>

#include "graphbuilder.h"

#include "geo/pointll.h"
#include "geo/aabbll.h"
#include "geo/tiles.h"
#include "geo/polyline2.h"
#include "baldr/graphid.h"
#include "mjolnir/graphtilebuilder.h"

using namespace valhalla::geo;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

void GraphBuilder::node_callback(uint64_t osmid, double lng, double lat,
                                 const Tags &tags) {

  Tags results = lua_.TransformInLua(false, tags);

  if (results.size() == 0)
    return;

  //TODO::  Save the tag results to disk.

  nodes_.insert(std::make_pair(osmid, OSMNode((float) lat, (float) lng)));
  nodecount++;

//   if (nodecount % 10000 == 0) std::cout << nodecount << " nodes" << std::endl;
}

void GraphBuilder::way_callback(uint64_t osmid, const Tags &tags,
                                const std::vector<uint64_t> &refs) {
  // Do not add ways with < 2 nodes. Log error or add to a problem list
  // TODO - find out if we do need these, why they exist...
  if (refs.size() < 2) {
    std::cout << "ERROR - way " << osmid << " with < 2 nodes" << std::endl;
    return;
  }

  Tags results = lua_.TransformInLua(true, tags);

  if (results.size() == 0)
    return;

  OSMWay w(osmid);
  w.nodelist_ = refs;

  //TODO::  Save the tag results to disk.

  ways_.push_back(w);

}

void GraphBuilder::relation_callback(uint64_t /*osmid*/, const Tags &/*tags*/,
                                     const References & /*refs*/) {
  relationcount++;
}

void GraphBuilder::PrintCounts() {
  std::cout << "Read and parsed " << nodecount << " nodes, " << ways_.size()
            << " ways and " << relationcount << " relations" << std::endl;
}

// Once all the ways and nodes are read, we compute how many times a node
// is used. This allows us to detect intersections (nodes in a graph).
void GraphBuilder::SetNodeUses() {
  // Iterate through the ways
  for (auto way : ways_) {
    // Iterate through the nodes that make up the way
    for (auto node : way.nodelist_) {
      nodes_[node].uses_++;
    }
    // make sure that the last node is considered as an extremity
    // TODO - could avoid this by altering ConstructEdges
    nodes_[way.nodelist_.back()].uses_++;
  }
}

// Construct edges in the graph.
// TODO - compare logic to example_routing app. to see why the edge
// count differs.
void GraphBuilder::ConstructEdges() {
  // Iterate through the OSM ways
  unsigned int edgeindex = 0;
  uint64_t source, target, current;
  for (auto way : ways_) {
    Edge edge;
    current = way.nodelist_[0];
    OSMNode& node = nodes_[current];
    OSMNode& edgestartnode = node;
    edge.sourcenode_ = current;
    edge.latlngs_.push_back(node.latlng_);
    for (size_t i = 1, n = way.nodelist_.size(); i < n; ++i) {
      current = way.nodelist_[i];
      // If a node is used more than once, it is an intersection, hence it's
      // a node of the road network graph
      node = nodes_[current];
      edge.sourcenode_ = current;
      edge.latlngs_.push_back(node.latlng_);
      if (node.uses_ > 1) {
        edge.targetnode_ = current;
        edges_.push_back(edge);

        // Add the edge index to the target node
        edgestartnode.edges_.push_back(edgeindex);
        node.edges_.push_back(edgeindex);

        // Start a new edge
        edge.sourcenode_ = current;
        edgestartnode = node;
        edgeindex++;
      }
    }
  }
  std::cout << "Constructed " << edges_.size() << " edges" << std::endl;
}

// Iterate through the node map and remove any nodes that are not part of
// the graph (they have been converted to part of the edge shape)
void GraphBuilder::RemoveUnusedNodes() {
  node_map_type::iterator itr = nodes_.begin();
  while (itr != nodes_.end()) {
    if (itr->second.uses_ < 2) {
      nodes_.erase(itr++);
    } else {
      ++itr;
    }
  }
  std::cout << "Removed unused nodes; size = " << nodes_.size() << std::endl;
}

void GraphBuilder::TileNodes(const float tilesize, const unsigned int level) {
//  std::cout << "Tile nodes..." << std::endl;
  int tileid;
  unsigned int n;
  Tiles tiles(AABBLL(-90.0f, -180.0f, 90.0f, 180.0f), tilesize);

  // Get number of tiles and resize the tilednodes vector
  unsigned int tilecount = tiles.TileCount();
  tilednodes_.resize(tilecount);

  for (auto node : nodes_) {
    // Get tile Id
    tileid = tiles.TileId(node.second.latlng_);
    if (tileid < 0) {
      // TODO: error!
      std::cout << "Tile error" << std::endl;
    }

    // Get the number of nodes currently in the tile.
    // Set the GraphId for this OSM node.
    n = tilednodes_[tileid].size();
    node_graphids_.insert(
        std::make_pair(node.first, GraphId((unsigned int) tileid, level, n)));

    // Add this OSM node to the list of nodes for this tile
    tilednodes_[tileid].push_back(node.first);

  }
//  std::cout << "Done TileNodes" << std::endl;
}

namespace {
struct NodePairHasher {
  std::size_t operator()(const node_pair& k) const {
    std::size_t seed = 13;
    boost::hash_combine(seed, k.first.HashCode());
    boost::hash_combine(seed, k.second.HashCode());
    return seed;
  }
};
}

// Build tiles for the local graph hierarchy (basically
void GraphBuilder::BuildLocalTiles(const std::string& outputdir,
                                   const unsigned int level) {
// Edge info offset
  size_t edge_info_offset = 0;

// Iterate through the tiles
  unsigned int tileid = 0;
  for (auto tile : tilednodes_) {
    // Skip empty tiles
    if (tile.size() == 0) {
      tileid++;
      continue;
    }

    GraphTileBuilder graphtile;
    std::unordered_map<node_pair, size_t, NodePairHasher> edge_offset_map;

    // Iterate through the nodes
    unsigned int directededgecount = 0;
    for (auto osmnodeid : tile) {
      OSMNode& node = nodes_[osmnodeid];
      NodeInfoBuilder nodebuilder;
      nodebuilder.set_latlng(node.latlng_);

      // Set the index of the first outbound edge within the tile.
      nodebuilder.set_edge_index(directededgecount++);
      nodebuilder.set_edge_count(node.edges_.size());

      // Set up directed edges
      std::vector<DirectedEdgeBuilder> directededges;
      for (auto edgeindex : node.edges_) {
        DirectedEdgeBuilder directededge;
        const Edge& edge = edges_[edgeindex];
        directededge.set_endnode(node_graphids_[edge.targetnode_]);
        directededge.set_speed(65.0f);    // KPH

        // Compute length from the latlngs.
        float length = node.latlng_.Length(edge.latlngs_);
        directededge.set_length(length);

        // TODO - add other attributes

        // Check if we need to add edge info
        auto node_pair = ComputeNodePair(node_graphids_[edge.sourcenode_],
                                         node_graphids_[edge.targetnode_]);
        auto existing_edge_offset_kv = edge_offset_map.find(node_pair);

        // Add new exit info
        if (existing_edge_offset_kv == edge_offset_map.end()) {
          // TODO

        }
        // Update directed edge with existing edge offset
        else {
          directededge.set_edgedataoffset(existing_edge_offset_kv->second);
        }

        // Add to the list
        directededges.push_back(directededge);
      }

      // Add information to the tile
      graphtile.AddNodeAndEdges(nodebuilder, directededges);
    }

    // File name for tile
    GraphId graphid(tileid, level, 0);
    graphtile.StoreTileData(outputdir, graphid);

    tileid++;
  }
}

node_pair GraphBuilder::ComputeNodePair(const baldr::GraphId& nodea,
                                        const baldr::GraphId& nodeb) const {
  if (nodea < nodeb)
    return std::make_pair(nodea, nodeb);
  return std::make_pair(nodeb, nodea);
}

}
}
