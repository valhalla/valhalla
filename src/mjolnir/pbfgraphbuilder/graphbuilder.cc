#include "graphbuilder.h"

#include "geo/pointll.h"
#include "geo/aabbll.h"
#include "geo/tiles.h"

using namespace valhalla::geo;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

void GraphBuilder::node_callback(uint64_t osmid, double lng, double lat,
                   const Tags &/*tags*/) {
  nodes_.insert(std::make_pair(osmid, OSMNode((float)lat, (float)lng)));
  nodecount++;

//   if (nodecount % 10000 == 0) std::cout << nodecount << " nodes" << std::endl;
}

void GraphBuilder::way_callback(uint64_t osmid, const Tags &tags,
                  const std::vector<uint64_t> &refs) {
  // Do not add ways with < 2 nodes. Log error or add to a problem list
  if (refs.size() < 2) {
//      std::cout << "ERROR - way " << osmid << " with < 2 nodes" << std::endl;
    return;
  }

  // Add the way if it has a highway tag.
  // There are other tags that correspond to the street network,
  // however for simplicity, we don't manage them
  if (tags.find("highway") != tags.end()) {
    OSMWay w(osmid);
    w.nodelist_ = refs;

    // TODO: Read more properties!!!

    ways_.push_back(w);
//      if (ways.size() % 100000 == 0)
//        std::cout << ways.size() << " ways" << std::endl;
  }
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

// Construct edges in the graph
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
  std::cout << "Tile nodes..." << std::endl;
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

    // Get the number of nodes currently in that tile. Add 1 to that
    // for the Id (Id == 0 is invalid). Set the GraphId for this OSM node.
    n = tilednodes_[tileid].size() + 1;
    node_graphids_.insert(std::make_pair(node.first,
               GraphId((unsigned int)tileid, level, n)));

    // Add this OSM node to the list of nodes for this tile
    tilednodes_[tileid].push_back(node.first);

  }
  std::cout << "Tiled Nodes" << std::endl;
}

void GraphBuilder::BuildLocalTiles(const std::string& outputdir,
            const float tilesize) {
  // Iterate through the tiles
  unsigned int tileid = 0;
  for (auto tile : tilednodes_) {
    // Skip empty tiles
    if (tile.size() == 0) {
      tileid++;
      continue;
    }

    std::cout << "Build Tile " << tileid << " with " << tile.size() <<
                   " nodes" << std::endl;

    // Iterate through the nodes
    for (auto node : tile) {

    }

    tileid++;
  }
}

}
}
