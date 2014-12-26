#ifndef VALHALLA_MJOLNIR_GRAPHBUILDER_H
#define VALHALLA_MJOLNIR_GRAPHBUILDER_H

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <boost/property_tree/ptree.hpp>

//TODO- add later
//#include <google/sparsetable>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include "pbfgraphbuilder.h"
#include "osmway.h"

// Use open source PBF reader from:
//     https://github.com/CanalTP/libosmpbfreader
#include "osmpbfreader.h"
#include "luatagtransform.h"

namespace valhalla {
namespace mjolnir {

// Node map
typedef std::map<uint64_t, OSMNode> node_map_type;

// Mapping from OSM node Id to GraphId
typedef std::map<uint64_t, baldr::GraphId> node_graphid_map_type;

using node_pair = std::pair<const baldr::GraphId&, const baldr::GraphId&>;

/**
 * Sparse table of node Ids used in ways we keep.
 */
/** TODO - add later
class NodeIDTable {
 public:
  NodeIDTable() {
    ids_.resize(10000000);
  }

  void set(const uint64_t id) {
    if (id >= ids_.size()) {
      ids_.resize(id + 10000000);
    }
    ids_[id] = true;
  }

  const bool operator[](const uint64_t id) const {
    return ids_[id];
  }

  uint64_t size() {
    return ids_.size();
  }

  size_t nonempty() const {
    return ids_.num_nonempty();
  }

  size_t memory_use() const {
    return (ids_.size() / 8page-not-found) + (ids_.num_nonempty() * sizeof(bool));
  }

 protected:
  google::sparsetable<bool> ids_;
};
*/

/**
 * Class used to construct temporary data used to build the initial graph.
 */
class GraphBuilder {
 public:
  /**
   * Constructor
   */
  GraphBuilder(const boost::property_tree::ptree& pt,
               const std::string& input_file);

  /**
   * Callback method for OSMPBFReader. Called when a node is parsed.
   */
  void node_callback(uint64_t osmid, double lng, double lat,
                     const Tags &/*tags*/);

  /**
   * Callback method for OSMPBFReader. Called when a way is parsed.
   */
  void way_callback(uint64_t osmid, const Tags &tags,
                    const std::vector<uint64_t> &refs);

  /**
   * Callback method for OSMPBFReader. Called when a relation is parsed.
   */
  void relation_callback(uint64_t /*osmid*/, const Tags &/*tags*/,
                         const CanalTP::References & /*refs*/);

  /**
   * Tell the builder to build the tiles from the provided datasource and configs
   *
   */
  void Build();

 private:

  /**
   * Initialize Lua with the scripts and functions.
   */
  void LuaInit(const std::string& nodetagtransformscript,
               const std::string& nodetagtransformfunction,
               const std::string& waytagtransformscript,
               const std::string& waytagtransformfunction);

  /**
   * This method computes how many times a node is used. Nodes with use count
   * > 1 are intersections (or ends of a way) and become nodes in the graph.
   */
  void SetNodeUses();

  /**
   * Construct edges in the graph.
   */
  void ConstructEdges();

  /**
   * Remove unused OSM nodes from the map. These are nodes with use count
   * < 2 - they are converted to shape within an edge with ConstructEdges.
   */
  void RemoveUnusedNodes();

  /**
   * Add the nodes to tiles.
   * @param  tilesize  Size of tiles in degrees.
   * @param  level  Hierarchy level.
   */
  void TileNodes(const float tilesize, const unsigned int level);

  /**
   * Build tiles representing the local graph
   */
  void BuildLocalTiles(const std::string& tiledir, const unsigned int level);


 protected:
  node_pair ComputeNodePair(const baldr::GraphId& nodea,
                            const baldr::GraphId& nodeb) const;

  bool preprocess_;

  // Reference to the set of OSM Node Ids used by ways
// TOD - add later
//  NodeIDTable osmnodeids_;

  uint32_t skippednodes_;
  uint32_t skippedhighway_;
  uint32_t relation_count_;
  uint32_t node_count_;

  // Map that stores all the nodes read
  node_map_type nodes_;

  // Stores all the nodes of all the ways that are part of the road network
  std::vector<OSMWay> ways_;

  // Stores all the edges
  std::vector<Edge> edges_;

  // Lua Tag Transformation class
  LuaTagTransform lua_;

  // Tiled nodes
  std::vector<std::vector<uint64_t>> tilednodes_;

  // Location of the protocol buffer input file
  std::string input_file_;

  // List of the tile levels to be created
  TileHierarchy tile_hierarchy_;

  // Map that stores all the ref info on a node
  std::unordered_map<uint64_t, std::string> map_ref_;

  // Map that stores all the exit to info on a node
  std::unordered_map<uint64_t, std::string> map_exit_to_;
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHBUILDER_H
