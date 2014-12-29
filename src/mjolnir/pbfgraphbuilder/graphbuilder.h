#ifndef VALHALLA_MJOLNIR_GRAPHBUILDER_H
#define VALHALLA_MJOLNIR_GRAPHBUILDER_H

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>
#include <algorithm>
#include <boost/property_tree/ptree.hpp>

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
using node_map_type = std::unordered_map<uint64_t, OSMNode>;

// Mapping from OSM node Id to GraphId
using node_graphid_map_type = std::unordered_map<uint64_t, baldr::GraphId>;

using node_pair = std::pair<const baldr::GraphId&, const baldr::GraphId&>;

/**
 * A method for marking OSM node Ids that are used by ways.
 * Uses a vector where 1 bit is used for each possible Id.
 */
class NodeIdTable {
 public:
   /**
    * Constructor
    * @param   maxosmid   Maximum OSM Id to support.
    */
  NodeIdTable(const uint64_t maxosmid) {
    maxosmid_ = maxosmid;

    // Create a vector to mark bits. Initialize to 0.
    bitmarkers_.resize((maxosmid / 64) + 1, 0);
  }

  /**
   * Destructor
   */
  ~NodeIdTable() { }

  /**
   * Sets the OSM Id as used.
   * @param   osmid   OSM Id of the node.
   */
  void set(const uint64_t id) {
    // Test if the max is exceeded
    if (id > maxosmid_) {
      throw std::runtime_error("NodeIDTable - OSM Id exceeds max specified");
    }
    bitmarkers_[id / 64] |= (1 << (id % 64));
  }

  /**
   * Test if the OSM Id is used / set in the bitmarker.
   * @param  id  OSM Id
   * @return  Returns true if the OSM Id is used. False if not.
   */
  const bool IsUsed(const uint64_t id) const {
    return (bitmarkers_[id / 64] & (1 << (id % 64)));
  }

 private:
  uint64_t maxosmid_;
  std::vector<uint64_t> bitmarkers_;
};

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
  void BuildLocalTiles(const std::string& tiledir, const unsigned int level) const;


 protected:
  node_pair ComputeNodePair(const baldr::GraphId& nodea,
                            const baldr::GraphId& nodeb) const;

  bool preprocess_;

  uint64_t maxosmid_;
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

  // Mark the OSM Node Ids used by ways
  NodeIdTable osmnodeids_;

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
