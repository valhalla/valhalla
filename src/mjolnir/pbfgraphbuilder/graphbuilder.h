#ifndef VALHALLA_MJOLNIR_GRAPHBUILDER_H
#define VALHALLA_MJOLNIR_GRAPHBUILDER_H

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <boost/property_tree/ptree.hpp>

#include "../../../include/geo/tilehierarchy.h"

#include "baldr/graphid.h"
#include "pbfgraphbuilder.h"

// Use open source PBF reader from:
//     https://github.com/CanalTP/libosmpbfreader
#include "osmpbfreader.h"
#include "geo/pointll.h"
#include "luatagtransform.h"


namespace valhalla {
namespace mjolnir {

// Node map
typedef std::map<uint64_t, OSMNode> node_map_type;

// Mapping from OSM node Id to GraphId
typedef std::map<uint64_t, baldr::GraphId> node_graphid_map_type;

using node_pair = std::pair<const baldr::GraphId&, const baldr::GraphId&>;

/**
 * Class used to construct temporary data used to build the initial graph.
 */
class GraphBuilder {
 public:
  /**
   * Constructor
   */
  GraphBuilder(const boost::property_tree::ptree& pt, const std::string& input_file);

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
  void LuaInit(std::string nodetagtransformscript, std::string nodetagtransformfunction,
               std::string waytagtransformscript, std::string waytagtransformfunction);


  /**
   * Debug method to print the number of nodes, ways, and relations that
   * were read and parsed from the pbf input file.
   */
  void PrintCounts();

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


  unsigned int relation_count_;
  unsigned int node_count_;

  // Map that stores all the nodes read
  node_map_type nodes_;

  // Stores all the nodes of all the ways that are part of the road network
  std::vector<OSMWay> ways_;

  // Stores all the edges
  std::vector<Edge> edges_;

  // Map of OSM node Ids to GraphIds
  node_graphid_map_type node_graphids_;

  // Lua Tag Transformation class
  LuaTagTransform lua_;

  // Tiled nodes
  std::vector<std::vector<uint64_t>> tilednodes_;

  // Location of the protocol buffer input file
  std::string input_file_;

  // List of the tile levels to be created
  geo::TileHierarchy tile_hierarchy_;

};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHBUILDER_H
