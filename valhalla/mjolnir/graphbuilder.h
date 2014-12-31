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
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>

#include <valhalla/mjolnir/luatagtransform.h>

// Forward declare
namespace CanalTP {
  class Reference;
}

namespace valhalla {
namespace mjolnir {

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
  NodeIdTable(const uint64_t maxosmid);

  /**
   * Destructor
   */
  ~NodeIdTable();

  /**
   * Sets the OSM Id as used.
   * @param   osmid   OSM Id of the node.
   */
  void set(const uint64_t id);

  /**
   * Test if the OSM Id is used / set in the bitmarker.
   * @param  id  OSM Id
   * @return  Returns true if the OSM Id is used. False if not.
   */
  const bool IsUsed(const uint64_t id) const;

 private:
  const uint64_t maxosmid_;
  std::vector<uint64_t> bitmarkers_;
};


//TODO: comment this data structure
struct Edge {
  uint64_t sourcenode_;
  uint64_t targetnode_;
  std::vector<midgard::PointLL> latlngs_;
  uint32_t wayindex_;

  // Construct a new edge. Target node and additional lat,lngs will
  // be filled in later.
  Edge(const uint64_t sourcenode, const uint32_t wayindex,
       const midgard::PointLL& ll)
      : sourcenode_(sourcenode),
        targetnode_(0),
        wayindex_(wayindex) {
    latlngs_.emplace_back(ll);
  }

  void AddLL(const midgard::PointLL& ll) {
    latlngs_.emplace_back(ll);
  }

 private:
  Edge()
      : sourcenode_(0),
        targetnode_(0),
        wayindex_(0){
  }
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

  //TODO: put these callbacks inside of an pimpl or just rewrite
  //the bits of canalTP that we care about

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
                         const std::vector<CanalTP::Reference> & /*refs*/);

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
   * Construct edges in the graph.
   */
  void ConstructEdges();

  /**
   * Add the nodes to tiles.
   * @param  tilesize  Size of tiles in degrees.
   * @param  level  Hierarchy level.
   */
  void TileNodes(const float tilesize, const uint8_t level);

  /**
   * Build tiles representing the local graph
   */
  void BuildLocalTiles(const uint8_t level) const;


 protected:

  //MAIN THREAD STUFF

  size_t node_count_;

  // Location of the protocol buffer input file
  std::string input_file_;

  // List of the tile levels to be created
  TileHierarchy tile_hierarchy_;

  // Lua Tag Transformation class
  LuaTagTransform lua_;

  // Stores all the ways that are part of the road network
  std::vector<OSMWay> ways_;

  // Mark the OSM Node Ids used by ways
  NodeIdTable shape_, intersection_;

  //CHILD THREAD STUFF

  // Map that stores all the nodes read
  std::unordered_map<uint64_t, OSMNode> nodes_;

  // Stores all the edges
  std::vector<Edge> edges_;

  // Map that stores all the ref info on a node
  std::unordered_map<uint64_t, std::string> map_ref_;

  // Map that stores all the exit to info on a node
  std::unordered_map<uint64_t, std::string> map_exit_to_;

  // A Task is a list of tiles which is just a list of node ids
  // Each tile writing thread gets a task
  using tile_t = std::list<uint64_t>;
  using task_t = std::vector<tile_t>;
  std::vector<task_t> tasks_;





};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHBUILDER_H
