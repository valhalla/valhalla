#ifndef VALHALLA_MJOLNIR_GRAPHBUILDER_H
#define VALHALLA_MJOLNIR_GRAPHBUILDER_H

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <set>
#include <utility>
#include <algorithm>
#include <memory>
#include <boost/property_tree/ptree.hpp>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>
#include <valhalla/mjolnir/dataquality.h>

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


/**
 * An edge in the graph. Connects 2 nodes that have 2 or more "uses" - meaning
 * the node forms an intersection (or is the end of an OSM way). OSM nodes
 * with less than 2 uses become a shape point (lat,lng) along the edge.
 */
struct Edge {
  // OSM node Id of the source (start) node of the edge
  uint64_t sourcenode_;

  // OSM node Id of the target (end) node of the edge
  uint64_t targetnode_;

  // Index into the list of OSM way information
  uint32_t wayindex_;

  // Attributes needed to sort the edges
  // Note - this doesn't change the size of the structure...
  union Attributes {
    struct Fields {
      uint32_t importance       : 3;
      uint32_t driveableforward : 1;
      uint32_t driveablereverse : 1;
      uint32_t link             : 1;
      uint32_t spare            : 26;
    } fields;
    uint32_t v;
  };
  Attributes attributes_;

  // Shape of the edge (polyline)
  std::vector<midgard::PointLL> latlngs_;

  /**
   * Construct a new edge. Target node and additional lat,lngs will
   * be filled in later.
   * @param sourcenode   Start node of the edge
   * @param wayindex     Index into list of OSM ways
   * @param ll           Lat,lng at the start of the edge.
   * @param importance   Importance (classification) of the edge
   * @param driveforward Auto use in the forward direction of the edge
   * @param drivereverse Auto use in the reverse direction of the edge
   */
  Edge(const uint64_t sourcenode, const uint32_t wayindex,
       const midgard::PointLL& ll, const OSMWay& way)
      : sourcenode_(sourcenode),
        targetnode_(0),
        wayindex_(wayindex) {
    attributes_.fields.importance = static_cast<uint32_t>(way.road_class());
    attributes_.fields.driveableforward = way.auto_forward();
    attributes_.fields.driveablereverse = way.auto_backward();
    attributes_.fields.link = way.link();
    latlngs_.emplace_back(ll);
  }

  /**
   * Add a lat,lng to the shape of the edge.
   * @param  ll  Lat,lng
   */
  void AddLL(const midgard::PointLL& ll) {
    latlngs_.emplace_back(ll);
  }

 private:
  /**
   * Default constructor
   */
  Edge()
      : sourcenode_(0),
        targetnode_(0),
        wayindex_(0) {
    attributes_.v = 0;
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
  GraphBuilder(const boost::property_tree::ptree& pt);

  /**
   * Loads a given input file
   */
   void Load(const std::vector<std::string>& input_file);

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
   * Sort edges from the nodes (by driveability and importance).
   */
  void SortEdgesFromNodes();

  /**
   * Add the nodes to tiles.
   * @param  tilesize  Size of tiles in degrees.
   * @param  level  Hierarchy level.
   */
  void TileNodes(const float tilesize, const uint8_t level);

  /**
   * Update road class / importance of links (ramps)
   */
  void ReclassifyLinks();

  /**
   * Get the best classification for any non-link edges from a node.
   * @param  node  Node - gets outbound edges from this node.
   * @return  Returns the best (most important) classification
   */
  uint32_t GetBestNonLinkClass(const OSMNode& node) const;

  /**
   * Build tiles representing the local graph
   */
  void BuildLocalTiles(const uint8_t level) const;


 protected:

  //MAIN THREAD STUFF

  size_t node_count_, edge_count_, speed_assignment_count_;

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

  // A place to keep each tile's nodes so that various threads can
  // write various tiles asynchronously
  std::unordered_map<GraphId, std::vector<uint64_t> > tilednodes_;

  // Data quality / statistics.
  std::unique_ptr<DataQuality> stats_;
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHBUILDER_H
