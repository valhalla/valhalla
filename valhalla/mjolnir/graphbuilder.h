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

#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/signinfo.h>

#include <valhalla/mjolnir/osmdata.h>
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>
#include <valhalla/mjolnir/sequence.h>
#include <valhalla/mjolnir/dataquality.h>
#include <valhalla/mjolnir/edgeinfobuilder.h>

namespace valhalla {
namespace mjolnir {

/**
 * An edge in the graph. Connects 2 nodes that have 2 or more "uses" - meaning
 * the node forms an intersection (or is the end of an OSM way). OSM nodes
 * with less than 2 uses become a shape point (lat,lng) along the edge.
 */
struct Edge {
  // index of the source (start) node of the edge
  uint32_t sourcenode_;

  // Index into the list of OSM way information
  uint32_t wayindex_;

  // Index of the first lat,lng into the GraphBuilder latlngs
  uint32_t llindex_;

  // Attributes needed to sort the edges
  struct EdgeAttributes {
    uint32_t llcount          : 16;
    uint32_t importance       : 3;
    uint32_t driveableforward : 1;
    uint32_t driveablereverse : 1;
    uint32_t traffic_signal   : 1;
    uint32_t forward_signal   : 1;
    uint32_t backward_signal  : 1;
    uint32_t link             : 1;
    uint32_t spare            : 7;
  };
  EdgeAttributes attributes;

  // index of the target (end) node of the edge
  uint32_t targetnode_;


  /**
   * Construct a new edge. Target node and additional lat,lngs will
   * be filled in later.
   * @param sourcenode   Start node of the edge
   * @param wayindex     Index into list of OSM ways
   * @param ll           Lat,lng at the start of the edge.
   */
  static Edge make_edge(const uint32_t sourcenode, const uint32_t wayindex,
       const uint32_t llindex, const OSMWay& way) {
    Edge e{sourcenode, wayindex, llindex};
    e.attributes.llcount = 1;
    e.attributes.importance = static_cast<uint32_t>(way.road_class());
    e.attributes.driveableforward = way.auto_forward();
    e.attributes.driveablereverse = way.auto_backward();
    e.attributes.link = way.link();
    return e;
  }

};

/**
 * Node within the graph
 */
struct Node {
  //the underlying osm node and attributes
  OSMNode node;
  //the graph edge that this node starts
  uint32_t start_of;
  //the graph edge that this node ends
  uint32_t end_of;
  //the graphid of the node
  GraphId graph_id;

  bool is_start() const {
    return start_of != - 1;
  }
  bool is_end() const {
    return end_of != - 1;
  }

};

/**
 * Class used to construct temporary data used to build the initial graph.
 */
class GraphBuilder {
 public:
  //not default constructable or copyable
  GraphBuilder() = delete;
  GraphBuilder(const GraphBuilder&) = delete;

  /**
   * Constructor
   */
  GraphBuilder(const boost::property_tree::ptree& pt);

  /**
   * Tell the builder to build the tiles from the provided datasource
   * and configs
   * @param  osmdata  OSM data used to build the graph.
   */
  void Build(OSMData& osmdata);

 protected:

  /**
   * Construct edges in the graph.
   * @param  osmdata  OSM data used to construct edges in the graph.
   * @param  tilesize Tile size in degrees.
   */
  void ConstructEdges(const OSMData& osmdata, const float tilesize);

  /**
   * Add a new node to the tile (based on the OSM node lat,lng). Return
   * the GraphId of the node.
   */
  GraphId AddNodeToTile(const OSMNode& osmnode, const uint32_t edgeindex, const bool link);

  /**
   * Get a reference to a node given its graph Id.
   * @param  id  GraphId of the node.
   * @return  Returns a const reference to the node information.
   */
  Node& GetNode(const baldr::GraphId& id);

  /**
   * Update road class / importance of links (ramps)
   */
  void ReclassifyLinks(const std::string& ways_file, DataQuality& stats);

  /**
   * Build tiles representing the local graph
   */
  void BuildLocalTiles(const uint8_t level, const OSMData& osmdata, const std::map<GraphId, size_t>& tiles, DataQuality& stats) const;

  static std::string GetRef(const std::string& way_ref,
                            const std::string& relation_ref);

  static std::vector<SignInfo> CreateExitSignInfoList(
      const GraphId& nodeid, const Node& node, const OSMWay& way,
      const OSMData& osmdata,
      const std::unordered_map<baldr::GraphId, std::string>& node_ref,
      const std::unordered_map<baldr::GraphId, std::string>& node_exit_to,
      const std::unordered_map<baldr::GraphId, std::string>& node_name);

  /**
   * Create the extended node information mapped by the node's GraphId.
   * This is needed since we do not keep osmnodeid around.
   * @param  osmdata  OSM data with the extended node information mapped by
   *                  OSM node Id.
   */
  void CreateNodeMaps(const OSMData& osmdata);

  /**
   * Update restrictions. Replace OSM node Ids with GraphIds.
   * @param  osmdata  Includes the restrictions
   */
  void UpdateRestrictions(OSMData& osmdata);

  // List of the tile levels to be created
  uint32_t level_;
  TileHierarchy tile_hierarchy_;

  //stores all the graph nodes in this file
  std::string nodes_file_;

  // Stores all the graph edges in this file
  std::string edges_file_;

  // How many threads to run
  const unsigned int threads_;
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHBUILDER_H
