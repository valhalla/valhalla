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
  // GraphId of the source (start) node of the edge
  baldr::GraphId sourcenode_;

  // GraphId of the target (end) node of the edge
  baldr::GraphId targetnode_;

  // Index into the list of OSM way information
  uint32_t wayindex_;

  // Attributes needed to sort the edges
  // Note - this doesn't change the size of the structure...
  struct EdgeAttributes {
    uint32_t llcount          : 16;
    uint32_t importance       : 3;
    uint32_t driveableforward : 1;
    uint32_t driveablereverse : 1;
    uint32_t link             : 1;
    uint32_t spare            : 10;
  };
  EdgeAttributes attributes;

  // Index of the first lat,lng into the GraphBuilder latlngs
  uint32_t llindex_;

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
  Edge(const baldr::GraphId& sourcenode, const uint32_t wayindex,
       const uint32_t llindex, const OSMWay& way)
      : sourcenode_(sourcenode),
        targetnode_{},
        wayindex_(wayindex),
        llindex_(llindex) {
    attributes.llcount = 1;
    attributes.importance = static_cast<uint32_t>(way.road_class());
    attributes.driveableforward = way.auto_forward();
    attributes.driveablereverse = way.auto_backward();
    attributes.link = way.link();
  }

 private:
  /**
   * Default constructor
   */
  Edge()
      : sourcenode_{},
        targetnode_{},
        attributes{},
        wayindex_(0),
        llindex_(0) {
  }
};

/**
 * Node within the graph
 */
struct Node {
  // List of edges connected to the node
  std::list<uint32_t> edges;

  // Node attributes
  NodeAttributes attributes;

  /**
   * Constructor.
   */
  Node()
      : attributes{} {
  }

  /**
   * Constructor with arguments
   */
  Node(const NodeAttributes& attr, const uint32_t edgeindex, const bool link)
      : attributes(attr) {
    AddEdge(edgeindex, link);
  }

  /**
   * Add an edge. Set flags to indicate a link and/or non-link edge
   * exists at the node.
   * @param  edgeindex  Index in the list of edges.
   * @param  link       Flag indicating whether this edge is a link
   *                    (highway=*_link)
   */
  void AddEdge(const uint32_t edgeindex, const bool link) {
    if (link) {
      attributes.link_edge = true;
    } else {
      attributes.non_link_edge = true;
    }

    // TODO - could insert into the list based on importance and
    // driveablity?
    edges.push_back(edgeindex);
  }

  /**
   * Get the number of edges beginning or ending at the node.
   * @return  Returns the number of edges.
   */
  uint32_t edge_count() const {
    return edges.size();
  }

  /**
   * Set the exit to flag
   */
  void set_exit_to(const bool exit_to) {
    attributes.exit_to = exit_to;
  }

  /**
   * Get the exit to flag
   */
  bool exit_to() const {
    return attributes.exit_to;
  }

  /**
   * Set the ref flag
   */
  void set_ref(const bool ref)  {
    attributes.ref = ref;
  }

  /**
   * Get the ref flag
   */
  bool ref() const {
    return attributes.ref;
  }

  /**
   * Get the name flag
   */
  bool name() const  {
    return attributes.name;
  }

  /**
   * Get the non-link edge flag. True if any connected edge is not a
   * highway=*_link.
   */
  bool non_link_edge() const {
    return attributes.non_link_edge;
  }

  /**
   * Get the non-link edge flag. True if any connected edge is a
   * highway=*_link.
   */
  bool link_edge() const  {
    return attributes.link_edge;
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
  void ConstructEdges(OSMData& osmdata, const float tilesize);

  /**
   * Add a new node to the tile (based on the OSM node lat,lng). Return
   * the GraphId of the node.
   */
  GraphId AddNodeToTile(const uint64_t osmnodeid, const OSMNode& osmnode,
                        const uint32_t edgeindex, const bool link);

  /**
   * Get a reference to a node given its graph Id.
   * @param  id  GraphId of the node.
   * @return  Returns a const reference to the node information.
   */
  Node& GetNode(const baldr::GraphId& id);

  /**
   * Update road class / importance of links (ramps)
   */
  void ReclassifyLinks(const WayVector& ways);

  /**
   * Get the best classification for any non-link edges from a node.
   * @param  node  Node - gets outbound edges from this node.
   * @return  Returns the best (most important) classification
   */
  uint32_t GetBestNonLinkClass(const Node& node) const;

  /**
   * Build tiles representing the local graph
   */
  void BuildLocalTiles(const uint8_t level, const OSMData& osmdata) const;

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

  // Vector to hold shape of edges. Each edge indexes into the vector
  std::vector<OSMLatLng> latlngs_;

  // Map of OSM node Ids to GraphIds
  std::unordered_map<uint64_t, GraphId> nodes_;

  // Map that stores all the refereence info on a node
  std::unordered_map<baldr::GraphId, std::string> node_ref_;

  // Map that stores all the exit to info on a node
  std::unordered_map<baldr::GraphId, std::string> node_exit_to_;

  // Map that stores all the name info on a node
  std::unordered_map<baldr::GraphId, std::string> node_name_;

  // Stores all the edges
  std::vector<Edge> edges_;

  // A place to keep each tile's nodes so that various threads can
  // write various tiles asynchronously
  std::unordered_map<GraphId, std::vector<Node> > tilednodes_;

  // Data quality / statistics.
  std::unique_ptr<DataQuality> stats_;

  // How many threads to run
  const unsigned int threads_;
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHBUILDER_H
