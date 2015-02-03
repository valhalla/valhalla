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
#include <valhalla/mjolnir/osmdata.h>
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>
#include <valhalla/mjolnir/dataquality.h>

#include <valhalla/baldr/exitsigninfo.h>
#include "mjolnir/edgeinfobuilder.h"

namespace valhalla {
namespace mjolnir {

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
  std::vector<OSMLatLng> latlngs_;

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
       const OSMLatLng& ll, const OSMWay& way)
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
  void AddLL(const OSMLatLng& ll) {
    latlngs_.emplace_back(ll);
  }

  /**
   * Method to get the edge shape. Converts the stored lat,lng pairs into
   * a vector of PointLL.
   * @return  Returns a vector of PointLL so we can get the length of an edge.
   */
  std::vector<midgard::PointLL> shape() const;

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
 * Extend OSMNode to add edge data and GraphId - information added
 * by GraphBuilder.
 */
class Node : public OSMNode {
 public:
  /**
   * Constructor.
   */
  Node()
      : OSMNode() {
  }

  /**
   * Constructor with arguments
   */
  Node(const OSMNode& osmnode, const uint32_t edgeindex, const bool link) {
    set_latlng(osmnode.latlng());
    attributes_.v = osmnode.attributes();
    AddEdge(edgeindex, link);
  }

  /**
   * Set the graph Id
   * @param  graphid  Graph ID for this node
   */
  void set_graphid(const baldr::GraphId& graphid) {
    graphid_ = graphid;
  }

  /**
   * Get the graph Id of this node (after tiling).
   * @return  Returns the graph Id of this node.
   */
  const baldr::GraphId& graphid() const  {
    return graphid_;
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
      attributes_.fields.link_edge = true;
    } else {
      attributes_.fields.non_link_edge = true;
    }
    edges_.emplace_back(edgeindex);
  }

  /**
   * Get the list of edges.
   * @return  Returns the list of edge indexes used by the node.
   */
  const std::vector<uint32_t>& edges() const {
    return edges_;
  }

  /**
   * Get a mutable list of edge indexes.
   * @return  Returns the list of edge indexes used by the node.
   */
  std::vector<uint32_t>& mutable_edges() {
    return edges_;
  }

  /**
   * Get the number of edges beginning or ending at the node.
   * @return  Returns the number of edges.
   */
  uint32_t edge_count() const {
    return edges_.size();
  }

  /**
   * Get the non-link edge flag. True if any connected edge is not a
   * highway=*_link.
   */
  bool non_link_edge() const {
    return attributes_.fields.non_link_edge;
  }

  /**
   * Get the non-link edge flag. True if any connected edge is a
   * highway=*_link.
   */
  bool link_edge() const  {
    return attributes_.fields.link_edge;
  }

 protected:
  // GraphId of the node (after tiling)
  baldr::GraphId graphid_;

  // List of edges beginning or ending at the node
  std::vector<uint32_t> edges_;
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
   */
  void ConstructEdges(const OSMData& osmdata);

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
  void BuildLocalTiles(const uint8_t level,
                       const WayVector& ways,
                       const std::unordered_map<uint64_t, std::string>& node_ref,
                       const std::unordered_map<uint64_t, std::string>& node_exit_to,
                       const std::unordered_map<uint64_t, std::string>& node_name) const;

  static std::vector<ExitSignInfo> CreateExitSignInfoList(
      const uint64_t osmnodeid, const Node& node, const OSMWay& way,
      const std::unordered_map<uint64_t, std::string>& map_ref,
      const std::unordered_map<uint64_t, std::string>& map_name,
      const std::unordered_map<uint64_t, std::string>& map_exit_to);

  // List of the tile levels to be created
  TileHierarchy tile_hierarchy_;

  // Map that stores all the nodes read
  std::unordered_map<uint64_t, Node> nodes_;

  // Stores all the edges
  std::vector<Edge> edges_;

  // A place to keep each tile's nodes so that various threads can
  // write various tiles asynchronously
  std::unordered_map<GraphId, std::vector<uint64_t> > tilednodes_;

  // Data quality / statistics.
  std::unique_ptr<DataQuality> stats_;

  // How many threads to run
  const unsigned int threads_;
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHBUILDER_H
