#ifndef VALHALLA_MJOLNIR_NODE_EXPANDER_H_
#define VALHALLA_MJOLNIR_NODE_EXPANDER_H_

#include <map>
#include <string>
#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/midgard/sequence.h>
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace mjolnir {

// Absurd classification.
constexpr uint32_t kAbsurdRoadClass = 777777;

/**
 * An edge in the graph. Connects 2 nodes that have 2 or more "uses" - meaning
 * the node forms an intersection (or is the end of an OSM way). OSM nodes
 * with less than 2 uses become a shape point (lat,lng) along the edge.
 */
struct Edge {
  // Index into the list of OSM way information
  uint32_t wayindex_;

  // Index of the first lat,lng into the GraphBuilder latlngs
  uint32_t llindex_;

  // Attributes needed to sort the edges
  struct EdgeAttributes {
    uint64_t llcount : 16;
    uint64_t importance : 3;
    uint64_t driveableforward : 1;
    uint64_t driveablereverse : 1;
    uint64_t traffic_signal : 1;
    uint64_t forward_signal : 1;
    uint64_t backward_signal : 1;
    uint64_t stop_sign : 1;
    uint64_t forward_stop : 1;
    uint64_t backward_stop : 1;
    uint64_t yield_sign : 1;
    uint64_t forward_yield : 1;
    uint64_t backward_yield : 1;
    uint64_t direction : 1;
    uint64_t link : 1;
    uint64_t reclass_link : 1;
    uint64_t has_names : 1;
    uint64_t driveforward : 1; // For sorting in collect_node_edges
                               //  - set based on source node
    uint64_t shortlink : 1;    // true if this is a link edge and is
                               //   short enough it may be internal to
                               //   an intersection
    uint64_t driveable_ferry : 1;
    uint64_t reclass_ferry : 1; // Has edge been reclassified due to
                                // ferry connection
    uint64_t turn_channel : 1;  // Link edge should be a turn channel
    uint64_t way_begin : 1;     // True if first edge of way
    uint64_t way_end : 1;       // True if last edge of way
    uint64_t spare : 23;
  };
  EdgeAttributes attributes;

  // index of the source (start) node of the edge
  uint32_t sourcenode_;

  // index of the target (end) node of the edge
  uint32_t targetnode_;

  /**
   * For now you cant be valid if you dont have any shape
   * @return true if the edge has at least 1 shape point
   */
  bool is_valid() const {
    return attributes.llcount > 0;
  }

  /**
   * Construct a new edge. Target node and additional lat,lngs will
   * be filled in later.
   * @param sourcenode   Start node of the edge
   * @param wayindex     Index into list of OSM ways
   * @param ll           Lat,lng at the start of the edge.
   */
  static Edge make_edge(const uint32_t wayindex,
                        const uint32_t llindex,
                        const OSMWay& way,
                        const bool infer_turn_channels) {
    Edge e{wayindex, llindex};
    e.attributes.llcount = 1;
    e.attributes.importance = static_cast<uint32_t>(way.road_class());
    if (way.use() == baldr::Use::kEmergencyAccess) {
      // Temporary until all access values are set
      e.attributes.driveableforward = false;
      e.attributes.driveablereverse = false;
    } else {
      e.attributes.driveableforward = way.auto_forward();
      e.attributes.driveablereverse = way.auto_backward();
    }
    e.attributes.link = way.link();
    e.attributes.driveable_ferry =
        (way.ferry() || way.rail()) && (way.auto_forward() || way.auto_backward());
    e.attributes.reclass_link = false;
    e.attributes.reclass_ferry = false;
    e.attributes.has_names =
        (way.name_index_ != 0 || way.name_en_index_ != 0 || way.alt_name_index_ != 0 ||
         way.official_name_index_ != 0 || way.ref_index_ != 0 || way.int_ref_index_ != 0);

    // If this data has turn_channels set and we are not inferring turn channels then we need to
    // use the flag. Otherwise the turn_channel is set in the reclassify links.  Also, an edge can't
    // be a ramp and a turn channel.
    if (!infer_turn_channels && way.turn_channel() && !way.link())
      e.attributes.turn_channel = true;
    else
      e.attributes.turn_channel = false;
    return e;
  }

  /**
   * For sorting edges. By driveability (forward), importance, and
   * presence of names.
   * (TODO - end of simple restriction?)
   */
  bool operator<(const Edge& other) const {
    // Is this a loop?
    if (targetnode_ == other.targetnode_ && sourcenode_ == other.sourcenode_ &&
        sourcenode_ == targetnode_) {
      return false;
    }

    // Sort by driveability (forward, importance, has_names)
    bool d = attributes.driveforward;
    bool od = other.attributes.driveforward;
    if (d == od) {
      if (attributes.importance == other.attributes.importance) {
        // Equal importance - check presence of names
        if (attributes.has_names == other.attributes.has_names) {
          return llindex_ < other.llindex_;
        } else {
          return attributes.has_names > other.attributes.has_names;
        }
      } else {
        return attributes.importance < other.attributes.importance;
      }
    } else {
      return d > od;
    }
  }
};

/**
 * Node within the graph
 */
struct Node {
  // the underlying osm node and attributes
  OSMNode node;
  // the graph edge that this node starts
  uint32_t start_of;
  // the graph edge that this node ends
  uint32_t end_of;
  // the graphid of the node
  baldr::GraphId graph_id;

  bool is_start() const {
    return start_of != static_cast<uint32_t>(-1);
  }
  bool is_end() const {
    return end_of != static_cast<uint32_t>(-1);
  }
};

// collect all the edges that start or end at this node
struct node_bundle : Node {
  size_t node_count;
  size_t link_count;
  size_t non_link_count;
  size_t driveforward_count;

  // TODO: to enable two directed edges per loop edge turn this into an
  // unordered_multimap or just a list of pairs
  std::map<Edge, size_t> node_edges;

  node_bundle(const Node& other)
      : Node(other), node_count(0), link_count(0), non_link_count(0), driveforward_count(0) {
  }
};

/**
 * Collect node information and edges from the node.
 */
node_bundle collect_node_edges(const sequence<Node>::iterator& node_itr,
                               sequence<Node>& nodes,
                               sequence<Edge>& edges);

} // namespace mjolnir
} // namespace valhalla
#endif // VALHALLA_MJOLNIR_NODE_EXPANDER_H_
