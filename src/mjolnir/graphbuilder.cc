
#include "mjolnir/graphbuilder.h"
#include "mjolnir/util.h"

#include <future>
#include <utility>
#include <thread>
#include <set>
#include <queue>
#include <unordered_map>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/polyline2.h>
#include <valhalla/midgard/tiles.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/signinfo.h>
#include <valhalla/baldr/graphreader.h>

#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/edgeinfobuilder.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;


namespace {

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
    uint32_t llcount          : 16;
    uint32_t importance       : 3;
    uint32_t driveableforward : 1;
    uint32_t driveablereverse : 1;
    uint32_t traffic_signal   : 1;
    uint32_t forward_signal   : 1;
    uint32_t backward_signal  : 1;
    uint32_t link             : 1;
    uint32_t reclass_link     : 1;
    uint32_t has_names        : 1;
    uint32_t driveforward     : 1;   // For sorting in collect_node_edges
                                     //  - set based on source node
    uint32_t shortlink        : 1;   // true if this is a link edge and is
                                     //   short enough it may be internal to
                                     //   an intersection
    uint32_t driveable_ferry  : 1;
    uint32_t reclass_ferry    : 1;   // Has edge been reclassified due to
                                     // ferry connection
    uint32_t spare            : 1;
  };
  EdgeAttributes attributes;

  // index of the source (start) node of the edge
  uint32_t sourcenode_;

  // index of the target (end) node of the edge
  uint32_t targetnode_;

  /**
   * Construct a new edge. Target node and additional lat,lngs will
   * be filled in later.
   * @param sourcenode   Start node of the edge
   * @param wayindex     Index into list of OSM ways
   * @param ll           Lat,lng at the start of the edge.
   */
  static Edge make_edge(const uint32_t wayindex,
       const uint32_t llindex, const OSMWay& way) {
    Edge e{wayindex, llindex};
    e.attributes.llcount = 1;
    e.attributes.importance = static_cast<uint32_t>(way.road_class());
    if (way.use() == Use::kEmergencyAccess) {
      // Temporary until all access values are set
      e.attributes.driveableforward = false;
      e.attributes.driveablereverse = false;
    } else {
      e.attributes.driveableforward = way.auto_forward();
      e.attributes.driveablereverse = way.auto_backward();
    }
    e.attributes.link = way.link();
    e.attributes.driveable_ferry = (way.ferry() || way.rail()) &&
                         (way.auto_forward() || way.auto_backward());
    e.attributes.reclass_link = false;
    e.attributes.reclass_ferry = false;
    e.attributes.has_names = (way.name_index_ != 0
                           || way.name_en_index_ != 0
                           || way.alt_name_index_ != 0
                           || way.official_name_index_ != 0
                           || way.ref_index_ != 0
                           || way.int_ref_index_ != 0);
    return e;
  }

  /**
   * For sorting edges. By driveability (forward), importance, and
   * presence of names.
   * (TODO - end of simple restriction?)
   */
  bool operator < (const Edge& other) const {
    // Is this a loop?
    if (targetnode_ == other.targetnode_ &&
        sourcenode_ == other.sourcenode_ &&
        sourcenode_ == targetnode_) {
      false;
    }

    // Sort by driveability (forward, importance, has_names)
    bool d  = attributes.driveforward;
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

// collect all the edges that start or end at this node
struct node_bundle : Node {
  size_t node_count;
  size_t link_count;
  //TODO: to enable two directed edges per loop edge turn this into an unordered_multimap or just a list of pairs
  std::map<Edge, size_t> node_edges;
  node_bundle(const Node& other):Node(other), node_count(0), link_count(0) {}
};
node_bundle collect_node_edges(const sequence<Node>::iterator& node_itr, sequence<Node>& nodes, sequence<Edge>& edges) {
  //copy out the first nodes attributes (as they are the correctly merged one)
  auto itr = node_itr;
  node_bundle bundle(*itr);
  Node node;
  //for each node with the same id (duplicate)
  for(; itr != nodes.end() && (node = *itr).node.osmid == bundle.node.osmid; ++itr) {
    ++bundle.node_count;
    if(node.is_start()) {
      auto edge_itr = edges[node.start_of];
      auto edge = *edge_itr;
      // Set driveforward - this edge is traversed in forward direction
      edge.attributes.driveforward = edge.attributes.driveableforward;
      bundle.node_edges.emplace(std::make_pair(edge, node.start_of));
      bundle.node.attributes_.link_edge = bundle.node.attributes_.link_edge || edge.attributes.link;
      bundle.node.attributes_.ferry_edge = bundle.node.attributes_.ferry_edge || edge.attributes.driveable_ferry;
      bundle.node.attributes_.shortlink |= edge.attributes.shortlink;
      // Do not count non-driveable (e.g. emergency service roads) as a
      // non-link edge or non-ferry edge
      if (edge.attributes.driveableforward || edge.attributes.driveablereverse) {
        bundle.node.attributes_.non_link_edge = bundle.node.attributes_.non_link_edge || !edge.attributes.link;
        bundle.node.attributes_.non_ferry_edge = bundle.node.attributes_.non_ferry_edge || !edge.attributes.driveable_ferry;
      }
      if (edge.attributes.link) {
        bundle.link_count++;
      }
    }
    if(node.is_end()) {
      auto edge_itr = edges[node.end_of];
      auto edge = *edge_itr;
      // Set driveforward - this edge is traversed in reverse direction
      edge.attributes.driveforward = edge.attributes.driveablereverse;
      bundle.node_edges.emplace(std::make_pair(edge, node.end_of));
      bundle.node.attributes_.link_edge = bundle.node.attributes_.link_edge || edge.attributes.link;
      bundle.node.attributes_.ferry_edge = bundle.node.attributes_.ferry_edge || edge.attributes.driveable_ferry;
      bundle.node.attributes_.shortlink |= edge.attributes.shortlink;
      // Do not count non-driveable (e.g. emergency service roads) as a non-link edge
      if (edge.attributes.driveableforward || edge.attributes.driveablereverse) {
        bundle.node.attributes_.non_link_edge = bundle.node.attributes_.non_link_edge || !edge.attributes.link;
        bundle.node.attributes_.non_ferry_edge = bundle.node.attributes_.non_ferry_edge || !edge.attributes.driveable_ferry;
      }
      if (edge.attributes.link) {
        bundle.link_count++;
      }
    }
  }
  return bundle;
}

/**
 * we need the nodes to be sorted by graphid and then by osmid to make a set of tiles
 * we also need to then update the egdes that pointed to them
 *
 */
std::map<GraphId, size_t> SortGraph(const std::string& nodes_file,
                                    const std::string& edges_file,
                                    const TileHierarchy& tile_hierarchy,
                                    const uint8_t level) {
  LOG_INFO("Sorting graph...");

  // Sort nodes by graphid then by osmid, so its basically a set of tiles
  sequence<Node> nodes(nodes_file, false);
  nodes.sort(
    [&tile_hierarchy, &level](const Node& a, const Node& b) {
      if(a.graph_id == b.graph_id)
        return a.node.osmid < b.node.osmid;
      return a.graph_id < b.graph_id;
    }
  );
  //run through the sorted nodes, going back to the edges they reference and updating each edge
  //to point to the first (out of the duplicates) nodes index. at the end of this there will be
  //tons of nodes that no edges reference, but we need them because they are the means by which
  //we know what edges connect to a given node from the nodes perspective
  sequence<Edge> edges(edges_file, false);
  uint32_t run_index = 0;
  uint32_t node_index = 0;
  size_t node_count = 0;
  Node last_node{};
  std::map<GraphId, size_t> tiles;
  nodes.transform(
    [&nodes, &edges, &run_index, &node_index, &node_count, &last_node, &tiles](Node& node) {
      //remember if this was a new tile
      if(node_index == 0 || node.graph_id != (--tiles.end())->first) {
        tiles.insert({node.graph_id, node_index});
        node.graph_id.fields.id = 0;
        run_index = node_index;
        ++node_count;
      }//but is it a new node
      else if(last_node.node.osmid != node.node.osmid) {
        node.graph_id.fields.id = last_node.graph_id.fields.id + 1;
        run_index = node_index;
        ++node_count;
      }//not new keep the same graphid
      else
        node.graph_id.fields.id = last_node.graph_id.fields.id;

      //if this node marks the start of an edge, go tell the edge where the first node in the series is
      if(node.is_start()) {
        auto element = edges[node.start_of];
        auto edge = *element;
        edge.sourcenode_ = run_index;
        element = edge;
      }
      //if this node marks the end of an edge, go tell the edge where the first node in the series is
      if(node.is_end()) {
        auto element = edges[node.end_of];
        auto edge = *element;
        edge.targetnode_ = run_index;
        element = edge;
      }

      //next node
      last_node = node;
      ++node_index;
      return node;
    }
  );

  LOG_INFO("Finished with " + std::to_string(node_count) + " graph nodes");
  return tiles;
}

// Construct edges in the graph and assign nodes to tiles.
void ConstructEdges(const OSMData& osmdata, const std::string& ways_file,
          const std::string& way_nodes_file,
          const std::string& nodes_file,
          const std::string& edges_file, const float tilesize,
          const std::function<GraphId (const OSMNode&)>& graph_id_predicate) {
  LOG_INFO("Creating graph edges from ways...")

  //so we can read ways and nodes and write edges
  sequence<OSMWay> ways(ways_file, false);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  sequence<Edge> edges(edges_file, true);
  sequence<Node> nodes(nodes_file, true);

  // Method to get length of an edge (used to find short link edges)
  const auto Length = [&way_nodes](const size_t idx1, const OSMNode& node2) {
    auto node1 = (*way_nodes[idx1]).node;
    PointLL a(node1.lng, node1.lat);
    PointLL b(node2.lng, node2.lat);
    return a.Distance(b);
  };

  // For each way traversed via the nodes
  uint32_t edgeindex = 0;
  GraphId graphid;
  size_t current_way_node_index = 0;
  while (current_way_node_index < way_nodes.size()) {
    // Grab the way and its first node
    auto way_node = *way_nodes[current_way_node_index];
    const auto way = *ways[way_node.way_index];
    const auto first_way_node_index = current_way_node_index;
    const auto last_way_node_index = first_way_node_index + way.node_count() - 1;

    // Remember this edge starts here
    Edge edge = Edge::make_edge(way_node.way_index, current_way_node_index, way);

    // Remember this node as starting this edge
    way_node.node.attributes_.link_edge = way.link();
    way_node.node.attributes_.non_link_edge = !way.link();
    nodes.push_back({way_node.node, static_cast<uint32_t>(edges.size()), static_cast<uint32_t>(-1), graph_id_predicate(way_node.node)});

    // Iterate through the nodes of the way until we find an intersection
    while(current_way_node_index < way_nodes.size()) {
      // Get the next shape point on this edge
      way_node = *way_nodes[++current_way_node_index];
      edge.attributes.llcount++;

      // If its an intersection or the end of the way it's a node of the road network graph
      if (way_node.node.intersection()) {

        // Finish off this edge
        edge.attributes.shortlink = (way.link() &&
                  Length(edge.llindex_, way_node.node) < kMaxInternalLength);
        way_node.node.attributes_.link_edge = way.link();
        way_node.node.attributes_.non_link_edge = !way.link();
        nodes.push_back({way_node.node, static_cast<uint32_t>(-1), static_cast<uint32_t>(edges.size()), graph_id_predicate(way_node.node)});
        edges.push_back(edge);

        // Start a new edge if this is not the last node in the way
        if (current_way_node_index != last_way_node_index) {
          edge = Edge::make_edge(way_node.way_index, current_way_node_index, way);
          sequence<Node>::iterator element = --nodes.end();
          auto node = *element;
          node.start_of = edges.size();
          element = node;
        }// This was the last shape point in the way
        else {
          ++current_way_node_index;
          break;
        }
      }// If this edge has a signal not at a intersection
      else if (way_node.node.traffic_signal()) {
        edge.attributes.traffic_signal = true;
        edge.attributes.forward_signal = way_node.node.forward_signal();
        edge.attributes.backward_signal = way_node.node.backward_signal();
      }
    }
  }
  LOG_INFO("Finished with " + std::to_string(edges.size()) + " graph edges");
}


/**
 * Get the best classification for any driveable non-link edges from a node.
 * @param  edges The file backed list of edges in the graph.
 * @return  Returns the best (most important) classification
 */
uint32_t GetBestNonLinkClass(const std::map<Edge, size_t>& edges) {
  uint32_t bestrc = kAbsurdRoadClass;
  for (const auto& edge : edges) {
    if (!edge.first.attributes.link &&
        (edge.first.attributes.driveableforward ||
         edge.first.attributes.driveablereverse)) {
      if (edge.first.attributes.importance < bestrc) {
        bestrc = edge.first.attributes.importance;
      }
    }
  }
  return bestrc;
}

// Reclassify links (ramps and turn channels). OSM usually classifies links as
// the best classification, while to more effectively create shortcuts it is
// better to "downgrade" link edges to the lower classification.
void ReclassifyLinks(const std::string& ways_file,
                     const std::string& nodes_file,
                     const std::string& edges_file,
                     DataQuality& stats) {
  LOG_INFO("Reclassifying link graph edges...")

  uint32_t count = 0;
  std::unordered_set<size_t> visitedset;  // Set of visited nodes
  std::unordered_set<size_t> expandset;   // Set of nodes to expand
  std::list<size_t> linkedgeindexes;     // Edge indexes to reclassify
  std::multiset<uint32_t> endrc;           //
  sequence<OSMWay> ways(ways_file, false);
  sequence<Edge> edges(edges_file, false);
  sequence<Node> nodes(nodes_file, false);
  //std::set<size_t> indices;

  //try to expand
  auto expand = [&expandset, &endrc, &nodes, &edges, &visitedset] (const Edge& edge, const sequence<Node>::iterator& node_itr) {
    auto end_node_itr = (edge.sourcenode_ == node_itr.position() ? nodes[edge.targetnode_] : node_itr);
    auto end_node_bundle = collect_node_edges(end_node_itr, nodes, edges);

    // Add the classification if there is a driveable non-link edge
    if (end_node_bundle.node.attributes_.non_link_edge) {
      endrc.insert(GetBestNonLinkClass(end_node_bundle.node_edges));

      // Expand if the link count > 1 and a "short link" is present (could be
      // an internal intersection link). Do not want to expand in cases where
      // an exit ramp crosses onto an entrance back onto the same highway
      // that was exited. But there are cases with internal intersection links
      // that we do need to expand.
      if (end_node_bundle.link_count > 1 &&
          end_node_bundle.node.attributes_.shortlink &&
          visitedset.find(end_node_itr.position()) == visitedset.end()) {
       expandset.insert(end_node_itr.position());
      }
    } else if (visitedset.find(end_node_itr.position()) == visitedset.end()) {
      expandset.insert(end_node_itr.position());
    }
  };

  //for each node
  sequence<Node>::iterator node_itr = nodes.begin();
  while (node_itr != nodes.end()) {
    // If the node has a both links and non links at it
    auto bundle = collect_node_edges(node_itr, nodes, edges);
    if (bundle.node.attributes_.link_edge && bundle.node.attributes_.non_link_edge) {
      // Get the highest classification of non-link edges at this node
      endrc = { GetBestNonLinkClass(bundle.node_edges) };

      // Expand from all link edges
      for (const auto& startedge : bundle.node_edges) {
        // Get the edge information. Skip non-link edges and link edges
        // already reclassified
        if (!startedge.first.attributes.link ||
             startedge.first.attributes.reclass_link) {
          continue;
        }

        // Clear the visited set and start expanding at the end of this edge
        visitedset = {};
        expandset = {};
        linkedgeindexes = { startedge.second };
        expand(startedge.first, node_itr);

        // Expand edges until all paths reach a node that has a non-link and
        // only one link edge
        while (!expandset.empty()) {
          // Expand all edges from this node and pop from expand set
          auto expand_node_itr = nodes[*expandset.begin()];
          auto expanded = collect_node_edges(expand_node_itr, nodes, edges);
          visitedset.insert(expand_node_itr.position());
          expandset.erase(expandset.begin());
          for (const auto& expandededge : expanded.node_edges) {
            // Do not allow use of the start edge or any non-link edge
            if (expandededge.second == startedge.second ||
                !expandededge.first.attributes.link) {
              continue;
            }
            // Expand from end node of this link edge
            expand(expandededge.first, expand_node_itr);
          }
        }

        // Once expand list is empty - mark all link edges encountered
        // with the specified classification / importance and break out
        // of this loop (can still expand other ramp edges
        // from the starting node
        // Make sure this connects...
        if (endrc.size() < 2) {
          stats.AddIssue(kUnconnectedLinkEdge, GraphId(), (*ways[startedge.first.wayindex_]).way_id(), 0);
        }
        else {
          // Set to the value of the 2nd best road class of all
          // connections. This protects against downgrading links
          // when branches occur.
          uint32_t rc = *(++endrc.cbegin());
          for (auto idx : linkedgeindexes) {
            sequence<Edge>::iterator element = edges[idx];
            auto edge = *element;
            edge.attributes.reclass_link = true;
            if (rc > edge.attributes.importance) {
              edge.attributes.importance = rc;
              element = edge;
              //indices.insert(idx);
              count++;
            }
          }
        }
      }
    }

    //go to the next node
    node_itr += bundle.node_count;
  }

  /*for(const auto& b : indices)
    LOG_INFO(std::to_string(b));*/

  LOG_INFO("Finished with " + std::to_string(count) + " reclassified.");
}

/**
 * Get the best classification for any driveable non-ferry and non-link
 * edges from a node. Skip any reclassified ferry edges
 * @param  edges The file backed list of edges in the graph.
 * @return  Returns the best (most important) classification
 */
uint32_t GetBestNonFerryClass(const std::map<Edge, size_t>& edges) {
  uint32_t bestrc = kAbsurdRoadClass;
  for (const auto& edge : edges) {
    if (!edge.first.attributes.driveable_ferry &&
        !edge.first.attributes.link &&
        !edge.first.attributes.reclass_ferry &&
        (edge.first.attributes.driveableforward ||
         edge.first.attributes.driveablereverse)) {
      if (edge.first.attributes.importance < bestrc) {
        bestrc = edge.first.attributes.importance;
      }
    }
  }
  return bestrc;
}

// NodeLabel - for simple shortest path
struct NodeLabel {
  float cost;
  uint32_t node_index;
  uint32_t pred_node_index;

  NodeLabel(const float c, const uint32_t n, const uint32_t p)
      :  cost(c),
         node_index(n),
         pred_node_index(p) {
  }
};

// Unreached - not yet encountered in search
constexpr uint32_t kUnreached = 0;

// Permanent - shortest path to this edge has been found
constexpr uint32_t kPermanent = 1;

// Temporary - edge has been encountered but there could
// still be a shorter path to this node. This node will
// be "adjacent" to an node that is permanently labeled.
constexpr uint32_t kTemporary = 2;

// Store the node label status and its index in the NodeLabels list
struct NodeStatusInfo {
  uint32_t set;
  uint32_t index;
  NodeStatusInfo() : set(kUnreached), index(0) { }
  NodeStatusInfo(const uint32_t s, const uint32_t idx)
      : set(s),
        index(idx){
  }
};

// Cost comparator for priority_queue
class CompareCost {
public:
  bool operator()(const std::pair<float, uint32_t>& n1,
                  const std::pair<float, uint32_t>& n2) {
    return n1.first > n2.first;
  }
};

uint32_t ShortestPath(const uint32_t start_node_idx,
                      const uint32_t node_idx,
                      sequence<OSMWay>& ways,
                      sequence<OSMWayNode>& way_nodes,
                      sequence<Edge>& edges,
                      sequence<Node>& nodes,
                      const bool inbound, const uint32_t rc) {
  // Method to get the shape for an edge - since LL is stored as a pair of
  // floats we need to change into PointLL to get length of an edge
  const auto EdgeShape = [&way_nodes](size_t idx, const size_t count) {
    std::vector<PointLL> shape;
    shape.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      auto node = (*way_nodes[idx++]).node;
      shape.emplace_back(node.lng, node.lat);
    }
    return shape;
  };

  // Map to store the status and index of nodes that have been encountered.
  // Any unreached nodes are not added to the map.
  std::unordered_map<uint32_t, NodeStatusInfo> node_status;
  std::vector<NodeLabel> node_labels;

  // Priority queue for the adjacency list
  std::priority_queue<std::pair<float, uint32_t>,
      std::vector<std::pair<float, uint32_t>>, CompareCost> adjset;

  // Add node to list of node labels, set the node status and add
  // to the adjacency set
  uint32_t nodelabel_index = 0;
  node_labels.emplace_back(0.0f, node_idx, node_idx);
  node_status[node_idx] = { kTemporary, nodelabel_index };
  adjset.push({0.0f, nodelabel_index});
  nodelabel_index++;

  // Expand edges until a node connected to specified road classification
  // is reached
  uint32_t n = 0;
  uint32_t index = 0;
  while (!adjset.empty()) {
    // Get the next node from the adjacency list/priority queue. Gets its
    // current cost and index
    const auto& expand_node = adjset.top();
    float current_cost = expand_node.first;
    index = expand_node.second;
    uint32_t node_index = node_labels[index].node_index;
    adjset.pop();

    // Skip if already labeled - this can happen if an edge is already in
    // adj. list and a lower cost is found
    if (node_status[node_index].set == kPermanent) {
      continue;
    }

    // Expand all edges from this node
    auto expand_node_itr = nodes[node_index];
    auto expanded = collect_node_edges(expand_node_itr, nodes, edges);

    // We are finished if node has RC <= rc (and beyond first edge
    if (n > 0 && GetBestNonFerryClass(expanded.node_edges) <= rc) {
      break;
    }
    n++;

    // Label the node as done/permanent
    node_status[node_index] = { kPermanent, index};

    // Expand edges. Skip ferry edges and non-driveable edges (based on
    // the inbound flag).
    for (const auto& expandededge : expanded.node_edges) {
      // Skip any ferry edge and any edge that includes the start node index
      const auto& edge = expandededge.first;
      if (edge.attributes.driveable_ferry ||
          edge.sourcenode_ == start_node_idx ||
          edge.targetnode_ == start_node_idx)
        continue;

      // Skip non-driveable edges (based on inbound flag)
      const OSMWay w = *ways[edge.wayindex_];
      bool forward = (edge.sourcenode_ == node_index);
      if (forward) {
        if (( inbound && !edge.attributes.driveablereverse) ||
            (!inbound && !edge.attributes.driveableforward)) {
          continue;
        }
      } else {
        if (( inbound && !edge.attributes.driveableforward) ||
            (!inbound && !edge.attributes.driveablereverse)) {
          continue;
        }
      }

      // Get the end node. Skip if already permanently labeled or this
      // edge is a loop
      uint32_t endnode = forward ? edge.targetnode_ : edge.sourcenode_;
      if (node_status[endnode].set == kPermanent || endnode == node_index) {
        continue;
      }

      // Get cost - need the length and speed of the edge
      auto shape = EdgeShape(edge.llindex_, edge.attributes.llcount);
      float cost = current_cost + (PointLL::Length(shape) * 3.6f) / w.speed();

      // Check if already in adj set - skip if cost is higher than prior path
      if (node_status[endnode].set == kTemporary) {
        uint32_t endnode_idx = node_status[endnode].index;
        if (node_labels[endnode_idx].cost < cost) {
          continue;
        }
      }

      // Add to the node labels and adjacency set. Skip if this is a loop.
      node_labels.emplace_back(cost, endnode, node_index);
      node_status[endnode] = { kTemporary, nodelabel_index };
      adjset.push({cost, nodelabel_index});
      nodelabel_index++;
    }
  }

  // If only one label we have immediately found an edge with proper
  // classification - or we cannot expand due to driveability
  if (node_labels.size() == 1) {
      LOG_DEBUG("Only 1 edge reclassified");
    return 0;
  }

  // Trace shortest path backwards and upgrade edge classifications
  uint32_t count = 0;
  while (true) {
    // Get the edge between this node and the predecessor
    uint32_t idx = node_labels[index].node_index;
    uint32_t pred_node = node_labels[index].pred_node_index;
    auto expand_node_itr = nodes[idx];
    auto bundle2 = collect_node_edges(expand_node_itr, nodes, edges);
    for (auto& edge : bundle2.node_edges) {
      if (edge.first.sourcenode_ == pred_node ||
          edge.first.targetnode_ == pred_node) {
        sequence<Edge>::iterator element = edges[edge.second];
        auto update_edge = *element;
        if (update_edge.attributes.importance > rc) {
          update_edge.attributes.importance = rc;
          update_edge.attributes.reclass_ferry = true;
          element = update_edge;
          count++;
        }
      }
    }

    // Get the predecessor - break once we have found the start node
    if (pred_node == node_idx) {
      break;
    }
    index = node_status[pred_node].index;
  }
  return count;
}

// Check if the ferry included in this node bundle is short. Must be
// just one edge and length < 2 km
bool ShortFerry(const uint32_t node_index, node_bundle& bundle,
                sequence<Edge>& edges, sequence<Node>& nodes,
                sequence<OSMWay>& ways,
                sequence<OSMWayNode>& way_nodes) {
  // Method to get the shape for an edge - since LL is stored as a pair of
  // floats we need to change into PointLL to get length of an edge
  const auto EdgeShape = [&way_nodes](size_t idx, const size_t count) {
    std::vector<PointLL> shape;
    shape.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      auto node = (*way_nodes[idx++]).node;
      shape.emplace_back(node.lng, node.lat);
    }
    return shape;
  };
  uint32_t wayid = 0;
  bool short_edge = false;
  for (const auto& edge : bundle.node_edges) {
    // Check ferry edge. If the end node has a non-ferry edge check
    // the length of the edge
    if (edge.first.attributes.driveable_ferry) {
      uint32_t endnode = (edge.first.sourcenode_ == node_index) ?
                          edge.first.targetnode_ : edge.first.sourcenode_;
      auto end_node_itr = nodes[endnode];
      auto bundle2 = collect_node_edges(end_node_itr, nodes, edges);
      if (bundle2.node.attributes_.non_ferry_edge) {
        auto shape = EdgeShape(edge.first.llindex_, edge.first.attributes.llcount);
        if (PointLL::Length(shape) < 2000.0f) {
          const OSMWay w = *ways[edge.first.wayindex_];
          wayid = w.way_id();
          short_edge = true;
        }
      } else {
        short_edge = false;
      }
    }
  }
  if (short_edge) {
    LOG_DEBUG("Skip short ferry: way_id = " + std::to_string(wayid));
  }
  return short_edge;
}

// Reclassify links (ramps and turn channels). OSM usually classifies links as
// the best classification, while to more effectively create shortcuts it is
// better to "downgrade" link edges to the lower classification.
void ReclassifyFerryConnections(const std::string& ways_file,
                                const std::string& way_nodes_file,
                                const std::string& nodes_file,
                                const std::string& edges_file,
                                const uint32_t rc,
                                DataQuality& stats) {
  LOG_INFO("Reclassifying ferry connection graph edges...");

  sequence<OSMWay> ways(ways_file, false);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  sequence<Edge> edges(edges_file, false);
  sequence<Node> nodes(nodes_file, false);

  // Need to expand from the end of the ferry until we meet a road with the
  // specified classification. Want to do simple shortest path (time based
  // only) and obey driveability.

  // Iterate through nodes and find any that connect to both a ferry and a
  // regular (non-ferry) edge. Skip short ferry edges (river crossing?)
  uint32_t ferry_endpoint_count = 0;
  uint32_t total_count = 0;
  sequence<Node>::iterator node_itr = nodes.begin();
  while (node_itr != nodes.end()) {
    auto bundle = collect_node_edges(node_itr, nodes, edges);
    if (bundle.node.attributes_.ferry_edge &&
        bundle.node.attributes_.non_ferry_edge &&
        GetBestNonFerryClass(bundle.node_edges) > rc &&
        !ShortFerry(node_itr.position(), bundle, edges,
                    nodes, ways, way_nodes)) {
      // Form shortest path from node along each edge connected to the ferry,
      // track until the specified RC is reached
      bool oneway_reverse = false;
      for (const auto& edge : bundle.node_edges) {
        // Skip ferry edges and non-driveable edges
        if (edge.first.attributes.driveable_ferry ||
          (!edge.first.attributes.driveablereverse &&
           !edge.first.attributes.driveableforward)) {
          continue;
        }

        // Expand/reclassify from the end node of this edge.
        uint32_t end_node_idx = (edge.first.sourcenode_ == node_itr.position()) ?
                    edge.first.targetnode_ : edge.first.sourcenode_;

        // Check if edge is oneway towards the ferry or outbound from the
        // ferry. If edge is drivable both ways we need to expand it twice-
        // once with a driveable path towards the ferry and once with a
        // driveable path away from the ferry
        if (edge.first.attributes.driveableforward ==
            edge.first.attributes.driveablereverse) {
          // Driveable in both directions - get an inbound path and an
          // outbound path.
          total_count += ShortestPath(node_itr.position(), end_node_idx, ways,
                            way_nodes,edges, nodes, true, rc);
          total_count += ShortestPath(node_itr.position(), end_node_idx, ways,
                            way_nodes, edges, nodes, false, rc);
        } else {
          // Check if oneway inbound to the ferry
          bool inbound = (edge.first.sourcenode_ == node_itr.position()) ?
                          edge.first.attributes.driveablereverse :
                          edge.first.attributes.driveableforward;
          total_count += ShortestPath(node_itr.position(), end_node_idx, ways,
                            way_nodes, edges, nodes, inbound, rc);
        }
        ferry_endpoint_count++;

        // Reclassify the first/start edge. Do this AFTER finding shortest path so
        // we do not immediately determine we hit the specified classification
        sequence<Edge>::iterator element = edges[edge.second];
        auto update_edge = *element;
        update_edge.attributes.importance = rc;
        element = update_edge;
        total_count++;
      }
    }

    // Go to the next node
    node_itr += bundle.node_count;
  }
  LOG_INFO("Finished ReclassifyFerryEdges: ferry_endpoint_count = " +
           std::to_string(ferry_endpoint_count) + ", " +
           std::to_string(total_count) + " edges reclassified.");
}

/*
struct DuplicateEdgeInfo {
  uint32_t edgeindex;
  uint32_t length;

  DuplicateEdgeInfo() : edgeindex(0), length(0) { }
  DuplicateEdgeInfo(const uint32_t idx, const uint32_t l)
      : edgeindex(idx),
        length(l) {
  }
};

void CheckForDuplicates(const GraphId& nodeid, const Node& node,
                const std::vector<uint32_t>& edgelengths,
                const std::unordered_map<GraphId, std::vector<Node>>& nodes,
                const std::vector<Edge>& edges,
                const std::vector<OSMWay>& ways, std::atomic<DataQuality*>& stats) {
  uint32_t edgeindex;
  GraphId endnode;
  std::unordered_map<GraphId, DuplicateEdgeInfo> endnodes;
  uint32_t n = 0;
  for (auto edgeindex : node.edges) {
    const Edge& edge = edges[edgeindex];
    if (edge.sourcenode_ == nodeid) {
      endnode = edge.targetnode_;
    } else {
      endnode = edge.sourcenode_;
    }

    // Check if the end node is already in the set of edges from this node
    const auto en = endnodes.find(endnode);
    if (en != endnodes.end() && en->second.length == edgelengths[n]) {
      uint64_t wayid1 = ways[edges[en->second.edgeindex].wayindex_].way_id();
      uint64_t wayid2 = ways[edges[edgeindex].wayindex_].way_id();
      (*stats).AddIssue(kDuplicateWays, GraphId(), wayid1, wayid2);
    } else {
      endnodes.emplace(std::piecewise_construct,
                       std::forward_as_tuple(endnode),
                       std::forward_as_tuple(edgeindex, edgelengths[n]));
    }
    n++;
  }
}
*/
uint32_t CreateSimpleTurnRestriction(const uint64_t wayid, const size_t endnode,
    sequence<Node>& nodes, sequence<Edge>& edges, const OSMData& osmdata,
    sequence<OSMWay>& ways, DataQuality& stats) {

  auto res = osmdata.restrictions.equal_range(wayid);
  if (res.first == osmdata.restrictions.end()) {
    return 0;
  }

  // Edge is the from edge of a restriction. Find all TRs (if any)
  // through the target (end) node of this directed edge.
  auto node_itr = nodes[endnode];
  auto node = *node_itr;
  std::vector<OSMRestriction> trs;
  for (auto r = res.first; r != res.second; ++r) {
    if (r->second.via() == node.node.osmid) {
      if (r->second.day_on() != DOW::kNone) {
        stats.timedrestrictions++;
      } else {
        trs.push_back(r->second);
      }
    }
  }
  if (trs.empty()) {
    return 0;
  }

  // Get the way Ids of the edges at the endnode
  std::vector<uint64_t> wayids;
  auto bundle = collect_node_edges(node_itr, nodes, edges);
  for (const auto& edge : bundle.node_edges) {
    wayids.push_back((*ways[edge.first.wayindex_]).osmwayid_);
  }

  // There are some cases where both ONLY and NO restriction types are
  // present. Allow this. Iterate through all restrictions and set the
  // restriction mask to include the indexes of restricted turns.
  uint32_t mask = 0;
  for (const auto& tr : trs) {
    switch (tr.type()) {
    case RestrictionType::kNoLeftTurn:
    case RestrictionType::kNoRightTurn:
    case RestrictionType::kNoStraightOn:
    case RestrictionType::kNoUTurn:
      // Iterate through the edge wayIds until the matching to way Id is found
      for (uint32_t idx = 0, n = wayids.size(); idx < n; idx++) {
        if (wayids[idx] == tr.to()) {
          mask |= (1 << idx);
          break;
        }
      }
      break;

    case RestrictionType::kOnlyRightTurn:
    case RestrictionType::kOnlyLeftTurn:
    case RestrictionType::kOnlyStraightOn:
      // Iterate through the edge wayIds - any non-matching edge is added
      // to the turn restriction
      for (uint32_t idx = 0, n = wayids.size(); idx < n; idx++) {
        if (wayids[idx] != tr.to()) {
          mask |= (1 << idx);
        }
      }
      break;
    }
  }

  // Return the restriction mask
  return mask;
}

// Walk the shape and look for any empty tiles that the shape intersects
void CheckForIntersectingTiles(const GraphId& tile1, const GraphId& tile2,
                const Tiles& tiling, std::vector<PointLL>& shape,
                DataQuality& stats) {
  // Walk the shape segments until we are outside
  uint32_t current_tile = tile1.tileid();
  auto shape1 = shape.begin();
  auto shape2 = shape1 + 1;
  while (shape2 < shape.end()) {
    uint32_t next_tile = tiling.TileId(shape2->lat(), shape2->lng());
    if (next_tile != current_tile) {
      // If a neighbor we can just add this tile
      if (tiling.AreNeighbors(current_tile, next_tile)) {
        stats.AddIntersectedTile(GraphId(next_tile, 2, 0));
      } else {
        // Not a neighbor - find any intermediate intersecting tiles

        // Get a bounding box or row, col surrounding the 2 tiles
        auto rc1 = tiling.GetRowColumn(current_tile);
        auto rc2 = tiling.GetRowColumn(next_tile);
        for (int32_t row = std::min(rc1.first, rc2.first);
             row <= std::max(rc1.first, rc2.first); row++) {
          for (int32_t col = std::min(rc1.second, rc2.second);
                       col <= std::max(rc1.second, rc2.second); col++) {
            // Get the tile Id for the row,col. Skip if either of the 2 tiles.
            int32_t tileid = tiling.TileId(row, col);
            GraphId id(tileid, 2, 0);
            if (tileid == current_tile || tileid == next_tile) {
              continue;
            }

            // Check if the shape segment intersects the tile
            if (tiling.TileBounds(tileid).Intersect(*shape1, *shape2)) {
              stats.AddIntersectedTile(id);
            }
          }
        }
      }
    }

    // Increment
    shape1 = shape2;
    shape2++;
  }
}

void BuildTileSet(const std::string& ways_file, const std::string& way_nodes_file,
    const std::string& nodes_file, const std::string& edges_file,
    const TileHierarchy& hierarchy, const OSMData& osmdata,
    std::map<GraphId, size_t>::const_iterator tile_start,
    std::map<GraphId, size_t>::const_iterator tile_end,
    std::promise<DataQuality>& result) {

  std::string thread_id = static_cast<std::ostringstream&>(std::ostringstream() << std::this_thread::get_id()).str();
  LOG_INFO("Thread " + thread_id + " started");

  sequence<OSMWay> ways(ways_file, false);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  sequence<Edge> edges(edges_file, false);
  sequence<Node> nodes(nodes_file, false);

  const auto& tl = hierarchy.levels().rbegin();
  Tiles tiling = tl->second.tiles;

  // Method to get the shape for an edge - since LL is stored as a pair of
  // floats we need to change into PointLL to get length of an edge
  const auto EdgeShape = [&way_nodes](size_t idx, const size_t count) {
    std::vector<PointLL> shape;
    shape.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      auto node = (*way_nodes[idx++]).node;
      shape.emplace_back(node.lng, node.lat);
    }
    return shape;
  };

  // For each tile in the task
  bool added = false;
  DataQuality stats;
  for(; tile_start != tile_end; ++tile_start) {
    try {
      // What actually writes the tile
      GraphTileBuilder graphtile;
      GraphId tileid1 = tile_start->first.Tile_Base();

      // Iterate through the nodes
      uint32_t idx = 0;                 // Current directed edge index
      uint32_t directededgecount = 0;
      //for each node in the tile
      auto node_itr = nodes[tile_start->second];
      while (node_itr != nodes.end() && (*node_itr).graph_id.Tile_Base() == tileid1) {
        //amalgamate all the node duplicates into one and the edges that connect to it
        //this moves the iterator for you
        auto bundle = collect_node_edges(node_itr, nodes, edges);
        const auto& node = bundle.node;
        PointLL node_ll{node.lng, node.lat};

        // Look for potential duplicates
        //CheckForDuplicates(nodeid, node, edgelengths, nodes, edges, osmdata.ways, stats);

        // Build directed edges. Track the best classification/importance
        // of outbound edges from this node.
        uint32_t n = 0;
        RoadClass bestclass = RoadClass::kServiceOther;
        std::vector<DirectedEdgeBuilder> directededges;
        for (const auto& edge_pair : bundle.node_edges) {
          // Get the edge and way
          const Edge& edge = edge_pair.first;
          const OSMWay w = *ways[edge.wayindex_];

          // Get the shape for the edge and compute its length
          auto shape = EdgeShape(edge.llindex_, edge.attributes.llcount);
          uint32_t length = static_cast<uint32_t>(PointLL::Length(shape) + .5f);

          // Determine orientation along the edge (forward or reverse between
          // the 2 nodes). Check for edge error.
          bool forward = edge.sourcenode_ == node_itr.position();
          size_t source = edge.sourcenode_, target = edge.targetnode_;
          if (!forward)
            std::swap(source, target);

          // Validate speed
          uint32_t speed = static_cast<uint32_t>(w.speed());
          if (speed > kMaxSpeedKph) {
            LOG_WARN("Speed = " + std::to_string(speed) + " wayId= " +
                       std::to_string(w.way_id()));
            speed = kMaxSpeedKph;
          }

          // Infer cul-de-sac if a road edge is a loop and is low
          // classification. TODO - do we need length limit?
          Use use = w.use();
          RoadClass rc = static_cast<RoadClass>(edge.attributes.importance);
          if (use == Use::kRoad && source == target &&
              rc > RoadClass::kTertiary) {
            use = Use::kCuldesac;
            stats.culdesaccount++;
          }

          // Handle simple turn restrictions that originate from this
          // directed edge
          uint32_t restrictions = CreateSimpleTurnRestriction(w.way_id(),
            target, nodes, edges, osmdata, ways, stats);
          if (restrictions != 0)
            stats.simplerestrictions++;

          // traffic signal exists at an intersection node
          // OR
          // traffic signal exists at a non-intersection node
          // forward signal must exist if forward direction and vice versa.
          // if forward and backward signal flags are not set then only set for oneways.
          bool has_signal = (!forward && node.traffic_signal()) ||
            ((edge.attributes.traffic_signal) &&
            ((forward && edge.attributes.forward_signal) || (!forward && edge.attributes.backward_signal) ||
            (w.oneway() && !edge.attributes.forward_signal && !edge.attributes.backward_signal)));

          // Add a directed edge and get a reference to it
          directededges.emplace_back(w, (*nodes[target]).graph_id, forward, length,
                        speed, use, rc, n, has_signal, restrictions);
          DirectedEdgeBuilder& directededge = directededges.back();

          // Update the node's best class
          bestclass = std::min(bestclass, directededge.classification());

          // Check for updated ref from relations.
          std::string ref;
          auto iter = osmdata.way_ref.find(w.way_id());
          if (iter != osmdata.way_ref.end()) {
            if (w.ref_index() != 0)
              ref = GraphBuilder::GetRef(osmdata.ref_offset_map.name(w.ref_index()),iter->second);
          }

          // Add edge info to the tile and set the offset in the directed edge
          uint32_t edge_info_offset = graphtile.AddEdgeInfo(
            edge_pair.second, (*nodes[source]).graph_id, (*nodes[target]).graph_id,
            w.way_id(), shape, w.GetNames(ref, osmdata.ref_offset_map,
                                         osmdata.name_offset_map),
            added);
          directededge.set_edgeinfo_offset(edge_info_offset);

          // TODO - update logic so we limit the CreateExitSignInfoList calls
          // TODO - Also, we will have to deal with non ramp signs
          // Any exits for this directed edge? is auto and oneway?
          std::vector<SignInfo> exits = GraphBuilder::CreateExitSignInfoList(node, w, osmdata);
          if (!exits.empty() && (directededge.forwardaccess() & kAutoAccess)
               && directededge.link()) {
            graphtile.AddSigns(idx, exits);
            directededge.set_exitsign(true);
          }

          //TODO: If this was a loop edge we need its twin because this node wont be encountered again
          /*if(source == target) {
            idx++;
            n++;
            auto flipped = directededge.flipped();
            flipped.set_localedgeidx(n);
            directededges.emplace_back();
          }*/

          // If the end node is in a different tile and the tile is not
          // a neighboring tile then check for possible shape intersection with
          // empty tiles
          GraphId tileid2 = (*nodes[target]).graph_id.Tile_Base();
          if (tileid1 != tileid2 && !tiling.AreNeighbors(tileid1, tileid2)) {
            CheckForIntersectingTiles(tileid1, tileid2, tiling,
                     shape, stats);
          }

          // Increment the directed edge index within the tile
          idx++;
          n++;
        }

        // Set the node lat,lng, index of the first outbound edge, and the
        // directed edge count from this edge and the best road class
        // from the node. Increment directed edge count.
        NodeInfoBuilder nodebuilder(node_ll, bestclass, node.access_mask(),
                                    node.type(), (n == 1), node.traffic_signal());

        directededgecount += n;

        // Add node and directed edge information to the tile
        graphtile.AddNodeAndDirectedEdges(nodebuilder, directededges);

        // Increment the counts in the histogram
        stats.nodecount++;
        stats.directededge_count += directededges.size();
        stats.node_counts[directededges.size()]++;

        // Next node in the tile
        node_itr += bundle.node_count;
      }

      // Write the actual tile to disk
      graphtile.StoreTileData(hierarchy, tile_start->first);

      // Made a tile
      LOG_INFO((boost::format("Thread %1% wrote tile %2%: %3% bytes") % thread_id % tile_start->first % graphtile.size()).str());
    }// Whatever happens in Vegas..
    catch(std::exception& e) {
      // ..gets sent back to the main thread
      result.set_exception(std::current_exception());
      LOG_ERROR((boost::format("Thread %1% failed tile %2%: %3%") % thread_id % tile_start->first % e.what()).str());
      return;
    }
  }
  // Let the main thread see how this thread faired
  result.set_value(stats);
}

// Build tiles for the local graph hierarchy
void BuildLocalTiles(const unsigned int thread_count, const OSMData& osmdata,
  const std::string& ways_file, const std::string& way_nodes_file,
  const std::string& nodes_file, const std::string& edges_file,
  const std::map<GraphId, size_t>& tiles, const TileHierarchy& tile_hierarchy, DataQuality& stats) {

  LOG_INFO("Building " + std::to_string(tiles.size()) + " tiles with " + std::to_string(thread_count) + " threads...");

  // A place to hold worker threads and their results, be they exceptions or otherwise
  std::vector<std::shared_ptr<std::thread> > threads(thread_count);

  // Hold the results (DataQuality/stats) for the threads
  std::vector<std::promise<DataQuality> > results(threads.size());

  // Divvy up the work
  size_t floor = tiles.size() / threads.size();
  size_t at_ceiling = tiles.size() - (threads.size() * floor);
  std::map<GraphId, size_t>::const_iterator tile_start, tile_end = tiles.begin();

  // Atomically pass around stats info
  LOG_INFO(std::to_string(tiles.size()) + " tiles");
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    tile_start = tile_end;
    // Where the range ends
    std::advance(tile_end, tile_count);
    // Make the thread
    threads[i].reset(
      new std::thread(BuildTileSet,  std::cref(ways_file), std::cref(way_nodes_file),
                      std::cref(nodes_file), std::cref(edges_file), std::cref(tile_hierarchy),
                      std::cref(osmdata), tile_start, tile_end,
                      std::ref(results[i]))
    );
  }

  // Join all the threads to wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  LOG_INFO("Finished");

  // Check all of the outcomes and accumulate stats
  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      // Add statistics and log issues on this thread
      const auto& stat = result.get_future().get();
      stats.AddStatistics(stat);
      stat.LogIssues();
    }
    catch(std::exception& e) {
      //TODO: throw further up the chain?
    }
  }

  // Add "empty" tiles for any intersected tiles
  for (auto& empty_tile : stats.intersected_tiles) {
    if (!GraphReader::DoesTileExist(tile_hierarchy, empty_tile)) {
      GraphTileBuilder graphtile;
      LOG_INFO("Add empty tile for: " + std::to_string(empty_tile.tileid()));
      graphtile.StoreTileData(tile_hierarchy, empty_tile);
    }
  }
  LOG_INFO("Added " + std::to_string(stats.intersected_tiles.size()) +
           " empty, intersected tiles");
}

}

namespace valhalla {
namespace mjolnir {

// Build the graph from the input
void GraphBuilder::Build(const boost::property_tree::ptree& pt, const OSMData& osmdata,
    const std::string& ways_file, const std::string& way_nodes_file) {
  std::string nodes_file = "nodes.bin";
  std::string edges_file = "edges.bin";
  TileHierarchy tile_hierarchy(pt.get_child("hierarchy"));
  unsigned int threads = std::max(static_cast<unsigned int>(1),
                                  pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency()));
  const auto& tl = tile_hierarchy.levels().rbegin();
  uint8_t level = tl->second.level;

  // Make the edges and nodes in the graph
  ConstructEdges(osmdata, ways_file, way_nodes_file, nodes_file, edges_file, tl->second.tiles.TileSize(),
    [&tile_hierarchy, &level](const OSMNode& node) {
      return tile_hierarchy.GetGraphId({node.lng, node.lat}, level);
    }
  );

  // Line up the nodes and then re-map the edges that the edges to them
  auto tiles = SortGraph(nodes_file, edges_file, tile_hierarchy, level);

  // Reclassify links (ramps). Cannot do this when building tiles since the
  // edge list needs to be modified
  DataQuality stats;
  ReclassifyLinks(ways_file, nodes_file, edges_file, stats);

  // Reclassify ferry connection edges - use the highway classification cutoff
  RoadClass rc = RoadClass::kPrimary;
  for (auto& level : tile_hierarchy.levels()) {
    if (level.second.name == "highway") {
      rc = level.second.importance;
    }
  }
  ReclassifyFerryConnections(ways_file, way_nodes_file, nodes_file, edges_file,
                             static_cast<uint32_t>(rc), stats);

  // Build tiles at the local level. Form connected graph from nodes and edges.
  BuildLocalTiles(threads, osmdata, ways_file, way_nodes_file, nodes_file,
                  edges_file, tiles, tile_hierarchy, stats);

  stats.LogStatistics();
}


// Get highway refs from relations
std::string GraphBuilder::GetRef(const std::string& way_ref, const std::string& relation_ref) {
  bool found = false;
  std::string refs;
  std::vector<std::string> way_refs = GetTagTokens(way_ref); // US 51;I 57
  std::vector<std::string> refdirs = GetTagTokens(relation_ref);// US 51|north;I 57|north
  for (auto& ref : way_refs) {
    found = false;
    for (const auto& refdir : refdirs) {
      std::vector<std::string> tmp = GetTagTokens(refdir,'|'); // US 51|north
      if (tmp.size() == 2) {
        if (tmp[0] == ref) { // US 51 == US 51
          if (!refs.empty())
            refs += ";" + ref + " " + tmp[1];// ref order of the way wins.
          else
            refs = ref + " " + tmp[1];
          found = true;
          break;
        }
      }
    }

    if (!found) {   // no direction found in relations for this ref
      if (!refs.empty())
        refs += ";" + ref;
      else
        refs = ref;
    }
  }
  return refs;
}

std::vector<SignInfo> GraphBuilder::CreateExitSignInfoList(const OSMNode& node, const OSMWay& way, const OSMData& osmdata) {

  std::vector<SignInfo> exit_list;

  ////////////////////////////////////////////////////////////////////////////
  // NUMBER

  // Exit sign number
  if (way.junction_ref_index() != 0) {
    exit_list.emplace_back(Sign::Type::kExitNumber,
            osmdata.ref_offset_map.name(way.junction_ref_index()));
  }  else if (node.ref()) {
    exit_list.emplace_back(Sign::Type::kExitNumber,
            osmdata.node_ref.find(node.osmid)->second);
  }

  ////////////////////////////////////////////////////////////////////////////
  // BRANCH

  bool has_branch = false;

  // Exit sign branch refs
  if (way.destination_ref_index() != 0) {
    has_branch = true;
    std::vector<std::string> branch_refs = GetTagTokens(
        osmdata.ref_offset_map.name(way.destination_ref_index()));
    for (auto& branch_ref : branch_refs) {
      exit_list.emplace_back(Sign::Type::kExitBranch, branch_ref);
    }
  }

  // Exit sign branch road names
  if (way.destination_street_index() != 0) {
    has_branch = true;
    std::vector<std::string> branch_streets = GetTagTokens(
        osmdata.name_offset_map.name(way.destination_street_index()));
    for (auto& branch_street : branch_streets) {
      exit_list.emplace_back(Sign::Type::kExitBranch, branch_street);
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // TOWARD

  bool has_toward = false;

  // Exit sign toward refs
  if (way.destination_ref_to_index() != 0) {
    has_toward = true;
    std::vector<std::string> toward_refs = GetTagTokens(
        osmdata.ref_offset_map.name(way.destination_ref_to_index()));
    for (auto& toward_ref : toward_refs) {
      exit_list.emplace_back(Sign::Type::kExitToward, toward_ref);
    }
  }

  // Exit sign toward streets
  if (way.destination_street_to_index() != 0) {
    has_toward = true;
    std::vector<std::string> toward_streets = GetTagTokens(
        osmdata.name_offset_map.name(way.destination_street_to_index()));
    for (auto& toward_street : toward_streets) {
      exit_list.emplace_back(Sign::Type::kExitToward, toward_street);
    }
  }

  // Exit sign toward locations
  if (way.destination_index() != 0) {
    has_toward = true;
    std::vector<std::string> toward_names = GetTagTokens(
        osmdata.name_offset_map.name(way.destination_index()));
    for (auto& toward_name : toward_names) {
      exit_list.emplace_back(Sign::Type::kExitToward, toward_name);
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // Process exit_to only if other branch or toward info does not exist
  if (!has_branch && !has_toward) {
    if (node.exit_to()) {

      std::string tmp;
      std::size_t pos;
      std::vector<std::string> exit_tos = GetTagTokens(
          osmdata.node_exit_to.find(node.osmid)->second);
      for (auto& exit_to : exit_tos) {

        tmp = exit_to;

        boost::algorithm::to_lower(tmp);

        //remove the "To" For example:  US 11;To I 81;Carlisle;Harrisburg
        if (boost::starts_with(tmp, "to ")) {
            exit_list.emplace_back(Sign::Type::kExitToward, exit_to.substr(3));
            continue;
        }
        //remove the "Toward" For example:  US 11;Toward I 81;Carlisle;Harrisburg
        if (boost::starts_with(tmp, "toward ")) {
            exit_list.emplace_back(Sign::Type::kExitToward, exit_to.substr(7));
            continue;
        }

        std::size_t found = tmp.find(" to ");

        //Default to kToward if found twice or "toward" found as well; otherwise, <branch> to <toward>
        //For example:  I 95 to I 695
        if (found != std::string::npos &&
           (tmp.find(" to ",found+4) == std::string::npos && tmp.find(" toward ") == std::string::npos)) {

            exit_list.emplace_back(Sign::Type::kExitBranch, exit_to.substr(0,found));

            exit_list.emplace_back(Sign::Type::kExitToward, exit_to.substr(found+4));
            continue;
        }

        found = tmp.find(" toward ");

        //Default to kToward if found twice or "to" found as well; otherwise, <branch> toward <toward>
        //For example:  I 95 to I 695
        if (found != std::string::npos &&
            (tmp.find(" toward ",found+8) == std::string::npos && tmp.find(" to ") == std::string::npos)) {

          exit_list.emplace_back(Sign::Type::kExitBranch, exit_to.substr(0,found));

          exit_list.emplace_back(Sign::Type::kExitToward, exit_to.substr(found+8));
          continue;
        }

        //default to toward.
        exit_list.emplace_back(Sign::Type::kExitToward, exit_to);
      }
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // NAME

  // Exit sign name
  if (node.name()) {
    std::vector<std::string> names = GetTagTokens(
            osmdata.node_name.find(node.osmid)->second);
    for (auto& name : names) {
      exit_list.emplace_back(Sign::Type::kExitName, name);
    }
  }

  return exit_list;
}

}
}
