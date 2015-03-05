
#include "mjolnir/graphbuilder.h"
#include "mjolnir/util.h"

#include <future>
#include <utility>
#include <thread>
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

#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/edgeinfobuilder.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;


namespace {
// Number of tries when determining not thru edges
constexpr uint32_t kMaxNoThruTries = 256;

// Absurd classification.
constexpr uint32_t kAbsurdRoadClass = 777777;

GraphId fromOSMNode(const OSMNode& node, const TileHierarchy& hierarchy, const uint8_t level) {
  return hierarchy.GetGraphId({node.lng, node.lat}, level);
}

// collect all the edges that start or end at this node
struct node_bundle : Node {
  std::list<std::pair<Edge, size_t> > edges;
};
node_bundle collect_node_edges(sequence<Node>::iterator& node_itr, sequence<Node>& nodes, sequence<Edge>& edges) {
  node_bundle bundle = (*node_itr);
  Node node;
  while(node_itr != nodes.end() && (node = *node_itr).node.osmid == bundle.node.osmid) {
    if(node.is_start()) {
      auto itr = edges[node.start_of];
      auto edge = *itr;
      bundle.edges.emplace_back(edge, itr.position());
      bundle.node.attributes_.link_edge = bundle.node.attributes_.link_edge || edge.attributes.link;
      bundle.node.attributes_.non_link_edge = bundle.node.attributes_.non_link_edge || !edge.attributes.link;
    }
    if(node.is_end()) {
      auto itr = edges[node.end_of];
      auto edge = *itr;
      bundle.edges.emplace_back(edge, itr.position());
      bundle.node.attributes_.link_edge = bundle.node.attributes_.link_edge || edge.attributes.link;
      bundle.node.attributes_.non_link_edge = bundle.node.attributes_.non_link_edge || !edge.attributes.link;
    }
    ++node_itr;
  }
  return bundle;
}

/**
 * Get the best classification for any non-link edges from a node.
 * @param  edges The file backed list of edges in the graph.
 * @return  Returns the best (most important) classification
 */
// Gets the most important class among the node's edges
uint32_t GetBestNonLinkClass(const std::list<std::pair<Edge, size_t> >& edges) {
  uint32_t bestrc = kAbsurdRoadClass;
  for (const auto& edge : edges) {
    if (!edge.first.attributes.link) {
      if (edge.first.attributes.importance < bestrc)
        bestrc = edge.first.attributes.importance;
    }
  }
  return bestrc;
}

}

namespace valhalla {
namespace mjolnir {



// Construct GraphBuilder based on properties file and input PBF extract
GraphBuilder::GraphBuilder(const boost::property_tree::ptree& pt)
    : level_(0),
      tile_hierarchy_(pt.get_child("hierarchy")),
      nodes_file_("nodes.bin"),
      edges_file_("edges.bin"),
      threads_(std::max(static_cast<unsigned int>(1),
               pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency()))){
}

// Build the graph from the input
void GraphBuilder::Build(OSMData& osmdata) {

  // Make the edges and nodes in the graph
  const auto& tl = tile_hierarchy_.levels().rbegin();
  level_ = tl->second.level;
  ConstructEdges(osmdata, tl->second.tiles.TileSize());

  // Sort nodes by graphid then by osmid, so its basically a set of tiles
  sequence<Node> nodes(nodes_file_, false);
  nodes.sort(
    [&tile_hierarchy, &level_](const Node& a, const Node& b) {
      if(a.graph_id == b.graph_id)
        return a.node.osmid < b.node.osmid;
      return a.graph_id < b.graph_id;
    }
  );
  //run through the sorted nodes, going back to the edges they reference and updating each edge to point to the new node index in the file
  sequence<Edge> edges(edges_file_, false);
  uint32_t node_index = 0;
  Node last_node{};
  std::map<GraphId, size_t> tiles;
  nodes.transform(
    [&edges, &node_index](Node& node) {
      //if this node marks the start of an edge, go tell the edge where the node is
      if(node.is_start()) {
        auto element = edges[node.start_of];
        auto edge = *element;
        edge.sourcenode_ = node_index;
        element = edge;
      }
      //if this node marks the end of an edge, go tell the dge where the node is
      if(node.is_end()) {
        auto element = edges[node.end_of];
        auto edge = *element;
        edge.targetnode_ = node_index;
        element = edge;
      }
      //remember if this was a new tile
      if(tiles.size() == 0 || node.graph_id != (tiles.end() - 1)->first) {
        tiles.insert({node.graph_id, node_index});
        node.graph_id.fields.id = 0;
        last_node = node;
      }//not a new tile so which node is it in the tile
      else {
        if(last_node.node.osmid != node.node.osmid)
          node.graph_id.fields.id = last_node.graph_id.fields + 1;
        last_node = node;
      }
      //next node
      ++node_index;
      return node;
    }
  );

  //TODO: a bunch of node osm id associations for restrictions, node_ref, node_exit_to, node_name

  DataQuality stats;

  // Reclassify links (ramps). Cannot do this when building tiles since the
  // edge list needs to be modified
  ReclassifyLinks(osmdata.ways_file, stats);

  // Build tiles at the local level. Form connected graph from nodes and edges.
  BuildLocalTiles(tl->second.level, osmdata, tiles, stats);

  stats.LogStatistics();
}

// Construct edges in the graph and assign nodes to tiles.
void GraphBuilder::ConstructEdges(const OSMData& osmdata, const float tilesize) {
  LOG_INFO("Creating graph edges from ways...")

  // Iterate through the OSM ways
  uint32_t edgeindex = 0;
  uint64_t startnodeid;
  GraphId graphid;
  //so we can read ways and nodes and write edges
  sequence<OSMWay> ways(osmdata.ways_file, false);
  sequence<OSMWayNode> way_nodes(osmdata.way_node_references_file, false);
  sequence<Edge> edges(edges_file_, true);
  sequence<Node> nodes(nodes_file_, true);

  //for each way traversed via the node refs
  size_t current_way_node_index = 0;
  while (current_way_node_index < way_nodes.size()) {
    //grab the way and its first node
    const auto first_way_node = *way_nodes[current_way_node_index];
    const auto first_way_node_index = current_way_node_index;
    const auto way = *ways[first_way_node.way_index];

    //remember this edge starts here
    Edge edge = Edge::make_edge(nodes.size(), first_way_node.way_index, current_way_node_index, way);

    //remember this node as starting this edge
    first_way_node.node.attributes_.link_edge |= way.link();
    nodes.push_back({first_way_node.node, ways.size(), -1, fromOSMNode(first_way_node.node, tile_hierarchy_, level_)});

    // Iterate through the nodes of the way until we find an intersection
    while(current_way_node_index < way_nodes.size()) {
      //check if we are done with this way, ie we are started on the next way
      const auto way_node = *way_nodes[++current_way_node_index];

      // Increment the count for this edge
      edge.attributes.llcount++;

      // If a is an intersection or the end of the way
      // it's a node of the road network graph
      if (way_node.node.intersection()) {
        //remember that this edge ends here
        edge.targetnode_ = nodes.size();

        //remember this node as ending this edge
        nodes.push_back({way_node.node, -1, ways.size(), fromOSMNode(way_node.node, tile_hierarchy_, level_)});

        // Add the edge to the list of edges
        edges.push_back(edge);

        // Start a new edge if this is not the last node in the way.
        // We can reuse the index of the latlng added above
        if (current_way_node_index - first_way_node_index < way.node_count() - 1) {
          startnodeid = way_node.node.osmid;
          edge = Edge::make_edge(nodes.size(), way_node.way_index, current_way_node_index, way);
          sequence<Node>::iterator element = --nodes.end();
          auto node = *element;
          node.start_of = ways.size();
          element = node;
        }// This was the last shape point in a dead end way so go to the next
        else {
          ++current_way_node_index;
          break;
        }
      }// if this edge has a signal not at a intersection
      else if (way_node.node.traffic_signal()) {
        edge.attributes.traffic_signal = true;
        edge.attributes.forward_signal = way_node.node.forward_signal();
        edge.attributes.backward_signal = way_node.node.backward_signal();
      }
    }
  }

  LOG_INFO("Finished with " + std::to_string(edges.size()) + " edges and " + std::to_string(nodes.size()) + " nodes");
}


// Reclassify links (ramps and turn channels).
void GraphBuilder::ReclassifyLinks(const std::string& ways_file, DataQuality& stats) {
  LOG_INFO("Reclassifying link graph edges...")

  uint32_t count = 0;
  std::unordered_set<size_t> visitedset;  // Set of visited nodes
  std::unordered_set<size_t> expandset;   // Set of nodes to expand
  std::list<size_t> linkedgeindexes;     // Edge indexes to reclassify
  std::vector<uint32_t> endrc;           //
  sequence<OSMWay> ways(ways_file, false);
  sequence<Edge> edges(edges_file_, false);
  sequence<Node> nodes(nodes_file_, false);

  //try to expand
  auto expand = [&expandset, &endrc, &nodes, &edges] (const Edge& edge, const sequence<Node>::iterator& node_itr) {
    // If the first end node contains a non-link edge we compute the best
    // classification. If not we add the end node to the expand set.
    auto end_node_itr = (edge.sourcenode_ == node_itr.position() ? nodes[edge.targetnode_] : node_itr);
    auto end_node = (*end_node_itr).node;
    if (end_node.attributes_.non_link_edge()) {
      endrc.push_back(GetBestNonLinkClass(collect_node_edges(end_node_itr, nodes, edges).edges));
    } else {
      expandset.insert(end_node_itr.position());
    }
  }

  //for each node
  sequence<Node>::iterator node_itr = nodes.begin();
  while (node_itr != nodes.end()) {
    auto bundle = collect_node_edges(node_itr, nodes, edges);
    auto& node = bundle.node;
    if (node.attributes_.link_edge() && node.attributes_.non_link_edge) {
      // Get the highest classification of non-link edges at this node
      endrc.clear();
      endrc.push_back(GetBestNonLinkClass(bundle.edges));

      // Expand from all link edges
      for (const auto& startedge : bundle.edges) {
        // Get the edge information. Skip non-link edges
        if (!startedge.first.attributes.link) {
          continue;
        }

        // Clear the sets and edge list
        visitedset.clear();
        expandset.clear();
        linkedgeindexes.clear();

        // Add start edge to list of links edges to potentially reclassify
        linkedgeindexes.push_back(startedge.second);

        expand(startedge.first, node_itr);

        // Expand edges until all paths reach a node that has a non-link
        for (uint32_t n = 0; n < 512; n++) {
          // Once expand list is empty - mark all link edges encountered
          // with the specified classification / importance and break out
          // of this loop (can still expand other ramp edges
          // from the starting node
          if (expandset.empty()) {
            // Make sure this connects...
            if (endrc.size() < 2)  {
              stats.AddIssue(kUnconnectedLinkEdge, GraphId(),
                             (*ways[startedge.first.wayindex_]).way_id(), 0);
            } else {
              // Set to the value of the 2nd best road class of all
              // connections. This protects against downgrading links
              // when branches occur.
              std::sort(endrc.begin(), endrc.end());
              uint32_t rc = endrc[1];
              for (auto idx : linkedgeindexes) {
                sequence<Edge>::iterator element = edges[idx];
                auto edge = *element;
                if (rc > edge.attributes.importance) {
                  edge.attributes.importance = rc;
                  element = edge;
                  count++;
                }
              }
            }
            break;
          }

          // Get the node off of the expand list and add it to the visited list
          auto expandnode = *expandset.begin();
          expandset.erase(expandset.begin());
          visitedset.insert(expandnode);

          // Expand all edges from this node
          auto expandnode_itr = nodes[expandnode];
          auto expanded = collect_node_edges(expandnode_itr, nodes, edges);
          for (const auto& expandededge : expanded.edges) {
            // Do not allow use of the start edge
            if (expandededge.second == startedge.second) {
              continue;
            }

            // Add this edge (it should be a link edge) and get its end node
            if (!expandededge.first.attributes.link) {
              LOG_ERROR("Expanding onto non-link edge!");
              continue;
            }
            expand(expandededge.first, expandnode_itr);
          }
        }
      }
    }
  }

  LOG_INFO("Finished with " + std::to_string(count) + " reclassified.");
}

namespace {
/*
// Test if this is a "not thru" edge. These are edges that enter a region that
// has no exit other than the edge entering the region
bool IsNoThroughEdge(const GraphId& startnode, const GraphId& endnode,
             const uint32_t startedgeindex,
             const std::unordered_map<GraphId, std::vector<Node>>& nodes,
             sequence<Edge>& edges) {
  // Add the end node Id to the set of nodes to expand
  std::unordered_set<GraphId> visitedset;
  std::unordered_set<GraphId> expandset;
  expandset.insert(endnode);

  // Expand edges until exhausted, the maximum number of expansions occur,
  // or end up back at the starting node. No node can be visited twice.
  for (uint32_t n = 0; n < kMaxNoThruTries; n++) {
    // If expand list is exhausted this is "not thru"
    if (expandset.empty()) {
      return true;
    }

    // Get the node off of the expand list and add it to the visited list.
    // Expand edges from this node.
    GraphId node = *expandset.begin();
    expandset.erase(expandset.begin());
    visitedset.emplace(node);
    const Node& nd = GetNode(node, nodes);
    for (const auto& edgeindex : nd.edges) {
      if (edgeindex == startedgeindex) {
        // Do not allow use of the start edge
        continue;
      }

      // Return false if we have returned back to the start node or we
      // encounter a tertiary road (or better)
      const Edge edge = *edges[edgeindex];
      GraphId nextendnode = (edge.sourcenode_ == node) ?
                edge.targetnode_ : edge.sourcenode_;
      if (nextendnode == startnode ||
          edge.attributes.importance <=
          static_cast<uint32_t>(RoadClass::kTertiary)) {
        return false;
      }

      // Add to the expand set if not in the visited set
      if (visitedset.find(nextendnode) == visitedset.end()) {
        expandset.insert(nextendnode);
      }
    }
  }
  return false;
}
*/
/**
 * Test if a pair of one-way edges exist at the node. One must be
 * inbound and one must be outbound. The current edge is skipped.
 */
bool OnewayPairEdgesExist(const GraphId& nodeid,
                          const Node& node,
                          const uint32_t edgeindex,
                          const uint64_t wayid,
                          sequence<Edge>& edges,
                          sequence<OSMWay>& ways) {
  // Iterate through the edges from this node. Skip the one with
  // the specified edgeindex
  uint32_t idx;
  bool forward;
  bool inbound  = false;
  bool outbound = false;
  for (const auto idx : node.edges) {
    if (idx == edgeindex) {
      continue;
    }

    // Get the edge and way.
    const Edge edge = *edges[idx];
    const OSMWay &w = ways[edge.wayindex_];

    // Skip if this has matching way Id or a link (ramps/turn channel)
    if (w.way_id() == wayid || edge.attributes.link) {
      continue;
    }

    // Check if edge is forward or reverse
    forward = (edge.sourcenode_ == nodeid);

    // Check if this is oneway inbound
    if ( (forward && !w.auto_forward() &&  w.auto_backward()) ||
        (!forward &&  w.auto_forward() && !w.auto_backward())) {
      inbound = true;
    }

    // Check if this is oneway outbound
    if ( (forward &&  w.auto_forward() && !w.auto_backward()) ||
        (!forward && !w.auto_forward() &&  w.auto_backward())) {
      outbound = true;
    }
  }
  return (inbound && outbound);
}

bool IsIntersectionInternal(const GraphId& startnode, const GraphId& endnode,
                const uint32_t edgeindex, const uint64_t wayid,
                const float length,
                const std::unordered_map<GraphId, std::vector<Node>>& nodes,
                sequence<Edge>& edges,
                sequence<OSMWay>& ways) {
  // Limit the length of intersection internal edges
  if (length > kMaxInternalLength) {
    return false;
  }

  // Both end nodes must connect to at least 3 edges
  const Node& node1 = GetNode(startnode, nodes);
  if (node1.edge_count() < 3) {
    return false;
  }
  const Node& node2 = GetNode(endnode, nodes);
  if (node2.edge_count() < 3) {
    return false;
  }

  // Each node must have a pair of oneways (one inbound and one outbound).
  // Exclude links (ramps/turn channels)
  if (!OnewayPairEdgesExist(startnode, node1, edgeindex, wayid, edges, ways) ||
      !OnewayPairEdgesExist(endnode, node2, edgeindex, wayid, edges, ways)) {
    return false;
  }

  // Assume this is an intersection internal edge
  return true;
}

/**
 * Get the use for a link (either a kRamp or kTurnChannel)
 * TODO - validate logic with some real world cases.
 */
Use GetLinkUse(const uint32_t edgeindex, const RoadClass rc,
               const float length, const size_t startnode,
               const size_t endnode,
               sequence<Node>& nodes,
               sequence<Edge>& edges) {
  // Assume link that has highway = motorway or trunk is a ramp.
  // Also, if length is > kMaxTurnChannelLength we assume this is a ramp
  if (rc == RoadClass::kMotorway || rc == RoadClass::kTrunk ||
      length > kMaxTurnChannelLength) {
    return Use::kRamp;
  }

  // TODO - if there is a exit sign or exit number present this is
  // considered kRamp. Do we have this information anywhere yet?

  // Both end nodes have to connect to a non-link edge. If either end node
  // connects only to "links" this likely indicates a split or fork,
  // which are not so prevalent in turn channels.
  auto startnode_itr = nodes[startnode];
  auto startnd = collect_node_edges(startnode_itr, nodes, edges);
  auto endnode_itr = nodes[endnode];
  auto endnd = collect_node_edges(endnode_itr, nodes, edges);
  if (startnd.node.attributes_.non_link_edge() && endnd.node.attributes_.non_link_edge()) {
    // If either end node connects to another link then still
    // call it a ramp. So turn channels are very short and ONLY connect
    // to non-link edges without any exit signs.
    for (const auto& edge : startnd.edges) {
      if (edge.second != edgeindex && edge.first.attributes.link) {
        return Use::kRamp;
      }
    }
    for (const auto& edge : endnd.edges) {
      if (edge.second != edgeindex && edge.first.attributes.link) {
        return Use::kRamp;
      }
    }
    return Use::kTurnChannel;
  }
  else {
    return Use::kRamp;
  }
}

float UpdateLinkSpeed(const Use use, const RoadClass rc, const float spd) {
  if (use == Use::kTurnChannel) {
    return spd * 0.9f;
  } else if (use == Use::kRamp) {
    if (rc == RoadClass::kMotorway) {
      return 95.0f;
    } else if (rc == RoadClass::kTrunk) {
      return 80.0f;
    } else if (rc == RoadClass::kPrimary) {
      return 65.0f;
    } else if (rc == RoadClass::kSecondary) {
      return 50.0f;
    } else if (rc == RoadClass::kTertiary) {
      return 40.0f;
    } else if (rc == RoadClass::kUnclassified) {
      return 35.0f;
    } else {
      return 25.0f;
    }
  }
  return spd;
}

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

uint32_t CreateSimpleTurnRestriction(const uint64_t wayid, const uint32_t edgeindex,
                 GraphTileBuilder& graphtile,
                 const GraphId& endnode, sequence<Edge>& edges,
                 const std::unordered_map<GraphId, std::vector<Node> >& nodes,
                 const OSMData& osmdata, sequence<OSMWay>& ways,
                 DataQuality& stats) {
  auto res = osmdata.restrictions.equal_range(wayid);
  if (res.first == osmdata.restrictions.end()) {
    return 0;
  }

  // Edge is the from edge of a restriction. Find all TRs (if any)
  // through the target (end) node of this directed edge.
  std::vector<OSMRestriction> trs;
  for (auto r = res.first; r != res.second; ++r) {
    if (r->second.via_graphid() == endnode) {
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
  const Node& node = GetNode(endnode, nodes);
  for (auto edgeindex : node.edges) {
    wayids.push_back((*ways[(*edges[edgeindex]).wayindex_]).way_id());
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

void BuildTileSet(
    std::unordered_map<GraphId, std::vector<Node> >::const_iterator tile_start,
    std::unordered_map<GraphId, std::vector<Node> >::const_iterator tile_end,
    const std::unordered_map<GraphId, std::vector<Node> >& nodes,
    const std::string& edges_file,
    const baldr::TileHierarchy& hierarchy,
    const OSMData& osmdata,
    const std::unordered_map<baldr::GraphId, std::string>& node_ref,
    const std::unordered_map<baldr::GraphId, std::string>& node_exit_to,
    const std::unordered_map<baldr::GraphId, std::string>& node_name,
    std::promise<DataQuality>& result) {

  DataQuality stats;

  std::string thread_id = static_cast<std::ostringstream&>(std::ostringstream() << std::this_thread::get_id()).str();
  LOG_INFO("Thread " + thread_id + " started");

  // TODO: try using mmap here for speed up
  sequence<OSMWay> ways(osmdata.ways_file, false);
  sequence<OSMWayNode> way_nodes(osmdata.way_node_references_file, false);
  sequence<Edge> edges(edges_file, false);

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

  // Get the lat,lng of the node
  const auto GetLL = [&way_nodes] (const size_t idx) {
    auto node = (*way_nodes[idx]).node;
    return PointLL(node.lng, node.lat);
  };

  // A place to keep information about what was done
  size_t written = 0;

  // For each tile in the task
  bool added = false;

  for(; tile_start != tile_end; ++tile_start) {
    try {
      // What actually writes the tile
      GraphTileBuilder graphtile;

      // Iterate through the nodes
      uint32_t idx = 0;                 // Current directed edge index
      uint32_t directededgecount = 0;
      GraphId nodeid = tile_start->first.Tile_Base();
      for (const Node& node : tile_start->second) {
        // Get the node lat,lng. Use the first edge from the node.
        PointLL node_ll;
        const Edge node_edge = *edges[node.edges.front()];
        if (node_edge.sourcenode_ == nodeid) {
          // Forward direction - use first lat,lng of the edge
          node_ll = GetLL(node_edge.llindex_);
        } else {
          // Reverse direction - use last lat,lng of the edge
          node_ll = GetLL(node_edge.llindex_ + node_edge.attributes.llcount-1);
        }

        // Look for potential duplicates
        // TODO - need to make stats thread safe
//        CheckForDuplicates(nodeid, node, edgelengths, nodes, edges,
//                           osmdata.ways, stats);

        // Build directed edges. Track the best classification/importance
        // of outbound edges from this node.
        uint32_t n = 0;
        uint32_t driveable = 0;
        RoadClass bestclass = RoadClass::kServiceOther;
        std::vector<DirectedEdgeBuilder> directededges;
        for (auto edgeindex : node.edges) {
          // Get the edge and way
          const Edge edge = edges[edgeindex];
          const OSMWay w = *ways[edge.wayindex_];

          // Get the 2 end nodes of the edge
          const Node& nodea = GetNode(edge.sourcenode_, nodes);
          const Node& nodeb = GetNode(edge.targetnode_, nodes);

          // Get the shape for the edge and compute its length
          auto shape = EdgeShape(edge.llindex_, edge.attributes.llcount);
          uint32_t length = static_cast<uint32_t>(PointLL::Length(shape) + .5f);

          // Determine orientation along the edge (forward or reverse between
          // the 2 nodes). Check for edge error.
          bool forward;
          GraphId source, target;
          if (edge.sourcenode_ == nodeid) {
            forward = true;
            source = edge.sourcenode_;
            target = edge.targetnode_;
          } else if (edge.targetnode_ == nodeid) {
            forward = false;
            source = edge.targetnode_;
            target = edge.sourcenode_;
          } else {
            // ERROR!!!
            LOG_ERROR((boost::format("WayID =  %1% Edge Index = %2% Edge nodes %3% and %4% did not match the OSM node Id %5%")
              % w.way_id() % edgeindex %  edge.sourcenode_  % edge.targetnode_ % nodeid).str());
          }

          // Increment driveable count if edge is driveable in either direction
          if (edge.attributes.driveableforward ||
              edge.attributes.driveablereverse) {
            driveable++;
          }

          // Check for not_thru edge (only on low importance edges)
          bool not_thru = false;
          if (edge.attributes.importance >
              static_cast<uint32_t>(RoadClass::kTertiary)) {
            not_thru = IsNoThroughEdge(source, target, edgeindex, nodes, edges);
            if (not_thru) {
              stats.not_thru_count++;
            }
          }

          // Test if an internal intersection edge
          bool internal = IsIntersectionInternal(source, target, edgeindex,
                         w.way_id(), length, nodes, edges, ways);
          if (internal) {   // TODO - move stats to GraphOptimizer
            stats.internalcount++;
          }
          // If link is set test to see if we can infer that the edge
          // is a turn channel. Update speed for link edges.
          float speed = w.speed();
          RoadClass rc = static_cast<RoadClass>(edge.attributes.importance);
          Use use = w.use();
          if (w.link()) {
//            if (use != Use::kNone) {
//              (*stats).AddIssue(kIncompatibleLinkUse, GraphId(), w.way_id(), 0);
 //           }
            use   = GetLinkUse(edgeindex, rc, length, edge.sourcenode_,
                               edge.targetnode_, nodes, edges);
            if (use == Use::kTurnChannel) {
              stats.turnchannelcount++;
            }
            speed = UpdateLinkSpeed(use, rc, w.speed());
          }

          // Infer cul-de-sac if a road edge is a loop and is low
          // classification. TODO - do we need length limit?
          if (use == Use::kRoad && source == target &&
              rc > RoadClass::kTertiary) {
            use = Use::kCuldesac;
            stats.culdesaccount++;        // TODO - move stats to GraphOptimizer...
          }

          // Handle simple turn restrictions that originate from this
          // directed edge
          uint32_t restrictions = CreateSimpleTurnRestriction(w.way_id(), idx,
                      graphtile, target, edges, nodes, osmdata, ways, stats);
          if (restrictions != 0) {
            stats.simplerestrictions++;
          }

          bool has_signal = false;
          // traffic signal exists at an intersection node
          if (!forward && node.traffic_signal())
            has_signal = true;
          // traffic signal exists at a non-intersection node
          // forward signal must exist if forward direction and vice versa.
          // if forward and backward signal flags are not set then only set for oneways.
          else if (edge.attributes.traffic_signal) {
            if ((forward && edge.attributes.forward_signal) || (!forward && edge.attributes.backward_signal) ||
                (w.oneway() && !edge.attributes.forward_signal && !edge.attributes.backward_signal))
              has_signal = true;
          }

          // Add a directed edge and get a reference to it
          directededges.emplace_back(w, target, forward, length,
                        speed, use, not_thru, internal, rc, n, has_signal, restrictions);
          DirectedEdgeBuilder& directededge = directededges.back();

          // Update the node's best class
          bestclass = std::min(bestclass, directededge.classification());

          // Check for updated ref from relations.
          std::string ref;
          auto iter = osmdata.way_ref.find(w.way_id());
          if (iter != osmdata.way_ref.end()) {
            if (w.ref_index() != 0)
              ref = graphbuilder::GetRef(osmdata.ref_offset_map.name(w.ref_index()),iter->second);
          }

          // Add edge info to the tile and set the offset in the directed edge
          uint32_t edge_info_offset = graphtile.AddEdgeInfo(
            edgeindex, source, target, shape,
            w.GetNames(ref, osmdata.ref_offset_map, osmdata.name_offset_map),
            added);
          directededge.set_edgeinfo_offset(edge_info_offset);

          // TODO - update logic so we limit the CreateExitSignInfoList calls
          // TODO - Also, we will have to deal with non ramp signs
          // Any exits for this directed edge? is auto and oneway?
          std::vector<SignInfo> exits = graphbuilder::CreateExitSignInfoList(
                    source, node, w, osmdata, node_ref, node_exit_to, node_name);
          if (!exits.empty() && directededge.forwardaccess()
               && directededge.use() == Use::kRamp) {
            graphtile.AddSigns(idx, exits);
            directededge.set_exitsign(true);
          }

          // Increment the directed edge index within the tile
          idx++;
          n++;
        }

        // Set the node lat,lng, index of the first outbound edge, and the
        // directed edge count from this edge and the best road class
        // from the node. Increment directed edge count.
        NodeInfoBuilder nodebuilder(node_ll, directededgecount,
                                    node.edge_count(), driveable,
                                    bestclass, node.access_mask(), node.type(),
                                    (node.edge_count() == 1),
                                    node.traffic_signal());

        directededgecount += node.edge_count();

        // Add node and directed edge information to the tile
        graphtile.AddNodeAndDirectedEdges(nodebuilder, directededges);

        // Increment nodeid
        nodeid++;

        // Increment the counts in the histogram
        stats.node_counts[directededges.size()]++;
      }

      // Write the actual tile to disk
      graphtile.StoreTileData(hierarchy, tile_start->first);

      // Made a tile
      LOG_INFO((boost::format("Thread %1% wrote tile %2%: %3% bytes") % thread_id % tile_start->first % graphtile.size()).str());
      written += graphtile.size();
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

}

// Get highway refs from relations
std::string GraphBuilder::GetRef(const std::string& way_ref,
                                 const std::string& relation_ref) {
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

std::vector<SignInfo> GraphBuilder::CreateExitSignInfoList(
    const GraphId& nodeid, const Node& node, const OSMWay& way,
    const OSMData& osmdata,
    const std::unordered_map<baldr::GraphId, std::string>& node_ref,
    const std::unordered_map<baldr::GraphId, std::string>& node_exit_to,
    const std::unordered_map<baldr::GraphId, std::string>& node_name) {

  std::vector<SignInfo> exit_list;

  ////////////////////////////////////////////////////////////////////////////
  // NUMBER

  // Exit sign number
  if (way.junction_ref_index() != 0) {
    exit_list.emplace_back(Sign::Type::kExitNumber,
            osmdata.ref_offset_map.name(way.junction_ref_index()));
  }  else if (node.ref()) {
    exit_list.emplace_back(Sign::Type::kExitNumber,
            node_ref.find(nodeid)->second);
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
          node_exit_to.find(nodeid)->second);
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
            node_name.find(nodeid)->second);
    for (auto& name : names) {
      exit_list.emplace_back(Sign::Type::kExitName, name);
    }
  }

  return exit_list;
}

// Build tiles for the local graph hierarchy
void GraphBuilder::BuildLocalTiles(const uint8_t level, const OSMData& osmdata, const std::map<GraphId, size_t>& tiles, DataQuality& stats) const {
  LOG_INFO("Building " + std::to_string(tiles.size()) + " tiles with " + std::to_string(threads_) + " threads...");

  // A place to hold worker threads and their results, be they exceptions or otherwise
  std::vector<std::shared_ptr<std::thread> > threads(threads_);

  // Hold the results (DataQuality/stats) for the threads
  std::vector<std::promise<DataQuality> > results(threads.size());

  // Divvy up the work
  size_t floor = tiles.size() / threads.size();
  size_t at_ceiling = tiles.size() - (threads.size() * floor);
  std::unordered_map<GraphId, std::vector<Node> >::const_iterator tile_start, tile_end = tiles.begin();

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
      new std::thread(BuildTileSet, tile_start, tile_end, std::cref(nodes_file_),
          std::cref(edges_file_), std::cref(tile_hierarchy_), std::cref(osmdata), std::cref(osmdata.node_ref),
          std::cref(osmdata.node_exit_to), std::cref(osmdata.node_name), std::ref(results[i]))
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
}


}
}
