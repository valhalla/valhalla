

#include "mjolnir/graphbuilder.h"
#include "mjolnir/util.h"

// Use open source PBF reader from:
//     https://github.com/CanalTP/libosmpbfreader
#include "osmpbfreader.h"

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
#include <valhalla/baldr/exitsigninfo.h>
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/edgeinfobuilder.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Number of tries when determining not thru edges
constexpr uint32_t kMaxNoThruTries = 256;

// Absurd classification.
constexpr uint32_t kAbsurdRoadClass = 777777;

// Method to get the shape for an edge - since LL is stored as a pair of
// floats we need to change into PointLL to get length of an edge
std::vector<midgard::PointLL> Edge::shape() const {
  std::vector<midgard::PointLL> lls;
  for (const OSMLatLng& ll : latlngs_) {
    lls.emplace_back(ll.first, ll.second);
  }
  return lls;
}


// Construct GraphBuilder based on properties file and input PBF extract
GraphBuilder::GraphBuilder(const boost::property_tree::ptree& pt)
    : tile_hierarchy_(pt.get_child("hierarchy")),
      stats_(new DataQuality()),
      threads_(std::max(static_cast<unsigned int>(1), pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency()))){
}

// Delete the OSM node map.
void delete_node_map(OSMData& osmdata) {
  std::unordered_map<uint64_t, OSMNode> tmp;
  tmp.swap(osmdata.nodes);
}

// Delete the OSM node reference vector.
void delete_noderefs(OSMData& osmdata) {
  NodeRefVector tmp;
  tmp.swap(osmdata.noderefs);
}

// Build the graph from the input
void GraphBuilder::Build(OSMData& osmdata) {
  // Construct edges
  std::clock_t start = std::clock();
  ConstructEdges(osmdata);
  uint32_t msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
  LOG_INFO("ConstructEdges took " + std::to_string(msecs) + " ms");

  // No longer need the OSMNodes map in osmdata - can we recover its
  // memory?
  LOG_INFO("Sizeof OSM node map: " + std::to_string(osmdata.nodes.size()));
  delete_node_map(osmdata);
  LOG_INFO("After delete: Sizeof OSM node map: " + std::to_string(osmdata.nodes.size()));
  LOG_INFO("Sizeof OSM node refs: " + std::to_string(osmdata.noderefs.size()));
  delete_noderefs(osmdata);
  LOG_INFO("After delete: Sizeof OSM node refs: " + std::to_string(osmdata.noderefs.size()));

  // Sort the edge indexes at the nodes (by driveability and importance)
  // TODO (do we still need to do this??)
/** start = std::clock();
  SortEdgesFromNodes();
  msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
  LOG_INFO("SortEdges took " + std::to_string(msecs) + " ms");  **/

  // Reclassify links (ramps). Cannot do this when building tiles since the
  // edge list needs to be modified
  start = std::clock();
  ReclassifyLinks(osmdata.ways);
  msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
  LOG_INFO("ReclassifyLinks took " + std::to_string(msecs) + " ms");

  // Tile the nodes at the base (local) level
  start = std::clock();
  const auto& tl = tile_hierarchy_.levels().rbegin();
  TileNodes(tl->second.tiles.TileSize(), tl->second.level);
  msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
  LOG_INFO("TileNodes took " + std::to_string(msecs) + " ms");

  // Iterate through edges - tile the end nodes to create connected graph
  start = std::clock();
  BuildLocalTiles(tl->second.level, osmdata.ways,osmdata.node_ref,
                  osmdata.node_exit_to, osmdata.node_name);
  msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
  LOG_INFO("BuildLocalTiles took " + std::to_string(msecs) + " ms");

  // Log statistics and issues
  stats_->Log();
}

// Construct edges in the graph.
// TODO - compare logic to example_routing app. to see why the edge
// count differs.
void GraphBuilder::ConstructEdges(const OSMData& osmdata) {
  // Reserve size for the Node map
  nodes_.reserve(osmdata.intersection_count);

  // Iterate through the OSM ways
  uint32_t edgeindex = 0;
  uint64_t startnodeid, nodeid;
  edges_.reserve(osmdata.edge_count);
  for (size_t wayindex = 0; wayindex < osmdata.ways.size(); wayindex++) {
    // Get some way attributes needed for the edge
    const auto& way = osmdata.ways[wayindex];

    // Start an edge at the first node of the way and add the
    // edge index to the node
    startnodeid = nodeid = osmdata.noderefs[way.noderef_index()];
    const auto& osmnode = osmdata.nodes.find(nodeid)->second;
    Edge edge(nodeid, wayindex, osmnode.latlng(), way);

    // If Node exists add an edge to it, otherwise construct Node, add an
    // edge and add it to the map
    auto node = nodes_.find(nodeid);
    if (node == nodes_.end()) {
      nodes_.emplace(std::piecewise_construct,
                     std::forward_as_tuple(nodeid),
                     std::forward_as_tuple(osmnode, edgeindex, way.link()));
    }
    else {
      node->second.AddEdge(edgeindex, way.link());
    }

    // Iterate through the nodes of the way and add lat,lng to the current
    // way until a node with > 1 uses is found.
    for (size_t i = 1; i < way.node_count(); i++) {
      // Add the node lat,lng to the edge shape.
      nodeid  = osmdata.noderefs[way.noderef_index() + i];
      const auto& osmnode = osmdata.nodes.find(nodeid)->second;
      edge.AddLL(osmnode.latlng());

      // If a is an intersection or the end of the way
      // it's a node of the road network graph
      if (osmnode.intersection()) {
        // End the current edge and add its edge index to the node
        edge.targetnode_ = nodeid;

        // If Node exists add an edge to it, otherwise construct Node, add an
        // edge and add it to the map
        auto node = nodes_.find(nodeid);
        if (node == nodes_.end()) {
          nodes_.emplace(std::piecewise_construct,
                         std::forward_as_tuple(nodeid),
                         std::forward_as_tuple(osmnode, edgeindex, way.link()));

          // Is there a better way to save this for use later?
          node = nodes_.find(nodeid);
        }
        else {
          // Add the edgeindex to the node (unless this is a loop with same
          // start and end node Ids)
          if (startnodeid != nodeid) {
            node->second.AddEdge(edgeindex, way.link());
          }
        }

        // Add the edge to the list of edges
        edges_.emplace_back(std::move(edge));
        edgeindex++;

        // Start a new edge if this is not the last node in the way
        if (i < way.node_count() - 1) {
          startnodeid = nodeid;
          edge = Edge(nodeid, wayindex, osmnode.latlng(), way);
          node->second.AddEdge(edgeindex, way.link());
        }
      }
    }
  }
  LOG_INFO("Constructed " + std::to_string(edges_.size()) + " edges");
}

class EdgeSorter {
 public:
  EdgeSorter(const std::vector<Edge>& edges)
     : osmnodeid_(0),
       edges_(edges) {
  }
  void SetNodeId(const uint64_t nodeid) {
    osmnodeid_ = nodeid;
  }
  bool operator() (const uint32_t e1index, const uint32_t e2index) const {
    const Edge& e1 = edges_[e1index];
    const Edge& e2 = edges_[e2index];

    // Check if edges are forward or reverse
    bool e1forward = (e1.sourcenode_ == osmnodeid_);
    bool e2forward = (e2.sourcenode_ == osmnodeid_);

    bool e1drive = ((e1forward && e1.attributes_.fields.driveableforward) ||
                       (!e1forward && e1.attributes_.fields.driveablereverse));
    bool e2drive = ((e2forward && e2.attributes_.fields.driveableforward) ||
                       (!e2forward && e2.attributes_.fields.driveablereverse));

    // If both driveable or both not driveable, compare importance
    if (e1drive == e2drive) {
      return e1.attributes_.fields.importance < e2.attributes_.fields.importance;
    } else if (e1drive && !e2drive) {
      return true;
    } else {
      return false;
    }
  }
 private:
  uint64_t osmnodeid_;
  const std::vector<Edge>& edges_;
};

// Sort edge indexes from each node
void GraphBuilder::SortEdgesFromNodes() {
  // Sort the directed edges from the node
  EdgeSorter sorter(edges_);
  for (auto& node : nodes_) {
    sorter.SetNodeId(node.first);
    std::sort(node.second.mutable_edges().begin(),
              node.second.mutable_edges().end(), sorter);
  }
}

uint32_t GraphBuilder::GetBestNonLinkClass(const Node& node) const {
  uint32_t bestrc = kAbsurdRoadClass;
  for (auto idx : node.edges()) {
    const Edge& edge = edges_[idx];
    if (!edge.attributes_.fields.link) {
      if (edge.attributes_.fields.importance < bestrc)
        bestrc = edge.attributes_.fields.importance;
    }
  }
  return bestrc;
}

// Reclassify links (ramps and turn channels).
void GraphBuilder::ReclassifyLinks(const WayVector& ways) {
  uint32_t count = 0;
  std::unordered_set<uint64_t> visitedset;  // Set of visited nodes
  std::unordered_set<uint64_t> expandset;   // Set of nodes to expand
  std::vector<uint32_t> linkedgeindexes;    // Edge indexes to reclassify
  for (const auto& node : nodes_) {
    if (node.second.link_edge() && node.second.non_link_edge()) {
      // Get the highest classification of non-link edges at this node
      uint32_t beststartrc = GetBestNonLinkClass(node.second);

      // Expand from all link edges
      for (auto startedgeindex : node.second.edges()) {
        // Get the edge information. Skip non-link edges
        const Edge& startedge = edges_[startedgeindex];
        if (!startedge.attributes_.fields.link) {
          continue;
        }

        // Clear the sets and edge list
        visitedset.clear();
        expandset.clear();
        linkedgeindexes.clear();

        // Add the start edge to list of links edges to potentially reclassify
        linkedgeindexes.push_back(startedgeindex);

        // If the first end node contains a non-link edge we compute the best
        // classification. If not we add the end node to the expand set.
        uint32_t bestendrc = kAbsurdRoadClass;
        uint64_t endnode = (startedge.sourcenode_ == node.first) ?
            startedge.targetnode_ : startedge.sourcenode_;
        auto firstendnode = nodes_.find(endnode);
        if (firstendnode != nodes_.end()) {
          if (firstendnode->second.non_link_edge()) {
            bestendrc = GetBestNonLinkClass(firstendnode->second);
          } else {
            expandset.insert(endnode);
          }
        }

        // Expand edges until all paths reach a node that has a non-link
        for (uint32_t n = 0; n < 512; n++) {
          // Once expand list is empty - mark all link edges encountered
          // with the specified classification / importance and break out
          // of this loop (can still expand other ramp edges
          // from the starting node
          if (expandset.empty()) {
            // Make sure this connects...
            if (bestendrc == kAbsurdRoadClass)  {
              stats_->AddIssue(kUnconnectedLinkEdge, GraphId(),
                             ways[startedge.wayindex_].way_id(), 0);
            } else {
              // Set to the lower of the 2 best road classes (start and end).
              // Apply this to each of the link edges
              uint32_t rc = std::max(beststartrc, bestendrc);
              for (auto idx : linkedgeindexes) {
                if (rc > edges_[idx].attributes_.fields.importance) {
                  edges_[idx].attributes_.fields.importance = rc;
                  count++;
                }
              }
            }
            break;
          }

          // Get the node off of the expand list and add it to the visited list
          uint64_t expandnode = *expandset.begin();
          expandset.erase(expandset.begin());
          visitedset.insert(expandnode);

          // Expand all edges from this node
          const auto nd = nodes_.find(expandnode);
          if (nd == nodes_.end()) {
            continue;
          }
          for (const auto& edgeindex : nd->second.edges()) {
            // Do not allow use of the start edge
            if (edgeindex == startedgeindex) {
              continue;
            }

            // Add this edge (it should be a link edge) and get its end node
            const Edge& nextedge = edges_[edgeindex];
            if (!nextedge.attributes_.fields.link) {
              LOG_ERROR("Expanding onto non-link edge!");
              continue;
            }
            uint32_t osmendnode = (nextedge.sourcenode_ == expandnode) ?
                    nextedge.targetnode_ : nextedge.sourcenode_;
            linkedgeindexes.push_back(edgeindex);
            const auto endnd = nodes_.find(osmendnode);
            if (endnd == nodes_.end()) {
              continue;
            }

            // If the end node contains any non-links find the best
            // classification - do not expand from this node
            if (endnd->second.non_link_edge()) {
              uint32_t rc = GetBestNonLinkClass(endnd->second);
              if (rc < bestendrc) {
                bestendrc = rc;
              }
            } else if (visitedset.find(osmendnode) == visitedset.end()) {
              // Add to the expand set if not in the visited set
              expandset.insert(osmendnode);
            }
          }
        }
      }
    }
  }
  LOG_INFO((boost::format("Reclassify Links: Count %1%") % count).str());
}

namespace {

class graphbuilder : public GraphBuilder {
 public:
  using GraphBuilder::CreateExitSignInfoList;
};

// Test if this is a "not thru" edge. These are edges that enter a region that
// has no exit other than the edge entering the region
bool IsNoThroughEdge(const uint64_t startnode, const uint64_t endnode,
                     const uint32_t startedgeindex,
                     const std::unordered_map<uint64_t, Node>& nodes,
                     const std::vector<Edge>& edges) {
  // Add the end node Id to the set of nodes to expand
  std::unordered_set<uint64_t> visitedset;
  std::unordered_set<uint64_t> expandset;
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
    uint64_t node = *expandset.begin();
    expandset.erase(expandset.begin());
    visitedset.emplace(node);
    const auto nd = nodes.find(node);
    if (nd != nodes.end()) {
      for (const auto& edgeindex : nd->second.edges()) {
        if (edgeindex == startedgeindex) {
          // Do not allow use of the start edge
          continue;
        }

        // Return false if we have returned back to the start node or we
        // encounter a tertiary road (or better)
        const Edge& edge = edges[edgeindex];
        uint64_t osmendnode = (edge.sourcenode_ == node) ?
                  edge.targetnode_ : edge.sourcenode_;
        if (osmendnode == startnode ||
            edge.attributes_.fields.importance <=
            static_cast<uint32_t>(RoadClass::kTertiaryUnclassified)) {
          return false;
        }

        // Add to the expand set if not in the visited set
        if (visitedset.find(osmendnode) == visitedset.end()) {
          expandset.insert(osmendnode);
        }
      }
    }
  }
  return false;
}

/**
 * Test if a pair of one-way edges exist at the node. One must be
 * inbound and one must be outbound. The current edge is skipped.
 */
bool OnewayPairEdgesExist(const Node& node,
                          const uint32_t edgeindex,
                          const uint64_t wayid,
                          const std::vector<Edge>& edges,
                          const WayVector& ways) {
  // Iterate through the edges from this node. Skip the one with
  // the specified edgeindex
  uint32_t idx;
  bool inbound  = false;
  bool outbound = false;
  for (const auto idx : node.edges()) {
    if (idx == edgeindex) {
      continue;
    }

    // Get the edge and way
    const Edge& edge = edges[idx];
    const OSMWay &w = ways[edge.wayindex_];

    // Skip if this has matching way Id
    if (w.way_id() == wayid) {
      return false;
    }

    // Check if this is oneway inbound
    if (!w.auto_forward() && w.auto_backward()) {
      inbound = true;
    }

    // Check if this is oneway outbound
    if (w.auto_forward() && !w.auto_backward()) {
      outbound = true;
    }
  }
}

bool IsIntersectionInternal(const uint64_t startnode, const uint64_t endnode,
                            const uint32_t edgeindex, const uint64_t wayid,
                            const float length,
                            const std::unordered_map<uint64_t, Node>& nodes,
                            const std::vector<Edge>& edges,
                            const WayVector& ways) {
  // Limit the length of intersection internal edges
  if (length > kMaxInternalLength) {
    return false;
  }

  // Both end nodes must connect to at least 3 edges
  const auto& node1 = nodes.find(startnode)->second;
  if (node1.edge_count() < 3) {
    return false;
  }
  const auto& node2 = nodes.find(endnode)->second;
  if (node2.edge_count() < 3) {
    return false;
  }

  // Each node must have a pair of oneways (one inbound and one outbound)
  if (!OnewayPairEdgesExist(node1, edgeindex, wayid, edges, ways) ||
      !OnewayPairEdgesExist(node2, edgeindex, wayid, edges, ways)) {
    return false;
  }

  // Assume this is an intersection internal edge
  return true;
}

/**
 * Get the use for a link (either a kRamp or kTurnChannel)
 * TODO - validate logic with some real world cases.
 */
Use GetLinkUse(const RoadClass rc, const float length,
               const uint64_t startnode,  const uint64_t endnode,
               const std::unordered_map<uint64_t, Node>& nodes) {
  // Assume link that has highway = motorway or trunk is a ramp.
  // Also, if length is > kMaxTurnChannelLength we assume this is a ramp
  if (rc == RoadClass::kMotorway || rc == RoadClass::kTrunk ||
      length > kMaxTurnChannelLength) {
    return Use::kRamp;
  }

  // TODO - if there is a exit sign or exit number present this is
  // considered kRamp

  // Both end nodes have to connect to a non-link edge. If either end node
  // connects only to "links" this likely indicates a split or fork,
  // which are not so prevalent in turn channels.
  auto startnd = nodes.find(startnode);
  if (startnd != nodes.end()) {
    auto endnd = nodes.find(endnode);
    if (endnd != nodes.end()) {
      if (startnd->second.non_link_edge() && endnd->second.non_link_edge())
        return Use::kTurnChannel;
    }
  }
  return Use::kRamp;
}

float UpdateLinkSpeed(const Use use, const RoadClass rc, const float spd) {
  if (use == Use::kTurnChannel) {
    return spd * 1.25f;
  } else if (use == Use::kRamp) {
    if (rc == RoadClass::kMotorway) {
      return 95.0f;
    } else if (rc == RoadClass::kTrunk) {
      return 80.0f;
    } else if (rc == RoadClass::kPrimary) {
      return 65.0f;
    } else if (rc == RoadClass::kSecondary) {
      return 50.0f;
    } else if (rc == RoadClass::kTertiaryUnclassified) {
      return 40.0f;
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

void CheckForDuplicates(const uint64_t osmnodeid, const Node& node,
                        const std::vector<uint32_t>& edgelengths,
                        const std::unordered_map<uint64_t, Node>& nodes,
                        const std::vector<Edge>& edges,
                        const WayVector& ways, std::atomic<DataQuality*>& stats) {
  uint32_t edgeindex;
  uint64_t endnode;
  std::unordered_map<uint64_t, DuplicateEdgeInfo> endnodes;
  for (size_t n = 0; n < node.edge_count(); n++) {
    edgeindex = node.edges().at(n);
    const Edge& edge = edges[edgeindex];
    if (edge.sourcenode_ == osmnodeid) {
      endnode = nodes.find(edge.targetnode_)->second.graphid().value;
    } else {
      endnode = nodes.find(edge.sourcenode_)->second.graphid().value;
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
  }
}

void BuildTileSet(
    std::unordered_map<GraphId, std::vector<uint64_t> >::const_iterator tile_start,
    std::unordered_map<GraphId, std::vector<uint64_t> >::const_iterator tile_end,
    const std::unordered_map<uint64_t, Node>& nodes,
    const WayVector& ways, const std::vector<Edge>& edges,
    const baldr::TileHierarchy& hierarchy,
    const std::unordered_map<uint64_t, std::string>& map_ref,
    const std::unordered_map<uint64_t, std::string>& map_name,
    const std::unordered_map<uint64_t, std::string>& map_exit_to,
    std::atomic<DataQuality*>& stats,
    std::promise<size_t>& result) {

  std::string thread_id = static_cast<std::ostringstream&>(std::ostringstream()
        << std::this_thread::get_id()).str();
  LOG_INFO("Thread " + thread_id + " started");

  // A place to keep information about what was done
  size_t written = 0;

  // For each tile in the task
  bool added = false;
  bool not_thru, forward, internal;
  uint32_t edgeindex;
  int64_t source, target;
  for(; tile_start != tile_end; ++tile_start) {
    try {
     // What actually writes the tile
      GraphTileBuilder graphtile;

      // Iterate through the nodes
      uint32_t directededgecount = 0;
      for (const auto& osmnodeid : tile_start->second) {
        const Node& node = nodes.find(osmnodeid)->second; //TODO: check validity?

        // Compute edge lengths from the edge lat,lngs (round to nearest meter)
        std::vector<uint32_t> edgelengths;
        for (auto edgeindex : node.edges()) {
          float length = PointLL().Length(edges[edgeindex].shape());
          edgelengths.push_back(static_cast<uint32_t>(length + 0.5f));
        }

        // Look for potential duplicates
        CheckForDuplicates(osmnodeid, node, edgelengths, nodes, edges,
                           ways, stats);

        // Build directed edges. Track the best classification/importance
        // of outbound edges from this node.
        RoadClass bestclass = RoadClass::kOther;
        std::vector<DirectedEdgeBuilder> directededges;
        for (size_t n = 0; n < node.edge_count(); n++) {
          // Get the edge and way
          edgeindex = node.edges().at(n);
          const Edge& edge = edges[edgeindex];
          const OSMWay &w = ways[edge.wayindex_];

          // Assign nodes
          const GraphId& nodea = nodes.find(edge.sourcenode_)->second.graphid();
          if (!nodea.Is_Valid()) {
            throw std::runtime_error("NodeA: OSMID = " +
                 std::to_string(edge.sourcenode_) + " GraphId is not valid");
          }
          const GraphId& nodeb = nodes.find(edge.targetnode_)->second.graphid();
          if (!nodeb.Is_Valid()) {
            throw std::runtime_error("NodeB: OSMID = " +
                 std::to_string(edge.targetnode_) + " GraphId is not valid");
          }

          // Determine orientation along the edge (forward or reverse between
          // the 2 nodes). Check for edge error.
          if (edge.sourcenode_ == osmnodeid) {
            forward = true;
            source = edge.sourcenode_;
            target = edge.targetnode_;
          } else if (edge.targetnode_ == osmnodeid) {
            forward = false;
            source = edge.targetnode_;
            target = edge.sourcenode_;
          } else {
            // ERROR!!!
            LOG_ERROR((boost::format("WayID =  %1% Edge Index = %2% Edge nodes %3% and %4% did not match the OSM node Id %5%")
              % w.way_id() % edgeindex %  edge.sourcenode_  % edge.targetnode_ % osmnodeid).str());
          }

          // Check for not_thru edge (only on low importance edges)
          if (edge.attributes_.fields.importance <=
              static_cast<uint32_t>(RoadClass::kTertiaryUnclassified)) {
            not_thru = false;
          } else {
            not_thru = IsNoThroughEdge(source, target, edgeindex,
                             nodes, edges);
          }

          // Test if an internal intersection edge
          internal = IsIntersectionInternal(source, target, edgeindex,
                  w.way_id(), edgelengths[n], nodes, edges, ways);

          // Set the end node
          const GraphId& endnode = (forward) ? nodeb : nodea;

          // If link is set test to see if we can infer that the edge
          // is a turn channel. Update speed for link edges.
          float speed = w.speed();
          RoadClass rc = static_cast<RoadClass>(edge.attributes_.fields.importance);
          Use use = w.use();
          if (w.link()) {
            if (use != Use::kNone) {
              (*stats).AddIssue(kIncompatibleLinkUse, GraphId(), w.way_id(), 0);
            }
            use   = GetLinkUse(rc, edgelengths[n], edge.sourcenode_,
                                edge.targetnode_, nodes);
            speed = UpdateLinkSpeed(use, rc, w.speed());
          }

          //Does the directed edge contain exit information?
          bool has_exitinfo = (node.ref() || node.name() || node.exit_to() || w.exit());

          // Add a directed edge and get a reference to it
          directededges.emplace_back(w, endnode, forward, edgelengths[n],
                        speed, use, not_thru, has_exitinfo, internal, rc);
          DirectedEdgeBuilder& directededge = directededges.back();

          // Update the node's best class
          bestclass = std::min(bestclass, directededge.importance());

          // Add edge info to the tile and set the offset in the directed edge
          uint32_t edge_info_offset = graphtile.AddEdgeInfo(edgeindex,
               nodea, nodeb, edge.shape(), w.GetNames(),
               graphbuilder::CreateExitSignInfoList(osmnodeid, node, w, map_ref, map_name, map_exit_to),
               added);
          directededge.set_edgeinfo_offset(edge_info_offset);

          // Add to general statistics
          (*stats).AddStats(tile_start->first, directededge);
        }

        // Set the node lat,lng, index of the first outbound edge, and the
        // directed edge count from this edge and the best road class
        // from the node. Increment directed edge count.
        NodeInfoBuilder nodebuilder(node.latlng(), directededgecount,
                                    node.edge_count(), bestclass);
        directededgecount += node.edge_count();

        // Add node and directed edge information to the tile
        graphtile.AddNodeAndDirectedEdges(nodebuilder, directededges);
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
  result.set_value(written);
}

}

std::vector<ExitSignInfo> GraphBuilder::CreateExitSignInfoList(
    const uint64_t osmnodeid, const Node& node, const OSMWay& way,
    const std::unordered_map<uint64_t, std::string>& map_ref,
    const std::unordered_map<uint64_t, std::string>& map_name,
    const std::unordered_map<uint64_t, std::string>& map_exit_to) {

  std::vector<ExitSignInfo> exit_list;

  // Exit sign number
  if (!way.junction_ref().empty()) {
    exit_list.emplace_back(ExitSign::Type::kNumber, way.junction_ref());
  }  else if (node.ref()) {
    exit_list.emplace_back(ExitSign::Type::kNumber, map_ref.find(osmnodeid)->second);
  }

  // Exit sign branch refs
  bool has_branch = false;
  if (!way.destination_ref().empty()) {
    has_branch = true;
    std::vector<std::string> branch_refs = GetTagTokens(way.destination_ref());
    for (auto& branch_ref : branch_refs) {
      exit_list.emplace_back(ExitSign::Type::kBranch, branch_ref);
    }
  }

  // Exit sign toward refs
  bool has_toward = false;
  if (!way.destination_ref_to().empty()) {
    has_toward = true;
    std::vector<std::string> toward_refs = GetTagTokens(way.destination_ref_to());
    for (auto& toward_ref : toward_refs) {
      exit_list.emplace_back(ExitSign::Type::kToward, toward_ref);
    }
  }

  // Exit sign toward names
  if (!way.destination().empty()) {
    has_toward = true;
    std::vector<std::string> toward_names = GetTagTokens(way.destination());
    for (auto& toward_name : toward_names) {
      exit_list.emplace_back(ExitSign::Type::kToward, toward_name);
    }
  }

  // Process exit_to only if other branch or toward info does not exist
  if (!has_branch && !has_toward) {
    if (node.exit_to()) {

      std::string tmp;
      std::size_t pos;

      std::vector<std::string> exit_tos = GetTagTokens(map_exit_to.find(osmnodeid)->second);
      for (auto& exit_to : exit_tos) {

        tmp = exit_to;

        boost::algorithm::to_lower(tmp);

        //remove the "To" For example:  US 11;To I 81;Carlisle;Harrisburg
        if (boost::starts_with(tmp, "to ")) {
            exit_list.emplace_back(ExitSign::Type::kToward, exit_to.substr(3));
            continue;
        }
        //remove the "Toward" For example:  US 11;Toward I 81;Carlisle;Harrisburg
        if (boost::starts_with(tmp, "toward ")) {
            exit_list.emplace_back(ExitSign::Type::kToward, exit_to.substr(7));
            continue;
        }

        std::size_t found = tmp.find(" to ");

        //Default to kToward if found twice or "toward" found as well; otherwise, <branch> to <toward>
        //For example:  I 95 to I 695
        if (found != std::string::npos &&
           (tmp.find(" to ",found+4) == std::string::npos && tmp.find(" toward ") == std::string::npos)) {

            exit_list.emplace_back(ExitSign::Type::kBranch, exit_to.substr(0,found));

            exit_list.emplace_back(ExitSign::Type::kToward, exit_to.substr(found+4));
            continue;
        }

        found = tmp.find(" toward ");

        //Default to kToward if found twice or "to" found as well; otherwise, <branch> toward <toward>
        //For example:  I 95 to I 695
        if (found != std::string::npos &&
            (tmp.find(" toward ",found+8) == std::string::npos && tmp.find(" to ") == std::string::npos)) {

          exit_list.emplace_back(ExitSign::Type::kBranch, exit_to.substr(0,found));

          exit_list.emplace_back(ExitSign::Type::kToward, exit_to.substr(found+8));
          continue;
        }

        //default to toward.
        exit_list.emplace_back(ExitSign::Type::kToward, exit_to);

      }
    }
  }

  // Exit sign name
  if (node.name()) {
    std::vector<std::string> names = GetTagTokens(map_name.find(osmnodeid)->second);
    for (auto& name : names) {
      exit_list.emplace_back(ExitSign::Type::kName, name);
    }
  }

  return exit_list;
}

void GraphBuilder::TileNodes(const float tilesize, const uint8_t level) {
  LOG_INFO("Tiling nodes");

  // Get number of tiles and reserve space for them
  // < 30% of the earth is land and most roads are on land, even less than that even has roads
  Tiles tiles(AABB2({-180.0f, -90.0f}, {180.0f, 90.0f}), tilesize);
  tilednodes_.reserve(tiles.TileCount() * .3f);
  // Iterate through all OSM nodes and assign GraphIds
  for (auto& node : nodes_) {
    // Skip any nodes that have no edges
    if (node.second.edge_count() == 0) {
      continue;
    }
    // Put the node into the tile
    GraphId id = tile_hierarchy_.GetGraphId(static_cast<midgard::PointLL>(node.second.latlng()), level);
    std::vector<uint64_t>& tile = tilednodes_[id];
    tile.emplace_back(node.first);
    // Set the GraphId for this OSM node.
    node.second.set_graphid(GraphId(id.tileid(), id.level(), tile.size() - 1));
  }

  LOG_INFO("Tiled nodes created");
}

// Build tiles for the local graph hierarchy
void GraphBuilder::BuildLocalTiles(const uint8_t level,
                                   const WayVector& ways,
                                   const std::unordered_map<uint64_t, std::string>& node_ref,
                                   const std::unordered_map<uint64_t, std::string>& node_exit_to,
                                   const std::unordered_map<uint64_t, std::string>& node_name) const {
  // A place to hold worker threads and their results, be they exceptions or otherwise
  std::vector<std::shared_ptr<std::thread> > threads(threads_);
  // A place to hold the results of those threads, be they exceptions or otherwise
  std::vector<std::promise<size_t> > results(threads.size());
  // Divvy up the work
  size_t floor = tilednodes_.size() / threads.size();
  size_t at_ceiling = tilednodes_.size() - (threads.size() * floor);
  std::unordered_map<GraphId, std::vector<uint64_t> >::const_iterator tile_start, tile_end = tilednodes_.begin();
  // Atomically pass around stats info
  std::atomic<DataQuality*> atomic_stats(stats_.get());
  LOG_INFO(std::to_string(tilednodes_.size()) + " tiles");
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    tile_start = tile_end;
    // Where the range ends
    std::advance(tile_end, tile_count);
    // Make the thread
    threads[i].reset(
      new std::thread(BuildTileSet, tile_start, tile_end, nodes_, ways, edges_,
                      tile_hierarchy_, node_ref, node_name, node_exit_to,
                      std::ref(atomic_stats), std::ref(results[i]))
    );
  }

  // Join all the threads to wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  // Check all of the outcomes
  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      auto pass_fail = result.get_future().get();
      //TODO: print out stats about how many tiles or bytes were written by the thread?
    }
    catch(std::exception& e) {
      //TODO: throw further up the chain?
    }
  }

  /*// Wait for results to come back from the threads, logging what happened and removing the result
  auto process_result = [] (std::unordered_map<GraphId, std::future<size_t> >& results) {
    // For each task
    for (std::unordered_map<GraphId, std::future<size_t> >::iterator result = results.begin(); result != results.end(); ++result) {
      try {
        // Block until the result is ready
        auto status = result->second.wait_for(std::chrono::microseconds(1));
        // If it was ready
        if (status == std::future_status::ready) {
          size_t bytes = result->second.get();
          LOG_INFO((boost::format("Wrote tile %1%: %2% bytes") % result->first % bytes).str());
          results.erase(result);
          break;
        }
      }
      catch (const std::exception& e) {
        //TODO: log and rethrow
        LOG_ERROR((boost::format("Failed tile %1%: %2%") % result->first % << e.what()).str());
        results.erase(result);
        break;
      }
    }
  };

  // Process each tile asynchronously
  std::unordered_map<GraphId, std::future<size_t> > results(8);
  for (const auto& tile : tilednodes_) {
    // If we can squeeze this task in
    if(results.size() < kMaxInFlightTasks) {
      // Build the tile
      results.emplace(tile.first, std::async(std::launch::async, BuildTile, tile, nodes_, ways_, edges_, tile_hierarchy_.tile_dir()));
      continue;
    }

    // If We already have too many in flight wait for some to finish
    while(results.size() >= kMaxInFlightTasks) {
      // Try to get some results
      process_result(results);
    }
  }

  // We're done adding tasks but there may be more results
  while(results.size())
    process_result(results);*/
}


}
}
