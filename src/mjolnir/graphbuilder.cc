
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
#include <valhalla/baldr/signinfo.h>

#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/edgeinfobuilder.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

uint32_t simplerestrictions = 0;
uint32_t timedrestrictions = 0;

// Number of tries when determining not thru edges
constexpr uint32_t kMaxNoThruTries = 256;

// Absurd classification.
constexpr uint32_t kAbsurdRoadClass = 777777;

// Construct GraphBuilder based on properties file and input PBF extract
GraphBuilder::GraphBuilder(const boost::property_tree::ptree& pt)
    : level_(0),
      tile_hierarchy_(pt.get_child("hierarchy")),
      stats_(new DataQuality()),
      threads_(std::max(static_cast<unsigned int>(1),
               pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency()))){
}

// Delete the OSM node map and extended node information maps.
void delete_osmnode_map(OSMData& osmdata) {
  OSMNodeMap().swap(osmdata.nodes);
  OSMStringMap().swap(osmdata.node_exit_to);
  OSMStringMap().swap(osmdata.node_ref);
  OSMStringMap().swap(osmdata.node_name);
}

// Delete the OSM node map.
void delete_node_map(std::unordered_map<uint64_t, GraphId>& nodemap) {
  std::unordered_map<uint64_t, GraphId>().swap(nodemap);
}

// Delete the OSM node reference vector.
void delete_noderefs(OSMData& osmdata) {
  NodeRefVector().swap(osmdata.noderefs);
}

// Build the graph from the input
void GraphBuilder::Build(OSMData& osmdata) {
  // Construct edges
  auto t1 = std::chrono::high_resolution_clock::now();
  const auto& tl = tile_hierarchy_.levels().rbegin();
  level_ = tl->second.level;
  ConstructEdges(osmdata, tl->second.tiles.TileSize());
  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
  LOG_INFO("ConstructEdges took " + std::to_string(msecs) + " ms");

  // Create mappings of extended node info by GraphId rather than
  // OSM node Id.
  CreateNodeMaps(osmdata);

  // Update restrictions - replace OSM node Id in via with a GraphId
  UpdateRestrictions(osmdata);

  // Try to recover memory by swapping empty maps/vectors for data we no
  // longer need.
  delete_osmnode_map(osmdata);
  delete_noderefs(osmdata);
  delete_node_map(nodes_);

  // Reclassify links (ramps). Cannot do this when building tiles since the
  // edge list needs to be modified
  t1 = std::chrono::high_resolution_clock::now();
  ReclassifyLinks(osmdata.ways);
  t2 = std::chrono::high_resolution_clock::now();
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
  LOG_INFO("ReclassifyLinks took " + std::to_string(msecs) + " ms");

  // Build tiles at the local level. Form connected graph from nodes and edges.
  t1 = std::chrono::high_resolution_clock::now();
  LOG_INFO("BuildLocalTile using " + std::to_string(threads_) + " threads");
  BuildLocalTiles(tl->second.level, osmdata);
  t2 = std::chrono::high_resolution_clock::now();
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
  LOG_INFO("BuildLocalTiles took " + std::to_string(msecs) + " ms");

  // Log statistics and issues
  stats_->Log();

  LOG_INFO("Simple Restriction Count = " + std::to_string(simplerestrictions));
  LOG_INFO("Timed  Restriction Count = " + std::to_string(timedrestrictions));
}

// Add a node to the appropriate tile.
GraphId GraphBuilder::AddNodeToTile(const uint64_t osmnodeid,
                                    const OSMNode& osmnode,
                                    const uint32_t edgeindex,
                                    const bool link) {
  // Get the tile
  GraphId id = tile_hierarchy_.GetGraphId(
      static_cast<midgard::PointLL>(osmnode.latlng()), level_);
  std::vector<Node>& tile = tilednodes_[id];

  // Add a new Node to the tile
  tile.emplace_back(osmnode.attributes(), edgeindex,link);

  // Set the GraphId for this OSM node.
  GraphId graphid(id.tileid(), id.level(), tile.size() - 1);
  nodes_[osmnodeid] = graphid;
  return graphid;
}

// Get a node from tilednodes_ given its graph Id
Node& GraphBuilder::GetNode(const GraphId& graphid) {
  return tilednodes_[graphid.Tile_Base()][graphid.id()];
}

// Construct edges in the graph and assign nodes to tiles.
void GraphBuilder::ConstructEdges(const OSMData& osmdata, const float tilesize) {
  // Reserve size for the Node map
  nodes_.reserve(osmdata.intersection_count);

  // Get number of tiles and reserve space for them
  // < 30% of the earth is land and most roads are on land
  // (less that have roads)
  Tiles tiles(AABB2({-180.0f, -90.0f}, {180.0f, 90.0f}), tilesize);
  tilednodes_.reserve(tiles.TileCount() * .3f);

  // Iterate through the OSM ways
  uint32_t edgeindex = 0;
  uint64_t startnodeid, nodeid;
  GraphId graphid;
  edges_.reserve(osmdata.edge_count);
  for (size_t wayindex = 0; wayindex < osmdata.ways.size(); wayindex++) {
    // Get some way attributes needed for the edge
    const auto& way = osmdata.ways[wayindex];

    // Get the OSM node information for the first node of the way
    startnodeid = nodeid = osmdata.noderefs[way.noderef_index()];
    const auto& osmnode = osmdata.nodes.find(nodeid)->second;

    // If a graph Node exists add an edge to it, otherwise construct a
    // graph Node with an initial edge and add it to the appropriate tile.
    auto it = nodes_.find(nodeid);
    if (it == nodes_.end()) {
      graphid = AddNodeToTile(nodeid, osmnode, edgeindex, way.link());
    }
    else {
      graphid = it->second;
      GetNode(graphid).AddEdge(edgeindex, way.link());
    }

    // Start an edge at the first node of the way. Add the node lat,lng
    // to the list.
    Edge edge(graphid, wayindex, latlngs_.size(), way);
    latlngs_.push_back(osmnode.latlng());

    // Iterate through the nodes of the way and add lat,lng to the current
    // way until a node with > 1 uses is found.
    for (size_t i = 1; i < way.node_count(); i++) {
      // Add the node lat,lng to the edge shape.
      nodeid  = osmdata.noderefs[way.noderef_index() + i];
      const auto& osmnode = osmdata.nodes.find(nodeid)->second;

      // Add the node's lat,lng to the latlng list and increment the count
      // for this edge
      latlngs_.push_back(osmnode.latlng());
      edge.attributes.llcount++;

      // If a is an intersection or the end of the way
      // it's a node of the road network graph
      if (osmnode.intersection()) {
        // End the current edge and add its edge index to the node
        // If a graph node exists add an edge to it, otherwise construct a
        // graph node, add an edge and add it to the map
        auto it = nodes_.find(nodeid);
        if (it == nodes_.end()) {
          graphid = AddNodeToTile(nodeid, osmnode, edgeindex, way.link());
        }
        else {
          // Add the edgeindex to the node (unless this is a loop with same
          // start and end node Ids)
          if (startnodeid != nodeid) {
            graphid = it->second;
            GetNode(graphid).AddEdge(edgeindex, way.link());
          }
        }

        // Set the target (end) node of the edge
        edge.targetnode_ = graphid;

        // Add the edge to the list of edges
        edges_.emplace_back(std::move(edge));
        edgeindex++;

        // Start a new edge if this is not the last node in the way.
        // We can reuse the index of the latlng added above
        if (i < way.node_count() - 1) {
          startnodeid = nodeid;
          edge = Edge(graphid, wayindex, latlngs_.size()-1, way);
          GetNode(graphid).AddEdge(edgeindex, way.link());
        }
      }
    }
  }

  // Shrink the latlngs vector and the edges vector to fit
  latlngs_.shrink_to_fit();
  edges_.shrink_to_fit();

  // Iterate through the tilednodes and shrink vectors
  for (auto& tile : tilednodes_) {
    tile.second.shrink_to_fit();
  }

  LOG_INFO("Constructed " + std::to_string(edges_.size()) + " edges and "
            + std::to_string(nodes_.size()) + " nodes");
  LOG_INFO("LatLng count for all edges: " + std::to_string(latlngs_.size()));
}

// Create the extended node information mapped by the node's GraphId.
// This is needed since we do not keep osmnodeid around.
void GraphBuilder::CreateNodeMaps(const OSMData& osmdata) {
  node_exit_to_.reserve(osmdata.node_exit_to.size());
  for (const auto& it : osmdata.node_exit_to) {
    const auto nd = nodes_.find(it.first);
    if (nd == nodes_.end()) {
      LOG_INFO("exit_to on a non-graph node");
    } else {
      node_exit_to_[nd->second] = it.second;
    }
  }
  node_ref_.reserve(osmdata.node_ref.size());
  for (const auto& it : osmdata.node_ref) {
    const auto nd = nodes_.find(it.first);
    if (nd == nodes_.end()) {
      LOG_INFO("node ref on a non-graph node");
    } else {
      node_ref_[nd->second] = it.second;
    }
  }
  node_name_.reserve(osmdata.node_name.size());
  for (const auto& it : osmdata.node_name) {
    const auto nd = nodes_.find(it.first);
    if (nd == nodes_.end()) {
      LOG_INFO("node name on a non-graph node");
    } else {
      node_name_[nd->second] = it.second;
    }
  }
}

// Create the extended node information mapped by the node's GraphId.
// This is needed since we do not keep osmnodeid around.
void GraphBuilder::UpdateRestrictions(OSMData& osmdata) {
  for (auto& rst : osmdata.restrictions) {
    const auto nd = nodes_.find(rst.second.via());
    if (nd == nodes_.end()) {
      LOG_INFO("Restriction Via node on a non-graph node");
    } else {
      rst.second.set_via(nd->second);
    }
  }
}

// Gets the most important class among the node's edges
uint32_t GraphBuilder::GetBestNonLinkClass(const Node& node) const {
  uint32_t bestrc = kAbsurdRoadClass;
  for (auto idx : node.edges) {
    const Edge& edge = edges_[idx];
    if (!edge.attributes.link) {
      if (edge.attributes.importance < bestrc)
        bestrc = edge.attributes.importance;
    }
  }
  return bestrc;
}

// Reclassify links (ramps and turn channels).
void GraphBuilder::ReclassifyLinks(const WayVector& ways) {
  uint32_t count = 0;
  std::unordered_set<GraphId> visitedset;  // Set of visited nodes
  std::unordered_set<GraphId> expandset;   // Set of nodes to expand
  std::list<uint32_t> linkedgeindexes;     // Edge indexes to reclassify

  for (const auto& tile : tilednodes_) {
    if (tile.second.size() == 0) {
      continue;
    }

    for (const auto& node : tile.second) {
      GraphId nodeid(tile.first.tileid(), tile.first.level(), 0);
      if (node.link_edge() && node.non_link_edge()) {
        // Get the highest classification of non-link edges at this node
        uint32_t beststartrc = GetBestNonLinkClass(node);

        // Expand from all link edges
        for (auto startedgeindex : node.edges) {
          // Get the edge information. Skip non-link edges
          const Edge& startedge = edges_[startedgeindex];
          if (!startedge.attributes.link) {
            continue;
          }

          // Clear the sets and edge list
          visitedset.clear();
          expandset.clear();
          linkedgeindexes.clear();

          // Add start edge to list of links edges to potentially reclassify
          linkedgeindexes.push_back(startedgeindex);

          // If the first end node contains a non-link edge we compute the best
          // classification. If not we add the end node to the expand set.
          uint32_t bestendrc = kAbsurdRoadClass;
          GraphId endnode = (startedge.sourcenode_ == nodeid) ?
              startedge.targetnode_ : startedge.sourcenode_;
          Node& firstendnode = GetNode(endnode);
          if (firstendnode.non_link_edge()) {
            bestendrc = GetBestNonLinkClass(firstendnode);
          } else {
            expandset.insert(endnode);
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
                  if (rc > edges_[idx].attributes.importance) {
                    edges_[idx].attributes.importance = rc;
                    count++;
                  }
                }
              }
              break;
            }

            // Get the node off of the expand list and add it to the visited list
            GraphId expandnode = *expandset.begin();
            expandset.erase(expandset.begin());
            visitedset.insert(expandnode);

            // Expand all edges from this node
            Node& nd = GetNode(expandnode);
            for (auto edgeindex : nd.edges) {
              // Do not allow use of the start edge
              if (edgeindex == startedgeindex) {
                continue;
              }

              // Add this edge (it should be a link edge) and get its end node
              const Edge& nextedge = edges_[edgeindex];
              if (!nextedge.attributes.link) {
                LOG_ERROR("Expanding onto non-link edge!");
                continue;
              }
              GraphId nextendnode = (nextedge.sourcenode_ == expandnode) ?
                      nextedge.targetnode_ : nextedge.sourcenode_;
              linkedgeindexes.push_back(edgeindex);
              Node& endnd = GetNode(nextendnode);

              // If the end node contains any non-links find the best
              // classification - do not expand from this node
              if (endnd.non_link_edge()) {
                uint32_t rc = GetBestNonLinkClass(endnd);
                if (rc < bestendrc) {
                  bestendrc = rc;
                }
              } else if (visitedset.find(nextendnode) == visitedset.end()) {
                // Add to the expand set if not in the visited set
                expandset.insert(nextendnode);
              }
            }
          }
        }
      }

      // Increment the node GraphId for the next node
      nodeid++;
    }
  }
  LOG_INFO((boost::format("Reclassify Links: Count %1%") % count).str());
}


namespace {

class graphbuilder : public GraphBuilder {
 public:
  using GraphBuilder::CreateExitSignInfoList;
  using GraphBuilder::GetRef;
};

const Node& GetNode(const GraphId& nodeid,
             const std::unordered_map<GraphId, std::vector<Node> >& nodes) {
  return nodes.find(nodeid.Tile_Base())->second.at(nodeid.id());
 // return nodes[nodeid.Tile_Base()][nodeid.id()];
}

// Method to get the shape for an edge - since LL is stored as a pair of
// floats we need to change into PointLL to get length of an edge
std::vector<PointLL> EdgeShape(const std::vector<OSMLatLng>& latlngs,
                     const uint32_t idx, const uint32_t count) {
  std::vector<PointLL> lls;
  for (uint32_t i = idx, n = idx + count; i < n; i++) {
    lls.emplace_back(latlngs[i].first, latlngs[i].second);
  }
  return lls;
}

// Get the lat,lng of the start of the edge
PointLL GetLL(const std::vector<OSMLatLng>& latlngs, const uint32_t idx) {
  return PointLL(latlngs[idx].first, latlngs[idx].second);
}

// Test if this is a "not thru" edge. These are edges that enter a region that
// has no exit other than the edge entering the region
bool IsNoThroughEdge(const GraphId& startnode, const GraphId& endnode,
             const uint32_t startedgeindex,
             const std::unordered_map<GraphId, std::vector<Node>>& nodes,
             const std::vector<Edge>& edges) {
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
      const Edge& edge = edges[edgeindex];
      GraphId nextendnode = (edge.sourcenode_ == node) ?
                edge.targetnode_ : edge.sourcenode_;
      if (nextendnode == startnode ||
          edge.attributes.importance <=
          static_cast<uint32_t>(RoadClass::kTertiaryUnclassified)) {
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
  for (const auto idx : node.edges) {
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
  return (inbound && outbound);
}

bool IsIntersectionInternal(const GraphId& startnode, const GraphId& endnode,
                const uint32_t edgeindex, const uint64_t wayid,
                const float length,
                const std::unordered_map<GraphId, std::vector<Node>>& nodes,
                const std::vector<Edge>& edges,
                const WayVector& ways) {
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
               const GraphId& startnode,  const GraphId& endnode,
               const std::unordered_map<GraphId, std::vector<Node>>& nodes) {
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
  const Node& startnd = GetNode(startnode, nodes);
  const Node& endnd   = GetNode(endnode, nodes);
  return (startnd.non_link_edge() && endnd.non_link_edge()) ?
            Use::kTurnChannel : Use::kRamp;
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

void CheckForDuplicates(const GraphId& nodeid, const Node& node,
                const std::vector<uint32_t>& edgelengths,
                const std::unordered_map<GraphId, std::vector<Node>>& nodes,
                const std::vector<Edge>& edges,
                const WayVector& ways, std::atomic<DataQuality*>& stats) {
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

void BuildTileSet(
    std::unordered_map<GraphId, std::vector<Node> >::const_iterator tile_start,
    std::unordered_map<GraphId, std::vector<Node> >::const_iterator tile_end,
    const std::unordered_map<GraphId, std::vector<Node> >& nodes,
    const std::vector<Edge>& edges,
    const std::vector<OSMLatLng>& latlngs,
    const baldr::TileHierarchy& hierarchy,
    const OSMData& osmdata,
    const std::unordered_map<baldr::GraphId, std::string>& node_ref,
    const std::unordered_map<baldr::GraphId, std::string>& node_exit_to,
    const std::unordered_map<baldr::GraphId, std::string>& node_name,
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
  GraphId source, target;
  PointLL node_ll;
  for(; tile_start != tile_end; ++tile_start) {
    try {
     // What actually writes the tile
      GraphTileBuilder graphtile;

      // Iterate through the nodes
      uint32_t idx = 0;                 // Current directed edge index
      uint32_t directededgecount = 0;
      GraphId nodeid = tile_start->first.Tile_Base();
      for (const Node& node : tile_start->second) {

        // Compute edge lengths from the edge lat,lngs (round to nearest meter)
        std::vector<uint32_t> edgelengths;
        for (auto edgeindex : node.edges) {
          std::vector<PointLL> shape = EdgeShape(latlngs,
                      edges[edgeindex].llindex_,
                      edges[edgeindex].attributes.llcount);
          float length = PointLL().Length(shape);
          edgelengths.push_back(static_cast<uint32_t>(length + 0.5f));
        }

        // Look for potential duplicates
        // TODO - need to make stats thread safe
//        CheckForDuplicates(nodeid, node, edgelengths, nodes, edges,
//                           osmdata.ways, stats);

        // Build directed edges. Track the best classification/importance
        // of outbound edges from this node.
        uint32_t n = 0;
        RoadClass bestclass = RoadClass::kOther;
        std::vector<DirectedEdgeBuilder> directededges;
        for (auto edgeindex : node.edges) {
          // Get the edge and way
          const Edge& edge = edges[edgeindex];
          const OSMWay &w = osmdata.ways[edge.wayindex_];

          // Get the 2 end nodes of the edge
          const Node& nodea = GetNode(edge.sourcenode_, nodes);
          const Node& nodeb = GetNode(edge.targetnode_, nodes);

          // Determine orientation along the edge (forward or reverse between
          // the 2 nodes). Check for edge error.
          if (edge.sourcenode_ == nodeid) {
            forward = true;
            source = edge.sourcenode_;
            target = edge.targetnode_;
            node_ll = GetLL(latlngs, edges[edgeindex].llindex_);
          } else if (edge.targetnode_ == nodeid) {
            forward = false;
            source = edge.targetnode_;
            target = edge.sourcenode_;
            node_ll = GetLL(latlngs,
                   edges[edgeindex].llindex_ + edges[edgeindex].attributes.llcount - 1);
          } else {
            // ERROR!!!
            LOG_ERROR((boost::format("WayID =  %1% Edge Index = %2% Edge nodes %3% and %4% did not match the OSM node Id %5%")
              % w.way_id() % edgeindex %  edge.sourcenode_  % edge.targetnode_ % nodeid).str());
          }

          // Check for not_thru edge (only on low importance edges)
          if (edge.attributes.importance <=
              static_cast<uint32_t>(RoadClass::kTertiaryUnclassified)) {
            not_thru = false;
          } else {
            not_thru = IsNoThroughEdge(source, target, edgeindex,
                             nodes, edges);
          }

          // Test if an internal intersection edge
          internal = IsIntersectionInternal(source, target, edgeindex,
                 w.way_id(), edgelengths[n], nodes, edges, osmdata.ways);

          // If link is set test to see if we can infer that the edge
          // is a turn channel. Update speed for link edges.
          float speed = w.speed();
          RoadClass rc = static_cast<RoadClass>(edge.attributes.importance);
          Use use = w.use();
          if (w.link()) {
//            if (use != Use::kNone) {
//              (*stats).AddIssue(kIncompatibleLinkUse, GraphId(), w.way_id(), 0);
 //           }
            use   = GetLinkUse(rc, edgelengths[n], edge.sourcenode_,
                                edge.targetnode_, nodes);
            speed = UpdateLinkSpeed(use, rc, w.speed());
          }

          // Does the directed edge contain exit information?
          //bool has_exitinfo = (node.ref() || node.name() || node.exit_to() || w.exit());

          // Add a directed edge and get a reference to it
          directededges.emplace_back(w, target, forward, edgelengths[n],
                        speed, use, not_thru, internal, rc);
          DirectedEdgeBuilder& directededge = directededges.back();

          // Update the node's best class
          bestclass = std::min(bestclass, directededge.importance());

          // Check for updated ref from relations.
          std::string ref;
          auto iter = osmdata.way_ref.find(w.way_id());
          if (iter != osmdata.way_ref.end()) {
            if (w.ref_index() != 0)
              ref = graphbuilder::GetRef(osmdata.ref_offset_map.name(w.ref_index()),iter->second);
          }

          // Add edge info to the tile and set the offset in the directed edge
          uint32_t edge_info_offset = graphtile.AddEdgeInfo(edgeindex,
                   source, target,
                   EdgeShape(latlngs, edges[edgeindex].llindex_,
                             edges[edgeindex].attributes.llcount),
                   w.GetNames(ref, osmdata.ref_offset_map,
                              osmdata.name_offset_map), added);
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

          // Handle restrictions
          auto res = osmdata.restrictions.equal_range(w.way_id());
          if (res.first != osmdata.restrictions.end()) {
            // Edge is the from edge of a restriction
            for (auto r = res.first; r != res.second; ++r) {
              if (r->second.via_graphid() == target) {
                if (r->second.day_on() != DOW::kNone) {
                  timedrestrictions++;
                } else {
                  simplerestrictions++;
                }
              }
            }
          }

          // Increment the directed edge index within the tile
          idx++;
          n++;

          // Add to general statistics
//          (*stats).AddStats(tile_start->first, directededge);
        }

        // Set the node lat,lng, index of the first outbound edge, and the
        // directed edge count from this edge and the best road class
        // from the node. Increment directed edge count.
        NodeInfoBuilder nodebuilder(node_ll, directededgecount,
                                    node.edge_count(), bestclass);
        directededgecount += node.edge_count();

        // Add node and directed edge information to the tile
        graphtile.AddNodeAndDirectedEdges(nodebuilder, directededges);

        // Increment nodeid
        nodeid++;
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

std::string GraphBuilder::GetRef(const std::string& way_ref,
                                 const std::string& relation_ref) {

  std::string refs;

  std::vector<std::string> way_refs = GetTagTokens(way_ref); // US 51;I 57

  std::vector<std::string> refdirs = GetTagTokens(relation_ref);// US 51|north;I 57|north

  bool found = false;

  for (auto& ref : way_refs) {

    found = false;

    for (auto& refdir : refdirs) {

      std::vector<std::string> tmp = GetTagTokens(refdir,'|'); // US 51|north

      if (tmp.size() == 2)
      {
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

    if (!found) // no direction found in relations for this ref
    {
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

  // Exit sign number
  if (way.junction_ref_index() != 0) {
    exit_list.emplace_back(Sign::Type::kExitNumber,
            osmdata.ref_offset_map.name(way.junction_ref_index()));
  }  else if (node.ref()) {
    exit_list.emplace_back(Sign::Type::kExitNumber,
            node_ref.find(nodeid)->second);
  }

  // Exit sign branch refs
  bool has_branch = false;
  if (way.destination_ref_index() != 0) {
    has_branch = true;
    std::vector<std::string> branch_refs = GetTagTokens(
        osmdata.ref_offset_map.name(way.destination_ref_index()));
    for (auto& branch_ref : branch_refs) {
      exit_list.emplace_back(Sign::Type::kExitBranch, branch_ref);
    }
  }

  // Exit sign toward refs
  bool has_toward = false;
  if (way.destination_ref_to_index() != 0) {
    has_toward = true;
    std::vector<std::string> toward_refs = GetTagTokens(
        osmdata.ref_offset_map.name(way.destination_ref_to_index()));
    for (auto& toward_ref : toward_refs) {
      exit_list.emplace_back(Sign::Type::kExitToward, toward_ref);
    }
  }

  // Exit sign toward names
  if (way.destination_index() != 0) {
    has_toward = true;
    std::vector<std::string> toward_names = GetTagTokens(
        osmdata.name_offset_map.name(way.destination_index()));
    for (auto& toward_name : toward_names) {
      exit_list.emplace_back(Sign::Type::kExitToward, toward_name);
    }
  }

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
void GraphBuilder::BuildLocalTiles(const uint8_t level, const OSMData& osmdata) const {
  // A place to hold worker threads and their results, be they exceptions or otherwise
  std::vector<std::shared_ptr<std::thread> > threads(threads_);
  // A place to hold the results of those threads, be they exceptions or otherwise
  std::vector<std::promise<size_t> > results(threads.size());
  // Divvy up the work
  size_t floor = tilednodes_.size() / threads.size();
  size_t at_ceiling = tilednodes_.size() - (threads.size() * floor);
  std::unordered_map<GraphId, std::vector<Node> >::const_iterator tile_start, tile_end = tilednodes_.begin();

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
      new std::thread(BuildTileSet, tile_start, tile_end, tilednodes_,
                      edges_, latlngs_, tile_hierarchy_, osmdata,
                      node_ref_, node_exit_to_, node_name_,
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
}


}
}
