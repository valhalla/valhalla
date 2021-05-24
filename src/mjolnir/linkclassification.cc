#include "mjolnir/ferry_connections.h"
#include "mjolnir/node_expander.h"

#include <list>
#include <queue>
#include <unordered_set>
#include <vector>

#include "baldr/graphid.h"
#include "midgard/util.h"

#include "mjolnir/util.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

constexpr uint32_t kMaxClassification = 8;
constexpr uint32_t kMaxLinkEdges = 16;
constexpr uint32_t kServiceClass = static_cast<uint32_t>(RoadClass::kServiceOther);
using nodelist_t = std::vector<std::vector<sequence<Node>::iterator>>;

// Structure that keeps information about link graph node (NOTE: assuming that
// the graph is acyclic!)
struct LinkGraphNode {
  uint32_t node_index;     // Node index in the sequence
  uint32_t classification; // Classification at this node
  node_bundle bundle;      // Info about node edges
  bool has_exit;

  std::vector<uint32_t> parents;       // Indices of parent nodes in the graph (don't confuse
                                       // these with node indices in the sequence!)
  std::vector<uint32_t> parents_edges; // Indices of parent edges in the sequence

  std::vector<uint32_t> children;       // Indices of children nodes in the graph (don't confuse
                                        // these with node indices in the sequence!)
  std::vector<uint32_t> children_edges; // Indices of children edges in the sequence
  uint32_t children_reclassified = 0;   // Number of reclassified children nodes (it's used on
                                        // reclassification stage)

  LinkGraphNode(uint32_t node_index, uint32_t rc, const node_bundle& bundle)
      : node_index(node_index), classification(rc), bundle(bundle),
        has_exit(bundle.node.has_exit_to() || bundle.node.has_ref()) {
  }
};

// Test if the set of edges can be classified as a turn channel. Total length
// must be less than kMaxTurnChannelLength and there cannot be any exit signs.
bool IsTurnChannel(sequence<OSMWay>& ways,
                   sequence<Edge>& edges,
                   sequence<OSMWayNode>& way_nodes,
                   std::vector<uint32_t>& linkedgeindexes) {
  // Method to get the shape for an edge - since LL is stored as a pair of
  // floats we need to change into PointLL to get length of an edge
  const auto EdgeShape = [&way_nodes](size_t idx, const size_t count) {
    std::list<PointLL> shape;
    for (size_t i = 0; i < count; ++i) {
      auto node = (*way_nodes[idx++]).node;
      shape.emplace_back(node.latlng());
    }
    return shape;
  };

  // Iterate through the link edges. Check if total length exceeds the
  // maximum turn channel length or an exit sign exists.
  float total_length = 0.0f;
  for (auto idx : linkedgeindexes) {
    // Get the shape and length of the edge
    sequence<Edge>::iterator element = edges[idx];
    auto edge = *element;
    auto shape = EdgeShape(edge.llindex_, edge.attributes.llcount);
    total_length += valhalla::midgard::length(shape);
    if (total_length > kMaxTurnChannelLength) {
      return false;
    }

    // Can not be bidirectional
    OSMWay way = *ways[edge.wayindex_];
    if (way.auto_forward() && way.auto_backward()) {
      return false;
    }
  }
  return true;
}

inline bool IsDriveableNonLink(const Edge& edge) {
  return !edge.attributes.link &&
         (edge.attributes.driveableforward || edge.attributes.driveablereverse) &&
         edge.attributes.importance != kServiceClass;
}

inline bool IsDriveForwardLink(const Edge& edge) {
  return edge.attributes.link && edge.attributes.driveforward;
}

// Get the best classification for any driveable non-link edges from a node.
uint32_t GetBestNonLinkClass(const std::map<Edge, size_t>& edges) {
  uint32_t bestrc = kAbsurdRoadClass;
  for (const auto& edge : edges) {
    if (IsDriveableNonLink(edge.first)) {
      bestrc = std::min<uint32_t>(bestrc, edge.first.attributes.importance);
    }
  }
  return bestrc;
}

// Form a list of all nodes - sorted by highest classification of non-link
// edges at the node.
nodelist_t FormExitNodes(sequence<Node>& nodes, sequence<Edge>& edges) {
  nodelist_t exit_nodes(kMaxClassification);
  sequence<Node>::iterator node_itr = nodes.begin();
  while (node_itr != nodes.end()) {
    // If the node has a both links and non links at it
    auto bundle = collect_node_edges(node_itr, nodes, edges);
    if (bundle.node.link_edge_ && bundle.node.non_link_edge_) {
      // Check if this node has a link edge that is driveable from the node
      for (const auto& edge : bundle.node_edges) {
        if (edge.first.attributes.link && edge.first.attributes.driveforward) {
          // Get the highest classification of non-link edges at this node.
          // Add to the exit node list if a valid classification...if no
          // connecting edge is driveable the node will be skipped.
          uint32_t rc = GetBestNonLinkClass(bundle.node_edges);
          if (rc < kMaxClassification) {
            exit_nodes[rc].push_back(node_itr);
          }
        }
      }
    }

    // Go to the next node
    node_itr += bundle.node_count;
  }

  // Output exit counts for each class
  for (uint32_t rc = 0; rc < kMaxClassification; rc++) {
    LOG_INFO("Class: " + std::to_string(rc) +
             " exit count = " + std::to_string(exit_nodes[rc].size()));
  }
  return exit_nodes;
}

// Helper structure that contains all osm data we need in one place
struct Data {
  Data(const std::string& nodes_file,
       const std::string& edges_file,
       const std::string& ways_file,
       const std::string& way_nodes_file,
       const OSMData& osmdata)
      : nodes(nodes_file, false), edges(edges_file, false), ways(ways_file, false),
        way_nodes(way_nodes_file, false), osmdata(osmdata) {
  }

  sequence<Node> nodes;
  sequence<Edge> edges;
  sequence<OSMWay> ways;
  sequence<OSMWayNode> way_nodes;
  const OSMData& osmdata;
};

// Way tags that are used to determine correct link class
struct WayTags {
  std::vector<std::string> refs;
  std::vector<std::string> dest_refs;

  bool is_empty() const {
    return refs.empty() && dest_refs.empty();
  }

  static WayTags Parse(const OSMWay& way, const OSMData& osmdata) {
    WayTags road_tags;

    // Parse 'destination:ref' tag
    if (way.destination_ref_index() != 0)
      road_tags.dest_refs = GetTagTokens(osmdata.name_offset_map.name(way.destination_ref_index()));

    // Parse 'ref' tag
    if (way.ref_index() != 0)
      road_tags.refs = GetTagTokens(osmdata.name_offset_map.name(way.ref_index()));

    return road_tags;
  }
};

// Check if these two references belong to the same road
inline bool MatchRefs(const std::string& ref_1, const std::string& ref_2) {
  size_t sz = std::min(ref_1.size(), ref_2.size());
  return (sz != 0) && (ref_1.compare(0, sz, ref_2, 0, sz) == 0);
}
// Check if these two ways belong to the same road
inline bool IsTheSameRoad(const WayTags& road_1, const WayTags& road_2) {
  return !road_1.refs.empty() && !road_2.refs.empty() &&
         MatchRefs(road_1.refs.front(), road_2.refs.front());
}
// Check if these three ways belong to the same road
inline bool IsTheSameRoad(const WayTags& road_1, const WayTags& road_2, const WayTags& road_3) {
  return IsTheSameRoad(road_1, road_2) && IsTheSameRoad(road_2, road_3);
}

// Check if the link contains this destination reference
inline bool IsDestinationRef(const std::string& ref, const WayTags& link) {
  return !ref.empty() && (std::find_if(link.dest_refs.begin(), link.dest_refs.end(),
                                       [&ref](const std::string& dest_ref) {
                                         return MatchRefs(ref, dest_ref);
                                       }) != link.dest_refs.end());
}

// Using destination link tags detect if the road is a final destination
inline bool IsDestinationRoad(const WayTags& road, const WayTags& link) {
  return !road.refs.empty() && IsDestinationRef(road.refs.front(), link);
}

// Check if these two links has common destination road
bool HasCommonDestination(const WayTags& link_1, const WayTags& link_2) {
  for (const auto& dest_ref_1 : link_1.dest_refs)
    if (IsDestinationRef(dest_ref_1, link_2)) {
      return true;
    }
  return false;
}
// Check if these three links has common destination road
bool HasCommonDestination(const WayTags& link_1, const WayTags& link_2, const WayTags& link_3) {
  for (const auto& dest_ref_1 : link_1.dest_refs)
    if (IsDestinationRef(dest_ref_1, link_2) && IsDestinationRef(dest_ref_1, link_3)) {
      return true;
    }
  return false;
}
// Check if this node contains a road that is a destination for the link
bool IsDestinationNode(const node_bundle& node, const WayTags& link, Data& data) {
  if (link.is_empty())
    return false;

  for (const auto& edge : node.node_edges) {
    if (IsDriveableNonLink(edge.first)) {
      const auto road = WayTags::Parse(*data.ways[edge.first.wayindex_], data.osmdata);
      if (IsTheSameRoad(road, link) || IsDestinationRoad(road, link))
        return true;
    }
  }
  return false;
}
// Check if any of destination roads for the root link can be reached from this node.
// This function builds all possible link paths from this node and checks if any of them
// leads to some destination road.
bool CheckIfNodeLeadsToDestination(size_t node_idx,
                                   const WayTags& root_link_tags,
                                   const std::unordered_set<size_t>& already_visited_nodes,
                                   Data& data) {
  auto bundle = collect_node_edges(data.nodes[node_idx], data.nodes, data.edges);
  // Initialize set of visited nodes
  std::unordered_set<size_t> visited_nodes = already_visited_nodes;
  std::queue<size_t> expand_queue;

  expand_queue.push(node_idx);
  while (!expand_queue.empty()) {
    node_idx = expand_queue.front();
    expand_queue.pop();
    // Skip if the node has already been visited
    if (visited_nodes.find(node_idx) != visited_nodes.end())
      continue;
    visited_nodes.insert(node_idx);

    auto bundle = collect_node_edges(data.nodes[node_idx], data.nodes, data.edges);
    if (IsDestinationNode(bundle, root_link_tags, data))
      return true;

    for (const auto& edge : bundle.node_edges) {
      if (!IsDriveForwardLink(edge.first)) {
        continue;
      }

      auto end_node_idx =
          edge.first.sourcenode_ == node_idx ? edge.first.targetnode_ : edge.first.sourcenode_;
      expand_queue.push(end_node_idx);
    }
  }
  return false;
}
// Check if the node lies on the path from the root link to some destination road (and not
// the last node in this path)
bool CanGoThroughNode(const node_bundle& node,
                      size_t node_idx,
                      const WayTags& inbound_link,
                      const WayTags& root_link,
                      const std::unordered_set<size_t>& visited_nodes,
                      Data& data) {
  if (!HasCommonDestination(root_link, inbound_link) && !IsTheSameRoad(root_link, inbound_link))
    return false;

  bool has_common_dest = false;
  for (const auto& edge : node.node_edges) {
    if (IsDriveForwardLink(edge.first)) {
      const auto link = WayTags::Parse(*data.ways[edge.first.wayindex_], data.osmdata);
      // We can go through this node if the root link, inbound link and outbound link
      // belong to the same road
      if (IsTheSameRoad(link, inbound_link, root_link))
        return true;

      if (!has_common_dest && HasCommonDestination(link, inbound_link, root_link))
        has_common_dest = true;
    }
  }
  return has_common_dest && CheckIfNodeLeadsToDestination(node_idx, root_link, visited_nodes, data);
}

/*
 * This class builds acyclic link graph starting from some exit node. This node is the root node.
 * Then we start recursively traversing the graph using only driveforward links. When we reach a
 * node that doesn't contain nonlink edges we add this node to the graph and continue to exapnd
 * (except when we've already visited the node). In case a node with nonlink edges is reached we
 * check if the node has some destination road or not and based on this we determine should we stop
 * or continue to expand. In order to avoid cycles there are two sets are used: 'processed' - contains
 * nodes where all their children have been processed; 'in_progress' - contains nodes that have been
 * visited but some of their children haven't been processed yet (in other words 'in progress' set
 * contains all nodes on the path from the root node to the current node). So, when we reach a node
 * from the 'processed' set we should add an edge to the graph (that goes from the current node to the
 * node from 'processed' set); but when we reach a node from the 'in progress' set - it's a cycle
 * (right now we just skip it).
 */
struct LinkGraphBuilder {
  Data& data_;
  // Way tags of the root link
  WayTags root_link_;
  // List of link graph nodes
  std::vector<LinkGraphNode> graph_;
  // Processed node indices (from the sequence). Map node indices to the graph indices
  std::unordered_map<size_t, uint32_t> processed_;
  // Node indices (from the sequence) that is processing now
  std::unordered_set<size_t> in_progress_;

  explicit LinkGraphBuilder(Data& data) : data_(data) {
  }

  std::vector<LinkGraphNode> operator()(sequence<Node>::iterator& exit_node,
                                        uint32_t classification) {
    auto exit_bundle = collect_node_edges(exit_node, data_.nodes, data_.edges);
    graph_.emplace_back(exit_node.position(), classification, exit_bundle);
    in_progress_.insert(exit_node.position());

    // Expand link edges from the exit node
    for (const auto& startedge : exit_bundle.node_edges) {
      // Get the edge information. Skip non-link edges, link edges that are
      // not driveable in the forward direction, and link edges already
      // tested for reclassification
      if (!IsDriveForwardLink(startedge.first) || startedge.first.attributes.reclass_link) {
        continue;
      }

      // 'destination:ref' and 'ref' tags are widely distributed only among motorway and trunk
      // links (see https://taginfo.openstreetmap.org/tags/highway=motorway_link#combinations).
      // TODO: extend reference-based classification to other link classes.
      if (startedge.first.attributes.importance <= static_cast<uint32_t>(RoadClass::kTrunk))
        root_link_ = WayTags::Parse(*data_.ways[startedge.first.wayindex_], data_.osmdata);

      ExpandLink(startedge.first, startedge.second, 0);
    }

    return graph_;
  }

  void ExpandLink(const Edge& in_edge, uint32_t in_edge_idx, uint32_t parent) {
    // Find end node of this link edge
    auto end_node = in_edge.sourcenode_ == graph_[parent].node_index
                        ? data_.nodes[in_edge.targetnode_]
                        : data_.nodes[in_edge.sourcenode_];

    // TODO: process cycles.
    if (in_progress_.find(end_node.position()) != in_progress_.end()) {
      return;
    }
    // Check if this node has already been processed
    auto processed_it = processed_.find(end_node.position());
    if (processed_it != processed_.end()) {
      // Add new edge to the graph
      AddGraphEdge(parent, processed_it->second, in_edge_idx);
      return;
    }

    // Get the edges at the end node and get best non-link classification
    auto bundle = collect_node_edges(end_node, data_.nodes, data_.edges);
    uint32_t rc = GetBestNonLinkClass(bundle.node_edges);
    uint32_t graph_idx = graph_.size();
    graph_.emplace_back(end_node.position(), rc, bundle);
    // Add new edge to the graph
    AddGraphEdge(parent, graph_idx, in_edge_idx);

    // Check "stop criterions" only if this link intersects a "major" road
    if (bundle.node.non_link_edge_ && rc <= static_cast<uint32_t>(RoadClass::kResidential)) {
      const auto edge_tags = WayTags::Parse(*data_.ways[in_edge.wayindex_], data_.osmdata);
      // We should stop if this node contains a destination road or if it doesn't belong
      // to any path from the root to a destination road
      if (edge_tags.is_empty() || IsDestinationNode(bundle, root_link_, data_) ||
          !CanGoThroughNode(bundle, end_node.position(), edge_tags, root_link_, in_progress_, data_))
        return;
    }

    ExpandGraphNode(graph_idx);
  }

  void ExpandGraphNode(uint32_t graph_idx) {
    const auto node_index = graph_[graph_idx].node_index;
    const auto bundle = graph_[graph_idx].bundle.node_edges;
    // Update 'processed' and 'in progress' sets in order to be able to detect cycles
    processed_.erase(node_index);
    in_progress_.insert(node_index);

    // Expand link edges from the node
    for (const auto& edge : bundle) {
      // Use only links drivable in forward direction
      if (!IsDriveForwardLink(edge.first)) {
        continue;
      }
      // If the edge has already been considered for reclassification,
      // update node classification
      if (edge.first.attributes.reclass_link) {
        graph_[graph_idx].classification =
            std::min<uint32_t>(graph_[graph_idx].classification, edge.first.attributes.importance);
      } else {
        ExpandLink(edge.first, edge.second, graph_idx);
      }
    }
    // This node has been processed. Move it from 'in progress' set to 'processed'
    in_progress_.erase(node_index);
    processed_.insert({node_index, graph_idx});
  }

  void AddGraphEdge(uint32_t from, uint32_t to, uint32_t edge_idx) {
    graph_[from].children.push_back(to);
    graph_[from].children_edges.push_back(edge_idx);

    // Make sure that number of children does not exceed the threshold
    if (graph_[from].children.size() >= kMaxLinkEdges) {
      throw std::runtime_error("Exceeding kMaxLinkEdges in ReclassifyLinks");
    }

    graph_[to].parents.push_back(from);
    graph_[to].parents_edges.push_back(edge_idx);
  }
};

/*
 * Reclassify links in the acyclic link graph. We maintain a queue of leaf nodes. On each step take
 * some leaf node from the queue, build a link chain, determine the final road class for the whole
 * chain (and set the new class), then virtually "remove" reclassified links from the graph and
 * update the queue if a new leaf is detected. The final link class is defined as maximum from the
 * root node classification and current leaf node classification. After reclassification is done we
 * update the parent node classification and continue.
 */
std::pair<uint32_t, uint32_t> ReclassifyLinkGraph(std::vector<LinkGraphNode>& link_graph,
                                                  uint32_t exit_classification,
                                                  Data& data,
                                                  bool infer_turn_channels) {
  // number of reclassified edges
  uint32_t reclass_count = 0;
  uint32_t tc_count = 0;
  // Collect start leaf nodes in the acyclic graph
  std::queue<size_t> leaves;
  for (size_t i = 0; i < link_graph.size(); ++i) {
    if (link_graph[i].children.empty())
      leaves.push(i);
  }

  // Iterate through leaf nodes and reclassify links
  while (!leaves.empty()) {
    const auto leaf_idx = leaves.front();
    leaves.pop();
    const auto& leaf = link_graph[leaf_idx];

    // Go through each parent
    for (auto parent_idx : leaf.parents) {
      std::vector<uint32_t> link_edges;
      auto current_idx = leaf_idx;

      // Track information required for turn channel tests.
      bool has_fork = false;
      bool has_exit = link_graph[current_idx].has_exit;
      bool ends_have_non_link = link_graph[current_idx].bundle.non_link_count > 0;

      // Make a chain of edges, stop if this is a root node
      while (!link_graph[current_idx].parents.empty()) {
        // Get the link edge index from the parent to the current node
        for (size_t i = 0; i < link_graph[current_idx].parents.size(); ++i) {
          if (link_graph[current_idx].parents[i] == parent_idx) {
            link_edges.push_back(link_graph[current_idx].parents_edges[i]);
            break;
          }
        }
        auto& parent = link_graph[parent_idx];
        // Increment count of reclassified children for the parent node
        parent.children_reclassified++;

        if (parent.bundle.link_count > 2) {
          has_fork = true;
        }
        if (parent.has_exit) {
          has_exit = true;
        }

        // Check if the parent has valid classification (contains non-link edges)
        // or has more than one child or parent (and not the root)
        if (parent.classification != kAbsurdRoadClass || parent.children.size() > 1 ||
            parent.parents.size() != 1) {
          // Update parent classification
          parent.classification = std::min(parent.classification, leaf.classification);
          // Add the parent to the leaves queue if all the children have already
          // been tested for reclassification
          if (parent.children_reclassified == parent.children.size()) {
            leaves.push(parent_idx);
          }
          current_idx = parent_idx;
          break;
        }

        // Set the current node to the parent - continue to move up the tree
        current_idx = parent_idx;
        // We would have exited the loop earlier if the number of parents wasn't equal to 1
        parent_idx = link_graph[current_idx].parents.front();
      }

      // Check the non-link count
      ends_have_non_link = ends_have_non_link && link_graph[current_idx].bundle.non_link_count > 0;

      // leaf classification may be invalid in case of cycle; use parent's
      // classification instead or just skip these links
      uint32_t leaf_classification = leaf.classification;
      if (leaf_classification == kAbsurdRoadClass) {
        if (link_graph[current_idx].classification != kAbsurdRoadClass)
          leaf_classification = link_graph[current_idx].classification;
        else {
          continue;
        }
      }

      const uint32_t rc = std::max(exit_classification, leaf_classification);
      if (rc == kAbsurdRoadClass) {
        LOG_ERROR("Trying to reclassify to invalid road class!");
        continue;
      }

      // Test if this link is a turn channel. Classification cannot be trunk or
      // motorway. No nodes can be marked as having an exit sign. None of the
      // nodes along the path can have more than 2 links (fork). The end nodes
      // must have a non-link edge.
      bool turn_channel = false;
      if (infer_turn_channels && (rc > static_cast<uint32_t>(RoadClass::kTrunk) && !has_fork &&
                                  !has_exit && ends_have_non_link)) {
        turn_channel = IsTurnChannel(data.ways, data.edges, data.way_nodes, link_edges);
      }

      // Reclassify link edges to the new classification.
      for (auto edge_idx : link_edges) {
        sequence<Edge>::iterator element = data.edges[edge_idx];
        auto edge = *element;

        if (rc > edge.attributes.importance) {
          if (rc < static_cast<uint32_t>(RoadClass::kUnclassified))
            edge.attributes.importance = rc;
          else
            edge.attributes.importance = static_cast<uint32_t>(RoadClass::kTertiary);

          ++reclass_count;
        }
        if (turn_channel) {
          edge.attributes.turn_channel = true;
          ++tc_count;
        }

        // Mark the edge so we don't try to reclassify it again. Copy
        // the updated edge back to the sequence.
        edge.attributes.reclass_link = true;
        element = edge;
      }
    } // for each leaf parent
  }   // for each leaf

  return {reclass_count, tc_count};
}

// Reclassify links (ramps and turn channels). OSM usually classifies links as
// the best classification, while to more effectively create shortcuts it is
// better to "downgrade" link edges to the lower classification. This method
// finds "exit" nodes sorted by classification. It then forms a "link graph"
// from the exit node and uses the classifications at the nodes of the link
// graph to potentially reclassify link edges. This also contains logic to
// identify turn channels / turn lanes (likely to be at-grade "slip roads").
void ReclassifyLinks(const std::string& ways_file,
                     const std::string& nodes_file,
                     const std::string& edges_file,
                     const std::string& way_nodes_file,
                     const OSMData& osmdata,
                     bool infer_turn_channels) {
  LOG_INFO("Reclassifying_V2 link graph edges...");

  Data data(nodes_file, edges_file, ways_file, way_nodes_file, osmdata);
  // Find list of exit nodes - nodes where driveable outbound links connect to
  // non-link edges. Group by best road class of the non-link connecting edges.
  nodelist_t exit_nodes = FormExitNodes(data.nodes, data.edges);

  // Iterate through the exit node list by classification so exits from major
  // roads are considered before exits from minor roads.
  uint32_t reclass_count = 0;
  uint32_t tc_count = 0;

  for (uint32_t classification = 0; classification < kMaxClassification; classification++) {
    for (auto& node : exit_nodes[classification]) {
      LinkGraphBuilder build_graph(data);
      // build link graph
      auto link_graph = build_graph(node, classification);
      // reclassify links and infer turn channels
      auto counts = ReclassifyLinkGraph(link_graph, classification, data, infer_turn_channels);
      // update counters
      reclass_count += counts.first;
      tc_count += counts.second;
    }
  }

  LOG_INFO("Finished with " + std::to_string(reclass_count) + " reclassified. " +
           " Turn channel count = " + std::to_string(tc_count));
}

} // namespace mjolnir
} // namespace valhalla
