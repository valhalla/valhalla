#include "mjolnir/ferry_connections.h"
#include <valhalla/mjolnir/node_expander.h>

#include <list>
#include <queue>
#include <unordered_set>
#include <vector>

#include "baldr/graphid.h"
#include "midgard/util.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

constexpr uint32_t kMaxClassification = 8;
constexpr uint32_t kMaxLinkEdges = 16;
constexpr uint32_t kServiceClass = static_cast<uint32_t>(RoadClass::kServiceOther);
using nodelist_t = std::vector<std::vector<sequence<Node>::iterator>>;

// Create a tree structure for storing link edges and nodes
struct LinkTreeNode {
  bool has_exit;
  uint32_t non_link_count;               // Number of non-link edges
  uint32_t link_count;                   // Total number of link edges
  uint32_t classification;               // Classification at this node
  uint32_t node_index;                   // Node index in the sequence
  std::vector<uint32_t> link_edge_index; // Edge indices of children (links)
  std::vector<LinkTreeNode> children;    // End nodes of link edges
  LinkTreeNode* parent;                  // Pointer back to the parent

  LinkTreeNode(const bool ex,
               const uint32_t nlc,
               const uint32_t lc,
               uint32_t rc,
               const uint32_t idx,
               LinkTreeNode* p)
      : has_exit(ex), non_link_count(nlc), link_count(lc), classification(rc), node_index(idx),
        parent(p) {
    // Reserve space so these do not get re-allocated. This is needed since
    // pointers to the parent node (within the parent's children vector) are
    // stored.
    children.reserve(kMaxLinkEdges);
    link_edge_index.reserve(kMaxLinkEdges);
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
      shape.emplace_back(node.lng_, node.lat_);
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

    // Check if an exit sign exists.
    OSMWay way = *ways[edge.wayindex_];
    if (way.junction_ref_index() != 0 || way.destination_ref_index() != 0 ||
        way.destination_street_index() != 0 || way.destination_ref_to_index() != 0 ||
        way.destination_street_to_index() != 0 || way.destination_index() != 0 ||
        way.destination_forward_index() != 0 || way.destination_backward_index() != 0) {
      return false;
    }
  }
  return true;
}

// Get the best classification for any driveable non-link edges from a node.
uint32_t GetBestNonLinkClass(const std::map<Edge, size_t>& edges) {
  uint32_t bestrc = kAbsurdRoadClass;
  for (const auto& edge : edges) {
    if (!edge.first.attributes.link &&
        (edge.first.attributes.driveableforward || edge.first.attributes.driveablereverse) &&
        edge.first.attributes.importance < bestrc &&
        edge.first.attributes.importance != kServiceClass) {
      bestrc = edge.first.attributes.importance;
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

// Reclassify links in the link tree. Work up from each leaf and potentially
// reclassify sets of connected links until a branch (more than 1 child link)
// or the root is encountered.
std::pair<uint32_t, uint32_t> Reclassify(LinkTreeNode& root,
                                         sequence<Edge>& edges,
                                         sequence<OSMWay>& ways,
                                         sequence<OSMWayNode>& way_nodes,
                                         std::queue<LinkTreeNode*>& leaves) {
  // Get the classification at the root node.
  std::pair<uint32_t, uint32_t> counts = std::make_pair(0, 0);
  uint32_t exit_classification = root.classification;

  // Work up from each leaf until the root is found or a branch occurs.
  // Add branches to expand to a list;
  uint32_t rc = 0;
  std::vector<uint32_t> link_edge_indexes;
  while (!leaves.empty()) {
    // Clear the list of link edge indices from the prior leaf
    link_edge_indexes.clear();

    // Get the next leaf node and its classification
    LinkTreeNode* leaf_node = leaves.front();
    LinkTreeNode* current_node = leaf_node;
    leaves.pop();

    // Track information required for turn channel tests.
    bool has_fork = false;
    bool has_exit = current_node->has_exit;
    bool ends_have_non_link = current_node->non_link_count > 0;

    // Go upward in the tree until a branch is found or the root is reached
    while (current_node->parent != nullptr) {
      LinkTreeNode* parent = current_node->parent;
      if (parent->link_count > 2) {
        has_fork = true;
      }
      if (parent->has_exit) {
        has_exit = true;
      }

      // Get the link edge index from the parent to the current node
      for (uint32_t i = 0; i < parent->children.size(); ++i) {
        if (&parent->children[i] == current_node) {
          link_edge_indexes.push_back(parent->link_edge_index[i]);
          break;
        }
      }

      // Check if the parent is a branch (and not the root)
      if (parent->children.size() > 1 && parent->parent != nullptr) {
        // Add this branch to the leaves queue. Update the classification
        // at the branch node.
        parent->classification = std::min(parent->classification, leaf_node->classification);
        leaves.push(parent);
        break;
      }

      // Set the current node to the parent - continue to move up the tree
      current_node = parent;
    }

    // Check the non-link count and lwhether there are > 2 links at this node
    if (current_node->link_count > 2) {
      has_fork = true;
    }
    ends_have_non_link = ends_have_non_link && current_node->non_link_count > 0;

    // Get the classification - max value of the exit classification and leaf
    // classification. Make sure the classification is valid.
    uint32_t rc = std::max(exit_classification, leaf_node->classification);
    if (rc == kAbsurdRoadClass) {
      LOG_ERROR("Trying to reclassify to invalid road class!");
      continue;
    }

    // Test if this link is a turn channel. Classification cannot be trunk or
    // motorway. No nodes can be marked as having an exit sign. None of the
    // nodes along the path can have more than 2 links (fork). The end nodes
    // must have a non-link edge.
    bool turn_channel = false;
    if (rc > static_cast<uint32_t>(RoadClass::kTrunk) && !has_fork && !has_exit &&
        ends_have_non_link) {
      turn_channel = IsTurnChannel(ways, edges, way_nodes, link_edge_indexes);
    }

    // Reclassify link edges to the new classification.
    for (auto idx : link_edge_indexes) {
      sequence<Edge>::iterator element = edges[idx];
      auto edge = *element;
      if (rc > edge.attributes.importance) {
        if (rc < static_cast<uint32_t>(RoadClass::kUnclassified))
          edge.attributes.importance = rc;
        else
          edge.attributes.importance = static_cast<uint32_t>(RoadClass::kTertiary);

        counts.first++;
      }
      if (turn_channel) {
        edge.attributes.turn_channel = true;
        counts.second++;
      }

      // Mark the edge so we don't try to reclassify it again. Copy
      // the updated edge back to the sequence.
      edge.attributes.reclass_link = true;
      element = edge;
    }
  }
  return counts;
}

// Reclassify links (ramps and turn channels). OSM usually classifies links as
// the best classification, while to more effectively create shortcuts it is
// better to "downgrade" link edges to the lower classification. This method
// finds "exit" nodes sorted by classification. It then forms a "link tree"
// from the exit node and uses the classifications at the nodes of the link
// tree to potentially reclassify link edges. This also contains logic to
// identify turn channels / turn lanes (likely to be at-grade "slip roads").
void ReclassifyLinks(const std::string& ways_file,
                     const std::string& nodes_file,
                     const std::string& edges_file,
                     const std::string& way_nodes_file) {
  LOG_INFO("Reclassifying link graph edges...");

  // Need to capture these in the expand lambda
  std::unordered_set<size_t> visitedset; // Set of visited nodes
  std::vector<LinkTreeNode*> expandset;  // Set of nodes to expand
  std::queue<LinkTreeNode*> leaves;      // Leaf nodes of the link tree
  sequence<Edge> edges(edges_file, false);
  sequence<Node> nodes(nodes_file, false);
  // Lambda to expand from the end node of an edge. Adds the link edge and
  // end node to the link tree. Possibly adds the end node to the expandset.
  // Where no expansion occurs a leaf node is identified.
  auto expand = [&expandset, &nodes, &edges, &visitedset,
                 &leaves](const Edge& edge, const uint32_t link_edge_index, LinkTreeNode* tree_node) {
    // Find end node of this link edge
    auto end_node =
        edge.sourcenode_ == tree_node->node_index ? nodes[edge.targetnode_] : nodes[edge.sourcenode_];

    // Get the edges at the end node and get best non-link classification
    auto bundle = collect_node_edges(end_node, nodes, edges);
    uint32_t rc = GetBestNonLinkClass(bundle.node_edges);

    // Add this link edge as a child of the root node and add the end node as
    // a child node in the tree. Make sure this does not exceed kMaxLinkEdges
    if (tree_node->link_edge_index.size() == kMaxLinkEdges) {
      throw std::runtime_error("Exceeding kMaxLinkEdges in ReclassifyLinks");
    }
    tree_node->link_edge_index.push_back(link_edge_index);
    tree_node->children.emplace_back((bundle.node.has_ref() || bundle.node.has_exit_to()),
                                     bundle.non_link_count, bundle.link_count, rc,
                                     end_node.position(), tree_node);

    // Return if this end node has already been visited (could be a loop)
    if (visitedset.find(end_node.position()) != visitedset.end()) {
      return;
    }

    // Do not continue if this link intersects a "major" road
    // TODO - might need to continue to expand...but will need to determine
    // if the link enters back onto the same road that was exited.
    if (bundle.node.non_link_edge_ && rc <= static_cast<uint32_t>(RoadClass::kResidential)) {
      leaves.push(&tree_node->children.back());
      return;
    }

    // Continue to expand if any link edges exist.
    if (bundle.link_count > 1) {
      // Set to kAbsurdRoadClass - only root and leaf nodes have valid
      // classifications
      tree_node->children.back().classification = kAbsurdRoadClass;
      expandset.push_back(&tree_node->children.back());
    } else {
      // Not expanding - add this node to the leaf queue. Make sure
      // classification is valid (set to the link edge classification if not).
      if (rc == kAbsurdRoadClass) {
        tree_node->children.back().classification = edge.attributes.importance;
      }
      leaves.push(&tree_node->children.back());
    }
  };

  // Find list of exit nodes - nodes where driveable outbound links connect to
  // non-link edges. Group by best road class of the non-link connecting edges.
  nodelist_t exit_nodes = FormExitNodes(nodes, edges);

  // Iterate through the exit node list by classification so exits from major
  // roads are considered before exits from minor roads.
  uint32_t count = 0;
  uint32_t tc_count = 0;
  sequence<OSMWay> ways(ways_file, false);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  for (uint32_t classification = 0; classification < 8; classification++) {
    for (auto& node : exit_nodes[classification]) {
      // Clear the visited and expand lists. Clear the leaf node queue.
      visitedset = {};
      expandset = {};
      std::queue<LinkTreeNode*> empty;
      std::swap(leaves, empty);

      // Collect edges from this node.
      auto bundle = collect_node_edges(node, nodes, edges);

      // Form the link tree from this exit node. Add the root exit node.
      LinkTreeNode exit_node((bundle.node.has_ref() || bundle.node.has_exit_to()),
                             bundle.non_link_count, bundle.link_count, classification,
                             node.position(), nullptr);
      visitedset.insert(node.position());

      // Expand link edges from the exit node
      for (const auto& startedge : bundle.node_edges) {
        // Get the edge information. Skip non-link edges, link edges that are
        // not driveable in the forward direction, and link edges already
        // tested for reclassification
        if (!startedge.first.attributes.link || !startedge.first.attributes.driveforward ||
            startedge.first.attributes.reclass_link) {
          continue;
        }

        // Expand from this link edge
        expand(startedge.first, startedge.second, &exit_node);
        while (!expandset.empty()) {
          // Get the next node in the link tree to expand from.
          LinkTreeNode* tree_node = expandset.back();
          expandset.pop_back();
          visitedset.insert(tree_node->node_index);

          // Collect edges at this node and expand along link edges
          auto expand_node_itr = nodes[tree_node->node_index];
          auto expanded = collect_node_edges(expand_node_itr, nodes, edges);
          for (const auto& expandededge : expanded.node_edges) {
            // Do not allow use of the start edge, any non-link edge, or any
            // edge not driveable in the forward direction from the node.
            if (expandededge.second == startedge.second || !expandededge.first.attributes.link ||
                !expandededge.first.attributes.driveforward) {
              continue;
            }

            // If the edge has already been considered for reclassification,
            // adds its classification (if lower class than current) and set
            // as a leaf of the link tree
            if (expandededge.first.attributes.reclass_link) {
              uint32_t rc1 = tree_node->classification;
              uint32_t rc2 = expandededge.first.attributes.importance;
              if (rc1 == kAbsurdRoadClass || (rc2 > rc1 && rc2 != kAbsurdRoadClass)) {
                tree_node->classification = rc2;
              }
              leaves.push(tree_node);
            } else {
              // Add this link edge as a child of the current tree node and
              // expand from the end of this link edge
              expand(expandededge.first, expandededge.second, tree_node);
            }
          }
        }
      }

      // Reclassify link edges within the link tree
      auto counts = Reclassify(exit_node, edges, ways, way_nodes, leaves);
      count += counts.first;
      tc_count += counts.second;
    }
  }
  LOG_INFO("Finished with " + std::to_string(count) + " reclassified. " +
           " Turn channel count = " + std::to_string(tc_count));
}

} // namespace mjolnir
} // namespace valhalla
