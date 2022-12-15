#include <queue>
#include <unordered_map>

#include "baldr/graphconstants.h"
#include "midgard/util.h"
#include "mjolnir/ferry_connections.h"

namespace valhalla {
namespace mjolnir {

// Get the best classification for any driveable non-ferry and non-link
// edges from a node. Skip any reclassified ferry edges
uint32_t GetBestNonFerryClass(const std::map<Edge, size_t>& edges) {
  uint32_t bestrc = kAbsurdRoadClass;
  for (const auto& edge : edges) {
    uint16_t fwd_access = edge.first.fwd_access & baldr::kVehicularAccess;
    uint16_t rev_access = edge.first.rev_access & baldr::kVehicularAccess;
    if (!edge.first.attributes.driveable_ferry && !edge.first.attributes.link &&
        !edge.first.attributes.reclass_ferry && (fwd_access || rev_access)) {
      if (edge.first.attributes.importance < bestrc) {
        bestrc = edge.first.attributes.importance;
      }
    }
  }
  return bestrc;
}

// Cost comparator for priority_queue
class CompareCost {
public:
  bool operator()(const std::pair<float, uint32_t>& n1, const std::pair<float, uint32_t>& n2) {
    return n1.first > n2.first;
  }
};

// Form the shortest path from the start node until a node that
// touches the specified road classification.
uint32_t ShortestPath(const uint32_t start_node_idx,
                      const uint32_t node_idx,
                      sequence<OSMWay>& ways,
                      sequence<OSMWayNode>& way_nodes,
                      sequence<Edge>& edges,
                      sequence<Node>& nodes,
                      const bool inbound,
                      const uint32_t rc) {
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

  // total edge count for all reclassified paths
  // and determine for how many modes we need to ensure access (hint: only the ones using hierarchies)
  uint32_t edge_count = 0;

  uint16_t overall_access_before = baldr::kVehicularAccess;
  uint16_t overall_access_after = 0;
  // find accessible paths for as many modes as we can but stop if we can't find any more
  while (overall_access_before != overall_access_after) {
    overall_access_before = overall_access_after;

    // which modes do we still have to find a path for?
    uint32_t access_filter = baldr::kVehicularAccess ^ overall_access_before;

    // Map to store the status and index of nodes that have been encountered.
    // Any unreached nodes are not added to the map.
    std::unordered_map<uint32_t, NodeStatusInfo> node_status;
    std::vector<NodeLabel> node_labels;

    // Priority queue for the adjacency list
    std::priority_queue<std::pair<float, uint32_t>, std::vector<std::pair<float, uint32_t>>,
                        CompareCost>
        adjset;

    // Add node to list of node labels, set the node status and add
    // to the adjacency set
    uint32_t nodelabel_index = 0;
    node_labels.emplace_back(0.0f, node_idx, node_idx);
    node_status[node_idx] = {kTemporary, nodelabel_index};
    adjset.push({0.0f, nodelabel_index});
    nodelabel_index++;

    // Expand edges until a node connected to specified road classification
    // is reached
    uint32_t n = 0;
    uint32_t label_idx = 0;
    while (!adjset.empty()) {
      // Get the next node from the adjacency list/priority queue. Gets its
      // current cost and index
      const auto& expand_node = adjset.top();
      float current_cost = expand_node.first;
      label_idx = expand_node.second;
      uint32_t expand_node_idx = node_labels[label_idx].node_index;
      adjset.pop();

      // Skip if already labeled - this can happen if an edge is already in
      // adj. list and a lower cost is found
      if (node_status[expand_node_idx].set == kPermanent) {
        continue;
      }

      // Expand all edges from this node
      auto expand_node_itr = nodes[expand_node_idx];
      auto expanded_bundle = collect_node_edges(expand_node_itr, nodes, edges);

      // We are finished if node has RC <= rc and beyond first several edges.
      // Have seen cases where the immediate connections are high class roads
      // but then there are service roads (lanes) immediately after (like
      // Twawwassen Terminal near Vancouver,BC)
      if (n > 400 && GetBestNonFerryClass(expanded_bundle.node_edges) <= rc) {
        break;
      }
      n++;

      // Label the node as done/permanent
      node_status[expand_node_idx] = {kPermanent, label_idx};

      // Expand edges. Skip ferry edges and non-driveable edges (based on
      // the inbound flag).
      for (const auto& expandededge : expanded_bundle.node_edges) {
        // Skip any ferry edge and any edge that includes the start node index
        const auto& edge = expandededge.first;
        if (edge.attributes.driveable_ferry || edge.sourcenode_ == start_node_idx ||
            edge.targetnode_ == start_node_idx) {
          continue;
        }

        // Skip uses other than road / other (service?)
        const OSMWay w = *ways[edge.wayindex_];
        if (w.use() != baldr::Use::kOther && w.use() != baldr::Use::kServiceRoad &&
            static_cast<int>(w.use()) > static_cast<int>(baldr::Use::kTurnChannel)) {
          continue;
        }

        uint16_t edge_fwd_access = edge.fwd_access & baldr::kVehicularAccess;
        uint16_t edge_rev_access = edge.rev_access & baldr::kVehicularAccess;

        // Skip non-driveable edges and ones which are not accessible for the current access_filter
        // (based on inbound flag)
        bool forward = (edge.sourcenode_ == expand_node_idx);
        if (forward) {
          if ((inbound && !edge_rev_access) || (inbound && !(access_filter & edge.rev_access)) ||
              (!inbound && !edge_fwd_access) || (!inbound && !(access_filter & edge.fwd_access))) {
            continue;
          }
        } else {
          if ((inbound && !edge_fwd_access) || (inbound && !(access_filter & edge.fwd_access)) ||
              (!inbound && !edge_rev_access) || (!inbound && !(access_filter & edge.rev_access))) {
            continue;
          }
        }

        // Get the end node. Skip if already permanently labeled or this
        // edge is a loop
        uint32_t endnode = forward ? edge.targetnode_ : edge.sourcenode_;
        if (node_status[endnode].set == kPermanent || endnode == expand_node_idx) {
          continue;
        }

        // Get cost - need the length and speed of the edge; Use a penalty if an edge is
        // destination_only and calculate it in the cost
        float penalty = w.destination_only() ? 300 : 0;
        auto shape = EdgeShape(edge.llindex_, edge.attributes.llcount);
        float cost = current_cost + ((valhalla::midgard::length(shape) * 3.6f) / w.speed()) + penalty;

        // Check if already in adj set - skip if cost is higher than prior path
        if (node_status[endnode].set == kTemporary) {
          uint32_t endnode_idx = node_status[endnode].index;
          if (node_labels[endnode_idx].cost < cost) {
            continue;
          }
        }

        // Add to the node labels and adjacency set. Skip if this is a loop.
        node_labels.emplace_back(cost, endnode, expand_node_idx);
        node_status[endnode] = {kTemporary, nodelabel_index};
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
    // did we find all modes in the path?
    uint16_t path_access = baldr::kVehicularAccess;
    while (true) {
      // Get the edge between this node and the predecessor
      uint32_t idx = node_labels[label_idx].node_index;
      uint32_t pred_node = node_labels[label_idx].pred_node_index;
      auto expand_node_itr = nodes[idx];
      auto bundle2 = collect_node_edges(expand_node_itr, nodes, edges);
      for (auto& edge : bundle2.node_edges) {
        bool forward = edge.first.sourcenode_ == pred_node;
        if (forward || edge.first.targetnode_ == pred_node) {
          sequence<Edge>::iterator element = edges[edge.second];
          auto update_edge = *element;
          if ((forward && inbound) || (!forward && !inbound)) {
            path_access &= update_edge.rev_access;
          } else if ((forward && !inbound) || (!forward && inbound)) {
            path_access &= update_edge.fwd_access;
          }
          if (update_edge.attributes.importance > rc) {
            update_edge.attributes.importance = rc;
            update_edge.attributes.reclass_ferry = true;
            element = update_edge;
            edge_count++;
          }
        }
      }

      // Get the predecessor - break once we have found the start node
      if (pred_node == node_idx) {
        break;
      }
      label_idx = node_status[pred_node].index;
    }
    // update the overall mode access with the previous path
    overall_access_after |= path_access;
  }

  return edge_count;
}

// Check if the ferry included in this node bundle is short. Must be
// just one edge and length < 2 km
bool ShortFerry(const uint32_t node_index,
                node_bundle& bundle,
                sequence<Edge>& edges,
                sequence<Node>& nodes,
                sequence<OSMWayNode>& way_nodes) {
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
  bool short_edge = false;
  for (const auto& edge : bundle.node_edges) {
    // Check ferry edge.
    if (edge.first.attributes.driveable_ferry) {
      uint32_t endnode =
          (edge.first.sourcenode_ == node_index) ? edge.first.targetnode_ : edge.first.sourcenode_;
      auto end_node_itr = nodes[endnode];
      auto bundle2 = collect_node_edges(end_node_itr, nodes, edges);

      // If we notice that either the end node or source node is a connection to another ferry edge,
      // be cautious and assume this isn't a short edge.
      for (const auto& edge2 : bundle.node_edges) {
        if (edge2.first.llindex_ != edge.first.llindex_ && edge2.first.attributes.driveable_ferry) {
          return false;
        }
      }
      for (const auto& edge2 : bundle2.node_edges) {
        if (edge2.first.llindex_ != edge.first.llindex_ && edge2.first.attributes.driveable_ferry) {
          return false;
        }
      }

      // If the end node has a non-ferry edge check the length of the edge
      if (bundle2.node.non_ferry_edge_) {
        auto shape = EdgeShape(edge.first.llindex_, edge.first.attributes.llcount);
        if (midgard::length(shape) < 2000.0f) {
          short_edge = true;
        }
      } else {
        short_edge = false;
      }
    }
  }
  return short_edge;
}

// Reclassify edges from a ferry along the shortest path to the
// specified road classification.
void ReclassifyFerryConnections(const std::string& ways_file,
                                const std::string& way_nodes_file,
                                const std::string& nodes_file,
                                const std::string& edges_file,
                                const uint32_t rc) {
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
    if (bundle.node.ferry_edge_ && bundle.node.non_ferry_edge_ &&
        GetBestNonFerryClass(bundle.node_edges) > rc &&
        !ShortFerry(node_itr.position(), bundle, edges, nodes, way_nodes)) {
      // Form shortest path from node along each edge connected to the ferry,
      // track until the specified RC is reached
      for (const auto& edge : bundle.node_edges) {
        uint16_t edge_fwd_access = edge.first.fwd_access & baldr::kVehicularAccess;
        uint16_t edge_rev_access = edge.first.rev_access & baldr::kVehicularAccess;

        // Skip ferry edges and non-driveable edges
        if (edge.first.attributes.driveable_ferry || (!edge_fwd_access && !edge_rev_access)) {
          continue;
        }

        // Expand/reclassify from the end node of this edge.
        uint32_t end_node_idx = (edge.first.sourcenode_ == node_itr.position())
                                    ? edge.first.targetnode_
                                    : edge.first.sourcenode_;

        // Check if edge is oneway towards the ferry or outbound from the
        // ferry. If edge is drivable both ways we need to expand it twice-
        // once with a driveable path towards the ferry and once with a
        // driveable path away from the ferry
        if (edge_fwd_access == edge_rev_access) {
          // Driveable in both directions - get an inbound path and an
          // outbound path.
          total_count += ShortestPath(node_itr.position(), end_node_idx, ways, way_nodes, edges,
                                      nodes, true, rc);
          total_count += ShortestPath(node_itr.position(), end_node_idx, ways, way_nodes, edges,
                                      nodes, false, rc);
        } else {
          // Check if oneway inbound to the ferry
          bool inbound =
              (edge.first.sourcenode_ == node_itr.position()) ? edge_rev_access : edge_fwd_access;
          total_count += ShortestPath(node_itr.position(), end_node_idx, ways, way_nodes, edges,
                                      nodes, inbound, rc);
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
           std::to_string(ferry_endpoint_count) + ", " + std::to_string(total_count) +
           " edges reclassified.");
}

} // namespace mjolnir
} // namespace valhalla
