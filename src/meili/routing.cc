#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/nodetransition.h"
#include "baldr/pathlocation.h"
#include "midgard/distanceapproximator.h"
#include "sif/costconstants.h"
#include "sif/dynamiccost.h"

#include "meili/routing.h"

namespace valhalla {

namespace meili {

LabelSet::LabelSet(const float max_cost, const float bucket_size) {
  const auto edgecost = [this](const uint32_t label) { return labels_[label].sortcost(); };
  queue_.reset(new baldr::DoubleBucketQueue(0.0f, max_cost, bucket_size, edgecost));
}

void LabelSet::put(const baldr::GraphId& nodeid,
                   const baldr::GraphId& edgeid,
                   const float source,
                   const float target,
                   const sif::Cost& cost,
                   const float turn_cost,
                   const float sortcost,
                   const uint32_t predecessor,
                   const baldr::DirectedEdge* edge,
                   const sif::TravelMode mode) {
  if (!nodeid.Is_Valid()) {
    throw std::runtime_error("invalid nodeid");
  }

  // Find the node Id. If not found, create a new label and push
  // it to the queue
  const auto it = node_status_.find(nodeid);
  if (it == node_status_.end()) {
    const uint32_t idx = labels_.size();
    labels_.emplace_back(nodeid, kInvalidDestination, edgeid, source, target, cost, turn_cost,
                         sortcost, predecessor, edge, mode);
    queue_->add(idx);
    node_status_.emplace(nodeid, idx);
  } else {
    // Node has been found. Check if there is a lower sortcost than the
    // existing label - if so update priority queue and Label
    const auto& status = it->second;
    if (!status.permanent && sortcost < labels_[status.label_idx].sortcost()) {
      // Update queue first since it uses the label cost within the decrease
      // method to determine the current bucket.
      queue_->decrease(status.label_idx, sortcost);
      labels_[status.label_idx] = {nodeid, kInvalidDestination, edgeid,   source,      target,
                                   cost,   turn_cost,           sortcost, predecessor, edge,
                                   mode};
    }
  }
}

void LabelSet::put(const uint16_t dest,
                   const baldr::GraphId& edgeid,
                   const float source,
                   const float target,
                   const sif::Cost& cost,
                   const float turn_cost,
                   const float sortcost,
                   const uint32_t predecessor,
                   const baldr::DirectedEdge* edge,
                   const sif::TravelMode travelmode) {
  if (dest == kInvalidDestination) {
    throw std::runtime_error("invalid destination");
  }

  // Find the destination. If not count, create a new label and push it
  // to the queue
  baldr::GraphId inv;
  const auto it = dest_status_.find(dest);
  if (it == dest_status_.end()) {
    const uint32_t idx = labels_.size();
    labels_.emplace_back(inv, dest, edgeid, source, target, cost, turn_cost, sortcost, predecessor,
                         edge, travelmode);
    queue_->add(idx);
    dest_status_.emplace(dest, idx);
  } else {
    // Decrease cost of the existing label
    const auto& status = it->second;
    if (!status.permanent && sortcost < labels_[status.label_idx].sortcost()) {
      // Update queue first since it uses the label cost within the decrease
      // method to determine the current bucket.
      queue_->decrease(status.label_idx, sortcost);
      labels_[status.label_idx] = {inv,       dest,     edgeid,      source, target,    cost,
                                   turn_cost, sortcost, predecessor, edge,   travelmode};
    }
  }
}

// Get the next label from the priority queue. Marks the popped label
// as permanent (best path found).
uint32_t LabelSet::pop() {
  const auto idx = queue_->pop();

  // Mark the popped label as permanent (optimal)
  if (idx != baldr::kInvalidLabel) {
    const auto& label = labels_[idx];
    if (label.nodeid().Is_Valid()) {
      const auto it = node_status_.find(label.nodeid());

      // When these logic errors happen, go check LabelSet::put
      if (it == node_status_.end()) {
        // No exception, unless BucketQueue::put was wrong: it said it
        // added but actually failed
        throw std::logic_error("all nodes in the queue should have its status");
      }
      auto& status = it->second;
      if (status.label_idx != idx) {
        throw std::logic_error(
            "the index stored in the node status " + std::to_string(status.label_idx) +
            " is not synced up with the index popped from the queue idx = " + std::to_string(idx));
      }
      if (status.permanent) {
        // For example, if the queue has popped up an index 2, and
        // marked the label at this index as permanent (optimal), then
        // some time later the queue pops up another index 2
        // (duplicated), this logic error will be thrown
        throw std::logic_error("the principle of optimality is violated during routing,"
                               " probably negative costs occurred");
      }

      status.permanent = true;
    } else { // assert(label.dest != kInvalidDestination)
      const auto it = dest_status_.find(label.dest());

      if (it == dest_status_.end()) {
        throw std::logic_error("all dests in the queue should have its status");
      }
      auto& status = it->second;
      if (status.label_idx != idx) {
        throw std::logic_error(
            "the index stored in the dest status " + std::to_string(status.label_idx) +
            " is not synced up with the index popped from the queue idx = " + std::to_string(idx));
      }
      if (status.permanent) {
        throw std::logic_error("the principle of optimality is violated during routing,"
                               " probably negative costs occurred");
      }

      status.permanent = true;
    }
  }
  return idx;
}

/**
 * Test if an edge is allowed. Checks if the predecessor edge label is valid
 * (invalid for origin edges) and whether costing allows the edge. Also checks
 * if still on the same edge as the predecessor (allow if so).
 */
inline bool IsEdgeAllowed(const baldr::DirectedEdge* edge,
                          const baldr::GraphId& edgeid,
                          const sif::cost_ptr_t& costing,
                          const Label& pred_edgelabel,
                          const baldr::GraphTile* tile) {
  // TODO We may want to optionally have map matching take time restrictions into account here
  bool i_dont_care_about_time_restrictions_here = false;
  return (!pred_edgelabel.edgeid().Is_Valid() && costing->GetEdgeFilter()(edge) != 0.f) ||
         edgeid == pred_edgelabel.edgeid() ||
         costing->Allowed(edge, pred_edgelabel, tile, edgeid, 0, 0,
                          i_dont_care_about_time_restrictions_here);
}

/**
 * Set origin.
 */
void set_origin(baldr::GraphReader& reader,
                const std::vector<baldr::PathLocation>& destinations,
                uint16_t origin_idx,
                const labelset_ptr_t& labelset,
                const sif::TravelMode travelmode,
                const sif::cost_ptr_t& costing,
                const Label* edgelabel) {
  // Push dummy labels (invalid edgeid, zero cost, no predecessor) to
  // the queue for the initial expansion later. These dummy labels
  // will also serve as roots in search trees, and sentinels to
  // indicate it reaches the beginning of a route when constructing the
  // route
  const baldr::GraphTile* tile = nullptr;
  for (const auto& edge : destinations[origin_idx].edges) {
    if (!edge.id.Is_Valid()) {
      continue;
    }

    auto edge_nodes = reader.GetDirectedEdgeNodes(edge.id, tile);
    if (edge.begin_node()) {
      const auto nodeid = edge_nodes.first;
      if (nodeid.Is_Valid()) {
        // If both origin and destination are nodes, then always check
        // the origin node but won't check the destination node
        const auto nodeinfo = reader.nodeinfo(nodeid, tile);
        if (!nodeinfo || !costing->Allowed(nodeinfo)) {
          continue;
        }
        labelset->put(nodeid, travelmode, edgelabel);
      }
    } else if (edge.end_node()) {
      const auto nodeid = edge_nodes.second;
      if (nodeid.Is_Valid()) {
        // If both origin and destination are nodes, then always check
        // the origin node but won't check the destination node
        const auto nodeinfo = reader.nodeinfo(nodeid, tile);
        if (!nodeinfo || !costing->Allowed(nodeinfo)) {
          continue;
        }
        labelset->put(nodeid, travelmode, edgelabel);
      }
    } else {
      // Will decide whether to filter out this edge later
      labelset->put(origin_idx, travelmode, edgelabel);
    }
  }
}

/**
 * Set destinations.
 */
void set_destinations(baldr::GraphReader& reader,
                      const std::vector<baldr::PathLocation>& destinations,
                      std::unordered_map<baldr::GraphId, std::unordered_set<uint16_t>>& node_dests,
                      std::unordered_map<baldr::GraphId, std::unordered_set<uint16_t>>& edge_dests) {
  const baldr::GraphTile* tile = nullptr;
  for (uint16_t dest = 0; dest < destinations.size(); dest++) {
    for (const auto& edge : destinations[dest].edges) {
      if (!edge.id.Is_Valid()) {
        continue;
      }

      auto edge_nodes = reader.GetDirectedEdgeNodes(edge.id, tile);
      if (edge.begin_node()) {
        const auto nodeid = edge_nodes.first;
        if (!nodeid.Is_Valid()) {
          continue;
        }
        node_dests[nodeid].insert(dest);
      } else if (edge.end_node()) {
        const auto nodeid = edge_nodes.second;
        if (!nodeid.Is_Valid()) {
          continue;
        }
        node_dests[nodeid].insert(dest);

      } else {
        edge_dests[edge.id].insert(dest);
      }
    }
  }
}

/**
 * Get the inbound heading of the predecessor edge (using the edge Label).
 * @param  reader   Graphreader - need this in cases where the node info
 *                  does not have the heading (more than 8 edges at the node).
 * @param  label    Predecessor edge label.
 * @param  nodeinfo Nodeinfo at the end of the predecessor edge.
 * @return  Returns the inbound edge heading.
 */
inline uint16_t get_inbound_edgelabel_heading(baldr::GraphReader& reader,
                                              const Label& label,
                                              const baldr::NodeInfo* nodeinfo) {
  // Get the opposing local index of the predecessor edge. If this is less
  // than 8 then we can get the heading from the nodeinfo.
  const auto idx = label.opp_local_idx();
  if (idx < 8) {
    return nodeinfo->heading(idx);
  } else {
    // Have to get the heading from the edge shape...
    const baldr::GraphTile* tile = nullptr;
    const auto directededge = reader.directededge(label.edgeid(), tile);
    const auto edgeinfo = tile->edgeinfo(directededge->edgeinfo_offset());
    const auto& shape = edgeinfo.shape();
    if (shape.size() >= 2) {
      float heading = (directededge->forward()) ? shape.back().Heading(shape.rbegin()[1])
                                                : shape.front().Heading(shape[1]);
      return static_cast<uint16_t>(std::max(0.f, std::min(359.f, heading)));
    } else {
      return 0;
    }
  }
}

/**
 * Get the heading of the outbound edge.
 * @param  tile           Graph tile at the node.
 * @param  outbound_edge  Outbound directed edge.
 * @param  nodeinfo       Nodeinfo at the start of the outbound edge.
 * @return Returns the outbound edge heading.
 */
// Get the outbound heading of the edge.
inline uint16_t get_outbound_edge_heading(const baldr::GraphTile* tile,
                                          const baldr::DirectedEdge* outbound_edge,
                                          const baldr::NodeInfo* nodeinfo) {
  // Get the local index of the outbound edge. If this is less
  // than 8 then we can get the heading from the nodeinfo.
  const auto idx = outbound_edge->localedgeidx();
  if (idx < 8) {
    return nodeinfo->heading(idx);
  } else {
    const auto edgeinfo = tile->edgeinfo(outbound_edge->edgeinfo_offset());
    const auto& shape = edgeinfo.shape();
    if (shape.size() >= 2) {
      float heading = (outbound_edge->forward()) ? shape.front().Heading(shape[1])
                                                 : shape.back().Heading(shape.rbegin()[1]);
      return static_cast<uint16_t>(std::max(0.f, std::min(359.f, heading)));
    } else {
      return 0;
    }
  }
}

// find_shortest_path(s) from an origin to a set of destination.
//
// Uses an "expand" lambda method to expand all edges from a node. Any
// transition edges immediately move to the end node of the transition edge
// and expand any edges from that node. In this manner, no transition edges
// get added to the label set.
//
// Also uses a heuristic function (lambda) that estimates cost
// from current node (incorporated in the approximator) to a cluster of
// destinations within a circle formed by the search radius around the lnglat
// (the location of next measurement).

// To not overestimate the heuristic cost:

// 1. if current node is outside the circle, the heuristic cost must
// be the great circle distance to the measurement minus the search
// radius, since there might be a destination at the boundary of the
// circle

// 2. if current node is within the circle, the heuristic cost must be
// zero since a destination could be anywhere within the circle,
// including the same location with current node

// Therefore, the heuristic cost is max(0, distance_to_lnglat - search_radius)

/**
 * Find the shortest path(s) from an origin to set of destinations.
 */
std::unordered_map<uint16_t, uint32_t>
find_shortest_path(baldr::GraphReader& reader,
                   const std::vector<baldr::PathLocation>& destinations,
                   uint16_t origin_idx,
                   labelset_ptr_t labelset,
                   const midgard::DistanceApproximator& approximator,
                   const float search_radius,
                   sif::cost_ptr_t costing,
                   const Label* edgelabel,
                   const float turn_cost_table[181],
                   const float max_dist,
                   const float max_time) {
  Label label;
  const sif::TravelMode travelmode = costing->travel_mode();

  // Destinations along edges
  std::unordered_map<baldr::GraphId, std::unordered_set<uint16_t>> edge_dests;

  // Destinations at nodes
  std::unordered_map<baldr::GraphId, std::unordered_set<uint16_t>> node_dests;

  // Lambda for heuristic
  float search_rad2 = search_radius * search_radius;
  const auto heuristic = [&approximator, &search_radius,
                          &search_rad2](const midgard::PointLL& lnglat) {
    float d2 = approximator.DistanceSquared(lnglat);
    return (d2 < search_rad2) ? 0.0f : sqrtf(d2) - search_radius;
  };

  // Lambda method to expand along edges from this node. This method has to be
  // set-up to be called recursively (for transition edges) so we set up a
  // function reference.
  std::function<void(const baldr::GraphId&, const uint32_t, const bool)> expand;
  expand = [&](const baldr::GraphId& node, const uint32_t label_idx, const bool from_transition) {
    // Get the node's info. The tile will be guaranteed to be nodeid's tile
    // in this block. Return if node is not found or is not allowed by costing
    const baldr::GraphTile* tile = reader.GetGraphTile(node);
    if (tile == nullptr) {
      return;
    }
    const baldr::NodeInfo* nodeinfo = tile->node(node);
    if (!costing->Allowed(nodeinfo)) {
      return;
    }

    // Get the inbound edge heading (clamped to range [0,360])
    const auto inbound_hdg =
        label.edgeid().Is_Valid() ? get_inbound_edgelabel_heading(reader, label, nodeinfo) : 0;

    // Expand from end node in forward direction.
    baldr::GraphId edgeid = {node.tileid(), node.level(), nodeinfo->edge_index()};
    const baldr::DirectedEdge* directededge = tile->directededge(edgeid);
    for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++directededge, ++edgeid) {
      // Skip it if its a shortcut or transit connection
      if (directededge->is_shortcut() || directededge->use() == baldr::Use::kTransitConnection) {
        continue;
      }

      // Skip it if its not allowed
      if (!IsEdgeAllowed(directededge, edgeid, costing, label, tile)) {
        continue;
      }

      // Get outbound heading (clamped to range [0,360]) and add to turn cost
      // based on turn degree
      float turn_cost = label.turn_cost();
      if (label.edgeid().Is_Valid()) {
        const auto outbound_hdg = get_outbound_edge_heading(tile, directededge, nodeinfo);
        turn_cost += turn_cost_table[midgard::get_turn_degree180(inbound_hdg, outbound_hdg)];
      }

      // If destinations found along the edge, add segments to each
      // destination to the queue
      const auto it = edge_dests.find(edgeid);
      if (it != edge_dests.end()) {
        for (const auto dest : it->second) {
          for (const auto& edge : destinations[dest].edges) {
            if (edge.id == edgeid) {
              // Get cost - use EdgeCost to get time along the edge. Override
              // cost portion to be distance. Heuristic cost from a destination
              // to itself must be 0, so sortcost = cost
              sif::Cost cost(label.cost().cost + directededge->length() * edge.percent_along,
                             label.cost().secs +
                                 costing->EdgeCost(directededge, tile).secs * edge.percent_along);
              // We only add the labels if we are under the limits for distance and for time or time
              // limit is 0
              if (cost.cost < max_dist && (max_time < 0 || cost.secs < max_time)) {
                labelset->put(dest, edgeid, 0.f, edge.percent_along, cost, turn_cost, cost.cost,
                              label_idx, directededge, travelmode);
              }
            }
          }
        }
      }

      // Get the end node tile and nodeinfo (to compute heuristic)
      const baldr::GraphTile* endtile =
          directededge->leaves_tile() ? reader.GetGraphTile(directededge->endnode()) : tile;
      if (endtile != nullptr) {
        // Get cost - use EdgeCost to get time along the edge. Override
        // cost portion to be distance. Add heuristic to get sort cost.
        sif::Cost cost(label.cost().cost + directededge->length(),
                       label.cost().secs + costing->EdgeCost(directededge, tile).secs);
        // We only add the labels if we are under the limits for distance and for time or time limit
        // is 0
        if (cost.cost < max_dist && (max_time < 0 || cost.secs < max_time)) {
          float sortcost = cost.cost + heuristic(endtile->get_node_ll(directededge->endnode()));
          labelset->put(directededge->endnode(), edgeid, 0.0f, 1.0f, cost, turn_cost, sortcost,
                        label_idx, directededge, travelmode);
        }
      }
    }

    // Handle transitions - expand from the end node each transition
    if (!from_transition && nodeinfo->transition_count() > 0) {
      const baldr::NodeTransition* trans = tile->transition(nodeinfo->transition_index());
      for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
        expand(trans->endnode(), label_idx, true);
      }
    }
  };

  // Load destinations
  set_destinations(reader, destinations, node_dests, edge_dests);

  // Load origin to the queue of the labelset
  set_origin(reader, destinations, origin_idx, labelset, travelmode, costing, edgelabel);

  std::unordered_map<uint16_t, uint32_t> results;
  while (true) {
    uint32_t label_idx = labelset->pop();
    if (label_idx == baldr::kInvalidLabel) {
      // Exhausted labels without finding all destinations
      break;
    }

    // Copy the Label since it is possible for it to be invalidated when new
    // labels are added.
    label = labelset->label(label_idx);
    if (label.nodeid().Is_Valid()) {
      // If this node is a destination, path to destinations at this
      // node is found: remember them and remove this node from the
      // destination list
      const auto it = node_dests.find(label.nodeid());
      if (it != node_dests.end()) {
        for (const auto dest : it->second) {
          results[dest] = label_idx;
        }
        node_dests.erase(it);
      }

      // Congrats!
      if (node_dests.empty() && edge_dests.empty()) {
        break;
      }

      // Expand edges from this node
      expand(label.nodeid(), label_idx, false);
    } else {
      // Path to a destination along an edge is found: remember it and
      // remove the destination from the destination list
      const auto dest = label.dest();
      results[dest] = label_idx;
      for (const auto& edge : destinations[dest].edges) {
        const auto it = edge_dests.find(edge.id);
        if (it != edge_dests.end()) {
          it->second.erase(dest);
          if (it->second.empty()) {
            edge_dests.erase(it);
          }
        }
      }

      // Congrats!
      if (edge_dests.empty() && node_dests.empty()) {
        break;
      }

      // Expand origin: add segments from origin to destinations ahead
      // at the same edge to the queue
      if (dest == origin_idx) {
        for (const auto& origin_edge : destinations[origin_idx].edges) {
          // The tile will be guaranteed to be directededge's tile in this loop
          const baldr::GraphTile* tile = nullptr;
          const auto directededge = reader.directededge(origin_edge.id, tile);

          // Skip if edge is not allowed
          if (!directededge || !IsEdgeAllowed(directededge, origin_edge.id, costing, label, tile)) {
            continue;
          }

          // U-turn cost
          float turn_cost = label.turn_cost();
          if (label.edgeid().Is_Valid() && label.edgeid() != origin_edge.id &&
              label.opp_local_idx() == directededge->localedgeidx()) {
            turn_cost += turn_cost_table[0];
          }

          // All destinations on this origin edge
          for (const auto other_dest : edge_dests[origin_edge.id]) {
            // All edges of this destination
            for (const auto& other_edge : destinations[other_dest].edges) {
              if (origin_edge.id == other_edge.id &&
                  origin_edge.percent_along <= other_edge.percent_along) {
                // Get cost - use EdgeCost to get time along the edge. Override
                // cost portion to be distance. The heuristic cost from a
                // destination to itself must be 0
                float f = (other_edge.percent_along - origin_edge.percent_along);
                sif::Cost cost(label.cost().cost + directededge->length() * f,
                               label.cost().secs + costing->EdgeCost(directededge, tile).secs * f);
                // We only add the labels if we are under the limits for distance and for time or
                // time limit is 0
                if (cost.cost < max_dist && (max_time < 0 || cost.secs < max_time)) {
                  labelset->put(other_dest, origin_edge.id, origin_edge.percent_along,
                                other_edge.percent_along, cost, turn_cost, cost.cost, label_idx,
                                directededge, travelmode);
                }
              }
            }
          }

          // Get cost - use EdgeCost to get time along the edge. Override
          // cost portion to be distance. The heuristic cost from a
          // destination to itself must be 0
          float f = (1.0f - origin_edge.percent_along);
          sif::Cost cost(label.cost().cost + directededge->length() * f,
                         label.cost().secs + costing->EdgeCost(directededge, tile).secs * f);
          // We only add the labels if we are under the limits for distance and for time or time
          // limit is 0
          if (cost.cost < max_dist && (max_time < 0 || cost.secs < max_time)) {
            // Get the end node tile and node lat,lon to compute heuristic
            const baldr::GraphTile* endtile = reader.GetGraphTile(directededge->endnode());
            if (endtile == nullptr) {
              continue;
            }
            float sortcost = cost.cost + heuristic(endtile->get_node_ll(directededge->endnode()));
            labelset->put(directededge->endnode(), origin_edge.id, origin_edge.percent_along, 1.f,
                          cost, turn_cost, sortcost, label_idx, directededge, travelmode);
          }
        }
      }
    }
  }
  // TODO - do we need to clear since it is constructed prior to each call??
  labelset->clear_queue();
  labelset->clear_status();
  return results;
}

} // namespace meili

} // namespace valhalla
