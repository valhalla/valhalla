#include <vector>
#include <unordered_set>
#include <unordered_map>

#include "midgard/distanceapproximator.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "sif/dynamiccost.h"
#include "sif/costconstants.h"

#include "meili/routing.h"

namespace valhalla {

namespace meili {

LabelSet::LabelSet(const float max_cost, const float bucket_size) {
  const auto edgecost = [this](const uint32_t label) {
    return labels_[label].sortcost;
  };
  max_cost_ = max_cost;
  queue_.reset(new baldr::DoubleBucketQueue(0.0f, max_cost_, bucket_size, edgecost));
}

bool LabelSet::put(const baldr::GraphId& nodeid, const baldr::GraphId& edgeid,
              float source, float target,
              const sif::Cost& cost, float turn_cost, float sortcost,
              uint32_t predecessor, const baldr::DirectedEdge* edge,
              sif::TravelMode travelmode,
              std::shared_ptr<const sif::EdgeLabel> edgelabel)
{
  if (!nodeid.Is_Valid()) {
    throw std::runtime_error("invalid nodeid");
  }
  const auto it = node_status_.find(nodeid);

  // Create a new label and push it to the queue
  if (it == node_status_.end()) {
    const uint32_t idx = labels_.size();
    if (sortcost < max_cost_) {
      queue_->add(idx, sortcost);
      labels_.emplace_back(nodeid, edgeid,
                         source, target,
                         cost, turn_cost, sortcost,
                         predecessor,
                         edge, travelmode, edgelabel);
      node_status_.emplace(nodeid, idx);
      return true;
    }
  } else {
    // Decrease cost of the existing label
    const auto& status = it->second;
    if (!status.permanent && sortcost < labels_[status.label_idx].sortcost) {
      // Update queue first since it uses the label cost within the decrease
      // method to determine the current bucket.
      queue_->decrease(status.label_idx, sortcost);
      labels_[status.label_idx] = {nodeid, edgeid,
                                   source, target,
                                   cost, turn_cost, sortcost,
                                   predecessor,
                                   edge, travelmode, edgelabel};
      return true;
    }
  }
  return false;
}

bool LabelSet::put(uint16_t dest,
              const baldr::GraphId& edgeid,
              float source, float target,
              const sif::Cost& cost, float turn_cost, float sortcost,
              uint32_t predecessor,
              const baldr::DirectedEdge* edge,
              sif::TravelMode travelmode,
              std::shared_ptr<const sif::EdgeLabel> edgelabel)
{
  if (dest == kInvalidDestination) {
    throw std::runtime_error("invalid destination");
  }

  const auto it = dest_status_.find(dest);

  // Create a new label and push it to the queue
  if (it == dest_status_.end()) {
    const uint32_t idx = labels_.size();
    if (sortcost < max_cost_) {
      queue_->add(idx, sortcost);
      labels_.emplace_back(dest, edgeid,
                         source, target,
                         cost, turn_cost, sortcost,
                         predecessor,
                         edge, travelmode, edgelabel);
      dest_status_.emplace(dest, idx);
      return true;
    }
  } else {
    // Decrease cost of the existing label
    const auto& status = it->second;
    if (!status.permanent && sortcost < labels_[status.label_idx].sortcost) {
      // Update queue first since it uses the label cost within the decrease
      // method to determine the current bucket.
      queue_->decrease(status.label_idx, sortcost);
      labels_[status.label_idx] = {dest, edgeid,
                                   source, target,
                                   cost, turn_cost, sortcost,
                                   predecessor,
                                   edge, travelmode, edgelabel};
      return true;
    }
  }
  return false;
}

uint32_t
LabelSet::pop()
{
  const auto idx = queue_->pop();

  // Mark the popped label as permanent (optimal)
  if (idx != baldr::kInvalidLabel) {
    const auto& label = labels_[idx];
    if (label.nodeid.Is_Valid()) {
      const auto it = node_status_.find(label.nodeid);

      // When these logic errors happen, go check LabelSet::put
      if (it == node_status_.end()) {
        // No exception, unless BucketQueue::put was wrong: it said it
        // added but actually failed
        throw std::logic_error("all nodes in the queue should have its status");
      }
      auto& status = it->second;
      if (status.label_idx != idx) {
        throw std::logic_error("the index stored in the status " + std::to_string(status.label_idx) +
                               " is not synced up with the index poped from the queue" + std::to_string(idx));
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
    } else {  // assert(label.dest != kInvalidDestination)
      const auto it = dest_status_.find(label.dest);

      if (it == dest_status_.end()) {
        throw std::logic_error("all dests in the queue should have its status");
      }
      auto& status = it->second;
      if (status.label_idx != idx) {
        throw std::logic_error("the index stored in the status " + std::to_string(status.label_idx) +
                               " is not synced up with the index poped from the queue" + std::to_string(idx));
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


inline bool IsEdgeAllowed(const baldr::DirectedEdge* edge,
              const baldr::GraphId& edgeid,
              const sif::cost_ptr_t costing,
              const std::shared_ptr<const sif::EdgeLabel>& pred_edgelabel,
              const baldr::GraphTile* tile)
{
  if (pred_edgelabel) {
    // TODO let sif do this?
    // Still on the same edge and the predecessor's show-up here
    // means it was allowed so we give it a pass directly
    return edgeid == pred_edgelabel->edgeid() ||
        costing->Allowed(edge, *pred_edgelabel, tile, edgeid);
  }
  return true;
}


void set_origin(baldr::GraphReader& reader,
                const std::vector<baldr::PathLocation>& destinations,
                uint16_t origin_idx, labelset_ptr_t labelset,
                const sif::TravelMode travelmode, sif::cost_ptr_t costing,
                std::shared_ptr<const sif::EdgeLabel> edgelabel)
{
  // Push dummy labels (invalid edgeid, zero cost, no predecessor) to
  // the queue for the initial expansion later. These dummy labels
  // will also serve as roots in search trees, and sentinels to
  // indicate it reaches the beginning of a route when constructing the
  // route
  const baldr::GraphTile* tile = nullptr;
  for (const auto& edge : destinations[origin_idx].edges) {
    if (!edge.id.Is_Valid()) continue;

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


void set_destinations(baldr::GraphReader& reader,
                      const std::vector<baldr::PathLocation>& destinations,
                      std::unordered_map<baldr::GraphId, std::unordered_set<uint16_t>>& node_dests,
                      std::unordered_map<baldr::GraphId, std::unordered_set<uint16_t>>& edge_dests)
{
  const baldr::GraphTile* tile = nullptr;
  for (uint16_t dest = 0; dest < destinations.size(); dest++) {
    for (const auto& edge : destinations[dest].edges) {
      if (!edge.id.Is_Valid()) continue;

      auto edge_nodes = reader.GetDirectedEdgeNodes(edge.id, tile);
      if (edge.begin_node()) {
        const auto nodeid = edge_nodes.first;
        if (!nodeid.Is_Valid()) continue;
        node_dests[nodeid].insert(dest);
      } else if (edge.end_node()) {
        const auto nodeid = edge_nodes.second;
        if (!nodeid.Is_Valid()) continue;
        node_dests[nodeid].insert(dest);

      } else {
        edge_dests[edge.id].insert(dest);
      }
    }
  }
}


// Need the graphreader to get the tile of edgelabel
inline uint16_t
get_inbound_edgelabel_heading(baldr::GraphReader& graphreader,
                              const sif::EdgeLabel& edgelabel,
                              const baldr::NodeInfo* nodeinfo)
{
  const auto idx = edgelabel.opp_local_idx();
  if (idx < 8) {
    return nodeinfo->heading(idx);
  } else {
    const baldr::GraphTile* tile = nullptr;
    const auto directededge = graphreader.directededge(edgelabel.edgeid(), tile);
    const auto edgeinfo = tile->edgeinfo(directededge->edgeinfo_offset());
    const auto& shape = edgeinfo.shape();
    if (shape.size() >= 2) {
      float heading;
      if (directededge->forward()) {
        heading = shape.back().Heading(shape.rbegin()[1]);
      } else {
        heading = shape.front().Heading(shape[1]);
      }
      return static_cast<uint16_t>(std::max(0.f, std::min(359.f, heading)));
    } else {
      return 0;
    }
  }
}


inline uint16_t
get_outbound_edge_heading(const baldr::DirectedEdge* outbound_edge,
                          const baldr::NodeInfo* nodeinfo)
{
  const auto idx = outbound_edge->localedgeidx();
  if (idx < 8) {
    return nodeinfo->heading(idx);
  } else {
    const baldr::GraphTile* tile = nullptr;
    const auto edgeinfo = tile->edgeinfo(outbound_edge->edgeinfo_offset());
    const auto& shape = edgeinfo.shape();
    if (shape.size() >= 2) {
      float heading;
      if (outbound_edge->forward()) {
        heading = shape.front().Heading(shape[1]);
      } else {
        heading = shape.back().Heading(shape.rbegin()[1]);
      }
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
                   std::shared_ptr<const sif::EdgeLabel> edgelabel,
                   const float turn_cost_table[181])
{
  sif::Cost label_cost;
  float label_turn_cost;
  std::shared_ptr<const sif::EdgeLabel> pred_edgelabel;
  const sif::TravelMode travelmode = costing->travel_mode();

  // Destinations along edges
  std::unordered_map<baldr::GraphId, std::unordered_set<uint16_t>> edge_dests;

  // Destinations at nodes
  std::unordered_map<baldr::GraphId, std::unordered_set<uint16_t>> node_dests;

  // Lambda for heuristic
  float search_rad2 = search_radius * search_radius;
  const auto heuristic = [&approximator, &search_radius, &search_rad2](const PointLL& lnglat) {
    float d2 = approximator.DistanceSquared(lnglat);
    return (d2 < search_rad2) ? 0.0f : sqrtf(d2) - search_radius;
  };

  // Lambda method to expand along edges from this node. This method have to be
  // set-up to be called recursively (for transition edges) so we set up a
  // function reference.
  std::function<void(const baldr::GraphId&, const uint32_t, const bool)> expand;
  expand = [&reader, &approximator, &pred_edgelabel, &labelset, &edge_dests,
            &destinations, &search_radius, &search_rad2, &costing, &travelmode, &label_cost,
            &label_turn_cost, &turn_cost_table, &heuristic, &expand]
           (const baldr::GraphId& node, const uint32_t label_idx, const bool from_transition) {
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
    const auto inbound_hdg = (pred_edgelabel) ?
             get_inbound_edgelabel_heading(reader, *pred_edgelabel, nodeinfo) : 0;

    // Expand from end node in forward direction.
    baldr::GraphId edgeid = { node.tileid(), node.level(), nodeinfo->edge_index() };
    const baldr::DirectedEdge* directededge = tile->directededge(edgeid);
    for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++directededge, ++edgeid) {
      // Skip it if its a shortcut or transit connection
      if (directededge->is_shortcut() ||
          directededge->use() == baldr::Use::kTransitConnection) {
        continue;
      }

      // Immediately expand from end node of transition edges unless expand is
      // called from a transition edge (disallow 2 successive transition edges)
      if (directededge->IsTransition()) {
        if (!from_transition) {
          expand(directededge->endnode(), label_idx, true);
        }
        continue;
      }

      // Skip it if its not allowed
      if (!IsEdgeAllowed(directededge, edgeid, costing, pred_edgelabel, tile)) {
        continue;
      }

      // Turn cost based on turn degree
      float turn_cost = label_turn_cost;
      if (pred_edgelabel) {
        // Get outbound heading (clamped to range [0,360])
        const auto outbound_hdg = get_outbound_edge_heading(directededge, nodeinfo);
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
              sif::Cost cost(label_cost.cost + directededge->length() * edge.dist,
                             label_cost.secs + costing->EdgeCost(directededge).secs * edge.dist);
              labelset->put(dest, edgeid, 0.f, edge.dist,
                           cost, turn_cost, cost.cost, label_idx,
                           directededge, travelmode, nullptr);
            }
          }
        }
      }

      // Get the end node tile and nodeinfo (to compute heuristic)
      const baldr::GraphTile* endtile = directededge->leaves_tile() ?
                    reader.GetGraphTile(directededge->endnode()) : tile;
      if (endtile != nullptr) {
        // Get cost - use EdgeCost to get time along the edge. Override
        // cost portion to be distance. Add heuristic to get sort cost.
        const auto end_nodeinfo = endtile->node(directededge->endnode());
        sif::Cost cost(label_cost.cost + directededge->length(),
                       label_cost.secs + costing->EdgeCost(directededge).secs);
        float sortcost = cost.cost + heuristic(end_nodeinfo->latlng());
        labelset->put(directededge->endnode(), edgeid, 0.f, 1.f,
                      cost, turn_cost, sortcost, label_idx,
                      directededge, travelmode, nullptr);
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

    // NOTE this reference is possible to be invalidated when you add
    // labels to the set later (which causes the label list
    // reallocated)
    const auto& label = labelset->label(label_idx);

    // So we cache values that will be used during expanding
    label_cost = label.cost;
    label_turn_cost = label.turn_cost;
    pred_edgelabel = label.edgelabel;
    const auto nodeid = label.nodeid;
    if (nodeid.Is_Valid()) {
      // If this node is a destination, path to destinations at this
      // node is found: remember them and remove this node from the
      // destination list
      const auto it = node_dests.find(nodeid);
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
      expand(nodeid, label_idx, false);
    } else { // assert(label.dest != kInvalidDestination)
      const auto dest = label.dest;

      // Path to a destination along an edge is found: remember it and
      // remove the destination from the destination list
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
          if (!directededge ||
              !IsEdgeAllowed(directededge, origin_edge.id, costing, pred_edgelabel, tile)) {
            continue;
          }

          // U-turn cost
          float turn_cost = label_turn_cost;
          if (pred_edgelabel && pred_edgelabel->edgeid() != origin_edge.id &&
              pred_edgelabel->opp_local_idx() == directededge->localedgeidx()) {
            turn_cost += turn_cost_table[0];
          }

          // All destinations on this origin edge
          for (const auto other_dest : edge_dests[origin_edge.id]) {
            // All edges of this destination
            for (const auto& other_edge : destinations[other_dest].edges) {
              if (origin_edge.id == other_edge.id && origin_edge.dist <= other_edge.dist) {
                // Get cost - use EdgeCost to get time along the edge. Override
                // cost portion to be distance. The heuristic cost from a
                // destination to itself must be 0
                float f = (other_edge.dist - origin_edge.dist);
                sif::Cost cost(label_cost.cost + directededge->length() * f,
                               label_cost.secs + costing->EdgeCost(directededge).secs * f);
                labelset->put(other_dest, origin_edge.id,
                             origin_edge.dist, other_edge.dist,
                             cost, turn_cost, cost.cost, label_idx,
                             directededge, travelmode, nullptr);
              }
            }
          }

          // Get the end node tile and nodeinfo (to compute heuristic)
          const baldr::GraphTile* endtile = directededge->leaves_tile() ?
                        reader.GetGraphTile(directededge->endnode()) : tile;
          if (endtile == nullptr) {
            continue;
          }
          const auto nodeinfo = endtile->node(directededge->endnode());

          // Get cost - use EdgeCost to get time along the edge. Override
          // cost portion to be distance. The heuristic cost from a
          // destination to itself must be 0
          float f = (1.f - origin_edge.dist);
          sif::Cost cost(label_cost.cost + directededge->length() * f,
                         label_cost.secs + costing->EdgeCost(directededge).secs * f);
          float sortcost = cost.cost + heuristic(nodeinfo->latlng());
          labelset->put(directededge->endnode(), origin_edge.id, origin_edge.dist, 1.f,
                       cost, turn_cost, sortcost, label_idx,
                       directededge, travelmode, nullptr);
        }
      }
    }
  }
  // TODO - do we need to clear since it is constructed prior to each call??
  labelset->clear_queue();
  labelset->clear_status();
  return results;
}

}

}
