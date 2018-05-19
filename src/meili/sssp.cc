#include <valhalla/meili/sssp.h>

namespace valhalla {
namespace meili {

/**
 * Get the inbound heading of the predecessor edge (using the edge Label).
 * @param  reader   Graphreader - need this in cases where the node info
 *                  does not have the heading (more than 8 edges at the node).
 * @param  label    Predecessor edge label.
 * @param  nodeinfo Nodeinfo at the end of the predecessor edge.
 * @return  Returns the inbound edge heading.
 */
inline uint16_t get_inbound_edgelabel_heading(
    baldr::GraphReader& reader,
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
      float heading = (directededge->forward()) ?
          shape.back().Heading(shape.rbegin()[1]) :
          shape.front().Heading(shape[1]);
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
inline uint16_t get_outbound_edge_heading(
    const baldr::GraphTile* tile,
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
      float heading = (outbound_edge->forward()) ?
          shape.front().Heading(shape[1]) :
          shape.back().Heading(shape.rbegin()[1]);
      return static_cast<uint16_t>(std::max(0.f, std::min(359.f, heading)));
    } else {
      return 0;
    }
  }
}

/**
 * Test if an edge is allowed. Checks if the predecessor edge label is valid
 * (invalid for origin edges) and whether costing allows the edge. Also checks
 * if still on the same edge as the predecessor (allow if so).
 */
inline bool
IsEdgeAllowed(const baldr::DirectedEdge* edge,
              const baldr::GraphId& edgeid,
              const sif::cost_ptr_t costing,
              const Label& pred_edgelabel,
              const baldr::GraphTile* tile) {
  return !pred_edgelabel.edgeid().Is_Valid()
      || edgeid == pred_edgelabel.edgeid()
      || costing->Allowed(edge, pred_edgelabel, tile, edgeid, 0, 0);
}

/**
 * Test if the path edge is located on the label.
 */
inline bool
IsPathEdgeOnLabel(const baldr::PathLocation::PathEdge& edge, const Label& label) {
  if (edge.id == label.edgeid()) {
    if (label.source() <= edge.percent_along && edge.percent_along <= label.target()) {
      return true;
    }
  }
  return false;
}

/**
 * Test if the path location has at least one path edge located on the label
 */
inline bool
IsPathLocationOnLabel(const baldr::PathLocation& location, const Label& label) {
  // dummy label
  if (!label.edgeid().Is_Valid()) {
    return false;
  }

  const auto it = std::find_if(
      location.edges.begin(),
      location.edges.end(),
      [&label](const baldr::PathLocation::PathEdge& edge) {
        return IsPathEdgeOnLabel(edge, label);
      });

  return it == location.edges.end();
}

/**
 * Find the path edge that has the minimal short cost on the label. This path
 * edge must be located on the label and it must be closest to the start of the
 * label. If not found the end iterator is returned.
 */
template<typename iter_t>
iter_t
FindShortestPathEdge(iter_t begin, iter_t end, const Label& label) {
  const auto& compare = [&label](const baldr::PathLocation::PathEdge& edge, const baldr::PathLocation::PathEdge& shortest) {
    const auto edge_cost = PathEdgeSortCost(edge, label);
    const auto shortest_cost = PathEdgeSortCost(shortest, label);
    if (0 <= edge_cost && 0 <= shortest_cost) {
      return edge_cost < shortest_cost;
    } else {
      return false;
    }
  };

  const auto it = std::min_element(begin, end, compare);
  if (it == end) {
    return it;
  } else {
    const auto cost = PathEdgeSortCost(*it, label);
    if (0 <= cost) {
      return it;
    } else {
      return end;
    }
  }
}

RoutePathIterator
SSSP::GetPath(const baldr::PathLocation& destination) const
{
  const auto& compare = [&destination](const Label& label, const Label& smallest) {
    // skip dummy label
    if (!label.edgeid().Is_Valid()) {
      return false;
    }

    const auto& edges = destination.edges;

    const auto edge = FindShortestPathEdge(edges.begin(), edges.end(), label);
    if (edge == destination.edges.end()) {
      return false;
    }

    const auto edge_on_smallest_label = FindShortestPathEdge(edges.begin(), edges.end(), smallest);
    if (edge_on_smallest_label == destination.edges.end()) {
      return false;
    }

    return (edge->percent_along - label.source()) < (edge_on_smallest_label->percent_along - smallest.source());
  };

  const auto& labels = tree_.labels();
  const auto it = std::min_element(labels.begin(), labels.end(), compare);
  if (it == labels.end()) {
    return path_end_;
  } else {
    // the smallest label is possible to be the first label which has no
    // destination on it, so check again
    if (IsPathLocationOnLabel(destination, *it)) {
      const auto label_idx = it - labels.begin();
      return RoutePathIterator(&tree_, label_idx);
    } else {
      return path_end_;
    }
  }
}

RoutePathIterator
SSSP::SearchPath(const baldr::PathLocation& destination) {
  const auto path = GetPath(destination);
  if (path != path_end()) {
    return path;
  }

  while (true) {
    const auto label_idx = tree_.pop();
    if (label_idx == baldr::kInvalidLabel) {
      // Exhausted labels without finding all destinations
      break;
    }

    // Copy the label since it is possible to be invalidated when new labels are
    // added.
    const auto& label = tree_.label(label_idx);

    // Congrats!
    if (IsPathLocationOnLabel(destination, label)) {
      break;
    }

    if (label.nodeid().Is_Valid()) {
      ExpandFromNode(label, label_idx, label.nodeid());
    } else {
      ExpandFromEdge(label, label_idx);
    }
  }

  return GetPath(destination);
}

// Push dummy labels (invalid edgeid, zero cost, no predecessor) to the search
// tree. These dummy labels will serve as initial labels for expansion later, as
// well as roots in the search tree (sentinels to indicate it reaches the
// beginning of a route when constructing the route).

// Two types of dummy labels:
// 1. dummy label that ends at the middle of an edge
// 2. dummy label that ends at an node
void SSSP::SetOrigin(const baldr::PathLocation& origin) {
  const sif::TravelMode travelmode = costing_->travel_mode();
  const baldr::GraphTile* tile = nullptr;
  for (const auto& edge : origin.edges) {
    if (!edge.id.Is_Valid()) continue;

    auto edge_nodes = graphreader_.GetDirectedEdgeNodes(edge.id, tile);
    if (edge.begin_node()) {
      const auto nodeid = edge_nodes.first;
      if (nodeid.Is_Valid()) {
        const auto nodeinfo = graphreader_.nodeinfo(nodeid, tile);
        if (!nodeinfo || !costing_->Allowed(nodeinfo)) {
          continue;
        }
        // Put type 1 dummy label, which will be expanded in Expand later
        tree_.put(nodeid, travelmode, nullptr);
      }
    } else if (edge.end_node()) {
      const auto nodeid = edge_nodes.second;
      if (nodeid.Is_Valid()) {
        const auto nodeinfo = graphreader_.nodeinfo(nodeid, tile);
        if (!nodeinfo || !costing_->Allowed(nodeinfo)) {
          continue;
        }
        // Put type 1 dummy label, which will be expanded in Expand later
        tree_.put(nodeid, travelmode, nullptr);
      }
    } else {
      // Put type 2 dummy label, which will be expanded in ExpandOrigin later
      tree_.put(0, travelmode, nullptr);
    }
  }
}

void SSSP::ExpandFromNode(const Label& label, const uint32_t label_idx, const baldr::GraphId& node, const bool from_transition) {
  const sif::TravelMode travelmode = costing_->travel_mode();

  // Get the node's info. The tile will be guaranteed to be nodeid's tile
  // in this block. Return if node is not found or is not allowed by costing
  const baldr::GraphTile* tile = graphreader_.GetGraphTile(node);
  if (!tile) {
    return;
  }

  const baldr::NodeInfo* nodeinfo = tile->node(node);
  if (!costing_->Allowed(nodeinfo)) {
    return;
  }

  // Get the inbound edge heading (clamped to range [0,360])
  const auto inbound_hdg = label.edgeid().Is_Valid() ?
                           get_inbound_edgelabel_heading(graphreader_, label, nodeinfo) : 0;

  // Expand from end node in forward direction.
  baldr::GraphId edgeid = { node.tileid(), node.level(), nodeinfo->edge_index() };
  auto directededge = tile->directededge(edgeid);
  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++directededge, ++edgeid) {
    // Skip it if its a shortcut or transit connection
    if (directededge->is_shortcut() || directededge->use() == baldr::Use::kTransitConnection) {
      continue;
    }

    // Immediately expand from end node of transition edges unless expand is
    // called from a transition edge (disallow 2 successive transition edges)
    if (directededge->IsTransition()) {
      if (!from_transition) {
        ExpandFromNode(label, label_idx, directededge->endnode(), true);
      }
      continue;
    }

    // Skip it if its not allowed
    if (!IsEdgeAllowed(directededge, edgeid, costing_, label, tile)) {
      continue;
    }

    // Get cost - use EdgeCost to get time along the edge. Override
    // cost portion to be distance.
    sif::Cost cost(label.cost().cost + directededge->length(),
                   label.cost().secs + costing_->EdgeCost(directededge).secs);
    if (AllowExpansion(cost)) {
      // Get outbound heading (clamped to range [0,360]) and add to turn cost
      // based on turn degree
      float turn_cost = label.turn_cost();
      if (label.edgeid().Is_Valid()) {
        const auto outbound_hdg = get_outbound_edge_heading(tile, directededge, nodeinfo);
        turn_cost += turn_cost_table_[midgard::get_turn_degree180(inbound_hdg, outbound_hdg)];
      }

      const float sortcost = cost.cost;

      tree_.put(
          directededge->endnode(),
          edgeid,
          0.0f,
          1.0f,
          cost,
          turn_cost,
          sortcost,
          label_idx,
          directededge,
          travelmode);
    }
  }
}

// TODO rename to ExpandFromEdge

/**
 * Expand dummy label
 */
void SSSP::ExpandFromEdge(const Label& label, uint32_t label_idx) {
  const sif::TravelMode travelmode = costing_->travel_mode();
  const baldr::GraphTile* tile = nullptr;
  for (const auto& origin_edge: source_.edges) {
    // The tile will be guaranteed to be directededge's tile in this block
    const auto directededge = graphreader_.directededge(origin_edge.id, tile);

    if (!directededge) {
      continue;
    }

    // Skip if edge is not allowed
    if (!IsEdgeAllowed(directededge, origin_edge.id, costing_, label, tile)) {
      continue;
    }

    // Get cost - use EdgeCost to get time along the edge. Override
    // cost portion to be distance. The heuristic cost from a
    // destination to itself must be 0
    float f = (1.0f - origin_edge.percent_along);
    const sif::Cost cost(label.cost().cost + directededge->length() * f,
                         label.cost().secs + costing_->EdgeCost(directededge).secs * f);
    if (AllowExpansion(cost)) {
      // U-turn cost
      float turn_cost = label.turn_cost();
      if (label.edgeid().Is_Valid() &&
          label.edgeid() != origin_edge.id &&
          label.opp_local_idx() == directededge->localedgeidx()) {
        turn_cost += turn_cost_table_[0];
      }

      const float sortcost = cost.cost;

      tree_.put(
          directededge->endnode(),
          origin_edge.id,
          origin_edge.percent_along,
          1.f,
          cost,
          turn_cost,
          sortcost,
          label_idx,
          directededge,
          travelmode);
    }
  }
}

}
}
