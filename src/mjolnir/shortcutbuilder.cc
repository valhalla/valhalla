#include "mjolnir/shortcutbuilder.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/util.h"
#include "scoped_timer.h"
#include "sif/osrm_car_duration.h"

#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>

#include <string>
#include <utility>
#include <vector>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

struct ShortcutAccessRestriction {
  std::unordered_map<AccessType, AccessRestriction> all_restrictions;
  // important to set the edge's attribute
  uint64_t modes = 0;

  ShortcutAccessRestriction(const std::vector<AccessRestriction>&& restrictions) {
    for (const auto& res : restrictions) {
      modes |= res.modes();
      all_restrictions.emplace(res.type(), std::move(res));
    }
  };

  // updates non-conditional restrictions if their value is lower than the current value
  // TODO(nils): we could also contract over conditional restrictions with a bit more work:
  //   kTimeDenied is fine to just append all restrictions of the base edges, but kTimeAllowed
  //   will be harder, there we'll have to merge overlapping time periods
  void update_nonconditional(const std::vector<AccessRestriction>&& other_restrictions) {
    for (const auto& new_ar : other_restrictions) {
      // update the modes for the edge attribute regardless
      if (new_ar.type() == AccessType::kTimedAllowed || new_ar.type() == AccessType::kTimedDenied ||
          new_ar.type() == AccessType::kDestinationAllowed) {
        continue;
      }
      modes |= new_ar.modes();
      auto ar_inserted = all_restrictions.emplace(new_ar.type(), new_ar);
      if (!ar_inserted.second && new_ar.value() < ar_inserted.first->second.value()) {
        ar_inserted.first->second = std::move(new_ar);
      }
    }
  }
};

// only keeps access restrictions which can fail contraction
void remove_nonconditional_restrictions(std::vector<AccessRestriction>& access_restrictions) {
  access_restrictions.erase(std::remove_if(std::begin(access_restrictions),
                                           std::end(access_restrictions),
                                           [](const AccessRestriction& elem) {
                                             return elem.type() != AccessType::kDestinationAllowed &&
                                                    elem.type() != AccessType::kTimedAllowed &&
                                                    elem.type() != AccessType::kTimedDenied;
                                           }),
                            std::end(access_restrictions));
}

// Simple structure to hold the 2 pair of directed edges at a node.
// First edge in the pair is incoming and second is outgoing
struct EdgePairs {
  std::pair<GraphId, GraphId> edge1;
  std::pair<GraphId, GraphId> edge2;
};

/**
 * Test if 2 edges have matching attributes such that they should be
 * considered for combining into a shortcut edge.
 */
bool EdgesMatch(const graph_tile_ptr& tile, const DirectedEdge* edge1, const DirectedEdge* edge2) {
  // Check if edges end at same node.
  if (edge1->endnode() == edge2->endnode()) {
    return false;
  }

  // Make sure access matches. Need to consider opposite direction for one of
  // the edges since both edges are outbound from the node.
  if (edge1->forwardaccess() != edge2->reverseaccess() ||
      edge1->reverseaccess() != edge2->forwardaccess()) {
    return false;
  }

  // Neither directed edge can have exit signs or be a roundabout.
  // TODO - other sign types?
  if (edge1->sign() || edge2->sign() || edge1->roundabout() || edge2->roundabout()) {
    return false;
  }

  // Neither edge can be part of a complex turn restriction
  if (edge1->start_restriction() || edge1->end_restriction() || edge2->start_restriction() ||
      edge2->end_restriction()) {
    return false;
  }

  // classification, link, use, and attributes must also match.
  // NOTE: might want "better" bridge attribution. Seems most overpasses
  // get marked as a bridge and lead to less shortcuts - so we don't consider
  // bridge and tunnel here
  if (edge1->classification() != edge2->classification() || edge1->link() != edge2->link() ||
      edge1->use() != edge2->use() || edge1->toll() != edge2->toll() ||
      edge1->destonly() != edge2->destonly() || edge1->destonly_hgv() != edge2->destonly_hgv() ||
      edge1->unpaved() != edge2->unpaved() || edge1->surface() != edge2->surface() ||
      edge1->roundabout() != edge2->roundabout()) {
    return false;
  }

  // if there's conditional access restrictions, they must match; others we can safely contract over
  if (edge1->access_restriction() || edge2->access_restriction()) {
    auto res1 = tile->GetAccessRestrictions(edge1 - tile->directededge(0), kVehicularAccess);
    remove_nonconditional_restrictions(res1);
    auto res2 = tile->GetAccessRestrictions(edge2 - tile->directededge(0), kVehicularAccess);
    remove_nonconditional_restrictions(res2);
    if (res1.size() != res2.size())
      return false;
    for (size_t i = 0; i < res1.size(); ++i) {
      if (res1[i].type() != res2[i].type() || res1[i].modes() != res2[i].modes() ||
          res1[i].value() != res2[i].value())
        return false;
    }
  }

  return true;
}

// Get the GraphId of the opposing edge.
GraphId GetOpposingEdge(const GraphId& node,
                        const DirectedEdge* edge,
                        GraphReader& reader,
                        const uint64_t wayid) {
  // Get the tile at the end node
  graph_tile_ptr tile = reader.GetGraphTile(edge->endnode());
  const NodeInfo* nodeinfo = tile->node(edge->endnode().id());

  // Get the directed edges and return when the end node matches
  // the specified node and length matches
  GraphId edgeid(edge->endnode().tileid(), edge->endnode().level(), nodeinfo->edge_index());
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n; i++, directededge++, ++edgeid) {
    if (directededge->use() == Use::kTransitConnection ||
        directededge->use() == Use::kEgressConnection ||
        directededge->use() == Use::kPlatformConnection ||
        directededge->use() == Use::kConstruction) {
      continue;
    }
    if (directededge->endnode() == node && directededge->classification() == edge->classification() &&
        directededge->length() == edge->length() &&
        ((directededge->link() && edge->link()) || (directededge->use() == edge->use())) &&
        wayid == tile->edgeinfo(directededge).wayid()) {
      return edgeid;
    }
  }
  PointLL ll = nodeinfo->latlng(tile->header()->base_ll());
  LOG_ERROR("Opposing directed edge not found at LL= " + std::to_string(ll.lat()) + "," +
            std::to_string(ll.lng()));
  return GraphId(0, 0, 0);
}

// Get the ISO country code at the end node
std::string EndNodeIso(const DirectedEdge* edge, GraphReader& reader) {
  graph_tile_ptr tile = reader.GetGraphTile(edge->endnode());
  const NodeInfo* nodeinfo = tile->node(edge->endnode().id());
  return tile->admininfo(nodeinfo->admin_index()).country_iso();
}

/**
 * Test if the node is eligible to be contracted (part of a shortcut).
 * @param reader    Graph reader.
 * @param tile      Current tile.
 * @param nodeinfo  Node information.
 * @param node      Node id.
 * @param edgepairs Ingoing and outgoing edge Ids (if node can be contracted).
 * @return  Returns true if the node can be contracted (part of shortcut),
 *          false if not.
 */
bool CanContract(GraphReader& reader,
                 const graph_tile_ptr& tile,
                 const GraphId& node,
                 EdgePairs& edgepairs) {
  const NodeInfo* nodeinfo = tile->node(node);
  if (!nodeinfo->can_contract()) {
    return false;
  }

  // Do not create a shortcut across a node that has any upward transitions
  if (nodeinfo->transition_count() > 0) {
    for (const auto& trans : tile->GetNodeTransitions(node)) {
      if (trans.up()) {
        return false;
      }
    }
  }

  // Get list of valid edges, excluding transit connection edges.
  // Also skip shortcut edge - this can happen if tile cache is cleared
  // and this enters a tile where shortcuts have already been created.
  std::vector<GraphId> edges;
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n; i++, ++edgeid) {
    const DirectedEdge* directededge = tile->directededge(edgeid);
    if (directededge->can_form_shortcut()) {
      edges.push_back(edgeid);
    }
  }

  // Must have only 2 edges at this level
  if (edges.size() != 2) {
    return false;
  }

  // Get the directed edges - these are the outbound edges from the node.
  const DirectedEdge* edge1 = tile->directededge(edges[0]);
  const DirectedEdge* edge2 = tile->directededge(edges[1]);

  if (!EdgesMatch(tile, edge1, edge2))
    return false;

  // Exactly one pair of edges match. Check if any other remaining edges
  // are drivable outbound from the node. If so this cannot be contracted.
  // NOTE-this seems to cause issues on PA Tpke / Breezewood
  /*  for (uint32_t i = 0; i < n; i++) {
      if (i != match.first && i != match.second) {
        if (tile->directededge(edges[i])->forwardaccess() & kAutoAccess)
          return false;
      }
    }*/

  // Get the opposing directed edges - these are the inbound edges to the node.
  uint64_t wayid1 = tile->edgeinfo(edge1).wayid();
  uint64_t wayid2 = tile->edgeinfo(edge2).wayid();
  GraphId oppedge1 = GetOpposingEdge(node, edge1, reader, wayid1);
  GraphId oppedge2 = GetOpposingEdge(node, edge2, reader, wayid2);
  const DirectedEdge* oppdiredge1 = reader.GetGraphTile(oppedge1)->directededge(oppedge1);
  const DirectedEdge* oppdiredge2 = reader.GetGraphTile(oppedge2)->directededge(oppedge2);

  // If either opposing directed edge has exit signs return false
  if (oppdiredge1->sign() || oppdiredge2->sign()) {
    return false;
  }

  // Do not allow a shortcut on a ramp crossing at a traffic signal or where
  // more than 3 edges meet.
  if (edge1->link() && edge2->link() && (nodeinfo->traffic_signal() || nodeinfo->edge_count() > 3)) {
    return false;
  }

  // Cannot have turn restriction from either inbound edge edge to
  // the other outbound edge
  if (((oppdiredge1->restrictions() & (1 << edge2->localedgeidx())) != 0) ||
      ((oppdiredge2->restrictions() & (1 << edge1->localedgeidx())) != 0)) {
    return false;
  }

  // ISO country codes at the end nodes must equal this node
  std::string iso = tile->admininfo(nodeinfo->admin_index()).country_iso();
  std::string e1_iso = EndNodeIso(edge1, reader);
  std::string e2_iso = EndNodeIso(edge2, reader);
  if (e1_iso != iso || e2_iso != iso) {
    return false;
  }

  // Simple check for a possible maneuver where the continuation is a turn
  // and there are other edges at the node (forward intersecting edge or a
  // 'T' intersection
  if (nodeinfo->local_edge_count() > 2) {
    // Find number of drivable edges
    uint32_t drivable = 0;
    for (uint32_t i = 0; i < nodeinfo->local_edge_count(); i++) {
      if (nodeinfo->local_driveability(i) != Traversability::kNone) {
        drivable++;
      }
    }
    if (drivable > 2) {
      uint32_t heading1 = (nodeinfo->heading(edge1->localedgeidx()) + 180) % 360;
      uint32_t turn_degree = GetTurnDegree(heading1, nodeinfo->heading(edge2->localedgeidx()));
      if (turn_degree > 60 && turn_degree < 300) {
        return false;
      }
    }
  }

  // Store the pairs of base edges entering and exiting this node
  edgepairs.edge1 = std::make_pair(oppedge1, edges[1]);
  edgepairs.edge2 = std::make_pair(oppedge2, edges[0]);
  return true;
}

// Connect 2 edges shape and update the next end node in the new level
void ConnectEdges(GraphReader& reader,
                  const GraphId& startnode,
                  const GraphId& edgeid,
                  std::list<PointLL>& shape,
                  GraphId& endnode,
                  uint32_t& opp_local_idx,
                  uint32_t& restrictions,
                  float& average_density,
                  float& total_duration,
                  float& total_truck_duration,
                  ShortcutAccessRestriction& access_restrictions) {
  // Get the tile and directed edge.
  auto tile = reader.GetGraphTile(startnode);
  const DirectedEdge* directededge = tile->directededge(edgeid);

  // Add edge and turn duration for car
  auto const nodeinfo = tile->node(startnode);
  auto const turn_duration = OSRMCarTurnDuration(directededge, nodeinfo, opp_local_idx);
  total_duration += turn_duration;
  auto const speed = tile->GetSpeed(directededge, kNoFlowMask);
  assert(speed != 0);
  auto const edge_duration = directededge->length() / (speed * kKPHtoMetersPerSec);
  total_duration += edge_duration;

  // Add edge and turn duration for truck
  total_truck_duration += turn_duration;
  auto const truck_speed = tile->GetSpeed(directededge, kNoFlowMask, kInvalidSecondsOfWeek, true);
  assert(truck_speed != 0);
  auto const edge_duration_truck = directededge->length() / (truck_speed * kKPHtoMetersPerSec);
  total_truck_duration += edge_duration_truck;

  // Copy the restrictions and opposing local index. Want to set the shortcut
  // edge's restrictions and opp_local_idx to the last directed edge in the chain
  opp_local_idx = directededge->opp_local_idx();
  restrictions = directededge->restrictions();

  // Get the shape for this edge. Reverse if directed edge is not forward.
  auto encoded = tile->edgeinfo(directededge).encoded_shape();
  std::list<PointLL> edgeshape = valhalla::midgard::decode7<std::list<PointLL>>(encoded);
  if (!directededge->forward()) {
    std::reverse(edgeshape.begin(), edgeshape.end());
  }

  // Append shape to the shortcut's shape. Skip first point since it
  // should equal the last of the prior edge.
  edgeshape.pop_front();
  shape.splice(shape.end(), edgeshape);

  // Add to the weighted average
  average_density += directededge->length() * directededge->density();

  // Preserve the most restrictive access restrictions
  access_restrictions.update_nonconditional(tile->GetAccessRestrictions(edgeid.id(), kAllAccess));

  // Update the end node
  endnode = directededge->endnode();
}

// Check if the edge is entering a contracted node
bool IsEnteringEdgeOfContractedNode(GraphReader& reader, const GraphId& nodeid, const GraphId& edge) {
  EdgePairs edgepairs;
  graph_tile_ptr tile = reader.GetGraphTile(nodeid);
  bool c = CanContract(reader, tile, nodeid, edgepairs);
  return c && (edgepairs.edge1.first == edge || edgepairs.edge2.first == edge);
}

// Add shortcut edges (if they should exist) from the specified node
std::pair<uint32_t, uint32_t> AddShortcutEdges(GraphReader& reader,
                                               const graph_tile_ptr& tile,
                                               GraphTileBuilder& tilebuilder,
                                               const GraphId& start_node,
                                               const uint32_t edge_index,
                                               const uint32_t edge_count,
                                               std::unordered_map<uint32_t, uint32_t>& shortcuts) {
  // Shortcut edges have to start at a node that is not contracted - return if
  // this node can be contracted.
  EdgePairs edgepairs;
  if (CanContract(reader, tile, start_node, edgepairs)) {
    return {uint32_t(0), uint32_t(0)};
  }

  // Check if this is the last edge in a shortcut (if the endnode cannot be contracted).
  auto last_edge = [&reader](const graph_tile_ptr& tile, const GraphId& endnode,
                             EdgePairs& edgepairs) {
    return !CanContract(reader, tile, endnode, edgepairs);
  };

  // Iterate through directed edges of the base node
  uint32_t shortcut = 0;
  uint32_t shortcut_count = 0;
  uint32_t total_edge_count = 0;
  GraphId edge_id(start_node.tileid(), start_node.level(), edge_index);
  for (uint32_t i = 0; i < edge_count; i++, ++edge_id) {
    // Skip transit connection edges.
    const DirectedEdge* directededge = tile->directededge(edge_id);
    if (!directededge->can_form_shortcut()) {
      continue;
    }

    // NOTE - only kMaxShortcutsFromNode are allowed from a node. However,
    // a problem exists if we do not add a shortcut and the opposing shortcut
    // is added. So more than kMaxShortcutsFromNode can be created, but we will
    // only mark up to kMaxShortcutsFromNode regular edges as superseded.

    // Get the end node and check if the edge is set as a matching, entering
    // edge of the contracted node.
    GraphId end_node = directededge->endnode();
    if (IsEnteringEdgeOfContractedNode(reader, end_node, edge_id)) {
      total_edge_count++;
      // Form a shortcut edge.
      DirectedEdge newedge = *directededge;

      // For computing weighted density and total turn duration along the shortcut
      uint32_t edge_length = newedge.length();
      float average_density = edge_length * newedge.density();
      uint32_t const speed = tile->GetSpeed(directededge, kNoFlowMask);
      assert(speed != 0);
      float total_duration = edge_length / (speed * kKPHtoMetersPerSec);
      uint32_t const truck_speed =
          std::min(tile->GetSpeed(directededge, kNoFlowMask, kInvalidSecondsOfWeek, true),
                   directededge->truck_speed() ? directededge->truck_speed() : kMaxAssumedTruckSpeed);
      assert(truck_speed != 0);
      float total_truck_duration = edge_length / (truck_speed * kKPHtoMetersPerSec);

      // Get the shape for this edge. If this initial directed edge is not
      // forward - reverse the shape so the edge info stored is forward for
      // the first added edge info
      auto edgeinfo = tile->edgeinfo(directededge);
      std::list<PointLL> shape =
          valhalla::midgard::decode7<std::list<PointLL>>(edgeinfo.encoded_shape());
      if (!directededge->forward()) {
        std::reverse(shape.begin(), shape.end());
      }

      // store all access_restrictions of the base edge: non-conditional ones will be updated while
      // contracting, conditional ones are breaking contraction and are safe to simply copy
      ShortcutAccessRestriction access_restrictions{
          tile->GetAccessRestrictions(edge_id.id(), kAllAccess)};

      // Connect edges to the shortcut while the end node is marked as
      // contracted (contains edge pairs in the shortcut info).
      uint32_t rst = 0;
      // For turn duration calculation during contraction
      uint32_t opp_local_idx = directededge->opp_local_idx();
      GraphId next_edge_id = edge_id;
      while (true) {
        EdgePairs edgepairs;
        graph_tile_ptr tile = reader.GetGraphTile(end_node);
        if (last_edge(tile, end_node, edgepairs)) {
          break;
        }

        // Edge should match one of the 2 first (inbound) edges in the
        // pair. Choose the matching outgoing (second) edge.
        if (edgepairs.edge1.first == next_edge_id) {
          next_edge_id = edgepairs.edge1.second;
        } else if (edgepairs.edge2.first == next_edge_id) {
          next_edge_id = edgepairs.edge2.second;
        } else {
          // Break out of loop. This case can happen when a shortcut edge
          // enters another shortcut edge (but is not drivable in reverse
          // direction from the node).
          const DirectedEdge* de = tile->directededge(next_edge_id);
          LOG_ERROR("Edge not found in edge pairs. WayID = " +
                    std::to_string(tile->edgeinfo(de).wayid()));
          break;
        }

        // Connect the matching outbound directed edge (updates the next
        // end node in the new level). Keep track of the last restriction
        // on the connected shortcut - need to set that so turn restrictions
        // off of shortcuts work properly
        ConnectEdges(reader, end_node, next_edge_id, shape, end_node, opp_local_idx, rst,
                     average_density, total_duration, total_truck_duration, access_restrictions);
        total_edge_count++;
      }

      // Get the length from the shape. This prevents roundoff issues when forming
      // elevation.
      uint32_t length = valhalla::midgard::length(shape);

      // Add the edge info. Use length and number of shape points to match an
      // edge in case multiple shortcut edges exist between the 2 nodes.
      // Test whether this shape is forward or reverse (in case an existing
      // edge exists). Shortcuts use way Id = 0.Set mean elevation to 0 as a placeholder,
      // set it later if adding elevation to this dataset. No need for names etc, shortcuts
      // aren't used in guidance
      bool forward = true;
      uint32_t idx = ((length & 0xfffff) | ((shape.size() & 0xfff) << 20));
      uint32_t edge_info_offset =
          tilebuilder.AddEdgeInfo(idx, start_node, end_node, 0, 0, edgeinfo.bike_network(),
                                  edgeinfo.speed_limit(), shape, {}, {}, {}, 0, forward, false);
      ;

      newedge.set_edgeinfo_offset(edge_info_offset);

      // Set the forward flag on this directed edge. If a new edge was added
      // the direction is forward otherwise the prior edge was the one stored
      // in the forward direction
      newedge.set_forward(forward);

      // Shortcut edge has the opp_local_idx and restrictions of the last
      // directed edge in the shortcut chain
      newedge.set_opp_local_idx(opp_local_idx);
      newedge.set_restrictions(rst);

      // add new access restrictions if any and set the mask on the edge
      if (access_restrictions.all_restrictions.size()) {
        newedge.set_access_restriction(access_restrictions.modes);
        for (const auto& res : access_restrictions.all_restrictions) {
          tilebuilder.AddAccessRestriction(AccessRestriction(tilebuilder.directededges().size(),
                                                             res.second.type(), res.second.modes(),
                                                             res.second.value(),
                                                             res.second.except_destination()));
        }
      }

      // set new access mask

      // Update the length, curvature, and end node
      newedge.set_length(length);
      newedge.set_curvature(compute_curvature(shape));
      newedge.set_endnode(end_node);

      // Set the default weighted grade for the edge. No edge elevation is added.
      newedge.set_weighted_grade(6);

      // Sanity check - should never see a shortcut with signs
      if (newedge.sign()) {
        LOG_ERROR("Shortcut edge with exit signs");
      }

      // Get turn lanes from the base directed edge. Add them if this is the last edge otherwise
      // set the turnlanes flag to false;
      if (directededge->turnlanes() && last_edge(reader.GetGraphTile(directededge->endnode()),
                                                 directededge->endnode(), edgepairs)) {
        uint32_t offset = tile->turnlanes_offset(edge_id.id());
        tilebuilder.AddTurnLanes(tilebuilder.directededges().size(), tile->GetName(offset));
        newedge.set_turnlanes(true);
      } else {
        newedge.set_turnlanes(false);
      }

      // TODO: for now just drop lane connectivity for shortcuts
      // they can be retrieved later by re-tracing path (via trace_attribute)
      if (newedge.laneconnectivity()) {
        newedge.set_laneconnectivity(false);
      }

      // Compute the weighted edge density
      newedge.set_density(average_density / (static_cast<float>(length)));

      // Update speed to the one that takes turn durations into account
      assert(total_duration > 0);
      uint32_t new_speed =
          static_cast<uint32_t>(std::round(length / total_duration * kMetersPerSectoKPH));
      newedge.set_speed(new_speed);

      assert(total_truck_duration > 0);
      uint32_t new_truck_speed =
          static_cast<uint32_t>(std::round(length / total_truck_duration * kMetersPerSectoKPH));
      newedge.set_truck_speed(new_truck_speed);

      // Add shortcut edge. Add to the shortcut map (associates the base edge
      // index to the shortcut index). Remove superseded mask that may have
      // been copied from base directed edge
      shortcuts[i] = shortcut + 1;
      newedge.set_shortcut(shortcut + 1);
      newedge.set_superseded(0);

      // Make sure shortcut edge is not marked as internal edge
      newedge.set_internal(false);

      // Add new directed edge to tile builder
      tilebuilder.directededges().emplace_back(std::move(newedge));
      shortcut_count++;
      shortcut++;
    }
  }

  // Log a warning (with the node lat,lon) if the max number of shortcuts from a node
  // is exceeded. This is not serious (see NOTE above) but good to know where it occurs.
  if (shortcut_count > kMaxShortcutsFromNode) {
    PointLL ll = tile->get_node_ll(start_node);
    LOG_WARN("Exceeding max shortcut edges from a node at LL = " + std::to_string(ll.lat()) + "," +
             std::to_string(ll.lng()));
  }
  return {shortcut_count, total_edge_count};
}

// Form shortcuts for tiles in this level.
std::pair<uint32_t, uint32_t> FormShortcuts(GraphReader& reader, const TileLevel& level) {
  // Iterate through the tiles at this level (TODO - can we mark the tiles
  // the tiles that shortcuts end within?)
  reader.Clear();
  bool added = false;
  uint32_t shortcut_count = 0;
  uint32_t total_edge_count = 0;
  uint32_t ntiles = level.tiles.TileCount();
  uint32_t tile_level = (uint32_t)level.level;
  graph_tile_ptr tile;
  for (uint32_t tileid = 0; tileid < ntiles; tileid++) {
    // Get the graph tile. Skip if no tile exists (common case)
    tile = reader.GetGraphTile(GraphId(tileid, tile_level, 0));
    if (!tile) {
      continue;
    }

    // Create GraphTileBuilder for the new tile
    GraphId new_tile(tileid, tile_level, 0);
    GraphTileBuilder tilebuilder(reader.tile_dir(), new_tile, false);

    // Since the old tile is not serialized we must copy any data that is not
    // dependent on edge Id into the new builders (e.g., node transitions)
    if (tile->header()->transitioncount() > 0) {
      for (uint32_t i = 0; i < tile->header()->transitioncount(); ++i) {
        tilebuilder.transitions().emplace_back(std::move(*(tile->transition(i))));
      }
    }

    // Iterate through the nodes in the tile
    GraphId node_id(tileid, tile_level, 0);
    for (uint32_t n = 0; n < tile->header()->nodecount(); n++, ++node_id) {
      // Get the node info, copy node index and count from old tile
      NodeInfo nodeinfo = *(tile->node(node_id));
      uint32_t old_edge_index = nodeinfo.edge_index();
      uint32_t old_edge_count = nodeinfo.edge_count();

      // Update node information
      const auto& admin = tile->admininfo(nodeinfo.admin_index());
      nodeinfo.set_edge_index(tilebuilder.directededges().size());
      nodeinfo.set_admin_index(tilebuilder.AddAdmin(admin.country_text(), admin.state_text(),
                                                    admin.country_iso(), admin.state_iso()));

      // Current edge count
      size_t edge_count = tilebuilder.directededges().size();

      // Add shortcut edges first.
      std::unordered_map<uint32_t, uint32_t> shortcuts;
      auto stats = AddShortcutEdges(reader, tile, tilebuilder, node_id, old_edge_index,
                                    old_edge_count, shortcuts);
      shortcut_count += stats.first;
      total_edge_count += stats.second;

      // Copy the rest of the directed edges from this node
      GraphId edgeid(tileid, tile_level, old_edge_index);
      for (uint32_t i = 0; i < old_edge_count; i++, ++edgeid) {
        // Copy the directed edge information and update end node,
        // edge data offset, and opp_index
        const DirectedEdge* directededge = tile->directededge(edgeid);
        DirectedEdge newedge = *directededge;

        // Get signs from the base directed edge
        if (directededge->sign()) {
          std::vector<SignInfo> signs = tile->GetSigns(edgeid.id());
          if (signs.size() == 0) {
            LOG_ERROR("Base edge should have signs, but none found");
          }
          tilebuilder.AddSigns(tilebuilder.directededges().size(), signs);
        }

        // Get turn lanes from the base directed edge
        if (directededge->turnlanes()) {
          uint32_t offset = tile->turnlanes_offset(edgeid.id());
          tilebuilder.AddTurnLanes(tilebuilder.directededges().size(), tile->GetName(offset));
        }

        // Get access restrictions from the base directed edge. Add these to
        // the list of access restrictions in the new tile. Update the
        // edge index in the restriction to be the current directed edge Id
        if (directededge->access_restriction()) {
          auto restrictions = tile->GetAccessRestrictions(edgeid.id(), kAllAccess);
          for (const auto& res : restrictions) {
            tilebuilder.AddAccessRestriction(AccessRestriction(tilebuilder.directededges().size(),
                                                               res.type(), res.modes(), res.value(),
                                                               res.except_destination()));
          }
        }

        // Copy lane connectivity
        if (directededge->laneconnectivity()) {
          auto laneconnectivity = tile->GetLaneConnectivity(edgeid.id());
          if (laneconnectivity.size() == 0) {
            LOG_ERROR("Base edge should have lane connectivity, but none found");
          }
          for (auto& lc : laneconnectivity) {
            lc.set_to(tilebuilder.directededges().size());
          }
          tilebuilder.AddLaneConnectivity(laneconnectivity);
        }

        // Names can be different in the forward and backward direction
        bool diff_names = tilebuilder.OpposingEdgeInfoDiffers(tile, directededge);

        // Get edge info, shape, and names from the old tile and add
        // to the new. Use prior edgeinfo offset as the key to make sure
        // edges that have the same end nodes are differentiated (this
        // should be a valid key since tile sizes aren't changed)
        auto edgeinfo = tile->edgeinfo(directededge);
        uint32_t edge_info_offset =
            tilebuilder.AddEdgeInfo(directededge->edgeinfo_offset(), node_id, directededge->endnode(),
                                    edgeinfo.wayid(), edgeinfo.mean_elevation(),
                                    edgeinfo.bike_network(), edgeinfo.speed_limit(),
                                    edgeinfo.encoded_shape(), edgeinfo.GetNames(),
                                    edgeinfo.GetTaggedValues(), edgeinfo.GetLinguisticTaggedValues(),
                                    edgeinfo.GetTypes(), added, diff_names);

        newedge.set_edgeinfo_offset(edge_info_offset);

        // Set the superseded mask - this is the shortcut mask that supersedes this edge
        // (outbound from the node). Do not set (keep as 0) if maximum number of shortcuts
        // from a node has been exceeded.
        auto s = shortcuts.find(i);
        uint32_t superseded_idx = (s != shortcuts.end()) ? s->second : 0;
        if (superseded_idx <= kMaxShortcutsFromNode) {
          newedge.set_superseded(superseded_idx);
        }

        // Add directed edge
        tilebuilder.directededges().emplace_back(std::move(newedge));
      }

      // Set the edge count for the new node
      nodeinfo.set_edge_count(tilebuilder.directededges().size() - edge_count);

      // Get named signs from the base node
      if (nodeinfo.named_intersection()) {

        std::vector<SignInfo> signs = tile->GetSigns(n, true);
        if (signs.size() == 0) {
          LOG_ERROR("Base node should have signs, but none found");
        }
        tilebuilder.AddSigns(tilebuilder.nodes().size(), signs);
      }
      tilebuilder.nodes().emplace_back(std::move(nodeinfo));
    }

    // Store the new tile
    tilebuilder.StoreTileData();
    LOG_DEBUG((boost::format("ShortcutBuilder created tile %1%: %2% bytes") % tile %
               tilebuilder.header_builder().end_offset())
                  .str());

    // Check if we need to clear the tile cache.
    if (reader.OverCommitted()) {
      reader.Trim();
    }
  }
  return {shortcut_count, total_edge_count};
}

} // namespace

namespace valhalla {
namespace mjolnir {

// Build shortcuts. Shortcut edges are possible through nodes that
// only connect to 2 edges on the hierarchy level, and have compatible
// attributes. Shortcut edges are inserted before regular edges.
void ShortcutBuilder::Build(const boost::property_tree::ptree& pt) {

  // TODO: thread this. would need to make sure we dont make shortcuts
  // across tile boundaries so that we are only messing with one tile
  // in one thread at a time

  SCOPED_TIMER();
  // Get GraphReader
  GraphReader reader(pt.get_child("mjolnir"));

  auto tile_level = TileHierarchy::levels().rbegin();
  tile_level++;
  for (; tile_level != TileHierarchy::levels().rend(); ++tile_level) {
    // Create shortcuts on this level
    LOG_INFO("Creating shortcuts on level " + std::to_string(tile_level->level));
    [[maybe_unused]] auto stats = FormShortcuts(reader, *tile_level);
    [[maybe_unused]] uint32_t avg = stats.first ? (stats.second / stats.first) : 0;
    LOG_INFO("Finished with " + std::to_string(stats.first) + " shortcuts superseding " +
             std::to_string(stats.second) + " edges, average ~" + std::to_string(avg) +
             " edges per shortcut");
  }
}

} // namespace mjolnir
} // namespace valhalla
