#include "mjolnir/shortcutbuilder.h"
#include "valhalla/mjolnir/graphtilebuilder.h"

#include <ostream>
#include <sstream>
#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <boost/property_tree/ptree.hpp>
#include <boost/format.hpp>

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/encoded.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/skadi/sample.h>
#include <valhalla/skadi/util.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::skadi;
using namespace valhalla::mjolnir;

namespace {

//how many meters to resample shape to when checking elevations
constexpr double POSTING_INTERVAL = 60.0;

// Do not compute grade for intervals less than 10 meters.
constexpr double kMinimumInterval = 10.0f;

// Simple structure to hold the 2 pair of directed edges at a node.
// First edge in the pair is incoming and second is outgoing
struct EdgePairs {
  std::pair<GraphId, GraphId> edge1;
  std::pair<GraphId, GraphId> edge2;
};

/**
 * Sample elevation along the shape to get weighted grade and max grades
 */
std::tuple<double, double, double> GetGrade(
                const std::unique_ptr<const valhalla::skadi::sample>& sample,
                const std::list<PointLL>& shape, const float length,
                const bool forward) {
  // For very short lengths just return 0 grades
  if (length < kMinimumInterval) {
    return std::make_tuple(0.0, 0.0, 0.0);
  }

  // Evenly sample the shape. If edge is really short, just do both ends
  std::list<PointLL> resampled;
  auto interval = POSTING_INTERVAL;
  if(length < POSTING_INTERVAL * 3) {
    resampled = {shape.front(), shape.back()};
    interval = length;
  } else {
    resampled = valhalla::midgard::resample_spherical_polyline(shape, POSTING_INTERVAL);
  }

  // Get the heights at each point
  auto heights = sample->get_all(resampled);
  if (!forward) {
    std::reverse(heights.begin(), heights.end());
  }

  // Compute the grade valid range is between -10 and +15
  return valhalla::skadi::weighted_grade(heights, interval);
}

/**
 * Test if 2 edges have matching attributes such that they should be
 * considered for combining into a shortcut edge.
 */
bool EdgesMatch(const GraphTile* tile, const DirectedEdge* edge1,
                const DirectedEdge* edge2) {
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
  if (edge1->exitsign() || edge2->exitsign() ||
      edge1->roundabout() || edge2->roundabout()) {
    return false;
  }

  // classification, link, use, and attributes must also match.
  // NOTE: might want "better" bridge attribution. Seems most overpasses
  // get marked as a bridge and lead to less shortcuts - so we don't consider
  // bridge and tunnel here
  if (edge1->classification() != edge2->classification()
      || edge1->link() != edge2->link()
      || edge1->use() != edge2->use()
      || edge1->speed() != edge2->speed()
      || edge1->toll() != edge2->toll()
      || edge1->destonly() != edge2->destonly()
      || edge1->unpaved() != edge2->unpaved()
      || edge1->surface() != edge2->surface()
      || edge1->roundabout() != edge2->roundabout()) {
    return false;
  }

  // Names must match
  // TODO - this allows matches in any order. Do we need to maintain order?
  // TODO - should allow near matches?
  std::vector<std::string> edge1names = tile->GetNames(edge1->edgeinfo_offset());
  std::vector<std::string> edge2names = tile->GetNames(edge2->edgeinfo_offset());
  if (edge1names.size() != edge2names.size()) {
    return false;
  }
  for (const auto& name1 : edge1names) {
    bool found = false;
    for (const auto& name2 : edge2names) {
      if (name1 == name2) {
        found = true;
        break;
      }
    }
    if (!found) {
      return false;
    }
  }
  return true;
}

// Get the GraphId of the opposing edge.
GraphId GetOpposingEdge(const GraphId& node, const DirectedEdge* edge,
                        GraphReader& reader) {
  // Get the tile at the end node
  const GraphTile* tile = reader.GetGraphTile(edge->endnode());
  const NodeInfo* nodeinfo = tile->node(edge->endnode().id());

  // Get the directed edges and return when the end node matches
  // the specified node and length matches
  GraphId edgeid(edge->endnode().tileid(), edge->endnode().level(),
                 nodeinfo->edge_index());
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
                i++, directededge++, edgeid++) {
    if (directededge->endnode() == node &&
        directededge->classification() == edge->classification() &&
        directededge->length() == edge->length() &&
      ((directededge->link() && edge->link()) || (directededge->use() == edge->use()))) {
      return edgeid;
    }
  }
  LOG_ERROR("Opposing directed edge not found at LL= " +
                  std::to_string(nodeinfo->latlng().lat()) + "," +
                  std::to_string(nodeinfo->latlng().lng()));
  return GraphId(0, 0, 0);
}

// Get the ISO country code at the end node
std::string EndNodeIso(const DirectedEdge* edge, GraphReader& reader) {
  const GraphTile* tile = reader.GetGraphTile(edge->endnode());
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
bool CanContract(GraphReader& reader, const GraphTile* tile,
                 const GraphId& node, EdgePairs& edgepairs) {
  // Return false if only 1 edge
  const NodeInfo* nodeinfo = tile->node(node);
  if (nodeinfo->edge_count() < 2) {
    return false;
  }

  // Do not contract if the node is a gate or toll booth
  if (nodeinfo->type() == NodeType::kGate ||
      nodeinfo->type() == NodeType::kTollBooth ||
      nodeinfo->intersection() == IntersectionType::kFork) {
    return false;
  }

  // Get list of valid edges, excluding transition and transit connection
  // edges. Also skip shortcut edge - this can happen if tile cache is cleared
  // and this enters a tile where shortcuts have already been created.
  std::vector<GraphId> edges;
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n; i++, edgeid++) {
    const DirectedEdge* directededge = tile->directededge(edgeid);
    if (!directededge->trans_down() && !directededge->trans_up() &&
        directededge->use() != Use::kTransitConnection &&
        !directededge->is_shortcut()) {
      edges.push_back(edgeid);
    }
  }

  // Must have only 2 edges at this level
  if (edges.size() != 2) {
    return false;
  }

  // Get pairs of matching edges. If more than 1 pair exists then
  // we cannot contract this node.
  uint32_t n = edges.size();
  bool matchfound = false;
  std::pair<uint32_t, uint32_t> match;
  for (uint32_t i = 0; i < n - 1; i++) {
    for (uint32_t j = i + 1; j < n; j++) {
      const DirectedEdge* edge1 = tile->directededge(edges[i]);
      const DirectedEdge* edge2 = tile->directededge(edges[j]);
      if (EdgesMatch(tile, edge1, edge2)) {
        if (matchfound) {
          // More than 1 match exists - return false
          return false;
        }
        // Save the match
        match = std::make_pair(i, j);
        matchfound = true;
      }
    }
  }

  // Return false if no matches exist
  if (!matchfound) {
    return false;
  }

  // Exactly one pair of edges match. Check if any other remaining edges
  // are driveable outbound from the node. If so this cannot be contracted.
  // NOTE-this seems to cause issues on PA Tpke / Breezewood
/*  for (uint32_t i = 0; i < n; i++) {
    if (i != match.first && i != match.second) {
      if (tile->directededge(edges[i])->forwardaccess() & kAutoAccess)
        return false;
    }
  }*/

  // Get the directed edges - these are the outbound edges from the node.
  // Get the opposing directed edges - these are the inbound edges to the node.
  const DirectedEdge* edge1 = tile->directededge(edges[match.first]);
  const DirectedEdge* edge2 = tile->directededge(edges[match.second]);
  GraphId oppedge1 = GetOpposingEdge(node, edge1, reader);
  GraphId oppedge2 = GetOpposingEdge(node, edge2, reader);
  const DirectedEdge* oppdiredge1 =
      reader.GetGraphTile(oppedge1)->directededge(oppedge1);
  const DirectedEdge* oppdiredge2 =
      reader.GetGraphTile(oppedge2)->directededge(oppedge2);

  // If either opposing directed edge has exit signs return false
  if (oppdiredge1->exitsign() || oppdiredge2->exitsign()) {
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
  if (e1_iso != iso || e2_iso != iso)
    return false;

  // Simple check for a possible maneuver where the continuation is a turn
  // and there are other edges at the node (forward intersecting edge or a
  // 'T' intersection
  if (nodeinfo->local_edge_count() > 2) {
    // Find number of driveable edges
    uint32_t driveable = 0;
    for (uint32_t i = 0; i < nodeinfo->local_edge_count(); i++) {
      if (nodeinfo->local_driveability(i) != Traversability::kNone) {
        driveable++;
      }
    }
    if (driveable > 2) {
      uint32_t heading1 = (nodeinfo->heading(edge1->localedgeidx()) + 180) % 360;
      uint32_t turn_degree = GetTurnDegree(heading1, nodeinfo->heading(edge2->localedgeidx()));
      if (turn_degree > 60 && turn_degree < 300) {
        return false;
      }
    }
  }

  // Store the pairs of base edges entering and exiting this node
  edgepairs.edge1 = std::make_pair(oppedge1, edges[match.second]);
  edgepairs.edge2 = std::make_pair(oppedge2, edges[match.first]);
  return true;
}

// Connect 2 edges shape and update the next end node in the new level
uint32_t ConnectEdges(GraphReader& reader, const GraphId& startnode,
                      const GraphId& edgeid,
                      std::list<PointLL>& shape, GraphId& endnode,
                      uint32_t& opp_local_idx,  uint32_t& restrictions) {
  // Get the tile and directed edge.
  const GraphTile* tile = reader.GetGraphTile(startnode);
  const DirectedEdge* directededge = tile->directededge(edgeid);

  // Copy the restrictions and opposing local index. Want to set the shortcut
  // edge's restrictions and opp_local_idx to the last directed edge in the chain
  opp_local_idx = directededge->opp_local_idx();
  restrictions = directededge->restrictions();

  // Get the shape for this edge. Reverse if directed edge is not forward.
  auto encoded = tile->edgeinfo(directededge->edgeinfo_offset()).encoded_shape();
  std::list<PointLL> edgeshape = valhalla::midgard::decode7<std::list<PointLL> >(encoded);
  if (!directededge->forward()) {
    std::reverse(edgeshape.begin(), edgeshape.end());
  }

  // Append shape to the shortcut's shape. Skip first point since it
  // should equal the last of the prior edge.
  edgeshape.pop_front();
  shape.splice(shape.end(), edgeshape);

  // Update the end node and return the length
  endnode = directededge->endnode();
  return directededge->length();
}

// Check if the edge is entering a contracted node
bool IsEnteringEdgeOfContractedNode(GraphReader& reader, const GraphId& nodeid,
                                    const GraphId& edge) {
  EdgePairs edgepairs;
  const GraphTile* tile = reader.GetGraphTile(nodeid);
  bool c = CanContract(reader, tile, nodeid, edgepairs);
  return c && (edgepairs.edge1.first == edge || edgepairs.edge2.first == edge);
}

// Add shortcut edges (if they should exist) from the specified node
// TODO - need to add access restrictions?
uint32_t AddShortcutEdges(GraphReader& reader, const GraphTile* tile,
              GraphTileBuilder& tilebuilder, const GraphId& start_node,
              const uint32_t edge_index, const uint32_t edge_count,
              std::unordered_map<uint32_t, uint32_t>& shortcuts,
              const std::unique_ptr<const valhalla::skadi::sample>& sample) {
  // Shortcut edges have to start at a node that is not contracted - return if
  // this node can be contracted.
  EdgePairs edgepairs;
  if (CanContract(reader, tile, start_node, edgepairs)) {
    return 0;
  }

  // Iterate through directed edges of the base node
  uint32_t shortcut = 0;
  uint32_t shortcut_count = 0;
  GraphId edge_id(start_node.tileid(), start_node.level(), edge_index);
  for (uint32_t i = 0; i < edge_count; i++, edge_id++) {
    // Skip transition edges and transit connections.
    const DirectedEdge* directededge = tile->directededge(edge_id);
    if (directededge->trans_up() || directededge->trans_down() ||
        directededge->use() == Use::kTransitConnection) {
      continue;
    }

    // Get the end node and check if the edge is set as a matching, entering
    // edge of the contracted node.
    GraphId end_node = directededge->endnode();
    if (IsEnteringEdgeOfContractedNode(reader, end_node, edge_id)) {
      // Form a shortcut edge.
      DirectedEdge newedge = *directededge;
      uint32_t length = newedge.length();

      // Get the shape for this edge. If this initial directed edge is not
      // forward - reverse the shape so the edge info stored is forward for
      // the first added edge info
      auto edgeinfo = tile->edgeinfo(directededge->edgeinfo_offset());
      std::list<PointLL> shape = valhalla::midgard::decode7<std::list<PointLL> >(edgeinfo.encoded_shape());
      if (!directededge->forward())
        std::reverse(shape.begin(), shape.end());

      // Get names - they apply over all edges of the shortcut
      auto names = tile->GetNames(directededge->edgeinfo_offset());

      // Add any access restriction records. TODO - make sure we don't contract
      // across edges with different restrictions.
      if (newedge.access_restriction()) {
        auto restrictions = tile->GetAccessRestrictions(edge_id.id(), kAllAccess);
        for (const auto& res : restrictions) {
          tilebuilder.AddAccessRestriction(
              AccessRestriction(tilebuilder.directededges().size(),
                  res.type(), res.modes(), res.days_of_week(), res.value()));
        }
      }

      // Connect edges to the shortcut while the end node is marked as
      // contracted (contains edge pairs in the shortcut info).
      uint32_t rst = 0;
      uint32_t opp_local_idx = 0;
      GraphId next_edge_id = edge_id;
      while (true) {
        EdgePairs edgepairs;
        const GraphTile* tile = reader.GetGraphTile(end_node);
        if (!CanContract(reader, tile, end_node, edgepairs)) {
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
          // enters another shortcut edge (but is not driveable in reverse
          // direction from the node).
          LOG_INFO("Edge not found in edge pairs?");
          break;
        }

        // Connect the matching outbound directed edge (updates the next
        // end node in the new level). Keep track of the last restriction
        // on the connected shortcut - need to set that so turn restrictions
        // off of shortcuts work properly
        length += ConnectEdges(reader, end_node, next_edge_id, shape, end_node,
                               opp_local_idx, rst);
      }

      // Add the edge info. Use length and number of shape points to match an
      // edge in case multiple shortcut edges exist between the 2 nodes.
      // Test whether this shape is forward or reverse (in case an existing
      // edge exists).
      // TODO - what should the wayId be?
      bool forward = true;
      uint32_t idx = ((length & 0xfffff) | ((shape.size() & 0xfff) << 20));
      uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(idx, start_node,
                          end_node, -1, shape, names, forward);
      newedge.set_edgeinfo_offset(edge_info_offset);

      // Set the forward flag on this directed edge. If a new edge was added
      // the direction is forward otherwise the prior edge was the one stored
      // in the forward direction
      newedge.set_forward(forward);

      // Shortcut edge has the opp_local_idx and restrictions of the last
      // directed edge in the shortcut chain
      newedge.set_opp_local_idx(opp_local_idx);
      newedge.set_restrictions(rst);

      // Update the length, elevation, curvature, and end node
      newedge.set_length(length);
      if (sample) {
        auto grades = GetGrade(sample, shape, length, forward);
        newedge.set_weighted_grade(static_cast<uint32_t>(std::get<0>(grades) * .6 + 6.5));
        newedge.set_max_up_slope(std::get<1>(grades));
        newedge.set_max_down_slope(std::get<2>(grades));
      } else {
        newedge.set_weighted_grade(6);  // 6 is flat
        newedge.set_max_up_slope(0.0f);
        newedge.set_max_down_slope(0.0f);
      }
      newedge.set_curvature(0); //TODO:
      newedge.set_endnode(end_node);

      // Sanity check - should never see a shortcut with signs
      if (newedge.exitsign()) {
        LOG_ERROR("Shortcut edge with exit signs");
      }

      // Add shortcut edge. Add to the shortcut map (associates the base edge
      // index to the shortcut index). Remove superseded mask that may have
      // been copied from base directed edge
      shortcuts[i] = shortcut+1;
      newedge.set_shortcut(shortcut+1);
      newedge.set_superseded(0);

      // Make sure shortcut edge is not marked as internal edge
      newedge.set_internal(false);

      // Add new directed edge to tile builder
      tilebuilder.directededges().emplace_back(std::move(newedge));
      shortcut_count++;
      shortcut++;
    }
  }
  return shortcut_count;
}

// Form shortcuts for tiles in this level.
uint32_t FormShortcuts(GraphReader& reader,
            const TileHierarchy::TileLevel& level,
            const std::unique_ptr<const valhalla::skadi::sample>& sample) {
  // Iterate through the tiles at this level (TODO - can we mark the tiles
  // the tiles that shortcuts end within?)
  reader.Clear();
  bool added = false;
  uint32_t shortcut_count = 0;
  uint32_t ntiles = level.tiles.TileCount();
  uint32_t tile_level = (uint32_t)level.level;
  const GraphTile* tile = nullptr;
  for (uint32_t tileid = 0; tileid < ntiles; tileid++) {
    // Get the graph tile. Skip if no tile exists (common case)
    tile = reader.GetGraphTile(GraphId(tileid, tile_level, 0));
    if (tile == nullptr || tile->header()->nodecount() == 0) {
      continue;
    }

    // Create GraphTileBuilder for the new tile
    GraphId new_tile(tileid, tile_level, 0);
    GraphTileBuilder tilebuilder(reader.GetTileHierarchy(), new_tile, false);

    // Create a dummy admin at index 0.  Used if admins are not used/created.
    tilebuilder.AddAdmin("None", "None", "", "");

    // Iterate through the nodes in the tile
    GraphId node_id(tileid, tile_level, 0);
    for (uint32_t n = 0; n < tile->header()->nodecount(); n++, node_id++) {
      // Get the node info, copy node index and count from old tile
      NodeInfo nodeinfo = *(tile->node(node_id));
      uint32_t old_edge_index = nodeinfo.edge_index();
      uint32_t old_edge_count = nodeinfo.edge_count();

      // Update node information
      const auto& admin = tile->admininfo(nodeinfo.admin_index());
      nodeinfo.set_edge_index(tilebuilder.directededges().size());
      nodeinfo.set_timezone(nodeinfo.timezone());
      nodeinfo.set_admin_index(tilebuilder.AddAdmin(admin.country_text(),
                admin.state_text(), admin.country_iso(), admin.state_iso()));

      // Current edge count
      size_t edge_count = tilebuilder.directededges().size();

      // Add shortcut edges first.
      std::unordered_map<uint32_t, uint32_t> shortcuts;
      shortcut_count += AddShortcutEdges(reader, tile, tilebuilder, node_id,
                   old_edge_index, old_edge_count, shortcuts, sample);

      // Copy the rest of the directed edges from this node
      GraphId edgeid(tileid, tile_level, old_edge_index);
      for (uint32_t i = 0; i < old_edge_count; i++, edgeid++) {
        // Copy the directed edge information and update end node,
        // edge data offset, and opp_index
        const DirectedEdge* directededge = tile->directededge(edgeid);
        DirectedEdge newedge = *directededge;

        // Transition edges are stored as is (no need for EdgeInfo, signs,
        // or restrictions).
        if (!directededge->trans_down() && !directededge->trans_up()) {
          // Get signs from the base directed edge
          if (directededge->exitsign()) {
            std::vector<SignInfo> signs = tile->GetSigns(edgeid.id());
            if (signs.size() == 0) {
              LOG_ERROR("Base edge should have signs, but none found");
            }
            tilebuilder.AddSigns(tilebuilder.directededges().size(), signs);
          }

          // Get access restrictions from the base directed edge. Add these to
          // the list of access restrictions in the new tile. Update the
          // edge index in the restriction to be the current directed edge Id
          if (directededge->access_restriction()) {
            auto restrictions = tile->GetAccessRestrictions(edgeid.id(), kAllAccess);
            for (const auto& res : restrictions) {
              tilebuilder.AddAccessRestriction(
                  AccessRestriction(tilebuilder.directededges().size(),
                     res.type(), res.modes(), res.days_of_week(), res.value()));
            }
          }

          // Get edge info, shape, and names from the old tile and add
          // to the new. Use edge length to protect against
          // edges that have same end nodes but different lengths
          auto edgeinfo = tile->edgeinfo(directededge->edgeinfo_offset());
          uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(directededge->length(),
                          node_id, directededge->endnode(), edgeinfo.wayid(), edgeinfo.shape(),
                          tile->GetNames(directededge->edgeinfo_offset()), added);
          newedge.set_edgeinfo_offset(edge_info_offset);

          // Set the superseded mask - this is the shortcut mask that
          // supersedes this edge (outbound from the node)
          auto s = shortcuts.find(i);
          uint32_t supersed_idx = (s != shortcuts.end()) ? s->second : 0;
          newedge.set_superseded(supersed_idx);
        }

        // Add directed edge
        tilebuilder.directededges().emplace_back(std::move(newedge));
      }

      // Set the edge count for the new node
      nodeinfo.set_edge_count(tilebuilder.directededges().size() - edge_count);
      tilebuilder.nodes().emplace_back(std::move(nodeinfo));
    }

    // Store the new tile
    tilebuilder.StoreTileData();
    LOG_DEBUG((boost::format("ShortcutBuilder created tile %1%: %2% bytes") %
         tile % tilebuilder.size()).str());

    // Check if we need to clear the tile cache.
    if (reader.OverCommitted()) {
      reader.Clear();
    }
  }
  return shortcut_count;
}

}

namespace valhalla {
namespace mjolnir {

// Build shortcuts. Shortcut edges are possible through nodes that
// only connect to 2 edges on the hierarchy level, and have compatible
// attributes. Shortcut edges are inserted before regular edges.
void ShortcutBuilder::Build(const boost::property_tree::ptree& pt) {

  //TODO: thread this. would need to make sure we dont make shortcuts
  //across tile boundaries so that we are only messing with one tile
  //in one thread at a time

  // Get GraphReader
  GraphReader reader(pt.get_child("mjolnir"));
  const auto& tile_hierarchy = reader.GetTileHierarchy();
  if (reader.GetTileHierarchy().levels().size() < 2) {
    throw std::runtime_error("Bad tile hierarchy - need 2 levels");
  }

  // Crack open some elevation data if its there
  boost::optional<std::string> elevation = pt.get_optional<std::string>("additional_data.elevation");
  std::unique_ptr<const skadi::sample> sample;
  if (elevation) {
    sample.reset(new skadi::sample(*elevation));
  }

  auto level = tile_hierarchy.levels().rbegin();
  level++;
  for ( ; level != tile_hierarchy.levels().rend(); ++level) {
    // Create shortcuts on this level
    auto tile_level = level->second;
    LOG_INFO("Creating shortcuts on level " + std::to_string(tile_level.level));
    uint32_t count = FormShortcuts(reader, tile_level, sample);
    LOG_INFO("Finished with " + std::to_string(count) + " shortcuts");
  }
}

}
}
