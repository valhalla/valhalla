#include "mjolnir/hierarchybuilder.h"
#include "valhalla/mjolnir/graphtilebuilder.h"

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <boost/property_tree/ptree.hpp>

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/skadi/sample.h>
#include <valhalla/skadi/util.h>


#include <boost/format.hpp>

#include <ostream>
#include <set>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::skadi;
using namespace valhalla::mjolnir;

namespace {

//how many meters to resample shape to when checking elevations
constexpr double POSTING_INTERVAL = 60.0;

// Simple structure to describe a connection between 2 levels
struct NodeConnection {
  GraphId basenode;
  GraphId newnode;

  NodeConnection(const GraphId& bn, const GraphId& nn)
      : basenode(bn),
        newnode(nn) {
  }

  // For sorting by Id
  bool operator < (const NodeConnection& other) const {
    return basenode.id() < other.basenode.id();
  }
};

// Simple structure representing nodes in the new level
struct NewNode {
  GraphId basenode;
  bool contract;

  NewNode(const GraphId& bn, const bool c)
      : basenode(bn),
        contract(c) {
  }
};

// Simple structure to hold the 2 pair of directed edges at a node.
// First edge in the pair is incoming and second is outgoing
struct EdgePairs {
  std::pair<GraphId, GraphId> edge1;
  std::pair<GraphId, GraphId> edge2;
};

struct hierarchy_info {
  uint32_t contractcount_;
  uint32_t shortcutcount_;
  GraphReader graphreader_;
  std::vector<std::vector<NewNode> > tilednodes_;
  std::unordered_map<uint64_t, GraphId> nodemap_;
  std::unordered_map<uint64_t, EdgePairs> contractions_;
};

bool EdgesMatch(const GraphTile* tile, const DirectedEdge* edge1,
                                  const DirectedEdge* edge2) {
  // Check if edges end at same node.
  if (edge1->endnode() == edge2->endnode()) {
    return false;
  }

  // Make sure access matches. Need to consider opposite direction for one of
  // the edges since both edges are outbound from the node.
  if (edge1->forwardaccess() != edge2->reverseaccess()
      || edge1->reverseaccess() != edge2->forwardaccess()) {
    return false;
  }

  // Neither directed edge can have exit signs.
  // TODO - other sign types?
  if (edge1->exitsign() || edge2->exitsign()) {
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
      || edge1->ferry() != edge2->ferry()
      || edge1->railferry() != edge2->railferry()
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
GraphId GetOpposingEdge(const GraphId& node,
                                          const DirectedEdge* edge, GraphReader& graphreader) {
  // Get the tile at the end node
  const GraphTile* tile = graphreader.GetGraphTile(edge->endnode());
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
        directededge->use() == edge->use() &&
        directededge->length() == edge->length()) {
      return edgeid;
    }
  }
  LOG_ERROR("Opposing directed edge not found!");
  return GraphId(0, 0, 0);
}

// Get the ISO country code at the end node
std::string EndNodeIso(const DirectedEdge* edge, hierarchy_info& info) {
  const GraphTile* tile = info.graphreader_.GetGraphTile(edge->endnode());
  const NodeInfo* nodeinfo = tile->node(edge->endnode().id());
  return tile->admininfo(nodeinfo->admin_index()).country_iso();
}

// Test if the node is eligible to be contracted (part of a shortcut) in
// the new level.
bool CanContract(const GraphTile* tile, const NodeInfo* nodeinfo,
                                   const GraphId& basenode,
                                   const GraphId& newnode,
                                   const RoadClass rcc, hierarchy_info& info) {
  // Return false if only 1 edge
  if (nodeinfo->edge_count() < 2) {
    return false;
  }

  // Get list of valid edges from the base level that remain at this level.
  // Exclude transition edges and shortcut edges on the base level.
  std::vector<GraphId> edges;
  GraphId edgeid(basenode.tileid(), basenode.level(), nodeinfo->edge_index());
  for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n; i++, edgeid++) {
    const DirectedEdge* directededge = tile->directededge(edgeid);
    if (directededge->classification() <= rcc && !directededge->trans_down()
        && !directededge->is_shortcut()) {
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
  GraphId oppedge1 = GetOpposingEdge(basenode, edge1, info.graphreader_);
  GraphId oppedge2 = GetOpposingEdge(basenode, edge2, info.graphreader_);
  const DirectedEdge* oppdiredge1 =
          info.graphreader_.GetGraphTile(oppedge1)->directededge(oppedge1);
  const DirectedEdge* oppdiredge2 =
      info.graphreader_.GetGraphTile(oppedge2)->directededge(oppedge2);

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
  std::string e1_iso = EndNodeIso(edge1, info);
  std::string e2_iso = EndNodeIso(edge2, info);
  if (e1_iso.empty() || e2_iso.empty() || iso.empty() ||
      e1_iso != iso || e2_iso != iso)
    return false;

  // Store the pairs of base edges entering and exiting this node
  EdgePairs edgepairs;
  edgepairs.edge1 = std::make_pair(oppedge1, edges[match.second]);
  edgepairs.edge2 = std::make_pair(oppedge2, edges[match.first]);
  info.contractions_[newnode.value] = edgepairs;

  info.contractcount_++;
  return true;
}

// Connect 2 edges shape and update the next end node in the new level
void ConnectEdges(const GraphId& basenode,
                                     const GraphId& edgeid,
                                     std::vector<PointLL>& shape,
                                     GraphId& nodeb,
                                     uint32_t& opp_local_idx, hierarchy_info& info) {
  // Get the tile and directed edge. Set the opp_local_idx
  const GraphTile* tile = info.graphreader_.GetGraphTile(basenode);
  const DirectedEdge* directededge = tile->directededge(edgeid);
  opp_local_idx = directededge->opp_local_idx();

  // Get the shape for this edge and append to the shortcut's shape
  std::vector<PointLL> edgeshape = tile->edgeinfo(
          directededge->edgeinfo_offset())->shape();
  if (directededge->forward()) {
    shape.insert(shape.end(), edgeshape.begin() + 1, edgeshape.end());
  } else {
    shape.insert(shape.end(), edgeshape.rbegin() + 1, edgeshape.rend());
  }

  // Update the end node and return the length
  nodeb = info.nodemap_[directededge->endnode().value];
}


bool IsEnteringEdgeOfContractedNode(const GraphId& node, const GraphId& edge,
              const std::unordered_map<uint64_t, EdgePairs>& contractions_) {
  auto edgepairs = contractions_.find(node.value);
  if (edgepairs == contractions_.cend()) {
    LOG_WARN("No edge pairs found for contracted node");
    return false;
  } else {
    return (edgepairs->second.edge1.first == edge
        || edgepairs->second.edge2.first == edge);
  }
}

uint32_t GetGrade(const std::unique_ptr<const valhalla::skadi::sample>& sample, const std::vector<PointLL>& shape, const float length, const bool forward) {
  //evenly sample the shape
  std::vector<PointLL> resampled;
  //if it was really short just do both ends
  auto interval = POSTING_INTERVAL;
  if(length <= POSTING_INTERVAL) {
    resampled = {shape.front(), shape.back()};
    interval = length;
  }
  else
    resampled = valhalla::midgard::resample_spherical_polyline(shape, POSTING_INTERVAL);
  //get the heights at each point
  auto heights = sample->get_all(resampled);
  if(!forward)
    std::reverse(heights.begin(), heights.end());
  //compute the grade valid range is between -10 and +15
  return static_cast<uint32_t>((valhalla::skadi::weighted_grade(heights, interval) + 10.0) / 25.0 + .5);
}

// Add shortcut edges (if they should exist) from the specified node
// Should never combine 2 directed edges with different exit information so
// no need to worry about it here.
void AddShortcutEdges(
    const NewNode& newnode, const GraphId& nodea, const NodeInfo* baseni,
    const GraphTile* tile, const RoadClass rcc, GraphTileBuilder& tilebuilder,
    std::vector<DirectedEdgeBuilder>& directededges,
    std::unordered_map<uint32_t, uint32_t>& shortcuts, hierarchy_info& info,
    const std::unique_ptr<const valhalla::skadi::sample>& sample) {
  // Get the edge pairs for this node (if contracted)
  auto edgepairs = newnode.contract ?
      info.contractions_.find(nodea.value) : info.contractions_.end();

  // Iterate through directed edges of the base node
  uint32_t shortcut = 0;
  GraphId base_edge_id(newnode.basenode.tileid(), newnode.basenode.level(),
                       baseni->edge_index());
  for (uint32_t i = 0, n = baseni->edge_count(); i < n; i++, base_edge_id++) {
    // Skip if > road class cutoff or a transition edge or shortcut in
    // the base level. Note that only downward transitions exist at this
    // point (upward transitions are created later).
    const DirectedEdge* directededge = tile->directededge(base_edge_id);
    if (directededge->classification() > rcc || directededge->trans_down()
        || directededge->is_shortcut()) {
      continue;
    }

    // Check edgepairs for this node. If this edge is in the pair of exiting
    // shortcut edges at this node we skip it
    if (edgepairs != info.contractions_.end()) {
      if (edgepairs->second.edge1.second == base_edge_id ||
          edgepairs->second.edge2.second == base_edge_id) {
        continue;
      }
    }

    // Get the end node and check if it is set for contraction and the edge
    // is set as a matching, entering edge of the contracted node. Cases like
    // entrance ramps can lead to a contracted node
    GraphId basenode = newnode.basenode;
    GraphId nodeb = info.nodemap_[directededge->endnode().value];
    if (info.tilednodes_[nodeb.tileid()][nodeb.id()].contract
        && IsEnteringEdgeOfContractedNode(nodeb, base_edge_id, info.contractions_)) {

      // Form a shortcut edge.
      //TODO: this seems really dangerous, we need a virtual destructor in directededge
      //then we need to do dynamic_cast<const DirectedEdgeBuilder&>(*directededge);
      DirectedEdgeBuilder newedge = static_cast<const DirectedEdgeBuilder&>(*directededge);

      // Get the shape for this edge. If this initial directed edge is not
      // forward - reverse the shape so the edge info stored is forward for
      // the first added edge info
      std::unique_ptr<const EdgeInfo> edgeinfo = tile->edgeinfo(
          directededge->edgeinfo_offset());
      std::vector<PointLL> shape;
      if (directededge->forward()) {
        shape = edgeinfo->shape();
      } else {
        // Reverse the shape
        std::vector<PointLL> edgeshape = edgeinfo->shape();
        shape.insert(shape.end(), edgeshape.rbegin(), edgeshape.rend());
      }

      // Get names - they apply over all edges of the shortcut
      std::vector<std::string> names = tile->GetNames(
          directededge->edgeinfo_offset());

      // TODO - should not be any signs

      // Connect while the node is marked as contracted. Use the edge pair
      // mapping
      uint32_t opp_local_idx = 0;
      GraphId next_edge_id = base_edge_id;
      while (info.tilednodes_[nodeb.tileid()][nodeb.id()].contract) {
        // Get base node and the contracted node
        basenode = info.tilednodes_[nodeb.tileid()][nodeb.id()].basenode;
        auto edgepairs = info.contractions_.find(nodeb.value);
        if (edgepairs == info.contractions_.end()) {
          LOG_WARN("No edge pairs found for contracted node");
          break;
        } else {
          // Oldedge should match one of the 2 first (inbound) edges in the
          // pair. Choose the matching outgoing (second) edge.
          if (edgepairs->second.edge1.first == next_edge_id) {
            next_edge_id = edgepairs->second.edge1.second;
          } else if (edgepairs->second.edge2.first == next_edge_id) {
            next_edge_id = edgepairs->second.edge2.second;
          } else {
            // Break out of loop. This case can happen when a shortcut edge
            // enters another shortcut edge (but is not driveable in reverse
            // direction from the node).
            break;
          }
        }

        // Connect the matching outbound directed edge (updates the next
        // end node in the new level).
        ConnectEdges(basenode, next_edge_id, shape, nodeb, opp_local_idx, info);
      }

      // Add the edge info. Use length to match edge in case multiple edges
      // exist between the 2 nodes. Test whether this shape is forward or
      // reverse (in case an existing edge exists).
      // TODO - what should the wayId be?
      bool forward = true;
      float length = PointLL::Length(shape);
      uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(static_cast<uint32_t>(length + .5f),
                                                          nodea, nodeb, -1, shape, names, forward);
      newedge.set_edgeinfo_offset(edge_info_offset);

      // Set the forward flag on this directed edge. If a new edge was added
      // the direction is forward otherwise the prior edge was the one stored
      // in the forward direction
      newedge.set_forward(forward);

      // Shortcut edge has the opp_local_idx of the last directed edge in
      // the shortcut chain
      newedge.set_opp_local_idx(opp_local_idx);

      // Update the length, elevation, curvature and end node
      newedge.set_length(length);
      newedge.set_weighted_grade(sample ? GetGrade(sample, shape, length, forward) : 6); //6 is flat
      newedge.set_curvature(0); //TODO:
      newedge.set_endnode(nodeb);

      if (newedge.exitsign()) {
        LOG_ERROR("Shortcut edge with exit signs");
      }

/**
if (nodea.level() == 0) {
  LOG_INFO((boost::format("Add shortcut from %1% LL %2%,%3% to %4%")
     % nodea % baseni->latlng().lat() % baseni->latlng().lng() % nodeb).str());
}
**/
      // Add shortcut edge. Add to the shortcut map (associates the base edge
      // index to the shortcut index).Remove superseded mask that may have
      // been copied from base level directed edge
      shortcuts[i] = shortcut+1;
      newedge.set_shortcut(shortcut+1);
      newedge.set_superseded(0);

      // Make sure shortcut edge is not marked as internal edge
      newedge.set_internal(false);

      directededges.emplace_back(std::move(newedge));
      shortcut++;
    }
  }
  info.shortcutcount_ += directededges.size();
}

// Form tiles in the new level.
void FormTilesInNewLevel(
    const TileHierarchy::TileLevel& base_level,
    const TileHierarchy::TileLevel& new_level, hierarchy_info& info,
    const std::unique_ptr<const valhalla::skadi::sample>& sample) {
  // Iterate through tiled nodes in the new level
  bool added = false;
  uint32_t tileid = 0;
  uint32_t nodeid = 0;
  uint32_t edgeindex = 0;
  uint32_t edge_info_offset;
  uint8_t level = new_level.level;
  RoadClass rcc = new_level.importance;

  info.graphreader_.Clear();

  for (const auto& newtile : info.tilednodes_) {
    // Skip if no nodes in the tile at the new level
    if (newtile.size() == 0) {
      tileid++;
      continue;
    }

    // Check if we need to clear the tile cache
    if(info.graphreader_.OverCommitted())
      info.graphreader_.Clear();

    // Create GraphTileBuilder for the new tile
    GraphTileBuilder tilebuilder;

    //Creating a dummy admin at index 0.  Used if admins are not used/created.
    tilebuilder.AddAdmin("None","None","","","","");

    // Iterate through the nodes in the tile at the new level
    nodeid = 0;
    edgeindex = 0;
    GraphId nodea, nodeb;
    for (const auto& newnode : newtile) {
      // Get the node in the base level
      const GraphTile* tile = info.graphreader_.GetGraphTile(newnode.basenode);

      // Copy node information
      nodea.Set(tileid, level, nodeid);
      NodeInfo baseni = *(tile->node(newnode.basenode.id()));
      NodeInfoBuilder node = static_cast<NodeInfoBuilder&>(baseni);
      node.set_edge_index(edgeindex);

      const auto& admin = tile->admininfo(baseni.admin_index());
      node.set_admin_index(tilebuilder.AddAdmin(admin.country_text(), admin.state_text(),
                                                admin.country_iso(), admin.state_iso(),
                                                admin.start_dst(), admin.end_dst()));
      // Add shortcut edges first
      std::unordered_map<uint32_t, uint32_t> shortcuts;
      std::vector<DirectedEdgeBuilder> directededges;
      AddShortcutEdges(newnode, nodea, &baseni, tile, rcc, tilebuilder,
                         directededges, shortcuts, info, sample);

      // Iterate through directed edges of the base node to get remaining
      // directed edges (based on classification/importance cutoff)
      GraphId oldedgeid(newnode.basenode.tileid(), newnode.basenode.level(),
                        baseni.edge_index());
      for (uint32_t i = 0, n = baseni.edge_count(); i < n; i++, oldedgeid++) {
        // Store the directed edge if less than the road class cutoff and
        // it is not a transition edge or shortcut in the base level
        const DirectedEdge* directededge = tile->directededge(oldedgeid);
        if (directededge->classification() <= rcc && !directededge->trans_down()
            && !directededge->is_shortcut()) {
          // Copy the directed edge information and update end node,
          // edge data offset, and opp_index
          DirectedEdge oldedge = *directededge;
          DirectedEdgeBuilder newedge =
              static_cast<DirectedEdgeBuilder&>(oldedge);

          // Set the end node for this edge. Opposing edge indexes
          // get set in graph optimizer so set to 0 here.
          nodeb = info.nodemap_[directededge->endnode().value];
          newedge.set_endnode(nodeb);
          newedge.set_opp_index(0);

          // Get signs from the base directed edge
          if (oldedge.exitsign()) {
            std::vector<SignInfo> signs = tile->GetSigns(oldedgeid.id());
            if (signs.size() == 0) {
              LOG_ERROR("Base edge should have signs, but none found");
            }
            tilebuilder.AddSigns(edgeindex + directededges.size(), signs);
          }

          // Get edge info, shape, and names from the old tile and add
          // to the new. Use edge length to protect against
          // edges that have same end nodes but different lengths
          std::unique_ptr<const EdgeInfo> edgeinfo = tile->edgeinfo(
                              directededge->edgeinfo_offset());
          edge_info_offset = tilebuilder.AddEdgeInfo(directededge->length(),
                             nodea, nodeb, edgeinfo->wayid(), edgeinfo->shape(),
                             tile->GetNames(directededge->edgeinfo_offset()),
                             added);
          newedge.set_edgeinfo_offset(edge_info_offset);

          // Set the superseded mask - this is the shortcut mask that
          // supersedes this edge (outbound from the node)
          auto s = shortcuts.find(i);
          if (s != shortcuts.end()) {
            newedge.set_superseded(s->second);
          } else {
            newedge.set_superseded(0);
          }

          // Add directed edge
          directededges.emplace_back(std::move(newedge));
        }
      }

      // Add the downward transition edge.
      // TODO - what access for downward transitions
      DirectedEdgeBuilder downwardedge;
      downwardedge.set_endnode(newnode.basenode);
      downwardedge.set_trans_down(true);
      downwardedge.set_all_forward_access();
      directededges.emplace_back(std::move(downwardedge));

      // Set the edge count for the new node
      node.set_edge_count(directededges.size());

      // Add node and directed edge information to the tile
      tilebuilder.AddNodeAndDirectedEdges(node, directededges);

      // Increment node Id and edgeindex
      nodeid++;
      edgeindex += directededges.size();
    }

    // Store the new tile
    GraphId tile(tileid, level, 0);
    tilebuilder.StoreTileData(info.graphreader_.GetTileHierarchy(), tile);
    LOG_DEBUG((boost::format("HierarchyBuilder created tile %1%: %2% bytes") %
         tile % tilebuilder.size()).str());

    // Increment tileid
    tileid++;
  }
}

// Add connections to the base tile. Rewrites the base tile with updated
// header information, node, and directed edge information.
void AddConnectionsToBaseTile(const uint32_t basetileid,
                              const std::vector<NodeConnection>& connections,
                              const TileHierarchy& tile_hierarchy) {
  // Read in existing tile
  uint8_t baselevel = connections[0].basenode.level();
  GraphId basetile(basetileid, baselevel, 0);
  GraphTileBuilder tilebuilder(tile_hierarchy, basetile, false);

  // TODO - anything index by directed edge index (e.g. Signs) needs
  // to be updated!

  // Get the header information and update counts and offsets. No new nodes
  // are added. Directed edge count is increased by size of the connection
  // list. The offsets to the edge info and text list are adjusted by the
  // size of the extra directed edges.

  // Copy existing header and update directed edge count and some offsets
  GraphTileHeader existinghdr = *(tilebuilder.header());
  GraphTileHeaderBuilder hdrbuilder =
        static_cast<GraphTileHeaderBuilder&>(existinghdr);
  hdrbuilder.set_directededgecount(
      existinghdr.directededgecount() + connections.size());
  std::size_t addedsize = connections.size() * sizeof(DirectedEdgeBuilder);
  hdrbuilder.set_edgeinfo_offset(existinghdr.edgeinfo_offset() + addedsize);
  hdrbuilder.set_textlist_offset(existinghdr.textlist_offset() + addedsize);

  // TODO - adjust these offsets if needed
  hdrbuilder.set_merlist_offset(existinghdr.merlist_offset());

  // Get the directed edge index of the first sign. If no signs are
  // present in this tile set a value > number of directed edges
  uint32_t nextsignidx = (tilebuilder.header()->signcount() > 0) ?
      tilebuilder.sign(0).edgeindex() : existinghdr.directededgecount() + 1;

  // Get the nodes. For any that have a connection add to the edge count
  // and increase the edge_index by (n = number of directed edges added so far)
  uint32_t n = 0;
  uint32_t signidx = 0;
  uint32_t signcount = existinghdr.signcount();
  uint32_t nextconnectionid = connections[0].basenode.id();
  std::vector<NodeInfoBuilder> nodes;
  std::vector<DirectedEdgeBuilder> directededges;
  std::vector<SignBuilder> signs;
  for (uint32_t id = 0; id < existinghdr.nodecount(); id++) {
    NodeInfoBuilder node = tilebuilder.node(id);

    // Add existing directed edges
    uint32_t idx = node.edge_index();
    for (uint32_t j = 0; j < node.edge_count(); j++) {
      bool has_sign = tilebuilder.directededge(idx).exitsign();
      directededges.emplace_back(std::move(tilebuilder.directededge(idx)));

      // Add any signs that use this idx - increment their index by the
      // number of added edges
      while (idx == nextsignidx && signidx < signcount) {
        if (!has_sign) {
          LOG_ERROR("Signs for this index but directededge says no sign");
        }
        SignBuilder sign = tilebuilder.sign(signidx);
        sign.set_edgeindex(idx + n);
        signs.emplace_back(std::move(sign));

        // Increment to the next sign and update nextsignidx
        signidx++;
        nextsignidx = (signidx >= signcount) ?
              0 : tilebuilder.sign(signidx).edgeindex();
      }

      // Increment index
      idx++;
    }

    // Update the edge index by n (# of new edges have been added)
    node.set_edge_index(node.edge_index() + n);

    // If a connection exists at this node, add it as well as a connection
    // directed edge (upward connection is last edge in list).
    if (id == nextconnectionid) {
      // Add 1 to the edge count from this node
      node.set_edge_count(node.edge_count() + 1);

      // Append a new directed edge that forms the connection.
      // TODO - what access do we allow on upward transitions?
      DirectedEdgeBuilder edgeconnection;
      edgeconnection.set_trans_up(true);
      edgeconnection.set_endnode(connections[n].newnode);
      edgeconnection.set_all_forward_access();
      directededges.emplace_back(std::move(edgeconnection));

      // Increment n and get the next base node Id that connects. Set next
      // connection id to 0 if we are at the end of the list
      n++;
      nextconnectionid =
          (n >= connections.size()) ? 0 : connections[n].basenode.id();
    }

    // Add the node to the list
    nodes.emplace_back(std::move(node));
  }

  if (connections.size() != n) {
    LOG_ERROR("Added " + std::to_string(n) + " directed edges. connections size = " +
              std::to_string(connections.size()));
  }
  if (signs.size() != hdrbuilder.signcount()) {
    LOG_ERROR("AddConnectionsToBaseTile: sign size = " +
              std::to_string(signs.size()) + " Header says: " +
              std::to_string(hdrbuilder.signcount()));
  }

  // Write the new file
  tilebuilder.Update(tile_hierarchy, hdrbuilder, nodes, directededges, signs);

  LOG_DEBUG((boost::format("HierarchyBuilder updated tile %1%: %2% bytes") %
      basetile % tilebuilder.size()).str());
}

// Connect nodes in the base level tiles to the new nodes in the new
// hierarchy level.
void ConnectBaseLevelToNewLevel(
    const TileHierarchy::TileLevel& base_level,
    const TileHierarchy::TileLevel& new_level, hierarchy_info& info) {
  // For each tile in the new level - form connections from the tiles
  // within the base level
  uint8_t level = new_level.level;
  uint32_t tileid = 0;
  for (const auto& newtile : info.tilednodes_) {
    if (newtile.size() != 0) {
      // Create lists of connections required from each base tile
      uint32_t id = 0;
      std::map<uint32_t, std::vector<NodeConnection>> connections;
      for (const auto& newnode : newtile) {
        // Add to the map of connections
        connections[newnode.basenode.tileid()].emplace_back(
              newnode.basenode, GraphId(tileid, level, id));
        id++;
      }

      // Iterate through each base tile and add connections
      for (auto& basetile : connections) {
        // Sort the connections by Id then add connections to the base tile
        std::sort(basetile.second.begin(), basetile.second.end());
        AddConnectionsToBaseTile(basetile.first, basetile.second, info.graphreader_.GetTileHierarchy());
      }
    }

    // Check if we need to clear the tile cache
    if(info.graphreader_.OverCommitted())
      info.graphreader_.Clear();

    // Increment tile Id
    tileid++;
  }
}

// Get the nodes that remain in the new level
void GetNodesInNewLevel(
    const TileHierarchy::TileLevel& base_level,
    const TileHierarchy::TileLevel& new_level, hierarchy_info& info) {
  // Iterate through all tiles in the lower level
  // TODO - can be concurrent if we divide by rows for example
  uint32_t ntiles = base_level.tiles.TileCount();
  uint32_t baselevel = (uint32_t) base_level.level;
  const GraphTile* tile = nullptr;
  for (uint32_t basetileid = 0; basetileid < ntiles; basetileid++) {
    // Check if we need to clear the tile cache
    if(info.graphreader_.OverCommitted())
      info.graphreader_.Clear();

    // Get the graph tile. Skip if no tile exists (common case)
    tile = info.graphreader_.GetGraphTile(GraphId(basetileid, baselevel, 0));
    if (tile == nullptr || tile->header()->nodecount() == 0) {
      continue;
    }

    // Iterate through the nodes. Add nodes to the new level when
    // best road class <= the new level classification cutoff
    uint32_t nodecount = tile->header()->nodecount();
    GraphId basenode(basetileid, baselevel, 0);
    const NodeInfo* nodeinfo = tile->node(basenode);
    for (uint32_t i = 0; i < nodecount; i++, nodeinfo++, basenode++) {
      if (nodeinfo->bestrc() <= new_level.importance) {
        // Test this node to see if it can be contracted (for adding shortcut
        // edges). Add the node to the new tile and add the mapping from base
        // level node to the new node
        uint32_t newtileid = new_level.tiles.TileId(nodeinfo->latlng());
        GraphId newnode(newtileid, new_level.level,
                        info.tilednodes_[newtileid].size());
        bool contract = CanContract(tile, nodeinfo, basenode, newnode,
                        new_level.importance, info);
        info.tilednodes_[newtileid].emplace_back(basenode, contract);
        info.nodemap_[basenode.value] = newnode;
      }
    }
  }
}

}

namespace valhalla {
namespace mjolnir {

// Build successive levels of the hierarchy, starting at the local
// base level. Each successive level of the hierarchy is based on
// and connected to the next. Also adds shortcut edges through nodes
// that only connect to 2 edges with compatible attributes and all other
// edges are on lower hierarchy level.
void HierarchyBuilder::Build(const boost::property_tree::ptree& pt) {

  //TODO: thread this. would need to make sure we dont make shortcuts
  //across tile boundaries so that we are only messing with one tile
  //in one thread at a time
  hierarchy_info info{0,0,{pt.get_child("mjolnir.hierarchy")}};
  const auto& tile_hierarchy = info.graphreader_.GetTileHierarchy();
  if (info.graphreader_.GetTileHierarchy().levels().size() < 2)
    throw std::runtime_error("Bad tile hierarchy - need 2 levels");

  // Crack open some elevation data if its there
  boost::optional<std::string> elevation = pt.get_optional<std::string>("additional_data.elevation");
  std::unique_ptr<const skadi::sample> sample;
  if(elevation)
    sample.reset(new skadi::sample(*elevation));

  auto base_level = tile_hierarchy.levels().rbegin();
  auto new_level = base_level;
  new_level++;
  for (; new_level != tile_hierarchy.levels().rend();
      base_level++, ++new_level) {
    LOG_INFO("Build Hierarchy Level " + new_level->second.name
              + " Base Level is " + base_level->second.name);

    // Clear the node map and contraction node map
    info.nodemap_.clear();
    info.contractions_.clear();

    // Size the vector for new tiles. Clear any nodes from these tiles
    info.tilednodes_.resize(new_level->second.tiles.TileCount());
    for (auto& tile : info.tilednodes_) {
      tile.clear();
    }

    // Get the nodes that exist in the new level
    GetNodesInNewLevel(base_level->second, new_level->second, info);
    LOG_DEBUG((boost::format("Can contract %1% nodes out of %2% nodes") % info.contractcount_ % info.nodemap_.size()).str());

    // Form all tiles in new level
    FormTilesInNewLevel(base_level->second, new_level->second, info, sample);
    LOG_DEBUG((boost::format("Created %1% shortcuts") % info.shortcutcount_).str());

    // Form connections (directed edges) in the base level tiles to
    // the new level. Note that the new tiles are created before adding
    // connections to base tiles. That way all access to old tiles is
    // complete and the base tiles can be updated.
    ConnectBaseLevelToNewLevel(base_level->second, new_level->second, info);
    LOG_INFO("Finished");
  }
}

}
}

