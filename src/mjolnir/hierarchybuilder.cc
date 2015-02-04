#include "mjolnir/hierarchybuilder.h"
#include <valhalla/midgard/logging.h>
#include <boost/format.hpp>

#include <ostream>
#include <set>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// TODO - preprocess ramps to lower their importance. This would improve
// the ability to create shortcut edges along highways...

HierarchyBuilder::HierarchyBuilder(const boost::property_tree::ptree& pt)
    : contractcount_(0),
      shortcutcount_(0),
      tile_hierarchy_(pt),
      graphreader_(tile_hierarchy_) {

  // Make sure there are at least 2 levels!
  if (tile_hierarchy_.levels().size() < 2)
    throw std::runtime_error("Bad tile hierarchy - need 2 levels");
}

bool HierarchyBuilder::Build() {
  // Build successive levels of the hierarchy, starting at the local
  // base level. Each successive level of the hierarchy is based on
  // and connected to the next.
  auto base_level = tile_hierarchy_.levels().rbegin();
  auto new_level = base_level;
  new_level++;
  for (; new_level != tile_hierarchy_.levels().rend();
      base_level++, ++new_level) {
    LOG_INFO("Build Hierarchy Level " + new_level->second.name
              + " Base Level is " + base_level->second.name);

    // Clear the node map, contraction node map, and map of superseded edges
    nodemap_.clear();
    supersededmap_.clear();
    contractions_.clear();

    // Size the vector for new tiles. Clear any nodes from these tiles
    tilednodes_.resize(new_level->second.tiles.TileCount());
    for (auto& tile : tilednodes_) {
      tile.clear();
    }

    // Get the nodes that exist in the new level
    contractcount_ = 0;
    shortcutcount_ = 0;
    GetNodesInNewLevel(base_level->second, new_level->second);
    LOG_INFO((boost::format("Can contract %1% nodes out of %2% nodes") % contractcount_ % nodemap_.size()).str());

    // Form all tiles in new level
    FormTilesInNewLevel(base_level->second, new_level->second);
    LOG_INFO((boost::format("Created %1% shortcuts") % shortcutcount_ ).str());

    // Form connections (directed edges) in the base level tiles to
    // the new level. Note that the new tiles are created before adding
    // connections to base tiles. That way all access to old tiles is
    // complete and the base tiles can be updated.
    ConnectBaseLevelToNewLevel(base_level->second, new_level->second);
  }
  return true;
}

// Get the nodes that remain in the new level
void HierarchyBuilder::GetNodesInNewLevel(
    const TileHierarchy::TileLevel& base_level,
    const TileHierarchy::TileLevel& new_level) {
  // Iterate through all tiles in the lower level
  // TODO - can be concurrent if we divide by rows for example
  uint32_t ntiles = base_level.tiles.TileCount();
  uint32_t baselevel = (uint32_t) base_level.level;
  const GraphTile* tile = nullptr;
  for (uint32_t basetileid = 0; basetileid < ntiles; basetileid++) {
    // Get the graph tile. Skip if no tile exists (common case)
    tile = graphreader_.GetGraphTile(GraphId(basetileid, baselevel, 0));
    if (tile == nullptr) {
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
                        tilednodes_[newtileid].size());
        bool contract = CanContract(tile, nodeinfo, basenode, newnode,
                        new_level.importance);
        tilednodes_[newtileid].emplace_back(basenode, contract);
        nodemap_[basenode.value] = newnode;
      }
    }
  }
}

// Test if the node is eligible to be contracted (part of a shortcut) in
// the new level.
bool HierarchyBuilder::CanContract(const GraphTile* tile, const NodeInfo* nodeinfo,
                                   const GraphId& basenode,
                                   const GraphId& newnode,
                                   const RoadClass rcc) {
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
    if (directededge->importance() <= rcc && !directededge->trans_down()
        && !directededge->shortcut()) {
      edges.push_back(edgeid);
    }
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
  for (uint32_t i = 0; i < n; i++) {
    if (i != match.first && i != match.second) {
      if (tile->directededge(edges[i])->forwardaccess() & kAutoAccess)
        return false;
    }
  }

  // Mark the 2 edges entering the node (opposing) and 2 edges exiting
  // (the directed edges) the node as "superseded"
  const DirectedEdge* edge1 = tile->directededge(edges[match.first]);
  const DirectedEdge* edge2 = tile->directededge(edges[match.second]);
  GraphId oppedge1 = GetOpposingEdge(basenode, edge1);
  GraphId oppedge2 = GetOpposingEdge(basenode, edge2);
  supersededmap_[edges[match.first].value] = true;
  supersededmap_[edges[match.second].value] = true;
  supersededmap_[oppedge1.value] = true;
  supersededmap_[oppedge2.value] = true;

  // Store the pairs of base edges entering and exiting this node
  EdgePairs edgepairs;
  edgepairs.edge1 = std::make_pair(oppedge1, edges[match.second]);
  edgepairs.edge2 = std::make_pair(oppedge2, edges[match.first]);
  contractions_[newnode.value] = edgepairs;

  contractcount_++;
  return true;
}

bool HierarchyBuilder::EdgesMatch(const GraphTile* tile, const DirectedEdge* edge1,
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

  // Importance (class), link, use, and attributes must also match.
  // NOTE: might want "better" bridge attribution. Seems most overpasses
  // get marked as a bridge and lead to less shortcuts
  if (edge1->importance() != edge2->importance()
      || edge1->link() != edge2->link()
      || edge1->use() != edge2->use()
      || edge1->internal() != edge2->internal()
      || edge1->speed() != edge2->speed()
      || edge1->ferry() != edge2->ferry()
      || edge1->railferry() != edge2->railferry()
      || edge1->toll() != edge2->toll()
      || edge1->destonly() != edge2->destonly()
      || edge1->unpaved() != edge2->unpaved()
      || edge1->surface() != edge2->surface()
      || edge1->exit() != edge2->exit() ||
//      edge1->tunnel()     != edge2->tunnel() ||
//      edge1->bridge()     != edge2->bridge() ||
      edge1->roundabout() != edge2->roundabout()) {
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
GraphId HierarchyBuilder::GetOpposingEdge(const GraphId& node,
                                          const DirectedEdge* edge) {
  // Get the tile at the end node
  const GraphTile* tile = graphreader_.GetGraphTile(edge->endnode());
  const NodeInfo* nodeinfo = tile->node(edge->endnode().id());

  // Get the directed edges and return when the end node matches
  // the specified node and length matches
  GraphId edgeid(edge->endnode().tileid(), edge->endnode().level(),
                 nodeinfo->edge_index());
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
      i++, directededge++, edgeid++) {
    if (directededge->endnode() == node &&
        directededge->importance() == edge->importance() &&
        directededge->use() == edge->use() &&
        directededge->length() == edge->length()) {
      return edgeid;
    }
  }
  LOG_ERROR("Opposing directed edge not found!");
  return GraphId(0, 0, 0);
}

// Is the edge superseded (used in a shortcut edge)?
bool HierarchyBuilder::IsSuperseded(const baldr::GraphId& edge) const {
  const auto& s = supersededmap_.find(edge.value);
  return (s == supersededmap_.end()) ? false : true;
}

// Form tiles in the new level.
void HierarchyBuilder::FormTilesInNewLevel(
    const TileHierarchy::TileLevel& base_level,
    const TileHierarchy::TileLevel& new_level) {
  // Iterate through tiled nodes in the new level
  bool added = false;
  uint32_t tileid = 0;
  uint32_t nodeid = 0;
  uint32_t edgeindex = 0;
  uint32_t edge_info_offset;
  uint8_t level = new_level.level;
  RoadClass rcc = new_level.importance;
  for (const auto& newtile : tilednodes_) {
    // Skip if no nodes in the tile at the new level
    if (newtile.size() == 0) {
      tileid++;
      continue;
    }

    // Create GraphTileBuilder for the new tile
    GraphTileBuilder tilebuilder;

    // Iterate through the nodes in the tile at the new level
    nodeid = 0;
    edgeindex = 0;
    GraphId nodea, nodeb;
    for (const auto& newnode : newtile) {
      // Get the node in the base level
      const GraphTile* tile = graphreader_.GetGraphTile(newnode.basenode);

      // Copy node information
      nodea.Set(tileid, level, nodeid);
      NodeInfoBuilder node;
      const NodeInfo* baseni = tile->node(newnode.basenode.id());
      node.set_latlng(baseni->latlng());
      node.set_edge_index(edgeindex);
      node.set_bestrc(baseni->bestrc());

      // Add shortcut edges first
      std::vector<DirectedEdgeBuilder> directededges;
      AddShortcutEdges(newnode, nodea, baseni, tile, rcc, tilebuilder,
                         directededges);

      // Iterate through directed edges of the base node to get remaining
      // directed edges (based on classification/importance cutoff)
      GraphId oldedgeid(newnode.basenode.tileid(), newnode.basenode.level(),
                        baseni->edge_index());
      for (uint32_t i = 0, n = baseni->edge_count(); i < n; i++, oldedgeid++) {
        // Store the directed edge if less than the road class cutoff and
        // it is not a transition edge or shortcut in the base level
        const DirectedEdge* directededge = tile->directededge(oldedgeid);
        if (directededge->importance() <= rcc && !directededge->trans_down()
            && !directededge->shortcut()) {
          // Copy the directed edge information and update end node,
          // edge data offset, and opp_index
          DirectedEdge oldedge = *directededge;
          DirectedEdgeBuilder newedge =
              static_cast<DirectedEdgeBuilder&>(oldedge);

          // Set the end node for this edge. Opposing edge indexes
          // get set in graph optimizer so set to 0 here.
          nodeb = nodemap_[directededge->endnode().value];
          newedge.set_endnode(nodeb);
          newedge.set_opp_index(0);

          // Get edge info, shape, and names from the old tile and add
          // to the new. Use edge length to protect against
          // edges that have same end nodes but different lengths
          std::unique_ptr<const EdgeInfo> edgeinfo = tile->edgeinfo(
                              directededge->edgeinfo_offset());
          edge_info_offset = tilebuilder.AddEdgeInfo(directededge->length(),
                             nodea, nodeb, edgeinfo->shape(),
                             tile->GetNames(directededge->edgeinfo_offset()),
                             tile->GetExitSigns(directededge->edgeinfo_offset()),
                             added);
          newedge.set_edgeinfo_offset(edge_info_offset);

          // Set the superseded flag
          newedge.set_superseded(IsSuperseded(oldedgeid));

          // Add directed edge
          directededges.emplace_back(std::move(newedge));
        }
      }

      // Add the downward transition edge
      DirectedEdgeBuilder downwardedge;
      downwardedge.set_endnode(newnode.basenode);
      downwardedge.set_trans_down(true);
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
    GraphId basetile(tileid, level, 0);
    tilebuilder.StoreTileData(tile_hierarchy_, basetile);

    LOG_INFO((boost::format("HierarchBuilder created tile %1%: %2% bytes") %
         basetile % tilebuilder.size()).str());

    // Increment tileid
    tileid++;
  }
}

// Add shortcut edges (if they should exist) from the specified node
void HierarchyBuilder::AddShortcutEdges(
    const NewNode& newnode, const GraphId& nodea, const NodeInfo* baseni,
    const GraphTile* tile, const RoadClass rcc, GraphTileBuilder& tilebuilder,
    std::vector<DirectedEdgeBuilder>& directededges) {
  // Get the edge pairs for this node (if contracted)
  auto edgepairs = newnode.contract ?
      contractions_.find(nodea.value) : contractions_.end();

  // Iterate through directed edges of the base node
  GraphId base_edge_id(newnode.basenode.tileid(), newnode.basenode.level(),
                       baseni->edge_index());
  for (uint32_t i = 0, n = baseni->edge_count(); i < n; i++, base_edge_id++) {
    // Skip if > road class cutoff or a transition edge or shortcut in
    // the base level. Note that only downward transitions exist at this
    // point (upward transitions are created later).
    const DirectedEdge* directededge = tile->directededge(base_edge_id);
    if (directededge->importance() > rcc || directededge->trans_down()
        || directededge->shortcut()) {
      continue;
    }

    // Check edgepairs for this node. If this edge is in the pair of exiting
    // shortcut edges at this node we skip it
    if (edgepairs != contractions_.end()) {
      if (edgepairs->second.edge1.second == base_edge_id ||
          edgepairs->second.edge2.second == base_edge_id) {
        continue;
      }
    }

    // Get the end node and check if it is set for contraction and the edge
    // is set as a matching, entering edge of the contracted node. Cases like
    // entrance ramps can lead to a contracted node
    GraphId basenode = newnode.basenode;
    GraphId nodeb = nodemap_[directededge->endnode().value];
    if (tilednodes_[nodeb.tileid()][nodeb.id()].contract
        && IsEnteringEdgeOfContractedNode(nodeb, base_edge_id)) {

      // Form a shortcut edge.
      DirectedEdge oldedge = *directededge;
      DirectedEdgeBuilder newedge = static_cast<DirectedEdgeBuilder&>(oldedge);
      uint32_t length = newedge.length();

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
        shape.insert(shape.end(), edgeshape.rbegin() + 1, edgeshape.rend());
      }

      // Get names - they apply over all edges of the shortcut
      std::vector<std::string> names = tile->GetNames(
          directededge->edgeinfo_offset());
      std::vector<ExitSignInfo> exits = tile->GetExitSigns(
          directededge->edgeinfo_offset());

      // Connect while the node is marked as contracted. Use the edge pair
      // mapping
      GraphId next_edge_id = base_edge_id;
      while (tilednodes_[nodeb.tileid()][nodeb.id()].contract) {
        // Get base node and the contracted node
        basenode = tilednodes_[nodeb.tileid()][nodeb.id()].basenode;
        auto edgepairs = contractions_.find(nodeb.value);
        if (edgepairs == contractions_.end()) {
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
        length += ConnectEdges(basenode, next_edge_id, shape, nodeb);
      }

      // Add the edge info. Use length to match edge in case multiple edges
      // exist between the 2 nodes. Test whether this shape is forward or
      // reverse (in case an existing edge exists)
      bool added = true;
      uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(length, nodea, nodeb,
                                                          shape, names, exits,
                                                          added);
      newedge.set_edgeinfo_offset(edge_info_offset);

      // Set the forward flag on this directed edge. If a new edge was added
      // the direction is forward otherwise the prior edge was the one stored
      // in the forward direction
      newedge.set_forward(added);

      // Update the length and end node
      newedge.set_length(length);
      newedge.set_endnode(nodeb);

      // Set this as a shortcut edge. Remove superseded flag that may
      // have been copied from base level directed edge
      newedge.set_shortcut(true);
      newedge.set_superseded(false);

/**
if (nodea.level() == 0) {
  LOG_INFO((boost::format("Add shortcut from %1% LL %2%,%3% to %4%")
     % nodea % baseni->latlng().lat() % baseni->latlng().lng() % nodeb).str());
}
**/

      // Add directed edge
      directededges.emplace_back(std::move(newedge));
    }
  }
  shortcutcount_ += directededges.size();
}

// Connect 2 edges shape and update the next end node in the new level
uint32_t HierarchyBuilder::ConnectEdges(const GraphId& basenode,
                                     const GraphId& edgeid,
                                     std::vector<PointLL>& shape,
                                     GraphId& nodeb) {
  // Get the tile and directed edge
  const GraphTile* tile = graphreader_.GetGraphTile(basenode);
  const DirectedEdge* directededge = tile->directededge(edgeid);

  // Get the shape for this edge and append to the shortcut's shape
  std::vector<PointLL> edgeshape = tile->edgeinfo(
          directededge->edgeinfo_offset())->shape();
  if (directededge->forward()) {
    shape.insert(shape.end(), edgeshape.begin() + 1, edgeshape.end());
  } else {
    shape.insert(shape.end(), edgeshape.rbegin() + 1, edgeshape.rend());
  }

  // Update the end node and return the length
  nodeb = nodemap_[directededge->endnode().value];
  return directededge->length();
}

bool HierarchyBuilder::IsEnteringEdgeOfContractedNode(const GraphId& node,
                                                      const GraphId& edge) {
  auto edgepairs = contractions_.find(node.value);
  if (edgepairs == contractions_.end()) {
    LOG_WARN("No edge pairs found for contracted node");
    return false;
  } else {
    return (edgepairs->second.edge1.first == edge
        || edgepairs->second.edge2.first == edge);
  }
}

// Connect nodes in the base level tiles to the new nodes in the new
// hierarchy level.
void HierarchyBuilder::ConnectBaseLevelToNewLevel(
    const TileHierarchy::TileLevel& base_level,
    const TileHierarchy::TileLevel& new_level) {
  // For each tile in the new level - form connections from the tiles
  // within the base level
  uint8_t level = new_level.level;
  uint32_t tileid = 0;
  for (const auto& newtile : tilednodes_) {
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
        AddConnectionsToBaseTile(basetile.first, basetile.second);
      }
    }

    // Increment tile Id
    tileid++;
  }
}

// Add connections to the base tile. Rewrites the base tile with updated
// header information, node, and directed edge information.
void HierarchyBuilder::AddConnectionsToBaseTile(
    const uint32_t basetileid, const std::vector<NodeConnection>& connections) {
  // Read in existing tile
  uint8_t baselevel = connections[0].basenode.level();
  GraphId basetile(basetileid, baselevel, 0);
  GraphTileBuilder tilebuilder(tile_hierarchy_, basetile);

  // Get the header information and update counts and offsets. No new nodes
  // are added. Directed edge count is increased by size of the connection
  // list. The offsets to the edge info and text list are adjusted by the
  // size of the extra directed edges.
  const GraphTileHeader* existinghdr = tilebuilder.header();
  GraphTileHeaderBuilder hdrbuilder;
  hdrbuilder.set_nodecount(existinghdr->nodecount());
  hdrbuilder.set_directededgecount(
      existinghdr->directededgecount() + connections.size());
  std::size_t addedsize = connections.size() * sizeof(DirectedEdgeBuilder);
  hdrbuilder.set_edgeinfo_offset(existinghdr->edgeinfo_offset() + addedsize);
  hdrbuilder.set_textlist_offset(existinghdr->textlist_offset() + addedsize);

  // Get the nodes. For any that have a connection add to the edge count
  // and increase the edge_index by (n = number of directed edges added so far)
  uint32_t n = 0;
  uint32_t nextconnectionid = connections[0].basenode.id();
  std::vector<NodeInfoBuilder> nodes;
  std::vector<DirectedEdgeBuilder> directededges;
  for (uint32_t id = 0; id < existinghdr->nodecount(); id++) {
    NodeInfoBuilder node = tilebuilder.node(id);

    // Add existing directed edges
    uint32_t idx = node.edge_index();
    for (uint32_t n = 0; n < node.edge_count(); n++) {
      directededges.emplace_back(std::move(tilebuilder.directededge(idx++)));
    }

    // Update the edge index by n (# of new edges have been added)
    node.set_edge_index(node.edge_index() + n);

    // If a connection exists at this node, add it as well as a connection
    // directed edge (upward connection is last edge in list).
    if (id == nextconnectionid) {
      // Add 1 to the edge count from this node
      node.set_edge_count(node.edge_count() + 1);

      // Append a new directed edge that forms the connection.
      // TODO - do we need to set access or any other attributes?
      DirectedEdgeBuilder edgeconnection;
      edgeconnection.set_trans_up(true);
      edgeconnection.set_endnode(connections[n].newnode);
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

  // Write the new file
  tilebuilder.Update(tile_hierarchy_, hdrbuilder, nodes, directededges);

  LOG_INFO((boost::format("HierarchBuilder updated tile %1%: %2% bytes") %
      basetile % tilebuilder.size()).str());
}

}
}

