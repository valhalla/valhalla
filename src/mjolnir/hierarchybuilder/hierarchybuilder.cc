#include "hierarchybuilder.h"
#include "config.h"

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
      nodecounts_{},
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
  auto new_level  = base_level;
  new_level++;
  for ( ; new_level != tile_hierarchy_.levels().rend();
            base_level++, ++new_level) {
    std::cout << " Build Hierarchy Level " << new_level->second.name
              << " Base Level is " << base_level->second.name << std::endl;

    // Clear the node map and map of superseded edges
    nodemap_.clear();
    supersededmap_.clear();

    // Size the vector for new tiles. Clear any nodes from these tiles
    tilednodes_.resize(new_level->second.tiles.TileCount());
    for (auto& tile : tilednodes_) {
      tile.clear();
    }

    // Debug counts
    contractcount_ = 0;
     for (uint32_t i = 0; i < 16; i++)
       nodecounts_[i] = 0;

    // Get the nodes that exist in the new level
    GetNodesInNewLevel(base_level->second, new_level->second);
    std::cout << "Can contract " << contractcount_ << " nodes"
              << " Out of " << nodemap_.size() << " nodes" << std::endl;

    // Debug counts
    for (uint32_t i = 0; i < 16; i++) {
      if (nodecounts_[i] > 0) {
        std::cout << " Nodes with " << i << " edges: " <<
            nodecounts_[i] << std::endl;
      }
    }

    // Form all tiles in new level
    shortcutcount_ = 0;
    FormTilesInNewLevel(base_level->second, new_level->second);
    std::cout << "Created " << shortcutcount_ << " shortcuts" << std::endl;

    // Form connections (directed edges) in the base level tiles to
    // the new level. Note that the new tiles are created before adding
    // connections to base tiles. That way all access to old tiles is
    // complete and the base tiles can be updated.
    ConnectBaseLevelToNewLevel(base_level->second, new_level->second);
  }
  return true;
}

// Get the nodes that remain in the new level
bool HierarchyBuilder::GetNodesInNewLevel(
        const TileHierarchy::TileLevel& base_level,
        const TileHierarchy::TileLevel& new_level) {
  // Iterate through all tiles in the lower level
  // TODO - can be concurrent if we divide by rows for example
  uint32_t ntiles = base_level.tiles.TileCount();
  uint32_t baselevel = (uint32_t)base_level.level;
  GraphTile* graphtile = nullptr;
  for (uint32_t tileid = 0; tileid < ntiles; tileid++) {
    if (!graphreader_.DoesTileExist(GraphId(tileid, baselevel, 0))) {
      continue;
    }

    // Get the graphtile
    graphtile = graphreader_.GetGraphTile(GraphId(tileid, baselevel, 0));
    if (graphtile == nullptr) {
      // No tile - common case, just continue
      continue;
    }

    // Iterate through the nodes
    uint32_t nodecount = graphtile->header()->nodecount();
    GraphId basenode(tileid, baselevel, 0);
    const NodeInfo* nodeinfo = graphtile->node(basenode);
    for (uint32_t i = 0; i < nodecount; i++, nodeinfo++, basenode++) {
      // Skip any nodes with best road class > road class cutoff. They
      // will not be included in the new level
      if (nodeinfo->bestrc() > new_level.importance)
        continue;

      // This node remains on the new level. Test if it can be contracted
      // (for adding shortcut edges). Add the node to the new tile
      // and add the mapping from base level node to the new node
      uint32_t tileid = new_level.tiles.TileId(nodeinfo->latlng());
      uint32_t nodeid = tilednodes_[tileid].size();
      bool contract = CanContract(graphtile, nodeinfo, basenode,
                                  new_level.importance);
      tilednodes_[tileid].push_back(NewNode(basenode, contract));
      nodemap_[basenode.value()] =  GraphId(tileid, new_level.level, nodeid);
    }
  }
  return true;
}

// Test if the node is eligible to be contracted (part of a shortcut) in
// the new level.
bool HierarchyBuilder::CanContract(GraphTile* graphtile,
                                   const NodeInfo* nodeinfo,
                                   const GraphId& node,
                                   const RoadClass rcc)  {
  // TODO - should we consider contracting due to driveability? That is
  // allow contraction at nodes where only 2 driveable outbound edges exist?
  // This would allow contraction at "entrance" nodes on highways

  // Get the number of directed edges on this hierarchy
  std::vector<GraphId> edges;
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  const DirectedEdge* directededge = graphtile->directededge(
                nodeinfo->edge_index());
  for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
              i++, directededge++, edgeid++) {
    // Do not count any transition edges or shortcut edges
    if (directededge->importance() <= rcc &&
        !directededge->trans_down() && !directededge->shortcut()) {
      edges.push_back(edgeid);
    }
  }

  // Keep count of # of nodes with each edge count
  nodecounts_[edges.size()]++;

  // If n is not equal to 2 we cannot contract
  if (edges.size() != 2)
    return false;

  // If n == 2 we need to check attributes and names to see if we can create a
  // shortcut through this node
  const DirectedEdge* edge1 = graphtile->directededge(edges[0]);
  const DirectedEdge* edge2 = graphtile->directededge(edges[1]);

  // Make sure access matches. Need to consider opposite direction for one of
  // the edges since both edges are outbound from the node.
  if (edge1->forwardaccess() != edge2->reverseaccess() ||
      edge1->reverseaccess() != edge2->forwardaccess()) {
    return false;
  }

  // Importance (class), link, use, and attributes must also match
  if (edge1->importance() != edge2->importance() ||
      edge1->link()       != edge2->link() ||
      edge1->use()        != edge2->use() ||
      edge1->speed()      != edge2->speed() ||
      edge1->ferry()      != edge2->ferry() ||
      edge1->railferry()  != edge2->railferry() ||
      edge1->toll()       != edge2->toll() ||
      edge1->destonly()   != edge2->destonly() ||
      edge1->unpaved()    != edge2->unpaved() ||
      edge1->tunnel()     != edge2->tunnel() ||
      edge1->bridge()     != edge2->bridge() ||
      edge1->roundabout() != edge2->roundabout()) {
    return false;
  }

  // Names must match
  // TODO - this allows matches in any order. Do we need to maintain order?
  // TODO - should allow near matches?
  std::vector<std::string> edge1names = graphtile->GetNames(edge1->edgedataoffset());
  std::vector<std::string> edge2names = graphtile->GetNames(edge2->edgedataoffset());
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

  // Mark the 2 edges entering the node and 2 edges exiting the node
  // as "superseded"
  GraphId oppedge0 = GetOpposingEdge(node, edge1);
  GraphId oppedge1 = GetOpposingEdge(node, edge2);
  supersededmap_[edges[0].value()] = true;
  supersededmap_[edges[1].value()] = true;
  supersededmap_[oppedge0.value()] = true;
  supersededmap_[oppedge1.value()] = true;

  contractcount_++;
  return true;
}

// Get the GraphId of the opposing edge.
GraphId HierarchyBuilder::GetOpposingEdge(const GraphId& node,
                                          const DirectedEdge* edge) {
  // Get the tile at the end node
  GraphTile* tile = graphreader_.GetGraphTile(edge->endnode());
  const NodeInfo* nodeinfo = tile->node(edge->endnode().id());

  // Get the directed edges and return when the end node matches
  // the specified node and length matches
  GraphId edgeid(edge->endnode().tileid(), edge->endnode().level(),
                 nodeinfo->edge_index());
  const DirectedEdge* directededge =  tile->directededge(
              nodeinfo->edge_index());
  for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
              i++, directededge++, edgeid++) {
    if (directededge->endnode() == node &&
        directededge->length() == edge->length()) {
      return edgeid;
    }
  }
  std::cout << "Opposing directed edge not found!" << std::endl;
  return GraphId(0,0,0);
}

// Is the edge superseded (used in a shortcut edge)?
bool HierarchyBuilder::IsSuperseded(const baldr::GraphId& edge) const {
  const auto& s = supersededmap_.find(edge.value());
  return (s == supersededmap_.end()) ? false : true;
}

// Form tiles in the new level.
void HierarchyBuilder::FormTilesInNewLevel(
      const TileHierarchy::TileLevel& base_level,
      const TileHierarchy::TileLevel& new_level) {
  // Iterate through tiled nodes in the new level
  uint32_t tileid = 0;
  uint32_t nodeid = 0;
  uint32_t edgeid = 0;
  uint32_t edgecount = 0;
  uint32_t edgeindex = 0;
  uint32_t edge_info_offset;
  uint8_t level = new_level.level;
  RoadClass rcc = new_level.importance;
  for (const auto& newtile : tilednodes_) {
    // Skip if no new nodes
    if (newtile.size() == 0) {
      tileid++;
      continue;
    }

    // Create GraphTileBuilder for the new tile
    GraphTileBuilder tilebuilder(tile_hierarchy_, GraphId(tileid, level, 0));

    // Iterate through the NewNodes in the tile at the new level
    nodeid = 0;
    edgeindex = 0;
    GraphId nodea, nodeb;
    for (const auto& newnode : newtile) {
      // Get the node in the base level
      GraphTile* tile = graphreader_.GetGraphTile(newnode.basenode);

      // Copy node information
      nodea.Set(tileid, level, nodeid);
      NodeInfoBuilder node;
      const NodeInfo* oldnodeinfo = tile->node(newnode.basenode.id());
      node.set_latlng(oldnodeinfo->latlng());
      node.set_edge_index(edgeindex);
      node.set_bestrc(oldnodeinfo->bestrc());

      // Add shortcut edges first. If node is not contracted then there
      // may be shortcuts. Set the edgecount from this node.
      edgecount = 0;
      std::vector<DirectedEdgeBuilder> directededges;
      if (!newnode.contract) {
        edgecount = AddShortcutEdges(newnode, nodea, oldnodeinfo, tile,
                      rcc, tilebuilder, directededges);
      }

      // Iterate through directed edges of the base node to get remaining
      // directed edges (based on classification/importance cutoff)
      GraphId oldedgeid(newnode.basenode.tileid(), newnode.basenode.level(),
                        oldnodeinfo->edge_index());
      const DirectedEdge* directededge = tile->directededge(
                        oldnodeinfo->edge_index());
      for (uint32_t i = 0, n = oldnodeinfo->edge_count(); i < n;
                      i++, directededge++, oldedgeid++) {
        // Store the directed edge if less than the road class cutoff and
        // it is not a transition edge or shortcut in the base level
        if (directededge->importance() <= rcc &&
           !directededge->trans_down() && !directededge->shortcut()) {
          // Copy the directed edge information and update end node,
          // edge data offset, and opp_index
          DirectedEdge oldedge = *directededge;
          DirectedEdgeBuilder newedge =
              static_cast<DirectedEdgeBuilder&>(oldedge);

          // Set the end node for this edge
          nodeb = nodemap_[directededge->endnode().value()];
          newedge.set_endnode(nodeb);

          // TODO - how do we set the opposing index?
          newedge.set_opp_index(0);

          // Get edge info, shape, and names from the old tile and add
          // to the new. Use a value based on length to protect against
          // edges that have same end nodes but different lengths
          std::unique_ptr<const EdgeInfo> edgeinfo =
                tile->edgeinfo(directededge->edgedataoffset());
          std::vector<std::string> names = tile->GetNames(directededge->edgedataoffset());
          edgeid = static_cast<uint32_t>(directededge->length() * 100.0f);
          edge_info_offset = tilebuilder.AddEdgeInfo(edgeid, nodea,
                 nodeb, edgeinfo->shape(), names);
          newedge.set_edgedataoffset(edge_info_offset);

          // Add superseded flag if this edge was included in a shortcut
          if (IsSuperseded(oldedgeid)) {
            newedge.set_superseded(true);
          } else {
            // Unset the superseded flag (may have been set on prior level)
            newedge.set_superseded(false);
          }

          // Add directed edge
          directededges.emplace_back(std::move(newedge));
          edgecount++;
        }
      }

      // Add the downward transition edge
      DirectedEdgeBuilder downwardedge;
      downwardedge.set_endnode(newnode.basenode);
      downwardedge.set_trans_down(true);
      directededges.emplace_back(std::move(downwardedge));
      edgecount++;

      // Set the edge count for the new node
      node.set_edge_count(edgecount);

      // Add node and directed edge information to the tile
      tilebuilder.AddNodeAndDirectedEdges(node, directededges);

      // Increment node Id and edgeindex
      nodeid++;
      edgeindex += edgecount;
    }

    // Store the new tile
    tilebuilder.StoreTileData(tile_hierarchy_, GraphId(tileid, level, 0));

    // Increment tileid
    tileid++;
  }
}

// Add shortcut edges (if they should exist) from the specified node
uint32_t HierarchyBuilder::AddShortcutEdges(const NewNode& newnode,
              const GraphId& nodea, const NodeInfo* oldnodeinfo,
              GraphTile* tile, const RoadClass rcc,
              GraphTileBuilder& tilebuilder,
              std::vector<DirectedEdgeBuilder>& directededges) {
  // Iterate through directed edges of the base node
  uint32_t edgeid;
  uint32_t edge_info_offset;
  uint32_t edgecount = 0;
  float length = 0.0f;

  std::vector<PointLL> shape;
  const DirectedEdge* directededge = tile->directededge(
                    oldnodeinfo->edge_index());
  const DirectedEdge* connectededge;
  for (uint32_t i = 0, n = oldnodeinfo->edge_count(); i < n;
              i++, directededge++) {
    // Skip if >  road class cutoff or a transition edge or shortcut in
    // the base level. Note that only downward transitions exist at this
    // point (upward transitions are created later)
    if (directededge->importance() > rcc ||
        directededge->trans_down() || directededge->shortcut()) {
      continue;
    }

    // Get the end node and check if it is set for contraction
    GraphId priornode = newnode.basenode;
    GraphId nodeb = nodemap_[directededge->endnode().value()];
    if (tilednodes_[nodeb.tileid()][nodeb.id()].contract) {
      // Form a shortcut edge.
      DirectedEdge oldedge = *directededge;
      DirectedEdgeBuilder newedge =
                    static_cast<DirectedEdgeBuilder&>(oldedge);
      length = newedge.length();

      // Get the shape for this edge
      std::unique_ptr<const EdgeInfo> edgeinfo =
              tile->edgeinfo(directededge->edgedataoffset());
      shape = edgeinfo->shape();

      // Get names - they apply over all edges of the shortcut
      std::vector<std::string> names = tile->GetNames(directededge->edgedataoffset());

      // Follow until the end is not contracted
      while ((connectededge = GetConnectedEdge(nodeb, priornode,
                          rcc)) != nullptr) {
        length += ConnectEdges(priornode, connectededge, shape);
      }

      // Add the edge info
      edgeid = static_cast<uint32_t>(length * 100.0f);
      edge_info_offset = tilebuilder.AddEdgeInfo(edgeid, nodea,
                  nodeb, shape, names);
      newedge.set_edgedataoffset(edge_info_offset);

      // Update the length and end node
      newedge.set_length(length);
      newedge.set_endnode(nodeb);

      // Set this as a shortcut edge. Remove superseded flag that may
      // have been copied from base level directed edge
      newedge.set_shortcut(true);
      newedge.set_superseded(false);

      // Add directed edge
      directededges.emplace_back(std::move(newedge));
      edgecount++;
    }
  }
  shortcutcount_ += edgecount;
  return edgecount;
}

// Get the connected edge at the contracted node. Returns false if no
// connected node is found (finalizes the shortcut).
const DirectedEdge* HierarchyBuilder::GetConnectedEdge(GraphId& nodeb,
           GraphId& priornode, RoadClass rcc) {
  // Get the node in the base level
  GraphId basenode = tilednodes_[nodeb.tileid()][nodeb.id()].basenode;
  GraphTile* tile = graphreader_.GetGraphTile(basenode);
  const NodeInfo* oldnodeinfo = tile->node(basenode.id());

  // Iterate through directed edges. Contracted nodes should only have
  // 2 directed edges in the new level.
  GraphId endnode;
  const DirectedEdge* directededge = tile->directededge(
              oldnodeinfo->edge_index());
  for (uint32_t i = 0, n = oldnodeinfo->edge_count(); i < n;
              i++, directededge++) {
    // Skip if > road class cutoff, a shortcut edge, or transition edge,
    // or this edge goes back to the prior node
    // (i.e., Uturn)
    if (directededge->importance() > rcc ||
        directededge->shortcut() || directededge->trans_up() ||
        directededge->trans_down() ||
        directededge->endnode() == priornode) {
      continue;
    }

    // Get the end node and check if it is set for contraction
    endnode = nodemap_[directededge->endnode().value()];
    if (tilednodes_[endnode.tileid()][endnode.id()].contract) {
      // Update the priornode and the new end node and return
      // this directed edge
      priornode = basenode;
      nodeb     = endnode;
      return directededge;
    }
  }
  return nullptr;
}

// Connect 2 edges shape
float HierarchyBuilder::ConnectEdges(const GraphId& basenode,
                   const DirectedEdge* directededge,
                   std::vector<PointLL>& shape) {
  // Get the shape for this edge
  GraphTile* tile = graphreader_.GetGraphTile(basenode);
  std::unique_ptr<const EdgeInfo> edgeinfo =
          tile->edgeinfo(directededge->edgedataoffset());
  std::vector<PointLL> edgeshape = edgeinfo->shape();
  bool forward = (edgeshape.front() == shape.back());

  // Append the shape
  if (forward) {
    shape.insert(shape.end(), edgeshape.begin() + 1, edgeshape.end());
  } else {
    shape.insert(shape.end(), edgeshape.rbegin() + 1, edgeshape.rend());
  }

  // Return the length
  return directededge->length();
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
              NodeConnection(newnode.basenode, GraphId(tileid, level, id)));
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
void HierarchyBuilder::AddConnectionsToBaseTile(const uint32_t basetileid,
      const std::vector<NodeConnection>& connections) {
  // Read in existing tile
  uint8_t baselevel = connections[0].basenode.level();
  GraphTileBuilder tilebuilder(tile_hierarchy_, GraphId(basetileid, baselevel, 0));

  // Get the header information and update counts and offsets. No new nodes
  // are added. Directed edge count is increased by size of the connection
  // list. The offsets to the edge info and text list are adjusted by the
  // size of the extra directed edges.
  const GraphTileHeader* existinghdr = tilebuilder.header();
  GraphTileHeaderBuilder hdrbuilder;
  hdrbuilder.set_nodecount(existinghdr->nodecount());
  hdrbuilder.set_directededgecount(existinghdr->directededgecount() +
                                   connections.size());
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
      directededges.emplace_back(
            std::move(tilebuilder.directededge(idx++)));
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

      // Increment n and get the next base node Id that connects
      n++;
      nextconnectionid = connections[n].basenode.id();
    }

    // Add the node to the list
    nodes.emplace_back(std::move(node));
  }

  // Write the new file
  tilebuilder.Update(tile_hierarchy_, hdrbuilder, nodes, directededges);
}

}
}

