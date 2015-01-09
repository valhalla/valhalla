#include "hierarchybuilder.h"

#include "config.h"

#include "mjolnir/graphtilebuilder.h"

#include <ostream>
#include <set>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

HierarchyBuilder::HierarchyBuilder(const boost::property_tree::ptree& pt)
    : tile_hierarchy_(pt),
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

    // Size the vector for new tiles
    tilednodes_.resize(new_level->second.tiles.TileCount());

    // Build the next level
    std::cout << " Build Hierarchy Level " << new_level->second.name
              << " Base Level is " << base_level->second.name << std::endl;
    GetNodesInNewLevel(base_level->second, new_level->second);

    // Form tiles in new level
    FormTilesInNewLevel(base_level->second, new_level->second);

    // Form connections (directed edges) in the base level tiles to
    // the new level
    ConnectBaseLevelToNewLevel(base_level->second, new_level->second);

    // Iterate through tiles in the new level. Form connections from
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

std::cout << "Tile exists..." << tileid << " add nodes with class <= "
        << static_cast<uint32_t>(new_level.importance) << std::endl;

    // Iterate through the nodes
    uint32_t nodecount = graphtile->header()->nodecount();
    GraphId priornode(tileid, baselevel, 0);
    const NodeInfo* nodeinfo = graphtile->node(priornode);
    for (uint32_t i = 0; i < nodecount; i++, nodeinfo++, priornode++) {
      // Skip any nodes with best road class > road class cutoff. They
      // will not be included in this hierarchy
      if (nodeinfo->bestrc() > new_level.importance)
        continue;

      // This node remains on the new hierarchy - add it to the new tile
      // and add the mapping from the old tile to this new node
      AddNewNode(graphtile, nodeinfo, priornode, new_level);
    }
  }
  return true;
}

void HierarchyBuilder::AddNewNode(GraphTile* graphtile,
                                  const NodeInfo* nodeinfo,
                                  const GraphId& basenode,
                                  const TileHierarchy::TileLevel& new_level) {
  // TODO - could make it so there is an easy mapping from base tiles
  // to new tiles (could be no overlap of bounds)
  uint32_t tileid = new_level.tiles.TileId(nodeinfo->latlng());
  uint32_t nodeid = tilednodes_[tileid].size();
  bool contract = false; // CanContract(graphtile, nodeinfo, basenode); // TODO
  tilednodes_[tileid].push_back(NewNode(basenode, contract));

  // Create mapping from base level node to new node
  GraphId newnode(tileid, new_level.level, nodeid);
  nodemap_[basenode.value()] = newnode;
}

bool HierarchyBuilder::CanContract(GraphTile* graphtile,
                                   const NodeInfo* nodeinfo,
                                   const GraphId& node) const {
  // Get the number of directed edges  on this hierarchy
  std::vector<DirectedEdge*> edges;
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  const DirectedEdge* directededge = graphtile->directededge(nodeinfo->edge_index());
  for (unsigned int i = 0, n = nodeinfo->edge_count(); i < n;
              i++, directededge++, edgeid++) {
    // TODO - need access methods for DirectedEdge attributes!
//    if (directededge->roadclass() <= rcc_) {
//      edges.push_back(directededge);
//    }
  }

  // If n is not equal to 2 we cannot contract
  if (edges.size() != 2)
    return false;

  // If n == 2 we need to check attributes and names to see if we can create a
  // shortcut through this node
  return false;
}

void HierarchyBuilder::FormTilesInNewLevel(
      const TileHierarchy::TileLevel& base_level,
      const TileHierarchy::TileLevel& new_level) {
  // Iterate through tiled nodes in the new level
  uint32_t tileid = 0;
  uint32_t nodeid = 0;
  uint32_t edgecount = 0;
  uint32_t edgeindex = 0;
  uint32_t edge_info_offset;
  uint8_t level = new_level.level;
  std::vector<std::string> names;
  std::string basedir = tile_hierarchy_.tile_dir();
  for (const auto& newtile : tilednodes_) {
    // Skip if no new nodes
    if (newtile.size() == 0) {
      tileid++;
      continue;
    }

    // Create GraphTileBuilder for the new tile
    GraphTileBuilder tilebuilder(basedir, GraphId(tileid, level, 0));

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

      // Set edge count from this node to 0
      edgecount = 0;

      // TODO - add shortcut edges first
      // If node is not contracted then there may be shortcuts.
      if (!newnode.contract) {

      }

      // Iterate through directed edges of the base node to get remaining
      // directed edges (based on classification/importance cutoff)
      std::vector<DirectedEdgeBuilder> directededges;
      const DirectedEdge* directededge = tile->directededge(
                oldnodeinfo->edge_index());
      for (uint32_t i = 0, n = oldnodeinfo->edge_count(); i < n;
                      i++, directededge++) {
        // Store the directed edge if less than the road class cutoff
        if (directededge->importance() <= new_level.importance) {
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
          // to the new
          tile->GetNames(directededge->edgedataoffset(), names);
          const std::shared_ptr<EdgeInfo> edgeinfo =
                tile->edgeinfo(directededge->edgedataoffset());
          edge_info_offset = tilebuilder.AddEdgeInfo(0, nodea,
                 nodeb, edgeinfo->shape(), names);
          newedge.set_edgedataoffset(edge_info_offset);

          // TODO - need to add superseded flag if this edge was included in
          // a shortcut

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
      node.set_edge_count(edgecount);;

      // Add node and directed edge information to the tile
      tilebuilder.AddNodeAndDirectedEdges(node, directededges);

      // Increment node Id
      nodeid++;
    }

    // Store the new tile
    tilebuilder.StoreTileData(basedir, GraphId(tileid, level, 0));

    // Increment tileid
    tileid++;
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
    // Skip if no new nodes
    if (newtile.size() == 0) {
      tileid++;
      continue;
    }

    // Create lists of connections required from each base tile
    uint32_t id = 0;
    std::map<uint32_t, std::vector<NodeConnection>> connections;
    for (const auto& newnode : newtile) {
      GraphId newnode_id(tileid, level, id);

      // Add to the map of connections
      connections[newnode.basenode.tileid()].emplace_back(
            NodeConnection(newnode.basenode, newnode_id));
    }

    // TODO - need to create new tile before adding connections to
    // base tile. That way all access to old tile is complete and we
    // can build/update  base tile

    // Iterate through each base tile and add connections
    for (auto& basetile : connections) {
      // Sort the connections by Id then add connections to the base tile
      std::sort(basetile.second.begin(), basetile.second.end());
      AddConnectionsToBaseTile(basetile.first, basetile.second);
    }

    tileid++;
  }
}

void HierarchyBuilder::AddConnectionsToBaseTile(const uint32_t basetileid,
      const std::vector<NodeConnection>& connections) {
  // Read in existing tile
  std::string basedir = tile_hierarchy_.tile_dir();
  uint8_t baselevel = connections[0].basenode.level();
  GraphTileBuilder tilebuilder(basedir, GraphId(basetileid, baselevel, 0));

  // Get the header information and update counts and offsets
  const GraphTileHeader* existinghdr = tilebuilder.header();
  GraphTileHeaderBuilder hdrbuilder;

  // No new nodes are added
  hdrbuilder.set_nodecount(existinghdr->nodecount());

  // Directed edges are added (equal to size of connection list)
  hdrbuilder.set_directededgecount(existinghdr->directededgecount() +
                                   connections.size());

std::cout << "Add " << connections.size() << " connections to " <<
                  basetileid << " Current nodecount= " <<
                  existinghdr->nodecount() << " Current directed edge count= "
                  << existinghdr->directededgecount() << std::endl;

  // Added size (add to edge info and text list offsets)
  std::size_t addedsize = connections.size() * sizeof(DirectedEdgeBuilder);
  hdrbuilder.set_edgeinfo_offset(existinghdr->edgeinfo_offset() + addedsize);
  hdrbuilder.set_textlist_offset(existinghdr->textlist_offset() + addedsize);

  // Get the nodes. For any that have a connection add to the edge count
  // and increase the edge_index by (n-1)
  uint32_t n = 0;
  uint32_t nextconnectionid = connections[0].basenode.id();
  std::vector<NodeInfoBuilder> nodes;
  std::vector<DirectedEdgeBuilder> directededges;
  for (uint32_t id = 0; id < existinghdr->nodecount(); id++) {
    NodeInfoBuilder node = tilebuilder.node(id);

    // Add directed edges
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
      // TODO - do we need to set all access to true?
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

std::cout << "New: NodeCount= " << nodes.size() << " DirectedEdges= "
    << directededges.size() << std::endl;

  // Write the new file
  tilebuilder.Update(basedir, hdrbuilder, nodes, directededges);
}

}
}

