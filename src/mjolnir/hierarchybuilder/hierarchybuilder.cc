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
    NodeInfoBuilder& node = tilebuilder.node(id);

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

