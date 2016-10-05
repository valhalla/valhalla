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
#include <valhalla/midgard/encoded.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphreader.h>

#include <boost/format.hpp>
#include <ostream>
#include <set>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

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

struct hierarchy_info {
  GraphReader graphreader_;
  std::vector<std::vector<GraphId> > tilednodes_;
  std::unordered_map<uint64_t, GraphId> nodemap_;
};

// Form tiles in the new level.
void FormTilesInNewLevel(const TileHierarchy::TileLevel& base_level,
    const TileHierarchy::TileLevel& new_level, hierarchy_info& info) {
  // Iterate through tiled nodes in the new level
  bool added = false;
  uint32_t tileid = 0;
  uint32_t nodeid = 0;
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
    if (info.graphreader_.OverCommitted()) {
      info.graphreader_.Clear();
    }

    // Create GraphTileBuilder for the new tile
    GraphId tile(tileid, level, 0);
    GraphTileBuilder tilebuilder(info.graphreader_.GetTileHierarchy(), tile, false);

    //Creating a dummy admin at index 0.  Used if admins are not used/created.
    tilebuilder.AddAdmin("None","None","","");

    // Iterate through the nodes in the tile at the new level
    nodeid = 0;
    GraphId nodea, nodeb;
    for (const auto& newnode : newtile) {
      // Get the node in the base level
      const GraphTile* tile = info.graphreader_.GetGraphTile(newnode);

      // Copy node information
      nodea.Set(tileid, level, nodeid);
      NodeInfo baseni = *(tile->node(newnode.id()));
      tilebuilder.nodes().push_back(baseni);
      const auto& admin = tile->admininfo(baseni.admin_index());

      NodeInfo& node = tilebuilder.nodes().back();
      node.set_edge_index(tilebuilder.directededges().size());
      node.set_timezone(baseni.timezone());
      node.set_admin_index(tilebuilder.AddAdmin(admin.country_text(), admin.state_text(),
                                                admin.country_iso(), admin.state_iso()));

      // Edge count
      size_t edge_count = tilebuilder.directededges().size();

      // Iterate through directed edges of the base node to get remaining
      // directed edges (based on classification/importance cutoff)
      GraphId oldedgeid(newnode.tileid(), newnode.level(), baseni.edge_index());
      for (uint32_t i = 0, n = baseni.edge_count(); i < n; i++, oldedgeid++) {
        // Store the directed edge if less than the road class cutoff and
        // it is not a transition edge
        const DirectedEdge* directededge = tile->directededge(oldedgeid);
        if (directededge->classification() <= rcc && !directededge->trans_down()) {
          // Copy the directed edge information and update end node,
          // edge data offset, and opp_index
          DirectedEdge newedge = *directededge;

          // Set the end node for this edge. Opposing edge indexes
          // get set in graph optimizer so set to 0 here.
          nodeb = info.nodemap_[directededge->endnode().value];
          newedge.set_endnode(nodeb);
          newedge.set_opp_index(0);

          // Get signs from the base directed edge
          if (directededge->exitsign()) {
            std::vector<SignInfo> signs = tile->GetSigns(oldedgeid.id());
            if (signs.size() == 0) {
              LOG_ERROR("Base edge should have signs, but none found");
            }
            tilebuilder.AddSigns(tilebuilder.directededges().size(), signs);
          }

          // Get access restrictions from the base directed edge. Add these to
          // the list of access restrictions in the new tile. Update the
          // edge index in the restriction to be the current directed edge Id
          if (directededge->access_restriction()) {
            auto restrictions = tile->GetAccessRestrictions(oldedgeid.id(), kAllAccess);
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
          edge_info_offset = tilebuilder.AddEdgeInfo(directededge->length(),
                             nodea, nodeb, edgeinfo.wayid(), edgeinfo.shape(),
                             tile->GetNames(directededge->edgeinfo_offset()),
                             added);
          newedge.set_edgeinfo_offset(edge_info_offset);

          // Add directed edge
          tilebuilder.directededges().emplace_back(std::move(newedge));
        }
      }

      // Add the downward transition edge.
      // TODO - what access for downward transitions
      DirectedEdge downwardedge;
      downwardedge.set_endnode(newnode);
      downwardedge.set_trans_down(true);
      downwardedge.set_all_forward_access();
      tilebuilder.directededges().emplace_back(std::move(downwardedge));

      // Set the edge count for the new node
      node.set_edge_count(tilebuilder.directededges().size() - edge_count);

      // Increment node Id and edgeindex
      nodeid++;
    }

    // Store the new tile
    tilebuilder.StoreTileData();
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
  GraphTileHeader hdr = existinghdr;
  hdr.set_directededgecount( existinghdr.directededgecount() + connections.size());
  std::size_t addedsize = connections.size() * sizeof(DirectedEdge);
  hdr.set_edgeinfo_offset(existinghdr.edgeinfo_offset() + addedsize);
  hdr.set_textlist_offset(existinghdr.textlist_offset() + addedsize);

  // TODO - adjust these offsets if needed
  hdr.set_complex_restriction_offset(existinghdr.complex_restriction_offset());

  // Get the directed edge index of the first sign. If no signs are
  // present in this tile set a value > number of directed edges
  uint32_t signidx = 0;
  uint32_t nextsignidx = (tilebuilder.header()->signcount() > 0) ?
      tilebuilder.sign(0).edgeindex() : existinghdr.directededgecount() + 1;
  uint32_t signcount = existinghdr.signcount();

  // Get the directed edge index of the first access restriction.
  uint32_t residx = 0;
  uint32_t nextresidx = (tilebuilder.header()->access_restriction_count() > 0) ?
      tilebuilder.accessrestriction(0).edgeindex() : existinghdr.directededgecount() + 1;
  uint32_t rescount = existinghdr.access_restriction_count();

  // Get the nodes. For any that have a connection add to the edge count
  // and increase the edge_index by (n = number of directed edges added so far)
  uint32_t n = 0;
  uint32_t nextconnectionid = connections[0].basenode.id();
  std::vector<NodeInfo> nodes;
  std::vector<DirectedEdge> directededges;
  std::vector<Sign> signs;
  std::vector<AccessRestriction> restrictions;
  for (uint32_t id = 0; id < existinghdr.nodecount(); id++) {
    NodeInfo node = tilebuilder.node(id);

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
        Sign sign = tilebuilder.sign(signidx);
        sign.set_edgeindex(idx + n);
        signs.emplace_back(std::move(sign));

        // Increment to the next sign and update nextsignidx
        signidx++;
        nextsignidx = (signidx >= signcount) ?
              0 : tilebuilder.sign(signidx).edgeindex();
      }

      // Add any restrictions that use this idx - increment their index by the
      // number of added edges
      while (idx == nextresidx && residx < rescount) {
        AccessRestriction res = tilebuilder.accessrestriction(residx);
        res.set_edgeindex(idx + n);
        restrictions.emplace_back(std::move(res));

        // Increment to the next restriction and update nextresidx
        residx++;
        nextresidx = (residx >= rescount) ?
              0 : tilebuilder.accessrestriction(residx).edgeindex();
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
      DirectedEdge edgeconnection;
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
  if (signs.size() != hdr.signcount()) {
    LOG_ERROR("AddConnectionsToBaseTile: sign size = " +
              std::to_string(signs.size()) + " Header says: " +
              std::to_string(hdr.signcount()));
  }
  if (restrictions.size() != hdr.access_restriction_count()) {
      LOG_ERROR("AddConnectionsToBaseTile: restriction size = " +
                std::to_string(restrictions.size()) + " Header says: " +
                std::to_string(hdr.access_restriction_count()));
  }

  // Write the new file
  tilebuilder.Update(hdr, nodes, directededges, signs, restrictions);

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
        connections[newnode.tileid()].emplace_back(
              newnode, GraphId(tileid, level, id));
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
        // Add the node to the new tile and add the mapping from base
        // level node to the new node
        uint32_t newtileid = new_level.tiles.TileId(nodeinfo->latlng());
        GraphId newnode(newtileid, new_level.level,
                        info.tilednodes_[newtileid].size());
        info.tilednodes_[newtileid].emplace_back(basenode);
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
// and connected to the next.
void HierarchyBuilder::Build(const boost::property_tree::ptree& pt) {

  // TODO: thread this. Might be more possible now that we don't create
  // shortcuts in the HierarchyBuilder

  // Construct GraphReader
  hierarchy_info info{{pt.get_child("mjolnir")}};
  const auto& tile_hierarchy = info.graphreader_.GetTileHierarchy();
  if (info.graphreader_.GetTileHierarchy().levels().size() < 2) {
    throw std::runtime_error("Bad tile hierarchy - need 2 levels");
  }

  // Iterate through the levels
  auto base_level = tile_hierarchy.levels().rbegin();
  auto new_level = base_level;
  new_level++;
  for (; new_level != tile_hierarchy.levels().rend();
          base_level++, ++new_level) {
    LOG_INFO("Build Hierarchy Level " + new_level->second.name
              + " Base Level is " + base_level->second.name);

    // Clear the node map
    info.nodemap_.clear();

    // Size the vector for new tiles. Clear any nodes from these tiles
    info.tilednodes_.resize(new_level->second.tiles.TileCount());
    for (auto& tile : info.tilednodes_) {
      tile.clear();
    }

    // Get the nodes that exist in the new level
    GetNodesInNewLevel(base_level->second, new_level->second, info);

    // Form all tiles in new level
    FormTilesInNewLevel(base_level->second, new_level->second, info);

    // Form connections (directed edges) in the base level tiles to
    // the new level. Note that the new tiles are created before adding
    // connections to base tiles. That way all access to old tiles is
    // complete and the base tiles can be updated.
    ConnectBaseLevelToNewLevel(base_level->second, new_level->second, info);
  }
}

}
}
