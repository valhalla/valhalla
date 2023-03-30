#include "mjolnir/hierarchybuilder.h"
#include "mjolnir/graphtilebuilder.h"

#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>

#include <iostream>
#include <map>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/sequence.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

// Structure to associate old nodes to new nodes. Stored in a sequence so
// this can work on lower memory computers. Note that an original node can
// associate to multiple nodes on different hierarchy levels. If a node does
// not exist on a level, the associated node will be invalid.
struct OldToNewNodes {
  GraphId node_id;       // Old node
  GraphId highway_node;  // New, associated node on highway level
  GraphId arterial_node; // New, associated node on arterial level
  GraphId local_node;    // New, associated node on local level
  uint32_t density;      // Density at the node (for edge density)

  OldToNewNodes(const GraphId& node,
                const GraphId& highway,
                const GraphId& arterial,
                const GraphId& local,
                const uint32_t d)
      : node_id(node), highway_node(highway), arterial_node(arterial), local_node(local), density(d) {
  }
};

// Add a downward transition edge if the node is valid.
bool AddDownwardTransition(const GraphId& node, GraphTileBuilder* tilebuilder) {
  if (node.Is_Valid()) {
    tilebuilder->transitions().emplace_back(node, false);
    return true;
  } else {
    return false;
  }
}

// Add an upward transition edge if the node is valid.
bool AddUpwardTransition(const GraphId& node, GraphTileBuilder* tilebuilder) {
  if (node.Is_Valid()) {
    tilebuilder->transitions().emplace_back(node, true);
    return true;
  } else {
    return false;
  }
}

void SortSequences(const std::string& new_to_old_file, const std::string& old_to_new_file) {
  // Sort the new nodes. Sort so highway level is first
  sequence<std::pair<GraphId, GraphId>> new_to_old(new_to_old_file, false);
  new_to_old.sort([](const std::pair<GraphId, GraphId>& a, const std::pair<GraphId, GraphId>& b) {
    if (a.first.level() == b.first.level()) {
      if (a.first.tileid() == b.first.tileid()) {
        return a.first.id() < b.first.id();
      }
      return a.first.tileid() < b.first.tileid();
    }
    return a.first.level() < b.first.level();
  });

  // Sort old to new by node Id
  sequence<OldToNewNodes> old_to_new(old_to_new_file, false);
  old_to_new.sort(
      [](const OldToNewNodes& a, const OldToNewNodes& b) { return a.node_id < b.node_id; });
}

// Convenience method to find the node association.
OldToNewNodes find_nodes(sequence<OldToNewNodes>& old_to_new, const GraphId& node) {
  GraphId dmy;
  OldToNewNodes target(node, dmy, dmy, dmy, 0);
  auto iter = old_to_new.find(target, [](const OldToNewNodes& a, const OldToNewNodes& b) {
    return a.node_id < b.node_id;
  });
  if (iter == old_to_new.end()) {
    throw std::runtime_error("Didn't find node!");
  } else {
    return *iter;
  }
}

/**
 * Is there an opposing edge with matching edgeinfo offset. The end node of the directed edge
 * must be in the same tile as the directed edge.
 * @param  tile          Graph tile of the edge
 * @param  directededge  Directed edge to match.
 */
bool OpposingEdgeInfoMatches(const graph_tile_ptr& tile, const DirectedEdge* edge) {
  // Get the nodeinfo at the end of the edge. Iterate through the directed edges and return
  // true if a matching edgeinfo offset if found.
  const NodeInfo* nodeinfo = tile->node(edge->endnode().id());
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++) {
    // Return true if the edge info matches (same name, shape, etc.)
    if (directededge->edgeinfo_offset() == edge->edgeinfo_offset()) {
      return true;
    }
  }
  return false;
}

// Form tiles in the new level.
void FormTilesInNewLevel(GraphReader& reader,
                         const std::string& new_to_old_file,
                         const std::string& old_to_new_file) {
  // Use the sequence that associate new nodes to old nodes
  sequence<std::pair<GraphId, GraphId>> new_to_old(new_to_old_file, false);

  // Use the sorted sequence that associates old nodes to new nodes
  sequence<OldToNewNodes> old_to_new(old_to_new_file, false);

  // lambda to indicate whether a directed edge should be included
  auto include_edge = [&old_to_new](const DirectedEdge* directededge, const GraphId& base_node,
                                    const uint8_t current_level) {
    if (directededge->use() == Use::kTransitConnection ||
        directededge->use() == Use::kEgressConnection ||
        directededge->use() == Use::kPlatformConnection) {
      // Transit connection edges should live on the lowest class level
      // where a new node exists
      auto f = find_nodes(old_to_new, base_node);
      uint8_t lowest_level;
      if (f.local_node.Is_Valid())
        lowest_level = 2;
      else if (f.arterial_node.Is_Valid())
        lowest_level = 1;
      else if (f.highway_node.Is_Valid())
        lowest_level = 0;
      else
        throw std::logic_error("Could not find valid node level");
      return (lowest_level == current_level);
    } else if (directededge->bss_connection()) {
      // Despite the road class, Bike Share Stations' connections are always at local level
      return (2 == current_level);
    } else {
      return (TileHierarchy::get_level(directededge->classification()) == current_level);
    }
  };

  // Iterate through the new nodes. They have been sorted by level so that
  // highway level is done first.
  reader.Clear();
  bool added = false;
  uint8_t current_level = std::numeric_limits<uint8_t>::max();
  GraphId tile_id;
  std::hash<std::string> hasher;
  PointLL base_ll;
  GraphTileBuilder* tilebuilder = nullptr;
  for (auto new_node = new_to_old.begin(); new_node != new_to_old.end(); new_node++) {
    // Get the node - check if a new tile
    GraphId nodea = (*new_node).first;
    if (nodea.Tile_Base() != tile_id) {
      // Store the prior tile
      if (tilebuilder != nullptr) {
        tilebuilder->StoreTileData();
        delete tilebuilder;
      }

      // New tilebuilder for the next tile. Update current level.
      tile_id = nodea.Tile_Base();
      tilebuilder = new GraphTileBuilder(reader.tile_dir(), tile_id, false);
      current_level = nodea.level();

      // Set the base ll for this tile
      base_ll = TileHierarchy::get_tiling(current_level).Base(tile_id.tileid());
      tilebuilder->header_builder().set_base_ll(base_ll);

      // Check if we need to clear the base/local tile cache
      if (reader.OverCommitted()) {
        reader.Trim();
      }
    }

    // Get the node in the base level
    GraphId base_node = (*new_node).second;
    graph_tile_ptr tile = reader.GetGraphTile(base_node);
    if (tile == nullptr) {
      LOG_ERROR("Base tile is null? ");
      continue;
    }

    // Copy the data version
    tilebuilder->header_builder().set_dataset_id(tile->header()->dataset_id());

    // Copy node information and set the node lat,lon offsets within the new tile
    NodeInfo baseni = *(tile->node(base_node.id()));
    tilebuilder->nodes().push_back(baseni);
    const auto& admin = tile->admininfo(baseni.admin_index());
    NodeInfo& node = tilebuilder->nodes().back();
    node.set_latlng(base_ll, baseni.latlng(tile->header()->base_ll()));
    node.set_edge_index(tilebuilder->directededges().size());
    node.set_timezone(baseni.timezone());
    node.set_admin_index(tilebuilder->AddAdmin(admin.country_text(), admin.state_text(),
                                               admin.country_iso(), admin.state_iso()));

    // Update node LL based on tile base
    // Density at this node
    uint32_t density1 = baseni.density();

    // Current edge count
    size_t edge_count = tilebuilder->directededges().size();

    // Iterate through directed edges of the base node to get remaining
    // directed edges (based on classification/importance cutoff)
    GraphId base_edge_id(base_node.tileid(), base_node.level(), baseni.edge_index());
    for (uint32_t i = 0; i < baseni.edge_count(); i++, ++base_edge_id) {
      // Check if the directed edge should exist on this level
      const DirectedEdge* directededge = tile->directededge(base_edge_id);
      if (!include_edge(directededge, base_node, current_level)) {
        continue;
      }

      // Copy the directed edge information
      DirectedEdge newedge = *directededge;

      // Set the end node for this edge. Transit connection edges
      // remain connected to the same node on the transit level.
      // Need to set nodeb for use in AddEdgeInfo
      uint32_t density2 = 32;
      GraphId nodeb;
      if (directededge->use() == Use::kTransitConnection ||
          directededge->use() == Use::kEgressConnection ||
          directededge->use() == Use::kPlatformConnection) {
        nodeb = directededge->endnode();
      } else {
        auto new_nodes = find_nodes(old_to_new, directededge->endnode());
        if (current_level == 0) {
          nodeb = new_nodes.highway_node;
        } else if (current_level == 1) {
          nodeb = new_nodes.arterial_node;
        } else {
          nodeb = new_nodes.local_node;
        }
        density2 = new_nodes.density;
      }
      if (!nodeb.Is_Valid()) {
        LOG_ERROR("Invalid end node - not found in old_to_new map");
      }
      newedge.set_endnode(nodeb);

      // Set the edge density  to the average of the relative density at the
      // end nodes.
      uint32_t edge_density = (density2 == 32) ? density1 : (density1 + density2) / 2;
      newedge.set_density(edge_density);

      // Set opposing edge indexes to 0 (gets set in graph validator).
      newedge.set_opp_index(0);

      // Get signs from the base directed edge
      if (directededge->sign()) {
        std::vector<SignInfo> signs = tile->GetSigns(base_edge_id.id());
        if (signs.size() == 0) {
          LOG_ERROR("Base edge should have signs, but none found");
        }
        tilebuilder->AddSigns(tilebuilder->directededges().size(), signs);
      }

      // Get turn lanes from the base directed edge
      if (directededge->turnlanes()) {
        uint32_t offset = tile->turnlanes_offset(base_edge_id.id());
        tilebuilder->AddTurnLanes(tilebuilder->directededges().size(), tile->GetName(offset));
      }

      // Get access restrictions from the base directed edge. Add these to
      // the list of access restrictions in the new tile. Update the
      // edge index in the restriction to be the current directed edge Id
      if (directededge->access_restriction()) {
        auto restrictions = tile->GetAccessRestrictions(base_edge_id.id(), kAllAccess);
        for (const auto& res : restrictions) {
          tilebuilder->AddAccessRestriction(AccessRestriction(tilebuilder->directededges().size(),
                                                              res.type(), res.modes(), res.value()));
        }
      }

      // Copy lane connectivity
      if (directededge->laneconnectivity()) {
        auto laneconnectivity = tile->GetLaneConnectivity(base_edge_id.id());
        if (laneconnectivity.size() == 0) {
          LOG_ERROR("Base edge should have lane connectivity, but none found");
        }
        for (auto& lc : laneconnectivity) {
          lc.set_to(tilebuilder->directededges().size());
        }
        tilebuilder->AddLaneConnectivity(laneconnectivity);
      }

      // Do we need to force adding edgeinfo (opposing edge could have diff names)?
      // If end node is in the same tile and there is no opposing edge with matching
      // edge_info_offset).
      bool diff_names = directededge->endnode().tile_value() == base_edge_id.tile_value() &&
                        !OpposingEdgeInfoMatches(tile, directededge);

      // Get edge info, shape, and names from the old tile and add to the
      // new. Cannot use edge info offset since edges in arterial and
      // highway hierarchy can cross base tiles! Use a hash based on the
      // encoded shape plus way Id.
      auto edgeinfo = tile->edgeinfo(directededge);
      std::string encoded_shape = edgeinfo.encoded_shape();
      uint32_t w = hasher(encoded_shape + std::to_string(edgeinfo.wayid()));
      uint32_t edge_info_offset =
          tilebuilder->AddEdgeInfo(w, nodea, nodeb, edgeinfo.wayid(), edgeinfo.mean_elevation(),
                                   edgeinfo.bike_network(), edgeinfo.speed_limit(), encoded_shape,
                                   edgeinfo.GetNames(), edgeinfo.GetTaggedValues(),
                                   edgeinfo.GetTaggedValues(true), edgeinfo.GetTypes(), added,
                                   diff_names);

      newedge.set_edgeinfo_offset(edge_info_offset);

      // Add directed edge
      tilebuilder->directededges().emplace_back(std::move(newedge));
    }

    // Add node transitions
    uint32_t index = tilebuilder->transitions().size();
    auto new_nodes = find_nodes(old_to_new, base_node);
    if (current_level == 0) {
      AddDownwardTransition(new_nodes.arterial_node, tilebuilder);
      AddDownwardTransition(new_nodes.local_node, tilebuilder);
    } else if (current_level == 1) {
      AddUpwardTransition(new_nodes.highway_node, tilebuilder);
      AddDownwardTransition(new_nodes.local_node, tilebuilder);
    } else if (current_level == 2) {
      AddUpwardTransition(new_nodes.highway_node, tilebuilder);
      AddUpwardTransition(new_nodes.arterial_node, tilebuilder);
    } else {
      throw std::logic_error("current_level was never set");
    }

    // Set the node transition count and index
    uint32_t count = tilebuilder->transitions().size() - index;
    if (count > 0) {
      node.set_transition_count(count);
      node.set_transition_index(index);
    }

    // Set the edge count for the new node
    node.set_edge_count(tilebuilder->directededges().size() - edge_count);

    // Get named signs from the base node
    if (baseni.named_intersection()) {
      std::vector<SignInfo> signs = tile->GetSigns(base_node.id(), true);
      if (signs.size() == 0) {
        LOG_ERROR("Base node should have signs, but none found");
      }
      node.set_named_intersection(true);
      tilebuilder->AddSigns(tilebuilder->nodes().size() - 1, signs);
    }
  }

  // Delete the tile builder
  if (tilebuilder != nullptr) {
    tilebuilder->StoreTileData();
    delete tilebuilder;
  }
}

/**
 * Create node associations between "new" nodes placed into respective
 * hierarchy levels and the existing nodes on the base/local level. The
 * associations go both ways: from the "old" nodes on the base/local level
 * to new nodes (using a mapping in memory) and from new nodes to old nodes
 * using a sequence (file).
 */
void CreateNodeAssociations(GraphReader& reader,
                            const std::string& new_to_old_file,
                            const std::string& old_to_new_file) {
  // Map of tiles vs. count of nodes. Used to construct new node Ids.
  std::unordered_map<GraphId, uint32_t> new_nodes;

  // lambda to get the next "new" node Id in a given tile
  auto get_new_node = [&new_nodes](const GraphId& tile) -> GraphId {
    auto itr = new_nodes.find(tile);
    if (itr == new_nodes.end()) {
      GraphId new_node(tile.tileid(), tile.level(), 0);
      new_nodes[tile] = 1;
      return new_node;
    } else {
      GraphId new_node(tile.tileid(), tile.level(), itr->second);
      itr->second++;
      return new_node;
    }
  };

  // Create a sequence to associate new nodes to old nodes
  sequence<std::pair<GraphId, GraphId>> new_to_old(new_to_old_file, true);

  // Create a sequence to associate new nodes to old nodes
  sequence<OldToNewNodes> old_to_new(old_to_new_file, true);

  // Hierarchy level information
  const auto& arterial_level = TileHierarchy::levels()[1];
  uint32_t al = static_cast<uint32_t>(arterial_level.level);
  const auto& highway_level = TileHierarchy::levels()[0];
  uint32_t hl = static_cast<uint32_t>(highway_level.level);

  // Iterate through all tiles in the local level
  auto local_tiles = reader.GetTileSet();
  for (const auto& base_tile_id : local_tiles) {
    // We keep all transit data inside the transit hierarchy
    if (base_tile_id.level() == TileHierarchy::GetTransitLevel().level) {
      continue;
    }

    // Get the graph tile. Skip if no tile exists or no nodes exist in the tile.
    graph_tile_ptr tile = reader.GetGraphTile(base_tile_id);
    if (!tile) {
      continue;
    }

    // Iterate through the nodes. Add nodes to the new level when
    // best road class <= the new level classification cutoff
    bool levels[3];
    uint32_t nodecount = tile->header()->nodecount();
    GraphId basenode = base_tile_id;
    GraphId edgeid = base_tile_id;
    PointLL base_ll = tile->header()->base_ll();
    const NodeInfo* nodeinfo = tile->node(basenode);
    for (uint32_t i = 0; i < nodecount; i++, nodeinfo++, ++basenode) {
      // Iterate through the edges to see which levels this node exists.
      levels[0] = levels[1] = levels[2] = false;
      for (uint32_t j = 0; j < nodeinfo->edge_count(); j++, ++edgeid) {
        // Update the flag for the level of this edge (skip transit
        // connection edges)
        const DirectedEdge* directededge = tile->directededge(edgeid);
        if (directededge->bss_connection()) {
          // Despite the road class, Bike Share Stations' connections are always at local level
          levels[2] = true;
        } else if (directededge->use() != Use::kTransitConnection &&
                   directededge->use() != Use::kEgressConnection &&
                   directededge->use() != Use::kPlatformConnection) {
          levels[TileHierarchy::get_level(directededge->classification())] = true;
        }
      }

      // Associate new nodes to base nodes and base node to new nodes
      GraphId highway_node, arterial_node, local_node;
      if (levels[0]) {
        // New node is on the highway level. Associate back to base/local node
        GraphId new_tile(highway_level.tiles.TileId(nodeinfo->latlng(base_ll)), hl, 0);
        highway_node = get_new_node(new_tile);
        new_to_old.push_back(std::make_pair(highway_node, basenode));
      }
      if (levels[1]) {
        // New node is on the arterial level. Associate back to base/local node
        GraphId new_tile(arterial_level.tiles.TileId(nodeinfo->latlng(base_ll)), al, 0);
        arterial_node = get_new_node(new_tile);
        new_to_old.push_back(std::make_pair(arterial_node, basenode));
      }
      if (levels[2]) {
        // New node is on the local level. Associate back to base/local node
        local_node = get_new_node(base_tile_id);
        new_to_old.push_back(std::make_pair(local_node, basenode));
      }

      if (!levels[0] && !levels[1] && !levels[2]) {
        LOG_ERROR("No valid level for this node!");
      }

      // Associate the old node to the new node(s). Entries in the tuple
      // that are invalid nodes indicate no node exists in the new level.
      OldToNewNodes assoc(basenode, highway_node, arterial_node, local_node, nodeinfo->density());
      old_to_new.push_back(assoc);
    }

    // Check if we need to clear the tile cache
    if (reader.OverCommitted()) {
      reader.Trim();
    }
  }
}

/**
 * Update end nodes of transit connection directed edges.
 */
void UpdateTransitConnections(GraphReader& reader, const std::string& old_to_new_file) {
  // Use the sorted sequence that associates old nodes to new nodes
  sequence<OldToNewNodes> old_to_new(old_to_new_file, false);

  uint8_t transit_level = TileHierarchy::GetTransitLevel().level;
  auto transit_tiles = reader.GetTileSet(transit_level);
  for (const auto& tile_id : transit_tiles) {
    // Skip if no nodes exist in the tile
    graph_tile_ptr tile = reader.GetGraphTile(tile_id);
    if (!tile) {
      continue;
    }

    // Create a new tile builder
    GraphTileBuilder tilebuilder(reader.tile_dir(), tile_id, false);

    // Update end nodes of transit connection directed edges
    std::vector<NodeInfo> nodes;
    std::vector<DirectedEdge> directededges;
    for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); i++) {
      NodeInfo nodeinfo = tilebuilder.node(i);
      uint32_t idx = nodeinfo.edge_index();
      for (uint32_t j = 0; j < nodeinfo.edge_count(); j++, idx++) {
        DirectedEdge directededge = tilebuilder.directededge(idx);

        // Update the end node of any transit connection edge
        if (directededge.use() == Use::kTransitConnection) {
          // Get the updated end node
          auto f = find_nodes(old_to_new, directededge.endnode());
          GraphId new_end_node;
          if (f.local_node.Is_Valid()) {
            new_end_node = f.local_node;
          } else if (f.arterial_node.Is_Valid()) {
            new_end_node = f.arterial_node;
          } else if (f.highway_node.Is_Valid()) {
            new_end_node = f.highway_node;
          } else {
            LOG_ERROR("Transit Connection does not connect to valid node");
          }
          directededge.set_endnode(new_end_node);
        }

        // Add the directed edge to the local list
        directededges.emplace_back(std::move(directededge));
      }

      // Add the node to the local list
      nodes.emplace_back(std::move(nodeinfo));
    }
    tilebuilder.Update(nodes, directededges);
  }
}

// Remove any base tiles that no longer have any data (nodes and edges
// only exist on arterial and highway levels)
void RemoveUnusedLocalTiles(const std::string& tile_dir, const std::string& old_to_new_file) {
  // Iterate through the node association sequence
  std::unordered_map<GraphId, bool> tile_map;
  sequence<OldToNewNodes> old_to_new(old_to_new_file, false);
  for (auto itr = old_to_new.begin(); itr != old_to_new.end(); itr++) {
    auto f = tile_map.find((*itr).node_id.Tile_Base());
    if (f == tile_map.end()) {
      tile_map[(*itr).node_id.Tile_Base()] = (*itr).local_node.Is_Valid();
    } else {
      if ((*itr).local_node.Is_Valid()) {
        f->second = true;
      }
    }
  }
  for (auto itr = tile_map.begin(); itr != tile_map.end(); itr++) {
    if (!itr->second) {
      // Remove the file
      GraphId empty_tile = itr->first;
      std::string file_location = tile_dir + filesystem::path::preferred_separator +
                                  GraphTile::FileSuffix(empty_tile.Tile_Base());
      remove(file_location.c_str());
      LOG_DEBUG("Remove file: " + file_location);
    }
  }
}

} // namespace

namespace valhalla {
namespace mjolnir {

// Build successive levels of the hierarchy, starting at the local
// base level. Each successive level of the hierarchy is based on
// and connected to the next.
void HierarchyBuilder::Build(const boost::property_tree::ptree& pt,
                             const std::string& new_to_old_file,
                             const std::string& old_to_new_file) {

  // TODO: thread this. Might be more possible now that we don't create
  // shortcuts in the HierarchyBuilder

  // Construct GraphReader
  LOG_INFO("HierarchyBuilder");
  GraphReader reader(pt.get_child("mjolnir"));

  // Association of old nodes to new nodes
  CreateNodeAssociations(reader, new_to_old_file, old_to_new_file);

  // Sort the sequences
  SortSequences(new_to_old_file, old_to_new_file);

  // Iterate through the hierarchy (from highway down to local) and build
  // new tiles
  FormTilesInNewLevel(reader, new_to_old_file, old_to_new_file);

  // Remove any base tiles that no longer have any data (nodes and edges
  // only exist on arterial and highway levels)
  RemoveUnusedLocalTiles(reader.tile_dir(), old_to_new_file);

  // Update the end nodes to all transit connections in the transit hierarchy
  auto hierarchy_properties = pt.get_child("mjolnir");
  auto transit_dir = hierarchy_properties.get_optional<std::string>("transit_dir");
  if (transit_dir && filesystem::exists(*transit_dir) && filesystem::is_directory(*transit_dir)) {
    UpdateTransitConnections(reader, old_to_new_file);
  }

  LOG_INFO("Done HierarchyBuilder");
}

} // namespace mjolnir
} // namespace valhalla
