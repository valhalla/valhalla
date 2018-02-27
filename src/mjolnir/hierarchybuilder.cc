#include "mjolnir/hierarchybuilder.h"
#include "mjolnir/graphtilebuilder.h"

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <boost/property_tree/ptree.hpp>
#include <boost/filesystem/operations.hpp>

#include "midgard/pointll.h"
#include "midgard/logging.h"
#include "midgard/encoded.h"
#include "midgard/sequence.h"
#include "baldr/filesystem_utils.h"
#include "baldr/tilehierarchy.h"
#include "baldr/graphid.h"
#include "baldr/graphconstants.h"
#include "baldr/graphtile.h"
#include "baldr/graphreader.h"

#include <boost/format.hpp>
#include <ostream>
#include <set>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

// Sequence file names (named as bin so it gets cleaned up/removed when done)
std::string new_to_old_file = std::string("new_nodes_to_old_nodes.bin");
std::string old_to_new_file = std::string("old_nodes_to_new_nodes.bin");

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

  OldToNewNodes(const GraphId& node, const GraphId& highway,
                const GraphId& arterial, const GraphId& local,
                const uint32_t d)
      : node_id(node),
        highway_node(highway),
        arterial_node(arterial),
        local_node(local),
        density(d) {
  }
};

// Add a downward transition edge if the node is valid.
bool AddDownwardTransition(const GraphId& node, GraphTileBuilder* tilebuilder,
                           const bool has_elevation) {
  if (node.Is_Valid()) {
    DirectedEdge downwardedge;
    downwardedge.set_endnode(node);
    downwardedge.set_trans_down();
    downwardedge.set_all_forward_access();
    tilebuilder->directededges().emplace_back(std::move(downwardedge));
    if (has_elevation) {
      tilebuilder->edge_elevations().emplace_back(0.0f, 0.0f, 0.0f);
    }
    return true;
  } else {
    return false;
  }
}

// Add an upward transition edge if the node is valid.
bool AddUpwardTransition(const GraphId& node, GraphTileBuilder* tilebuilder,
                         const bool has_elevation) {
  if (node.Is_Valid()) {
    DirectedEdge upwardedge;
    upwardedge.set_endnode(node);
    upwardedge.set_trans_up();
    upwardedge.set_all_forward_access();
    tilebuilder->directededges().emplace_back(std::move(upwardedge));
    if (has_elevation) {
      tilebuilder->edge_elevations().emplace_back(0.0f, 0.0f, 0.0f);
    }
    return true;
  } else {
    return false;
  }
}

void SortSequences() {
  // Sort the new nodes. Sort so highway level is first
  sequence<std::pair<GraphId, GraphId>> new_to_old(new_to_old_file, false);
  new_to_old.sort(
    [](const std::pair<GraphId, GraphId>& a, const std::pair<GraphId, GraphId>& b){
      if (a.first.level() == b.first.level()) {
        if (a.first.tileid() == b.first.tileid()) {
          return a.first.id() < b.first.id();
        }
        return a.first.tileid() < b.first.tileid();
      }
      return a.first.level() < b.first.level();
    }
  );

  // Sort old to new by node Id
  sequence<OldToNewNodes> old_to_new(old_to_new_file, false);
  old_to_new.sort([](const OldToNewNodes& a, const OldToNewNodes& b)
                  {return a.node_id < b.node_id;});
}

// Convencience method to find the node association.
OldToNewNodes find_nodes(sequence<OldToNewNodes>& old_to_new, const GraphId& node) {
  GraphId dmy;
  OldToNewNodes target(node, dmy, dmy, dmy, 0);
  auto iter = old_to_new.find(target, [](const OldToNewNodes& a, const OldToNewNodes& b)
                              { return a.node_id < b.node_id;});
  if (iter == old_to_new.end()) {
    throw std::runtime_error("Didn't find node!");
  } else {
    return *iter;
  }
}

// Form tiles in the new level.
void FormTilesInNewLevel(GraphReader& reader, bool has_elevation) {
  // Use the sequence that associate new nodes to old nodes
  sequence<std::pair<GraphId, GraphId>> new_to_old(new_to_old_file, false);

  // Use the sorted sequence that associates old nodes to new nodes
  sequence<OldToNewNodes> old_to_new(old_to_new_file, false);

  // lambda to indicate whether a directed edge should be included
  auto include_edge = [&old_to_new](const DirectedEdge* directededge,
        const GraphId& base_node, const uint8_t current_level) {
    if (directededge->use() == Use::kTransitConnection ||
        directededge->use() == Use::kEgressConnection ||
        directededge->use() == Use::kPlatformConnection) {
      // Transit connection edges should live on the lowest class level
      // where a new node exists
      uint8_t lowest_level;
      auto f = find_nodes(old_to_new, base_node);
      if (f.local_node.Is_Valid()) {
        lowest_level = 2;
      } else if (f.arterial_node.Is_Valid()) {
        lowest_level = 1;
      } else if (f.highway_node.Is_Valid()) {
        lowest_level = 0;
      }
      return (lowest_level == current_level);
    } else {
      return (TileHierarchy::get_level(directededge->classification()) == current_level);
    }
  };

  // Iterate through the new nodes. They have been sorted by level so that
  // highway level is done first.
  reader.Clear();
  bool added = false;
  uint8_t current_level;
  GraphId tile_id;
  std::hash<std::string> hasher;
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

      // Check if we need to clear the base/local tile cache
      if (reader.OverCommitted()) {
        reader.Clear();
      }
    }

    // Get the node in the base level
    GraphId base_node = (*new_node).second;
    const GraphTile* tile = reader.GetGraphTile(base_node);
    if (tile == nullptr) {
      LOG_ERROR("Base tile is null? ");
      continue;
    }

    // Copy the data version
    tilebuilder->header_builder().set_dataset_id(tile->header()->dataset_id());

    // Copy node information
    NodeInfo baseni = *(tile->node(base_node.id()));
    tilebuilder->nodes().push_back(baseni);
    const auto& admin = tile->admininfo(baseni.admin_index());
    NodeInfo& node = tilebuilder->nodes().back();
    node.set_edge_index(tilebuilder->directededges().size());
    node.set_timezone(baseni.timezone());
    node.set_admin_index(tilebuilder->AddAdmin(admin.country_text(), admin.state_text(),
                                               admin.country_iso(), admin.state_iso()));

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
      uint32_t edge_density = (density2 == 32) ? density1 :
                (density1 + density2) / 2;
      newedge.set_density(edge_density);

      // Set opposing edge indexes to 0 (gets set in graph validator).
      newedge.set_opp_index(0);

      // Get signs from the base directed edge
      if (directededge->exitsign()) {
        std::vector<SignInfo> signs = tile->GetSigns(base_edge_id.id());
        if (signs.size() == 0) {
          LOG_ERROR("Base edge should have signs, but none found");
        }
        tilebuilder->AddSigns(tilebuilder->directededges().size(), signs);
      }

      // Get access restrictions from the base directed edge. Add these to
      // the list of access restrictions in the new tile. Update the
      // edge index in the restriction to be the current directed edge Id
      if (directededge->access_restriction()) {
        auto restrictions = tile->GetAccessRestrictions(base_edge_id.id(), kAllAccess);
        for (const auto& res : restrictions) {
          tilebuilder->AddAccessRestriction(
              AccessRestriction(tilebuilder->directededges().size(),
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

      // Get edge info, shape, and names from the old tile and add to the
      // new. Cannot use edge info offset since edges in arterial and
      // highway hierarchy can cross base tiles! Use a hash based on the
      // encoded shape plus way Id.
      uint32_t idx = directededge->edgeinfo_offset();
      auto edgeinfo = tile->edgeinfo(idx);
      std::string encoded_shape = edgeinfo.encoded_shape();
      uint32_t w = hasher(encoded_shape + std::to_string(edgeinfo.wayid()));
      uint32_t edge_info_offset = tilebuilder->AddEdgeInfo(w, nodea, nodeb,
                    edgeinfo.wayid(), encoded_shape,
                    tile->GetNames(idx), tile->GetTypes(idx), added);
      newedge.set_edgeinfo_offset(edge_info_offset);

      // Add directed edge
      tilebuilder->directededges().emplace_back(std::move(newedge));

      // Add edge elevation
      if (has_elevation) {
        const EdgeElevation* elev = tile->edge_elevation(base_edge_id);
        if (elev == nullptr) {
          tilebuilder->edge_elevations().emplace_back(0.0f, 0.0f, 0.0f);
        } else {
          tilebuilder->edge_elevations().emplace_back(std::move(*elev));
        }
      }
    }

    // Add transition edges
    auto new_nodes = find_nodes(old_to_new, base_node);
    if (current_level == 0) {
      AddDownwardTransition(new_nodes.arterial_node, tilebuilder, has_elevation);
      AddDownwardTransition(new_nodes.local_node, tilebuilder, has_elevation);
    } else if (current_level == 1) {
      AddDownwardTransition(new_nodes.local_node, tilebuilder, has_elevation);
      AddUpwardTransition(new_nodes.highway_node, tilebuilder, has_elevation);
    }
    if (current_level == 2) {
      AddUpwardTransition(new_nodes.arterial_node, tilebuilder, has_elevation);
      AddUpwardTransition(new_nodes.highway_node, tilebuilder, has_elevation);
    }

    // Set the edge count for the new node
    node.set_edge_count(tilebuilder->directededges().size() - edge_count);
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
 * @return  Returns true if any base tiles have edge elevation data.
 */
bool CreateNodeAssociations(GraphReader& reader) {
  // Map of tiles vs. count of nodes. Used to construct new node Ids.
  std::unordered_map<GraphId, uint32_t> new_nodes;

  // lambda to get the next "new" node Id given a tile
  auto get_new_node = [&new_nodes](const GraphId& tile) {
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
  auto tile_level = TileHierarchy::levels().rbegin();
  auto& base_level = tile_level->second;
  tile_level++;
  auto& arterial_level = tile_level->second;
  tile_level++;
  auto& highway_level = tile_level->second;

  // Get the set of tiles on the local level
  auto local_tiles = reader.GetTileSet(base_level.level);

  // Iterate through all tiles in the local level
  bool has_elevation = false;
  uint32_t ntiles = base_level.tiles.TileCount();
  uint32_t bl = static_cast<uint32_t>(base_level.level);
  uint32_t al = static_cast<uint32_t>(arterial_level.level);
  uint32_t hl = static_cast<uint32_t>(highway_level.level);
  for (const auto& base_tile_id : local_tiles) {
    // Get the graph tile. Skip if no tile exists or no nodes exist in the tile.
    const GraphTile* tile = reader.GetGraphTile(base_tile_id);
    if (tile == nullptr || tile->header()->nodecount() == 0) {
      continue;
    }

    // Update the has_elevation flag
    if (tile->header()->has_edge_elevation()) {
      has_elevation = true;
    }

    // Iterate through the nodes. Add nodes to the new level when
    // best road class <= the new level classification cutoff
    bool levels[3];
    uint32_t nodecount = tile->header()->nodecount();
    GraphId basenode = base_tile_id;
    GraphId edgeid = base_tile_id;
    const NodeInfo* nodeinfo = tile->node(basenode);
    for (uint32_t i = 0; i < nodecount; i++, nodeinfo++, ++basenode) {
      // Iterate through the edges to see which levels this node exists.
      levels[0] = levels[1] = levels[2] = false;
      for (uint32_t j = 0; j < nodeinfo->edge_count(); j++, ++edgeid) {
        // Update the flag for the level of this edge (skip transit
        // connection edges)
        const DirectedEdge* directededge = tile->directededge(edgeid);
        if (directededge->use() != Use::kTransitConnection &&
            directededge->use() != Use::kEgressConnection &&
            directededge->use() != Use::kPlatformConnection) {
          levels[TileHierarchy::get_level(directededge->classification())] = true;
        }
      }

      // Associate new nodes to base nodes and base node to new nodes
      GraphId highway_node, arterial_node, local_node;
      if (levels[0]) {
        // New node is on the highway level. Associate back to base/local node
        GraphId new_tile(highway_level.tiles.TileId(nodeinfo->latlng()), hl, 0);
        highway_node = get_new_node(new_tile);
        new_to_old.push_back(std::make_pair(highway_node, basenode));
      }
      if (levels[1]) {
        // New node is on the arterial level. Associate back to base/local node
        GraphId new_tile(arterial_level.tiles.TileId(nodeinfo->latlng()), al, 0);
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
      OldToNewNodes assoc(basenode, highway_node, arterial_node,
                           local_node, nodeinfo->density());
      old_to_new.push_back(assoc);
    }

    // Check if we need to clear the tile cache
    if(reader.OverCommitted()) {
      reader.Clear();
    }
  }
  return has_elevation;
}

/**
 * Update end nodes of transit connection directed edges.
 */
void UpdateTransitConnections(GraphReader& reader) {
  // Use the sorted sequence that associates old nodes to new nodes
  sequence<OldToNewNodes> old_to_new(old_to_new_file, false);

  auto tile_level = TileHierarchy::levels().rbegin();
  uint8_t transit_level = tile_level->second.level + 1;
  auto transit_tiles = reader.GetTileSet(transit_level);
  for (const auto& tile_id : transit_tiles) {
    // Skip if no nodes exist in the tile
    const GraphTile* tile = reader.GetGraphTile(tile_id);
    if (tile == nullptr || tile->header()->nodecount() == 0) {
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
       for (uint32_t j = 0; j <  nodeinfo.edge_count(); j++, idx++) {
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
void RemoveUnusedLocalTiles(const std::string& tile_dir) {
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
    if (!itr->second ) {
      // Remove the file
      GraphId empty_tile = itr->first;
      std::string file_location = tile_dir + filesystem::path_separator +
          GraphTile::FileSuffix(empty_tile.Tile_Base());
      remove(file_location.c_str());
      LOG_DEBUG("Remove file: " + file_location);
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
  LOG_INFO("HierarchyBuilder");
  GraphReader reader(pt.get_child("mjolnir"));

  // Association of old nodes to new nodes
  bool has_elevation = CreateNodeAssociations(reader);
  if (has_elevation) {
    LOG_INFO("Base tiles have edge elevation information");
  }

  // Sort the sequences
  SortSequences();

  // Iterate through the hierarchy (from highway down to local) and build
  // new tiles
  FormTilesInNewLevel(reader, has_elevation);

  // Remove any base tiles that no longer have any data (nodes and edges
  // only exist on arterial and highway levels)
  RemoveUnusedLocalTiles(reader.tile_dir());

  // Update the end nodes to all transit connections in the transit hierarchy
  auto hierarchy_properties = pt.get_child("mjolnir");
  auto transit_dir = hierarchy_properties.get_optional<std::string>("transit_dir");
  if (transit_dir && boost::filesystem::exists(*transit_dir) && boost::filesystem::is_directory(*transit_dir)) {
    UpdateTransitConnections(reader);
  }

  LOG_INFO("Done HierarchyBuilder");
}

}
}
