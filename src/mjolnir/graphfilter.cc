#include "mjolnir/graphfilter.h"
#include "mjolnir/graphtilebuilder.h"

#include <boost/property_tree/ptree.hpp>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/sequence.h"

using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

uint32_t n_original_edges = 0;
uint32_t n_original_nodes = 0;
uint32_t n_filtered_edges = 0;
uint32_t n_filtered_nodes = 0;
uint32_t can_aggregate = 0;

// Group wheelchair and pedestrian access together
constexpr uint32_t kAllPedestrianAccess = (kPedestrianAccess | kWheelchairAccess);

/**
 * Filter edges to optionally remove edges by access.
 * @param  reader  Graph reader.
 * @param  old_to_new  Map of original node Ids to new nodes Ids (after filtering).
 * @param  include_driving  Include edge if driving (any vehicular) access in either direction.
 * @param  include_bicycle  Include edge if bicycle access in either direction.
 * @param  include_pedestrian  Include edge if pedestrian or wheelchair access in either direction.
 */
void FilterTiles(GraphReader& reader,
                 std::unordered_map<GraphId, GraphId>& old_to_new,
                 const bool include_driving,
                 const bool include_bicycle,
                 const bool include_pedestrian) {

  // lambda to check if an edge should be included
  auto include_edge = [&include_driving, &include_bicycle,
                       &include_pedestrian](const DirectedEdge* edge) {
    // Edge filtering
    bool bicycle_access =
        (edge->forwardaccess() & kBicycleAccess) || (edge->reverseaccess() & kBicycleAccess);
    bool pedestrian_access = (edge->forwardaccess() & kAllPedestrianAccess) ||
                             (edge->reverseaccess() & kAllPedestrianAccess);
    bool driving_access =
        (edge->forwardaccess() & kVehicularAccess) || (edge->reverseaccess() & kVehicularAccess);
    return (driving_access && include_driving) || (bicycle_access && include_bicycle) ||
           (pedestrian_access && include_pedestrian);
  };

  // Iterate through all tiles in the local level
  auto local_tiles = reader.GetTileSet(TileHierarchy::levels().back().level);
  for (const auto& tile_id : local_tiles) {
    // Create a new tilebuilder - should copy header information
    GraphTileBuilder tilebuilder(reader.tile_dir(), tile_id, false);
    n_original_nodes += tilebuilder.header()->nodecount();
    n_original_edges += tilebuilder.header()->directededgecount();

    // Get the graph tile. Read from this tile to create the new tile.
    graph_tile_ptr tile = reader.GetGraphTile(tile_id);
    assert(tile);

    std::hash<std::string> hasher;
    GraphId nodeid(tile_id.tileid(), tile_id.level(), 0);
    for (uint32_t i = 0; i < tile->header()->nodecount(); ++i, ++nodeid) {
      // Count of edges added for this node
      uint32_t edge_count = 0;

      // Current edge index for first edge from this node
      uint32_t edge_index = tilebuilder.directededges().size();

      // Iterate through directed edges outbound from this node
      std::vector<uint64_t> wayid;
      std::vector<GraphId> endnode;
      const NodeInfo* nodeinfo = tile->node(nodeid);
      GraphId edgeid(nodeid.tileid(), nodeid.level(), nodeinfo->edge_index());
      for (uint32_t j = 0; j < nodeinfo->edge_count(); ++j, ++edgeid) {
        // Check if the directed edge should be included
        const DirectedEdge* directededge = tile->directededge(edgeid);
        if (!include_edge(directededge)) {
          ++n_filtered_edges;
          continue;
        }

        // Copy the directed edge information
        DirectedEdge newedge = *directededge;

        // Set opposing edge indexes to 0 (gets set in graph validator).
        newedge.set_opp_index(0);

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
                                                               res.type(), res.modes(), res.value()));
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

        // Get edge info, shape, and names from the old tile and add to the
        // new. Cannot use edge info offset since edges in arterial and
        // highway hierarchy can cross base tiles! Use a hash based on the
        // encoded shape plus way Id.
        bool added;
        auto edgeinfo = tile->edgeinfo(directededge);
        std::string encoded_shape = edgeinfo.encoded_shape();
        uint32_t w = hasher(encoded_shape + std::to_string(edgeinfo.wayid()));
        uint32_t edge_info_offset =
            tilebuilder.AddEdgeInfo(w, nodeid, directededge->endnode(), edgeinfo.wayid(),
                                    edgeinfo.mean_elevation(), edgeinfo.bike_network(),
                                    edgeinfo.speed_limit(), encoded_shape, edgeinfo.GetNames(),
                                    edgeinfo.GetTaggedValues(), edgeinfo.GetTypes(), added);
        newedge.set_edgeinfo_offset(edge_info_offset);
        wayid.push_back(edgeinfo.wayid());
        endnode.push_back(directededge->endnode());

        // Add directed edge
        tilebuilder.directededges().emplace_back(std::move(newedge));
        ++edge_count;
      }

      // Add the node to the tilebuilder unless no edges remain
      if (edge_count > 0) {
        // Add a node builder to the tile. Update the edge count and edgeindex
        GraphId new_node(nodeid.tileid(), nodeid.level(), tilebuilder.nodes().size());
        tilebuilder.nodes().push_back(*nodeinfo);
        NodeInfo& node = tilebuilder.nodes().back();
        node.set_edge_count(edge_count);
        node.set_edge_index(edge_index);
        const auto& admin = tile->admininfo(nodeinfo->admin_index());
        node.set_admin_index(tilebuilder.AddAdmin(admin.country_text(), admin.state_text(),
                                                  admin.country_iso(), admin.state_iso()));

        // Get named signs from the base node
        if (nodeinfo->named_intersection()) {
          std::vector<SignInfo> signs = tile->GetSigns(nodeid.id(), true);
          if (signs.size() == 0) {
            LOG_ERROR("Base node should have signs, but none found");
          }
          node.set_named_intersection(true);
          tilebuilder.AddSigns(tilebuilder.nodes().size() - 1, signs);
        }

        // Associate the old node to the new node.
        old_to_new[nodeid] = new_node;

        // Check if edges at this node can be aggregated. Only 2 edges, same way Id (so that
        // edge attributes should match), don't end at same node (no loops).
        if (edge_count == 2 && wayid[0] == wayid[1] && endnode[0] != endnode[1]) {
          ++can_aggregate;
        }
      } else {
        ++n_filtered_nodes;
      }
    }

    // Store the updated tile data (or remove tile if all edges are filtered)
    if (tilebuilder.nodes().size() > 0) {
      tilebuilder.StoreTileData();
    } else {
      // Remove the tile - all nodes and edges were filtered
      std::string file_location =
          reader.tile_dir() + filesystem::path::preferred_separator + GraphTile::FileSuffix(tile_id);
      remove(file_location.c_str());
      LOG_INFO("Remove file: " + file_location + " all edges were filtered");
    }

    if (reader.OverCommitted()) {
      reader.Trim();
    }
  }
  LOG_INFO("Filtered " + std::to_string(n_filtered_nodes) + " nodes out of " +
           std::to_string(n_original_nodes));
  LOG_INFO("Filtered " + std::to_string(n_filtered_edges) + " directededges out of " +
           std::to_string(n_original_edges));
  LOG_INFO("Can aggregate: " + std::to_string(can_aggregate));
}

/**
 * Update end nodes of all directed edges.
 * @param  reader  Graph reader.
 * @param  old_to_new  Map of original node Ids to new nodes Ids (after filtering).
 */
void UpdateEndNodes(GraphReader& reader, std::unordered_map<GraphId, GraphId>& old_to_new) {
  LOG_INFO("Update end nodes of directed edges");

  int found = 0;

  // Iterate through all tiles in the local level
  auto local_tiles = reader.GetTileSet(TileHierarchy::levels().back().level);
  for (const auto& tile_id : local_tiles) {
    // Get the graph tile. Skip if no tile exists (should not happen!?)
    graph_tile_ptr tile = reader.GetGraphTile(tile_id);
    assert(tile);

    // Create a new tilebuilder - should copy header information
    GraphTileBuilder tilebuilder(reader.tile_dir(), tile_id, false);

    // Copy nodes (they do not change)
    std::vector<NodeInfo> nodes;
    size_t n = tile->header()->nodecount();
    nodes.reserve(n);
    const NodeInfo* orig_nodes = tile->node(0);
    std::copy(orig_nodes, orig_nodes + n, std::back_inserter(nodes));

    // Iterate through all directed edges - update end nodes
    std::vector<DirectedEdge> directededges;
    GraphId edgeid(tile_id.tileid(), tile_id.level(), 0);
    for (uint32_t j = 0; j < tile->header()->directededgecount(); ++j, ++edgeid) {
      const DirectedEdge* edge = tile->directededge(j);

      // Find the end node in the old_to_new mapping
      GraphId end_node;
      auto iter = old_to_new.find(edge->endnode());
      if (iter == old_to_new.end()) {
        LOG_ERROR("UpdateEndNodes - failed to find associated node");
      } else {
        end_node = iter->second;
        found++;
      }

      // Copy the edge to the directededges vector and update the end node
      directededges.push_back(*edge);
      DirectedEdge& new_edge = directededges.back();
      new_edge.set_endnode(end_node);
    }

    // Update the tile with new directededges.
    tilebuilder.Update(nodes, directededges);

    if (reader.OverCommitted()) {
      reader.Trim();
    }
  }
}

} // namespace

namespace valhalla {
namespace mjolnir {

// Optionally filter edges and nodes based on access.
void GraphFilter::Filter(const boost::property_tree::ptree& pt) {

  // TODO: thread this. Could be difficult due to sequence creates to associate nodes

  // Edge filtering (optionally exclude edges)
  bool include_driving = pt.get_child("mjolnir").get<bool>("include_driving", true);
  if (!include_driving) {
    LOG_INFO("GraphFilter: Filter driving edges");
  }
  bool include_bicycle = pt.get_child("mjolnir").get<bool>("include_bicycle", true);
  if (!include_bicycle) {
    LOG_INFO("GraphFilter: Filter bicycle edges");
  }
  bool include_pedestrian = pt.get_child("mjolnir").get<bool>("include_pedestrian", true);
  if (!include_pedestrian) {
    LOG_INFO("GraphFilter: Filter pedestrian edges");
  }
  if (include_bicycle && include_driving && include_pedestrian) {
    // Nothing to filter!
    LOG_INFO("GraphFilter - nothing to filter. Skipping...");
    return;
  }

  // Map of old node Ids to new node Ids (after filtering).
  std::unordered_map<baldr::GraphId, baldr::GraphId> old_to_new;

  // Construct GraphReader
  GraphReader reader(pt.get_child("mjolnir"));

  // Filter edges (and nodes) by access
  FilterTiles(reader, old_to_new, include_driving, include_bicycle, include_pedestrian);

  // TODO - aggregate / combine edges across false nodes (only 2 directed edges)
  // where way Ids are equal

  // Update end nodes. Clear the GraphReader cache first.
  reader.Clear();
  UpdateEndNodes(reader, old_to_new);

  LOG_INFO("Done GraphFilter");
}

} // namespace mjolnir
} // namespace valhalla
