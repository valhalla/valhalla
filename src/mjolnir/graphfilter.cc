#include "mjolnir/graphfilter.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/util.h"

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
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;

namespace {

uint32_t n_original_edges = 0;
uint32_t n_original_nodes = 0;
uint32_t n_filtered_edges = 0;
uint32_t n_filtered_nodes = 0;
uint32_t can_aggregate = 0;
uint32_t aggregated = 0;

// Group wheelchair and pedestrian access together
constexpr uint32_t kAllPedestrianAccess = (kPedestrianAccess | kWheelchairAccess);

bool CanAggregate(const DirectedEdge* de) {
  if (de->start_restriction() || de->part_of_complex_restriction() || de->end_restriction() ||
      de->restrictions() || de->traffic_signal() || de->access_restriction()) {
    return false;
  }
  return true;
}

// ExpandFromNode and ExpandFromNodeInner is reused code from restriction builder with some slight
// modifications. We are using recursion for graph traversal. We have to make sure we don't loop back
// to ourselves, walk in the correct direction, have not already visited a node, etc. Once we meet our
// criteria or not we stop.
bool ExpandFromNode(GraphReader& reader,
                    std::list<PointLL>& shape,
                    GraphId& en,
                    const GraphId& from_node,
                    std::unordered_set<std::string>& isos,
                    bool forward,
                    std::unordered_set<GraphId>& visited_nodes,
                    uint64_t& way_id,
                    const graph_tile_ptr& prev_tile,
                    GraphId prev_node,
                    GraphId current_node,
                    const RoadClass& rc,
                    bool validate);

/*
 * Expand from the current node
 * @param  reader  Graph reader.
 * @param  shape  shape that we need to update
 * @param  en  current end node that we started at
 * @param  from_node  node that we started from
 * @param  isos  country ISOs. Used to see if we cross into a new country
 * @param  forward  traverse in the forward or backward direction
 * @param  visited_nodes  nodes that we already visited.  don't visit again
 * @param  way_id  only interested in edges with this way_id
 * @param  prev_tile  previous tile
 * @param  prev_node  previous node
 * @param  current_node  current node
 * @param  node_info  current node's info
 * @param  validate  Are we validating data?
 *
 */
bool ExpandFromNodeInner(GraphReader& reader,
                         std::list<PointLL>& shape,
                         GraphId& en,
                         const GraphId& from_node,
                         std::unordered_set<std::string>& isos,
                         bool forward,
                         std::unordered_set<GraphId>& visited_nodes,
                         uint64_t& way_id,
                         const graph_tile_ptr& prev_tile,
                         GraphId prev_node,
                         GraphId current_node,
                         const NodeInfo* node_info,
                         const RoadClass& rc,
                         bool validate) {

  for (size_t j = 0; j < node_info->edge_count(); ++j) {
    GraphId edge_id(prev_tile->id().tileid(), prev_tile->id().level(), node_info->edge_index() + j);
    const DirectedEdge* de = prev_tile->directededge(edge_id);
    const auto& edge_info = prev_tile->edgeinfo(de);

    auto tile = prev_tile;
    if (tile->id() != de->endnode().Tile_Base()) {
      tile = reader.GetGraphTile(de->endnode());
    }

    const NodeInfo* en_info = tile->node(de->endnode().id());
    // check the direction, if we looped back, or are we done
    if ((de->endnode() != prev_node) && (de->forward() == forward) &&
        (de->endnode() != from_node || (de->endnode() == from_node && visited_nodes.size() > 1))) {
      if (edge_info.wayid() == way_id &&
          (en_info->mode_change() || (node_info->mode_change() && !en_info->mode_change()))) {

        // If this edge has special attributes, then we can't aggregate
        if (!CanAggregate(de) || de->classification() != rc) {
          way_id = 0;
          return false;
        }

        if (validate) {
          if (isos.size() >= 1) {
            isos.insert(tile->admin(en_info->admin_index())->country_iso());
          }
        } else {
          std::list<PointLL> edgeshape =
              valhalla::midgard::decode7<std::list<PointLL>>(edge_info.encoded_shape());
          if (!de->forward()) {
            std::reverse(edgeshape.begin(), edgeshape.end());
          }

          // Append shape. Skip first point since it
          // should equal the last of the prior edge.
          edgeshape.pop_front();
          shape.splice(shape.end(), edgeshape);
        }

        // found a node that does not have aggregation marked (using mode_change flag)
        // we are done.
        if (node_info->mode_change() && !en_info->mode_change()) {
          en = de->endnode();
          aggregated++;
          return true;
        }
        aggregated++;

        bool found;
        if (visited_nodes.find(de->endnode()) == visited_nodes.end()) {
          visited_nodes.insert(de->endnode());

          // expand with the same way_id
          found = ExpandFromNode(reader, shape, en, from_node, isos, forward, visited_nodes, way_id,
                                 tile, current_node, de->endnode(), rc, validate);
          if (found) {
            return true;
          }

          visited_nodes.erase(de->endnode());
        }
      }
    }
  }
  return false;
}

/*
 * Expand from the next node which is now our new current node
 * @param  reader  Graph reader.
 * @param  shape  shape that we need to update
 * @param  en  current end node that we started at
 * @param  from_node  node that we started from
 * @param  isos  country ISOs. Used to see if we cross into a new country
 * @param  forward  traverse in the forward or backward direction
 * @param  visited_nodes  nodes that we already visited.  don't visit again
 * @param  way_id  only interested in edges with this way_id
 * @param  prev_tile  previous tile
 * @param  prev_node  previous node
 * @param  current_node  current node
 * @param  validate  Are we validating data?
 *
 */
bool ExpandFromNode(GraphReader& reader,
                    std::list<PointLL>& shape,
                    GraphId& en,
                    const GraphId& from_node,
                    std::unordered_set<std::string>& isos,
                    bool forward,
                    std::unordered_set<GraphId>& visited_nodes,
                    uint64_t& way_id,
                    const graph_tile_ptr& prev_tile,
                    GraphId prev_node,
                    GraphId current_node,
                    const RoadClass& rc,
                    bool validate) {

  auto tile = prev_tile;
  if (tile->id() != current_node.Tile_Base()) {
    tile = reader.GetGraphTile(current_node);
  }

  auto* node_info = tile->node(current_node);
  // expand from the current node
  return ExpandFromNodeInner(reader, shape, en, from_node, isos, forward, visited_nodes, way_id, tile,
                             prev_node, current_node, node_info, rc, validate);
}

bool Aggregate(GraphId& start_node,
               GraphReader& reader,
               std::list<PointLL>& shape,
               GraphId& en,
               const GraphId& from_node,
               uint64_t& way_id,
               std::unordered_set<std::string>& isos,
               const RoadClass& rc,
               bool forward,
               bool validate) {

  graph_tile_ptr tile = reader.GetGraphTile(start_node);
  std::unordered_set<GraphId> visited_nodes{start_node};
  return ExpandFromNode(reader, shape, en, from_node, isos, forward, visited_nodes, way_id, tile,
                        GraphId(), start_node, rc, validate);
}

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
      bool diff_names = false;
      bool diff_tile = false;
      bool edge_filtered = false;
      // Count of edges added for this node
      uint32_t edge_count = 0;

      // Current edge index for first edge from this node
      uint32_t edge_index = tilebuilder.directededges().size();

      // Iterate through directed edges outbound from this node
      std::vector<uint64_t> wayid;
      std::vector<RoadClass> classification;
      std::vector<GraphId> endnode;
      const NodeInfo* nodeinfo = tile->node(nodeid);
      std::string begin_node_iso = tile->admin(nodeinfo->admin_index())->country_iso();

      GraphId edgeid(nodeid.tileid(), nodeid.level(), nodeinfo->edge_index());
      for (uint32_t j = 0; j < nodeinfo->edge_count(); ++j, ++edgeid) {
        // Check if the directed edge should be included
        const DirectedEdge* directededge = tile->directededge(edgeid);
        if (!include_edge(directededge)) {
          ++n_filtered_edges;
          edge_filtered = true;
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

        // Names can be different in the forward and backward direction
        diff_names = tilebuilder.OpposingEdgeInfoDiffers(tile, directededge);

        // Get edge info, shape, and names from the old tile and add to the
        // new. Cannot use edge info offset since edges in arterial and
        // highway hierarchy can cross base tiles! Use a hash based on the
        // encoded shape plus way Id.
        bool added;
        const auto& edgeinfo = tile->edgeinfo(directededge);
        std::string encoded_shape = edgeinfo.encoded_shape();
        uint32_t w = hasher(encoded_shape + std::to_string(edgeinfo.wayid()));
        uint32_t edge_info_offset =
            tilebuilder.AddEdgeInfo(w, nodeid, directededge->endnode(), edgeinfo.wayid(),
                                    edgeinfo.mean_elevation(), edgeinfo.bike_network(),
                                    edgeinfo.speed_limit(), encoded_shape, edgeinfo.GetNames(),
                                    edgeinfo.GetTaggedValues(), edgeinfo.GetLinguisticTaggedValues(),
                                    edgeinfo.GetTypes(), added, diff_names);
        newedge.set_edgeinfo_offset(edge_info_offset);
        wayid.push_back(edgeinfo.wayid());
        classification.push_back(directededge->classification());
        endnode.push_back(directededge->endnode());

        if (directededge->endnode().tile_value() != tile->header()->graphid().tile_value()) {
          diff_tile = true;
        }

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
        // edge attributes should match), don't end at same node (no loops), no traffic signal,
        // no signs exist at the node(named_intersection), does not have different
        // names, and end node of edges are not in a different tile.
        //
        // Note: The classification check is here due to the reclassification of ferries.  Found
        // that some edges that were split at pedestrian edges had different classifications due to
        // the reclassification of ferry edges (e.g., https://www.openstreetmap.org/way/204337649)
        if (edge_filtered && edge_count == 2 && wayid[0] == wayid[1] &&
            classification[0] == classification[1] && endnode[0] != endnode[1] &&
            !nodeinfo->traffic_signal() && !nodeinfo->named_intersection() && !diff_names &&
            !diff_tile) {

          // one more check on intersection and node type.  similar to shortcuts
          bool aggregate =
              (nodeinfo->intersection() != IntersectionType::kFork &&
               nodeinfo->type() != NodeType::kGate && nodeinfo->type() != NodeType::kTollBooth &&
               nodeinfo->type() != NodeType::kTollGantry && nodeinfo->type() != NodeType::kBollard &&
               nodeinfo->type() != NodeType::kSumpBuster &&
               nodeinfo->type() != NodeType::kBorderControl);

          if (aggregate) {
            // temporarily used to check aggregating edges from this node
            node.set_mode_change(true);
            ++can_aggregate;
          }
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
  LOG_INFO("Nodes to aggregate: " + std::to_string(can_aggregate));
}

void GetAggregatedData(GraphReader& reader,
                       std::list<PointLL>& shape,
                       GraphId& en,
                       const GraphId& from_node,
                       const graph_tile_ptr& tile,
                       const DirectedEdge* directededge) {
  std::unordered_set<std::string> isos;
  bool isForward = directededge->forward();
  auto id = directededge->endnode();
  if (!isForward) {
    std::reverse(shape.begin(), shape.end());
  }

  // walk in the correct direction.
  uint64_t wayid = tile->edgeinfo(directededge).wayid();
  if (Aggregate(id, reader, shape, en, from_node, wayid, isos, directededge->classification(),
                isForward, false)) {
    aggregated++; // count the current edge
    // flip the shape back for storing in edgeinfo
    if (!isForward) {
      std::reverse(shape.begin(), shape.end());
    }
  }
}

// If we cross into another country we can't aggregate the edges
// as the access can be different in each country.  Also, bollards
// and or gates could exist at the node blocking access.
//
// Also, we need to handle islands that we created by tossing the
// pedestrian edges.  For example://
// https://www.openstreetmap.org/way/993706522
// https://www.openstreetmap.org/way/975845893
// As of 01/15/2024 there are only ~180 of these.

void ValidateData(GraphReader& reader,
                  std::list<PointLL>& shape,
                  GraphId& en,
                  std::unordered_set<GraphId>& processed_nodes,
                  std::unordered_set<uint64_t>& no_agg_ways,
                  const GraphId& from_node,
                  const graph_tile_ptr& tile,
                  const DirectedEdge* directededge) {

  // Get the tile at the end node.  Skip if node is in another tile.
  // mode_change is not set for end nodes that are in diff tiles
  if (directededge->endnode().tile_value() == tile->header()->graphid().tile_value()) {

    // original edge data.
    const auto& edgeinfo = tile->edgeinfo(directededge);
    const NodeInfo* en_info = tile->node(directededge->endnode().id());
    const NodeInfo* sn_info = tile->node(from_node);

    if (en_info->mode_change()) {

      // If this edge has special attributes, then we can't aggregate
      if (!CanAggregate(directededge)) {
        processed_nodes.insert(directededge->endnode());
        no_agg_ways.insert(edgeinfo.wayid());
        return;
      }

      std::unordered_set<std::string> isos;
      bool isForward = directededge->forward();
      auto id = directededge->endnode();

      isos.insert(tile->admin(sn_info->admin_index())->country_iso()); // start node
      isos.insert(tile->admin(en_info->admin_index())->country_iso()); // end node

      if (isos.size() > 1) { // already in diff country
        processed_nodes.insert(directededge->endnode());
        return;
      }

      // walk in the correct direction.
      uint64_t wayid = edgeinfo.wayid();
      if (!Aggregate(id, reader, shape, en, from_node, wayid, isos, directededge->classification(),
                     isForward, true)) {
        // LOG_WARN("ValidateData - failed to validate node.  Will not aggregate.");
        // for debugging only
        // std::cout << "End node: " << directededge->endnode().value << " WayId: " <<
        // edgeinfo.wayid()
        //           << std::endl;

        if (wayid == 0) { // This edge has special attributes, we can't aggregate
          no_agg_ways.insert(edgeinfo.wayid());
        }

        processed_nodes.insert(directededge->endnode()); // turn off so that we don't fail
      } else if (isos.size() > 1) {                      // in diff country
        processed_nodes.insert(directededge->endnode());
      }
    }
  }
}

void AggregateTiles(GraphReader& reader, std::unordered_map<GraphId, GraphId>& old_to_new) {

  LOG_INFO("Validating edges for aggregation");
  // Iterate through all tiles in the local level
  auto local_tiles = reader.GetTileSet(TileHierarchy::levels().back().level);
  for (const auto& tile_id : local_tiles) {
    // Get the graph tile. Read from this tile to create the new tile.
    graph_tile_ptr tile = reader.GetGraphTile(tile_id);
    assert(tile);

    std::unordered_set<GraphId> processed_nodes;
    std::unordered_set<uint64_t> no_agg_ways;
    processed_nodes.reserve(tile->header()->nodecount());
    no_agg_ways.reserve(tile->header()->directededgecount());

    GraphId nodeid = GraphId(tile_id.tileid(), tile_id.level(), 0);
    for (uint32_t i = 0; i < tile->header()->nodecount(); ++i, ++nodeid) {
      const NodeInfo* nodeinfo = tile->node(i);
      uint32_t idx = nodeinfo->edge_index();
      for (uint32_t j = 0; j < nodeinfo->edge_count(); j++, idx++) {
        const DirectedEdge* directededge = tile->directededge(idx);
        if (processed_nodes.find(nodeid) == processed_nodes.end()) {
          GraphId en = directededge->endnode();
          std::list<PointLL> shape;
          // check if we can aggregate the edges at this node.
          ValidateData(reader, shape, en, processed_nodes, no_agg_ways, nodeid, tile, directededge);
        }
      }
    }

    // Now loop again double checking the ways.
    nodeid = GraphId(tile_id.tileid(), tile_id.level(), 0);
    for (uint32_t i = 0; i < tile->header()->nodecount(); ++i, ++nodeid) {
      const NodeInfo* nodeinfo = tile->node(i);
      uint32_t idx = nodeinfo->edge_index();
      for (uint32_t j = 0; j < nodeinfo->edge_count(); j++, idx++) {
        const DirectedEdge* directededge = tile->directededge(idx);
        if (no_agg_ways.find(tile->edgeinfo(directededge).wayid()) != no_agg_ways.end()) {
          processed_nodes.insert(directededge->endnode());
        }
      }
    }

    // Create a new tile builder
    GraphTileBuilder tilebuilder(reader.tile_dir(), tile_id, false);
    std::vector<NodeInfo> nodes;

    // Copy edges (they do not change)
    std::vector<DirectedEdge> directededges;
    size_t n = tile->header()->directededgecount();
    directededges.reserve(n);
    const DirectedEdge* orig_edges = tile->directededge(0);
    std::copy(orig_edges, orig_edges + n, std::back_inserter(directededges));

    nodeid = GraphId(tile_id.tileid(), tile_id.level(), 0);
    for (uint32_t i = 0; i < tile->header()->nodecount(); ++i, ++nodeid) {
      NodeInfo nodeinfo = tilebuilder.node(i);
      bool found = (processed_nodes.find(nodeid) != processed_nodes.end());

      // We can not aggregate at this node.  Turn off the mode change(aggregation) bit
      if (found) {
        nodeinfo.set_mode_change(false);
      }
      // Add the node to the local list
      nodes.emplace_back(std::move(nodeinfo));
    }
    tilebuilder.Update(nodes, directededges);

    if (reader.OverCommitted()) {
      reader.Trim();
    }
  }

  LOG_INFO("Aggregating edges");
  reader.Clear();
  // Iterate through all tiles in the local level
  local_tiles = reader.GetTileSet(TileHierarchy::levels().back().level);
  // Iterate through all tiles in the local level
  for (const auto& tile_id : local_tiles) {
    // Create a new tilebuilder - should copy header information
    GraphTileBuilder tilebuilder(reader.tile_dir(), tile_id, false);

    // Get the graph tile. Read from this tile to create the new tile.
    graph_tile_ptr tile = reader.GetGraphTile(tile_id);
    assert(tile);

    std::hash<std::string> hasher;
    GraphId nodeid(tile_id.tileid(), tile_id.level(), 0);
    for (uint32_t i = 0; i < tile->header()->nodecount(); ++i, ++nodeid) {
      bool diff_names = false;

      // Count of edges added for this node
      uint32_t edge_count = 0;

      // Current edge index for first edge from this node
      uint32_t edge_index = tilebuilder.directededges().size();

      // Iterate through directed edges outbound from this node
      std::vector<uint64_t> wayid;
      std::vector<GraphId> endnode;
      const NodeInfo* nodeinfo = tile->node(nodeid);

      // Nodes marked with mode_change = true are tossed.
      if (nodeinfo->mode_change()) {
        continue;
      }

      GraphId edgeid(nodeid.tileid(), nodeid.level(), nodeinfo->edge_index());

      for (uint32_t j = 0; j < nodeinfo->edge_count(); ++j, ++edgeid) {
        // Check if the directed edge should be included
        const DirectedEdge* directededge = tile->directededge(edgeid);

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

        // Names can be different in the forward and backward direction
        diff_names = tilebuilder.OpposingEdgeInfoDiffers(tile, directededge);

        const auto& edgeinfo = tile->edgeinfo(directededge);
        std::string encoded_shape = edgeinfo.encoded_shape();
        std::list<PointLL> shape = valhalla::midgard::decode7<std::list<PointLL>>(encoded_shape);

        // Aggregate if end node is marked and in same tile
        bool aggregated = false;
        GraphId en = directededge->endnode();

        if (en.tile_value() == tile_id) {
          if (tile->node(en.id())->mode_change()) {
            GetAggregatedData(reader, shape, en, nodeid, tile, directededge);
            newedge.set_endnode(en);
            aggregated = true;
          }
        }

        // Hammerhead specific.  bike network not saved to edgeinfo
        bool added;
        encoded_shape = encode7(shape);
        uint32_t w = hasher(encoded_shape + std::to_string(edgeinfo.wayid()));
        uint32_t edge_info_offset =
            tilebuilder.AddEdgeInfo(w, nodeid, en, edgeinfo.wayid(), edgeinfo.mean_elevation(),
                                    edgeinfo.bike_network(), edgeinfo.speed_limit(), encoded_shape,
                                    edgeinfo.GetNames(), edgeinfo.GetTaggedValues(),
                                    edgeinfo.GetLinguisticTaggedValues(), edgeinfo.GetTypes(), added,
                                    diff_names);
        newedge.set_edgeinfo_offset(edge_info_offset);

        // Update length and curvature if the edge was aggregated
        if (aggregated) {
          newedge.set_length(valhalla::midgard::length(shape));
          newedge.set_curvature(compute_curvature(shape));
        }

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

  LOG_INFO("Aggregated " + std::to_string(aggregated) + " directededges out of " +
           std::to_string(n_original_edges));
}

/**
 * Update end nodes of all directed edges.
 * @param  reader  Graph reader.
 * @param  old_to_new  Map of original node Ids to new nodes Ids (after filtering).
 */
void UpdateEndNodes(GraphReader& reader, std::unordered_map<GraphId, GraphId>& old_to_new) {
  LOG_INFO("Update end nodes of directed edges");
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
        std::cout << std::to_string(edge->endnode().value) << " "
                  << std::to_string(tile->edgeinfo(edge).wayid()) << std::endl;
      } else {
        end_node = iter->second;
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

/**
 * Update Opposing Edge Index of all directed edges.
 * @param  reader  Graph reader.
 */
void UpdateOpposingEdgeIndex(GraphReader& reader) {
  LOG_INFO("Update Opposing Edge Index of directed edges");

  // Iterate through all tiles in the local level
  auto local_tiles = reader.GetTileSet(TileHierarchy::levels().back().level);
  for (const auto& tile_id : local_tiles) {
    GraphTileBuilder tilebuilder(reader.tile_dir(), tile_id, false);

    // Get the graph tile. Read from this tile to create the new tile.
    graph_tile_ptr tile = reader.GetGraphTile(tile_id);
    assert(tile);

    // Copy nodes (they do not change)
    std::vector<NodeInfo> nodes;
    size_t n = tile->header()->nodecount();
    nodes.reserve(n);
    const NodeInfo* orig_nodes = tile->node(0);
    std::copy(orig_nodes, orig_nodes + n, std::back_inserter(nodes));

    // Iterate through all directed edges - update end nodes
    std::vector<DirectedEdge> directededges;

    GraphId nodeid(tile_id.tileid(), tile_id.level(), 0);
    for (uint32_t i = 0; i < tile->header()->nodecount(); ++i, ++nodeid) {
      const NodeInfo* nodeinfo = tile->node(nodeid);
      GraphId edgeid(nodeid.tileid(), nodeid.level(), nodeinfo->edge_index());
      for (uint32_t j = 0; j < nodeinfo->edge_count(); ++j, ++edgeid) {
        // Check if the directed edge should be included
        const DirectedEdge* edge = tile->directededge(edgeid);

        // Copy the edge to the directededges vector and update the end node
        directededges.push_back(*edge);
        DirectedEdge& new_edge = directededges.back();

        // Get the tile at the end node
        graph_tile_ptr endnodetile;
        if (tile->id() == edge->endnode().Tile_Base()) {
          endnodetile = tile;
        } else {
          endnodetile = reader.GetGraphTile(edge->endnode());
        }

        // Set the opposing index on the local level
        new_edge.set_opp_local_idx(GetOpposingEdgeIndex(endnodetile, nodeid, tile, *edge));
      }
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

  // Update end nodes. Clear the GraphReader cache first.
  reader.Clear();
  UpdateEndNodes(reader, old_to_new);

  reader.Clear();
  old_to_new.clear();
  AggregateTiles(reader, old_to_new);

  // Update end nodes. Clear the GraphReader cache first.
  reader.Clear();
  UpdateEndNodes(reader, old_to_new);

  // Update Opposing Edge Index. Clear the GraphReader cache first.
  reader.Clear();
  UpdateOpposingEdgeIndex(reader);

  LOG_INFO("Done GraphFilter");
}

} // namespace mjolnir
} // namespace valhalla
