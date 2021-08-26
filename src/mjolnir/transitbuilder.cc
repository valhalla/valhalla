#include "mjolnir/transitbuilder.h"
#include "mjolnir/graphtilebuilder.h"

#include <fstream>
#include <future>
#include <iostream>
#include <list>
#include <mutex>
#include <queue>
#include <set>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <boost/algorithm/string.hpp>

#include "baldr/datetime.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/distanceapproximator.h"
#include "midgard/logging.h"
#include "midgard/util.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::baldr::DateTime;
using namespace valhalla::mjolnir;

namespace {

struct OSMConnectionEdge {
  GraphId osm_node;
  GraphId stop_node;
  double length;
  uint64_t wayid;
  std::vector<std::string> names;
  std::vector<std::string> tagged_values;
  std::vector<std::string> pronunciations;
  std::list<PointLL> shape;

  OSMConnectionEdge(const GraphId& f,
                    const GraphId& t,
                    const double l,
                    const uint64_t w,
                    const std::vector<std::string>& n,
                    const std::list<PointLL>& s)
      : osm_node(f), stop_node(t), length(l), wayid(w), names(n), shape(s) {
  }

  // operator < for sorting
  bool operator<(const OSMConnectionEdge& other) const {
    if (osm_node.tileid() == other.osm_node.tileid()) {
      return osm_node.id() < other.osm_node.id();
    } else {
      return osm_node.tileid() < other.osm_node.tileid();
    }
  }
};

// Struct to hold stats information during each threads work
struct builder_stats {
  uint32_t stats;

  // Accumulate stats from all threads
  void operator()(const builder_stats& other) {
    stats += other.stats;
  }
};

// Converts a transit stop GraphId on local level to be on the transit level
GraphId GetGraphId(const GraphId& nodeid, const std::unordered_set<GraphId>& tiles) {
  auto t = tiles.find(nodeid.Tile_Base());
  if (t == tiles.end()) {
    return GraphId(); // Invalid graph Id
  } else {
    return {nodeid.tileid(), nodeid.level() + 1, nodeid.id()};
  }
}

void ConnectToGraph(GraphTileBuilder& tilebuilder_local,
                    GraphTileBuilder& tilebuilder_transit,
                    const graph_tile_ptr& tile,
                    GraphReader& reader_transit_level,
                    std::mutex& lock,
                    const std::unordered_set<GraphId>& tiles,
                    const std::vector<OSMConnectionEdge>& connection_edges,
                    const std::unordered_map<GraphId, Traversability>& egress_traversability) {
  auto t1 = std::chrono::high_resolution_clock::now();

  // Move existing nodes and directed edge builder vectors and clear the lists
  std::vector<NodeInfo> currentnodes(std::move(tilebuilder_local.nodes()));
  uint32_t nodecount = currentnodes.size();
  tilebuilder_local.nodes().clear();
  std::vector<DirectedEdge> currentedges(std::move(tilebuilder_local.directededges()));
  tilebuilder_local.directededges().clear();

  // Get the directed edge index of the first sign. If no signs are
  // present in this tile set a value > number of directed edges
  uint32_t signidx = 0;
  uint32_t nextsignidx = (tilebuilder_local.header()->signcount() > 0)
                             ? tilebuilder_local.sign(0).index()
                             : currentedges.size() + 1;
  uint32_t signcount = tilebuilder_local.header()->signcount();

  // Get the directed edge index of the first access restriction.
  uint32_t residx = 0;
  uint32_t nextresidx = (tilebuilder_local.header()->access_restriction_count() > 0)
                            ? tilebuilder_local.accessrestriction(0).edgeindex()
                            : currentedges.size() + 1;
  ;
  uint32_t rescount = tilebuilder_local.header()->access_restriction_count();

  // Iterate through the nodes - add back any stored edges and insert any
  // connections from a node to a transit stop. Update each nodes edge index.
  uint32_t nodeid = 0;
  uint32_t added_edges = 0;
  uint32_t connedges = 0;
  for (auto& nb : currentnodes) {
    // Copy existing directed edges from this node and update any signs using
    // the directed edge index
    size_t edge_index = tilebuilder_local.directededges().size();
    for (uint32_t i = 0, idx = nb.edge_index(); i < nb.edge_count(); i++, idx++) {
      tilebuilder_local.directededges().emplace_back(std::move(currentedges[idx]));

      // Update any signs that use this idx - increment their index by the
      // number of added edges
      while (idx == nextsignidx && signidx < signcount) {
        if (!currentedges[idx].sign()) {
          LOG_ERROR("Signs for this index but directededge says no sign");
        }
        tilebuilder_local.sign_builder(signidx).set_index(idx + added_edges);

        // Increment to the next sign and update nextsignidx
        signidx++;
        nextsignidx = (signidx >= signcount) ? 0 : tilebuilder_local.sign(signidx).index();
      }

      // Add any restrictions that use this idx - increment their index by the
      // number of added edges
      while (idx == nextresidx && residx < rescount) {
        if (!currentedges[idx].access_restriction()) {
          LOG_ERROR("Access restrictions for this index but directededge says none");
        }
        tilebuilder_local.accessrestriction_builder(residx).set_edgeindex(idx + added_edges);

        // Increment to the next restriction and update nextresidx
        residx++;
        nextresidx =
            (residx >= rescount) ? 0 : tilebuilder_local.accessrestriction(residx).edgeindex();
      }
    }

    // Add directed edges for any connections from the OSM node
    // to an egress
    // level 2
    graph_tile_ptr end_tile;
    while (added_edges < connection_edges.size() &&
           connection_edges[added_edges].osm_node.id() == nodeid) {
      const OSMConnectionEdge& conn = connection_edges[added_edges];

      // Get the transit node Graph Id
      GraphId endnode = GetGraphId(conn.stop_node, tiles);
      if (!endnode.Is_Valid()) {
        continue;
      }

      if (!end_tile || (end_tile->id().Tile_Base() != endnode.Tile_Base())) {
        lock.lock();
        end_tile = reader_transit_level.GetGraphTile(endnode);
        lock.unlock();
      }
      // use the access from the transit end node
      const auto& tc_access = end_tile->node(endnode)->access();
      DirectedEdge directededge;
      directededge.set_endnode(endnode);
      directededge.set_length(conn.length);
      directededge.set_use(Use::kTransitConnection);
      directededge.set_speed(5);
      directededge.set_classification(RoadClass::kServiceOther);
      directededge.set_localedgeidx(tilebuilder_local.directededges().size() - edge_index);

      auto e_traversability = egress_traversability.find(conn.stop_node);
      if (e_traversability != egress_traversability.end()) {
        if (e_traversability->second == Traversability::kBoth) {
          directededge.set_forwardaccess(tc_access);
          directededge.set_reverseaccess(tc_access);
        } else if (e_traversability->second == Traversability::kForward) {
          directededge.set_forwardaccess(tc_access);
        } else if (e_traversability->second == Traversability::kBackward) {
          directededge.set_reverseaccess(tc_access);
        }
      }
      directededge.set_named(conn.names.size() > 0 || conn.tagged_values.size() > 0);

      // Add edge info to the tile and set the offset in the directed edge
      bool added = false;
      uint32_t edge_info_offset =
          tilebuilder_local.AddEdgeInfo(0, conn.osm_node, endnode, conn.wayid, 0, 0, 0, conn.shape,
                                        conn.names, conn.tagged_values, conn.pronunciations, 0,
                                        added);
      directededge.set_edgeinfo_offset(edge_info_offset);
      directededge.set_forward(true);
      tilebuilder_local.directededges().emplace_back(std::move(directededge));

      LOG_DEBUG("Add conn from OSM to stop: ei offset = " + std::to_string(edge_info_offset));

      // increment to next connection edge
      connedges++;
      added_edges++;
    }

    // Add the node and directed edges
    nb.set_edge_index(edge_index);
    nb.set_edge_count(tilebuilder_local.directededges().size() - edge_index);
    tilebuilder_local.nodes().emplace_back(std::move(nb));
    nodeid++;
  }

  // Some validation here...
  if (added_edges != connection_edges.size()) {
    LOG_ERROR("Part 1: Added " + std::to_string(added_edges) + " but there are " +
              std::to_string(connection_edges.size()) + " connections");
  }

  // Move existing nodes and directed edge builder vectors and clear the lists
  currentnodes = tilebuilder_transit.nodes();
  nodecount = currentnodes.size();
  tilebuilder_transit.nodes().clear();
  currentedges = tilebuilder_transit.directededges();
  tilebuilder_transit.directededges().clear();

  // Get the directed edge index of the first sign. If no signs are
  // present in this tile set a value > number of directed edges
  signidx = 0;
  nextsignidx = (tilebuilder_transit.header()->signcount() > 0) ? tilebuilder_transit.sign(0).index()
                                                                : currentedges.size() + 1;
  signcount = tilebuilder_transit.header()->signcount();

  // Get the directed edge index of the first access restriction.
  residx = 0;
  nextresidx = (tilebuilder_transit.header()->access_restriction_count() > 0)
                   ? tilebuilder_transit.accessrestriction(0).edgeindex()
                   : currentedges.size() + 1;
  ;
  rescount = tilebuilder_transit.header()->access_restriction_count();

  // Iterate through the nodes - add back any stored edges and insert any
  // connections from a transit stop to a node.. Update each nodes edge index.
  added_edges = 0;
  connedges = 0;
  for (auto& nb : currentnodes) {
    // Copy existing directed edges from this node and update any signs using
    // the directed edge index
    size_t edge_index = tilebuilder_transit.directededges().size();

    // Temporary - kill transit edges above some number so we allow
    // transit connections. Should be based on kMaxEdgesPerNode
    constexpr uint32_t kMaxTransitEdges = 120;
    if (nb.edge_count() > kMaxTransitEdges) {
      LOG_ERROR("More than 120 transit edges");
    }

    // Reserve size for several connection edges
    uint32_t ec = (nb.edge_count() < kMaxTransitEdges) ? nb.edge_count() : kMaxTransitEdges;
    for (uint32_t i = 0, idx = nb.edge_index(); i < ec; i++, idx++) {
      tilebuilder_transit.directededges().emplace_back(std::move(currentedges[idx]));
    }

    // Add directed edges for any connections from an egress
    // to the osm node
    // level 3
    bool admin_set = false;
    for (const auto& conn : connection_edges) {
      if (conn.stop_node.id() == nb.stop_index()) {

        // Get the Valhalla graphId of the origin node (transit stop)
        GraphId origin_node = GetGraphId(conn.stop_node, tiles);
        if (!origin_node.Is_Valid()) {
          continue;
        }
        // use the access from the transit node
        const auto& tc_access = nb.access();
        DirectedEdge directededge;
        directededge.set_endnode(conn.osm_node);
        directededge.set_length(conn.length);
        directededge.set_use(Use::kTransitConnection);
        directededge.set_speed(5);
        directededge.set_classification(RoadClass::kServiceOther);
        directededge.set_localedgeidx(tilebuilder_transit.directededges().size() - edge_index);

        auto e_traversability = egress_traversability.find(conn.stop_node);
        if (e_traversability != egress_traversability.end()) {
          if (e_traversability->second == Traversability::kBoth) {
            directededge.set_forwardaccess(tc_access);
            directededge.set_reverseaccess(tc_access);
            // exit only.  if backward is true then set forward access as we are going from
            // the egress to the osm node.
          } else if (e_traversability->second == Traversability::kBackward) {
            directededge.set_forwardaccess(tc_access);
            // enter only.  if forward is true then set backward access as we are going from
            // the egress to the osm node.
          } else if (e_traversability->second == Traversability::kForward) {
            directededge.set_reverseaccess(tc_access);
          }
        }
        directededge.set_named(conn.names.size() > 0 || conn.tagged_values.size() > 0);

        // Add edge info to the tile and set the offset in the directed edge
        bool added = false;
        std::list<PointLL> r_shape = conn.shape;
        std::reverse(r_shape.begin(), r_shape.end());
        uint32_t edge_info_offset =
            tilebuilder_transit.AddEdgeInfo(0, origin_node, conn.osm_node, conn.wayid, 0, 0, 0,
                                            r_shape, conn.names, conn.tagged_values,
                                            conn.pronunciations, 0, added);
        LOG_DEBUG("Add conn from stop to OSM: ei offset = " + std::to_string(edge_info_offset));
        directededge.set_edgeinfo_offset(edge_info_offset);
        directededge.set_forward(true);

        // set the admin index from the first de.
        if (!admin_set) {
          const NodeInfo* node = tile->node(conn.osm_node);
          const auto& admin = tile->admininfo(node->admin_index());
          nb.set_admin_index(tilebuilder_transit.AddAdmin(admin.country_text(), admin.state_text(),
                                                          admin.country_iso(), admin.state_iso()));
          admin_set = true;
        }

        // TODO - this should use kMaxEdgesPerNode
        uint32_t n = tilebuilder_transit.directededges().size() - edge_index;
        if (n < 127) {
          tilebuilder_transit.directededges().emplace_back(std::move(directededge));
        } else {
          LOG_ERROR("Could not add transit connection edge!");
        }

        LOG_DEBUG("Add conn from OSM to stop: ei offset = " + std::to_string(edge_info_offset));

        // increment to next connection edge
        connedges++;
        added_edges++;
      }
    }
    // Add the node and directed edges
    nb.set_edge_index(edge_index);

    // reset the access to defaults.
    nb.set_access((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
    nb.set_edge_count(tilebuilder_transit.directededges().size() - edge_index);
    tilebuilder_transit.nodes().emplace_back(std::move(nb));
  }

  // Some validation here...
  if (added_edges != connection_edges.size()) {
    LOG_ERROR("Part 1: Added " + std::to_string(added_edges) + " but there are " +
              std::to_string(connection_edges.size()) + " connections");
  }

  // Log the number of added nodes and edges
  auto t2 = std::chrono::high_resolution_clock::now();
  LOG_INFO("Tile " + std::to_string(tilebuilder_local.header()->graphid().tileid()) + ": added " +
           std::to_string(connedges) + " connection edges, " + std::to_string(nodecount) +
           " nodes. time = " +
           std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()) +
           " ms");
}

// Fallback to find connection edges from the transit stop to an OSM edge.
void FindOSMConnection(const PointLL& stop_ll,
                       GraphReader& reader_local_level,
                       std::mutex& lock,
                       std::vector<std::string>& names,
                       uint64_t& wayid,
                       GraphId& startnode,
                       GraphId& endnode,
                       std::vector<PointLL>& closest_shape,
                       std::tuple<PointLL, PointLL::first_type, int>& closest) {
  // Let's try a fallback.  Use approximator to find the closest edge.
  // We do this because the associated way could have been deleted from the
  // OSM data, but we may have not updated the stops yet in TransitLand.
  double mindist = 10000000.0;
  double rm = kMetersPerKm; // one km
  double mr2 = rm * rm;

  const auto& tiles = TileHierarchy::levels().back().tiles;
  const auto kTransitLatDeg = kMetersPerKm / kMetersPerDegreeLat; // one km radius

  // Get a list of tiles required for a node search within this radius
  double lngdeg = (rm / DistanceApproximator<PointLL>::MetersPerLngDegree(stop_ll.lat()));
  AABB2<PointLL> bbox(Point2d(stop_ll.lng() - lngdeg, stop_ll.lat() - kTransitLatDeg),
                      Point2d(stop_ll.lng() + lngdeg, stop_ll.lat() + kTransitLatDeg));
  std::vector<int32_t> tilelist = tiles.TileList(bbox);

  const auto& local_level = TileHierarchy::levels().back().level;
  for (auto t : tilelist) {
    // Check all the nodes within the tile. Skip if tile has no nodes
    lock.lock();
    auto newtile = reader_local_level.GetGraphTile(GraphId(t, local_level, 0));
    lock.unlock();
    if (!newtile || newtile->header()->nodecount() == 0) {
      continue;
    }

    // Use distance approximator for all distance checks
    PointLL base_ll = newtile->header()->base_ll();
    DistanceApproximator<PointLL> approximator(stop_ll);
    for (uint32_t i = 0; i < newtile->header()->nodecount(); i++) {
      const NodeInfo* node = newtile->node(i);
      // Check if within radius
      if (approximator.DistanceSquared(node->latlng(base_ll)) < mr2) {
        for (uint32_t j = 0, n = node->edge_count(); j < n; j++) {
          const DirectedEdge* directededge = newtile->directededge(node->edge_index() + j);
          auto edgeinfo = newtile->edgeinfo(directededge);

          // Get shape and find closest point
          auto this_shape = edgeinfo.shape();

          // Reverse the shape if directed edge is not the forward direction
          // along the shape
          if (!directededge->forward()) {
            std::reverse(this_shape.begin(), this_shape.end());
          }

          auto this_closest = stop_ll.ClosestPoint(this_shape);
          // Get names
          names = edgeinfo.GetNames();

          if (std::get<1>(this_closest) < mindist) {
            // use the new wayid

            wayid = edgeinfo.wayid();
            startnode = {newtile->header()->graphid().tileid(), newtile->header()->graphid().level(),
                         i};
            endnode = directededge->endnode();
            mindist = std::get<1>(this_closest);
            closest = this_closest;
            closest_shape = this_shape;
          }
        }
      }
    }
  }
}

// Add connection edges from the transit stop to an OSM edge
void AddOSMConnection(const GraphId& transit_stop_node,
                      const NodeInfo* transit_node,
                      const std::string& stop_name,
                      const graph_tile_ptr& tile,
                      GraphReader& reader_local_level,
                      std::mutex& lock,
                      std::vector<OSMConnectionEdge>& connection_edges) {
  assert(tile);
  const PointLL& stop_ll = transit_node->latlng(tile->header()->base_ll());
  uint64_t wayid = transit_node->connecting_wayid();

  double mindist = 10000000.0;
  uint32_t edgelength = 0;
  GraphId startnode, endnode;
  std::vector<PointLL> closest_shape;
  std::tuple<PointLL, PointLL::first_type, int> closest;
  std::vector<std::string> names;
  for (uint32_t i = 0; i < tile->header()->nodecount(); i++) {
    const NodeInfo* node = tile->node(i);
    for (uint32_t j = 0, n = node->edge_count(); j < n; j++) {
      const DirectedEdge* directededge = tile->directededge(node->edge_index() + j);
      auto edgeinfo = tile->edgeinfo(directededge);

      if (edgeinfo.wayid() == wayid) {

        // Get shape and find closest point
        auto this_shape = edgeinfo.shape();

        if (!directededge->forward()) {
          std::reverse(this_shape.begin(), this_shape.end());
        }
        auto this_closest = stop_ll.ClosestPoint(this_shape);
        // Get names
        names = edgeinfo.GetNames();

        if (std::get<1>(this_closest) < mindist) {
          startnode = {tile->header()->graphid().tileid(), tile->header()->graphid().level(), i};
          endnode = directededge->endnode();
          mindist = std::get<1>(this_closest);
          closest = this_closest;
          closest_shape = this_shape;
          edgelength = directededge->length();
        }
      }
    }
  }

  // Check for invalid tile Ids
  if (!startnode.Is_Valid() && !endnode.Is_Valid()) {
    FindOSMConnection(stop_ll, reader_local_level, lock, names, wayid, startnode, endnode,
                      closest_shape, closest);

    // Check for invalid tile Ids...are we still no good?
    if (!startnode.Is_Valid() && !endnode.Is_Valid()) {
      const AABB2<PointLL>& aabb = tile->BoundingBox();

      LOG_ERROR("No closest edge found for this stop: " + stop_name +
                " way Id = " + std::to_string(wayid) + " LL= " + std::to_string(stop_ll.lat()) + "," +
                std::to_string(stop_ll.lng()) + " tile " + std::to_string(aabb.minx()) + ", " +
                std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ", " +
                std::to_string(aabb.maxy()));
      return;
    }
  }

  LOG_DEBUG("edge found for this stop: " + stop_name + " way Id = " + std::to_string(wayid));

  // Check if stop is in same tile as the start node
  uint32_t conn_count = 0;
  double length = 0.0;
  if (transit_stop_node.Tile_Base() == startnode.Tile_Base()) {
    // Add shape from node along the edge until the closest point, then add
    // the closest point and a straight line to the stop lat,lng
    std::list<PointLL> shape;
    for (int i = 0; i <= std::get<2>(closest); i++) {
      shape.push_back(closest_shape[i]);
    }
    shape.push_back(std::get<0>(closest));
    shape.push_back(stop_ll);
    length = std::max(1.0, valhalla::midgard::length(shape));

    // Add connection to start node
    connection_edges.push_back({startnode, transit_stop_node, length, wayid, names, shape});
    conn_count++;
  }

  // Check if stop is in same tile as end node
  double length2 = 0.0;
  if (transit_stop_node.Tile_Base() == endnode.Tile_Base()) {
    // Add connection to end node
    if (startnode.tileid() == endnode.tileid()) {
      // Add shape from the end to closest point on edge
      std::list<PointLL> shape2;
      for (int32_t i = closest_shape.size() - 1; i > std::get<2>(closest); i--) {
        shape2.push_back(closest_shape[i]);
      }
      shape2.push_back(std::get<0>(closest));
      shape2.push_back(stop_ll);
      length2 = std::max(1.0, valhalla::midgard::length(shape2));

      // Add connection to the end node
      connection_edges.push_back({endnode, transit_stop_node, length2, wayid, names, shape2});
      conn_count++;
    }
  }

  // Check for errors
  if (length != 0.0f && length2 != 0.0 && (length + length2) < edgelength - 1) {
    LOG_ERROR("EdgeLength= " + std::to_string(edgelength) +
              " < connection lengths: " + std::to_string(length) + "," + std::to_string(length2) +
              " when connecting to stop " + stop_name);
  }
  if (conn_count == 0) {
    LOG_ERROR("Stop " + stop_name + " has no connections to OSM!" +
              " Start Node Tile: " + std::to_string(startnode.tileid()) +
              " End Node Tile: " + std::to_string(endnode.tileid()));
  }
  UNUSED(stop_name);
}

// We make sure to lock on reading and writing since tiles are now being
// written. Also lock on queue access since shared by different threads.
void build(const boost::property_tree::ptree& pt,
           std::mutex& lock,
           const std::unordered_set<GraphId>& tiles,
           std::unordered_set<GraphId>::const_iterator tile_start,
           std::unordered_set<GraphId>::const_iterator tile_end,
           std::promise<builder_stats>& results) {
  // GraphReader for local level and for transit level.
  GraphReader reader_local_level(pt);
  GraphReader reader_transit_level(pt);

  // Iterate through the tiles in the queue and find any that include stops
  for (; tile_start != tile_end; ++tile_start) {
    // Get the next tile Id from the queue and get a tile builder
    if (reader_local_level.OverCommitted()) {
      reader_local_level.Trim();
    }

    if (reader_transit_level.OverCommitted()) {
      reader_transit_level.Trim();
    }

    GraphId tile_id = tile_start->Tile_Base();

    // Get Valhalla tile - get a read only instance for reference and
    // a writeable instance (deserialize it so we can add to it)
    lock.lock();
    graph_tile_ptr local_tile = reader_local_level.GetGraphTile(tile_id);
    GraphTileBuilder tilebuilder_local(reader_local_level.tile_dir(), tile_id, true);

    GraphId transit_tile_id = GraphId(tile_id.tileid(), tile_id.level() + 1, tile_id.id());
    graph_tile_ptr transit_tile = reader_transit_level.GetGraphTile(transit_tile_id);
    GraphTileBuilder tilebuilder_transit(reader_transit_level.tile_dir(), transit_tile_id, true);

    lock.unlock();

    // Iterate through all the transit tiles and form connections from
    // Iterate through stops and form connections to OSM network. Each
    // stop connects to 1 or 2 OSM nodes along the closest OSM way.
    // TODO - future - how to handle connections that reach nodes
    // outside the tile - may have to move this outside the tile
    // iteration...?
    // TODO - handle a list of connections/egrees points
    // TODO - what if we split the edge and insert a node?
    GraphId transit_stop_node(tile_id.tileid(), tile_id.level(), 0);
    std::vector<OSMConnectionEdge> connection_edges;
    std::unordered_map<GraphId, Traversability> egress_traversability;
    for (uint32_t i = 0; i < transit_tile->header()->nodecount(); i++, ++transit_stop_node) {
      const NodeInfo* transit_stop = transit_tile->node(i);

      if (transit_stop->type() == NodeType::kTransitEgress) {
        auto ts = transit_tile->GetTransitStop(transit_stop->stop_index());
        std::string stop_name = transit_tile->GetName(ts->name_offset());
        egress_traversability[transit_stop_node] = ts->traversability();

        // Form connections to the stop
        // TODO - deal with station hierarchy (only connect egress locations)
        AddOSMConnection(transit_stop_node, transit_stop, stop_name, local_tile, reader_local_level,
                         lock, connection_edges);
      }
    }

    // this happens when you are running against small extracts...no work to be done.
    if (connection_edges.size() == 0) {
      continue;
    }

    // Sort the connection edges
    std::sort(connection_edges.begin(), connection_edges.end());

    // Connect the transit graph to the route graph
    ConnectToGraph(tilebuilder_local, tilebuilder_transit, local_tile, reader_transit_level, lock,
                   tiles, connection_edges, egress_traversability);

    // Write the new file
    lock.lock();
    tilebuilder_local.StoreTileData();
    tilebuilder_transit.StoreTileData();
    lock.unlock();
  }

  // Send back the statistics
  results.set_value({});
}

} // namespace

namespace valhalla {
namespace mjolnir {

// Add transit to the graph
void TransitBuilder::Build(const boost::property_tree::ptree& pt) {

  auto t1 = std::chrono::high_resolution_clock::now();
  std::unordered_set<GraphId> tiles;

  // Bail if nothing
  auto hierarchy_properties = pt.get_child("mjolnir");
  auto transit_dir = hierarchy_properties.get_optional<std::string>("transit_dir");
  if (!transit_dir || !filesystem::exists(*transit_dir) || !filesystem::is_directory(*transit_dir)) {
    LOG_INFO("Transit directory not found. Transit will not be added.");
    return;
  }

  // Get a list of tiles that are on both level 2 (local) and level 3 (transit)
  transit_dir->push_back(filesystem::path::preferred_separator);
  GraphReader reader(hierarchy_properties);
  auto local_level = TileHierarchy::levels().back().level;
  if (filesystem::is_directory(*transit_dir + std::to_string(local_level + 1) +
                               filesystem::path::preferred_separator)) {
    filesystem::recursive_directory_iterator transit_file_itr(
        *transit_dir + std::to_string(local_level + 1) + filesystem::path::preferred_separator),
        end_file_itr;
    for (; transit_file_itr != end_file_itr; ++transit_file_itr) {
      if (filesystem::is_regular_file(transit_file_itr->path()) &&
          transit_file_itr->path().string().find(".gph") ==
              (transit_file_itr->path().string().size() - 4)) {
        auto graph_id = GraphTile::GetTileId(transit_file_itr->path().string());
        GraphId local_graph_id(graph_id.tileid(), graph_id.level() - 1, graph_id.id());
        if (reader.DoesTileExist(local_graph_id)) {
          graph_tile_ptr tile = reader.GetGraphTile(local_graph_id);
          tiles.emplace(local_graph_id);
          const std::string destination_path = pt.get<std::string>("mjolnir.tile_dir") +
                                               filesystem::path::preferred_separator +
                                               GraphTile::FileSuffix(graph_id);
          filesystem::path root = destination_path;
          root.replace_filename("");
          // Make sure the directory exists on the system and copy to the tile_dir
          if (!filesystem::exists(root)) {
            filesystem::create_directories(root);
          }
          std::ifstream in(transit_file_itr->path().string(),
                           std::ios_base::in | std::ios_base::binary);
          std::ofstream out(destination_path,
                            std::ios_base::out | std::ios_base::trunc | std::ios_base::binary);

          out << in.rdbuf();
        }
      }
    }
  }

  // Bail if no matching tiles
  if (!tiles.size()) {
    LOG_INFO("No transit tiles found. Transit will not be added.");
    return;
  }

  // TODO - intermediate pass to find any connections that cross into different
  // tile than the stop

  // Second pass - for all tiles with transit stops get all transit information
  // and populate tiles

  // A place to hold worker threads and their results
  std::vector<std::shared_ptr<std::thread>> threads(
      std::max(static_cast<uint32_t>(1),
               pt.get<uint32_t>("mjolnir.concurrency", std::thread::hardware_concurrency())));

  // An atomic object we can use to do the synchronization
  std::mutex lock;

  // A place to hold the results of those threads (exceptions, stats)
  std::list<std::promise<builder_stats>> results;

  // Start the threads, divvy up the work
  LOG_INFO("Adding " + std::to_string(tiles.size()) + " transit tiles to the local graph...");
  size_t floor = tiles.size() / threads.size();
  size_t at_ceiling = tiles.size() - (threads.size() * floor);
  std::unordered_set<GraphId>::const_iterator tile_start, tile_end = tiles.begin();

  // Atomically pass around stats info
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    tile_start = tile_end;
    // Where the range ends
    std::advance(tile_end, tile_count);
    // Make the thread
    results.emplace_back();
    threads[i].reset(new std::thread(build, std::cref(pt.get_child("mjolnir")), std::ref(lock),
                                     std::cref(tiles), tile_start, tile_end,
                                     std::ref(results.back())));
  }

  // Wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  // Check all of the outcomes, to see about maximum density (km/km2)
  builder_stats stats{};
  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      auto thread_stats = result.get_future().get();
      stats(thread_stats);
    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }

  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t secs = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
  LOG_INFO("Finished - TransitBuilder took " + std::to_string(secs) + " secs");
}

} // namespace mjolnir
} // namespace valhalla
