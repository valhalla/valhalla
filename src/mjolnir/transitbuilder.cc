#include "mjolnir/transitbuilder.h"
#include "mjolnir/graphtilebuilder.h"

#include <fstream>
#include <future>
#include <iostream>
#include <list>
#include <mutex>
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

constexpr float SNAP_DISTANCE_CUTOFF = 500.f;

const static auto VALID_EDGE_USES = std::unordered_set<Use>{
    Use::kRoad, Use::kLivingStreet, Use::kCycleway, Use::kSidewalk,    Use::kFootway,
    Use::kPath, Use::kPedestrian,   Use::kAlley,    Use::kServiceRoad,
};

struct OSMConnectionEdge {
  GraphId osm_node;
  GraphId stop_node;
  double length;
  uint64_t wayid;
  std::vector<std::string> names;
  std::vector<PointLL> shape;
  std::vector<std::string> tagged_values;
  std::vector<std::string> pronunciations;
};

// Struct to hold stats information during each threads work
struct builder_stats {
  uint64_t conn_edges;

  // Accumulate stats from all threads
  void operator()(const builder_stats& other) {
    conn_edges += other.conn_edges;
  }
};

void ConnectToGraph(GraphTileBuilder& tilebuilder_local,
                    GraphTileBuilder& tilebuilder_transit,
                    const graph_tile_ptr& tile,
                    GraphReader& reader_transit_level,
                    std::mutex& lock,
                    const std::vector<OSMConnectionEdge>& connection_edges,
                    builder_stats& stats) {
  // Move existing nodes and directed edge builder vectors and clear the lists
  std::vector<NodeInfo> currentnodes(std::move(tilebuilder_local.nodes()));
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
      GraphId endnode = conn.stop_node;
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

      const auto& transit_stop = tilebuilder_transit.nodes()[conn.stop_node.id()];
      auto e_traversability =
          tilebuilder_transit.GetTransitStop(transit_stop.stop_index())->traversability();
      if (e_traversability == Traversability::kBoth) {
        directededge.set_forwardaccess(tc_access);
        directededge.set_reverseaccess(tc_access);
      } else if (e_traversability == Traversability::kForward) {
        directededge.set_forwardaccess(tc_access);
      } else if (e_traversability == Traversability::kBackward) {
        directededge.set_reverseaccess(tc_access);
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
    LOG_ERROR("Added " + std::to_string(added_edges) + " from local to transit but there are " +
              std::to_string(connection_edges.size()) + " connections");
  }

  // HERE WE CONNECT BACK FROM TRANSIT NODES TO THE LOCAL GRAPH

  // Move existing nodes and directed edge builder vectors and clear the lists
  currentnodes = std::move(tilebuilder_transit.nodes());
  tilebuilder_transit.nodes().clear();
  currentedges = std::move(tilebuilder_transit.directededges());
  tilebuilder_transit.directededges().clear();

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

        // use the access from the transit node
        const auto& tc_access = nb.access();
        DirectedEdge directededge;
        directededge.set_endnode(conn.osm_node);
        directededge.set_length(conn.length);
        directededge.set_use(Use::kTransitConnection);
        directededge.set_speed(5);
        directededge.set_classification(RoadClass::kServiceOther);
        directededge.set_localedgeidx(tilebuilder_transit.directededges().size() - edge_index);

        // we have to go back to the copy of the transit nodes
        const auto& transit_stop = currentnodes[conn.stop_node.id()];
        auto e_traversability =
            tilebuilder_transit.GetTransitStop(transit_stop.stop_index())->traversability();
        if (e_traversability == Traversability::kBoth) {
          directededge.set_forwardaccess(tc_access);
          directededge.set_reverseaccess(tc_access);
          // exit only.  if backward is true then set forward access as we are going from
          // the egress to the osm node.
        } else if (e_traversability == Traversability::kBackward) {
          directededge.set_forwardaccess(tc_access);
          // enter only.  if forward is true then set backward access as we are going from
          // the egress to the osm node.
        } else if (e_traversability == Traversability::kForward) {
          directededge.set_reverseaccess(tc_access);
        }
        directededge.set_named(conn.names.size() > 0 || conn.tagged_values.size() > 0);

        // Add edge info to the tile and set the offset in the directed edge
        bool added = false;
        auto r_shape = conn.shape;
        std::reverse(r_shape.begin(), r_shape.end());
        uint32_t edge_info_offset =
            tilebuilder_transit.AddEdgeInfo(0, conn.stop_node, conn.osm_node, conn.wayid, 0, 0, 0,
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
    LOG_ERROR("Added " + std::to_string(added_edges) + " from transit to local but there are " +
              std::to_string(connection_edges.size()) + " connections");
  }

  // Log the number of added nodes and edges
  LOG_DEBUG("Tile " + std::to_string(tilebuilder_local.header()->graphid().tileid()) + ": added " +
            std::to_string(connedges) + " connection edges.");
  stats.conn_edges = connedges;
}

std::vector<OSMConnectionEdge> MakeConnections(const graph_tile_ptr& local_tile,
                                               const graph_tile_ptr& transit_tile) {
  assert(local_tile && transit_tile->header()->nodecount());
  std::vector<OSMConnectionEdge> connections;
  connections.reserve(transit_tile->header()->nodecount() * 2 / 3);

  // for each transit egress
  for (auto egress_id = transit_tile->header()->graphid();
       egress_id.id() < transit_tile->header()->nodecount(); ++egress_id) {
    // we only connect egress/ingress
    const auto* egress = transit_tile->node(egress_id);
    if (egress->type() != NodeType::kTransitEgress)
      continue;
    auto egress_ll = egress->latlng(transit_tile->header()->base_ll());

    // we have two options for helping find better connections to the road network
    // 1. you can provide a way id, this is coarse because it doesnt tell exactly where along the way
    //    you should connect the station, the code just picks the closest point
    // 2. the other option is encoding a lat lon in the way id field which lets you pick the best spot
    //    in space to connect to the road network regardless of which way, the idea being that its
    //    very very close to the actual osm geometry where it should connect
    uint64_t way_id = egress->connecting_wayid();
    PointLL connecting_ll = egress->connecting_point();
    if (connecting_ll.IsValid())
      egress_ll = connecting_ll;

    // We assume we dont find a matching way id to start, but once we do all better results must have
    // a matching way id. This allows us to only use the way id when it makes sense and still get a
    // good match when the way id doesnt make sense
    bool should_match_way_id = false;

    // keep this info about the closest point to an edge shape
    GraphId closest_start_node;
    const DirectedEdge* closest_edge = nullptr;
    boost::optional<EdgeInfo> closest_edgeinfo;
    std::tuple<PointLL, decltype(PointLL::first), int> closest_point({}, SNAP_DISTANCE_CUTOFF, 0);

    // Loop over all the graph that is in this tile
    auto startnode_id = local_tile->header()->graphid();
    const auto* startnode = local_tile->node(0);
    auto directededge_id = local_tile->header()->graphid();
    const auto* directededge = local_tile->directededge(0);
    for (; directededge_id.id() < local_tile->header()->directededgecount();
         ++directededge_id, ++directededge) {
      // next node
      if (startnode->edge_index() + startnode->edge_count() == directededge_id.id()) {
        ++startnode_id;
        ++startnode;
      }

      // there will be a more convenient opposing edge to use for this one so lets wait for it
      if (!directededge->forward() && (directededge->endnode().Tile_Base() == local_tile->id()) &&
          (directededge->reverseaccess() & kPedestrianAccess)) {
        continue;
      }

      // bail if its an invalid use
      if (VALID_EDGE_USES.count(directededge->use()) == 0) {
        continue;
      }

      // bail if pedestrians cant use it
      if ((!(directededge->forwardaccess() & kPedestrianAccess)) || directededge->is_shortcut()) {
        continue;
      }

      // project onto the shape
      auto ei = local_tile->edgeinfo(directededge);
      auto point = egress_ll.Project(ei.shape());
      auto distance = std::get<1>(point);

      // must be closer than the closest (or limit) AND way ids match if provided/found
      bool does_match_way_id = way_id == ei.wayid();
      if (distance < std::get<1>(closest_point) && should_match_way_id == does_match_way_id) {
        closest_start_node = startnode_id;
        closest_edge = directededge;
        closest_point = point;
        closest_edgeinfo = std::move(ei);
        should_match_way_id = does_match_way_id;
      }
    }

    // so long as we found something we should store info about where to connect it
    // we make new edges that follow the existing edge from its end and begin nodes up to the snap
    // point and then make a straight line from there to the in/egress
    if (!closest_edge) {
      LOG_WARN("Could not find connection point for in/egress near: " +
               std::to_string(egress_ll.second) + "," + std::to_string(egress_ll.first));
    }

    // TODO: if the point we found is further away than the tile edge then there could be a better
    //  match in an adjacent tile. For now we should detect this and log, to get a feeling of
    //  prevalence. In the future we should look at other tiles (once we fix the below issue this will
    //  be easy)

    // flip the shape if we have to
    std::vector<PointLL> edge_shape = closest_edgeinfo->shape();
    if (!closest_edge->forward()) {
      std::reverse(edge_shape.begin(), edge_shape.end());
      // size - 1 because we are dealing with indices not counts, but we take another off because its
      // the index of the segment in the line not the point (1 fewer segments than points)
      std::get<2>(closest_point) = (edge_shape.size() - 2) - std::get<2>(closest_point);
    }

    // by definition this edge will be in the tile (starting at the begin node of the snapped edge
    std::vector<PointLL> shape(edge_shape.begin(),
                               edge_shape.begin() + std::get<2>(closest_point) + 1);
    shape.push_back(std::get<0>(closest_point));
    shape.push_back(egress_ll);
    auto length = std::max(1.0, valhalla::midgard::length(shape));
    connections.emplace_back(OSMConnectionEdge{closest_start_node, egress_id, length,
                                               closest_edgeinfo->wayid(),
                                               closest_edgeinfo->GetNames(), shape});

    // the end node is in another tile
    if (closest_edge->endnode().Tile_Base() != local_tile->id()) {
      // TODO: its a big pain in the butt, but we can fix this. to do so we need to split up the jobs
      //  of finding the connection points and modifying the tiles, which means we need to keep all
      //  the connections in memory (its small), organize them by tile and then update the tiles
      LOG_WARN("Could not create transit connect edge from end node because its in another tile");
      continue;
    }

    // the end node is in this tile too so we can keep this connection as well
    std::get<2>(closest_point) = (edge_shape.size() - 2) - std::get<2>(closest_point);
    std::vector<PointLL> opp_shape(edge_shape.rbegin(),
                                   edge_shape.rbegin() + std::get<2>(closest_point) + 1);
    opp_shape.push_back(std::get<0>(closest_point));
    opp_shape.push_back(egress_ll);
    length = std::max(1.0, valhalla::midgard::length(opp_shape));
    connections.emplace_back(OSMConnectionEdge{closest_edge->endnode(), egress_id, length,
                                               closest_edgeinfo->wayid(),
                                               closest_edgeinfo->GetNames(), opp_shape});
  }
  return connections;
}

// We make sure to lock on reading and writing since tiles are now being
// written. Also lock on queue access since shared by different threads.
void build(const boost::property_tree::ptree& pt,
           std::mutex& lock,
           std::unordered_set<GraphId>::const_iterator tile_start,
           std::unordered_set<GraphId>::const_iterator tile_end,
           std::promise<builder_stats>& results) {
  // GraphReader for local level and for transit level.
  GraphReader reader_local_level(pt);
  GraphReader reader_transit_level(pt);

  builder_stats stats;
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

    // TODO - how to handle connections that reach nodes outside the tile? Need to break up the calls
    //  to MakeConnections and ConnectToGraph below. First we have threads find all the connections
    //  even accross tile boundaries, we keep those in ram and then we run another pass where we add
    //  those to the tiles when we rewrite them in ConnectToGraph. So two separate threaded steps
    //  rather than cramming both of them into the single build function

    // Iterate through all transit tile nodes and form connections to the OSM network for those stops
    // which are in/egresses. Each in/egress connects to 2 OSM nodes along the closest edge optionally
    // matching the OSM way id
    std::vector<OSMConnectionEdge> connection_edges = MakeConnections(local_tile, transit_tile);

    // this happens when you are running against small extracts...no work to be done.
    if (connection_edges.size() == 0) {
      continue;
    }

    // Sort the connection edges
    std::sort(connection_edges.begin(), connection_edges.end(), [](const auto& a, const auto& b) {
      return a.osm_node.tileid() == b.osm_node.tileid() ? a.osm_node.id() < b.osm_node.id()
                                                        : a.osm_node.tileid() < b.osm_node.tileid();
    });

    // Connect the transit graph to the route graph
    ConnectToGraph(tilebuilder_local, tilebuilder_transit, local_tile, reader_transit_level, lock,
                   connection_edges, stats);

    // Write the new file
    lock.lock();
    tilebuilder_local.StoreTileData();
    tilebuilder_transit.StoreTileData();
    lock.unlock();
  }

  // Send back the statistics
  results.set_value(stats);
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
    // look at each file in the transit dir
    for (; transit_file_itr != end_file_itr; ++transit_file_itr) {
      // check if its a graph tile
      if (filesystem::is_regular_file(transit_file_itr->path()) &&
          transit_file_itr->path().string().find(".gph") ==
              (transit_file_itr->path().string().size() - 4)) {
        // turn the id from the level 3 transit tile into the level 2 tile under it
        auto graph_id = GraphTile::GetTileId(transit_file_itr->path().string());
        GraphId local_graph_id(graph_id.tileid(), graph_id.level() - 1, graph_id.id());
        // if the level 2 tile exists
        if (reader.DoesTileExist(local_graph_id)) {
          // remember the id for the level 2 tile
          tiles.emplace(local_graph_id);
          // figure out the path in the new tileset for the transit tile
          const std::string destination_path = pt.get<std::string>("mjolnir.tile_dir") +
                                               filesystem::path::preferred_separator +
                                               GraphTile::FileSuffix(graph_id);
          filesystem::path root = destination_path;
          root.replace_filename("");
          filesystem::create_directories(root);
          // and copy over the the transit into the tileset we are building
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
                                     tile_start, tile_end, std::ref(results.back())));
  }

  // Wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  uint64_t total_conn_edges = 0;

  // Check all of the outcomes, to see about maximum density (km/km2)
  builder_stats stats{};
  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      auto thread_stats = result.get_future().get();
      stats(thread_stats);
      total_conn_edges += stats.conn_edges;
    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }

  if (total_conn_edges) {
    LOG_INFO("Found " + std::to_string(total_conn_edges) + " connection edges");
  }

  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t secs = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
  LOG_INFO("Finished - TransitBuilder took " + std::to_string(secs) + " secs");
}

} // namespace mjolnir
} // namespace valhalla
