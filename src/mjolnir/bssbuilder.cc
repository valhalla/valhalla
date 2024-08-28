#include "mjolnir/bssbuilder.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "midgard/pointll.h"
#include "mjolnir/graphtilebuilder.h"

#include <algorithm>
#include <limits>
#include <mutex>
#include <thread>
#include <tuple>
#include <vector>

#include <boost/range/algorithm.hpp>

#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"
#include "midgard/logging.h"
#include "midgard/sequence.h"
#include "midgard/util.h"
#include "mjolnir/osmnode.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

struct BestProjection {
  const DirectedEdge* directededge = nullptr;
  uint32_t startnode = std::numeric_limits<uint32_t>::max();
  std::vector<PointLL> shape;
  std::tuple<PointLL, float, int> closest;
};

/*
 * We store in this struct all information about the bss connections which
 * connect the bss node and the way node.
 * From each instance of BSSConnection, we are going to create TWO edges:
 *  BSS -> waynode
 *  waynode -> BSS
 */
struct BSSConnection {
  OSMNode osm_node = {};
  PointLL bss_ll = {};
  GraphId bss_node_id = {};
  GraphId way_node_id = {};

  uint64_t wayid = std::numeric_limits<uint64_t>::max();
  std::vector<std::string> names = {};
  std::vector<std::string> tagged_values = {};
  std::vector<std::string> linguistics = {};

  std::vector<PointLL> shape = {};
  // Is the outbound edge from the waynode is forward?
  bool is_forward_from_waynode = true;
  uint32_t speed = 0;
  Surface surface = Surface::kCompacted;
  CycleLane cyclelane = CycleLane::kNone;
  RoadClass roadclass = RoadClass::kUnclassified;
  Use use = Use::kOther;

  uint32_t forwardaccess = kPedestrianAccess | kBicycleAccess;
  uint32_t reverseaccess = kPedestrianAccess | kBicycleAccess;

  BSSConnection() = default;

  BSSConnection(OSMNode osm_node,
                PointLL bss_ll,
                GraphId way_node_id,
                const EdgeInfo& edgeinfo,
                bool is_forward,
                const BestProjection& best)
      : osm_node(osm_node), bss_ll(std::move(bss_ll)), way_node_id(way_node_id) {
    /*
     * In this constructor: bss_node_id, shapes are left on default value on purpose
     * 	they are to be updated once the bss node is added into the local tile
     * */
    wayid = edgeinfo.wayid();
    names = edgeinfo.GetNames();
    tagged_values = edgeinfo.GetTaggedValues();

    linguistics = edgeinfo.GetLinguisticTaggedValues();
    is_forward_from_waynode = is_forward;
    speed = best.directededge->speed();
    surface = best.directededge->surface();
    cyclelane = best.directededge->cyclelane();
    roadclass = best.directededge->classification();
    use = best.directededge->use();
    forwardaccess = best.directededge->forwardaccess();
    reverseaccess = best.directededge->reverseaccess();
  }
  // operator < for sorting
  bool operator<(const BSSConnection& other) const {
    if (way_node_id.tileid() != other.way_node_id.tileid()) {
      return way_node_id.tileid() < other.way_node_id.tileid();
    }
    return way_node_id.id() < other.way_node_id.id();
  }
};

DirectedEdge make_directed_edge(const GraphId endnode,
                                const std::vector<PointLL>& shape,
                                const BSSConnection& conn,
                                const bool is_forward,
                                const uint32_t localedgeidx) {
  DirectedEdge directededge;
  directededge.set_endnode(endnode);

  directededge.set_length(valhalla::midgard::length(shape));
  directededge.set_use(conn.use);
  directededge.set_speed(conn.speed);
  directededge.set_surface(conn.surface);
  directededge.set_cyclelane(conn.cyclelane);
  directededge.set_classification(conn.roadclass);
  directededge.set_localedgeidx(localedgeidx);

  auto accesses = std::vector<uint32_t>{conn.forwardaccess, conn.reverseaccess};
  directededge.set_forwardaccess(accesses[static_cast<size_t>(!is_forward)]);
  directededge.set_reverseaccess(accesses[static_cast<size_t>(is_forward)]);

  directededge.set_named(conn.names.size() > 0 || conn.tagged_values.size() > 0);
  directededge.set_forward(is_forward);
  directededge.set_bss_connection(true);
  return directededge;
}

using bss_by_tile_t = std::unordered_map<GraphId, std::vector<OSMNode>>;

void compute_and_fill_shape(const BestProjection& best,
                            const PointLL& bss_ll,
                            BSSConnection& start,
                            BSSConnection& end) {
  const auto& closest_point = std::get<0>(best.closest);
  auto closest_index = std::get<2>(best.closest);

  std::copy(best.shape.begin(), best.shape.begin() + closest_index + 1,
            std::back_inserter(start.shape));
  start.shape.push_back(closest_point);
  start.shape.push_back(bss_ll);

  end.shape.push_back(bss_ll);
  end.shape.push_back(closest_point);
  std::copy(best.shape.begin() + closest_index + 1, best.shape.end(), std::back_inserter(end.shape));
}

const static auto VALID_EDGE_USES = std::unordered_set<Use>{
    Use::kRoad, Use::kLivingStreet, Use::kCycleway, Use::kSidewalk,    Use::kFootway,
    Use::kPath, Use::kPedestrian,   Use::kAlley,    Use::kServiceRoad,

};

std::vector<BSSConnection> project(const GraphTile& local_tile, const std::vector<OSMNode>& osm_bss) {
  auto t1 = std::chrono::high_resolution_clock::now();
  auto scoped_finally = make_finally([&t1, size = osm_bss.size()]() {
    auto t2 = std::chrono::high_resolution_clock::now();
    [[maybe_unused]] uint32_t secs =
        std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
    LOG_INFO("Projection Finished - Projection of " + std::to_string(size) + " bike station  took " +
             std::to_string(secs) + " secs");
  });

  std::vector<BSSConnection> res;
  auto local_level = TileHierarchy::levels().back().level;

  std::map<GraphId, size_t> edge_count;

  // In this loop, we try to find the way on which to project the bss node by iterating all nodes in
  // its corresponding tile... Not a good idea in term of performance... any better idea???
  for (const auto& bss : osm_bss) {

    auto latlng = bss.latlng();
    auto bss_ll = PointLL{latlng.first, latlng.second};

    float mindist_ped = std::numeric_limits<float>::max();
    float mindist_bicycle = std::numeric_limits<float>::max();

    auto best_ped = BestProjection{};
    auto best_bicycle = BestProjection{};

    // Loop over all nodes in the tile to find the nearest edge
    for (uint32_t i = 0; i < local_tile.header()->nodecount(); ++i) {
      const NodeInfo* node = local_tile.node(i);
      for (uint32_t j = 0; j < node->edge_count(); ++j) {
        const DirectedEdge* directededge = local_tile.directededge(node->edge_index() + j);
        auto edgeinfo = local_tile.edgeinfo(directededge);

        auto found = VALID_EDGE_USES.count(directededge->use());
        if (!found) {
          continue;
        }

        if ((!(directededge->forwardaccess() & kBicycleAccess) &&
             !(directededge->forwardaccess() & kPedestrianAccess)) ||
            directededge->is_shortcut()) {
          continue;
        }

        std::vector<PointLL> this_shape = edgeinfo.shape();
        if (!directededge->forward()) {
          std::reverse(this_shape.begin(), this_shape.end());
        }
        auto this_closest = bss_ll.Project(this_shape);

        if (directededge->forwardaccess() & kPedestrianAccess) {
          if (std::get<1>(this_closest) < mindist_ped) {
            mindist_ped = std::get<1>(this_closest);
            best_ped.directededge = directededge;
            best_ped.shape = this_shape;
            best_ped.closest = this_closest;
            best_ped.startnode = i;
          }
        }
        if (directededge->forwardaccess() & kBicycleAccess) {
          if (std::get<1>(this_closest) < mindist_bicycle) {
            mindist_bicycle = std::get<1>(this_closest);
            best_bicycle.directededge = directededge;
            best_bicycle.shape = this_shape;
            best_bicycle.closest = this_closest;
            best_bicycle.startnode = i;
          }
        }
      }
    }
    if (best_ped.startnode == static_cast<uint32_t>(-1) ||
        best_bicycle.startnode == static_cast<uint32_t>(-1)) {
      LOG_ERROR("Cannot find any edge to project the BSS: " + std::to_string(bss.osmid_));
      continue;
    }

    auto edgeinfo_ped = local_tile.edgeinfo(best_ped.directededge);
    // Store the information of the edge start <-> bss for pedestrian
    auto start_ped = BSSConnection{bss,
                                   bss_ll,
                                   {local_tile.id().tileid(), local_level, best_ped.startnode},
                                   edgeinfo_ped,
                                   // In order to simplify the problem, we ALWAYS consider that the
                                   // outbound edge of start node is forward
                                   true,
                                   best_ped};

    // Store the information of the edge end <-> bss for pedestrian
    auto end_ped =
        BSSConnection{bss, bss_ll, best_ped.directededge->endnode(), edgeinfo_ped, false, best_ped};

    auto edgeinfo_bicycle = local_tile.edgeinfo(best_bicycle.directededge);

    // Store the information of the edge start <-> bss for bicycle
    auto start_bicycle =
        BSSConnection{bss,
                      bss_ll,
                      {local_tile.id().tileid(), local_level, best_bicycle.startnode},
                      edgeinfo_bicycle,
                      true,
                      best_bicycle};

    // Store the information of the edge end <-> bss for bicycle
    auto end_bicycle =
        BSSConnection{bss,   bss_ll,      best_bicycle.directededge->endnode(), edgeinfo_bicycle,
                      false, best_bicycle};

    compute_and_fill_shape(best_ped, bss_ll, start_ped, end_ped);
    compute_and_fill_shape(best_bicycle, bss_ll, start_bicycle, end_bicycle);

    res.push_back(std::move(start_ped));
    res.push_back(std::move(end_ped));
    res.push_back(std::move(start_bicycle));
    res.push_back(std::move(end_bicycle));
  }

  return res;
}

void add_bss_nodes_and_edges(GraphTileBuilder& tilebuilder_local,
                             const GraphTile& tile,
                             const OSMData& osm_data,
                             std::mutex& lock,
                             std::vector<BSSConnection>& new_connections) {
  auto local_level = TileHierarchy::levels().back().level;
  auto scoped_finally = make_finally([&tilebuilder_local, &tile, &lock]() {
    LOG_INFO("Storing local tile data with bss nodes, tile id: " +
             std::to_string(tile.id().tileid()));
    UNUSED(tile);
    std::lock_guard<std::mutex> l(lock);
    tilebuilder_local.StoreTileData();
  });

  for (auto it = new_connections.begin(); it != new_connections.end(); std::advance(it, 4)) {
    size_t edge_index = tilebuilder_local.directededges().size();

    NodeInfo new_bss_node{tile.header()->base_ll(),
                          it->bss_ll,
                          (kPedestrianAccess | kBicycleAccess),
                          NodeType::kBikeShare,
                          false,
                          true,
                          false,
                          false};

    new_bss_node.set_mode_change(true);
    new_bss_node.set_edge_index(edge_index);

    // there should be two outbound edge for the bss node
    new_bss_node.set_edge_count(4);

    GraphId new_bss_node_graphid{tile.header()->graphid().tileid(), local_level,
                                 static_cast<uint32_t>(tilebuilder_local.nodes().size())};

    tilebuilder_local.nodes().emplace_back(std::move(new_bss_node));

    auto encode_tag = [](TaggedValue tag) {
      return std::string(1, static_cast<std::string::value_type>(tag));
    };

    for (int j = 0; j < 4; j++) {
      auto& bss_to_waynode = *(it + j);
      bss_to_waynode.bss_node_id = new_bss_node_graphid;
      bss_to_waynode.tagged_values.push_back(encode_tag(TaggedValue::kBssInfo) +
                                             osm_data.node_names.name(it->osm_node.bss_info_index()));

      bool added{false};
      auto directededge =
          make_directed_edge(bss_to_waynode.way_node_id, bss_to_waynode.shape, bss_to_waynode,
                             !bss_to_waynode.is_forward_from_waynode, 0);

      uint32_t edge_info_offset =
          tilebuilder_local.AddEdgeInfo(tilebuilder_local.directededges().size(),
                                        new_bss_node_graphid, bss_to_waynode.way_node_id,
                                        bss_to_waynode.wayid, 0, 0, 0, bss_to_waynode.shape,
                                        bss_to_waynode.names, bss_to_waynode.tagged_values,
                                        bss_to_waynode.linguistics, 0, added);

      directededge.set_edgeinfo_offset(edge_info_offset);
      tilebuilder_local.directededges().emplace_back(std::move(directededge));
    }
  }
}

void project_and_add_bss_nodes(const boost::property_tree::ptree& pt,
                               std::mutex& lock,
                               bss_by_tile_t::const_iterator tile_start,
                               bss_by_tile_t::const_iterator tile_end,
                               const OSMData& osm_data,
                               std::vector<BSSConnection>& all) {

  GraphReader reader_local_level(pt);
  for (; tile_start != tile_end; ++tile_start) {

    graph_tile_ptr local_tile = nullptr;
    std::unique_ptr<GraphTileBuilder> tilebuilder_local = nullptr;
    {
      std::lock_guard<std::mutex> l(lock);

      auto tile_id = tile_start->first;
      local_tile = reader_local_level.GetGraphTile(tile_id);
      tilebuilder_local.reset(new GraphTileBuilder{reader_local_level.tile_dir(), tile_id, true});
    }

    auto new_connections = project(*local_tile, tile_start->second);
    add_bss_nodes_and_edges(*tilebuilder_local, *local_tile, osm_data, lock, new_connections);
    {
      std::lock_guard<std::mutex> l{lock};
      std::move(new_connections.begin(), new_connections.end(), std::back_inserter(all));
    }
  }
}

void create_edges(GraphTileBuilder& tilebuilder_local,
                  const GraphTile& tile,
                  std::mutex& lock,
                  const std::vector<BSSConnection>& bss_connections) {
  auto t1 = std::chrono::high_resolution_clock::now();

  auto scoped_finally = make_finally([&tilebuilder_local, &tile, &lock, t1]() {
    auto t2 = std::chrono::high_resolution_clock::now();
    uint32_t secs = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();

    LOG_INFO("Tile id: " + std::to_string(tile.id().tileid()) + " It took " + std::to_string(secs) +
             " seconds to create edges. Now storing local tile data with new edges");
    UNUSED(tile);
    UNUSED(secs);
    std::lock_guard<std::mutex> l(lock);
    tilebuilder_local.StoreTileData();
  });

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
  uint32_t added_edges = 0;

  for (auto& nb : currentnodes) {
    size_t nodeid = tilebuilder_local.nodes().size();
    size_t edge_index = tilebuilder_local.directededges().size();

    // recreate the node and its edges
    {
      // Copy existing directed edges from this node and update any signs using
      // the directed edge index
      for (uint32_t i = 0, idx = nb.edge_index(); i < nb.edge_count(); i++, idx++) {
        tilebuilder_local.directededges().emplace_back(std::move(currentedges[idx]));

        // Update any signs that use this idx - increment their index by the
        // number of added edges
        while (idx == nextsignidx && signidx < signcount) {
          if (!currentedges[idx].sign()) {
            LOG_ERROR("Signs for this index but directededge says no sign");
          }
          tilebuilder_local.sign_builder(signidx).set_index(idx + added_edges);

          // Increment to the next sign and update next signidx
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

          // Increment to the next restriction and update next residx
          residx++;
          nextresidx =
              (residx >= rescount) ? 0 : tilebuilder_local.accessrestriction(residx).edgeindex();
        }
      }
    }

    auto target = BSSConnection{};
    target.way_node_id = {0, 0, static_cast<uint32_t>(nodeid)};

    auto comp = [](const BSSConnection& lhs, const BSSConnection& rhs) {
      return lhs.way_node_id.id() < rhs.way_node_id.id();
    };
    auto lower = std::lower_bound(bss_connections.begin(), bss_connections.end(), target, comp);
    auto upper = std::upper_bound(bss_connections.begin(), bss_connections.end(), target, comp);

    while (lower != upper && lower != bss_connections.end()) {
      size_t local_idx = tilebuilder_local.directededges().size() - edge_index;

      auto directededge = make_directed_edge(lower->bss_node_id, lower->shape, *lower,
                                             lower->is_forward_from_waynode, local_idx);
      bool added;
      uint32_t edge_info_offset =
          tilebuilder_local.AddEdgeInfo(tilebuilder_local.directededges().size(), lower->way_node_id,
                                        lower->bss_node_id, lower->wayid, 0, 0, 0, lower->shape,
                                        lower->names, lower->tagged_values, lower->linguistics, 0,
                                        added);

      directededge.set_edgeinfo_offset(edge_info_offset);

      tilebuilder_local.directededges().emplace_back(std::move(directededge));
      added_edges++;
      std::advance(lower, 1);
    };

    // Add the node and directed edges
    nb.set_edge_index(edge_index);
    nb.set_edge_count(tilebuilder_local.directededges().size() - edge_index);
    tilebuilder_local.nodes().emplace_back(std::move(nb));
  }

  LOG_INFO(std::string("Added: ") + std::to_string(added_edges) + " edges");
}

void create_edges_from_way_node(
    const boost::property_tree::ptree& pt,
    std::mutex& lock,
    std::unordered_map<GraphId, std::vector<BSSConnection>>::const_iterator tile_start,
    std::unordered_map<GraphId, std::vector<BSSConnection>>::const_iterator tile_end) {

  GraphReader reader_local_level(pt);
  for (; tile_start != tile_end; ++tile_start) {

    graph_tile_ptr local_tile = nullptr;
    std::unique_ptr<GraphTileBuilder> tilebuilder_local = nullptr;
    {
      std::lock_guard<std::mutex> l(lock);

      auto tile_id = tile_start->first;
      local_tile = reader_local_level.GetGraphTile(tile_id);
      tilebuilder_local.reset(new GraphTileBuilder{reader_local_level.tile_dir(), tile_id, true});
    }
    create_edges(*tilebuilder_local, *local_tile, lock, tile_start->second);
  }
}

} // namespace

namespace valhalla {
namespace mjolnir {

// Add bss to the graph
/* The import of bike share station(BSS) into the tiles is done in two steps with some hypothesis in
 * order to simply the problem.
 *
 * We assume that the BSS node and the startnode of projected edge(either the forward edge or its evil
 * twin reverse edge) are always in the same tile.
 *
 * We handle only two cases and we assume that there are very rare cases that the BSS node,
 * the startnode and the endnode of projected edge are in 3 different tiles, but it's common that the
 * projected edge crosses tiles.
 *
 * Case 1 (handled):
 *
 *
 *  (Tile 1)             (Tile 1)
 *       S ---------------> E
 *           ^
 *           |
 *          Bss
 *
 *
 *
 * Case 2 (handled):
 *
 *                 |
 *  (Tile 2)       |      (Tile 1)
 *       S ---------------> E
 *           ^     |
 *           |     |
 *          Bss
 *
 * Case 3 (not handled, rare):
 *
 *
 *                 |
 *  (Tile 1)       |      (Tile 2)
 *       S ---------------> E
 *           ^     |
 *           |     |
 *      _____|_____|________________
 *           |     |
 *          Bss    |
 *   (Tile 3)      |
 *
 *
 * The import is done in two steps:
 *
 * 1. Find the nearest edge on which the BSS node should be projected, then add the bss nodes and
 * their outbound edge to the local tiles. In this step, we assume that every BSS node will have 2
 * outbound edges: one is towards the start and another is towards the end. Since we know to which
 * node these outbound edges point, we can easily compute their oppo_edgelocalidx which is essential.
 *
 * 2. Now the Bss nodes and their outbound edges are added into the local tiles, it's time to add
 * their inbound edges(in other words, outbound edges of startnodes and endnodes). These edges are
 * just considered as the same outbound edges from a way node (outbound edges of either startnode or
 * endnode are technically the same). We group those edges whose origin are in the same tiles and work
 * on it in batch.
 *
 *
 * */
void BssBuilder::Build(const boost::property_tree::ptree& pt,
                       const OSMData& osmdata,
                       const std::string& bss_nodes_bin) {

  if (!pt.get<bool>("mjolnir.import_bike_share_stations", false)) {
    return;
  }

  LOG_INFO("Importing Bike Share station");

  auto t1 = std::chrono::high_resolution_clock::now();

  auto scoped_finally = make_finally([&t1]() {
    auto t2 = std::chrono::high_resolution_clock::now();
    [[maybe_unused]] uint32_t secs =
        std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
    LOG_INFO("Finished - BssBuilder took " + std::to_string(secs) + " secs");
  });

  midgard::sequence<mjolnir::OSMNode> osm_nodes{bss_nodes_bin, false};

  // bss_by_tile_t bss_by_tile;
  bss_by_tile_t bss_by_tile;

  GraphReader reader(pt.get_child("mjolnir"));
  auto local_level = TileHierarchy::levels().back().level;

  // Group the nodes by their tiles. In the next step, we will work on each tile only once
  for (auto node : osm_nodes) {
    auto latlng = node.latlng();
    auto tile_id = TileHierarchy::GetGraphId({latlng.first, latlng.second}, local_level);
    graph_tile_ptr local_tile = reader.GetGraphTile(tile_id);
    if (!local_tile) {
      LOG_INFO("Cannot find node in tiles");
      continue;
    }
    bss_by_tile[tile_id].push_back(node);
  }

  size_t nb_threads =
      std::max(static_cast<uint32_t>(1),
               pt.get<uint32_t>("mjolnir.concurrency", std::thread::hardware_concurrency()));
  std::vector<std::shared_ptr<std::thread>> threads(nb_threads);

  // An atomic object we can use to do the synchronization
  std::mutex lock;

  // Start the threads
  LOG_INFO("Adding " + std::to_string(osm_nodes.size()) + " bike share stations to " +
           std::to_string(bss_by_tile.size()) + " local graphs with " + std::to_string(nb_threads) +
           " thread(s)");

  std::vector<BSSConnection> all;
  {
    size_t floor = bss_by_tile.size() / threads.size();
    size_t at_ceiling = bss_by_tile.size() - (threads.size() * floor);
    bss_by_tile_t::const_iterator tile_start, tile_end = bss_by_tile.begin();

    for (size_t i = 0; i < threads.size(); ++i) {
      // Figure out how many this thread will work on (either ceiling or floor)
      size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
      // Where the range begins
      tile_start = tile_end;
      // Where the range ends
      std::advance(tile_end, tile_count);
      // Make the thread
      threads[i].reset(new std::thread(project_and_add_bss_nodes, std::cref(pt.get_child("mjolnir")),
                                       std::ref(lock), tile_start, tile_end, std::cref(osmdata),
                                       std::ref(all)));
    }

    for (auto& thread : threads) {
      thread->join();
    }
  }

  // the collection is sorted so that the search will be much faster later.
  boost::sort(all);

  // outboud edges from way node are grouped by tiles.
  std::unordered_map<GraphId, std::vector<BSSConnection>> map;
  if (!all.empty()) {
    auto chunk_start = all.begin();
    do {
      auto tileid = chunk_start->way_node_id.tileid();
      auto chunk_end = std::stable_partition(chunk_start, all.end(), [tileid](const auto& conn) {
        return tileid == conn.way_node_id.tileid();
      });

      std::move(chunk_start, chunk_end, std::back_inserter(map[{tileid, local_level, 0}]));

      chunk_start = chunk_end;

    } while (chunk_start != all.end());
  }

  {
    size_t floor = map.size() / threads.size();
    size_t at_ceiling = map.size() - (threads.size() * floor);
    std::unordered_map<GraphId, std::vector<BSSConnection>>::const_iterator tile_start,
        tile_end = map.begin();

    for (size_t i = 0; i < threads.size(); ++i) {
      // Figure out how many this thread will work on (either ceiling or floor)
      size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
      // Where the range begins
      tile_start = tile_end;
      // Where the range ends
      std::advance(tile_end, tile_count);
      // Make the thread
      threads[i].reset(new std::thread(create_edges_from_way_node, std::cref(pt.get_child("mjolnir")),
                                       std::ref(lock), tile_start, tile_end));
    }

    for (auto& thread : threads) {
      thread->join();
    }
  }
}

} // namespace mjolnir
} // namespace valhalla
