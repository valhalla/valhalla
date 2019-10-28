#include "mjolnir/bssbuilder.h"
#include "mjolnir/graphtilebuilder.h"

#include <boost/filesystem/operations.hpp>
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
#include <boost/foreach.hpp>
#include <boost/range/algorithm.hpp>

#include "baldr/datetime.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/distanceapproximator.h"
#include "midgard/logging.h"
#include "midgard/sequence.h"
#include "midgard/util.h"
#include "mjolnir/osmnode.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

struct OSMConnectionEdge {
  GraphId startnode = {};
  GraphId endnode = {};
  uint64_t wayid = -1;
  std::vector<std::string> names = {};
  std::list<PointLL> startshape = {};
  std::list<PointLL> endshape = {};
  PointLL bss_ll = {};

  uint32_t start_to_bss_edge_idx = -1;
  uint32_t end_to_bss_edge_idx = -1;

  OSMConnectionEdge(GraphId startnode,
                    GraphId endnode,
                    uint64_t wayid,
                    std::vector<std::string> names,
                    std::list<PointLL> startshape,
                    std::list<PointLL> endshape,
                    const PointLL& bss_ll)
      : startnode(startnode), endnode(endnode), wayid(wayid), names(std::move(names)),
        startshape(std::move(startshape)), endshape(std::move(endshape)), bss_ll(bss_ll) {
  }

  // operator < for sorting
  bool operator<(const OSMConnectionEdge& other) const {
    if (startnode.tileid() == startnode.tileid()) {
      return startnode.id() < other.startnode.id();
    } else {
      return endnode.id() < other.endnode.id();
    }
  }
};

template <typename T> struct Finally {
  T t;
  explicit Finally(T t) : t(t){};
  Finally() = delete;
  Finally(Finally&& f) = default;
  Finally(const Finally&) = delete;
  Finally& operator=(const Finally&) = delete;
  Finally& operator=(Finally&&) = delete;
  ~Finally() {
    t();
  };
};

template <typename T> Finally<T> make_finally(T t) {
  return Finally<T>{t};
};

std::vector<OSMConnectionEdge> project(const GraphTile& local_tile,
                                       const std::vector<OSMNode>& osm_bss) {

  std::vector<OSMConnectionEdge> res;
  auto local_level = TileHierarchy::levels().rbegin()->first;
  // In this loop, we try to find the way on which to project the bss node by iterating all nodes in
  // its corresponding tile... Not a good idea in term of performance... any better idea???
  for (const auto& bss : osm_bss) {
    auto latlng = bss.latlng();
    const auto bss_ll = PointLL{latlng.first, latlng.second};
    float mindist = 10000000.0f;
    uint32_t edgelength = 0;
    GraphId startnode, endnode;
    std::vector<PointLL> closest_shape;
    std::tuple<PointLL, float, int> closest;
    std::vector<std::string> names;
    uint32_t way_id = 0;
    const DirectedEdge* best_directededge = nullptr;

    // Loop over all nodes in the tile to find the nearest edge
    for (uint32_t i = 0; i < local_tile.header()->nodecount(); ++i) {
      const NodeInfo* node = local_tile.node(i);
      for (uint32_t j = 0; j < node->edge_count(); ++j) {
        const DirectedEdge* directededge = local_tile.directededge(node->edge_index() + j);
        auto edgeinfo = local_tile.edgeinfo(directededge->edgeinfo_offset());

        if (directededge->use() == Use::kTransitConnection ||
            directededge->use() == Use::kEgressConnection ||
            directededge->use() == Use::kPlatformConnection) {
          continue;
        }
        if ((!(directededge->forwardaccess() & kBicycleAccess) &&
             !(directededge->forwardaccess() & kPedestrianAccess)) ||
            directededge->is_shortcut()) {
          continue;
        }

        auto this_shape = edgeinfo.shape();
        if (!directededge->forward()) {
          std::reverse(this_shape.begin(), this_shape.end());
        }
        auto this_closest = bss_ll.ClosestPoint(this_shape);
        names = edgeinfo.GetNames();
        if (std::get<1>(this_closest) < mindist) {
          startnode = {local_tile.id().tileid(), local_level, i};
          endnode = directededge->endnode();
          mindist = std::get<1>(this_closest);
          closest = this_closest;
          closest_shape = this_shape;
          edgelength = directededge->length();
          way_id = edgeinfo.wayid();
          best_directededge = directededge;
        }
      }
    }
    if (!startnode.Is_Valid() && !endnode.Is_Valid()) {
      LOG_ERROR("Cannot find any edge to project");
      continue;
    }
    // Create a temporary connection which starts from a existing way node in the tile and point to
    // the bss node
    {
      std::list<PointLL> startshape;
      for (uint32_t i = 0; i <= std::get<2>(closest); i++) {
        startshape.push_back(closest_shape[i]);
      }
      startshape.push_back(bss_ll);

      std::list<PointLL> endshape;
      endshape.push_back(bss_ll);
      for (uint32_t i = std::get<2>(closest); i < closest_shape.size(); i++) {
        endshape.push_back(closest_shape[i]);
      }
      res.emplace_back(startnode, endnode, way_id, std::move(names), std::move(startshape),
                       std::move(endshape), bss_ll);
    }
  }
  boost::sort(res);
  return res;
}

DirectedEdge make_directed_edge(const GraphId endnode,
                                const std::list<PointLL>& shape,
                                const uint32_t localedgeidx,
                                const bool is_named) {
  DirectedEdge directededge;
  directededge.set_endnode(endnode);
  directededge.set_length(std::max(1.0f, valhalla::midgard::length(shape)));
  directededge.set_use(Use::kBikeShareConnection);
  directededge.set_speed(0);
  directededge.set_classification(RoadClass::kServiceOther);
  directededge.set_localedgeidx(localedgeidx);
  directededge.set_forwardaccess(kPedestrianAccess | kBicycleAccess);
  directededge.set_reverseaccess(kPedestrianAccess | kBicycleAccess);
  directededge.set_named(is_named);
  directededge.set_forward(true);
  return directededge;
}

void create_bss_node_and_edges(GraphTileBuilder& tilebuilder_local,
                               const GraphTile& tile,
                               std::mutex& lock,
                               std::vector<OSMConnectionEdge> new_connections) {
  // GraphTileBuilder tilebuilder_local(reader.tile_dir(), tile.header()->graphid(), true);
  auto local_level = TileHierarchy::levels().rbegin()->first;

  auto scoped_finally = make_finally([&tilebuilder_local, &tile, &lock]() {
    LOG_INFO("Storing local tile data with bss nodes, tile id: " +
             std::to_string(tile.id().tileid()));
    std::lock_guard<std::mutex> l(lock);
    tilebuilder_local.StoreTileData();
  });

  // Move existing nodes and directed edge builder vectors and clear the lists
  std::vector<NodeInfo> currentnodes(std::move(tilebuilder_local.nodes()));
  uint32_t nodecount = currentnodes.size();

  tilebuilder_local.nodes().clear();
  std::vector<DirectedEdge> currentedges(std::move(tilebuilder_local.directededges()));
  uint32_t edgecount = currentedges.size();
  tilebuilder_local.directededges().clear();

  // Get the directed edge index of the first sign. If no signs are
  // present in this tile set a value > number of directed edges
  uint32_t signidx = 0;
  uint32_t nextsignidx = (tilebuilder_local.header()->signcount() > 0)
                             ? tilebuilder_local.sign(0).edgeindex()
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
  uint32_t added_nodes = 0;

  for (auto& nb : currentnodes) {
    uint32_t nodeid = tilebuilder_local.nodes().size();
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
          if (!currentedges[idx].exitsign()) {
            LOG_ERROR("Signs for this index but directededge says no sign");
          }
          tilebuilder_local.sign_builder(signidx).set_edgeindex(idx + added_edges);

          // Increment to the next sign and update nextsignidx
          signidx++;
          nextsignidx = (signidx >= signcount) ? 0 : tilebuilder_local.sign(signidx).edgeindex();
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
    }
    // If the node should be a part of bss's edge then create the edge and update the node's
    // edge_count
    for (auto& conn : new_connections) {

      // directedge: startnode -> bssnode
      if (conn.startnode.id() == nodeid) {
        // since the bss node's graphid cannot be known at the stage,
        // the endnode graphid is left invalid on purpose, it'll be updated later
        auto directededge = make_directed_edge({}, conn.startshape,
                                               tilebuilder_local.directededges().size() - edge_index,
                                               conn.names.size());
        conn.start_to_bss_edge_idx = tilebuilder_local.directededges().size();
        tilebuilder_local.directededges().emplace_back(std::move(directededge));
        ++added_edges;
      }

      // directedge: endnode -> bssnode
      if (conn.endnode.id() == nodeid) {
        // since the bss node's graphid cannot be known at the stage,
        // the endnode graphid is left invalid on purpose, it'll be updated later
        auto directededge = make_directed_edge({}, conn.endshape,
                                               tilebuilder_local.directededges().size() - edge_index,
                                               conn.names.size());
        conn.end_to_bss_edge_idx = tilebuilder_local.directededges().size();
        tilebuilder_local.directededges().emplace_back(std::move(directededge));
        ++added_edges;
      }
    }
    // Add the node and directed edges
    nb.set_edge_index(edge_index);
    nb.set_edge_count(tilebuilder_local.directededges().size() - edge_index);
    tilebuilder_local.nodes().emplace_back(std::move(nb));
  }

  for (const auto& conn : new_connections) {

    size_t edge_index = tilebuilder_local.directededges().size();
    NodeInfo new_bss_node{tile.header()->base_ll(), conn.bss_ll,
                          RoadClass::kServiceOther, (kPedestrianAccess | kBicycleAccess),
                          NodeType::kBikeShare,     false};
    new_bss_node.set_mode_change(true);
    new_bss_node.set_edge_index(edge_index);
    // there should be two outbound edge for the bss node
    new_bss_node.set_edge_count(2);

    GraphId new_bss_node_graphid{tile.header()->graphid().tileid(), local_level,
                                 static_cast<uint32_t>(tilebuilder_local.nodes().size())};

    if (conn.start_to_bss_edge_idx == -1 || conn.end_to_bss_edge_idx == -1) {
      LOG_ERROR("edge index is invalid for the bss node: " + std::to_string(new_bss_node_graphid));
      continue;
    }

    // update startnode -> bssnode
    {
      auto& directededge = tilebuilder_local.directededges()[conn.start_to_bss_edge_idx];
      directededge.set_endnode(new_bss_node_graphid);
      bool added;
      uint32_t edge_info_offset =
          tilebuilder_local.AddEdgeInfo(conn.start_to_bss_edge_idx, conn.startnode,
                                        new_bss_node_graphid, conn.wayid, 0, 0, 0, conn.startshape,
                                        conn.names, 0, added);
      directededge.set_edgeinfo_offset(edge_info_offset);
    }

    // update endnode -> bssnode
    {
      auto& directededge = tilebuilder_local.directededges()[conn.end_to_bss_edge_idx];
      directededge.set_endnode(new_bss_node_graphid);
      bool added;
      uint32_t edge_info_offset =
          tilebuilder_local.AddEdgeInfo(conn.end_to_bss_edge_idx, conn.endnode, new_bss_node_graphid,
                                        conn.wayid, 0, 0, 0, conn.endshape, conn.names, 0, added);
      directededge.set_edgeinfo_offset(edge_info_offset);
    }

    // create bssnode -> startnode
    {
      auto directededge = make_directed_edge(conn.startnode, conn.startshape, 0, conn.names.size());
      bool added;
      uint32_t edge_info_offset =
          tilebuilder_local.AddEdgeInfo(conn.start_to_bss_edge_idx, new_bss_node_graphid,
                                        conn.startnode, conn.wayid, 0, 0, 0, conn.startshape,
                                        conn.names, 0, added);
      directededge.set_edgeinfo_offset(edge_info_offset);
      tilebuilder_local.directededges().emplace_back(std::move(directededge));
      ++added_edges;
    }

    // create bssnode -> endnode
    {
      auto directededge = make_directed_edge(conn.endnode, conn.endshape, 1, conn.names.size());
      bool added;
      uint32_t edge_info_offset =
          tilebuilder_local.AddEdgeInfo(conn.end_to_bss_edge_idx, new_bss_node_graphid, conn.endnode,
                                        conn.wayid, 0, 0, 0, conn.endshape, conn.names, 0, added);
      directededge.set_edgeinfo_offset(edge_info_offset);
      tilebuilder_local.directededges().emplace_back(std::move(directededge));
      ++added_edges;
    }
    tilebuilder_local.nodes().emplace_back(std::move(new_bss_node));
    ++added_nodes;
  }
  LOG_INFO(std::string("Added: ") + std::to_string(added_edges) + " edges and " +
           std::to_string(added_nodes) + " bss nodes");
}

using bss_by_tile_t = std::unordered_map<GraphId, std::vector<OSMNode>>;

void build(const boost::property_tree::ptree& pt,
           std::mutex& lock,
           bss_by_tile_t::const_iterator tile_start,
           bss_by_tile_t::const_iterator tile_end) {

  auto reader_local_level = GraphReader{pt};
  for (; tile_start != tile_end; ++tile_start) {

    const GraphTile* local_tile = nullptr;
    std::unique_ptr<GraphTileBuilder> tilebuilder_local = nullptr;
    {
      std::lock_guard<std::mutex> l(lock);

      auto tile_id = tile_start->first;
      local_tile = reader_local_level.GetGraphTile(tile_id);
      tilebuilder_local.reset(new GraphTileBuilder{reader_local_level.tile_dir(), tile_id, true});
    }

    auto new_connections = project(*local_tile, tile_start->second);
    create_bss_node_and_edges(*tilebuilder_local, *local_tile, lock, std::move(new_connections));
  }
}

} // namespace

namespace valhalla {
namespace mjolnir {

// Add bss to the graph
void BssBuilder::Build(const boost::property_tree::ptree& pt, const std::string& bss_nodes_bin) {

  if (!pt.get<bool>("mjolnir.import_bike_share_stations", false)) {
    return;
  }

  LOG_INFO("Importing Bike Share station");

  auto t1 = std::chrono::high_resolution_clock::now();

  auto scoped_finally = make_finally([&t1]() {
    auto t2 = std::chrono::high_resolution_clock::now();
    uint32_t secs = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
    LOG_INFO("Finished - BssBuilder took " + std::to_string(secs) + " secs");
  });

  midgard::sequence<mjolnir::OSMNode> osm_nodes{bss_nodes_bin, false};

  // bss_by_tile_t bss_by_tile;
  bss_by_tile_t bss_by_tile;

  auto reader = GraphReader{pt.get_child("mjolnir")};
  auto local_level = TileHierarchy::levels().rbegin()->first;
  // Group the nodes by their tiles. In the next step, we will work on each tile only once
  for (const auto& node : osm_nodes) {
    auto latlng = node.latlng();
    auto tile_id = TileHierarchy::GetGraphId({latlng.first, latlng.second}, local_level);
    const GraphTile* local_tile = reader.GetGraphTile(tile_id);
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
    threads[i].reset(new std::thread(build, std::cref(pt.get_child("mjolnir")), std::ref(lock),
                                     tile_start, tile_end));
  }

  for (auto& thread : threads) {
    thread->join();
  }
}

} // namespace mjolnir
} // namespace valhalla
