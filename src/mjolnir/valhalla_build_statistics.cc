#include "argparse_utils.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/nodeinfo.h"
#include "baldr/tilehierarchy.h"
#include "midgard/aabb2.h"
#include "midgard/distanceapproximator.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "statistics.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <cstdint>
#include <deque>
#include <filesystem>
#include <future>
#include <iostream>
#include <list>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

struct HGVRestrictionTypes {
  bool hazmat;
  bool axle_load;
  bool height;
  bool length;
  bool weight;
  bool width;
  bool axles;
};

bool IsUnroutableNode(const GraphTile& tile,
                      const NodeInfo& startnodeinfo,
                      statistics::RouletteData& rd) {

  const DirectedEdge* diredge = tile.directededge(startnodeinfo.edge_index());
  size_t inbound = 0, outbound = 0;
  // Check all the edges from the current node and count inbound and outbound edges
  for (size_t i = 0; i < startnodeinfo.edge_count(); i++, diredge++) {
    if (diredge->shortcut() || diredge->use() == Use::kTransitConnection ||
        diredge->use() == Use::kEgressConnection || diredge->use() == Use::kPlatformConnection) {
      continue;
    }
    if ((diredge->forwardaccess() & kAutoAccess)) {
      outbound++;
    }
    if ((diredge->reverseaccess() & kAutoAccess)) {
      inbound++;
    }
  }

  // If there is a way in and no way out, or vice versa
  // And it's not a dead end
  // Or it is a dead end, but is a high class road
  if (((!outbound && inbound >= 2) || (outbound >= 2 && !inbound))) {
    rd.AddNode(startnodeinfo.latlng(tile.header()->base_ll()));
    return true;
  }

  return false;
}

void checkExitInfo(GraphReader& reader,
                   const GraphId& startnode,
                   const NodeInfo& startnodeinfo,
                   const DirectedEdge& directededge,
                   statistics& stats) {
  // If this edge is right after a motorway junction it is an exit and should
  // have signs
  if (startnodeinfo.type() == NodeType::kMotorWayJunction) {
    // Check to see if the motorway continues, if it does, this is an exit ramp,
    // otherwise if all forward edges are links, it is a fork
    graph_tile_ptr tile = reader.GetGraphTile(startnode);
    const DirectedEdge* otheredge = tile->directededge(startnodeinfo.edge_index());
    std::vector<std::pair<uint64_t, bool>> tile_fork_signs;
    std::vector<std::pair<std::string, bool>> ctry_fork_signs;
    // Assume it is a fork
    bool fork = true;
    for (size_t i = 0; i < startnodeinfo.edge_count(); ++i, ++otheredge) {
      // If it is an outgoing edge that is not a link, it is not a fork
      if (((otheredge->forwardaccess() & kAutoAccess) &&
           !(otheredge->reverseaccess() & kAutoAccess)) &&
          !otheredge->link()) {
        fork = false;
        // no need to keep checking if it's not a fork
        break;
      } else {
        // store exit info in case this is a fork
        std::string iso_code = tile->admin(startnodeinfo.admin_index())->country_iso();
        tile_fork_signs.push_back({tile->id(), otheredge->sign()});
        ctry_fork_signs.push_back({iso_code, otheredge->sign()});
      }
    }
    // If it was a fork, store the data appropriately
    if (fork) {
      for (auto& sign : tile_fork_signs) {
        stats.add_fork_exitinfo(sign);
      }
      for (auto& sign : ctry_fork_signs) {
        stats.add_fork_exitinfo(sign);
      }
    } else {
      // Otherwise store original edge info as a normal exit
      std::string iso_code = tile->admin(startnodeinfo.admin_index())->country_iso();
      stats.add_exitinfo({tile->id(), directededge.sign()});
      stats.add_exitinfo({iso_code, directededge.sign()});
    }
  }
}

void AddStatistics(statistics& stats,
                   const DirectedEdge& directededge,
                   const uint32_t tileid,
                   std::string& begin_node_iso,
                   HGVRestrictionTypes& hgv,
                   const GraphTile& tile,
                   GraphReader& graph_reader,
                   GraphId& node,
                   const NodeInfo& nodeinfo) {

  const auto rclass = directededge.classification();
  float edge_length = (tileid == directededge.endnode().tileid()) ? directededge.length() * 0.5f
                                                                  : directededge.length() * 0.25f;

  // Add truck stats.
  if (directededge.truck_route()) {
    stats.add_tile_truck_route(tileid, rclass, edge_length);
    stats.add_country_truck_route(begin_node_iso, rclass, edge_length);
  }
  if (hgv.hazmat) {
    stats.add_tile_hazmat(tileid, rclass, edge_length);
    stats.add_country_hazmat(begin_node_iso, rclass, edge_length);
  }
  if (hgv.axle_load) {
    stats.add_tile_axle_load(tileid, rclass);
    stats.add_country_axle_load(begin_node_iso, rclass);
  }
  if (hgv.height) {
    stats.add_tile_height(tileid, rclass);
    stats.add_country_height(begin_node_iso, rclass);
  }
  if (hgv.length) {
    stats.add_tile_length(tileid, rclass);
    stats.add_country_length(begin_node_iso, rclass);
  }
  if (hgv.weight) {
    stats.add_tile_weight(tileid, rclass);
    stats.add_country_weight(begin_node_iso, rclass);
  }
  if (hgv.width) {
    stats.add_tile_width(tileid, rclass);
    stats.add_country_width(begin_node_iso, rclass);
  }

  // Check for exit signage if it is a highway link
  if (directededge.link() && (rclass == RoadClass::kMotorway || rclass == RoadClass::kTrunk)) {
    checkExitInfo(graph_reader, node, nodeinfo, directededge, stats);
  }

  // Add all other statistics
  // Only consider edge if edge is good and it's not a link
  if (!directededge.link()) {
    edge_length *= 0.5f;
    bool found = IsUnroutableNode(tile, nodeinfo, stats.roulette_data);
    if (!found) {
      // IsLoop(graph_reader,directededge,node,stats.roulette_data);
    }
    stats.add_tile_one_way(tileid, rclass, edge_length);
    stats.add_country_one_way(begin_node_iso, rclass, edge_length);

    // Check if this edge is internal
    if (directededge.internal()) {
      stats.add_tile_int_edge(tileid, rclass);
      stats.add_country_int_edge(begin_node_iso, rclass);
    }
    // Check if edge has maxspeed tag
    if (directededge.speed_type() == SpeedType::kTagged) {
      stats.add_tile_speed_info(tileid, rclass, edge_length);
      stats.add_country_speed_info(begin_node_iso, rclass, edge_length);
    }
    // Check if edge has any names
    if (tile.edgeinfo(&directededge).name_count() > 0) {
      stats.add_tile_named(tileid, rclass, edge_length);
      stats.add_country_named(begin_node_iso, rclass, edge_length);
    }

    // Add road lengths to statistics for current country and tile
    stats.add_country_road(begin_node_iso, rclass, edge_length);
    stats.add_tile_road(tileid, rclass, edge_length);
  }
}

void build(const boost::property_tree::ptree& pt,
           std::deque<GraphId>& tilequeue,
           std::mutex& lock,
           std::promise<statistics>& result) {
  // Our local class for gathering the stats
  statistics stats;
  // Local Graphreader
  GraphReader graph_reader(pt.get_child("mjolnir"));

  // Check for more tiles
  while (true) {
    lock.lock();
    if (tilequeue.empty()) {
      lock.unlock();
      break;
    }
    // Get the next tile Id
    GraphId tile_id = tilequeue.front();
    tilequeue.pop_front();
    lock.unlock();

    // Point tiles to the set we need for current level
    auto level = tile_id.level();
    if (TileHierarchy::levels().back().level + 1u == level) {
      level = TileHierarchy::levels().back().level;
    }

    const auto& tiles = TileHierarchy::levels()[level].tiles;
    level = tile_id.level();
    auto tileid = tile_id.tileid();

    // Update nodes and directed edges as needed
    std::vector<DirectedEdge> directededges;

    // Get this tile
    graph_tile_ptr tile = graph_reader.GetGraphTile(tile_id);

    // Iterate through the nodes and the directed edges
    uint32_t nodecount = tile->header()->nodecount();
    GraphId node = tile_id;
    for (uint64_t i = 0; i < nodecount; i++, ++node) {
      // The node we will modify
      const NodeInfo* nodeinfo = tile->node(node);
      std::string begin_node_iso = tile->admin(nodeinfo->admin_index())->country_iso();

      // Go through directed edges
      uint32_t idx = nodeinfo->edge_index();
      for (uint32_t j = 0, n = nodeinfo->edge_count(); j < n; j++, idx++) {
        auto de = tile->directededge(idx);

        // HGV restriction mask (for stats)
        HGVRestrictionTypes hgv = {};

        // check access restrictions. TODO - should check modes as well
        uint32_t ar_modes = de->access_restriction();
        if (ar_modes) {
          // since only truck restrictions exist, we can still get all restrictions
          // later we may only want to get just the truck ones for stats.
          auto res = tile->GetAccessRestrictions(idx, kAllAccess);
          if (res.size() == 0) {
            LOG_ERROR(
                "Directed edge marked as having access restriction but none found ; tile level = " +
                std::to_string(tile_id.level()));
          } else {
            for (auto r : res) {
              if (r.edgeindex() != idx) {
                LOG_ERROR("Access restriction edge index does not match idx");
              } else {
                switch (r.type()) {
                  case AccessType::kHazmat:
                    hgv.hazmat = true;
                    break;
                  case AccessType::kMaxAxleLoad:
                    hgv.axle_load = true;
                    break;
                  case AccessType::kMaxAxles:
                    hgv.axles = true;
                    break;
                  case AccessType::kMaxHeight:
                    hgv.height = true;
                    break;
                  case AccessType::kMaxLength:
                    hgv.length = true;
                    break;
                  case AccessType::kMaxWeight:
                    hgv.weight = true;
                    break;
                  case AccessType::kMaxWidth:
                    hgv.width = true;
                    break;
                  default:
                    break;
                }
              }
            }
          }
        }
        // The edge we will modify
        const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index() + j);

        // Road Length and some variables for statistics
        bool valid_length = false;
        if (!directededge->shortcut()) {
          valid_length = true;
        }

        // Statistics
        if (valid_length) {
          AddStatistics(stats, *directededge, tileid, begin_node_iso, hgv, *tile, graph_reader, node,
                        *nodeinfo);
        }
      }
    }

    // Add density to return class. Approximate the tile area square km
    AABB2<PointLL> bb = tiles.TileBounds(tileid);
    float area = ((bb.maxy() - bb.miny()) * kMetersPerDegreeLat * kKmPerMeter) *
                 ((bb.maxx() - bb.minx()) *
                  DistanceApproximator<PointLL>::MetersPerLngDegree(bb.Center().y()) * kKmPerMeter);
    stats.add_tile_area(tileid, area);
    stats.add_tile_geom(tileid, tiles.TileBounds(tileid));

    // Check if we need to clear the tile cache
    lock.lock();
    if (graph_reader.OverCommitted()) {
      graph_reader.Trim();
    }
    lock.unlock();
  }

  // Fill promise with statistics
  result.set_value(std::move(stats));
}
} // namespace

void BuildStatistics(const boost::property_tree::ptree& pt) {

  // Graph tile properties
  auto tile_properties = pt.get_child("mjolnir");

  GraphReader reader(tile_properties);

  // Create a randomized queue of tiles to work from
  std::deque<GraphId> tilequeue;
  for (const auto& tier : TileHierarchy::levels()) {
    auto level = tier.level;
    const auto& tiles = tier.tiles;
    for (uint32_t id = 0; id < tiles.TileCount(); id++) {
      // If tile exists add it to the queue
      GraphId tile_id(id, level, 0);
      if (reader.DoesTileExist(tile_id)) {
        tilequeue.emplace_back(std::move(tile_id));
      }
    }

    // transit level
    if (level == TileHierarchy::levels().back().level) {
      level += 1;
      for (uint32_t id = 0; id < tiles.TileCount(); id++) {
        // If tile exists add it to the queue
        GraphId tile_id(id, level, 0);
        if (reader.DoesTileExist(tile_id)) {
          tilequeue.emplace_back(std::move(tile_id));
        }
      }
    }
  }
  std::random_device rd;
  std::shuffle(tilequeue.begin(), tilequeue.end(), std::mt19937(rd()));

  // A mutex we can use to do the synchronization
  std::mutex lock;

  LOG_INFO("Gathering information about the tiles in " + pt.get<std::string>("mjolnir.tile_dir"));

  // Setup threads
  std::vector<std::shared_ptr<std::thread>> threads(
      std::max(static_cast<unsigned int>(1),
               pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency())));

  // Setup promises
  std::list<std::promise<statistics>> results;

  // Spawn the threads
  for (auto& thread : threads) {
    results.emplace_back();
    thread = std::make_shared<std::thread>(build, std::cref(pt), std::ref(tilequeue), std::ref(lock),
                                           std::ref(results.back()));
  }

  // Wait for threads to finish
  for (auto& thread : threads) {
    thread->join();
  }
  // Get the promise from the future
  statistics stats;
  for (auto& result : results) {
    auto r = result.get_future().get();
    // keep track of stats
    stats.add(r);
  }
  LOG_INFO("Finished");

  stats.build_db();
  stats.roulette_data.GenerateTasks(pt);
}

int main(int argc, char** argv) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  // args
  boost::property_tree::ptree config;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_PRINT_VERSION + "\n\n"
      "valhalla_build_statistics is a program that builds a statistics database.\n\n");

    options.add_options()
      ("h,help", "Print this help message")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("j,concurrency", "Number of threads to use. Defaults to all threads.", cxxopts::value<uint32_t>());
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config, "mjolnir.logging", true))
      return EXIT_SUCCESS;
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  BuildStatistics(config);

  return EXIT_SUCCESS;
}
