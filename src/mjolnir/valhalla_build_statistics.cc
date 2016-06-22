
#include "mjolnir/graphvalidator.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/statistics.h"

#include <valhalla/midgard/logging.h>

#include <ostream>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <queue>
#include <list>
#include <thread>
#include <future>
#include <mutex>
#include <numeric>

#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/nodeinfo.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace bpo = boost::program_options;
boost::filesystem::path config_file_path;

namespace {

struct HGVRestrictionTypes {
  bool hazmat;
  bool axle_load;
  bool height;
  bool length;
  bool weight;
  bool width;
};

bool IsPedestrianTerminal(GraphTileBuilder &tilebuilder, GraphReader& reader, std::mutex& lock,
                const GraphId& startnode,
                NodeInfo& startnodeinfo,
                DirectedEdge& directededge,
                validator_stats::RouletteData& rd,
                const uint32_t idx) {

  bool is_terminal = true;
  lock.lock();
  const GraphTile* tile = reader.GetGraphTile(startnode);
  lock.unlock();
  const DirectedEdge* diredge = tile->directededge(startnodeinfo.edge_index());
  for (uint32_t i = 0; i < startnodeinfo.edge_count(); i++, diredge++) {

    if (i == idx)
      continue;

    if (((diredge->reverseaccess() & kPedestrianAccess) ||
        (diredge->forwardaccess() & kPedestrianAccess)) &&
        (!(diredge->forwardaccess() & kAutoAccess) &&
          !(diredge->reverseaccess() & kAutoAccess))){
      continue;
    }
    else {
      is_terminal = false;
      break;
    }
  }

  if (is_terminal) {
    if (startnodeinfo.edge_count() > 1) {
      rd.AddTask(startnodeinfo.latlng(),
                 tilebuilder.edgeinfo(directededge.edgeinfo_offset())->wayid(),
                 tilebuilder.edgeinfo(directededge.edgeinfo_offset())->shape());
      return true;
    }
  }
  return false;
}

bool IsLoopTerminal(GraphTileBuilder &tilebuilder, GraphReader& reader, std::mutex& lock,
                const GraphId& startnode,
                NodeInfo& startnodeinfo,
                DirectedEdge& directededge,
                validator_stats::RouletteData& rd) {

  lock.lock();
  const GraphTile* tile = reader.GetGraphTile(startnode);
  lock.unlock();
  const DirectedEdge* diredge = tile->directededge(startnodeinfo.edge_index());
  uint32_t inbound = 0, outbound = 0;

  for (uint32_t i = 0; i < startnodeinfo.edge_count(); i++, diredge++) {

    if (((diredge->forwardaccess() & kAutoAccess) &&
        !(diredge->reverseaccess() & kAutoAccess)) ||
        ((diredge->forwardaccess() & kAutoAccess) &&
         (diredge->reverseaccess() & kAutoAccess)))
      outbound++;

    if ((!(diredge->forwardaccess() & kAutoAccess) &&
        (diredge->reverseaccess() & kAutoAccess)) ||
        ((diredge->forwardaccess() & kAutoAccess) &&
         (diredge->reverseaccess() & kAutoAccess)))
      inbound++;
  }

  if ((outbound >= 2 && inbound == 0) || (inbound >= 2 && outbound == 0)) {
    rd.AddTask(startnodeinfo.latlng(),
               tilebuilder.edgeinfo(directededge.edgeinfo_offset())->wayid(),
               tilebuilder.edgeinfo(directededge.edgeinfo_offset())->shape());
    return true;
  }
  return false;
}

bool IsReversedOneway(GraphTileBuilder &tilebuilder, GraphReader& reader, std::mutex& lock,
                const GraphId& startnode,
                NodeInfo& startnodeinfo,
                DirectedEdge& directededge,
                validator_stats::RouletteData& rd) {

  lock.lock();
  const GraphTile* tile = reader.GetGraphTile(startnode);
  lock.unlock();
  const DirectedEdge* diredge = tile->directededge(startnodeinfo.edge_index());
  uint32_t inbound = 0, outbound = 0;

  for (uint32_t i = 0; i < startnodeinfo.edge_count(); i++, diredge++) {

    if ((diredge->forwardaccess() & kAutoAccess) &&
        !(diredge->reverseaccess() & kAutoAccess))
      outbound++;

    if (!(diredge->forwardaccess() & kAutoAccess) &&
        (diredge->reverseaccess() & kAutoAccess))
      inbound++;
  }

  if (!outbound && inbound)
  {
    const GraphId endnode = directededge.endnode();
    lock.lock();
    const GraphTile* tile = reader.GetGraphTile(endnode);
    lock.unlock();
    const NodeInfo* nodeinfo = tile->node(endnode.id());

    const DirectedEdge* diredge = tile->directededge(nodeinfo->edge_index());
    uint32_t inbound = 0, outbound = 0;

    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, diredge++) {

      if ((diredge->forwardaccess() & kAutoAccess) &&
          !(diredge->reverseaccess() & kAutoAccess))
        outbound++;

      if (!(diredge->forwardaccess() & kAutoAccess) &&
          (diredge->reverseaccess() & kAutoAccess))
        inbound++;
    }

    if (!outbound && inbound) {
      rd.AddTask(startnodeinfo.latlng(),
                 tilebuilder.edgeinfo(directededge.edgeinfo_offset())->wayid(),
                 tilebuilder.edgeinfo(directededge.edgeinfo_offset())->shape());
      return true;
    }
  }

  return false;
}

void AddStatistics(validator_stats& stats, DirectedEdge& directededge,
                   const uint32_t tileid, std::string& begin_node_iso,
                   HGVRestrictionTypes& hgv, GraphTileBuilder& tilebuilder,
                   GraphReader& graph_reader, std::mutex& lock, GraphId& node, NodeInfo& nodeinfo, uint32_t idx) {
  auto rclass = directededge.classification();
  float edge_length = (tileid == directededge.endnode().tileid()) ?
        directededge.length() * 0.5f : directededge.length() * 0.25f;

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

  // Add all other statistics
  // Only consider edge if edge is good and it's not a link
  if (!directededge.link()) {
    //Determine access for directed edge
    auto fward = ((kAutoAccess & directededge.forwardaccess()) == kAutoAccess);
    auto bward = ((kAutoAccess & directededge.reverseaccess()) == kAutoAccess);
    // Check if one way
    if ((!fward || !bward) && (fward || bward)) {
      edge_length *= 0.5f;
      const GraphTile* tile = graph_reader.GetGraphTile(node);
      bool found = IsPedestrianTerminal(tilebuilder,graph_reader,lock,node,nodeinfo,directededge,stats.roulette_data,idx);
      if (!found && directededge.endnode().id() == node.id()) {
        lock.lock();
        const GraphTile* end_tile = graph_reader.GetGraphTile(directededge.endnode());
        lock.unlock();
        if (tile->id() == end_tile->id())
          found = IsLoopTerminal(tilebuilder,graph_reader,lock,node,nodeinfo,directededge,stats.roulette_data);
      }
      if (!found && directededge.endnode().id() != node.id()) {
        found = IsReversedOneway(tilebuilder,graph_reader,lock,node,nodeinfo,directededge,stats.roulette_data);
      }
      stats.add_tile_one_way(tileid, rclass, edge_length);
      stats.add_country_one_way(begin_node_iso, rclass, edge_length);
    }
    // Check if this edge is internal
    if (directededge.internal()) {
      stats.add_tile_int_edge(tileid, rclass);
      stats.add_country_int_edge(begin_node_iso, rclass);
    }
    // Check if edge has maxspeed tag
    if (directededge.speed_type() == SpeedType::kTagged){
      stats.add_tile_speed_info(tileid, rclass, edge_length);
      stats.add_country_speed_info(begin_node_iso, rclass, edge_length);
    }
    // Check if edge has any names
    if (tilebuilder.edgeinfo(directededge.edgeinfo_offset())->name_count() > 0) {
      stats.add_tile_named(tileid, rclass, edge_length);
      stats.add_country_named(begin_node_iso, rclass, edge_length);
    }

    // Add road lengths to statistics for current country and tile
    stats.add_country_road(begin_node_iso, rclass, edge_length);
    stats.add_tile_road(tileid, rclass, edge_length);
  }
}

void validate(const boost::property_tree::ptree& pt,
              std::deque<GraphId>& tilequeue, std::mutex& lock,
              std::promise<validator_stats>& result) {

    // Our local class for gathering the stats
    validator_stats stats;
    // Local Graphreader
    GraphReader graph_reader(pt.get_child("mjolnir"));
    // Get some things we need throughout
    const auto& hierarchy = graph_reader.GetTileHierarchy();

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
      if (hierarchy.levels().rbegin()->second.level+1 == level)
        level = hierarchy.levels().rbegin()->second.level;

      const auto& tiles = hierarchy.levels().find(level)->second.tiles;
      level = tile_id.level();
      auto tileid = tile_id.tileid();

      // Get the tile
      GraphTileBuilder tilebuilder(hierarchy, tile_id, false);

      // Update nodes and directed edges as needed
      std::vector<NodeInfo> nodes;
      std::vector<DirectedEdge> directededges;

      // Get this tile
      lock.lock();
      const GraphTile* tile = graph_reader.GetGraphTile(tile_id);
      lock.unlock();

      // Iterate through the nodes and the directed edges
      float roadlength = 0.0f;
      uint32_t nodecount = tilebuilder.header()->nodecount();
      GraphId node = tile_id;
      for (uint32_t i = 0; i < nodecount; i++, node++) {
        // The node we will modify
        NodeInfo nodeinfo = tilebuilder.node(i);
        auto ni = tile->node(i);
        std::string begin_node_iso = tile->admin(nodeinfo.admin_index())->country_iso();

        // Go through directed edges and validate/update data
        uint32_t idx = ni->edge_index();
        for (uint32_t j = 0, n = nodeinfo.edge_count(); j < n; j++, idx++) {
          auto de = tile->directededge(idx);

          // HGV restriction mask (for stats)
          HGVRestrictionTypes hgv = {};

          // Validate access restrictions. TODO - should check modes as well
          uint32_t ar_modes = de->access_restriction();
          if (ar_modes) {
            //since only truck restrictions exist, we can still get all restrictions
            //later we may only want to get just the truck ones for stats.
            auto res = tile->GetAccessRestrictions(idx, kAllAccess);
            if (res.size() == 0) {
              LOG_ERROR("Directed edge marked as having access restriction but none found ; tile level = " +
                        std::to_string(tile_id.level()));
            } else {
              for (auto r : res) {
                if (r.edgeindex() != idx) {
                  LOG_ERROR("Access restriction edge index does not match idx");
                } else {// log stats.  currently only for truck right now
                  switch (r.type()) {
                    case AccessType::kHazmat:
                      hgv.hazmat = true;
                      break;
                    case AccessType::kMaxAxleLoad:
                      hgv.axle_load = true;
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
          DirectedEdge& directededge = tilebuilder.directededge(nodeinfo.edge_index() + j);

          // Road Length and some variables for statistics
          float edge_length;
          bool valid_length = false;
          if (!directededge.shortcut() && !directededge.trans_up() &&
              !directededge.trans_down()) {
            edge_length = directededge.length();
            roadlength += edge_length;
            valid_length = true;
          }

          // Statistics
          if (valid_length) {
            AddStatistics(stats, directededge, tileid, begin_node_iso,
                          hgv, tilebuilder, graph_reader, lock, node, nodeinfo, j);
          }
        }

        // Add the node to the list
        nodes.emplace_back(std::move(nodeinfo));
      }

      // Add density to return class. Approximate the tile area square km
      AABB2<PointLL> bb = tiles.TileBounds(tileid);
      float area = ((bb.maxy() - bb.miny()) * kMetersPerDegreeLat * kKmPerMeter) *
                   ((bb.maxx() - bb.minx()) *
                     DistanceApproximator::MetersPerLngDegree(bb.Center().y()) * kKmPerMeter);
      stats.add_tile_area(tileid, area);
      stats.add_tile_geom(tileid, tiles.TileBounds(tileid));


      // Check if we need to clear the tile cache
      lock.lock();
      if (graph_reader.OverCommitted())
        graph_reader.Clear();
      lock.unlock();
    }

    // Fill promise with statistics
    result.set_value(std::move(stats));
  }
}

void GraphValidator::Validate(const boost::property_tree::ptree& pt) {

  // Graphreader
  TileHierarchy hierarchy(pt.get<std::string>("mjolnir.tile_dir"));
  // Make sure there are at least 2 levels!
  if (hierarchy.levels().size() < 2)
    throw std::runtime_error("Bad tile hierarchy - need 2 levels");

  // Create a randomized queue of tiles to work from
  std::deque<GraphId> tilequeue;
  for (auto tier : hierarchy.levels()) {
    auto level = tier.second.level;
    auto tiles = tier.second.tiles;
    for (uint32_t id = 0; id < tiles.TileCount(); id++) {
      // If tile exists add it to the queue
      GraphId tile_id(id, level, 0);
      if (GraphReader::DoesTileExist(hierarchy, tile_id)) {
        tilequeue.emplace_back(std::move(tile_id));
      }
    }

    //transit level
    if (level == hierarchy.levels().rbegin()->second.level) {
      level += 1;
      for (uint32_t id = 0; id < tiles.TileCount(); id++) {
        // If tile exists add it to the queue
        GraphId tile_id(id, level, 0);
        if (GraphReader::DoesTileExist(hierarchy, tile_id)) {
          tilequeue.emplace_back(std::move(tile_id));
        }
      }
    }
  }
  std::random_shuffle(tilequeue.begin(), tilequeue.end());

  // A mutex we can use to do the synchronization
  std::mutex lock;

  LOG_INFO("Validating signs and connectivity and binning edges");

  // Setup threads
  std::vector<std::shared_ptr<std::thread> > threads(
      std::max(static_cast<unsigned int>(1),
               pt.get<unsigned int>("concurrency",std::thread::hardware_concurrency())));

  // Setup promises
  std::list<std::promise<validator_stats> > results;

  // Spawn the threads
  for (auto& thread : threads) {
    results.emplace_back();
    thread.reset(new std::thread(validate, std::cref(pt), std::ref(tilequeue),
                                 std::ref(lock), std::ref(results.back())));
  }

  // Wait for threads to finish
  for (auto& thread : threads)
    thread->join();
  // Get the promise from the future
  validator_stats stats;
  for (auto& result : results) {
    auto r = result.get_future().get();
    //keep track of stats
    stats.add(r);
  }
  LOG_INFO("Finished");

  // Add up total dupcount_ and find densities
  for (uint8_t level = 0; level <= 2; level++) {
    // Print duplicates info for level
    std::vector<uint32_t> dups = stats.get_dups(level);
    uint32_t dupcount = std::accumulate(dups.begin(), dups.end(), 0);
    LOG_WARN((boost::format("Possible duplicates at level: %1% = %2%")
    % std::to_string(level) % dupcount).str());
    // Get the average density and the max density
    float max_density = 0.0f;
    float sum = 0.0f;
    for (auto density : stats.get_densities(level)) {
      if (density > max_density) {
        max_density = density;
      }
      sum += density;
    }
    float average_density = sum / stats.get_densities(level).size();
    LOG_DEBUG("Average density = " + std::to_string(average_density) +
             " max = " + std::to_string(max_density));
  }
  stats.build_db(pt);
  stats.roulette_data.GenerateTasks();
}

bool ParseArguments(int argc, char *argv[]) {
  bpo::options_description options("Usage: valhalla_build_statistics --config conf/valhalla.json");
  options.add_options()
    ("help,h", "Print this help message")
    ("config,c",
     boost::program_options::value<boost::filesystem::path>(&config_file_path)->required(),
     "Path to the json configuration file.");
  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).run(), vm);
    bpo::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n";
    return false;
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return true;
  }
  if (vm.count("config")) {
    if (boost::filesystem::is_regular_file(config_file_path))
      return true;
    else
      std::cerr << "Configuration file is required\n\n" << options << "\n\n";
  }

  return false;
}

int main (int argc, char** argv) {
  if (!ParseArguments(argc, argv))
    return EXIT_FAILURE;

}
