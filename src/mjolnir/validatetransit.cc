#include "mjolnir/validatetransit.h"
#include "mjolnir/dataquality.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/osmrestriction.h"
#include "mjolnir/servicedays.h"

#include <future>
#include <queue>
#include <thread>

#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "midgard/sequence.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

// Struct to hold stats information during each threads work
struct validate_stats {
  uint32_t failure_count;

  // Accumulate stats from all threads
  void operator()(const validate_stats& other) {
    failure_count = other.failure_count;
  }
};

// Walk the transit data starting at the origin onestop_id/node and walk the edges
// until we find the destination onestop_id/node.  Must walk via the route
// onestop_id provided in the tests.
bool WalkTransitLines(const GraphId& n_graphId,
                      GraphReader& reader,
                      std::mutex& lock,
                      std::unordered_multimap<GraphId, uint64_t>& visited_map,
                      const std::string& date_time,
                      const std::string& end_name,
                      const std::string& route_name) {
  lock.lock();
  auto endnodetile = reader.GetGraphTile(n_graphId);
  assert(endnodetile);
  lock.unlock();
  const NodeInfo* n_info = endnodetile->node(n_graphId);
  GraphId currentNode = n_graphId;

  uint32_t tripid = 0;
  uint32_t j = 0;
  uint32_t localtime = DateTime::seconds_from_midnight(date_time);
  uint32_t date = DateTime::days_from_pivot_date(DateTime::get_formatted_date(date_time));
  uint32_t dow = DateTime::day_of_week_mask(date_time);
  uint32_t date_created = endnodetile->header()->date_created();
  uint32_t day = 0;
  uint32_t time_added = 0;
  if (date >= date_created) {
    day = date - date_created;
  }

  bool bDone = false;

  while (j < n_info->edge_count() && time_added <= 14400) {

    const DirectedEdge* de = endnodetile->directededge(n_info->edge_index() + j);

    GraphId edgeid(currentNode.tileid(), currentNode.level(), n_info->edge_index() + j);

    // all should be a transit lines, but make sure.
    if (de->IsTransitLine()) {

      const TransitDeparture* departure =
          endnodetile->GetNextDeparture(de->lineid(), localtime, day, dow, false, false, false);

      // Don't want to walk an edge at a particular time more than once.
      bool visited = false;
      if (departure) {
        auto itr = visited_map.equal_range(edgeid);
        for (auto it = itr.first; it != itr.second; ++it) {
          if (it->second == departure->departure_time()) {
            visited = true;
            break;
          }
        }
      }

      // are we on the same trip from the previous edge?
      if (departure && !visited && (tripid == 0 || departure->tripid() == tripid)) {

        const TransitRoute* route = endnodetile->GetTransitRoute(departure->routeindex());
        // are we on the correct route?
        if (endnodetile->GetName(route->one_stop_offset()) == route_name) {
          visited_map.emplace(edgeid, departure->departure_time());
          currentNode = de->endnode();

          // if (tripid == 0) then this is the first pass.
          if (tripid == 0) {
            tripid = departure->tripid();
          }

          // get the new tile if needed.
          if (endnodetile->id() != currentNode.Tile_Base()) {
            lock.lock();
            endnodetile = reader.GetGraphTile(currentNode);
            lock.unlock();
          }
          // get new end node and start over if needed.
          n_info = endnodetile->node(currentNode);

          // are we done?
          std::string station_name;
          // stations are in the same tile as platforms and the tests contain the station
          // onestop ids and not the platforms
          uint32_t z = 0;
          while (z < n_info->edge_count()) {

            const DirectedEdge* de = endnodetile->directededge(n_info->edge_index() + z);

            // get the station node
            if (de->use() == Use::kPlatformConnection) {

              const TransitStop* transit_station =
                  endnodetile->GetTransitStop(endnodetile->node(de->endnode())->stop_index());

              station_name = endnodetile->GetName(transit_station->one_stop_offset());
              break;
            }
            z++;
          }

          if (station_name == end_name) {
            bDone = true;
            break;
          }
          localtime = departure->departure_time() + departure->elapsed_time();
          time_added = 0;
          j = 0;
          continue;
        }
      }
    }
    // if we get here add 30 sec and try again.  up to 4 hrs.
    if (j + 1 == n_info->edge_count()) {
      time_added += 30;
      localtime += 30;
      j = 0;
      tripid = 0;
      continue;
    }
    j++;
  }
  return bDone;
}

// validate the transit data using the one_stop tests.
void validate(const boost::property_tree::ptree& pt,
              std::mutex& lock,
              std::unordered_set<GraphId>::const_iterator tile_start,
              std::unordered_set<GraphId>::const_iterator tile_end,
              const std::vector<OneStopTest>& onestoptests,
              std::promise<validate_stats>& results) {

  uint32_t failure_count = 0;
  GraphReader reader_transit_level(pt);

  std::unordered_multimap<std::string, std::string> passed_tests;
  std::unordered_multimap<std::string, std::string> failed_tests;

  // Iterate through the tiles in the queue and find any that include stops
  for (; tile_start != tile_end; ++tile_start) {
    // Get the next tile Id from the queue and get a tile builder
    if (reader_transit_level.OverCommitted()) {
      reader_transit_level.Trim();
    }
    GraphId tile_id = tile_start->Tile_Base();

    lock.lock();
    GraphId transit_tile_id = GraphId(tile_id.tileid(), tile_id.level() + 1, tile_id.id());
    auto transit_tile = reader_transit_level.GetGraphTile(transit_tile_id);
    GraphTileBuilder tilebuilder(reader_transit_level.tile_dir(), transit_tile_id, true);
    lock.unlock();

    for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); i++) {
      NodeInfo& nodeinfo = tilebuilder.node_builder(i);

      std::string station_name;
      // all should be multiuseplatform, but check just to be sure.
      if (nodeinfo.type() == NodeType::kMultiUseTransitPlatform) {

        // stations are in the same tile as platforms and the tests contain the station
        // onestop ids and not the platforms
        uint32_t j = 0;
        while (j < nodeinfo.edge_count()) {

          const DirectedEdge* de = transit_tile->directededge(nodeinfo.edge_index() + j);

          // get the station node
          if (de->use() == Use::kPlatformConnection) {
            const TransitStop* transit_station =
                transit_tile->GetTransitStop(transit_tile->node(de->endnode())->stop_index());

            station_name = transit_tile->GetName(transit_station->one_stop_offset());
            break;
          }
          j++;
        }

        OneStopTest ost;
        ost.origin = station_name;
        auto p = std::equal_range(onestoptests.begin(), onestoptests.end(), ost);

        for (auto t = p.first; t != p.second; ++t) {

          // has this test passed already?
          // don't want to run again for another platform under a station.
          bool bfound = false;
          auto tests = passed_tests.equal_range(t->origin);
          for (auto it = tests.first; it != tests.second; ++it) {
            if (it->second == (t->destination + " @ " + t->date_time + " " + t->route_id)) {
              bfound = true;
              break;
            }
          }

          if (bfound) {
            continue;
          }

          GraphId currentNode = GraphId(transit_tile->id().tileid(), transit_tile->id().level(), i);
          std::unordered_multimap<GraphId, uint64_t> visited_map;

          if (!WalkTransitLines(currentNode, reader_transit_level, lock, visited_map, t->date_time,
                                t->destination, t->route_id)) {

            // Try again avoiding the departures found in the previous "walk"
            // We do this because we could of walked in the incorrect direction and
            // the route line could have the same name and id.
            if (!WalkTransitLines(currentNode, reader_transit_level, lock, visited_map, t->date_time,
                                  t->destination, t->route_id)) {

              LOG_DEBUG("Test from " + t->origin + " to " + t->destination + " @ " + t->date_time +
                        " route id " + t->route_id + " failed.");
              failed_tests.emplace(t->origin,
                                   (t->destination + " @ " + t->date_time + " " + t->route_id));
              continue;
            }
          }
          LOG_DEBUG("Test from " + t->origin + " to " + t->destination + " @ " + t->date_time +
                    " route id " + t->route_id + " passed.");
          passed_tests.emplace(t->origin,
                               (t->destination + " @ " + t->date_time + " " + t->route_id));
        }
      }
    }
  }

  std::set<std::string> failures;
  for (auto p = failed_tests.begin(); p != failed_tests.end(); ++p) {
    bool bfound = false;
    auto tests = passed_tests.equal_range(p->first);
    for (auto it = tests.first; it != tests.second; ++it) {
      if (it->second == (p->second)) {
        bfound = true;
        break;
      }
    }

    if (bfound) {
      continue;
    } else { // only report a failure one time.  A station has multiple platforms and we could of
      // walked the transit lines of the wrong platform.
      if (failures.find(p->first + p->second) == failures.end()) {
        failure_count++;
        LOG_ERROR("Test from " + p->first + " to " + p->second + " failed.");
        failures.emplace(p->first + p->second);
      }
    }
  }

  // Send back the statistics
  results.set_value({failure_count});
}

namespace valhalla {
namespace mjolnir {

// Parse the generated test file.
std::vector<OneStopTest> ParseTestFile(const std::string& filename) {
  typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
  boost::char_separator<char> sep{","};
  std::vector<OneStopTest> onestoptests;
  std::string default_date_time = get_testing_date_time();

  // Open file
  std::string line;
  std::ifstream file(filename);
  if (file.is_open()) {
    while (getline(file, line)) {
      tokenizer tok{line, sep};
      uint32_t field_num = 0;
      OneStopTest onestoptest{};
      for (const auto& t : tok) {
        switch (field_num) {
          case 0:
            onestoptest.origin = remove_double_quotes(t);
            break;
          case 1:
            onestoptest.destination = remove_double_quotes(t);
            break;
          case 2:
            onestoptest.route_id = remove_double_quotes(t);
            break;
          case 3:
            onestoptest.date_time = remove_double_quotes(t);
            break;
        }
        field_num++;
      }
      if (onestoptest.date_time.empty()) {
        onestoptest.date_time = default_date_time;
      }

      onestoptests.emplace_back(std::move(onestoptest));
    }
    file.close();
  } else {
    std::cout << "One stop test file: " << filename << " not found" << std::endl;
  }

  return onestoptests;
}

// create parse a Valhalla responses and create transit tests.
// Example request and results from a log file:
/*28 2017/01/26 15:03:16.482590 GET
/route?json=%7B%22locations%22%3A%5B%7B%22lon%22%3A-73.98827%2C%22lat%22%3A40.74912%2C%22type%22%3A%22break%22%7D%2C%7B%22lon%22%3A-74.06289%2C%22lat%22%3A40.73301%2C%22type%22%3A%22break%22%7D%5D%2C%22costing%22%3A%22multimodal%22%2C%22costin
g_options%22%3A%7B%22transit%22%3A%7B%22use_bus%22%3A0.3%2C%22use_rail%22%3A0.6%2C%22use_transfers%22%3A0.3%7D%7D%2C%22date_time%22%3A%7B%22type%22%3A1%2C%22value%22%3A%222017-01-24T08%3A00%22%7D%7D%0A
HTTP/1.1 2017/01/26 15:03:16.482691 [INFO] Got Loki Request 28 2017/01/26 15:03:16.483346
[ANALYTICS] location_count::2 2017/01/26 15:03:16.483358 [ANALYTICS] costing_type::multimodal
2017/01/26 15:03:16.483423 [ANALYTICS] location_distance::6.543840km
2017/01/26 15:03:16.562768 [INFO] Got Thor Request 28
2017/01/26 15:03:16.565691 [ANALYTICS] travel_mode::1
2017/01/26 15:03:16.565705 [ANALYTICS] #_passes::1
2017/01/26 15:03:16.605057 [ANALYTICS] admin_state_iso::NY
2017/01/26 15:03:16.605082 [ANALYTICS] admin_country_iso::US
2017/01/26 15:03:16.605087 [ANALYTICS] admin_state_iso::NJ
2017/01/26 15:03:16.605091 [ANALYTICS] admin_country_iso::US
2017/01/26 15:03:16.605515 [INFO] Got Odin Request 28
2017/01/26 15:03:16.608499 [INFO] trip_path_->node_size()=13
2017/01/26 15:03:16.609881 [INFO] maneuver_count::7
2017/01/26 15:03:16.610329 [INFO] Got Tyr Request 28
2017/01/26 15:03:16.613096 [ANALYTICS] language::en-US
2017/01/26 15:03:16.613184 [ANALYTICS] trip_time::1532s
2017/01/26 15:03:16.613192 [ANALYTICS] trip_length::8.365996km
2017/01/26 15:03:16.613226 [ANALYTICS] transit_route_stopid::r-dr5re-path
2017/01/26 15:03:16.613242 [ANALYTICS] transit_stopid::s-dr5ru65ku6-33rdstreet<781740
2017/01/26 15:03:16.613251 [ANALYTICS] transit_stopid::s-dr5ru300e7-23rdstreet<781739
2017/01/26 15:03:16.613258 [ANALYTICS] transit_stopid::s-dr5ru0j0k8-14thstreet<781737
2017/01/26 15:03:16.613265 [ANALYTICS] transit_stopid::s-dr5rsp7qpu-9thstreet<781735
2017/01/26 15:03:16.613271 [ANALYTICS] transit_stopid::s-dr5rezjwwk-christopherstreet<781733
2017/01/26 15:03:16.613278 [ANALYTICS] transit_stopid::s-dr5req45yt-newport<781729
2017/01/26 15:03:16.613284 [ANALYTICS] transit_stopid::s-dr5rehgcjy-grovestreet<781727
2017/01/26 15:03:16.613291 [ANALYTICS] transit_stopid::s-dr5rdxjncx-journalsquare<781724
28 2017/01/26 15:03:16.613753 200 6116

Test generated:
from_onestop_id,to_onestop_id,via_route_onestop_id
s-dr5ru65ku6-33rdstreet<781740,s-dr5rdxjncx-journalsquare<781724,r-dr5re-path
*/
void ParseLogFile(const std::string& filename) {
  typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
  boost::char_separator<char> sep{" "};

  std::string type, origin, dest, tranisit_route;
  bool route = false;
  origin = "";
  dest = "";
  tranisit_route = "";
  // Open file
  std::string line;
  std::ifstream file(filename);
  if (file.is_open()) {
    while (getline(file, line)) {
      tokenizer tok{line, sep};
      uint32_t field_num = 0;
      OneStopTest onestoptest{};
      for (const auto& t : tok) {
        switch (field_num) {
          case 2:
            type = remove_double_quotes(t);
            break;
          case 3:
            if (type == "[ANALYTICS]") {
              auto o_s = remove_double_quotes(t);
              std::string transit, value;
              std::size_t found = o_s.find_last_of("::");
              if (found != std::string::npos) {
                transit = o_s.substr(0, found - 1);
                value = o_s.substr(found + 1);
              }

              if (transit == "transit_route_stopid") {
                if (route) {
                  std::cout << origin << "," << dest << "," << tranisit_route << std::endl;
                  origin = "";
                  dest = "";
                  tranisit_route = "";
                }
                tranisit_route = value;
                route = true;
              } else if (transit == "transit_stopid") {
                if (route && origin.empty()) {
                  origin = value;
                } else {
                  dest = value;
                }
              }
            } else if (type == "[INFO]") {
              origin = "";
              dest = "";
              tranisit_route = "";
              type = "";
              route = false;
            } else if (route) {
              std::cout << origin << "," << dest << "," << tranisit_route << std::endl;
              origin = "";
              dest = "";
              tranisit_route = "";
              type = "";
              route = false;
            }
            break;
        }
        field_num++;
      }
    }
    file.close();
  } else {
    std::cout << "One stop test file: " << filename << " not found" << std::endl;
  }
}

// Enhance the local level of the graph
bool ValidateTransit::Validate(const boost::property_tree::ptree& pt,
                               const std::unordered_set<GraphId>& tiles,
                               const std::vector<OneStopTest>& onestoptests) {

  unsigned int thread_count =
      std::max(static_cast<unsigned int>(1), std::thread::hardware_concurrency());
  auto t1 = std::chrono::high_resolution_clock::now();

  std::unordered_set<GraphId> all_tiles;
  LOG_INFO("Validating transit network.");
  boost::property_tree::ptree local_pt = pt;

  // if we called validate from valhalla_validate_transit then tiles is empty....go get the tiles.
  if (!tiles.size()) {
    // Bail if nothing
    auto hierarchy_properties = pt.get_child("mjolnir");
    auto transit_dir = hierarchy_properties.get_optional<std::string>("transit_dir");
    if (!transit_dir || !filesystem::exists(*transit_dir) ||
        !filesystem::is_directory(*transit_dir)) {
      LOG_INFO("Transit directory not found. Transit will not be added.");
      return false;
    }
    // Also bail if nothing inside
    transit_dir->push_back(filesystem::path::preferred_separator);
    GraphReader reader(hierarchy_properties);
    auto transit_level = TileHierarchy::GetTransitLevel().level;
    if (filesystem::is_directory(*transit_dir + std::to_string(transit_level) +
                                 filesystem::path::preferred_separator)) {
      filesystem::recursive_directory_iterator transit_file_itr(
          *transit_dir + std::to_string(transit_level) + filesystem::path::preferred_separator),
          end_file_itr;
      for (; transit_file_itr != end_file_itr; ++transit_file_itr) {
        if (filesystem::is_regular_file(transit_file_itr->path()) &&
            transit_file_itr->path().extension() == ".gph") {
          auto graph_id = GraphTile::GetTileId(transit_file_itr->path().string());
          GraphId transit_tile_id = GraphId(graph_id.tileid(), graph_id.level() - 1, graph_id.id());
          all_tiles.emplace(transit_tile_id);
        }
      }
    }
    local_pt.get_child("mjolnir").erase("tile_dir");
    local_pt.add("mjolnir.tile_dir", std::string(*transit_dir));

  } else {
    all_tiles = tiles; // we called validate from valhalla_build_transit and tiles is not empty.
  }

  if (!all_tiles.size()) {
    LOG_INFO("No transit tiles found. Transit will not be validated.");
    return false;
  }

  if (!onestoptests.size()) {
    LOG_INFO("No transit tests found. Transit will not be validated.");
    return false;
  }

  // A place to hold worker threads and their results
  std::vector<std::shared_ptr<std::thread>> threads(thread_count);

  // An atomic object we can use to do the synchronization
  std::mutex lock;

  // A place to hold the results of those threads (exceptions, stats)
  std::list<std::promise<validate_stats>> results;

  // Start the threads, divvy up the work
  LOG_INFO("Validating " + std::to_string(all_tiles.size()) + " transit tiles...");
  size_t floor = all_tiles.size() / threads.size();
  size_t at_ceiling = all_tiles.size() - (threads.size() * floor);
  std::unordered_set<GraphId>::const_iterator tile_start, tile_end = all_tiles.begin();

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
    threads[i].reset(new std::thread(validate, std::cref(local_pt.get_child("mjolnir")),
                                     std::ref(lock), tile_start, tile_end, std::cref(onestoptests),
                                     std::ref(results.back())));
  }

  // Wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  // Check all of the outcomes, to see about maximum density (km/km2)
  validate_stats stats{};
  uint32_t failure_count = 0;
  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      auto thread_stats = result.get_future().get();
      stats(thread_stats);
      failure_count += stats.failure_count;
    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }

  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t secs = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
  LOG_INFO("Finished validating transit network - took " + std::to_string(secs) + " secs");

  if (failure_count) {
    LOG_ERROR("There were " + std::to_string(failure_count) + " failures!");
    return false;
  }

  LOG_INFO("Success!  Validation tests passed.");
  return true;
}

} // namespace mjolnir
} // namespace valhalla
