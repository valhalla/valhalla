#pragma once

#include <algorithm>
#include <list>
#include <queue>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/graphid.h>
#include <valhalla/proto/transit.pb.h>

// how to include this third party item
// #include </just_gtfs/just_gtfs.h>

namespace valhalla {
namespace mjolnir {

// place operator in struct
struct tileTransitInfo {
  GraphId graphId;
  std::set<gtfs::Id> tile_stops;
  std::set<gtfs::Id> tile_trips;
  std::set<gtfs::Id> tile_routes;
  std::set<gtfs::Id> tile_services;
  std::set<gtfs::Id> tile_agencies;
  std::set<gtfs::Id> tile_shapes;

  bool operator<(const tileTransitInfo& t1) {
    // sort tileTransitInfo by size
    return tile_stops.size() < t1.tile_stops.size();
  }
};

/**
 * @brief Sorts GTFS transit data into tiles so that they can be processed by tile complexity
 *
 * @param pt Property tree containing the hierarchy configuration
 *             and other configuration needed to build transit.
 * @param path Path to where raw gtfs data is stored
 * @return std::priority_queue<tileTransitInfo>
 */
// unknown type name?
std::priority_queue<tileTransitInfo> select_transit_tiles(const boost::property_tree::ptree& pt,
                                                          const std::string& path);

/**
 * @brief Processes the data from 'select_transit_tiles' into GraphTiles, adding the data into the
 * valhalla graph.
 *
 * @param pt Property tree containing the hierarchy configuration
 *             and other configuration needed to build transit.
 * @param tiles queue of all tiles that have transit data (tiles that need processing)
 * @param thread_count
 * @return std::list<GraphId>
 */
std::list<GraphId> ingest_transit(const boost::property_tree::ptree& pt,
                                  std::priority_queue<tileTransitInfo>& tiles,
                                  unsigned int thread_count);

/**
 * @brief Processes transit routes that go through multiple tiles and stitches the graphs that are
 * connected.
 *
 * @param pt Property tree containing the hierarchy configuration
 *             and other configuration needed to build transit.
 * @param all_tiles set of all tiles that have transit data.
 * @param dangling_tiles tiles where routes go past its boundaries
 * @param thread_count
 */
void stitch_transit(const boost::property_tree::ptree& pt,
                    const std::unordered_set<GraphId>& all_tiles,
                    std::list<GraphId>& dangling_tiles,
                    unsigned int thread_count);
} // namespace mjolnir
} // namespace valhalla