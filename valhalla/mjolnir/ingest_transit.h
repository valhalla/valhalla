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
#include <valhalla/filesystem.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/proto/transit.pb.h>

namespace valhalla {
namespace mjolnir {

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
std::list<baldr::GraphId> ingest_transit(const boost::property_tree::ptree& pt);

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
void stitch_transit(const boost::property_tree::ptree& pt, std::list<baldr::GraphId>& dangling_tiles);

// Shape
struct Shape {
  uint32_t begins;
  uint32_t ends;
  std::vector<midgard::PointLL> shape;
};

struct Departure {
  baldr::GraphId orig_pbf_graphid; // GraphId in pbf tiles
  baldr::GraphId dest_pbf_graphid; // GraphId in pbf tiles
  uint32_t trip;
  uint32_t route;
  uint32_t blockid;
  uint32_t shapeid;
  uint32_t headsign_offset;
  uint32_t dep_time;
  uint32_t schedule_index;
  uint32_t frequency_end_time;
  uint16_t elapsed_time;
  uint16_t frequency;
  float orig_dist_traveled;
  float dest_dist_traveled;
  bool wheelchair_accessible;
  bool bicycle_accessible;
};

// Unique route and stop
struct TransitLine {
  uint32_t lineid;
  uint32_t routeid;
  baldr::GraphId dest_pbf_graphid; // GraphId (from pbf) of the destination stop
  uint32_t shapeid;
  float orig_dist_traveled;
  float dest_dist_traveled;
};

struct StopEdges {
  baldr::GraphId origin_pbf_graphid;        // GraphId (from pbf) of the origin stop
  std::vector<baldr::GraphId> intrastation; // List of intra-station connections
  std::vector<TransitLine> lines;           // Set of unique route/stop pairs
};

Transit read_pbf(const std::string& file_name, std::mutex& lock);
Transit read_pbf(const std::string& file_name);

// Get PBF transit data given a GraphId / tile
Transit read_pbf(const baldr::GraphId& id, const std::string& transit_dir, std::string& file_name);
void write_pbf(const Transit& tile, const filesystem::path& transit_tile);
// Converts a stop's pbf graph Id to a Valhalla graph Id by adding the
// tile's node count. Returns an Invalid GraphId if the tile is not found
// in the list of Valhalla tiles
baldr::GraphId GetGraphId(const baldr::GraphId& nodeid,
                          const std::unordered_set<baldr::GraphId>& all_tiles);

} // namespace mjolnir
} // namespace valhalla