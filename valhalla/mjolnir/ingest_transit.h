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
} // namespace mjolnir
} // namespace valhalla