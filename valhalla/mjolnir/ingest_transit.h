#pragma once

#include <algorithm>
#include <list>
#include <string>

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
 * @return std::list<baldr::GraphId> dangling tiles that contain stop_pairs that go inter-tiles
 */
std::list<baldr::GraphId> ingest_transit(const boost::property_tree::ptree& pt);

/**
 * @brief Processes transit routes that go through multiple tiles and stitches the graphs that are
 * connected.
 *
 * @param pt Property tree containing the hierarchy configuration
 *             and other configuration needed to build transit.
 * @param dangling_tiles tiles where routes go past its boundaries
 */
void stitch_transit(const boost::property_tree::ptree& pt, std::list<baldr::GraphId>& dangling_tiles);

/**
 * @brief Get a protobuf file and create a Valhalla Transit tile according to its data
 *
 * @param file_name the path where target protobuf is located
 * @param lock (optional) lock for threading
 * @return Transit tile that is read from the protobuf data
 */
Transit read_pbf(const std::string& file_name, std::mutex& lock);
Transit read_pbf(const std::string& file_name);

/**
 * @brief writes transit information inside the tile to a protobuf
 *
 * @param tile contains transit data
 * @param transit_tile destination where the protobuf is written
 */
void write_pbf(const Transit& tile, const filesystem::path& transit_tile);

} // namespace mjolnir
} // namespace valhalla