#pragma once

#include <boost/property_tree/ptree.hpp>
#include <unordered_set>

#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace mjolnir {

/**
 * @brief Grabs protobufs written in ingest_transit and converts them into transit level tiles
 *        Non-transit graph tiles are also required to find locations where to connect the
 *        transit subgraph (nodes where we can add transit connect edges)
 * @param pt Property tree containing the hierarchy configuration
 *             and other configuration needed to build transit.
 * @return std::unordered_set<baldr::GraphId> all tiles created
 */
std::unordered_set<baldr::GraphId> convert_transit(const boost::property_tree::ptree& pt);
} // namespace mjolnir
} // namespace valhalla