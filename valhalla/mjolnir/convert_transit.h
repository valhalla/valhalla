#pragma once

#include <boost/property_tree/ptree.hpp>
#include <unordered_set>

#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace mjolnir {

/**
 * @brief Grabs protobufs written in fetch_transit and converts them into transit level tiles
 * @param pt Property tree containing the hierarchy configuration
 *             and other configuration needed to build transit.
 * @return std::unordered_set<baldr::GraphId> all tiles created
 */
std::unordered_set<baldr::GraphId> convert_transit(const boost::property_tree::ptree& pt);
} // namespace mjolnir
} // namespace valhalla