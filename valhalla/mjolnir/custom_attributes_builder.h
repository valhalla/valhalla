#pragma once

#include <boost/property_tree/ptree.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace valhalla {
namespace mjolnir {

// how a shortcut's value is derived from its base edges values
enum class ShortcutStrategy { kNone, kMin, kMax, kMean };

/**
 * Allows custom attributes be derived from osm way tags via a user-supplied Lua script. The script
 * must define custom_attributes(tags) returning a table of attribute name -> number for both
 * directions of the way, or a forward and a backward table, relative to the OSM way.
 *
 * @param config             Valhalla config
 * @param input_files        OSM PBF files the graph was built from
 * @param tag_keys           list of tag keys to pre-filter ways by; if none is present, the way gets
 * skipped
 * @param lua_script         Lua source code
 * @param shortcut_strategy  How shortcut values are derived from their base edges
 *
 * @return number of values written
 */
size_t add_custom_attributes(const boost::property_tree::ptree& config,
                             const std::vector<std::string>& input_files,
                             const std::vector<std::string>& tag_keys,
                             const std::string& lua_script,
                             ShortcutStrategy shortcut_strategy = ShortcutStrategy::kMean);

} // namespace mjolnir
} // namespace valhalla
