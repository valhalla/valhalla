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
 * Writes per-way values from a user Lua script into the existing custom attributes tar at
 * mjolnir.custom_attributes_extract. Ways without any of tag_keys are skipped before Lua runs.
 * The script must define custom_attributes(tags) returning a table of attribute name -> number
 * for both directions of the way, or a forward and a backward table, relative to the OSM way.
 * Afterwards every shortcut gets its values from the edges it replaces, unless the strategy is
 * kNone; kMean weights them by edge length.
 *
 * @param config             Valhalla config, mjolnir section is used to read the graph and the tar
 * @param input_files        OSM PBF files the graph was built from
 * @param tag_keys           OSM tag keys a way needs at least one of to be passed to Lua
 * @param lua_script         Lua source code
 * @param shortcut_strategy  How shortcut values are derived from their edges
 * @return number of (edge, attribute) values written from Lua
 */
size_t add_custom_attributes(const boost::property_tree::ptree& config,
                             const std::vector<std::string>& input_files,
                             const std::vector<std::string>& tag_keys,
                             const std::string& lua_script,
                             ShortcutStrategy shortcut_strategy = ShortcutStrategy::kMean);

} // namespace mjolnir
} // namespace valhalla
