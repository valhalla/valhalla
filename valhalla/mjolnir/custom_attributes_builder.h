#pragma once

#include <boost/property_tree/ptree.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace valhalla {
namespace mjolnir {

// How shortcut edges get their values from the base edges they supersede. kMean is weighted by
// base edge length.
enum class ShortcutAggregation { kNone, kMean, kMin, kMax };

/**
 * Writes per-way values from a user Lua script into the existing custom attributes tar at
 * mjolnir.custom_attributes_extract. Ways without any of tag_keys are skipped before Lua runs.
 * The script must define custom_attributes(tags) returning a table of attribute name -> number.
 *
 * @param config       Valhalla config, mjolnir section is used to read the graph and the tar
 * @param input_files  OSM PBF files the graph was built from
 * @param tag_keys     OSM tag keys a way needs at least one of to be passed to Lua
 * @param lua_script   Lua source code
 * @param shortcuts    how to fill shortcut edges from their base edges after the ways are written
 * @return number of (edge, attribute) values written
 */
size_t add_custom_attributes(const boost::property_tree::ptree& config,
                             const std::vector<std::string>& input_files,
                             const std::vector<std::string>& tag_keys,
                             const std::string& lua_script,
                             ShortcutAggregation shortcuts = ShortcutAggregation::kMean);

} // namespace mjolnir
} // namespace valhalla
