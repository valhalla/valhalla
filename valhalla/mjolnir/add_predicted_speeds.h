#pragma once

#include "baldr/graphid.h"
#include "baldr/predictedspeeds.h"

#include <boost/property_tree/ptree.hpp>

#include <filesystem>
#include <optional>
#include <string>

namespace valhalla {
namespace mjolnir {

/**
 * Processes traffic tiles and updates graph data with traffic information.
 *
 * This method is the primary entry point for updating graph tiles with traffic data.
 * It handles parsing traffic files, updating tile speeds, and generating processing statistics.
 *
 * @param config Configuration property tree containing processing parameters
 *               Must include 'mjolnir.tile_dir' and 'mjolnir.concurrency' settings
 *
 * @throws std::runtime_error if tile processing encounters critical errors
 * @throws std::invalid_argument if configuration is invalid
 */
void ProcessTrafficTiles(const std::string& tile_dir,
                         const std::filesystem::path& traffic_tile_dir,
                         const bool summary,
                         const boost::property_tree::ptree& config);

} // namespace mjolnir
} // namespace valhalla
