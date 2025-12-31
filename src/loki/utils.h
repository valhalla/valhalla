#pragma once

#include <filesystem>
#include <string>

namespace valhalla::loki::detail {

/**
 * Builds the path on the filesystem for a x/y/z MVT cache file, analogous to
 * GraphTiles with padded 1000's directory separators.
 *
 * @param z the z level
 * @param x the tile x
 * @param y the tile y
 * @param root the cache root directory
 * @returns the passed tile's cache file path in the form of <root>/<z>/numeric/tile/id.mvt
 */
std::filesystem::path
mvt_local_path(const uint32_t z, const uint32_t x, const uint32_t y, const std::string& root);

} // namespace valhalla::loki::detail
