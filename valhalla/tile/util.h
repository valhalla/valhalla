#ifndef __VALHALLA_TILE_UTIL_H__
#define __VALHALLA_TILE_UTIL_H__

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>

#include <cstdint>

namespace valhalla {
namespace tile {

/**
 * Earth radius in meters for EPSG:3857 Web Mercator projection.
 * This is the WGS84 ellipsoid semi-major axis.
 */
constexpr double kEarthRadiusMeters = 6378137.0;

/**
 * Convert tile coordinates (z/x/y) to WGS84 bounding box.
 * Uses the standard Web Mercator / Slippy Map tile addressing scheme.
 *
 * @param z  Zoom level (0-30)
 * @param x  Tile X coordinate (0 to 2^z - 1)
 * @param y  Tile Y coordinate (0 to 2^z - 1)
 * @return   Bounding box in WGS84 coordinates (longitude/latitude)
 */
midgard::AABB2<midgard::PointLL> tile_to_bbox(uint32_t z, uint32_t x, uint32_t y);

/**
 * Convert longitude to Web Mercator X coordinate (in meters).
 *
 * @param lon  Longitude in degrees
 * @return     X coordinate in Web Mercator projection (meters)
 */
double lon_to_merc_x(double lon);

/**
 * Convert latitude to Web Mercator Y coordinate (in meters).
 * Note: Web Mercator is only valid for latitudes approximately ±85.051129°
 *
 * @param lat  Latitude in degrees
 * @return     Y coordinate in Web Mercator projection (meters)
 */
double lat_to_merc_y(double lat);

} // namespace tile
} // namespace valhalla

#endif // __VALHALLA_TILE_UTIL_H__

