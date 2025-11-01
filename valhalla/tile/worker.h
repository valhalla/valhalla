#ifndef __VALHALLA_TILE_WORKER_H__
#define __VALHALLA_TILE_WORKER_H__

#include <valhalla/baldr/graphreader.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>

#include <boost/property_tree/ptree.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace valhalla {
namespace tile {

/**
 * Tile worker for rendering vector tiles from Valhalla graph data.
 * Generates Mapbox Vector Tiles (MVT) compatible output.
 */
class tile_worker_t {
public:
  /**
   * Constructor
   * @param config  Configuration property tree
   * @param graph_reader  Shared pointer to graph reader
   */
  tile_worker_t(const boost::property_tree::ptree& config,
                const std::shared_ptr<baldr::GraphReader>& graph_reader);

  /**
   * Render a vector tile for the specified tile coordinates
   * @param z  Zoom level
   * @param x  Tile X coordinate
   * @param y  Tile Y coordinate
   * @return  Serialized MVT tile data as a string
   */
  std::string render_tile(uint32_t z, uint32_t x, uint32_t y);

private:
  boost::property_tree::ptree config_;
  std::shared_ptr<baldr::GraphReader> reader_;

  /**
   * Convert tile coordinates (z/x/y) to WGS84 bounding box
   */
  static midgard::AABB2<midgard::PointLL> tile_to_bbox(uint32_t z, uint32_t x, uint32_t y);
};

} // namespace tile
} // namespace valhalla

#endif // __VALHALLA_TILE_WORKER_H__
