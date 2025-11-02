#ifndef __VALHALLA_HEIMDALL_WORKER_H__
#define __VALHALLA_HEIMDALL_WORKER_H__

#include <valhalla/baldr/graphreader.h>
#include <valhalla/meili/candidate_search.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>

#include <boost/property_tree/ptree.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace vtzero {
class tile_builder;
}

namespace valhalla {
namespace heimdall {

/**
 * Heimdall worker for rendering vector tiles from Valhalla graph data.
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
  /**
   * Encapsulates tile projection parameters for Web Mercator coordinate conversion
   */
  struct TileProjection {
    double tile_merc_minx;
    double tile_merc_maxx;
    double tile_merc_miny;
    double tile_merc_maxy;
    double tile_merc_width;
    double tile_merc_height;
    int32_t tile_extent;
    int32_t tile_buffer;
  };

  // Default minimum zoom levels for each road class:
  // Motorway=8, Trunk=9, Primary=10, Secondary=11, Tertiary=12,
  // Unclassified=12, Residential=12, Service/Other=12
  static constexpr std::array kDefaultMinZoomRoadClass = {8u, 9u, 10u, 11u, 12u, 12u, 12u, 12u};
  static constexpr size_t kNumRoadClasses = kDefaultMinZoomRoadClass.size();
  using ZoomConfig = std::array<uint32_t, kNumRoadClasses>;

  void ReadZoomConfig(const boost::property_tree::ptree& config);

  /**
   * Build the edges layer for the vector tile
   * @return Set of unique nodes encountered while building edges
   */
  std::unordered_set<baldr::GraphId>
  build_edges_layer(vtzero::tile_builder& tile,
                    const midgard::AABB2<midgard::PointLL>& bounds,
                    const std::unordered_set<baldr::GraphId>& edge_ids,
                    uint32_t z,
                    const TileProjection& projection);

  /**
   * Build the nodes layer for the vector tile
   */
  void build_nodes_layer(vtzero::tile_builder& tile,
                         const std::unordered_set<baldr::GraphId>& unique_nodes,
                         const TileProjection& projection);

  boost::property_tree::ptree config_;
  std::shared_ptr<baldr::GraphReader> reader_;
  meili::CandidateGridQuery candidate_query_;

  // Minimum zoom level for each road class (indexed by RoadClass enum value)
  ZoomConfig min_zoom_road_class_;

  // Overall minimum zoom level (computed from min_zoom_road_class_)
  uint32_t min_zoom_;
};

} // namespace heimdall
} // namespace valhalla

#endif // __VALHALLA_HEIMDALL_WORKER_H__
