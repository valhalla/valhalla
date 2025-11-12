#ifndef __VALHALLA_HEIMDALL_WORKER_H__
#define __VALHALLA_HEIMDALL_WORKER_H__

#include <valhalla/baldr/graphreader.h>
#include <valhalla/meili/candidate_search.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>

#include <boost/property_tree/ptree_fwd.hpp>

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
class heimdall_worker_t {
public:
  /**
   * Constructor
   * @param config  Configuration property tree
   * @param graph_reader  Shared pointer to graph reader
   */
  heimdall_worker_t(const boost::property_tree::ptree& config,
                    const std::shared_ptr<baldr::GraphReader>& graph_reader);

  /**
   * Render a vector tile for the specified tile coordinates
   * @param z  Zoom level
   * @param x  Tile X coordinate
   * @param y  Tile Y coordinate
   * @param return_shortcuts  If false (default), filter out shortcut edges
   * @return  Serialized MVT tile data as a string
   */
  std::string render_tile(const uint32_t z,
                          const uint32_t x,
                          const uint32_t y,
                          const bool return_shortcuts = false);

private:
  // Default minimum zoom levels for each road class:
  // Motorway=7, Trunk=7, Primary=8, Secondary=10, Tertiary=11,
  // Unclassified=11, Residential=13, Service/Other=14
  static constexpr std::array kDefaultMinZoomRoadClass = {7u, 7u, 8u, 10u, 11u, 11u, 13u, 14u};
  static constexpr size_t kNumRoadClasses = kDefaultMinZoomRoadClass.size();
  using ZoomConfig = std::array<uint32_t, kNumRoadClasses>;

  /**
   * Build the edges & nodes layers for the vector tile
   * @param return_shortcuts  If false, filter out shortcut edges
   */
  void build_layers(vtzero::tile_builder& tile,
                    const midgard::AABB2<midgard::PointLL>& bounds,
                    const std::unordered_set<baldr::GraphId>& edge_ids,
                    uint32_t z,
                    bool return_shortcuts);

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
