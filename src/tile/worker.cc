#include "tile/worker.h"
#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "meili/candidate_search.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "valhalla/exceptions.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/multi_linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <vtzero/builder.hpp>

#include <algorithm>
#include <climits>
#include <cmath>
#include <unordered_set>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace tile {

tile_worker_t::tile_worker_t(const boost::property_tree::ptree& config,
                             const std::shared_ptr<baldr::GraphReader>& graph_reader)
    : config_(config), reader_(graph_reader) {
}

midgard::AABB2<midgard::PointLL> tile_worker_t::tile_to_bbox(uint32_t z, uint32_t x, uint32_t y) {
  const double n = std::pow(2.0, z);

  double min_lon = x / n * 360.0 - 180.0;
  double max_lon = (x + 1) / n * 360.0 - 180.0;

  double min_lat_rad = std::atan(std::sinh(kPiD * (1 - 2 * (y + 1) / n)));
  double max_lat_rad = std::atan(std::sinh(kPiD * (1 - 2 * y / n)));

  double min_lat = min_lat_rad * 180.0 / kPiD;
  double max_lat = max_lat_rad * 180.0 / kPiD;

  return AABB2<PointLL>(PointLL(min_lon, min_lat), PointLL(max_lon, max_lat));
}

std::string tile_worker_t::render_tile(uint32_t z, uint32_t x, uint32_t y) {
  // Validate tile coordinates
  uint32_t max_coord = (1u << z);
  if (x >= max_coord || y >= max_coord || z > 30) {
    throw valhalla_exception_t{400, "Invalid tile coordinates"};
  }

  // Calculate tile bounding box
  auto bounds = tile_to_bbox(z, x, y);

  // Create vector tile
  vtzero::tile_builder tile;

  if (z < 12) { // TODO: move to config
    return tile.serialize();
  }

  // Query edges within the tile bounding box
  const auto& hierarchy_levels = TileHierarchy::levels();
  const auto& tiles = hierarchy_levels.back().tiles;
  LOG_INFO("  Using Valhalla hierarchy level: " + std::to_string(hierarchy_levels.back().level) +
           " (tile size: " + std::to_string(tiles.TileSize()) + " degrees)");
  float cell_size = tiles.TileSize() / 10.0f;
  meili::CandidateGridQuery candidate_query(*reader_, cell_size, cell_size);

  auto edge_ids = candidate_query.RangeQuery(bounds);

  // Pre-compute Web Mercator projection functions and tile bounds (once for all edges)
  const double earth_radius = 6378137.0; // EPSG:3857
  const int32_t TILE_EXTENT = 4096;
  const int32_t TILE_BUFFER = 128; // Match OSRM's buffer size

  auto lon_to_merc_x = [earth_radius](double lon) { return earth_radius * lon * kPiD / 180.0; };

  auto lat_to_merc_y = [earth_radius](double lat) {
    return earth_radius * std::log(std::tan(kPiD / 4.0 + lat * kPiD / 360.0));
  };

  const double tile_merc_minx = lon_to_merc_x(bounds.minx());
  const double tile_merc_maxx = lon_to_merc_x(bounds.maxx());
  const double tile_merc_miny = lat_to_merc_y(bounds.miny());
  const double tile_merc_maxy = lat_to_merc_y(bounds.maxy());
  const double tile_merc_width = tile_merc_maxx - tile_merc_minx;
  const double tile_merc_height = tile_merc_maxy - tile_merc_miny;

  using point_t = boost::geometry::model::d2::point_xy<double>;
  using linestring_t = boost::geometry::model::linestring<point_t>;
  using multi_linestring_t = boost::geometry::model::multi_linestring<linestring_t>;
  using box_t = boost::geometry::model::box<point_t>;

  // Create clip box with buffer (like OSRM does) to handle edges that cross boundaries
  const box_t clip_box(point_t(-TILE_BUFFER, -TILE_BUFFER),
                       point_t(TILE_EXTENT + TILE_BUFFER, TILE_EXTENT + TILE_BUFFER));

  // Create roads layer for edges
  vtzero::layer_builder layer_roads{tile, "roads"};

  // Pre-add keys for properties with forward/reverse suffixes
  auto key_level = layer_roads.add_key_without_dup_check("level");
  auto key_edge_id_fwd = layer_roads.add_key_without_dup_check("edge_id:forward");
  auto key_edge_id_rev = layer_roads.add_key_without_dup_check("edge_id:reverse");
  auto key_road_class_fwd = layer_roads.add_key_without_dup_check("road_class:forward");
  auto key_road_class_rev = layer_roads.add_key_without_dup_check("road_class:reverse");
  auto key_use_fwd = layer_roads.add_key_without_dup_check("use:forward");
  auto key_use_rev = layer_roads.add_key_without_dup_check("use:reverse");
  auto key_tunnel_fwd = layer_roads.add_key_without_dup_check("tunnel:forward");
  auto key_tunnel_rev = layer_roads.add_key_without_dup_check("tunnel:reverse");
  auto key_bridge_fwd = layer_roads.add_key_without_dup_check("bridge:forward");
  auto key_bridge_rev = layer_roads.add_key_without_dup_check("bridge:reverse");
  auto key_roundabout_fwd = layer_roads.add_key_without_dup_check("roundabout:forward");
  auto key_roundabout_rev = layer_roads.add_key_without_dup_check("roundabout:reverse");

  // Collect unique nodes for the nodes layer
  std::unordered_set<GraphId> unique_nodes;

  for (const auto& edge_id : edge_ids) {
    baldr::graph_tile_ptr edge_tile;
    const auto* edge = reader_->directededge(edge_id, edge_tile);
    if (!edge || !edge_tile) {
      continue;
    }

    // Get edge geometry
    auto edge_info = edge_tile->edgeinfo(edge);
    auto shape = edge_info.shape();

    // Reverse if needed
    if (!edge->forward()) {
      std::reverse(shape.begin(), shape.end());
    }

    if (shape.size() < 2) {
      continue;
    }

    // Convert lat/lon shape to tile coordinates (unclipped)
    linestring_t unclipped_line;

    for (const auto& ll : shape) {
      // Convert point to Web Mercator
      double merc_x = lon_to_merc_x(ll.lng());
      double merc_y = lat_to_merc_y(ll.lat());

      // Normalize to 0-1 within tile's mercator bounds
      double norm_x = (merc_x - tile_merc_minx) / tile_merc_width;
      double norm_y = (tile_merc_maxy - merc_y) / tile_merc_height; // Y is inverted in tiles

      // Scale to tile extent and convert to tile coordinates
      double tile_x = norm_x * TILE_EXTENT;
      double tile_y = norm_y * TILE_EXTENT;

      boost::geometry::append(unclipped_line, point_t(tile_x, tile_y));
    }

    // Clip the line to the tile boundaries using Boost.Geometry
    // This properly handles lines that cross tile boundaries
    multi_linestring_t clipped_lines;
    boost::geometry::intersection(clip_box, unclipped_line, clipped_lines);

    // Skip if no clipped segments (edge is completely outside tile)
    if (clipped_lines.empty()) {
      continue;
    }

    // Process each clipped line segment (there may be multiple if line crosses tile multiple
    // times)
    for (const auto& clipped_line : clipped_lines) {
      // Skip degenerate lines (less than 2 points)
      if (clipped_line.size() < 2) {
        continue;
      }

      // Convert to vtzero points, removing consecutive duplicates
      std::vector<vtzero::point> tile_coords;
      tile_coords.reserve(clipped_line.size());

      int32_t last_x = INT32_MIN;
      int32_t last_y = INT32_MIN;

      for (const auto& pt : clipped_line) {
        int32_t x = static_cast<int32_t>(std::round(pt.x()));
        int32_t y = static_cast<int32_t>(std::round(pt.y()));

        // Skip consecutive duplicate points (can happen after rounding)
        if (x == last_x && y == last_y) {
          continue;
        }

        tile_coords.emplace_back(x, y);
        last_x = x;
        last_y = y;
      }

      // Must have at least 2 unique points to create a valid linestring
      if (tile_coords.size() < 2) {
        continue;
      }

      // Create linestring feature for this clipped segment
      // Use GraphId as feature ID for global uniqueness and debugging
      vtzero::linestring_feature_builder feature{layer_roads};
      feature.set_id(static_cast<uint64_t>(edge_id));
      feature.add_linestring_from_container(tile_coords);

      // Add properties
      // Get hierarchy level (shared for both directions)
      uint32_t level = edge_id.level();
      feature.add_property(key_level, vtzero::encoded_property_value(level));

      // Determine direction suffix based on edge->forward()
      std::string dir_suffix = edge->forward() ? "forward" : "reverse";

      // Add this edge's properties with appropriate suffix
      std::string edge_id_str = std::to_string(static_cast<uint64_t>(edge_id));
      auto key_edge_id = edge->forward() ? key_edge_id_fwd : key_edge_id_rev;
      auto key_road_class = edge->forward() ? key_road_class_fwd : key_road_class_rev;
      auto key_use = edge->forward() ? key_use_fwd : key_use_rev;
      auto key_tunnel = edge->forward() ? key_tunnel_fwd : key_tunnel_rev;
      auto key_bridge = edge->forward() ? key_bridge_fwd : key_bridge_rev;
      auto key_roundabout = edge->forward() ? key_roundabout_fwd : key_roundabout_rev;

      feature.add_property(key_edge_id, vtzero::encoded_property_value(edge_id_str));
      feature.add_property(key_road_class, vtzero::encoded_property_value(
                                               static_cast<uint32_t>(edge->classification())));
      feature.add_property(key_use,
                           vtzero::encoded_property_value(static_cast<uint32_t>(edge->use())));
      feature.add_property(key_tunnel, vtzero::encoded_property_value(edge->tunnel()));
      feature.add_property(key_bridge, vtzero::encoded_property_value(edge->bridge()));
      feature.add_property(key_roundabout, vtzero::encoded_property_value(edge->roundabout()));

      // Check for opposing edge and add its metadata too (bidirectional roads)
      baldr::graph_tile_ptr opp_tile = edge_tile;
      const DirectedEdge* opp_edge = nullptr;
      GraphId opp_edge_id = reader_->GetOpposingEdgeId(edge_id, opp_edge, opp_tile);

      if (opp_edge_id.Is_Valid() && opp_edge) {
        // Add opposing edge's properties with opposite suffix
        auto key_opp_edge_id = edge->forward() ? key_edge_id_rev : key_edge_id_fwd;
        auto key_opp_road_class = edge->forward() ? key_road_class_rev : key_road_class_fwd;
        auto key_opp_use = edge->forward() ? key_use_rev : key_use_fwd;
        auto key_opp_tunnel = edge->forward() ? key_tunnel_rev : key_tunnel_fwd;
        auto key_opp_bridge = edge->forward() ? key_bridge_rev : key_bridge_fwd;
        auto key_opp_roundabout = edge->forward() ? key_roundabout_rev : key_roundabout_fwd;

        std::string opp_edge_id_str = std::to_string(static_cast<uint64_t>(opp_edge_id));
        feature.add_property(key_opp_edge_id, vtzero::encoded_property_value(opp_edge_id_str));
        feature.add_property(key_opp_road_class, vtzero::encoded_property_value(static_cast<uint32_t>(
                                                     opp_edge->classification())));
        feature.add_property(key_opp_use,
                             vtzero::encoded_property_value(static_cast<uint32_t>(opp_edge->use())));
        feature.add_property(key_opp_tunnel, vtzero::encoded_property_value(opp_edge->tunnel()));
        feature.add_property(key_opp_bridge, vtzero::encoded_property_value(opp_edge->bridge()));
        feature.add_property(key_opp_roundabout,
                             vtzero::encoded_property_value(opp_edge->roundabout()));
      }

      feature.commit();

      // Collect the end node of this edge for the nodes layer
      unique_nodes.insert(edge->endnode());
    }
  }

  // Create nodes layer
  vtzero::layer_builder layer_nodes{tile, "nodes"};

  // Pre-add keys for node properties
  auto key_node_id = layer_nodes.add_key_without_dup_check("node_id");
  auto key_node_type = layer_nodes.add_key_without_dup_check("type");
  auto key_traffic_signal = layer_nodes.add_key_without_dup_check("traffic_signal");
  auto key_access = layer_nodes.add_key_without_dup_check("access");

  // Render unique nodes as points
  for (const auto& node_id : unique_nodes) {
    baldr::graph_tile_ptr node_tile;
    const auto* node = reader_->nodeinfo(node_id, node_tile);
    if (!node || !node_tile) {
      continue;
    }

    // Get node position
    auto node_ll = node->latlng(node_tile->header()->base_ll());

    // Convert to Web Mercator and then to tile coordinates
    double merc_x = lon_to_merc_x(node_ll.lng());
    double merc_y = lat_to_merc_y(node_ll.lat());

    double norm_x = (merc_x - tile_merc_minx) / tile_merc_width;
    double norm_y = (tile_merc_maxy - merc_y) / tile_merc_height;

    int32_t tile_x = static_cast<int32_t>(std::round(norm_x * TILE_EXTENT));
    int32_t tile_y = static_cast<int32_t>(std::round(norm_y * TILE_EXTENT));

    // Only render nodes that are within the tile (including buffer)
    if (tile_x < -TILE_BUFFER || tile_x > TILE_EXTENT + TILE_BUFFER || tile_y < -TILE_BUFFER ||
        tile_y > TILE_EXTENT + TILE_BUFFER) {
      continue;
    }

    // Create point feature
    vtzero::point_feature_builder node_feature{layer_nodes};
    node_feature.set_id(static_cast<uint64_t>(node_id));
    node_feature.add_point(vtzero::point{tile_x, tile_y});

    // Add node properties
    std::string node_id_str = std::to_string(static_cast<uint64_t>(node_id));
    node_feature.add_property(key_node_id, vtzero::encoded_property_value(node_id_str));
    node_feature.add_property(key_node_type,
                              vtzero::encoded_property_value(static_cast<uint32_t>(node->type())));
    node_feature.add_property(key_traffic_signal,
                              vtzero::encoded_property_value(node->traffic_signal()));
    node_feature.add_property(key_access,
                              vtzero::encoded_property_value(static_cast<uint32_t>(node->access())));

    node_feature.commit();
  }

  LOG_INFO("  Rendered " + std::to_string(unique_nodes.size()) + " unique nodes");

  return tile.serialize();
}

} // namespace tile
} // namespace valhalla
