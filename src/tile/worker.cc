#include "tile/worker.h"
#include "baldr/directededge.h"
#include "baldr/graphreader.h"
#include "baldr/nodeinfo.h"
#include "baldr/tilehierarchy.h"
#include "meili/candidate_search.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "tile/util.h"
#include "valhalla/exceptions.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/multi_linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <vtzero/builder.hpp>

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <unordered_set>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace tile {

namespace {

/**
 * Helper class to build the edges layer with pre-registered keys
 */
class EdgesLayerBuilder {
public:
  EdgesLayerBuilder(vtzero::tile_builder& tile) : layer_(tile, "edges") {
    // Pre-add keys for properties with forward/reverse suffixes
    key_level_ = layer_.add_key_without_dup_check("level");
    key_edge_id_fwd_ = layer_.add_key_without_dup_check("edge_id:forward");
    key_edge_id_rev_ = layer_.add_key_without_dup_check("edge_id:reverse");
    key_road_class_fwd_ = layer_.add_key_without_dup_check("road_class:forward");
    key_road_class_rev_ = layer_.add_key_without_dup_check("road_class:reverse");
    key_use_fwd_ = layer_.add_key_without_dup_check("use:forward");
    key_use_rev_ = layer_.add_key_without_dup_check("use:reverse");
    key_speed_fwd_ = layer_.add_key_without_dup_check("speed:forward");
    key_speed_rev_ = layer_.add_key_without_dup_check("speed:reverse");
    key_tunnel_fwd_ = layer_.add_key_without_dup_check("tunnel:forward");
    key_tunnel_rev_ = layer_.add_key_without_dup_check("tunnel:reverse");
    key_bridge_fwd_ = layer_.add_key_without_dup_check("bridge:forward");
    key_bridge_rev_ = layer_.add_key_without_dup_check("bridge:reverse");
    key_roundabout_fwd_ = layer_.add_key_without_dup_check("roundabout:forward");
    key_roundabout_rev_ = layer_.add_key_without_dup_check("roundabout:reverse");
  }

  void add_feature(const std::vector<vtzero::point>& geometry,
                   baldr::GraphId edge_id,
                   const baldr::DirectedEdge* forward_edge,
                   baldr::GraphId reverse_edge_id,
                   const baldr::DirectedEdge* reverse_edge) {
    // Must have at least one edge and valid geometry
    if (geometry.size() < 2) {
      return;
    }

    assert(forward_edge || reverse_edge);
    assert(!forward_edge || forward_edge->forward());
    assert(!reverse_edge || !reverse_edge->forward());

    // Create linestring feature for this edge
    vtzero::linestring_feature_builder feature{layer_};
    feature.set_id(static_cast<uint64_t>(edge_id));
    feature.add_linestring_from_container(geometry);

    // Add hierarchy level (shared for both directions)
    uint32_t level = edge_id.level();
    feature.add_property(key_level_, vtzero::encoded_property_value(level));

    // Add forward edge properties if present
    if (forward_edge) {
      std::string edge_id_str = std::to_string(static_cast<uint64_t>(edge_id));
      feature.add_property(key_edge_id_fwd_, vtzero::encoded_property_value(edge_id_str));
      feature.add_property(key_road_class_fwd_, vtzero::encoded_property_value(static_cast<uint32_t>(
                                                    forward_edge->classification())));
      feature.add_property(key_use_fwd_, vtzero::encoded_property_value(
                                             static_cast<uint32_t>(forward_edge->use())));
      feature.add_property(key_speed_fwd_, vtzero::encoded_property_value(forward_edge->speed()));
      feature.add_property(key_tunnel_fwd_, vtzero::encoded_property_value(forward_edge->tunnel()));
      feature.add_property(key_bridge_fwd_, vtzero::encoded_property_value(forward_edge->bridge()));
      feature.add_property(key_roundabout_fwd_,
                           vtzero::encoded_property_value(forward_edge->roundabout()));
    }

    // Add reverse edge properties if present
    if (reverse_edge && reverse_edge_id.Is_Valid()) {
      std::string opp_edge_id_str = std::to_string(static_cast<uint64_t>(reverse_edge_id));
      feature.add_property(key_edge_id_rev_, vtzero::encoded_property_value(opp_edge_id_str));
      feature.add_property(key_road_class_rev_, vtzero::encoded_property_value(static_cast<uint32_t>(
                                                    reverse_edge->classification())));
      feature.add_property(key_use_rev_, vtzero::encoded_property_value(
                                             static_cast<uint32_t>(reverse_edge->use())));
      feature.add_property(key_speed_rev_, vtzero::encoded_property_value(reverse_edge->speed()));
      feature.add_property(key_tunnel_rev_, vtzero::encoded_property_value(reverse_edge->tunnel()));
      feature.add_property(key_bridge_rev_, vtzero::encoded_property_value(reverse_edge->bridge()));
      feature.add_property(key_roundabout_rev_,
                           vtzero::encoded_property_value(reverse_edge->roundabout()));
    }

    feature.commit();
  }

private:
  vtzero::layer_builder layer_;

  // Pre-registered keys
  vtzero::index_value key_level_;
  vtzero::index_value key_edge_id_fwd_;
  vtzero::index_value key_edge_id_rev_;
  vtzero::index_value key_road_class_fwd_;
  vtzero::index_value key_road_class_rev_;
  vtzero::index_value key_use_fwd_;
  vtzero::index_value key_use_rev_;
  vtzero::index_value key_speed_fwd_;
  vtzero::index_value key_speed_rev_;
  vtzero::index_value key_tunnel_fwd_;
  vtzero::index_value key_tunnel_rev_;
  vtzero::index_value key_bridge_fwd_;
  vtzero::index_value key_bridge_rev_;
  vtzero::index_value key_roundabout_fwd_;
  vtzero::index_value key_roundabout_rev_;
};

/**
 * Helper class to build the nodes layer with pre-registered keys
 */
class NodesLayerBuilder {
public:
  NodesLayerBuilder(vtzero::tile_builder& tile) : layer_(tile, "nodes") {
    // Pre-add keys for node properties
    key_node_id_ = layer_.add_key_without_dup_check("node_id");
    key_node_type_ = layer_.add_key_without_dup_check("type");
    key_traffic_signal_ = layer_.add_key_without_dup_check("traffic_signal");
    key_access_ = layer_.add_key_without_dup_check("access");
  }

  void
  add_feature(const vtzero::point& position, baldr::GraphId node_id, const baldr::NodeInfo* node) {
    if (!node) {
      return;
    }

    // Create point feature
    vtzero::point_feature_builder node_feature{layer_};
    node_feature.set_id(static_cast<uint64_t>(node_id));
    node_feature.add_point(position);

    // Add node properties
    std::string node_id_str = std::to_string(static_cast<uint64_t>(node_id));
    node_feature.add_property(key_node_id_, vtzero::encoded_property_value(node_id_str));
    node_feature.add_property(key_node_type_,
                              vtzero::encoded_property_value(static_cast<uint32_t>(node->type())));
    node_feature.add_property(key_traffic_signal_,
                              vtzero::encoded_property_value(node->traffic_signal()));
    node_feature.add_property(key_access_,
                              vtzero::encoded_property_value(static_cast<uint32_t>(node->access())));

    node_feature.commit();
  }

private:
  vtzero::layer_builder layer_;

  // Pre-registered keys
  vtzero::index_value key_node_id_;
  vtzero::index_value key_node_type_;
  vtzero::index_value key_traffic_signal_;
  vtzero::index_value key_access_;
};

} // anonymous namespace

void tile_worker_t::ReadZoomConfig(const boost::property_tree::ptree& config) {
  min_zoom_road_class_ = kDefaultMinZoomRoadClass;

  auto tile_config = config.get_child_optional("tile");
  if (tile_config) {
    auto zoom_array = tile_config->get_child_optional("min_zoom_road_class");
    if (zoom_array) {
      size_t i = 0;
      for (const auto& item : *zoom_array) {
        if (i < kNumRoadClasses) {
          min_zoom_road_class_[i] = item.second.get_value<uint32_t>();
          ++i;
        }
      }
      // Fill remaining values with defaults if array is too short
      for (; i < kNumRoadClasses; ++i) {
        min_zoom_road_class_[i] = kDefaultMinZoomRoadClass[i];
      }
    }
  }

  // Compute overall minimum zoom level (minimum of all road class zooms)
  min_zoom_ = *std::min_element(min_zoom_road_class_.begin(), min_zoom_road_class_.end());
}

tile_worker_t::tile_worker_t(const boost::property_tree::ptree& config,
                             const std::shared_ptr<baldr::GraphReader>& graph_reader)
    : config_(config), reader_(graph_reader),
      candidate_query_(*graph_reader,
                       TileHierarchy::levels().back().tiles.TileSize() / 10.0f,
                       TileHierarchy::levels().back().tiles.TileSize() / 10.0f) {
  ReadZoomConfig(config_);
}

std::unordered_set<GraphId>
tile_worker_t::build_edges_layer(vtzero::tile_builder& tile,
                                 const midgard::AABB2<midgard::PointLL>& bounds,
                                 const std::unordered_set<baldr::GraphId>& edge_ids,
                                 uint32_t z,
                                 const TileProjection& projection) {
  using point_t = boost::geometry::model::d2::point_xy<double>;
  using linestring_t = boost::geometry::model::linestring<point_t>;
  using multi_linestring_t = boost::geometry::model::multi_linestring<linestring_t>;
  using box_t = boost::geometry::model::box<point_t>;

  // Create clip box with buffer to handle edges that cross boundaries
  const box_t clip_box(point_t(-projection.tile_buffer, -projection.tile_buffer),
                       point_t(projection.tile_extent + projection.tile_buffer,
                               projection.tile_extent + projection.tile_buffer));

  // Create edges layer builder
  EdgesLayerBuilder edges_builder(tile);

  // Collect unique nodes for the nodes layer
  std::unordered_set<GraphId> unique_nodes;

  for (const auto& edge_id : edge_ids) {
    baldr::graph_tile_ptr edge_tile;
    const auto* edge = reader_->directededge(edge_id, edge_tile);
    if (!edge || !edge_tile) {
      continue;
    }

    // Filter by road class and zoom level
    auto road_class = edge->classification();
    uint32_t road_class_idx = static_cast<uint32_t>(road_class);
    assert(road_class_idx < min_zoom_road_class_.size());
    if (z < min_zoom_road_class_[road_class_idx]) {
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
      double norm_x = (merc_x - projection.tile_merc_minx) / projection.tile_merc_width;
      double norm_y = (projection.tile_merc_maxy - merc_y) /
                      projection.tile_merc_height; // Y is inverted in tiles

      // Scale to tile extent and convert to tile coordinates
      double tile_x = norm_x * projection.tile_extent;
      double tile_y = norm_y * projection.tile_extent;

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

      // Check for opposing edge
      baldr::graph_tile_ptr opp_tile = edge_tile;
      const DirectedEdge* opp_edge = nullptr;
      GraphId opp_edge_id = reader_->GetOpposingEdgeId(edge_id, opp_edge, opp_tile);

      // Add feature using the builder
      const GraphId& forward_edge_id = edge->forward() ? edge_id : opp_edge_id;
      const auto* forward_edge = edge->forward() ? edge : opp_edge;
      const GraphId& reverse_edge_id = edge->forward() ? opp_edge_id : edge_id;
      const auto* reverse_edge = edge->forward() ? opp_edge : edge;

      edges_builder.add_feature(tile_coords, forward_edge_id, forward_edge, reverse_edge_id,
                                reverse_edge);

      // Collect the end node of this edge for the nodes layer
      unique_nodes.insert(edge->endnode());
    }
  }

  return unique_nodes;
}

void tile_worker_t::build_nodes_layer(vtzero::tile_builder& tile,
                                      const std::unordered_set<baldr::GraphId>& unique_nodes,
                                      const TileProjection& projection) {
  // Create nodes layer builder
  NodesLayerBuilder nodes_builder(tile);

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

    double norm_x = (merc_x - projection.tile_merc_minx) / projection.tile_merc_width;
    double norm_y = (projection.tile_merc_maxy - merc_y) / projection.tile_merc_height;

    int32_t tile_x = static_cast<int32_t>(std::round(norm_x * projection.tile_extent));
    int32_t tile_y = static_cast<int32_t>(std::round(norm_y * projection.tile_extent));

    // Only render nodes that are within the tile (including buffer)
    if (tile_x < -projection.tile_buffer ||
        tile_x > projection.tile_extent + projection.tile_buffer ||
        tile_y < -projection.tile_buffer ||
        tile_y > projection.tile_extent + projection.tile_buffer) {
      continue;
    }

    // Add feature using the builder
    nodes_builder.add_feature(vtzero::point{tile_x, tile_y}, node_id, node);
  }
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

  // Don't render anything below minimum zoom level
  if (z < min_zoom_) {
    return tile.serialize();
  }

  // Query edges within the tile bounding box
  auto edge_ids = candidate_query_.RangeQuery(bounds);

  // Pre-compute Web Mercator projection tile bounds (once for all edges)
  const int32_t TILE_EXTENT = 4096;
  const int32_t TILE_BUFFER = 128;

  TileProjection projection{lon_to_merc_x(bounds.minx()),
                            lon_to_merc_x(bounds.maxx()),
                            lat_to_merc_y(bounds.miny()),
                            lat_to_merc_y(bounds.maxy()),
                            lon_to_merc_x(bounds.maxx()) - lon_to_merc_x(bounds.minx()),
                            lat_to_merc_y(bounds.maxy()) - lat_to_merc_y(bounds.miny()),
                            TILE_EXTENT,
                            TILE_BUFFER};

  // Build edges layer and collect unique nodes
  auto unique_nodes = build_edges_layer(tile, bounds, edge_ids, z, projection);

  // Build nodes layer
  build_nodes_layer(tile, unique_nodes, projection);

  return tile.serialize();
}

} // namespace tile
} // namespace valhalla
