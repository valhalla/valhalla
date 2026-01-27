#include "baldr/attributes_controller.h"
#include "baldr/datetime.h"
#include "baldr/directededge.h"
#include "baldr/graphreader.h"
#include "baldr/nodeinfo.h"
#include "baldr/tilehierarchy.h"
#include "loki/tiles.h"
#include "loki/worker.h"
#include "meili/candidate_search.h"
#include "midgard/boost_geom_types.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "midgard/polyline2.h"
#include "valhalla/exceptions.h"

#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/equals.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/property_tree/ptree.hpp>
#include <vtzero/builder.hpp>
#include <vtzero/property_mapper.hpp>

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string_view>
#include <unordered_set>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace {

// approx tolerance in meters at equator
// reflects the maximum zoom we allow for, based on tippecanoe calculations with "detail = 12" (i.e.
// 4096 pixels/tile)
// "generalize" parameter can be used to scale
// clang-format off
constexpr double PeuckerEpsilons[] = {
  9'781, // z0
  4'891,
  2'445,
  1'223,
  611, // z4
  306,
  153,
  76,
  38,
  19, // z9
  10,
  5,
  2,
  1,
  0.6, // z14
  0.3,
  0.15
};
// clang-format on

/**
 * Earth radius in meters for EPSG:3857 Web Mercator projection.
 * This is the WGS84 ellipsoid semi-major axis.
 */
constexpr double kEarthRadiusMeters = 6378137.0;
constexpr std::string_view kEdgeLayerName = "edges";
constexpr std::string_view kNodeLayerName = "nodes";

double lon_to_merc_x(const double lon) {
  return kEarthRadiusMeters * lon * kPiD / 180.0;
}

double lat_to_merc_y(const double lat) {
  return kEarthRadiusMeters * std::log(std::tan(kPiD / 4.0 + lat * kPiD / 360.0));
}

midgard::AABB2<midgard::PointLL> tile_to_bbox(const uint32_t x, const uint32_t y, const uint32_t z) {

  const double n = std::pow(2.0, z);

  double min_lon = x / n * 360.0 - 180.0;
  double max_lon = (x + 1) / n * 360.0 - 180.0;

  double min_lat_rad = std::atan(std::sinh(kPiD * (1 - 2 * (y + 1) / n)));
  double max_lat_rad = std::atan(std::sinh(kPiD * (1 - 2 * y / n)));

  double min_lat = min_lat_rad * 180.0 / kPiD;
  double max_lat = max_lat_rad * 180.0 / kPiD;

  return AABB2<PointLL>(PointLL(min_lon, min_lat), PointLL(max_lon, max_lat));
}

/**
 * Encapsulates tile projection parameters for Web Mercator coordinate conversion
 */
struct TileProjection {
  TileProjection(const midgard::AABB2<midgard::PointLL>& bbox) {
    tile_merc_minx = lon_to_merc_x(bbox.minx());
    tile_merc_maxx = lon_to_merc_x(bbox.maxx());
    tile_merc_miny = lat_to_merc_y(bbox.miny());
    tile_merc_maxy = lat_to_merc_y(bbox.maxy());
    tile_merc_width = tile_merc_maxx - tile_merc_minx;
    tile_merc_height = tile_merc_maxy - tile_merc_miny;
  }
  double tile_merc_minx;
  double tile_merc_maxx;
  double tile_merc_miny;
  double tile_merc_maxy;
  double tile_merc_width;
  double tile_merc_height;
  int32_t tile_extent = 4096;
  int32_t tile_buffer = 128;
};

bg::point_2i_t ll_to_tile_coords(const midgard::Point2d merc_ll, const TileProjection& projection) {

  double norm_x = (merc_ll.x() - projection.tile_merc_minx) / projection.tile_merc_width;
  double norm_y = (projection.tile_merc_maxy - merc_ll.y()) / projection.tile_merc_height;

  int32_t tile_x = static_cast<int32_t>(std::round(norm_x * projection.tile_extent));
  int32_t tile_y = static_cast<int32_t>(std::round(norm_y * projection.tile_extent));

  return {tile_x, tile_y};
}

void filter_tile(const std::string& tile_bytes,
                 vtzero::tile_builder& filtered_tile,
                 const baldr::AttributesController& controller,
                 const bool return_shortcuts) {

  vtzero::vector_tile tile_full{tile_bytes};

  auto build_filtered_layer =
      [&](vtzero::layer& full_layer,
          const std::unordered_map<std::string_view, std::string_view>& prop_map) {
        vtzero::layer_builder filtered_layer{filtered_tile, full_layer};
        vtzero::property_mapper props_mapper{full_layer, filtered_layer};

        // pre-compute enabled attributes
        uint32_t shortcut_key_idx = UINT32_MAX;
        const auto& key_table = full_layer.key_table();
        std::vector<bool> attrs_allowed(key_table.size(), false);
        for (uint32_t i = 0; i < key_table.size(); ++i) {
          auto key = full_layer.key(vtzero::index_value{i});
          const std::string_view key_str{key.data(), key.size()};
          // mandatory fields are not part of AttributeController and that throws
          try {
            attrs_allowed[i] = controller(prop_map.at(key_str));
          } catch (const std::exception& e) { attrs_allowed[i] = true; }

          // on the full layer, shortcuts will always be enabled
          // if return_shortcuts=false, we'll discard shortcut features
          // TODO: make shortcuts their own layer, this is annoying
          if (key_str == "shortcut") {
            shortcut_key_idx = i;
          }
        }

        while (auto full_feat = full_layer.next_feature()) {
          vtzero::geometry_feature_builder filtered_feat{filtered_layer};
          filtered_feat.copy_id(full_feat);
          filtered_feat.set_geometry(full_feat.geometry());

          // filter attributes and treat shortcuts
          bool add_feat =
              full_feat.for_each_property_indexes([&](const vtzero::index_value_pair& idxs) {
                if (attrs_allowed[idxs.key().value()]) {
                  filtered_feat.add_property(props_mapper(idxs));
                }
                // TODO: make shortcuts their own layer, this is annoying
                if (!return_shortcuts && shortcut_key_idx == idxs.key().value()) {
                  return !full_layer.value(idxs.value().value()).bool_value();
                }
                return true;
              });
          if (add_feat) [[likely]]
            filtered_feat.commit();
          else [[unlikely]]
            filtered_feat.rollback();
        }
      };

  tile_full.for_each_layer([&](vtzero::layer&& full_layer) {
    const std::string_view layer_name{full_layer.name().data(), full_layer.name().size()};
    const auto& prop_map = layer_name == kNodeLayerName ? loki::detail::kNodePropToAttributeFlag
                                                        : loki::detail::kEdgePropToAttributeFlag;
    build_filtered_layer(full_layer, prop_map);

    return true;
  });
}

void build_nodes_layer(NodesLayerBuilder& nodes_builder,
                       const baldr::graph_tile_ptr& graph_tile,
                       const baldr::GraphId& node_id,
                       const TileProjection& projection) {
  const auto& node = *graph_tile->node(node_id);
  const auto& node_ll = node.latlng(graph_tile->header()->base_ll());
  const auto& admin_info = graph_tile->admininfo(node.admin_index());

  // Convert to tile coordinates
  const auto x = lon_to_merc_x(node_ll.x());
  const auto y = lat_to_merc_y(node_ll.y());
  const auto tile_coord = ll_to_tile_coords({x, y}, projection);

  auto tile_x = boost::geometry::get<0>(tile_coord);
  auto tile_y = boost::geometry::get<1>(tile_coord);

  // Only render nodes that are within the tile (including buffer)
  if (tile_x < -projection.tile_buffer || tile_x > projection.tile_extent + projection.tile_buffer ||
      tile_y < -projection.tile_buffer || tile_y > projection.tile_extent + projection.tile_buffer) {
    return;
  }

  // Add feature using the builder
  nodes_builder.add_feature(vtzero::point{tile_x, tile_y}, node_id, node, admin_info);
}

void build_layers(const std::shared_ptr<GraphReader>& reader,
                  vtzero::tile_builder& tile,
                  const midgard::AABB2<midgard::PointLL>& bounds,
                  const std::unordered_set<baldr::GraphId>& edge_ids,
                  const loki_worker_t::ZoomConfig& min_zoom_road_class,
                  uint32_t z,
                  const bool return_shortcuts,
                  const double generalize,
                  const baldr::AttributesController& controller) {
  const TileProjection projection{bounds};

  // create clip box with buffer to handle edges that cross boundaries
  const bg::box_2i_t clip_box(bg::point_2i_t(-projection.tile_buffer, -projection.tile_buffer),
                              bg::point_2i_t(projection.tile_extent + projection.tile_buffer,
                                             projection.tile_extent + projection.tile_buffer));
  const auto min_x = clip_box.min_corner().get<0>();
  const auto max_x = clip_box.max_corner().get<0>();
  const auto min_y = clip_box.min_corner().get<1>();
  const auto max_y = clip_box.max_corner().get<1>();

  EdgesLayerBuilder edges_builder(tile, kEdgeLayerName.data(), controller);
  NodesLayerBuilder nodes_builder(tile, kNodeLayerName.data(), controller);

  std::unordered_set<GraphId> unique_nodes;
  unique_nodes.reserve(edge_ids.size());
  bg::linestring_2d_t unclipped_line;
  unclipped_line.reserve(20);
  bg::multilinestring_2d_t clipped_lines;
  baldr::graph_tile_ptr edge_tile;
  // TODO(nils): sort edge_ids for better cache coherence
  for (const auto& edge_id : edge_ids) {
    // TODO(nils): create another array for tile level to quickly discard edges on lower zooms
    const auto* edge = reader->directededge(edge_id, edge_tile);

    // no shortcuts if not requested
    if (!return_shortcuts && edge->is_shortcut()) {
      continue;
    }

    // filter road classes by zoom
    auto road_class = edge->classification();
    uint32_t road_class_idx = static_cast<uint32_t>(road_class);
    if (z < min_zoom_road_class[road_class_idx]) {
      continue;
    }

    auto edge_info = edge_tile->edgeinfo(edge);
    auto shape = edge_info.shape();
    if (!edge->forward()) {
      std::reverse(shape.begin(), shape.end());
    }

    // project to pseudo mercator x/y for the generalization
    unclipped_line.clear();
    for (const auto& ll : shape) {
      auto tile_x = lon_to_merc_x(ll.lng());
      auto tile_y = lat_to_merc_y(ll.lat());

      boost::geometry::append(unclipped_line, decltype(unclipped_line)::value_type(tile_x, tile_y));
    }

    // scale the epsilon with generalize query parameter
    if (const auto gen_factor = PeuckerEpsilons[z] * generalize; generalize > 0. && gen_factor > 0.5)
      Polyline2<Point2d>::Generalize(unclipped_line, gen_factor);

    // convert to tile-local coords for the rest of the operations
    std::vector<vtzero::point> tile_coords;
    tile_coords.reserve(unclipped_line.size());
    bg::point_2i_t last_pt{INT32_MIN, INT32_MIN};
    bool line_leaves_bbox = false;
    for (const auto& pt : unclipped_line) {
      const auto& tile_coord = ll_to_tile_coords(pt, projection);
      // Skip consecutive duplicate points (can happen after rounding)
      if (boost::geometry::equals(tile_coord, last_pt)) {
        continue;
      }
      const auto x = tile_coord.get<0>();
      const auto y = tile_coord.get<1>();

      // only in this case we need an intersection with the clip_box
      line_leaves_bbox = x < min_x || x > max_x || y < min_y || y > max_y;

      tile_coords.emplace_back(x, y);
      last_pt = tile_coord;
    }

    // Must have at least 2 unique points to create a valid linestring
    if (tile_coords.size() < 2) {
      continue;
    }

    // lambda to add VT line & nodes features
    auto process_single_line = [&](const bg::linestring_2d_t& line) {
      // Check for opposing edge
      baldr::graph_tile_ptr opp_tile = edge_tile;
      const DirectedEdge* opp_edge = nullptr;
      GraphId opp_edge_id = reader->GetOpposingEdgeId(edge_id, opp_edge, opp_tile);

      const volatile baldr::TrafficSpeed* forward_traffic = &edge_tile->trafficspeed(edge);
      const volatile baldr::TrafficSpeed* reverse_traffic =
          opp_edge ? &opp_tile->trafficspeed(opp_edge) : nullptr;

      edges_builder.add_feature(tile_coords, edge_id, edge, opp_edge_id, opp_edge, forward_traffic,
                                reverse_traffic, edge_info);

      // adding nodes only works if we have the opposing tile
      if (opp_tile) {
        if (const auto& it = unique_nodes.insert(edge->endnode()); it.second) {
          build_nodes_layer(nodes_builder, opp_tile, *it.first, projection);
        }
        if (const auto& it = unique_nodes.insert(opp_edge->endnode()); it.second) {
          build_nodes_layer(nodes_builder, edge_tile, *it.first, projection);
        }
      }
    };

    if (!line_leaves_bbox) {
      process_single_line(unclipped_line);
      continue;
    }

    clipped_lines.clear();
    boost::geometry::intersection(clip_box, unclipped_line, clipped_lines);

    // skip if no clipped segments, i.e. edge is completely outside tile
    if (clipped_lines.empty()) {
      continue;
    }

    // process each clipped line segment (there may be multiple if line crosses tile multiple
    // times)
    for (const auto& clipped_line : clipped_lines) {
      process_single_line(clipped_line);
    }
  }
}
} // anonymous namespace

namespace valhalla {
namespace loki {

EdgesLayerBuilder::EdgesLayerBuilder(vtzero::tile_builder& tile,
                                     const char* name,
                                     const baldr::AttributesController& controller)
    : vtzero::layer_builder(tile, name), controller_(controller) {
  key_tile_level_ = add_key_without_dup_check("tile_level");
  key_road_class_ = add_key_without_dup_check("road_class");

  init_attribute_keys(loki::detail::kSharedEdgeAttributes, controller);
  init_attribute_keys(loki::detail::kForwardEdgeAttributes, controller);
  init_attribute_keys(loki::detail::kForwardLiveSpeedAttributes, controller);
  init_attribute_keys(loki::detail::kReverseEdgeAttributes, controller);
  init_attribute_keys(loki::detail::kReverseLiveSpeedAttributes, controller);

  // edge ids don't need all those attributes
  if (controller(kEdgeId)) {
    key_edge_id_fwd_ = add_key_without_dup_check("edge_id:fwd");
    key_edge_id_rev_ = add_key_without_dup_check("edge_id:bwd");
  }
}

void EdgesLayerBuilder::add_feature(const std::vector<vtzero::point>& geometry,
                                    baldr::GraphId forward_edge_id,
                                    const baldr::DirectedEdge* forward_edge,
                                    baldr::GraphId reverse_edge_id,
                                    const baldr::DirectedEdge* reverse_edge,
                                    const volatile baldr::TrafficSpeed* forward_traffic,
                                    const volatile baldr::TrafficSpeed* reverse_traffic,
                                    const baldr::EdgeInfo& edge_info) {
  assert(forward_edge || reverse_edge);

  const auto* edge = forward_edge ? forward_edge : reverse_edge;
  const GraphId& edge_id = forward_edge ? forward_edge_id : reverse_edge_id;

  // Create linestring feature for this edge
  vtzero::linestring_feature_builder feature{*this};
  feature.set_id(static_cast<uint64_t>(edge_id));
  feature.add_linestring_from_container(geometry);

  // Add shared tile properties (same for both directions)
  feature.add_property(key_tile_level_, vtzero::encoded_property_value(edge_id.level()));
  feature.add_property(key_road_class_,
                       vtzero::encoded_property_value(static_cast<uint32_t>(edge->classification())));

  set_attribute_values(loki::detail::kSharedEdgeAttributes, controller_, feature, *edge, edge_info,
                       nullptr);

  if (forward_edge) {
    if (controller_(kEdgeId))
      feature.add_property(key_edge_id_fwd_, vtzero::encoded_property_value(forward_edge_id.id()));
    set_attribute_values(loki::detail::kForwardEdgeAttributes, controller_, feature, *forward_edge,
                         edge_info, nullptr);
    if (forward_traffic) {
      set_attribute_values(loki::detail::kForwardLiveSpeedAttributes, controller_, feature,
                           *forward_edge, edge_info, forward_traffic);
    }
  }

  if (reverse_edge && reverse_edge_id.is_valid()) {
    if (controller_(kEdgeId))
      feature.add_property(key_edge_id_rev_, vtzero::encoded_property_value(reverse_edge_id.id()));
    set_attribute_values(loki::detail::kReverseEdgeAttributes, controller_, feature, *reverse_edge,
                         edge_info, nullptr);
    if (reverse_traffic) {
      set_attribute_values(loki::detail::kReverseLiveSpeedAttributes, controller_, feature,
                           *reverse_edge, edge_info, reverse_traffic);
    }
  }

  feature.commit();
}

NodesLayerBuilder::NodesLayerBuilder(vtzero::tile_builder& tile,
                                     const char* name,
                                     const baldr::AttributesController& controller)
    : vtzero::layer_builder(tile, name), controller_(controller) {
  // Pre-add keys for node properties
  key_tile_level_ = add_key_without_dup_check("tile_level");
  key_node_id_ = add_key_without_dup_check("node_id");
  key_node_type_ = add_key_without_dup_check("type");
  key_traffic_signal_ = add_key_without_dup_check("traffic_signal");

  for (const auto& def : loki::detail::kNodeAttributes) {
    if (controller(def.attribute_flag)) {
      this->*(def.key_member) = add_key_without_dup_check(def.key_name);
    }
  }

  if (controller(kAdminCountryCode))
    key_iso_3166_1_ = add_key_without_dup_check("iso_3166_1");
  if (controller(kAdminStateCode))
    key_iso_3166_2_ = add_key_without_dup_check("iso_3166_2");
}

void NodesLayerBuilder::add_feature(const vtzero::point& position,
                                    baldr::GraphId node_id,
                                    const baldr::NodeInfo& node,
                                    const baldr::AdminInfo& admin_info) {
  // Create point feature
  vtzero::point_feature_builder node_feature{*this};
  node_feature.set_id(static_cast<uint64_t>(node_id));
  node_feature.add_point(position);

  // Add tile properties (same structure as edges layer)
  node_feature.add_property(key_tile_level_, vtzero::encoded_property_value(node_id.level()));
  node_feature.add_property(key_node_id_, vtzero::encoded_property_value(node_id.id()));
  node_feature.add_property(key_node_type_,
                            vtzero::encoded_property_value(static_cast<uint32_t>(node.type())));
  node_feature.add_property(key_traffic_signal_,
                            vtzero::encoded_property_value(node.traffic_signal()));

  // Add node properties
  for (const auto& def : loki::detail::kNodeAttributes) {
    if (controller_(def.attribute_flag)) {
      const auto key = this->*(def.key_member);
      node_feature.add_property(key, def.value_func(node));
    }
  }

  if (controller_(kAdminCountryCode))
    node_feature.add_property(key_iso_3166_1_,
                              vtzero::encoded_property_value(admin_info.country_iso()));
  if (controller_(kAdminStateCode))
    node_feature.add_property(key_iso_3166_2_,
                              vtzero::encoded_property_value(admin_info.state_iso()));

  node_feature.commit();
}

std::string loki_worker_t::render_tile(Api& request) {
  const auto& options = request.options();

  vtzero::tile_builder tile;
  const auto z = options.tile_xyz().z();
  if (z < min_zoom_road_class_.front()) {
    return tile.serialize();
  } else if (z > min_zoom_road_class_.back()) {
    // throwing allows clients (mapblibre at least) to overzoom
    throw valhalla_exception_t{175, std::to_string(min_zoom_road_class_.back())};
  }

  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  // respect "allow_verbose" config
  const bool return_verbose = options.verbose() && allow_verbose;

  // creates controller, disables all attributes by default, but respect filters & verbose
  auto get_controller = [&options, &return_verbose]() {
    auto controller = baldr::AttributesController(options, true);
    bool only_base_props = (options.filter_action() == valhalla::no_action && (!return_verbose));
    if (only_base_props)
      controller.set_all(false);
    else if (return_verbose)
      controller.set_all(true);

    return controller;
  };
  auto controller = get_controller();

  // do we have it cached?
  const auto x = options.tile_xyz().x();
  const auto y = options.tile_xyz().y();
  const auto tile_path = detail::mvt_local_path(z, x, y, mvt_cache_dir_);
  bool cache_allowed = (z >= mvt_cache_min_zoom_) && !mvt_cache_dir_.empty();
  bool is_cached = false;
  bool return_shortcuts = options.tile_options().return_shortcuts();
  if (cache_allowed) {
    is_cached = std::filesystem::exists(tile_path);
    if (is_cached) {
      std::ifstream tile_file(tile_path, std::ios::binary);
      std::string buffer{std::istreambuf_iterator<char>(tile_file), std::istreambuf_iterator<char>()};
      // we only have cached tiles with all attributes
      if (return_verbose) {
        return buffer;
      }
      filter_tile(buffer, tile, controller, return_shortcuts);

      return tile.serialize();
    }
    // if we're caching, we need the full attributes
    controller.set_all(true);
    return_shortcuts = true;
  }

  // get lat/lon bbox
  const auto bounds = tile_to_bbox(x, y, z);

  // query edges in bbox, omits opposing edges
  const auto edge_ids = candidate_query_.RangeQuery(bounds);

  // we use generalize as a scaling factor to our default generalization
  const double generalize = options.has_generalize_case() ? options.generalize() : 4.;
  // build the full layers if cache is allowed, else whatever is in the controller
  build_layers(reader, tile, bounds, edge_ids, min_zoom_road_class_, z, return_shortcuts, generalize,
               controller);

  std::string tile_bytes;
  tile.serialize(tile_bytes);

  if (cache_allowed && !is_cached) {
    // atomically create the file
    auto tmp = tile_path;
    tmp += loki::detail::make_temp_name("_XXXXXX.tmp");
    try {
      std::filesystem::create_directories(tmp.parent_path());
    } catch (const std::filesystem::filesystem_error& e) {
      if (e.code() != std::errc::file_exists) {
        throw;
      }
    }
    std::ofstream out(tmp.string(), std::ios::binary);
    out.write(tile_bytes.data(), static_cast<std::streamsize>(tile_bytes.size()));
    out.close();
    if (!out) {
      LOG_WARN("Couldnt cache tile {}", tile_path.string());
    }

    std::filesystem::rename(tmp, tile_path);
  }

  if (return_verbose) {
    return tile_bytes;
  } else if (cache_allowed) {
    // only apply filter to the tile if we have a full tile (due to caching) but the request
    // wants a filtered tile

    vtzero::tile_builder filtered_tile;

    // need a fresh controller, the other one might have been changed if it was cacheable
    filter_tile(tile_bytes, filtered_tile, get_controller(),
                options.tile_options().return_shortcuts());

    return filtered_tile.serialize();
  }

  // we can only land here if cache isn't allowed, verbose=false and/or a filter is in the request
  return tile_bytes;
}

namespace detail {
/**
 * Make temp file name, mkstemp is POSIX & not implemented on Win
 *
 * @param template_name expects to end on XXXXXX (6 x "X")
 */
std::string make_temp_name(std::string template_name) {
  auto pos = template_name.rfind("XXXXXX");

  std::random_device rd;
  std::mt19937_64 rng((static_cast<uint64_t>(rd()) << 32) ^ static_cast<uint64_t>(rd()));

  static const char table[] = "abcdefghijklmnopqrstuvwxyz"
                              "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                              "0123456789";
  std::uniform_int_distribution<size_t> dist(0, sizeof(table) - 2);

  for (int i = 0; i < 6; ++i)
    template_name[pos + i] = table[dist(rng)];

  return template_name;
}

std::filesystem::path
mvt_local_path(const uint32_t z, const uint32_t x, const uint32_t y, const std::string& root) {
  static std::string kMvtExt = ".mvt";
  // number of cols & rows
  size_t dim = 1ull << z;

  // determine zero padded width for the full path
  size_t max_index = (dim * dim) - 1;
  size_t path_width = static_cast<size_t>(std::log10(max_index)) + 1;
  const size_t remainder = path_width % 3;
  if (remainder) {
    path_width += 3 - remainder;
  }
  assert(path_width % 3 == 0);

  // convert index to zero-padded decimal string
  size_t tile_index = static_cast<size_t>(y) * dim + static_cast<size_t>(x);
  std::ostringstream oss;
  oss << std::setw(path_width) << std::setfill('0') << tile_index;
  std::string path_no_sep = oss.str();

  // split into groups of 3 digits
  std::vector<std::string> groups;
  size_t i = 0;
  while (i < path_no_sep.size()) {
    groups.push_back(path_no_sep.substr(i, 3));
    i += 3;
  }

  std::filesystem::path tile_path = root;
  tile_path /= std::to_string(z);

  // append all groups but the last one, which is the filename
  for (size_t i = 0; i < groups.size() - 1; ++i) {
    tile_path /= groups[i];
  }
  tile_path /= groups.back() + kMvtExt;

  return tile_path;
}
} // namespace detail
} // namespace loki
} // namespace valhalla
