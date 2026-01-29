#ifndef __VALHALLA_LOKI_TILES_H__
#define __VALHALLA_LOKI_TILES_H__

#include "baldr/admininfo.h"
#include "baldr/attributes_controller.h"
#include "baldr/datetime.h"
#include "baldr/directededge.h"
#include "baldr/edgeinfo.h"
#include "baldr/nodeinfo.h"
#include "baldr/traffictile.h"

#include <vtzero/builder.hpp>

#include <cassert>
#include <random>
#include <string_view>
#include <unordered_map>

namespace valhalla::loki {

class EdgesLayerBuilder;
struct EdgeAttributeTile {
  const char* key_name;
  std::string_view attribute_flag;
  vtzero::index_value EdgesLayerBuilder::*key_member;

  using prop_func_t = void (*)(EdgesLayerBuilder*,
                               vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
                               vtzero::linestring_feature_builder&,
                               const baldr::DirectedEdge&,
                               const baldr::EdgeInfo&,
                               const volatile baldr::TrafficSpeed*);
  prop_func_t prop_func;
};

/**
 * Helper class to build the edges layer with pre-registered keys
 */
class EdgesLayerBuilder : vtzero::layer_builder {
public:
  explicit EdgesLayerBuilder(vtzero::tile_builder& tile,
                             const char* name,
                             const baldr::AttributesController& controller);

  void set_attribute_values(const std::span<const EdgeAttributeTile> arr,
                            const baldr::AttributesController& controller,
                            vtzero::linestring_feature_builder& feature,
                            const baldr::DirectedEdge& edge,
                            const baldr::EdgeInfo& edge_info,
                            const volatile baldr::TrafficSpeed* live_speed) {
    for (const auto& def : arr) {
      if (controller(def.attribute_flag)) {
        def.prop_func(this, def.key_member, feature, edge, edge_info, live_speed);
      }
    }
  }

  void init_attribute_keys(const std::span<const EdgeAttributeTile> arr,
                           const baldr::AttributesController& controller) {
    for (const auto& def : arr) {
      if (controller(def.attribute_flag))
        this->*(def.key_member) = add_key_without_dup_check(def.key_name);
    }
  }

  void add_feature(const std::vector<vtzero::point>& geometry,
                   baldr::GraphId forward_edge_id,
                   const baldr::DirectedEdge* forward_edge,
                   baldr::GraphId reverse_edge_id,
                   const baldr::DirectedEdge* reverse_edge,
                   const volatile baldr::TrafficSpeed* forward_traffic,
                   const volatile baldr::TrafficSpeed* reverse_traffic,
                   const baldr::EdgeInfo& edge_info);

  // Pre-registered keys
  vtzero::index_value key_tile_level_;
  vtzero::index_value key_edge_id_fwd_;
  vtzero::index_value key_edge_id_rev_;
  vtzero::index_value key_road_class_;
  vtzero::index_value key_use_;
  vtzero::index_value key_speed_fwd_;
  vtzero::index_value key_speed_rev_;
  vtzero::index_value key_tunnel_;
  vtzero::index_value key_bridge_;
  vtzero::index_value key_roundabout_;
  vtzero::index_value key_is_shortcut_;
  vtzero::index_value key_leaves_tile_;
  // Shared edge properties
  vtzero::index_value key_length_;
  vtzero::index_value key_weighted_grade_;
  vtzero::index_value key_max_up_slope_;
  vtzero::index_value key_max_down_slope_;
  vtzero::index_value key_curvature_;
  vtzero::index_value key_toll_;
  vtzero::index_value key_destonly_;
  vtzero::index_value key_destonly_hgv_;
  vtzero::index_value key_indoor_;
  vtzero::index_value key_hov_type_;
  vtzero::index_value key_cyclelane_;
  vtzero::index_value key_bike_network_;
  vtzero::index_value key_truck_route_;
  vtzero::index_value key_lanecount_;
  vtzero::index_value key_speed_type_;
  vtzero::index_value key_ctry_crossing_;
  vtzero::index_value key_sac_scale_;
  vtzero::index_value key_unpaved_;
  vtzero::index_value key_surface_;
  vtzero::index_value key_link_;
  vtzero::index_value key_internal_;
  vtzero::index_value key_shoulder_;
  vtzero::index_value key_dismount_;
  vtzero::index_value key_use_sidepath_;
  vtzero::index_value key_density_;
  vtzero::index_value key_sidewalk_left_;
  vtzero::index_value key_sidewalk_right_;
  vtzero::index_value key_bss_connection_;
  vtzero::index_value key_lit_;
  vtzero::index_value key_not_thru_;
  vtzero::index_value key_part_of_complex_restriction_;
  vtzero::index_value key_osm_way_id_;
  vtzero::index_value key_speed_limit_;
  vtzero::index_value key_layer_;
  // Direction-specific properties
  vtzero::index_value key_truck_speed_fwd_;
  vtzero::index_value key_truck_speed_rev_;
  vtzero::index_value key_deadend_fwd_;
  vtzero::index_value key_deadend_rev_;
  vtzero::index_value key_lanecount_fwd_;
  vtzero::index_value key_lanecount_rev_;
  vtzero::index_value key_traffic_signal_fwd_;
  vtzero::index_value key_traffic_signal_rev_;
  vtzero::index_value key_stop_sign_fwd_;
  vtzero::index_value key_stop_sign_rev_;
  vtzero::index_value key_yield_sign_fwd_;
  vtzero::index_value key_yield_sign_rev_;
  // Access properties (forward)
  vtzero::index_value key_access_fwd_;
  // Access properties (reverse)
  vtzero::index_value key_access_rev_;
  // Traffic speed keys (forward)
  vtzero::index_value key_live_speed_fwd_;
  vtzero::index_value key_live_speed1_fwd_;
  vtzero::index_value key_live_speed2_fwd_;
  vtzero::index_value key_live_speed3_fwd_;
  vtzero::index_value key_live_breakpoint1_fwd_;
  vtzero::index_value key_live_breakpoint2_fwd_;
  vtzero::index_value key_live_congestion1_fwd_;
  vtzero::index_value key_live_congestion2_fwd_;
  vtzero::index_value key_live_congestion3_fwd_;
  // Traffic speed keys (reverse)
  vtzero::index_value key_live_speed_rev_;
  vtzero::index_value key_live_speed1_rev_;
  vtzero::index_value key_live_speed2_rev_;
  vtzero::index_value key_live_speed3_rev_;
  vtzero::index_value key_live_breakpoint1_rev_;
  vtzero::index_value key_live_breakpoint2_rev_;
  vtzero::index_value key_live_congestion1_rev_;
  vtzero::index_value key_live_congestion2_rev_;
  vtzero::index_value key_live_congestion3_rev_;

private:
  const baldr::AttributesController& controller_;
};

class NodesLayerBuilder;
struct NodeAttributeTile {
  const char* key_name;
  std::string_view attribute_flag;
  vtzero::index_value NodesLayerBuilder::*key_member;

  using value_func_t = vtzero::encoded_property_value (*)(const baldr::NodeInfo&);
  value_func_t value_func;
};

/**
 * Helper class to build the nodes layer with pre-registered keys
 */
class NodesLayerBuilder : vtzero::layer_builder {
public:
  NodesLayerBuilder(vtzero::tile_builder& tile,
                    const char* name,
                    const baldr::AttributesController& controller);

  void add_feature(const vtzero::point& position,
                   baldr::GraphId node_id,
                   const baldr::NodeInfo& node,
                   const baldr::AdminInfo& admin_info);

  vtzero::index_value key_tile_level_;
  vtzero::index_value key_node_id_;
  vtzero::index_value key_node_type_;
  vtzero::index_value key_traffic_signal_;
  vtzero::index_value key_access_auto_;
  vtzero::index_value key_access_pedestrian_;
  vtzero::index_value key_access_bicycle_;
  vtzero::index_value key_access_truck_;
  vtzero::index_value key_access_emergency_;
  vtzero::index_value key_access_taxi_;
  vtzero::index_value key_access_bus_;
  vtzero::index_value key_access_hov_;
  vtzero::index_value key_access_wheelchair_;
  vtzero::index_value key_access_moped_;
  vtzero::index_value key_access_motorcycle_;
  vtzero::index_value key_intersection_;
  vtzero::index_value key_drive_on_right_;
  vtzero::index_value key_elevation_;
  vtzero::index_value key_tagged_access_;
  vtzero::index_value key_private_access_;
  vtzero::index_value key_cash_only_toll_;
  vtzero::index_value key_mode_change_;
  vtzero::index_value key_named_intersection_;
  vtzero::index_value key_timezone_;
  vtzero::index_value key_iso_3166_1_;
  vtzero::index_value key_iso_3166_2_;

private:
  const baldr::AttributesController controller_;
};

namespace detail {
/**
 * Make temp file name, mkstemp is POSIX & not implemented on Win
 *
 * @param template_name expects to end on XXXXXX (6 x "X")
 */
std::string make_temp_name(std::string template_name);

/**
 * Make the disk path for a given MVT tile.
 *
 * @param z the zoom level
 * @param x the x coordinate
 * @param y the y coordinate
 * @param root the MVT cache dir root
 */
std::filesystem::path
mvt_local_path(const uint32_t z, const uint32_t x, const uint32_t y, const std::string& root);

static constexpr EdgeAttributeTile kForwardEdgeAttributes[] = {
    {
        "speed:fwd",
        baldr::kEdgeSpeedFwd,
        &EdgesLayerBuilder::key_speed_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.speed())));
        },
    },
    {
        "deadend:fwd",
        baldr::kEdgeDeadendFwd,
        &EdgesLayerBuilder::key_deadend_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.deadend())));
        },
    },
    {
        "lanecount:fwd",
        baldr::kEdgeLaneCountFwd,
        &EdgesLayerBuilder::key_lanecount_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.lanecount())));
        },
    },
    {
        "truck_speed:fwd",
        baldr::kEdgeTruckSpeedFwd,
        &EdgesLayerBuilder::key_truck_speed_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.truck_speed())));
        },
    },
    {
        "traffic_signal:fwd",
        baldr::kEdgeSignalFwd,
        &EdgesLayerBuilder::key_traffic_signal_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.traffic_signal())));
        },
    },
    {
        "stop_sign:fwd",
        baldr::kEdgeStopSignFwd,
        &EdgesLayerBuilder::key_stop_sign_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.stop_sign())));
        },
    },
    {
        "yield_sign:fwd",
        baldr::kEdgeYieldFwd,
        &EdgesLayerBuilder::key_yield_sign_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.yield_sign())));
        },
    },
    {
        "access:fwd",
        baldr::kEdgeAccessFwd,
        &EdgesLayerBuilder::key_access_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(static_cast<bool>(e.forwardaccess())));
        },
    },
};

static constexpr EdgeAttributeTile kForwardLiveSpeedAttributes[] = {

    {
        "live_speed:fwd",
        baldr::kEdgeLiveSpeedFwd,
        &EdgesLayerBuilder::key_live_speed_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(live_speed->get_overall_speed()));
          feature.add_property(layer_builder->key_live_speed1_fwd_,
                               vtzero::encoded_property_value(live_speed->get_speed(0)));
          feature.add_property(layer_builder->key_live_speed2_fwd_,
                               vtzero::encoded_property_value(live_speed->get_speed(1)));
          feature.add_property(layer_builder->key_live_speed3_fwd_,
                               vtzero::encoded_property_value(live_speed->get_speed(2)));
          feature.add_property(layer_builder->key_live_breakpoint1_fwd_,
                               vtzero::encoded_property_value(live_speed->breakpoint1));
          feature.add_property(layer_builder->key_live_breakpoint2_fwd_,
                               vtzero::encoded_property_value(live_speed->breakpoint2));
          feature.add_property(layer_builder->key_live_congestion1_fwd_,
                               vtzero::encoded_property_value(live_speed->congestion1));
          feature.add_property(layer_builder->key_live_congestion2_fwd_,
                               vtzero::encoded_property_value(live_speed->congestion2));
          feature.add_property(layer_builder->key_live_congestion3_fwd_,
                               vtzero::encoded_property_value(live_speed->congestion3));
        },
    },
    // kept for conceptual consistency sake
    {
        "live_speed:fwd:speed1",
        baldr::kEdgeLiveSpeedFwd,
        &EdgesLayerBuilder::key_live_speed1_fwd_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:fwd:speed2",
        baldr::kEdgeLiveSpeedFwd,
        &EdgesLayerBuilder::key_live_speed2_fwd_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:fwd:speed3",
        baldr::kEdgeLiveSpeedFwd,
        &EdgesLayerBuilder::key_live_speed3_fwd_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:fwd:bp1",
        baldr::kEdgeLiveSpeedFwd,
        &EdgesLayerBuilder::key_live_breakpoint1_fwd_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:fwd:bp2",
        baldr::kEdgeLiveSpeedFwd,
        &EdgesLayerBuilder::key_live_breakpoint2_fwd_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:fwd:congestion1",
        baldr::kEdgeLiveSpeedFwd,
        &EdgesLayerBuilder::key_live_congestion1_fwd_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:fwd:congestion2",
        baldr::kEdgeLiveSpeedFwd,
        &EdgesLayerBuilder::key_live_congestion2_fwd_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:fwd:congestion3",
        baldr::kEdgeLiveSpeedFwd,
        &EdgesLayerBuilder::key_live_congestion3_fwd_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
};

static constexpr EdgeAttributeTile kReverseEdgeAttributes[] = {
    {
        "speed:bwd",
        baldr::kEdgeSpeedBwd,
        &EdgesLayerBuilder::key_speed_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.speed())));
        },
    },
    {
        "deadend:bwd",
        baldr::kEdgeDeadendBwd,
        &EdgesLayerBuilder::key_deadend_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.deadend())));
        },
    },
    {
        "lanecount:bwd",
        baldr::kEdgeLaneCountBwd,
        &EdgesLayerBuilder::key_lanecount_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.lanecount())));
        },
    },
    {
        "truck_speed:bwd",
        baldr::kEdgeTruckSpeedBwd,
        &EdgesLayerBuilder::key_truck_speed_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.truck_speed())));
        },
    },
    {
        "traffic_signal:bwd",
        baldr::kEdgeSignalBwd,
        &EdgesLayerBuilder::key_traffic_signal_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.traffic_signal())));
        },
    },
    {
        "stop_sign:bwd",
        baldr::kEdgeStopSignBwd,
        &EdgesLayerBuilder::key_stop_sign_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.stop_sign())));
        },
    },
    {
        "yield_sign:bwd",
        baldr::kEdgeYieldBwd,
        &EdgesLayerBuilder::key_yield_sign_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.yield_sign())));
        },
    },
    {
        "access:bwd",
        baldr::kEdgeAccessBwd,
        &EdgesLayerBuilder::key_access_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(static_cast<bool>(e.forwardaccess())));
        },
    },
};

static constexpr EdgeAttributeTile kReverseLiveSpeedAttributes[] = {

    {
        "live_speed:bwd",
        baldr::kEdgeLiveSpeedBwd,
        &EdgesLayerBuilder::key_live_speed_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          feature.add_property(layer_builder->key_live_speed_rev_,
                               vtzero::encoded_property_value(live_speed->get_overall_speed()));
          feature.add_property(layer_builder->key_live_speed1_rev_,
                               vtzero::encoded_property_value(live_speed->get_speed(0)));
          feature.add_property(layer_builder->key_live_speed2_rev_,
                               vtzero::encoded_property_value(live_speed->get_speed(1)));
          feature.add_property(layer_builder->key_live_speed3_rev_,
                               vtzero::encoded_property_value(live_speed->get_speed(2)));
          feature.add_property(layer_builder->key_live_breakpoint1_rev_,
                               vtzero::encoded_property_value(live_speed->breakpoint1));
          feature.add_property(layer_builder->key_live_breakpoint2_rev_,
                               vtzero::encoded_property_value(live_speed->breakpoint2));
          feature.add_property(layer_builder->key_live_congestion1_rev_,
                               vtzero::encoded_property_value(live_speed->congestion1));
          feature.add_property(layer_builder->key_live_congestion2_rev_,
                               vtzero::encoded_property_value(live_speed->congestion2));
          feature.add_property(layer_builder->key_live_congestion3_rev_,
                               vtzero::encoded_property_value(live_speed->congestion3));
        },
    },
    {
        "live_speed:bwd:speed1",
        baldr::kEdgeLiveSpeedBwd,
        &EdgesLayerBuilder::key_live_speed1_rev_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:bwd:speed2",
        baldr::kEdgeLiveSpeedBwd,
        &EdgesLayerBuilder::key_live_speed2_rev_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:bwd:speed3",
        baldr::kEdgeLiveSpeedBwd,
        &EdgesLayerBuilder::key_live_speed3_rev_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:bwd:bp1",
        baldr::kEdgeLiveSpeedBwd,
        &EdgesLayerBuilder::key_live_breakpoint1_rev_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:bwd:bp2",
        baldr::kEdgeLiveSpeedBwd,
        &EdgesLayerBuilder::key_live_breakpoint2_rev_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:bwd:congestion1",
        baldr::kEdgeLiveSpeedBwd,
        &EdgesLayerBuilder::key_live_congestion1_rev_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:bwd:congestion2",
        baldr::kEdgeLiveSpeedBwd,
        &EdgesLayerBuilder::key_live_congestion2_rev_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:bwd:congestion3",
        baldr::kEdgeLiveSpeedBwd,
        &EdgesLayerBuilder::key_live_congestion3_rev_,
        [](EdgesLayerBuilder*,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
};

static constexpr EdgeAttributeTile kSharedEdgeAttributes[] = {
    {
        "use",
        baldr::kEdgeUse,
        &EdgesLayerBuilder::key_use_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.use())));
        },
    },
    {
        "tunnel",
        baldr::kEdgeTunnel,
        &EdgesLayerBuilder::key_tunnel_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.tunnel())));
        },
    },
    {
        "bridge",
        baldr::kEdgeBridge,
        &EdgesLayerBuilder::key_bridge_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.bridge())));
        },
    },
    {
        "roundabout",
        baldr::kEdgeRoundabout,
        &EdgesLayerBuilder::key_roundabout_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.roundabout())));
        },
    },
    {
        "shortcut",
        baldr::kEdgeShortcut,
        &EdgesLayerBuilder::key_is_shortcut_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<bool>(e.is_shortcut())));
        },
    },
    {
        "leaves_tile",
        baldr::kEdgeLeavesTile,
        &EdgesLayerBuilder::key_leaves_tile_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.leaves_tile())));
        },
    },
    {
        "length",
        baldr::kEdgeLength,
        &EdgesLayerBuilder::key_length_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.length())));
        },
    },
    {
        "weighted_grade",
        baldr::kEdgeWeightedGrade,
        &EdgesLayerBuilder::key_weighted_grade_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.weighted_grade())));
        },
    },
    {
        "max_up_slope",
        baldr::kEdgeMaxUpwardGrade,
        &EdgesLayerBuilder::key_max_up_slope_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.max_up_slope())));
        },
    },
    {
        "max_down_slope",
        baldr::kEdgeMaxDownwardGrade,
        &EdgesLayerBuilder::key_max_down_slope_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.max_down_slope())));
        },
    },
    {
        "curvature",
        baldr::kEdgeCurvature,
        &EdgesLayerBuilder::key_curvature_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.curvature())));
        },
    },
    {
        "toll",
        baldr::kEdgeToll,
        &EdgesLayerBuilder::key_toll_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.curvature())));
        },
    },
    {
        "destonly",
        baldr::kEdgeDestinationOnly,
        &EdgesLayerBuilder::key_destonly_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.destonly())));
        },
    },
    {
        "destonly_hgv",
        baldr::kEdgeDestinationOnlyHGV,
        &EdgesLayerBuilder::key_destonly_hgv_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.destonly_hgv())));
        },
    },
    {
        "indoor",
        baldr::kEdgeIndoor,
        &EdgesLayerBuilder::key_indoor_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.indoor())));
        },
    },
    {
        "hov_type",
        baldr::kEdgeHovType,
        &EdgesLayerBuilder::key_hov_type_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.hov_type())));
        },
    },
    {
        "cyclelane",
        baldr::kEdgeCycleLane,
        &EdgesLayerBuilder::key_cyclelane_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.cyclelane())));
        },
    },
    {
        "bike_network",
        baldr::kEdgeBicycleNetwork,
        &EdgesLayerBuilder::key_bike_network_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.bike_network())));
        },
    },
    {
        "truck_route",
        baldr::kEdgeTruckRoute,
        &EdgesLayerBuilder::key_truck_route_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.truck_route())));
        },
    },
    {
        "speed_type",
        baldr::kEdgeSpeedType,
        &EdgesLayerBuilder::key_speed_type_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.speed_type())));
        },
    },
    {
        "country_crossing",
        baldr::kEdgeCountryCrossing,
        &EdgesLayerBuilder::key_ctry_crossing_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.ctry_crossing())));
        },
    },
    {
        "sac_scale",
        baldr::kEdgeSacScale,
        &EdgesLayerBuilder::key_sac_scale_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.sac_scale())));
        },
    },
    {
        "unpaved",
        baldr::kEdgeUnpaved,
        &EdgesLayerBuilder::key_unpaved_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.unpaved())));
        },
    },
    {
        "surface",
        baldr::kEdgeSurface,
        &EdgesLayerBuilder::key_surface_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.surface())));
        },
    },
    {
        "ramp",
        baldr::kEdgeRamp,
        &EdgesLayerBuilder::key_link_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.link())));
        },
    },
    {
        "internal",
        baldr::kEdgeInternalIntersection,
        &EdgesLayerBuilder::key_internal_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.internal())));
        },
    },
    {
        "shoulder",
        baldr::kEdgeShoulder,
        &EdgesLayerBuilder::key_shoulder_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.shoulder())));
        },
    },
    {
        "dismount",
        baldr::kEdgeDismount,
        &EdgesLayerBuilder::key_dismount_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.dismount())));
        },
    },
    {
        "use_sidepath",
        baldr::kEdgeUseSidepath,
        &EdgesLayerBuilder::key_use_sidepath_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.use_sidepath())));
        },
    },
    {
        "density",
        baldr::kEdgeDensity,
        &EdgesLayerBuilder::key_density_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.density())));
        },
    },
    {
        "sidewalk_left",
        baldr::kEdgeSidewalkLeft,
        &EdgesLayerBuilder::key_sidewalk_left_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.sidewalk_left())));
        },
    },
    {
        "sidewalk_right",
        baldr::kEdgeSidewalkRight,
        &EdgesLayerBuilder::key_sidewalk_right_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.sidewalk_right())));
        },
    },
    {
        "bss_connection",
        baldr::kEdgeBSSConnection,
        &EdgesLayerBuilder::key_bss_connection_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.bss_connection())));
        },
    },
    {
        "lit",
        baldr::kEdgeLit,
        &EdgesLayerBuilder::key_lit_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.bss_connection())));
        },
    },
    {
        "not_thru",
        baldr::kEdgeNotThru,
        &EdgesLayerBuilder::key_not_thru_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(e.not_thru())));
        },
    },
    {
        "part_of_complex_restriction",
        baldr::kEdgePartComplexRestriction,
        &EdgesLayerBuilder::key_part_of_complex_restriction_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.part_of_complex_restriction())));
        },
    },
    {
        "osm_id",
        baldr::kEdgeWayId,
        &EdgesLayerBuilder::key_osm_way_id_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo& ei,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(ei.wayid())));
        },
    },
    {
        "speed_limit",
        baldr::kEdgeSpeedLimit,
        &EdgesLayerBuilder::key_speed_limit_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo& ei,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(ei.speed_limit())));
        },
    },
    {
        "layer",
        baldr::kEdgeLayer,
        &EdgesLayerBuilder::key_layer_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo& ei,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               vtzero::encoded_property_value(static_cast<uint32_t>(ei.layer())));
        },
    },
};

static constexpr NodeAttributeTile kNodeAttributes[] = {
    {
        "drive_on_right",
        baldr::kNodeDriveOnRight,
        &NodesLayerBuilder::key_drive_on_right_,
        [](const baldr::NodeInfo& ni) { return vtzero::encoded_property_value(ni.drive_on_right()); },
    },
    {
        "elevation",
        baldr::kNodeElevation,
        &NodesLayerBuilder::key_elevation_,
        [](const baldr::NodeInfo& ni) { return vtzero::encoded_property_value(ni.elevation()); },
    },
    {
        "tagged_access",
        baldr::kNodeTaggedAccess,
        &NodesLayerBuilder::key_tagged_access_,
        [](const baldr::NodeInfo& ni) { return vtzero::encoded_property_value(ni.tagged_access()); },
    },
    {
        "private_access",
        baldr::kNodePrivateAccess,
        &NodesLayerBuilder::key_private_access_,
        [](const baldr::NodeInfo& ni) { return vtzero::encoded_property_value(ni.private_access()); },
    },
    {
        "cash_only_toll",
        baldr::kNodeCashOnlyToll,
        &NodesLayerBuilder::key_cash_only_toll_,
        [](const baldr::NodeInfo& ni) { return vtzero::encoded_property_value(ni.cash_only_toll()); },
    },
    {
        "mode_change_allowed",
        baldr::kNodeModeChangeAllowed,
        &NodesLayerBuilder::key_mode_change_,
        [](const baldr::NodeInfo& ni) { return vtzero::encoded_property_value(ni.mode_change()); },
    },
    {
        "named_intersection",
        baldr::kNodeNamedIntersection,
        &NodesLayerBuilder::key_named_intersection_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(ni.named_intersection());
        },
    },
    {
        "timezone",
        baldr::kNodeTimeZone,
        &NodesLayerBuilder::key_timezone_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(
              ni.timezone() ? baldr::DateTime::get_tz_db().from_index(ni.timezone())->name() : "");
        },
    },
    {
        "access",
        baldr::kNodeAccess,
        &NodesLayerBuilder::key_access_auto_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access()));
        },
    },
};

// map from MVT prop name to controller attribute flag for edge properties
static const std::unordered_map<std::string_view, std::string_view> kEdgePropToAttributeFlag = {
    // Forward edge attributes
    {"speed:fwd", baldr::kEdgeSpeedFwd},
    {"deadend:fwd", baldr::kEdgeDeadendFwd},
    {"lanecount:fwd", baldr::kEdgeLaneCountFwd},
    {"truck_speed:fwd", baldr::kEdgeTruckSpeedFwd},
    {"traffic_signal:fwd", baldr::kEdgeSignalFwd},
    {"stop_sign:fwd", baldr::kEdgeStopSignFwd},
    {"yield_sign:fwd", baldr::kEdgeYieldFwd},
    {"access:fwd", baldr::kEdgeAccessFwd},
    // Forward live speed
    {"live_speed:fwd", baldr::kEdgeLiveSpeedFwd},
    {"live_speed:fwd:speed1", baldr::kEdgeLiveSpeedFwd},
    {"live_speed:fwd:speed2", baldr::kEdgeLiveSpeedFwd},
    {"live_speed:fwd:speed3", baldr::kEdgeLiveSpeedFwd},
    {"live_speed:fwd:bp1", baldr::kEdgeLiveSpeedFwd},
    {"live_speed:fwd:bp2", baldr::kEdgeLiveSpeedFwd},
    {"live_speed:fwd:congestion1", baldr::kEdgeLiveSpeedFwd},
    {"live_speed:fwd:congestion2", baldr::kEdgeLiveSpeedFwd},
    {"live_speed:fwd:congestion3", baldr::kEdgeLiveSpeedFwd},
    // Reverse edge attributes
    {"speed:bwd", baldr::kEdgeSpeedBwd},
    {"deadend:bwd", baldr::kEdgeDeadendBwd},
    {"lanecount:bwd", baldr::kEdgeLaneCountBwd},
    {"truck_speed:bwd", baldr::kEdgeTruckSpeedBwd},
    {"traffic_signal:bwd", baldr::kEdgeSignalBwd},
    {"stop_sign:bwd", baldr::kEdgeStopSignBwd},
    {"yield_sign:bwd", baldr::kEdgeYieldBwd},
    {"access:bwd", baldr::kEdgeAccessBwd},
    // Reverse live speed
    {"live_speed:bwd", baldr::kEdgeLiveSpeedBwd},
    {"live_speed:bwd:speed1", baldr::kEdgeLiveSpeedBwd},
    {"live_speed:bwd:speed2", baldr::kEdgeLiveSpeedBwd},
    {"live_speed:bwd:speed3", baldr::kEdgeLiveSpeedBwd},
    {"live_speed:bwd:bp1", baldr::kEdgeLiveSpeedBwd},
    {"live_speed:bwd:bp2", baldr::kEdgeLiveSpeedBwd},
    {"live_speed:bwd:congestion1", baldr::kEdgeLiveSpeedBwd},
    {"live_speed:bwd:congestion2", baldr::kEdgeLiveSpeedBwd},
    {"live_speed:bwd:congestion3", baldr::kEdgeLiveSpeedBwd},
    // Shared edge attributes
    {"use", baldr::kEdgeUse},
    {"tunnel", baldr::kEdgeTunnel},
    {"bridge", baldr::kEdgeBridge},
    {"roundabout", baldr::kEdgeRoundabout},
    {"shortcut", baldr::kEdgeShortcut},
    {"leaves_tile", baldr::kEdgeLeavesTile},
    {"length", baldr::kEdgeLength},
    {"weighted_grade", baldr::kEdgeWeightedGrade},
    {"max_up_slope", baldr::kEdgeMaxUpwardGrade},
    {"max_down_slope", baldr::kEdgeMaxDownwardGrade},
    {"curvature", baldr::kEdgeCurvature},
    {"toll", baldr::kEdgeToll},
    {"destonly", baldr::kEdgeDestinationOnly},
    {"destonly_hgv", baldr::kEdgeDestinationOnlyHGV},
    {"indoor", baldr::kEdgeIndoor},
    {"hov_type", baldr::kEdgeHovType},
    {"cyclelane", baldr::kEdgeCycleLane},
    {"bike_network", baldr::kEdgeBicycleNetwork},
    {"truck_route", baldr::kEdgeTruckRoute},
    {"speed_type", baldr::kEdgeSpeedType},
    {"country_crossing", baldr::kEdgeCountryCrossing},
    {"sac_scale", baldr::kEdgeSacScale},
    {"unpaved", baldr::kEdgeUnpaved},
    {"surface", baldr::kEdgeSurface},
    {"ramp", baldr::kEdgeRamp},
    {"internal", baldr::kEdgeInternalIntersection},
    {"shoulder", baldr::kEdgeShoulder},
    {"dismount", baldr::kEdgeDismount},
    {"use_sidepath", baldr::kEdgeUseSidepath},
    {"density", baldr::kEdgeDensity},
    {"sidewalk_left", baldr::kEdgeSidewalkLeft},
    {"sidewalk_right", baldr::kEdgeSidewalkRight},
    {"bss_connection", baldr::kEdgeBSSConnection},
    {"lit", baldr::kEdgeLit},
    {"not_thru", baldr::kEdgeNotThru},
    {"part_of_complex_restriction", baldr::kEdgePartComplexRestriction},
    {"osm_id", baldr::kEdgeWayId},
    {"speed_limit", baldr::kEdgeSpeedLimit},
    {"layer", baldr::kEdgeLayer},
    // additional keys, they're added directly, not via EdgeAttributeTile
    {"edge_id:fwd", baldr::kEdgeId},
    {"edge_id:bwd", baldr::kEdgeId},
};

// map from MVT prop name to controller attribute flag for node properties
static const std::unordered_map<std::string_view, std::string_view> kNodePropToAttributeFlag = {
    {"drive_on_right", baldr::kNodeDriveOnRight},
    {"elevation", baldr::kNodeElevation},
    {"tagged_access", baldr::kNodeTaggedAccess},
    {"private_access", baldr::kNodePrivateAccess},
    {"cash_only_toll", baldr::kNodeCashOnlyToll},
    {"mode_change_allowed", baldr::kNodeModeChangeAllowed},
    {"named_intersection", baldr::kNodeNamedIntersection},
    {"timezone", baldr::kNodeTimeZone},
    {"access", baldr::kNodeAccess},
    // additional keys, they're added directly, not via NodeAttribute
    {"iso_3166_1", baldr::kAdminCountryCode},
    {"iso_3166_2", baldr::kAdminStateCode},
};
} // namespace detail

} // namespace valhalla::loki

#endif // __VALHALLA_LOKI_TILES_H__