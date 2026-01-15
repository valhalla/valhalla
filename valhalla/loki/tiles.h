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
        const auto key = this->*(def.key_member);
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
  vtzero::index_value key_access_auto_fwd_;
  vtzero::index_value key_access_pedestrian_fwd_;
  vtzero::index_value key_access_bicycle_fwd_;
  vtzero::index_value key_access_truck_fwd_;
  vtzero::index_value key_access_emergency_fwd_;
  vtzero::index_value key_access_taxi_fwd_;
  vtzero::index_value key_access_bus_fwd_;
  vtzero::index_value key_access_hov_fwd_;
  vtzero::index_value key_access_wheelchair_fwd_;
  vtzero::index_value key_access_moped_fwd_;
  vtzero::index_value key_access_motorcycle_fwd_;
  // Access properties (reverse)
  vtzero::index_value key_access_auto_rev_;
  vtzero::index_value key_access_pedestrian_rev_;
  vtzero::index_value key_access_bicycle_rev_;
  vtzero::index_value key_access_truck_rev_;
  vtzero::index_value key_access_emergency_rev_;
  vtzero::index_value key_access_taxi_rev_;
  vtzero::index_value key_access_bus_rev_;
  vtzero::index_value key_access_hov_rev_;
  vtzero::index_value key_access_wheelchair_rev_;
  vtzero::index_value key_access_moped_rev_;
  vtzero::index_value key_access_motorcycle_rev_;
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
        "access:auto:fwd",
        baldr::kEdgeAccessAutoFwd,
        &EdgesLayerBuilder::key_access_auto_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.forwardaccess() & baldr::kAutoAccess)));
        },
    },
    {
        "access:pedestrian:fwd",
        baldr::kEdgeAccessPedestrianFwd,
        &EdgesLayerBuilder::key_access_pedestrian_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.forwardaccess() & baldr::kPedestrianAccess)));
        },
    },
    {
        "access:bicycle:fwd",
        baldr::kEdgeAccessBicycleFwd,
        &EdgesLayerBuilder::key_access_bicycle_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.forwardaccess() & baldr::kBicycleAccess)));
        },
    },
    {
        "access:truck:fwd",
        baldr::kEdgeAccessTruckFwd,
        &EdgesLayerBuilder::key_access_truck_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.forwardaccess() & baldr::kTruckAccess)));
        },
    },
    {
        "access:emergency:fwd",
        baldr::kEdgeAccessEmergencyFwd,
        &EdgesLayerBuilder::key_access_emergency_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.forwardaccess() & baldr::kEmergencyAccess)));
        },
    },
    {
        "access:taxi:fwd",
        baldr::kEdgeAccessTaxiFwd,
        &EdgesLayerBuilder::key_access_taxi_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.forwardaccess() & baldr::kTaxiAccess)));
        },
    },
    {
        "access:bus:fwd",
        baldr::kEdgeAccessBusFwd,
        &EdgesLayerBuilder::key_access_bus_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.forwardaccess() & baldr::kBusAccess)));
        },
    },
    {
        "access:hov:fwd",
        baldr::kEdgeAccessHovFwd,
        &EdgesLayerBuilder::key_access_hov_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.forwardaccess() & baldr::kHOVAccess)));
        },
    },
    {
        "access:wheelchair:fwd",
        baldr::kEdgeAccessWheelchairFwd,
        &EdgesLayerBuilder::key_access_wheelchair_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.forwardaccess() & baldr::kWheelchairAccess)));
        },
    },
    {
        "access:moped:fwd",
        baldr::kEdgeAccessMopedFwd,
        &EdgesLayerBuilder::key_access_moped_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.forwardaccess() & baldr::kMopedAccess)));
        },
    },
    {
        "access:motorcycle:fwd",
        baldr::kEdgeAccessMotorcycleFwd,
        &EdgesLayerBuilder::key_access_motorcycle_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.forwardaccess() & baldr::kMotorcycleAccess)));
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
        baldr::kEdgeLiveSpeed1Fwd,
        &EdgesLayerBuilder::key_live_speed1_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:fwd:speed2",
        baldr::kEdgeLiveSpeed2Fwd,
        &EdgesLayerBuilder::key_live_speed2_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:fwd:speed3",
        baldr::kEdgeLiveSpeed3Fwd,
        &EdgesLayerBuilder::key_live_speed3_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:fwd:bp1",
        baldr::kEdgeLiveSpeedBreakpoint1Fwd,
        &EdgesLayerBuilder::key_live_breakpoint1_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:fwd:bp2",
        baldr::kEdgeLiveSpeedBreakpoint2Fwd,
        &EdgesLayerBuilder::key_live_breakpoint2_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:fwd:congestion1",
        baldr::kEdgeLiveSpeedCongestion1Fwd,
        &EdgesLayerBuilder::key_live_congestion1_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:fwd:congestion2",
        baldr::kEdgeLiveSpeedCongestion2Fwd,
        &EdgesLayerBuilder::key_live_congestion2_fwd_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:fwd:congestion3",
        baldr::kEdgeLiveSpeedCongestion3Fwd,
        &EdgesLayerBuilder::key_live_congestion3_fwd_,
        [](EdgesLayerBuilder* layer_builder,
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
        "access:auto:bwd",
        baldr::kEdgeAccessAutoBwd,
        &EdgesLayerBuilder::key_access_auto_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.reverseaccess() & baldr::kAutoAccess)));
        },
    },
    {
        "access:pedestrian:bwd",
        baldr::kEdgeAccessPedestrianBwd,
        &EdgesLayerBuilder::key_access_pedestrian_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.reverseaccess() & baldr::kPedestrianAccess)));
        },
    },
    {
        "access:bicycle:bwd",
        baldr::kEdgeAccessBicycleBwd,
        &EdgesLayerBuilder::key_access_bicycle_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.reverseaccess() & baldr::kBicycleAccess)));
        },
    },
    {
        "access:truck:bwd",
        baldr::kEdgeAccessTruckBwd,
        &EdgesLayerBuilder::key_access_truck_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.reverseaccess() & baldr::kTruckAccess)));
        },
    },
    {
        "access:emergency:bwd",
        baldr::kEdgeAccessEmergencyBwd,
        &EdgesLayerBuilder::key_access_emergency_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.reverseaccess() & baldr::kEmergencyAccess)));
        },
    },
    {
        "access:taxi:bwd",
        baldr::kEdgeAccessTaxiBwd,
        &EdgesLayerBuilder::key_access_taxi_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.reverseaccess() & baldr::kTaxiAccess)));
        },
    },
    {
        "access:bus:bwd",
        baldr::kEdgeAccessBusBwd,
        &EdgesLayerBuilder::key_access_bus_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.reverseaccess() & baldr::kBusAccess)));
        },
    },
    {
        "access:hov:bwd",
        baldr::kEdgeAccessHovBwd,
        &EdgesLayerBuilder::key_access_hov_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.reverseaccess() & baldr::kHOVAccess)));
        },
    },
    {
        "access:wheelchair:bwd",
        baldr::kEdgeAccessWheelchairBwd,
        &EdgesLayerBuilder::key_access_wheelchair_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.reverseaccess() & baldr::kWheelchairAccess)));
        },
    },
    {
        "access:moped:bwd",
        baldr::kEdgeAccessMopedBwd,
        &EdgesLayerBuilder::key_access_moped_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.reverseaccess() & baldr::kMopedAccess)));
        },
    },
    {
        "access:motorcycle:bwd",
        baldr::kEdgeAccessMotorcycleBwd,
        &EdgesLayerBuilder::key_access_motorcycle_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const key_member,
           vtzero::linestring_feature_builder& feature,
           const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          feature.add_property(layer_builder->*(key_member),
                               static_cast<uint32_t>(
                                   static_cast<bool>(e.reverseaccess() & baldr::kMotorcycleAccess)));
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
        baldr::kEdgeLiveSpeed1Bwd,
        &EdgesLayerBuilder::key_live_speed1_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:bwd:speed2",
        baldr::kEdgeLiveSpeed2Bwd,
        &EdgesLayerBuilder::key_live_speed2_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:bwd:speed3",
        baldr::kEdgeLiveSpeed3Bwd,
        &EdgesLayerBuilder::key_live_speed3_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:bwd:bp1",
        baldr::kEdgeLiveSpeedBreakpoint1Bwd,
        &EdgesLayerBuilder::key_live_breakpoint1_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:bwd:bp2",
        baldr::kEdgeLiveSpeedBreakpoint2Bwd,
        &EdgesLayerBuilder::key_live_breakpoint2_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:bwd:congestion1",
        baldr::kEdgeLiveSpeedCongestion1Bwd,
        &EdgesLayerBuilder::key_live_congestion1_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:bwd:congestion2",
        baldr::kEdgeLiveSpeedCongestion2Bwd,
        &EdgesLayerBuilder::key_live_congestion2_rev_,
        [](EdgesLayerBuilder* layer_builder,
           vtzero::index_value valhalla::loki::EdgesLayerBuilder::*const,
           vtzero::linestring_feature_builder&,
           const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {},
    },
    {
        "live_speed:bwd:congestion3",
        baldr::kEdgeLiveSpeedCongestion3Bwd,
        &EdgesLayerBuilder::key_live_congestion3_rev_,
        [](EdgesLayerBuilder* layer_builder,
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
                               vtzero::encoded_property_value(
                                   static_cast<uint32_t>(e.is_shortcut())));
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
        baldr::kEdgeOsmId,
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
        "access:auto",
        baldr::kNodeAccessAuto,
        &NodesLayerBuilder::key_access_auto_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & baldr::kAutoAccess));
        },
    },
    {
        "access:pedestrian",
        baldr::kNodeAccessPedestrian,
        &NodesLayerBuilder::key_access_pedestrian_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(
              static_cast<bool>(ni.access() & baldr::kPedestrianAccess));
        },
    },
    {
        "access:bicycle",
        baldr::kNodeAccessBicycle,
        &NodesLayerBuilder::key_access_bicycle_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(
              static_cast<bool>(ni.access() & baldr::kBicycleAccess));
        },
    },
    {
        "access:truck",
        baldr::kNodeAccessTruck,
        &NodesLayerBuilder::key_access_truck_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & baldr::kTruckAccess));
        },
    },
    {
        "access:emergency",
        baldr::kNodeAccessEmergency,
        &NodesLayerBuilder::key_access_emergency_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(
              static_cast<bool>(ni.access() & baldr::kEmergencyAccess));
        },
    },
    {
        "access:taxi",
        baldr::kNodeAccessTaxi,
        &NodesLayerBuilder::key_access_taxi_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & baldr::kTaxiAccess));
        },
    },
    {
        "access:bus",
        baldr::kNodeAccessBus,
        &NodesLayerBuilder::key_access_bus_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & baldr::kBusAccess));
        },
    },
    {
        "access:hov",
        baldr::kNodeAccessHov,
        &NodesLayerBuilder::key_access_hov_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & baldr::kHOVAccess));
        },
    },
    {
        "access:wheelchair",
        baldr::kNodeAccessWheelchair,
        &NodesLayerBuilder::key_access_wheelchair_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(
              static_cast<bool>(ni.access() & baldr::kWheelchairAccess));
        },
    },
    {
        "access:moped",
        baldr::kNodeAccessMoped,
        &NodesLayerBuilder::key_access_moped_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & baldr::kMopedAccess));
        },
    },
    {
        "access:motorcycle",
        baldr::kNodeAccessMotorcycle,
        &NodesLayerBuilder::key_access_motorcycle_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(
              static_cast<bool>(ni.access() & baldr::kMotorcycleAccess));
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
    {"access:auto:fwd", baldr::kEdgeAccessAutoFwd},
    {"access:pedestrian:fwd", baldr::kEdgeAccessPedestrianFwd},
    {"access:bicycle:fwd", baldr::kEdgeAccessBicycleFwd},
    {"access:truck:fwd", baldr::kEdgeAccessTruckFwd},
    {"access:emergency:fwd", baldr::kEdgeAccessEmergencyFwd},
    {"access:taxi:fwd", baldr::kEdgeAccessTaxiFwd},
    {"access:bus:fwd", baldr::kEdgeAccessBusFwd},
    {"access:hov:fwd", baldr::kEdgeAccessHovFwd},
    {"access:wheelchair:fwd", baldr::kEdgeAccessWheelchairFwd},
    {"access:moped:fwd", baldr::kEdgeAccessMopedFwd},
    {"access:motorcycle:fwd", baldr::kEdgeAccessMotorcycleFwd},
    // Forward live speed
    {"live_speed:fwd", baldr::kEdgeLiveSpeedFwd},
    {"live_speed:fwd:speed1", baldr::kEdgeLiveSpeed1Fwd},
    {"live_speed:fwd:speed2", baldr::kEdgeLiveSpeed2Fwd},
    {"live_speed:fwd:speed3", baldr::kEdgeLiveSpeed3Fwd},
    {"live_speed:fwd:bp1", baldr::kEdgeLiveSpeedBreakpoint1Fwd},
    {"live_speed:fwd:bp2", baldr::kEdgeLiveSpeedBreakpoint2Fwd},
    {"live_speed:fwd:congestion1", baldr::kEdgeLiveSpeedCongestion1Fwd},
    {"live_speed:fwd:congestion2", baldr::kEdgeLiveSpeedCongestion2Fwd},
    {"live_speed:fwd:congestion3", baldr::kEdgeLiveSpeedCongestion3Fwd},
    // Reverse edge attributes
    {"speed:bwd", baldr::kEdgeSpeedBwd},
    {"deadend:bwd", baldr::kEdgeDeadendBwd},
    {"lanecount:bwd", baldr::kEdgeLaneCountBwd},
    {"truck_speed:bwd", baldr::kEdgeTruckSpeedBwd},
    {"traffic_signal:bwd", baldr::kEdgeSignalBwd},
    {"stop_sign:bwd", baldr::kEdgeStopSignBwd},
    {"yield_sign:bwd", baldr::kEdgeYieldBwd},
    {"access:auto:bwd", baldr::kEdgeAccessAutoBwd},
    {"access:pedestrian:bwd", baldr::kEdgeAccessPedestrianBwd},
    {"access:bicycle:bwd", baldr::kEdgeAccessBicycleBwd},
    {"access:truck:bwd", baldr::kEdgeAccessTruckBwd},
    {"access:emergency:bwd", baldr::kEdgeAccessEmergencyBwd},
    {"access:taxi:bwd", baldr::kEdgeAccessTaxiBwd},
    {"access:bus:bwd", baldr::kEdgeAccessBusBwd},
    {"access:hov:bwd", baldr::kEdgeAccessHovBwd},
    {"access:wheelchair:bwd", baldr::kEdgeAccessWheelchairBwd},
    {"access:moped:bwd", baldr::kEdgeAccessMopedBwd},
    {"access:motorcycle:bwd", baldr::kEdgeAccessMotorcycleBwd},
    // Reverse live speed
    {"live_speed:bwd", baldr::kEdgeLiveSpeedBwd},
    {"live_speed:bwd:speed1", baldr::kEdgeLiveSpeed1Bwd},
    {"live_speed:bwd:speed2", baldr::kEdgeLiveSpeed2Bwd},
    {"live_speed:bwd:speed3", baldr::kEdgeLiveSpeed3Bwd},
    {"live_speed:bwd:bp1", baldr::kEdgeLiveSpeedBreakpoint1Bwd},
    {"live_speed:bwd:bp2", baldr::kEdgeLiveSpeedBreakpoint2Bwd},
    {"live_speed:bwd:congestion1", baldr::kEdgeLiveSpeedCongestion1Bwd},
    {"live_speed:bwd:congestion2", baldr::kEdgeLiveSpeedCongestion2Bwd},
    {"live_speed:bwd:congestion3", baldr::kEdgeLiveSpeedCongestion3Bwd},
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
    {"osm_id", baldr::kEdgeOsmId},
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
    {"access:auto", baldr::kNodeAccessAuto},
    {"access:pedestrian", baldr::kNodeAccessPedestrian},
    {"access:bicycle", baldr::kNodeAccessBicycle},
    {"access:truck", baldr::kNodeAccessTruck},
    {"access:emergency", baldr::kNodeAccessEmergency},
    {"access:taxi", baldr::kNodeAccessTaxi},
    {"access:bus", baldr::kNodeAccessBus},
    {"access:hov", baldr::kNodeAccessHov},
    {"access:wheelchair", baldr::kNodeAccessWheelchair},
    {"access:moped", baldr::kNodeAccessMoped},
    {"access:motorcycle", baldr::kNodeAccessMotorcycle},
    // additional keys, they're added directly, not via NodeAttribute
    {"iso_3166_1", baldr::kAdminCountryCode},
    {"iso_3166_2", baldr::kAdminStateCode},
};
} // namespace detail

} // namespace valhalla::loki

#endif // __VALHALLA_LOKI_TILES_H__