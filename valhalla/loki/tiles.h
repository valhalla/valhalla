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

  using value_func_t = vtzero::encoded_property_value (*)(const baldr::DirectedEdge&,
                                                          const baldr::EdgeInfo&,
                                                          const volatile baldr::TrafficSpeed*);
  value_func_t value_func;
};

/**
 * Helper class to build the edges layer with pre-registered keys
 */
class EdgesLayerBuilder {
public:
  // it's a public layer, vtzero::property_mapper needs a mutable instance
  vtzero::layer_builder layer;

  explicit EdgesLayerBuilder(vtzero::tile_builder& tile,
                             const baldr::AttributesController& controller);

  template <std::size_t N>
  void set_attribute_values(const EdgeAttributeTile (&arr)[N],
                            const baldr::AttributesController& controller,
                            vtzero::linestring_feature_builder& feature,
                            const baldr::DirectedEdge& edge,
                            const baldr::EdgeInfo& edge_info,
                            const volatile baldr::TrafficSpeed* live_speed) {
    for (const auto& def : arr) {
      if (controller(def.attribute_flag)) {
        const auto key = this->*(def.key_member);
        feature.add_property(key, def.value_func(edge, edge_info, live_speed));
      }
    }
  }

  template <std::size_t N>
  void init_attribute_keys(const EdgeAttributeTile (&arr)[N],
                           const baldr::AttributesController& controller) {
    for (const auto& def : arr) {
      if (controller(def.attribute_flag))
        this->*(def.key_member) = layer.add_key_without_dup_check(def.key_name);
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
class NodesLayerBuilder {
public:
  vtzero::layer_builder layer;

  NodesLayerBuilder(vtzero::tile_builder& tile, const baldr::AttributesController& controller);

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
        "speed:forward",
        baldr::kEdgeSpeedFwd,
        &EdgesLayerBuilder::key_speed_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.speed()));
        },
    },
    {
        "deadend:forward",
        baldr::kEdgeDeadendFwd,
        &EdgesLayerBuilder::key_deadend_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.deadend()));
        },
    },
    {
        "lanecount:forward",
        baldr::kEdgeLaneCountFwd,
        &EdgesLayerBuilder::key_lanecount_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.lanecount()));
        },
    },
    {
        "truck_speed:forward",
        baldr::kEdgeTruckSpeedFwd,
        &EdgesLayerBuilder::key_truck_speed_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.truck_speed()));
        },
    },
    {
        "traffic_signal:forward",
        baldr::kEdgeSignalFwd,
        &EdgesLayerBuilder::key_traffic_signal_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.traffic_signal()));
        },
    },
    {
        "stop_sign:forward",
        baldr::kEdgeStopSignFwd,
        &EdgesLayerBuilder::key_stop_sign_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.stop_sign()));
        },
    },
    {
        "yield_sign:forward",
        baldr::kEdgeYieldFwd,
        &EdgesLayerBuilder::key_yield_sign_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.yield_sign()));
        },
    },
    {
        "access:auto:forward",
        baldr::kEdgeAccessAutoFwd,
        &EdgesLayerBuilder::key_access_auto_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & baldr::kAutoAccess)));
        },
    },
    {
        "access:pedestrian:forward",
        baldr::kEdgeAccessPedestrianFwd,
        &EdgesLayerBuilder::key_access_pedestrian_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & baldr::kPedestrianAccess)));
        },
    },
    {
        "access:bicycle:forward",
        baldr::kEdgeAccessBicycleFwd,
        &EdgesLayerBuilder::key_access_bicycle_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & baldr::kBicycleAccess)));
        },
    },
    {
        "access:truck:forward",
        baldr::kEdgeAccessTruckFwd,
        &EdgesLayerBuilder::key_access_truck_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & baldr::kTruckAccess)));
        },
    },
    {
        "access:emergency:forward",
        baldr::kEdgeAccessEmergencyFwd,
        &EdgesLayerBuilder::key_access_emergency_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & baldr::kEmergencyAccess)));
        },
    },
    {
        "access:taxi:forward",
        baldr::kEdgeAccessTaxiFwd,
        &EdgesLayerBuilder::key_access_taxi_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & baldr::kTaxiAccess)));
        },
    },
    {
        "access:bus:forward",
        baldr::kEdgeAccessBusFwd,
        &EdgesLayerBuilder::key_access_bus_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & baldr::kBusAccess)));
        },
    },
    {
        "access:hov:forward",
        baldr::kEdgeAccessHovFwd,
        &EdgesLayerBuilder::key_access_hov_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & baldr::kHOVAccess)));
        },
    },
    {
        "access:wheelchair:forward",
        baldr::kEdgeAccessWheelchairFwd,
        &EdgesLayerBuilder::key_access_wheelchair_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & baldr::kWheelchairAccess)));
        },
    },
    {
        "access:moped:forward",
        baldr::kEdgeAccessMopedFwd,
        &EdgesLayerBuilder::key_access_moped_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & baldr::kMopedAccess)));
        },
    },
    {
        "access:motorcycle:forward",
        baldr::kEdgeAccessMotorcycleFwd,
        &EdgesLayerBuilder::key_access_motorcycle_fwd_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & baldr::kMotorcycleAccess)));
        },
    },
};

static constexpr EdgeAttributeTile kForwardLiveSpeedAttributes[] = {

    {
        "live_speed:forward",
        baldr::kEdgeLiveSpeedFwd,
        &EdgesLayerBuilder::key_live_speed_fwd_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->get_overall_speed());
        },
    },
    {
        "live_speed:forward:speed1",
        baldr::kEdgeLiveSpeed1Fwd,
        &EdgesLayerBuilder::key_live_speed1_fwd_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->get_speed(0));
        },
    },
    {
        "live_speed:forward:speed2",
        baldr::kEdgeLiveSpeed2Fwd,
        &EdgesLayerBuilder::key_live_speed2_fwd_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->get_speed(1));
        },
    },
    {
        "live_speed:forward:speed3",
        baldr::kEdgeLiveSpeed3Fwd,
        &EdgesLayerBuilder::key_live_speed3_fwd_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->get_speed(2));
        },
    },
    {
        "live_speed:forward:breakpoint1",
        baldr::kEdgeLiveSpeedBreakpoint1Fwd,
        &EdgesLayerBuilder::key_live_breakpoint1_fwd_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->breakpoint1);
        },
    },
    {
        "live_speed:forward:breakpoint2",
        baldr::kEdgeLiveSpeedBreakpoint2Fwd,
        &EdgesLayerBuilder::key_live_breakpoint2_fwd_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->breakpoint2);
        },
    },
    {
        "live_speed:forward:congestion1",
        baldr::kEdgeLiveSpeedCongestion1Fwd,
        &EdgesLayerBuilder::key_live_congestion1_fwd_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->congestion1);
        },
    },
    {
        "live_speed:forward:congestion2",
        baldr::kEdgeLiveSpeedCongestion2Fwd,
        &EdgesLayerBuilder::key_live_congestion2_fwd_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->congestion2);
        },
    },
    {
        "live_speed:forward:congestion3",
        baldr::kEdgeLiveSpeedCongestion3Fwd,
        &EdgesLayerBuilder::key_live_congestion3_fwd_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->congestion3);
        },
    },
};

static constexpr EdgeAttributeTile kReverseEdgeAttributes[] = {
    {
        "speed:backward",
        baldr::kEdgeSpeedBwd,
        &EdgesLayerBuilder::key_speed_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.speed()));
        },
    },
    {
        "deadend:backward",
        baldr::kEdgeDeadendBwd,
        &EdgesLayerBuilder::key_deadend_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.deadend()));
        },
    },
    {
        "lanecount:backward",
        baldr::kEdgeLaneCountBwd,
        &EdgesLayerBuilder::key_lanecount_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.lanecount()));
        },
    },
    {
        "truck_speed:backward",
        baldr::kEdgeTruckSpeedBwd,
        &EdgesLayerBuilder::key_truck_speed_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.truck_speed()));
        },
    },
    {
        "traffic_signal:backward",
        baldr::kEdgeSignalBwd,
        &EdgesLayerBuilder::key_traffic_signal_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.traffic_signal()));
        },
    },
    {
        "stop_sign:backward",
        baldr::kEdgeStopSignBwd,
        &EdgesLayerBuilder::key_stop_sign_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.stop_sign()));
        },
    },
    {
        "yield_sign:backward",
        baldr::kEdgeYieldBwd,
        &EdgesLayerBuilder::key_yield_sign_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.yield_sign()));
        },
    },
    {
        "access:auto:backward",
        baldr::kEdgeAccessAutoBwd,
        &EdgesLayerBuilder::key_access_auto_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & baldr::kAutoAccess)));
        },
    },
    {
        "access:pedestrian:backward",
        baldr::kEdgeAccessPedestrianBwd,
        &EdgesLayerBuilder::key_access_pedestrian_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & baldr::kPedestrianAccess)));
        },
    },
    {
        "access:bicycle:backward",
        baldr::kEdgeAccessBicycleBwd,
        &EdgesLayerBuilder::key_access_bicycle_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & baldr::kBicycleAccess)));
        },
    },
    {
        "access:truck:backward",
        baldr::kEdgeAccessTruckBwd,
        &EdgesLayerBuilder::key_access_truck_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & baldr::kTruckAccess)));
        },
    },
    {
        "access:emergency:backward",
        baldr::kEdgeAccessEmergencyBwd,
        &EdgesLayerBuilder::key_access_emergency_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & baldr::kEmergencyAccess)));
        },
    },
    {
        "access:taxi:backward",
        baldr::kEdgeAccessTaxiBwd,
        &EdgesLayerBuilder::key_access_taxi_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & baldr::kTaxiAccess)));
        },
    },
    {
        "access:bus:backward",
        baldr::kEdgeAccessBusBwd,
        &EdgesLayerBuilder::key_access_bus_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & baldr::kBusAccess)));
        },
    },
    {
        "access:hov:backward",
        baldr::kEdgeAccessHovBwd,
        &EdgesLayerBuilder::key_access_hov_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & baldr::kHOVAccess)));
        },
    },
    {
        "access:wheelchair:backward",
        baldr::kEdgeAccessWheelchairBwd,
        &EdgesLayerBuilder::key_access_wheelchair_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & baldr::kWheelchairAccess)));
        },
    },
    {
        "access:moped:backward",
        baldr::kEdgeAccessMopedBwd,
        &EdgesLayerBuilder::key_access_moped_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & baldr::kMopedAccess)));
        },
    },
    {
        "access:motorcycle:backward",
        baldr::kEdgeAccessMotorcycleBwd,
        &EdgesLayerBuilder::key_access_motorcycle_rev_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & baldr::kMotorcycleAccess)));
        },
    },
};

static constexpr EdgeAttributeTile kReverseLiveSpeedAttributes[] = {

    {
        "live_speed:backward",
        baldr::kEdgeLiveSpeedBwd,
        &EdgesLayerBuilder::key_live_speed_rev_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->get_overall_speed());
        },
    },
    {
        "live_speed:backward:speed1",
        baldr::kEdgeLiveSpeed1Bwd,
        &EdgesLayerBuilder::key_live_speed1_rev_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->get_speed(0));
        },
    },
    {
        "live_speed:backward:speed2",
        baldr::kEdgeLiveSpeed2Bwd,
        &EdgesLayerBuilder::key_live_speed2_rev_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->get_speed(1));
        },
    },
    {
        "live_speed:backward:speed3",
        baldr::kEdgeLiveSpeed3Bwd,
        &EdgesLayerBuilder::key_live_speed3_rev_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->get_speed(2));
        },
    },
    {
        "live_speed:backward:breakpoint1",
        baldr::kEdgeLiveSpeedBreakpoint1Bwd,
        &EdgesLayerBuilder::key_live_breakpoint1_rev_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->breakpoint1);
        },
    },
    {
        "live_speed:backward:breakpoint2",
        baldr::kEdgeLiveSpeedBreakpoint2Bwd,
        &EdgesLayerBuilder::key_live_breakpoint2_rev_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->breakpoint2);
        },
    },
    {
        "live_speed:backward:congestion1",
        baldr::kEdgeLiveSpeedCongestion1Bwd,
        &EdgesLayerBuilder::key_live_congestion1_rev_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->congestion1);
        },
    },
    {
        "live_speed:backward:congestion2",
        baldr::kEdgeLiveSpeedCongestion2Bwd,
        &EdgesLayerBuilder::key_live_congestion2_rev_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->congestion2);
        },
    },
    {
        "live_speed:backward:congestion3",
        baldr::kEdgeLiveSpeedCongestion3Bwd,
        &EdgesLayerBuilder::key_live_congestion3_rev_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed* live_speed) {
          return vtzero::encoded_property_value(live_speed->congestion3);
        },
    },
};

static constexpr EdgeAttributeTile kSharedEdgeAttributes[] = {
    {
        "use",
        baldr::kEdgeUse,
        &EdgesLayerBuilder::key_use_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.use()));
        },
    },
    {
        "tunnel",
        baldr::kEdgeTunnel,
        &EdgesLayerBuilder::key_tunnel_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.tunnel());
        },
    },
    {
        "bridge",
        baldr::kEdgeBridge,
        &EdgesLayerBuilder::key_bridge_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.bridge());
        },
    },
    {
        "roundabout",
        baldr::kEdgeRoundabout,
        &EdgesLayerBuilder::key_roundabout_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.roundabout());
        },
    },
    {
        "leaves_tile",
        baldr::kEdgeLeavesTile,
        &EdgesLayerBuilder::key_leaves_tile_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.leaves_tile());
        },
    },
    {
        "length",
        baldr::kEdgeLength,
        &EdgesLayerBuilder::key_length_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.length());
        },
    },
    {
        "weighted_grade",
        baldr::kEdgeWeightedGrade,
        &EdgesLayerBuilder::key_weighted_grade_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.weighted_grade());
        },
    },
    {
        "max_up_slope",
        baldr::kEdgeMaxUpwardGrade,
        &EdgesLayerBuilder::key_max_up_slope_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.max_up_slope());
        },
    },
    {
        "max_down_slope",
        baldr::kEdgeMaxDownwardGrade,
        &EdgesLayerBuilder::key_max_down_slope_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.max_down_slope());
        },
    },
    {
        "curvature",
        baldr::kEdgeCurvature,
        &EdgesLayerBuilder::key_curvature_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.curvature());
        },
    },
    {
        "toll",
        baldr::kEdgeToll,
        &EdgesLayerBuilder::key_toll_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) { return vtzero::encoded_property_value(e.toll()); },
    },
    {
        "destonly",
        baldr::kEdgeDestinationOnly,
        &EdgesLayerBuilder::key_destonly_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.destonly());
        },
    },
    {
        "destonly_hgv",
        baldr::kEdgeDestinationOnlyHGV,
        &EdgesLayerBuilder::key_destonly_hgv_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.destonly_hgv());
        },
    },
    {
        "indoor",
        baldr::kEdgeIndoor,
        &EdgesLayerBuilder::key_indoor_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.indoor());
        },
    },
    {
        "hov_type",
        baldr::kEdgeHovType,
        &EdgesLayerBuilder::key_hov_type_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.hov_type()));
        },
    },
    {
        "cyclelane",
        baldr::kEdgeCycleLane,
        &EdgesLayerBuilder::key_cyclelane_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.cyclelane()));
        },
    },
    {
        "bike_network",
        baldr::kEdgeBicycleNetwork,
        &EdgesLayerBuilder::key_bike_network_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.bike_network());
        },
    },
    {
        "truck_route",
        baldr::kEdgeTruckRoute,
        &EdgesLayerBuilder::key_truck_route_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.truck_route());
        },
    },
    {
        "speed_type",
        baldr::kEdgeSpeedType,
        &EdgesLayerBuilder::key_speed_type_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.speed_type()));
        },
    },
    {
        "country_crossing",
        baldr::kEdgeCountryCrossing,
        &EdgesLayerBuilder::key_ctry_crossing_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.ctry_crossing());
        },
    },
    {
        "sac_scale",
        baldr::kEdgeSacScale,
        &EdgesLayerBuilder::key_sac_scale_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.sac_scale()));
        },
    },
    {
        "unpaved",
        baldr::kEdgeUnpaved,
        &EdgesLayerBuilder::key_unpaved_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.unpaved());
        },
    },
    {
        "surface",
        baldr::kEdgeSurface,
        &EdgesLayerBuilder::key_surface_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(static_cast<uint32_t>(e.surface()));
        },
    },
    {
        "ramp",
        baldr::kEdgeRamp,
        &EdgesLayerBuilder::key_link_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) { return vtzero::encoded_property_value(e.link()); },
    },
    {
        "internal",
        baldr::kEdgeInternalIntersection,
        &EdgesLayerBuilder::key_internal_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.internal());
        },
    },
    {
        "shoulder",
        baldr::kEdgeShoulder,
        &EdgesLayerBuilder::key_shoulder_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.shoulder());
        },
    },
    {
        "dismount",
        baldr::kEdgeDismount,
        &EdgesLayerBuilder::key_dismount_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.dismount());
        },
    },
    {
        "use_sidepath",
        baldr::kEdgeUseSidepath,
        &EdgesLayerBuilder::key_use_sidepath_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.use_sidepath());
        },
    },
    {
        "density",
        baldr::kEdgeDensity,
        &EdgesLayerBuilder::key_density_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.density());
        },
    },
    {
        "sidewalk_left",
        baldr::kEdgeSidewalkLeft,
        &EdgesLayerBuilder::key_sidewalk_left_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.sidewalk_left());
        },
    },
    {
        "sidewalk_right",
        baldr::kEdgeSidewalkRight,
        &EdgesLayerBuilder::key_sidewalk_right_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.sidewalk_right());
        },
    },
    {
        "bss_connection",
        baldr::kEdgeBSSConnection,
        &EdgesLayerBuilder::key_bss_connection_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.bss_connection());
        },
    },
    {
        "lit",
        baldr::kEdgeLit,
        &EdgesLayerBuilder::key_lit_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) { return vtzero::encoded_property_value(e.lit()); },
    },
    {
        "not_thru",
        baldr::kEdgeNotThru,
        &EdgesLayerBuilder::key_not_thru_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.not_thru());
        },
    },
    {
        "part_of_complex_restriction",
        baldr::kEdgePartComplexRestriction,
        &EdgesLayerBuilder::key_part_of_complex_restriction_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.part_of_complex_restriction());
        },
    },
    {
        "osm_id",
        baldr::kEdgeOsmId,
        &EdgesLayerBuilder::key_osm_way_id_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo& ei,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(ei.wayid());
        },
    },
    {
        "speed_limit",
        baldr::kEdgeSpeedLimit,
        &EdgesLayerBuilder::key_speed_limit_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo& ei,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(ei.speed_limit());
        },
    },
    {
        "layer",
        baldr::kEdgeLayer,
        &EdgesLayerBuilder::key_layer_,
        [](const baldr::DirectedEdge&,
           const baldr::EdgeInfo& ei,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(ei.layer());
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
    {"speed:forward", baldr::kEdgeSpeedFwd},
    {"deadend:forward", baldr::kEdgeDeadendFwd},
    {"lanecount:forward", baldr::kEdgeLaneCountFwd},
    {"truck_speed:forward", baldr::kEdgeTruckSpeedFwd},
    {"traffic_signal:forward", baldr::kEdgeSignalFwd},
    {"stop_sign:forward", baldr::kEdgeStopSignFwd},
    {"yield_sign:forward", baldr::kEdgeYieldFwd},
    {"access:auto:forward", baldr::kEdgeAccessAutoFwd},
    {"access:pedestrian:forward", baldr::kEdgeAccessPedestrianFwd},
    {"access:bicycle:forward", baldr::kEdgeAccessBicycleFwd},
    {"access:truck:forward", baldr::kEdgeAccessTruckFwd},
    {"access:emergency:forward", baldr::kEdgeAccessEmergencyFwd},
    {"access:taxi:forward", baldr::kEdgeAccessTaxiFwd},
    {"access:bus:forward", baldr::kEdgeAccessBusFwd},
    {"access:hov:forward", baldr::kEdgeAccessHovFwd},
    {"access:wheelchair:forward", baldr::kEdgeAccessWheelchairFwd},
    {"access:moped:forward", baldr::kEdgeAccessMopedFwd},
    {"access:motorcycle:forward", baldr::kEdgeAccessMotorcycleFwd},
    // Forward live speed
    {"live_speed:forward", baldr::kEdgeLiveSpeedFwd},
    {"live_speed:forward:speed1", baldr::kEdgeLiveSpeed1Fwd},
    {"live_speed:forward:speed2", baldr::kEdgeLiveSpeed2Fwd},
    {"live_speed:forward:speed3", baldr::kEdgeLiveSpeed3Fwd},
    {"live_speed:forward:breakpoint1", baldr::kEdgeLiveSpeedBreakpoint1Fwd},
    {"live_speed:forward:breakpoint2", baldr::kEdgeLiveSpeedBreakpoint2Fwd},
    {"live_speed:forward:congestion1", baldr::kEdgeLiveSpeedCongestion1Fwd},
    {"live_speed:forward:congestion2", baldr::kEdgeLiveSpeedCongestion2Fwd},
    {"live_speed:forward:congestion3", baldr::kEdgeLiveSpeedCongestion3Fwd},
    // Reverse edge attributes
    {"speed:backward", baldr::kEdgeSpeedBwd},
    {"deadend:backward", baldr::kEdgeDeadendBwd},
    {"lanecount:backward", baldr::kEdgeLaneCountBwd},
    {"truck_speed:backward", baldr::kEdgeTruckSpeedBwd},
    {"traffic_signal:backward", baldr::kEdgeSignalBwd},
    {"stop_sign:backward", baldr::kEdgeStopSignBwd},
    {"yield_sign:backward", baldr::kEdgeYieldBwd},
    {"access:auto:backward", baldr::kEdgeAccessAutoBwd},
    {"access:pedestrian:backward", baldr::kEdgeAccessPedestrianBwd},
    {"access:bicycle:backward", baldr::kEdgeAccessBicycleBwd},
    {"access:truck:backward", baldr::kEdgeAccessTruckBwd},
    {"access:emergency:backward", baldr::kEdgeAccessEmergencyBwd},
    {"access:taxi:backward", baldr::kEdgeAccessTaxiBwd},
    {"access:bus:backward", baldr::kEdgeAccessBusBwd},
    {"access:hov:backward", baldr::kEdgeAccessHovBwd},
    {"access:wheelchair:backward", baldr::kEdgeAccessWheelchairBwd},
    {"access:moped:backward", baldr::kEdgeAccessMopedBwd},
    {"access:motorcycle:backward", baldr::kEdgeAccessMotorcycleBwd},
    // Reverse live speed
    {"live_speed:backward", baldr::kEdgeLiveSpeedBwd},
    {"live_speed:backward:speed1", baldr::kEdgeLiveSpeed1Bwd},
    {"live_speed:backward:speed2", baldr::kEdgeLiveSpeed2Bwd},
    {"live_speed:backward:speed3", baldr::kEdgeLiveSpeed3Bwd},
    {"live_speed:backward:breakpoint1", baldr::kEdgeLiveSpeedBreakpoint1Bwd},
    {"live_speed:backward:breakpoint2", baldr::kEdgeLiveSpeedBreakpoint2Bwd},
    {"live_speed:backward:congestion1", baldr::kEdgeLiveSpeedCongestion1Bwd},
    {"live_speed:backward:congestion2", baldr::kEdgeLiveSpeedCongestion2Bwd},
    {"live_speed:backward:congestion3", baldr::kEdgeLiveSpeedCongestion3Bwd},
    // Shared edge attributes
    {"use", baldr::kEdgeUse},
    {"tunnel", baldr::kEdgeTunnel},
    {"bridge", baldr::kEdgeBridge},
    {"roundabout", baldr::kEdgeRoundabout},
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
    {"edge_id:forward", baldr::kEdgeId},
    {"edge_id:backward", baldr::kEdgeId},
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