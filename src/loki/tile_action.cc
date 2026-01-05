#include "baldr/attributes_controller.h"
#include "baldr/datetime.h"
#include "baldr/directededge.h"
#include "baldr/graphreader.h"
#include "baldr/nodeinfo.h"
#include "baldr/tilehierarchy.h"
#include "loki/worker.h"
#include "meili/candidate_search.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "utils.h"
#include "valhalla/exceptions.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/multi_linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/property_tree/ptree.hpp>
#include <vtzero/builder.hpp>
#include <vtzero/property_mapper.hpp>

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <string_view>
#include <unordered_set>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace {
/**
 * Earth radius in meters for EPSG:3857 Web Mercator projection.
 * This is the WGS84 ellipsoid semi-major axis.
 */
constexpr double kEarthRadiusMeters = 6378137.0;
constexpr std::string_view kEdgeLayerName = "edges";
constexpr std::string_view kNodeLayerName = "nodes";

class EdgesLayerBuilder;
struct EdgeAttribute {
  const char* key_name;
  std::string_view attribute_flag;
  vtzero::index_value EdgesLayerBuilder::*key_member;

  using value_func_t = vtzero::encoded_property_value (*)(const baldr::DirectedEdge&,
                                                          const baldr::EdgeInfo&,
                                                          const volatile baldr::TrafficSpeed*);
  value_func_t value_func;
};

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
  void set_attribute_values(const EdgeAttribute (&arr)[N],
                            const AttributesController& controller,
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
  void init_attribute_keys(const EdgeAttribute (&arr)[N], const AttributesController& controller) {
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
  vtzero::index_value key_tile_id_;
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

static constexpr EdgeAttribute kForwardEdgeAttributes[] = {
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
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & kAutoAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & kPedestrianAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & kBicycleAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & kTruckAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & kEmergencyAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & kTaxiAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & kBusAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & kHOVAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & kWheelchairAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & kMopedAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.forwardaccess() & kMotorcycleAccess)));
        },
    },
};

static constexpr EdgeAttribute kForwardLiveSpeedAttributes[] = {

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

static constexpr EdgeAttribute kReverseEdgeAttributes[] = {
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
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & kAutoAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & kPedestrianAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & kBicycleAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & kTruckAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & kEmergencyAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & kTaxiAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & kBusAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & kHOVAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & kWheelchairAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & kMopedAccess)));
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
              static_cast<uint32_t>(static_cast<bool>(e.reverseaccess() & kMotorcycleAccess)));
        },
    },
};

static constexpr EdgeAttribute kReverseLiveSpeedAttributes[] = {

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

static constexpr EdgeAttribute kSharedEdgeAttributes[] = {
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
        "shortcut",
        baldr::kEdgeShortcut,
        &EdgesLayerBuilder::key_is_shortcut_,
        [](const baldr::DirectedEdge& e,
           const baldr::EdgeInfo&,
           const volatile baldr::TrafficSpeed*) {
          return vtzero::encoded_property_value(e.is_shortcut());
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
        kEdgeCurvature,
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
        baldr::kEdgeDestinationOnly,
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

EdgesLayerBuilder::EdgesLayerBuilder(vtzero::tile_builder& tile,
                                     const baldr::AttributesController& controller)
    : layer(tile, kEdgeLayerName.data()), controller_(controller) {
  key_tile_level_ = layer.add_key_without_dup_check("tile_level");
  key_tile_id_ = layer.add_key_without_dup_check("tile_id");
  key_road_class_ = layer.add_key_without_dup_check("road_class");

  init_attribute_keys(kSharedEdgeAttributes, controller);
  init_attribute_keys(kForwardEdgeAttributes, controller);
  init_attribute_keys(kForwardLiveSpeedAttributes, controller);
  init_attribute_keys(kReverseEdgeAttributes, controller);
  init_attribute_keys(kReverseLiveSpeedAttributes, controller);

  // edge ids don't need all those attributes
  if (controller(kEdgeId)) {
    key_edge_id_fwd_ = layer.add_key_without_dup_check("edge_id:forward");
    key_edge_id_rev_ = layer.add_key_without_dup_check("edge_id:backward");
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
  // Must have at least one edge and valid geometry
  if (geometry.size() < 2) {
    return;
  }

  assert(forward_edge || reverse_edge);

  const auto* edge = forward_edge ? forward_edge : reverse_edge;
  const GraphId& edge_id = forward_edge ? forward_edge_id : reverse_edge_id;

  // Create linestring feature for this edge
  vtzero::linestring_feature_builder feature{layer};
  feature.set_id(static_cast<uint64_t>(edge_id));
  feature.add_linestring_from_container(geometry);

  // Add shared tile properties (same for both directions)
  feature.add_property(key_tile_level_, vtzero::encoded_property_value(edge_id.level()));
  feature.add_property(key_tile_id_, vtzero::encoded_property_value(edge_id.tileid()));
  feature.add_property(key_road_class_,
                       vtzero::encoded_property_value(static_cast<uint32_t>(edge->classification())));

  set_attribute_values(kSharedEdgeAttributes, controller_, feature, *edge, edge_info, nullptr);

  if (forward_edge) {
    if (controller_(kEdgeId))
      feature.add_property(key_edge_id_fwd_, vtzero::encoded_property_value(forward_edge_id.id()));
    set_attribute_values(kForwardEdgeAttributes, controller_, feature, *forward_edge, edge_info,
                         nullptr);
    if (forward_traffic) {
      set_attribute_values(kForwardLiveSpeedAttributes, controller_, feature, *forward_edge,
                           edge_info, forward_traffic);
    }
  }

  if (reverse_edge && reverse_edge_id.is_valid()) {
    if (controller_(kEdgeId))
      feature.add_property(key_edge_id_rev_, vtzero::encoded_property_value(reverse_edge_id.id()));
    set_attribute_values(kReverseEdgeAttributes, controller_, feature, *reverse_edge, edge_info,
                         nullptr);
    if (reverse_traffic) {
      set_attribute_values(kReverseLiveSpeedAttributes, controller_, feature, *reverse_edge,
                           edge_info, reverse_traffic);
    }
  }

  feature.commit();
}

class NodesLayerBuilder;
struct NodeAttribute {
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
  vtzero::index_value key_tile_id_;
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
  vtzero::index_value key_is_transit_;
  vtzero::index_value key_timezone_;
  vtzero::index_value key_iso_3166_1_;
  vtzero::index_value key_iso_3166_2_;

private:
  const baldr::AttributesController controller_;
};

static constexpr NodeAttribute kNodeAttributes[] = {
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
        "is_transit",
        baldr::kNodeNamedIntersection,
        &NodesLayerBuilder::key_is_transit_,
        [](const baldr::NodeInfo& ni) { return vtzero::encoded_property_value(ni.is_transit()); },
    },
    {
        "timezone",
        baldr::kNodeTimeZone,
        &NodesLayerBuilder::key_timezone_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(
              ni.timezone() ? DateTime::get_tz_db().from_index(ni.timezone())->name() : "");
        },
    },
    {
        "access:auto",
        baldr::kNodeAccessAuto,
        &NodesLayerBuilder::key_access_auto_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & kAutoAccess));
        },
    },
    {
        "access:pedestrian",
        baldr::kNodeAccessPedestrian,
        &NodesLayerBuilder::key_access_pedestrian_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & kPedestrianAccess));
        },
    },
    {
        "access:bicycle",
        baldr::kNodeAccessBicycle,
        &NodesLayerBuilder::key_access_bicycle_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & kBicycleAccess));
        },
    },
    {
        "access:truck",
        baldr::kNodeAccessTruck,
        &NodesLayerBuilder::key_access_truck_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & kTruckAccess));
        },
    },
    {
        "access:emergency",
        baldr::kNodeAccessEmergency,
        &NodesLayerBuilder::key_access_emergency_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & kEmergencyAccess));
        },
    },
    {
        "access:taxi",
        baldr::kNodeAccessTaxi,
        &NodesLayerBuilder::key_access_taxi_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & kTaxiAccess));
        },
    },
    {
        "access:bus",
        baldr::kNodeAccessBus,
        &NodesLayerBuilder::key_access_bus_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & kBusAccess));
        },
    },
    {
        "access:hov",
        baldr::kNodeAccessHov,
        &NodesLayerBuilder::key_access_hov_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & kHOVAccess));
        },
    },
    {
        "access:wheelchair",
        baldr::kNodeAccessWheelchair,
        &NodesLayerBuilder::key_access_wheelchair_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & kWheelchairAccess));
        },
    },
    {
        "access:moped",
        baldr::kNodeAccessMoped,
        &NodesLayerBuilder::key_access_moped_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & kMopedAccess));
        },
    },
    {
        "access:motorcycle",
        baldr::kNodeAccessMotorcycle,
        &NodesLayerBuilder::key_access_motorcycle_,
        [](const baldr::NodeInfo& ni) {
          return vtzero::encoded_property_value(static_cast<bool>(ni.access() & kMotorcycleAccess));
        },
    },
};

NodesLayerBuilder::NodesLayerBuilder(vtzero::tile_builder& tile,
                                     const baldr::AttributesController& controller)
    : layer(tile, kNodeLayerName.data()), controller_(controller) {
  // Pre-add keys for node properties
  key_tile_level_ = layer.add_key_without_dup_check("tile_level");
  key_tile_id_ = layer.add_key_without_dup_check("tile_id");
  key_node_id_ = layer.add_key_without_dup_check("node_id");
  key_node_type_ = layer.add_key_without_dup_check("type");
  key_traffic_signal_ = layer.add_key_without_dup_check("traffic_signal");

  for (const auto& def : kNodeAttributes) {
    if (controller(def.attribute_flag)) {
      this->*(def.key_member) = layer.add_key_without_dup_check(def.key_name);
    }
  }

  if (controller(kAdminCountryCode))
    key_iso_3166_1_ = layer.add_key_without_dup_check("iso_3166_1");
  if (controller(kAdminStateCode))
    key_iso_3166_2_ = layer.add_key_without_dup_check("iso_3166_2");
}

void NodesLayerBuilder::add_feature(const vtzero::point& position,
                                    baldr::GraphId node_id,
                                    const baldr::NodeInfo& node,
                                    const baldr::AdminInfo& admin_info) {
  // Create point feature
  vtzero::point_feature_builder node_feature{layer};
  node_feature.set_id(static_cast<uint64_t>(node_id));
  node_feature.add_point(position);

  // Add tile properties (same structure as edges layer)
  node_feature.add_property(key_tile_level_, vtzero::encoded_property_value(node_id.level()));
  node_feature.add_property(key_tile_id_, vtzero::encoded_property_value(node_id.tileid()));
  node_feature.add_property(key_node_id_, vtzero::encoded_property_value(node_id.id()));
  node_feature.add_property(key_node_type_,
                            vtzero::encoded_property_value(static_cast<uint32_t>(node.type())));
  node_feature.add_property(key_traffic_signal_,
                            vtzero::encoded_property_value(node.traffic_signal()));

  // Add node properties
  for (const auto& def : kNodeAttributes) {
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

std::pair<int32_t, int32_t> ll_to_tile_coords(const midgard::PointLL node_ll,
                                              const TileProjection& projection) {
  double merc_x = lon_to_merc_x(node_ll.lng());
  double merc_y = lat_to_merc_y(node_ll.lat());

  double norm_x = (merc_x - projection.tile_merc_minx) / projection.tile_merc_width;
  double norm_y = (projection.tile_merc_maxy - merc_y) / projection.tile_merc_height;

  int32_t tile_x = static_cast<int32_t>(std::round(norm_x * projection.tile_extent));
  int32_t tile_y = static_cast<int32_t>(std::round(norm_y * projection.tile_extent));

  return {tile_x, tile_y};
}

void filter_layer(const char* name,
                  vtzero::layer& full_layer,
                  vtzero::layer_builder& filtered_layer,
                  bool return_shortcuts) {
  while (auto full_feat = full_layer.next_feature()) {
    // TODO: make shortcuts their own layer, this is annoying
    if (!return_shortcuts) {
      bool is_not_shortcut = full_feat.for_each_property([&](const vtzero::property& p) {
        return !(p.key().compare(vtzero::data_view{baldr::kEdgeShortcut.data()}) == 0);
      });
      if (!is_not_shortcut) {
        continue;
      }
    }

    // create new feature and copy all attributes via a mapper
    vtzero::geometry_feature_builder filtered_feat{filtered_layer};
    filtered_feat.copy_id(full_feat);
    filtered_feat.set_geometry(full_feat.geometry());

    vtzero::property_mapper props_mapper{full_layer, filtered_layer};
    filtered_feat.copy_properties(full_feat, props_mapper);

    filtered_feat.commit();
  }
}

void build_nodes_layer(NodesLayerBuilder& nodes_builder,
                       const baldr::graph_tile_ptr& graph_tile,
                       const baldr::GraphId& node_id,
                       const TileProjection& projection) {
  const auto& node = *graph_tile->node(node_id);
  const auto& node_ll = node.latlng(graph_tile->header()->base_ll());
  const auto& admin_info = graph_tile->admininfo(node.admin_index());

  // Convert to tile coordinates
  const auto [tile_x, tile_y] = ll_to_tile_coords(node_ll, projection);

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
                  bool return_shortcuts,
                  const baldr::AttributesController& controller) {
  using point_t = boost::geometry::model::d2::point_xy<double>;
  using linestring_t = boost::geometry::model::linestring<point_t>;
  using multi_linestring_t = boost::geometry::model::multi_linestring<linestring_t>;
  using box_t = boost::geometry::model::box<point_t>;

  const TileProjection projection{bounds};

  // create clip box with buffer to handle edges that cross boundaries
  const box_t clip_box(point_t(-projection.tile_buffer, -projection.tile_buffer),
                       point_t(projection.tile_extent + projection.tile_buffer,
                               projection.tile_extent + projection.tile_buffer));

  EdgesLayerBuilder edges_builder(tile, controller);
  NodesLayerBuilder nodes_builder(tile, controller);

  std::unordered_set<GraphId> unique_nodes;
  unique_nodes.reserve(edge_ids.size());
  linestring_t unclipped_line;
  unclipped_line.reserve(20);
  multi_linestring_t clipped_lines;
  baldr::graph_tile_ptr edge_tile;
  // TODO(nils): sort edge_ids
  for (const auto& edge_id : edge_ids) {
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

    // lambda to add VT line & nodes features
    auto process_single_line = [&](const linestring_t& line) {
      // convert to vtzero points, removing consecutive duplicates
      std::vector<vtzero::point> tile_coords;
      tile_coords.reserve(line.size());

      int32_t last_x = INT32_MIN;
      int32_t last_y = INT32_MIN;

      for (const auto& pt : line) {
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
        return;
      }

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

    unclipped_line.clear();
    bool line_leaves_bbox = false;
    for (const auto& ll : shape) {
      const auto [tile_x, tile_y] = ll_to_tile_coords(ll, projection);

      // only in this case we need an intersection with the clip_box
      line_leaves_bbox = tile_x > clip_box.max_corner().x() || tile_y > clip_box.max_corner().y() ||
                         tile_x < clip_box.min_corner().x() || tile_y < clip_box.min_corner().y();

      // TODO(nils): is there a point promoting int32 to double? could just register a custom boost
      // geometry type for vtzero points
      boost::geometry::append(unclipped_line, point_t(tile_x, tile_y));
    }

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

namespace detail {
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

  // disable all attributes by default, but respect filters & verbose
  auto controller = baldr::AttributesController(options, true);
  bool only_base_props = (options.filter_action() == valhalla::no_action && !options.verbose());
  if (only_base_props)
    controller.set_all(false);
  else if (options.verbose())
    controller.set_all(true);

  // do we have it cached?
  const auto x = options.tile_xyz().x();
  const auto y = options.tile_xyz().y();
  const auto tile_path = detail::mvt_local_path(z, x, y, mvt_cache_dir_);
  bool cache_allowed = (z >= mvt_cache_min_zoom_) && !mvt_cache_dir_.empty();
  bool is_cached = false;
  if (cache_allowed) {
    is_cached = std::filesystem::exists(tile_path);
    if (is_cached) {
      std::ifstream tile_file(tile_path, std::ios::binary);
      std::string buffer{std::istreambuf_iterator<char>(tile_file), std::istreambuf_iterator<char>()};
      // we only have cached tiles with all attributes
      if (options.verbose()) {
        return buffer;
      }
      // if the layers should be filtered, we need to clone the relevant attributes
      vtzero::vector_tile tile_full{buffer};
      tile_full.for_each_layer([&](vtzero::layer&& full_layer) {
        if (full_layer.name().data() == kEdgeLayerName.data()) {
          EdgesLayerBuilder filtered_edge_layer{tile, controller};
          filter_layer(kEdgeLayerName.data(), full_layer, filtered_edge_layer.layer,
                       options.tile_options().return_shortcuts());
        } else if (full_layer.name().data() == kNodeLayerName.data()) {
          NodesLayerBuilder filtered_node_layer{tile, controller};
          filter_layer(kNodeLayerName.data(), full_layer, filtered_node_layer.layer, true);
        }
        return true;
      });

      return tile.serialize();
    }
    // if we're caching, we need the full attributes
    controller.set_all(true);
  }

  // get lat/lon bbox
  const auto bounds = tile_to_bbox(x, y, z);

  // query edges in bbox, omits opposing edges
  const auto edge_ids = candidate_query_.RangeQuery(bounds);

  // build the full layers if cache is allowed, else whatever is in the controller
  build_layers(reader, tile, bounds, edge_ids, min_zoom_road_class_, z,
               options.tile_options().return_shortcuts(), controller);

  std::string tile_bytes;
  tile.serialize(tile_bytes);

  if (cache_allowed && !is_cached) {
    // atomically create the file
    auto tmp = tile_path;
    tmp += make_temp_name("_XXXXXX.tmp");
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

  if (options.verbose()) {
    return tile_bytes;
  } else if (cache_allowed) {
    // only apply filter to the tile if we have a full tile (due to caching) but the request
    // wants a filtered tile

    // need a fresh controller, the other one might have been changed if it was cacheable
    baldr::AttributesController fresh_controller{options, true};

    vtzero::vector_tile tile_full{tile_bytes};
    vtzero::tile_builder filtered_tile;

    tile_full.for_each_layer([&](vtzero::layer&& full_layer) {
      if (full_layer.name().data() == kEdgeLayerName.data()) {
        EdgesLayerBuilder filtered_edge_layer{filtered_tile, fresh_controller};
        filter_layer(kEdgeLayerName.data(), full_layer, filtered_edge_layer.layer,
                     options.tile_options().return_shortcuts());
      } else if (full_layer.name().data() == kNodeLayerName.data()) {
        NodesLayerBuilder filtered_node_layer{filtered_tile, fresh_controller};
        filter_layer(kNodeLayerName.data(), full_layer, filtered_node_layer.layer, true);
      }
      return true;
    });

    return filtered_tile.serialize();
  }

  // we can only land here if cache isn't allowed, verbose=false and/or a filter is in the request
  return tile_bytes;
}
} // namespace loki
} // namespace valhalla
