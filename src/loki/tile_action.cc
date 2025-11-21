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
#include "valhalla/exceptions.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/multi_linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/property_tree/ptree.hpp>
#include <vtzero/builder.hpp>

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
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

struct EdgeId {
  const char* key_name;
  std::string_view attribute_flag;
  vtzero::index_value EdgesLayerBuilder::*key_member;

  using value_func_t = vtzero::encoded_property_value (*)(const uint32_t edge_id);
  value_func_t value_func;
};

/**
 * Helper class to build the edges layer with pre-registered keys
 */
class EdgesLayerBuilder {
public:
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
        this->*(def.key_member) = layer_.add_key_without_dup_check(def.key_name);
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

  vtzero::layer_builder layer_;
  const baldr::AttributesController controller_;

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
          return vtzero::encoded_property_value(e.indoor());
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
    : layer_(tile, "edges"), controller_(controller) {
  key_tile_level_ = layer_.add_key_without_dup_check("tile_level");
  key_tile_id_ = layer_.add_key_without_dup_check("tile_id");
  key_road_class_ = layer_.add_key_without_dup_check("road_class");

  init_attribute_keys(kSharedEdgeAttributes, controller);
  init_attribute_keys(kForwardEdgeAttributes, controller);
  init_attribute_keys(kForwardLiveSpeedAttributes, controller);
  init_attribute_keys(kReverseEdgeAttributes, controller);
  init_attribute_keys(kReverseLiveSpeedAttributes, controller);

  // edge ids don't need all those attributes
  if (controller(kEdgeId)) {
    key_edge_id_fwd_ = layer_.add_key_without_dup_check("edge_id:forward");
    key_edge_id_rev_ = layer_.add_key_without_dup_check("edge_id:backward");
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
  vtzero::linestring_feature_builder feature{layer_};
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

  if (reverse_edge && reverse_edge_id.Is_Valid()) {
    if (controller_(kEdgeId))
      feature.add_property(key_edge_id_rev_, vtzero::encoded_property_value(reverse_edge_id.id()));
    set_attribute_values(kReverseEdgeAttributes, controller_, feature, *reverse_edge, edge_info,
                         nullptr);
    if (forward_traffic) {
      for (const auto& def : kReverseLiveSpeedAttributes) {
        set_attribute_values(kReverseLiveSpeedAttributes, controller_, feature, *reverse_edge,
                             edge_info, reverse_traffic);
      }
    }
  }

  feature.commit();
}

/**
 * Helper class to build the nodes layer with pre-registered keys
 */
class NodesLayerBuilder {
public:
  NodesLayerBuilder(vtzero::tile_builder& tile, const baldr::AttributesController& controller)
      : layer_(tile, "nodes"), controller_(controller) {
    // Pre-add keys for node properties
    key_tile_level_ = layer_.add_key_without_dup_check("tile_level");
    key_tile_id_ = layer_.add_key_without_dup_check("tile_id");
    key_node_id_ = layer_.add_key_without_dup_check("node_id");
    key_node_type_ = layer_.add_key_without_dup_check("type");
    key_traffic_signal_ = layer_.add_key_without_dup_check("traffic_signal");
    // Individual access mode keys
    key_access_auto_ = layer_.add_key_without_dup_check("access:auto");
    key_access_pedestrian_ = layer_.add_key_without_dup_check("access:pedestrian");
    key_access_bicycle_ = layer_.add_key_without_dup_check("access:bicycle");
    key_access_truck_ = layer_.add_key_without_dup_check("access:truck");
    key_access_emergency_ = layer_.add_key_without_dup_check("access:emergency");
    key_access_taxi_ = layer_.add_key_without_dup_check("access:taxi");
    key_access_bus_ = layer_.add_key_without_dup_check("access:bus");
    key_access_hov_ = layer_.add_key_without_dup_check("access:hov");
    key_access_wheelchair_ = layer_.add_key_without_dup_check("access:wheelchair");
    key_access_moped_ = layer_.add_key_without_dup_check("access:moped");
    key_access_motorcycle_ = layer_.add_key_without_dup_check("access:motorcycle");
    key_edge_count_ = layer_.add_key_without_dup_check("edge_count");
    key_intersection_ = layer_.add_key_without_dup_check("intersection");
    key_density_ = layer_.add_key_without_dup_check("density");
    key_local_edge_count_ = layer_.add_key_without_dup_check("local_edge_count");
    key_drive_on_right_ = layer_.add_key_without_dup_check("drive_on_right");
    key_elevation_ = layer_.add_key_without_dup_check("elevation");
    key_tagged_access_ = layer_.add_key_without_dup_check("tagged_access");
    key_private_access_ = layer_.add_key_without_dup_check("private_access");
    key_cash_only_toll_ = layer_.add_key_without_dup_check("cash_only_toll");
    key_mode_change_ = layer_.add_key_without_dup_check("mode_change");
    key_named_intersection_ = layer_.add_key_without_dup_check("named_intersection");
    key_is_transit_ = layer_.add_key_without_dup_check("is_transit");
    key_transition_count_ = layer_.add_key_without_dup_check("transition_count");
    key_timezone_ = layer_.add_key_without_dup_check("timezone");
    key_iso_3166_1_ = layer_.add_key_without_dup_check("iso_3166_1");
    key_iso_3166_2_ = layer_.add_key_without_dup_check("iso_3166_2");
  }

  void add_feature(const vtzero::point& position,
                   baldr::GraphId node_id,
                   const baldr::NodeInfo& node,
                   const baldr::AdminInfo& admin_info) {
    // Create point feature
    vtzero::point_feature_builder node_feature{layer_};
    node_feature.set_id(static_cast<uint64_t>(node_id));
    node_feature.add_point(position);

    // Add tile properties (same structure as edges layer)
    node_feature.add_property(key_tile_level_, vtzero::encoded_property_value(node_id.level()));
    node_feature.add_property(key_tile_id_, vtzero::encoded_property_value(node_id.tileid()));
    node_feature.add_property(key_node_id_, vtzero::encoded_property_value(node_id.id()));

    // Add node properties
    node_feature.add_property(key_node_type_,
                              vtzero::encoded_property_value(static_cast<uint32_t>(node.type())));
    node_feature.add_property(key_traffic_signal_,
                              vtzero::encoded_property_value(node.traffic_signal()));

    // Add individual access mode properties
    uint16_t access = node.access();
    node_feature.add_property(key_access_auto_, vtzero::encoded_property_value(
                                                    static_cast<bool>(access & kAutoAccess)));
    node_feature.add_property(key_access_pedestrian_,
                              vtzero::encoded_property_value(
                                  static_cast<bool>(access & kPedestrianAccess)));
    node_feature.add_property(key_access_bicycle_, vtzero::encoded_property_value(
                                                       static_cast<bool>(access & kBicycleAccess)));
    node_feature.add_property(key_access_truck_, vtzero::encoded_property_value(
                                                     static_cast<bool>(access & kTruckAccess)));
    node_feature.add_property(key_access_emergency_, vtzero::encoded_property_value(static_cast<bool>(
                                                         access & kEmergencyAccess)));
    node_feature.add_property(key_access_taxi_, vtzero::encoded_property_value(
                                                    static_cast<bool>(access & kTaxiAccess)));
    node_feature.add_property(key_access_bus_,
                              vtzero::encoded_property_value(static_cast<bool>(access & kBusAccess)));
    node_feature.add_property(key_access_hov_,
                              vtzero::encoded_property_value(static_cast<bool>(access & kHOVAccess)));
    node_feature.add_property(key_access_wheelchair_,
                              vtzero::encoded_property_value(
                                  static_cast<bool>(access & kWheelchairAccess)));
    node_feature.add_property(key_access_moped_, vtzero::encoded_property_value(
                                                     static_cast<bool>(access & kMopedAccess)));
    node_feature.add_property(key_access_motorcycle_,
                              vtzero::encoded_property_value(
                                  static_cast<bool>(access & kMotorcycleAccess)));

    node_feature.add_property(key_edge_count_, vtzero::encoded_property_value(node.edge_count()));
    node_feature.add_property(key_intersection_, vtzero::encoded_property_value(
                                                     static_cast<uint32_t>(node.intersection())));
    node_feature.add_property(key_density_, vtzero::encoded_property_value(node.density()));
    node_feature.add_property(key_local_edge_count_,
                              vtzero::encoded_property_value(node.local_edge_count()));
    node_feature.add_property(key_drive_on_right_,
                              vtzero::encoded_property_value(node.drive_on_right()));
    node_feature.add_property(key_elevation_, vtzero::encoded_property_value(node.elevation()));
    node_feature.add_property(key_tagged_access_,
                              vtzero::encoded_property_value(node.tagged_access()));
    node_feature.add_property(key_private_access_,
                              vtzero::encoded_property_value(node.private_access()));
    node_feature.add_property(key_cash_only_toll_,
                              vtzero::encoded_property_value(node.cash_only_toll()));
    node_feature.add_property(key_mode_change_, vtzero::encoded_property_value(node.mode_change()));
    node_feature.add_property(key_is_transit_, vtzero::encoded_property_value(node.is_transit()));
    node_feature.add_property(key_transition_count_,
                              vtzero::encoded_property_value(node.transition_count()));
    node_feature.add_property(key_iso_3166_1_,
                              vtzero::encoded_property_value(admin_info.country_iso()));
    node_feature.add_property(key_iso_3166_2_,
                              vtzero::encoded_property_value(admin_info.state_iso()));

    if (node.timezone()) {
      node_feature.add_property(key_timezone_,
                                vtzero::encoded_property_value(
                                    DateTime::get_tz_db().from_index(node.timezone())->name()));
    }

    node_feature.commit();
  }

private:
  vtzero::layer_builder layer_;
  const baldr::AttributesController controller_;

  // Pre-registered keys
  vtzero::index_value key_tile_level_;
  vtzero::index_value key_tile_id_;
  vtzero::index_value key_node_id_;
  vtzero::index_value key_node_type_;
  vtzero::index_value key_traffic_signal_;
  // Individual access mode keys
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
  vtzero::index_value key_edge_count_;
  vtzero::index_value key_intersection_;
  vtzero::index_value key_density_;
  vtzero::index_value key_local_edge_count_;
  vtzero::index_value key_drive_on_right_;
  vtzero::index_value key_elevation_;
  vtzero::index_value key_tagged_access_;
  vtzero::index_value key_private_access_;
  vtzero::index_value key_cash_only_toll_;
  vtzero::index_value key_mode_change_;
  vtzero::index_value key_named_intersection_;
  vtzero::index_value key_is_transit_;
  vtzero::index_value key_transition_count_;
  vtzero::index_value key_timezone_;
  vtzero::index_value key_iso_3166_1_;
  vtzero::index_value key_iso_3166_2_;
};

double lon_to_merc_x(const double lon) {
  return kEarthRadiusMeters * lon * kPiD / 180.0;
}

double lat_to_merc_y(const double lat) {
  return kEarthRadiusMeters * std::log(std::tan(kPiD / 4.0 + lat * kPiD / 360.0));
}

midgard::AABB2<midgard::PointLL> tile_to_bbox(const valhalla::Tile& xyz) {
  const auto x = xyz.x();
  const auto y = xyz.y();
  const auto z = xyz.z();

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
  // TODO: sort edge_ids
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
      if (line.size() < 2) {
        return;
      }

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

std::string loki_worker_t::render_tile(Api& request) {
  const auto& options = request.options();

  vtzero::tile_builder tile;
  const auto z = options.tile_xyz().z();
  if (z < min_zoom_) {
    return tile.serialize();
  }

  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  // get lat/lon bbox
  const auto bounds = tile_to_bbox(options.tile_xyz());

  // query edges in bbox, omits opposing edges
  // TODO(nils): can RangeQuery be updated to skip hierarchy levels?
  // How about filtering for costing? Probably better client filtering?
  const auto edge_ids = candidate_query_.RangeQuery(bounds);

  // disable all attributes by default
  auto controller = baldr::AttributesController(options, true);
  if (options.filter_action() == valhalla::no_action && !options.verbose())
    controller.set_all(false);
  else if (options.verbose()) {
    controller.set_all(true);
  }

  build_layers(reader, tile, bounds, edge_ids, min_zoom_road_class_, z,
               options.tile_options().return_shortcuts(), controller);

  return tile.serialize();
}

} // namespace loki
} // namespace valhalla
