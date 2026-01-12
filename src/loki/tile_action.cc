#include "baldr/datetime.h"
#include "baldr/directededge.h"
#include "baldr/graphreader.h"
#include "baldr/nodeinfo.h"
#include "baldr/tilehierarchy.h"
#include "loki/worker.h"
#include "meili/candidate_search.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "midgard/polyline2.h"
#include "utils.h"
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
#include <filesystem>
#include <fstream>
#include <random>
#include <unordered_set>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace {

// approx tolerance in meters at equator
constexpr double PeuckerEpsilons[] = {626'172, // z0
                                      313'086, 156'543, 78'271,
                                      39'135, // z4
                                      19'567,  4'891,   2'445,  1'222,
                                      611, // z9
                                      152,     76,      38,     19,
                                      5, // z14
                                      2,       1};

/**
 * Earth radius in meters for EPSG:3857 Web Mercator projection.
 * This is the WGS84 ellipsoid semi-major axis.
 */
constexpr double kEarthRadiusMeters = 6378137.0;

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
  EdgesLayerBuilder(vtzero::tile_builder& tile) : layer_(tile, "edges") {
    // Pre-add keys for edge properties
    key_tile_level_ = layer_.add_key_without_dup_check("tile_level");
    key_tile_id_ = layer_.add_key_without_dup_check("tile_id");
    // Shared edge properties
    key_speed_limit_ = layer_.add_key_without_dup_check("speed_limit");
    key_road_class_ = layer_.add_key_without_dup_check("road_class");
    key_use_ = layer_.add_key_without_dup_check("use");
    key_tunnel_ = layer_.add_key_without_dup_check("tunnel");
    key_bridge_ = layer_.add_key_without_dup_check("bridge");
    key_roundabout_ = layer_.add_key_without_dup_check("roundabout");
    key_is_shortcut_ = layer_.add_key_without_dup_check("is_shortcut");
    key_leaves_tile_ = layer_.add_key_without_dup_check("leaves_tile");
    key_length_ = layer_.add_key_without_dup_check("length");
    key_weighted_grade_ = layer_.add_key_without_dup_check("weighted_grade");
    key_max_up_slope_ = layer_.add_key_without_dup_check("max_up_slope");
    key_max_down_slope_ = layer_.add_key_without_dup_check("max_down_slope");
    key_curvature_ = layer_.add_key_without_dup_check("curvature");
    key_toll_ = layer_.add_key_without_dup_check("toll");
    key_destonly_ = layer_.add_key_without_dup_check("destonly");
    key_destonly_hgv_ = layer_.add_key_without_dup_check("destonly_hgv");
    key_indoor_ = layer_.add_key_without_dup_check("indoor");
    key_hov_type_ = layer_.add_key_without_dup_check("hov_type");
    key_cyclelane_ = layer_.add_key_without_dup_check("cyclelane");
    key_bike_network_ = layer_.add_key_without_dup_check("bike_network");
    key_truck_route_ = layer_.add_key_without_dup_check("truck_route");
    key_speed_type_ = layer_.add_key_without_dup_check("speed_type");
    key_ctry_crossing_ = layer_.add_key_without_dup_check("ctry_crossing");
    key_sac_scale_ = layer_.add_key_without_dup_check("sac_scale");
    key_unpaved_ = layer_.add_key_without_dup_check("unpaved");
    key_surface_ = layer_.add_key_without_dup_check("surface");
    key_link_ = layer_.add_key_without_dup_check("link");
    key_internal_ = layer_.add_key_without_dup_check("internal");
    key_shoulder_ = layer_.add_key_without_dup_check("shoulder");
    key_dismount_ = layer_.add_key_without_dup_check("dismount");
    key_use_sidepath_ = layer_.add_key_without_dup_check("use_sidepath");
    key_density_ = layer_.add_key_without_dup_check("density");
    key_named_ = layer_.add_key_without_dup_check("named");
    key_sidewalk_left_ = layer_.add_key_without_dup_check("sidewalk_left");
    key_sidewalk_right_ = layer_.add_key_without_dup_check("sidewalk_right");
    key_bss_connection_ = layer_.add_key_without_dup_check("bss_connection");
    key_lit_ = layer_.add_key_without_dup_check("lit");
    key_not_thru_ = layer_.add_key_without_dup_check("not_thru");
    key_part_of_complex_restriction_ =
        layer_.add_key_without_dup_check("part_of_complex_restriction");
    key_osm_way_id_ = layer_.add_key_without_dup_check("osm_way_id");
    key_layer_ = layer_.add_key_without_dup_check("layer");
    // Direction-specific properties
    key_edge_id_fwd_ = layer_.add_key_without_dup_check("edge_id:forward");
    key_edge_id_rev_ = layer_.add_key_without_dup_check("edge_id:backward");
    key_speed_fwd_ = layer_.add_key_without_dup_check("speed:forward");
    key_speed_rev_ = layer_.add_key_without_dup_check("speed:backward");
    key_deadend_fwd_ = layer_.add_key_without_dup_check("deadend:forward");
    key_deadend_rev_ = layer_.add_key_without_dup_check("deadend:backward");
    key_lanecount_fwd_ = layer_.add_key_without_dup_check("lanecount:forward");
    key_lanecount_rev_ = layer_.add_key_without_dup_check("lanecount:backward");
    key_truck_speed_fwd_ = layer_.add_key_without_dup_check("truck_speed:forward");
    key_truck_speed_rev_ = layer_.add_key_without_dup_check("truck_speed:backward");
    key_traffic_signal_fwd_ = layer_.add_key_without_dup_check("traffic_signal:forward");
    key_traffic_signal_rev_ = layer_.add_key_without_dup_check("traffic_signal:backward");
    key_stop_sign_fwd_ = layer_.add_key_without_dup_check("stop_sign:forward");
    key_stop_sign_rev_ = layer_.add_key_without_dup_check("stop_sign:backward");
    key_yield_sign_fwd_ = layer_.add_key_without_dup_check("yield_sign:forward");
    key_yield_sign_rev_ = layer_.add_key_without_dup_check("yield_sign:backward");
    // Access properties (forward)
    key_access_auto_fwd_ = layer_.add_key_without_dup_check("access:auto:forward");
    key_access_pedestrian_fwd_ = layer_.add_key_without_dup_check("access:pedestrian:forward");
    key_access_bicycle_fwd_ = layer_.add_key_without_dup_check("access:bicycle:forward");
    key_access_truck_fwd_ = layer_.add_key_without_dup_check("access:truck:forward");
    key_access_emergency_fwd_ = layer_.add_key_without_dup_check("access:emergency:forward");
    key_access_taxi_fwd_ = layer_.add_key_without_dup_check("access:taxi:forward");
    key_access_bus_fwd_ = layer_.add_key_without_dup_check("access:bus:forward");
    key_access_hov_fwd_ = layer_.add_key_without_dup_check("access:hov:forward");
    key_access_wheelchair_fwd_ = layer_.add_key_without_dup_check("access:wheelchair:forward");
    key_access_moped_fwd_ = layer_.add_key_without_dup_check("access:moped:forward");
    key_access_motorcycle_fwd_ = layer_.add_key_without_dup_check("access:motorcycle:forward");
    // Access properties (reverse)
    key_access_auto_rev_ = layer_.add_key_without_dup_check("access:auto:backward");
    key_access_pedestrian_rev_ = layer_.add_key_without_dup_check("access:pedestrian:backward");
    key_access_bicycle_rev_ = layer_.add_key_without_dup_check("access:bicycle:backward");
    key_access_truck_rev_ = layer_.add_key_without_dup_check("access:truck:backward");
    key_access_emergency_rev_ = layer_.add_key_without_dup_check("access:emergency:backward");
    key_access_taxi_rev_ = layer_.add_key_without_dup_check("access:taxi:backward");
    key_access_bus_rev_ = layer_.add_key_without_dup_check("access:bus:backward");
    key_access_hov_rev_ = layer_.add_key_without_dup_check("access:hov:backward");
    key_access_wheelchair_rev_ = layer_.add_key_without_dup_check("access:wheelchair:backward");
    key_access_moped_rev_ = layer_.add_key_without_dup_check("access:moped:backward");
    key_access_motorcycle_rev_ = layer_.add_key_without_dup_check("access:motorcycle:backward");
    // Traffic speed properties (forward)
    key_live_speed_fwd_ = layer_.add_key_without_dup_check("live_speed:forward");
    key_live_speed1_fwd_ = layer_.add_key_without_dup_check("live_speed:forward:speed1");
    key_live_speed2_fwd_ = layer_.add_key_without_dup_check("live_speed:forward:speed2");
    key_live_speed3_fwd_ = layer_.add_key_without_dup_check("live_speed:forward:speed3");
    key_live_breakpoint1_fwd_ = layer_.add_key_without_dup_check("live_speed:forward:breakpoint1");
    key_live_breakpoint2_fwd_ = layer_.add_key_without_dup_check("live_speed:forward:breakpoint2");
    key_live_congestion1_fwd_ = layer_.add_key_without_dup_check("live_speed:forward:congestion1");
    key_live_congestion2_fwd_ = layer_.add_key_without_dup_check("live_speed:forward:congestion2");
    key_live_congestion3_fwd_ = layer_.add_key_without_dup_check("live_speed:forward:congestion3");
    // Traffic speed properties (reverse)
    key_live_speed_rev_ = layer_.add_key_without_dup_check("live_speed:backward");
    key_live_speed1_rev_ = layer_.add_key_without_dup_check("live_speed:backward:speed1");
    key_live_speed2_rev_ = layer_.add_key_without_dup_check("live_speed:backward:speed2");
    key_live_speed3_rev_ = layer_.add_key_without_dup_check("live_speed:backward:speed3");
    key_live_breakpoint1_rev_ = layer_.add_key_without_dup_check("live_speed:backward:breakpoint1");
    key_live_breakpoint2_rev_ = layer_.add_key_without_dup_check("live_speed:backward:breakpoint2");
    key_live_congestion1_rev_ = layer_.add_key_without_dup_check("live_speed:backward:congestion1");
    key_live_congestion2_rev_ = layer_.add_key_without_dup_check("live_speed:backward:congestion2");
    key_live_congestion3_rev_ = layer_.add_key_without_dup_check("live_speed:backward:congestion3");
  }

  void add_feature(const std::vector<vtzero::point>& geometry,
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
    vtzero::linestring_feature_builder feature{layer_};
    feature.set_id(static_cast<uint64_t>(edge_id));
    feature.add_linestring_from_container(geometry);

    // Add shared tile properties (same for both directions)
    feature.add_property(key_tile_level_, vtzero::encoded_property_value(edge_id.level()));
    feature.add_property(key_tile_id_, vtzero::encoded_property_value(edge_id.tileid()));

    // Add shared edge properties (same for both directions)
    feature.add_property(key_road_class_, vtzero::encoded_property_value(
                                              static_cast<uint32_t>(edge->classification())));
    feature.add_property(key_use_,
                         vtzero::encoded_property_value(static_cast<uint32_t>(edge->use())));
    feature.add_property(key_tunnel_, vtzero::encoded_property_value(edge->tunnel()));
    feature.add_property(key_bridge_, vtzero::encoded_property_value(edge->bridge()));
    feature.add_property(key_roundabout_, vtzero::encoded_property_value(edge->roundabout()));
    feature.add_property(key_is_shortcut_, vtzero::encoded_property_value(edge->is_shortcut()));
    feature.add_property(key_leaves_tile_, vtzero::encoded_property_value(edge->leaves_tile()));
    feature.add_property(key_length_, vtzero::encoded_property_value(edge->length()));
    feature.add_property(key_weighted_grade_, vtzero::encoded_property_value(edge->weighted_grade()));
    feature.add_property(key_max_up_slope_, vtzero::encoded_property_value(edge->max_up_slope()));
    feature.add_property(key_max_down_slope_, vtzero::encoded_property_value(edge->max_down_slope()));
    feature.add_property(key_curvature_, vtzero::encoded_property_value(edge->curvature()));
    feature.add_property(key_toll_, vtzero::encoded_property_value(edge->toll()));
    feature.add_property(key_destonly_, vtzero::encoded_property_value(edge->destonly()));
    feature.add_property(key_destonly_hgv_, vtzero::encoded_property_value(edge->destonly_hgv()));
    feature.add_property(key_indoor_, vtzero::encoded_property_value(edge->indoor()));
    feature.add_property(key_hov_type_,
                         vtzero::encoded_property_value(static_cast<uint32_t>(edge->hov_type())));
    feature.add_property(key_cyclelane_,
                         vtzero::encoded_property_value(static_cast<uint32_t>(edge->cyclelane())));
    feature.add_property(key_bike_network_, vtzero::encoded_property_value(edge->bike_network()));
    feature.add_property(key_truck_route_, vtzero::encoded_property_value(edge->truck_route()));
    feature.add_property(key_speed_type_,
                         vtzero::encoded_property_value(static_cast<uint32_t>(edge->speed_type())));
    feature.add_property(key_ctry_crossing_, vtzero::encoded_property_value(edge->ctry_crossing()));
    feature.add_property(key_sac_scale_,
                         vtzero::encoded_property_value(static_cast<uint32_t>(edge->sac_scale())));
    feature.add_property(key_unpaved_, vtzero::encoded_property_value(edge->unpaved()));
    feature.add_property(key_surface_,
                         vtzero::encoded_property_value(static_cast<uint32_t>(edge->surface())));
    feature.add_property(key_link_, vtzero::encoded_property_value(edge->link()));
    feature.add_property(key_internal_, vtzero::encoded_property_value(edge->internal()));
    feature.add_property(key_shoulder_, vtzero::encoded_property_value(edge->shoulder()));
    feature.add_property(key_dismount_, vtzero::encoded_property_value(edge->dismount()));
    feature.add_property(key_use_sidepath_, vtzero::encoded_property_value(edge->use_sidepath()));
    feature.add_property(key_density_, vtzero::encoded_property_value(edge->density()));
    feature.add_property(key_named_, vtzero::encoded_property_value(edge->named()));
    feature.add_property(key_sidewalk_left_, vtzero::encoded_property_value(edge->sidewalk_left()));
    feature.add_property(key_sidewalk_right_, vtzero::encoded_property_value(edge->sidewalk_right()));
    feature.add_property(key_bss_connection_, vtzero::encoded_property_value(edge->bss_connection()));
    feature.add_property(key_lit_, vtzero::encoded_property_value(edge->lit()));
    feature.add_property(key_not_thru_, vtzero::encoded_property_value(edge->not_thru()));
    feature.add_property(key_part_of_complex_restriction_,
                         vtzero::encoded_property_value(edge->part_of_complex_restriction()));
    feature.add_property(key_osm_way_id_, vtzero::encoded_property_value(edge_info.wayid()));
    feature.add_property(key_speed_limit_, vtzero::encoded_property_value(edge_info.speed_limit()));
    feature.add_property(key_layer_, vtzero::encoded_property_value(edge_info.layer()));

    // Add direction-specific properties
    if (forward_edge) {
      feature.add_property(key_edge_id_fwd_, vtzero::encoded_property_value(forward_edge_id.id()));
      feature.add_property(key_speed_fwd_, vtzero::encoded_property_value(forward_edge->speed()));
      feature.add_property(key_truck_speed_fwd_,
                           vtzero::encoded_property_value(forward_edge->truck_speed()));
      feature.add_property(key_traffic_signal_fwd_,
                           vtzero::encoded_property_value(forward_edge->traffic_signal()));
      feature.add_property(key_stop_sign_fwd_,
                           vtzero::encoded_property_value(forward_edge->stop_sign()));
      feature.add_property(key_yield_sign_fwd_,
                           vtzero::encoded_property_value(forward_edge->yield_sign()));
      feature.add_property(key_deadend_fwd_, vtzero::encoded_property_value(edge->deadend()));
      feature.add_property(key_lanecount_fwd_, vtzero::encoded_property_value(edge->deadend()));

      // Forward access properties
      uint32_t fwd_access = forward_edge->forwardaccess();
      feature.add_property(key_access_auto_fwd_, vtzero::encoded_property_value(
                                                     static_cast<bool>(fwd_access & kAutoAccess)));
      feature.add_property(key_access_pedestrian_fwd_,
                           vtzero::encoded_property_value(
                               static_cast<bool>(fwd_access & kPedestrianAccess)));
      feature.add_property(key_access_bicycle_fwd_, vtzero::encoded_property_value(static_cast<bool>(
                                                        fwd_access & kBicycleAccess)));
      feature.add_property(key_access_truck_fwd_, vtzero::encoded_property_value(
                                                      static_cast<bool>(fwd_access & kTruckAccess)));
      feature.add_property(key_access_emergency_fwd_,
                           vtzero::encoded_property_value(
                               static_cast<bool>(fwd_access & kEmergencyAccess)));
      feature.add_property(key_access_taxi_fwd_, vtzero::encoded_property_value(
                                                     static_cast<bool>(fwd_access & kTaxiAccess)));
      feature.add_property(key_access_bus_fwd_, vtzero::encoded_property_value(
                                                    static_cast<bool>(fwd_access & kBusAccess)));
      feature.add_property(key_access_hov_fwd_, vtzero::encoded_property_value(
                                                    static_cast<bool>(fwd_access & kHOVAccess)));
      feature.add_property(key_access_wheelchair_fwd_,
                           vtzero::encoded_property_value(
                               static_cast<bool>(fwd_access & kWheelchairAccess)));
      feature.add_property(key_access_moped_fwd_, vtzero::encoded_property_value(
                                                      static_cast<bool>(fwd_access & kMopedAccess)));
      feature.add_property(key_access_motorcycle_fwd_,
                           vtzero::encoded_property_value(
                               static_cast<bool>(fwd_access & kMotorcycleAccess)));

      // Add live traffic data if available
      if (forward_traffic) {
        feature.add_property(key_live_speed_fwd_,
                             vtzero::encoded_property_value(forward_traffic->get_overall_speed()));
        feature.add_property(key_live_speed1_fwd_,
                             vtzero::encoded_property_value(forward_traffic->get_speed(0)));
        feature.add_property(key_live_speed2_fwd_,
                             vtzero::encoded_property_value(forward_traffic->get_speed(1)));
        feature.add_property(key_live_speed3_fwd_,
                             vtzero::encoded_property_value(forward_traffic->get_speed(2)));
        feature.add_property(key_live_breakpoint1_fwd_,
                             vtzero::encoded_property_value(
                                 static_cast<uint32_t>(forward_traffic->breakpoint1)));
        feature.add_property(key_live_breakpoint2_fwd_,
                             vtzero::encoded_property_value(
                                 static_cast<uint32_t>(forward_traffic->breakpoint2)));
        feature.add_property(key_live_congestion1_fwd_,
                             vtzero::encoded_property_value(
                                 static_cast<uint32_t>(forward_traffic->congestion1)));
        feature.add_property(key_live_congestion2_fwd_,
                             vtzero::encoded_property_value(
                                 static_cast<uint32_t>(forward_traffic->congestion2)));
        feature.add_property(key_live_congestion3_fwd_,
                             vtzero::encoded_property_value(
                                 static_cast<uint32_t>(forward_traffic->congestion3)));
      }
    }

    if (reverse_edge && reverse_edge_id.is_valid()) {
      feature.add_property(key_edge_id_rev_, vtzero::encoded_property_value(reverse_edge_id.id()));
      feature.add_property(key_speed_rev_, vtzero::encoded_property_value(reverse_edge->speed()));
      feature.add_property(key_truck_speed_rev_,
                           vtzero::encoded_property_value(reverse_edge->truck_speed()));
      feature.add_property(key_traffic_signal_rev_,
                           vtzero::encoded_property_value(reverse_edge->traffic_signal()));
      feature.add_property(key_stop_sign_rev_,
                           vtzero::encoded_property_value(reverse_edge->stop_sign()));
      feature.add_property(key_yield_sign_rev_,
                           vtzero::encoded_property_value(reverse_edge->yield_sign()));
      feature.add_property(key_deadend_rev_, vtzero::encoded_property_value(edge->deadend()));
      feature.add_property(key_lanecount_rev_, vtzero::encoded_property_value(edge->deadend()));

      // Reverse access properties
      uint32_t rev_access = reverse_edge->reverseaccess();
      feature.add_property(key_access_auto_rev_, vtzero::encoded_property_value(
                                                     static_cast<bool>(rev_access & kAutoAccess)));
      feature.add_property(key_access_pedestrian_rev_,
                           vtzero::encoded_property_value(
                               static_cast<bool>(rev_access & kPedestrianAccess)));
      feature.add_property(key_access_bicycle_rev_, vtzero::encoded_property_value(static_cast<bool>(
                                                        rev_access & kBicycleAccess)));
      feature.add_property(key_access_truck_rev_, vtzero::encoded_property_value(
                                                      static_cast<bool>(rev_access & kTruckAccess)));
      feature.add_property(key_access_emergency_rev_,
                           vtzero::encoded_property_value(
                               static_cast<bool>(rev_access & kEmergencyAccess)));
      feature.add_property(key_access_taxi_rev_, vtzero::encoded_property_value(
                                                     static_cast<bool>(rev_access & kTaxiAccess)));
      feature.add_property(key_access_bus_rev_, vtzero::encoded_property_value(
                                                    static_cast<bool>(rev_access & kBusAccess)));
      feature.add_property(key_access_hov_rev_, vtzero::encoded_property_value(
                                                    static_cast<bool>(rev_access & kHOVAccess)));
      feature.add_property(key_access_wheelchair_rev_,
                           vtzero::encoded_property_value(
                               static_cast<bool>(rev_access & kWheelchairAccess)));
      feature.add_property(key_access_moped_rev_, vtzero::encoded_property_value(
                                                      static_cast<bool>(rev_access & kMopedAccess)));
      feature.add_property(key_access_motorcycle_rev_,
                           vtzero::encoded_property_value(
                               static_cast<bool>(rev_access & kMotorcycleAccess)));

      // Add live traffic data if available
      if (reverse_traffic) {
        feature.add_property(key_live_speed_rev_,
                             vtzero::encoded_property_value(reverse_traffic->get_overall_speed()));
        feature.add_property(key_live_speed1_rev_,
                             vtzero::encoded_property_value(reverse_traffic->get_speed(0)));
        feature.add_property(key_live_speed2_rev_,
                             vtzero::encoded_property_value(reverse_traffic->get_speed(1)));
        feature.add_property(key_live_speed3_rev_,
                             vtzero::encoded_property_value(reverse_traffic->get_speed(2)));
        feature.add_property(key_live_breakpoint1_rev_,
                             vtzero::encoded_property_value(
                                 static_cast<uint32_t>(reverse_traffic->breakpoint1)));
        feature.add_property(key_live_breakpoint2_rev_,
                             vtzero::encoded_property_value(
                                 static_cast<uint32_t>(reverse_traffic->breakpoint2)));
        feature.add_property(key_live_congestion1_rev_,
                             vtzero::encoded_property_value(
                                 static_cast<uint32_t>(reverse_traffic->congestion1)));
        feature.add_property(key_live_congestion2_rev_,
                             vtzero::encoded_property_value(
                                 static_cast<uint32_t>(reverse_traffic->congestion2)));
        feature.add_property(key_live_congestion3_rev_,
                             vtzero::encoded_property_value(
                                 static_cast<uint32_t>(reverse_traffic->congestion3)));
      }
    }

    feature.commit();
  }

private:
  vtzero::layer_builder layer_;

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
  vtzero::index_value key_named_;
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

/**
 * Helper class to build the nodes layer with pre-registered keys
 */
class NodesLayerBuilder {
public:
  NodesLayerBuilder(vtzero::tile_builder& tile) : layer_(tile, "nodes") {
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
    node_feature.add_property(key_named_intersection_,
                              vtzero::encoded_property_value(node.named_intersection()));
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
  constexpr double f = kEarthRadiusMeters * kPiD / 180.0;
  return lon * f;
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
                  valhalla::Options options) {
  using point_t = boost::geometry::model::d2::point_xy<double>;
  using linestring_t = boost::geometry::model::linestring<point_t>;
  using multi_linestring_t = boost::geometry::model::multi_linestring<linestring_t>;
  using box_t = boost::geometry::model::box<point_t>;
  const bool return_shortcuts = options.tile_options().return_shortcuts();
  // we use generalize as a scaling factor to our default generalization, i.e. [0.f, 1.f] range
  const float generalize = options.has_generalize_case() ? std::min(options.generalize(), 1.f) : 1.f;
  const TileProjection projection{bounds};

  // create clip box with buffer to handle edges that cross boundaries
  const box_t clip_box(point_t(-projection.tile_buffer, -projection.tile_buffer),
                       point_t(projection.tile_extent + projection.tile_buffer,
                               projection.tile_extent + projection.tile_buffer));

  EdgesLayerBuilder edges_builder(tile);
  NodesLayerBuilder nodes_builder(tile);

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

    // scale the epsilon with generalize query parameter
    if (const auto gen_factor = PeuckerEpsilons[z] * static_cast<double>(generalize); gen_factor > 1.)
      Polyline2<PointLL>::Generalize(shape, gen_factor);

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
      return std::string(std::istreambuf_iterator<char>(tile_file), std::istreambuf_iterator<char>());
    }
  }

  // get lat/lon bbox
  const auto bounds = tile_to_bbox(x, y, z);

  // query edges in bbox, omits opposing edges
  const auto edge_ids = candidate_query_.RangeQuery(bounds);

  build_layers(reader, tile, bounds, edge_ids, min_zoom_road_class_, z, options);

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

  return tile_bytes;
}
} // namespace loki
} // namespace valhalla
