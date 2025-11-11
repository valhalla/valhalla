#include "heimdall/worker.h"
#include "baldr/directededge.h"
#include "baldr/graphreader.h"
#include "baldr/nodeinfo.h"
#include "baldr/tilehierarchy.h"
#include "heimdall/util.h"
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
#include <array>
#include <climits>
#include <cmath>
#include <unordered_set>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace heimdall {

namespace {

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

    if (reverse_edge && reverse_edge_id.Is_Valid()) {
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

    // Add tile properties (same structure as edges layer)
    node_feature.add_property(key_tile_level_, vtzero::encoded_property_value(node_id.level()));
    node_feature.add_property(key_tile_id_, vtzero::encoded_property_value(node_id.tileid()));
    node_feature.add_property(key_node_id_, vtzero::encoded_property_value(node_id.id()));

    // Add node properties
    node_feature.add_property(key_node_type_,
                              vtzero::encoded_property_value(static_cast<uint32_t>(node->type())));
    node_feature.add_property(key_traffic_signal_,
                              vtzero::encoded_property_value(node->traffic_signal()));

    // Add individual access mode properties
    uint16_t access = node->access();
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

    node_feature.add_property(key_edge_count_, vtzero::encoded_property_value(node->edge_count()));
    node_feature.add_property(key_intersection_, vtzero::encoded_property_value(
                                                     static_cast<uint32_t>(node->intersection())));
    node_feature.add_property(key_density_, vtzero::encoded_property_value(node->density()));
    node_feature.add_property(key_local_edge_count_,
                              vtzero::encoded_property_value(node->local_edge_count()));
    node_feature.add_property(key_drive_on_right_,
                              vtzero::encoded_property_value(node->drive_on_right()));
    node_feature.add_property(key_elevation_, vtzero::encoded_property_value(node->elevation()));
    node_feature.add_property(key_tagged_access_,
                              vtzero::encoded_property_value(node->tagged_access()));
    node_feature.add_property(key_private_access_,
                              vtzero::encoded_property_value(node->private_access()));
    node_feature.add_property(key_cash_only_toll_,
                              vtzero::encoded_property_value(node->cash_only_toll()));
    node_feature.add_property(key_mode_change_, vtzero::encoded_property_value(node->mode_change()));
    node_feature.add_property(key_named_intersection_,
                              vtzero::encoded_property_value(node->named_intersection()));
    node_feature.add_property(key_is_transit_, vtzero::encoded_property_value(node->is_transit()));
    node_feature.add_property(key_transition_count_,
                              vtzero::encoded_property_value(node->transition_count()));

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
};

} // anonymous namespace

void heimdall_worker_t::ReadZoomConfig(const boost::property_tree::ptree& config) {
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

heimdall_worker_t::heimdall_worker_t(const boost::property_tree::ptree& config,
                                     const std::shared_ptr<baldr::GraphReader>& graph_reader)
    : config_(config), reader_(graph_reader),
      candidate_query_(*graph_reader,
                       TileHierarchy::levels().back().tiles.TileSize() / 10.0f,
                       TileHierarchy::levels().back().tiles.TileSize() / 10.0f) {
  ReadZoomConfig(config_);
}

std::unordered_set<GraphId>
heimdall_worker_t::build_edges_layer(vtzero::tile_builder& tile,
                                     const midgard::AABB2<midgard::PointLL>& bounds,
                                     const std::unordered_set<baldr::GraphId>& edge_ids,
                                     uint32_t z,
                                     const TileProjection& projection,
                                     bool return_shortcuts) {
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

    // Filter out shortcut edges if return_shortcuts is false
    if (!return_shortcuts && edge->is_shortcut()) {
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

      const volatile baldr::TrafficSpeed* forward_traffic =
          edge ? &edge_tile->trafficspeed(edge) : nullptr;
      const volatile baldr::TrafficSpeed* reverse_traffic =
          opp_edge ? &opp_tile->trafficspeed(opp_edge) : nullptr;

      edges_builder.add_feature(tile_coords, edge_id, edge, opp_edge_id, opp_edge, forward_traffic,
                                reverse_traffic, edge_info);

      // Collect the end node of this edge for the nodes layer
      unique_nodes.insert(edge->endnode());
    }
  }

  return unique_nodes;
}

void heimdall_worker_t::build_nodes_layer(vtzero::tile_builder& tile,
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

std::string
heimdall_worker_t::render_tile(uint32_t z, uint32_t x, uint32_t y, bool return_shortcuts) {
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
  auto unique_nodes = build_edges_layer(tile, bounds, edge_ids, z, projection, return_shortcuts);

  // Build nodes layer
  build_nodes_layer(tile, unique_nodes, projection);

  return tile.serialize();
}

} // namespace heimdall
} // namespace valhalla
