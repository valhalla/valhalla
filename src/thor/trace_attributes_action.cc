#include <cstdint>
#include <algorithm>
#include <tuple>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "baldr/json.h"
#include "baldr/graphconstants.h"
#include "baldr/directededge.h"
#include "baldr/edge_elevation.h"
#include "midgard/logging.h"
#include "midgard/constants.h"
#include "odin/util.h"
#include "odin/enhancedtrippath.h"
#include "proto/tripdirections.pb.h"
#include "proto/trippath.pb.h"
#include "exception.h"
#include "thor/attributes_controller.h"
#include "thor/match_result.h"
#include "thor/worker.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;
using namespace valhalla::thor;

namespace {

  json::ArrayPtr serialize_admins(const TripPath& trip_path) {
    auto admin_array = json::array({});
    for (const auto& admin : trip_path.admin()) {
      auto admin_map = json::map({});
      if (admin.has_country_code())
        admin_map->emplace("country_code", admin.country_code());
      if (admin.has_country_text())
        admin_map->emplace("country_text", admin.country_text());
      if (admin.has_state_code())
        admin_map->emplace("state_code", admin.state_code());
      if (admin.has_state_text())
        admin_map->emplace("state_text", admin.state_text());

      admin_array->push_back(admin_map);
    }

    return admin_array;
  }

  json::ArrayPtr serialize_edges(const AttributesController& controller,
      const DirectionsOptions& directions_options, const TripPath& trip_path) {
    json::ArrayPtr edge_array = json::array({});

    // Length and speed default to kilometers
    double scale = 1;
    if (directions_options.has_units()
        && directions_options.units() == DirectionsOptions::kMiles) {
      scale = kMilePerKm;
    }

    // Loop over edges to add attributes
    for (int i = 1; i < trip_path.node().size(); i++) {

      if (trip_path.node(i-1).has_edge()) {
        const auto& edge = trip_path.node(i - 1).edge();

        // Process each edge
        auto edge_map = json::map({});
        if (edge.has_truck_route())
          edge_map->emplace("truck_route", static_cast<bool>(edge.truck_route()));
        if (edge.has_truck_speed() && (edge.truck_speed() > 0))
          edge_map->emplace("truck_speed", static_cast<uint64_t>(std::round(edge.truck_speed() * scale)));
        if (edge.has_speed_limit() && (edge.speed_limit() > 0))
          edge_map->emplace("speed_limit", static_cast<uint64_t>(std::round(edge.speed_limit() * scale)));
        if (edge.has_density())
          edge_map->emplace("density", static_cast<uint64_t>(edge.density()));
        if (edge.has_sidewalk())
          edge_map->emplace("sidewalk", to_string(edge.sidewalk()));
        if (edge.has_bicycle_network())
          edge_map->emplace("bicycle_network", static_cast<uint64_t>(edge.bicycle_network()));
        if (edge.has_cycle_lane())
          edge_map->emplace("cycle_lane", to_string(static_cast<CycleLane>(edge.cycle_lane())));
        if (edge.has_lane_count())
          edge_map->emplace("lane_count", static_cast<uint64_t>(edge.lane_count()));
        if (edge.lane_connectivity_size()) {
          auto lane_connectivity = json::array({});
          for (const auto& l : edge.lane_connectivity()) {
            auto element = json::map({});
            element->emplace("from", l.from_way_id());
            element->emplace("to_lanes", l.to_lanes());
            element->emplace("from_lanes", l.from_lanes());
            lane_connectivity->push_back(element);
          }
          edge_map->emplace("lane_connectivity", lane_connectivity);
        }
        if (edge.has_max_downward_grade())
          edge_map->emplace("max_downward_grade", static_cast<int64_t>(edge.max_downward_grade()));
        if (edge.has_max_upward_grade())
          edge_map->emplace("max_upward_grade", static_cast<int64_t>(edge.max_upward_grade()));
        if (edge.has_weighted_grade())
          edge_map->emplace("weighted_grade", json::fp_t{edge.weighted_grade(), 3});
        if (edge.has_mean_elevation()) {
          // Convert to feet if a valid elevation and units are miles
          float mean = edge.mean_elevation();
          if (mean != kNoElevationData && directions_options.has_units()
              && directions_options.units() == DirectionsOptions::kMiles) {
            mean *= kFeetPerMeter;
          }
          edge_map->emplace("mean_elevation", static_cast<int64_t>(mean));
        }
        if (edge.has_way_id())
          edge_map->emplace("way_id", static_cast<uint64_t>(edge.way_id()));
        if (edge.has_id())
          edge_map->emplace("id", static_cast<uint64_t>(edge.id()));
        if (edge.has_travel_mode())
          edge_map->emplace("travel_mode", to_string(edge.travel_mode()));
        if (edge.has_vehicle_type())
          edge_map->emplace("vehicle_type", to_string(edge.vehicle_type()));
        if (edge.has_pedestrian_type())
          edge_map->emplace("pedestrian_type", to_string(edge.pedestrian_type()));
        if (edge.has_bicycle_type())
          edge_map->emplace("bicycle_type", to_string(edge.bicycle_type()));
        if (edge.has_surface())
          edge_map->emplace("surface", to_string(static_cast<baldr::Surface>(edge.surface())));
        if (edge.has_drive_on_right())
          edge_map->emplace("drive_on_right", static_cast<bool>(edge.drive_on_right()));
        if (edge.has_internal_intersection())
          edge_map->emplace("internal_intersection", static_cast<bool>(edge.internal_intersection()));
        if (edge.has_roundabout())
          edge_map->emplace("roundabout", static_cast<bool>(edge.roundabout()));
        if (edge.has_bridge())
          edge_map->emplace("bridge", static_cast<bool>(edge.bridge()));
        if (edge.has_tunnel())
          edge_map->emplace("tunnel", static_cast<bool>(edge.tunnel()));
        if (edge.has_unpaved())
          edge_map->emplace("unpaved", static_cast<bool>(edge.unpaved()));
        if (edge.has_toll())
            edge_map->emplace("toll", static_cast<bool>(edge.toll()));
        if (edge.has_use())
          edge_map->emplace("use", to_string(static_cast<baldr::Use>(edge.use())));
        if (edge.has_traversability())
          edge_map->emplace("traversability", to_string(edge.traversability()));
        if (edge.has_end_shape_index())
          edge_map->emplace("end_shape_index", static_cast<uint64_t>(edge.end_shape_index()));
        if (edge.has_begin_shape_index())
          edge_map->emplace("begin_shape_index", static_cast<uint64_t>(edge.begin_shape_index()));
        if (edge.has_end_heading())
          edge_map->emplace("end_heading", static_cast<uint64_t>(edge.end_heading()));
        if (edge.has_begin_heading())
          edge_map->emplace("begin_heading", static_cast<uint64_t>(edge.begin_heading()));
        if (edge.has_road_class())
          edge_map->emplace("road_class", to_string(static_cast<baldr::RoadClass>(edge.road_class())));
        if (edge.has_speed())
          edge_map->emplace("speed", static_cast<uint64_t>(std::round(edge.speed() * scale)));
        if (edge.has_length())
          edge_map->emplace("length", json::fp_t{edge.length() * scale, 3});
        if (edge.name_size() > 0) {
          auto names_array = json::array({});
          for (const auto& name : edge.name())
            names_array->push_back(name);
          edge_map->emplace("names", names_array);
        }
        if (edge.traffic_segment().size() > 0) {
          auto segments_array = json::array({});
          for(auto segment : edge.traffic_segment()) {
            json::MapPtr segmap = json::map({
                  {"segment_id", segment.segment_id()},
                  {"begin_percent", json::fp_t{segment.begin_percent(), 3}},
                  {"end_percent", json::fp_t{segment.end_percent(), 3}},
                  {"starts_segment", segment.starts_segment()},
                  {"ends_segment", segment.ends_segment()} });
            segments_array->emplace_back(segmap);
          }
          edge_map->emplace("traffic_segments", segments_array);
        }

        // Process edge sign
        if (edge.has_sign()) {
          auto sign_map = json::map({});

          // Populate exit number array
          if (edge.sign().exit_number_size() > 0) {
            auto exit_number_array = json::array({});
            for (const auto& exit_number : edge.sign().exit_number()) {
              exit_number_array->push_back(exit_number);
            }
            sign_map->emplace("exit_number", exit_number_array);
          }

          // Populate exit branch array
          if (edge.sign().exit_branch_size() > 0) {
            auto exit_branch_array = json::array({});
            for (const auto& exit_branch : edge.sign().exit_branch()) {
              exit_branch_array->push_back(exit_branch);
            }
            sign_map->emplace("exit_branch", exit_branch_array);
          }

          // Populate exit toward array
          if (edge.sign().exit_toward_size() > 0) {
            auto exit_toward_array = json::array({});
            for (const auto& exit_toward : edge.sign().exit_toward()) {
              exit_toward_array->push_back(exit_toward);
            }
            sign_map->emplace("exit_toward", exit_toward_array);
          }

          // Populate exit name array
          if (edge.sign().exit_name_size() > 0) {
            auto exit_name_array = json::array({});
            for (const auto& exit_name : edge.sign().exit_name()) {
              exit_name_array->push_back(exit_name);
            }
            sign_map->emplace("exit_name", exit_name_array);
          }

          edge_map->emplace("sign", sign_map);
        }

        // Process edge end node only if any node items are enabled
        if (controller.category_attribute_enabled(kNodeCategory)) {
          const auto& node = trip_path.node(i);
          auto end_node_map = json::map({});

          if (node.intersecting_edge_size() > 0) {
            auto intersecting_edge_array = json::array({});
            for (const auto& xedge : node.intersecting_edge()) {
              auto xedge_map = json::map({});
              if (xedge.has_walkability() && (xedge.walkability() != TripPath_Traversability_kNone))
                xedge_map->emplace("walkability", to_string(xedge.walkability()));
              if (xedge.has_cyclability() && (xedge.cyclability() != TripPath_Traversability_kNone))
                xedge_map->emplace("cyclability", to_string(xedge.cyclability()));
              if (xedge.has_driveability() && (xedge.driveability() != TripPath_Traversability_kNone))
                xedge_map->emplace("driveability", to_string(xedge.driveability()));
              xedge_map->emplace("from_edge_name_consistency", static_cast<bool>(xedge.prev_name_consistency()));
              xedge_map->emplace("to_edge_name_consistency", static_cast<bool>(xedge.curr_name_consistency()));
              xedge_map->emplace("begin_heading", static_cast<uint64_t>(xedge.begin_heading()));

              intersecting_edge_array->emplace_back(xedge_map);
            }
            end_node_map->emplace("intersecting_edges", intersecting_edge_array);
          }

          if (node.has_elapsed_time())
            end_node_map->emplace("elapsed_time", static_cast<uint64_t>(node.elapsed_time()));
          if (node.has_admin_index())
            end_node_map->emplace("admin_index", static_cast<uint64_t>(node.admin_index()));
          if (node.has_type())
            end_node_map->emplace("type", to_string(static_cast<baldr::NodeType>(node.type())));
          if (node.has_fork())
            end_node_map->emplace("fork", static_cast<bool>(node.fork()));
          if (node.has_time_zone())
            end_node_map->emplace("time_zone", node.time_zone());

          // TODO transit info at node
          // kNodeTransitStopInfoType = "node.transit_stop_info.type";
          // kNodeTransitStopInfoOnestopId = "node.transit_stop_info.onestop_id";
          // kNodetransitStopInfoName = "node.transit_stop_info.name";
          // kNodeTransitStopInfoArrivalDateTime = "node.transit_stop_info.arrival_date_time";
          // kNodeTransitStopInfoDepartureDateTime = "node.transit_stop_info.departure_date_time";
          // kNodeTransitStopInfoIsParentStop = "node.transit_stop_info.is_parent_stop";
          // kNodeTransitStopInfoAssumedSchedule = "node.transit_stop_info.assumed_schedule";
          // kNodeTransitStopInfoLatLon = "node.transit_stop_info.lat_lon";

          edge_map->emplace("end_node", end_node_map);
        }

        // TODO - transit info on edge
        // kEdgeTransitType = "edge.transit_type";
        // kEdgeTransitRouteInfoOnestopId = "edge.transit_route_info.onestop_id";
        // kEdgeTransitRouteInfoBlockId = "edge.transit_route_info.block_id";
        // kEdgeTransitRouteInfoTripId = "edge.transit_route_info.trip_id";
        // kEdgeTransitRouteInfoShortName = "edge.transit_route_info.short_name";
        // kEdgeTransitRouteInfoLongName = "edge.transit_route_info.long_name";
        // kEdgeTransitRouteInfoHeadsign = "edge.transit_route_info.headsign";
        // kEdgeTransitRouteInfoColor = "edge.transit_route_info.color";
        // kEdgeTransitRouteInfoTextColor = "edge.transit_route_info.text_color";
        // kEdgeTransitRouteInfoDescription = "edge.transit_route_info.description";
        // kEdgeTransitRouteInfoOperatorOnestopId = "edge.transit_route_info.operator_onestop_id";
        // kEdgeTransitRouteInfoOperatorName = "edge.transit_route_info.operator_name";
        // kEdgeTransitRouteInfoOperatorUrl = "edge.transit_route_info.operator_url";

        edge_array->emplace_back(edge_map);
      }
    }
    return edge_array;
  }

  json::ArrayPtr serialize_matched_points(const AttributesController& controller,
      const std::vector<thor::MatchResult>& match_results) {
    auto match_points_array = json::array({});
    for (const auto& match_result : match_results) {
      auto match_points_map = json::map({});

      // Process matched point
      if (controller.attributes.at(kMatchedPoint)) {
        match_points_map->emplace("lon", json::fp_t{match_result.lnglat.first,6});
        match_points_map->emplace("lat", json::fp_t{match_result.lnglat.second,6});
      }

      // Process matched type
      if (controller.attributes.at(kMatchedType)) {
        switch (match_result.type) {
          case thor::MatchResult::Type::kMatched:
            match_points_map->emplace("type", std::string("matched"));
            break;
          case thor::MatchResult::Type::kInterpolated:
            match_points_map->emplace("type", std::string("interpolated"));
            break;
          default:
            match_points_map->emplace("type", std::string("unmatched"));
            break;
        }
      }

      // Process matched point edge index
      if (controller.attributes.at(kMatchedEdgeIndex) && match_result.HasEdgeIndex())
        match_points_map->emplace("edge_index", static_cast<uint64_t>(match_result.edge_index));

      // Process matched point begin route discontinuity
      if (controller.attributes.at(kMatchedBeginRouteDiscontinuity) && match_result.begin_route_discontinuity)
        match_points_map->emplace("begin_route_discontinuity", static_cast<bool>(match_result.begin_route_discontinuity));

      // Process matched point end route discontinuity
      if (controller.attributes.at(kMatchedEndRouteDiscontinuity) && match_result.end_route_discontinuity)
        match_points_map->emplace("end_route_discontinuity", static_cast<bool>(match_result.end_route_discontinuity));

      // Process matched point distance along edge
      if (controller.attributes.at(kMatchedDistanceAlongEdge) && (match_result.type != thor::MatchResult::Type::kUnmatched))
        match_points_map->emplace("distance_along_edge", json::fp_t{match_result.distance_along,3});

      // Process matched point distance from trace point
      if (controller.attributes.at(kMatchedDistanceFromTracePoint) && (match_result.type != thor::MatchResult::Type::kUnmatched))
        match_points_map->emplace("distance_from_trace_point", json::fp_t{match_result.distance_from,3});

      match_points_array->push_back(match_points_map);
    }
    return match_points_array;
  }

  void append_trace_info(json::MapPtr json,
      const AttributesController& controller,
      const DirectionsOptions& directions_options,
      const std::tuple<float, float, std::vector<thor::MatchResult>, TripPath>& map_match_result) {
    // Set trip path and match results
    const auto& match_results = std::get<kMatchResultsIndex>(map_match_result);
    const auto& trip_path = std::get<kTripPathIndex>(map_match_result);

    // Add osm_changeset
    if (trip_path.has_osm_changeset())
      json->emplace("osm_changeset", trip_path.osm_changeset());

    // Add shape
    if (trip_path.has_shape())
      json->emplace("shape", trip_path.shape());

    // Add confidence_score
    if (controller.attributes.at(kConfidenceScore)) {
      json->emplace("confidence_score",
          json::fp_t { std::get<kConfidenceScoreIndex>(map_match_result), 3 });
    }

    // Add raw_score
    if (controller.attributes.at(kRawScore)) {
      json->emplace("raw_score",
          json::fp_t { std::get<kRawScoreIndex>(map_match_result), 3 });
    }

    // Add admins list
    if (trip_path.admin_size() > 0) {
      json->emplace("admins", serialize_admins(trip_path));
    }

    // Add edges
    json->emplace("edges", serialize_edges(controller, directions_options, trip_path));

    // Add matched points, if requested
    if (controller.category_attribute_enabled(kMatchedCategory)
        && !match_results.empty()) {
      json->emplace("matched_points", serialize_matched_points(controller, match_results));
    }
  }

  json::MapPtr serialize(const AttributesController& controller,
      const boost::optional<std::string>& id,
      const DirectionsOptions& directions_options,
      std::vector<std::tuple<float, float, std::vector<thor::MatchResult>, TripPath>>& map_match_results) {

    // Create json map to return
    auto json = json::map({});

    // Add result id, if supplied
    if (id)
      json->emplace("id", *id);

    // Add units, if specified
    if (directions_options.has_units()) {
      json->emplace("units", std::string(
        (directions_options.units() == valhalla::odin::DirectionsOptions::kKilometers)
          ? "kilometers" : "miles"));
    }

    // Loop over all results to process the best path
    // and the alternate paths (if alternates exist)
    bool best_path = true;
    auto alt_paths_array = json::array({});
    json->emplace("alternate_paths", alt_paths_array);
    for (const auto& map_match_result : map_match_results) {
      if (best_path) {
        // Append the best path trace info
        append_trace_info(json, controller, directions_options,
            map_match_result);
        best_path = false;
      } else {
        // Append alternate path trace info to alternate path array
        auto alt_path_json = json::map({});
        append_trace_info(alt_path_json, controller, directions_options,
            map_match_result);
        alt_paths_array->push_back(alt_path_json);
      }
    }
    return json;
  }
}

namespace valhalla {
namespace thor {

void thor_worker_t::filter_attributes(const boost::property_tree::ptree& request, AttributesController& controller) {
  std::string filter_action = request.get("filters.action", "");

  if (filter_action.size() && filter_action == "include") {
    controller.disable_all();
    for (const auto& kv : request.get_child("filters.attributes"))
      controller.attributes.at(kv.second.get_value<std::string>()) = true;

  } else if (filter_action.size() && filter_action == "exclude") {
    controller.enable_all();
    for (const auto& kv : request.get_child("filters.attributes"))
      controller.attributes.at(kv.second.get_value<std::string>()) = false;

  } else {
    controller.enable_all();
  }
}

/*
 * The trace_attributes action takes a GPS trace or latitude, longitude positions
 * from a portion of an existing route and returns detailed attribution along the
 * portion of the route. This includes details for each section of road along the
 * path as well as any intersections along the path.
 */
json::MapPtr thor_worker_t::trace_attributes(
    const boost::property_tree::ptree &request) {

  // Parse request
  parse_locations(request);
  parse_costing(request);
  parse_trace_config(request);
  parse_measurements(request);
  /*
   * A flag indicating whether the input shape is a GPS trace or exact points from a
   * prior route run against the Valhalla road network.  Knowing that the input is from
   * Valhalla will allow an efficient “edge-walking” algorithm rather than a more extensive
   * map-matching method. If true, this enforces to only use exact route match algorithm.
   */
  odin::TripPath trip_path;
  std::vector<std::tuple<float, float, std::vector<thor::MatchResult>, odin::TripPath>> map_match_results;
  AttributesController controller;
  filter_attributes(request, controller);
  auto shape_match = STRING_TO_MATCH.find(request.get<std::string>("shape_match", "walk_or_snap"));
  if (shape_match == STRING_TO_MATCH.cend())
    throw valhalla_exception_t{445};
  else {
    // If the exact points from a prior route that was run against the Valhalla road network,
    // then we can traverse the exact shape to form a path by using edge-walking algorithm
    switch (shape_match->second) {
      case EDGE_WALK:
        try {
          trip_path = route_match(controller);
          if (trip_path.node().size() == 0)
            throw std::exception{};
          map_match_results.emplace_back(1.0f, 0.0f, std::vector<thor::MatchResult>{}, trip_path);
        } catch (const std::exception& e) {
          throw valhalla_exception_t{443, shape_match->first + " algorithm failed to find exact route match.  Try using shape_match:'walk_or_snap' to fallback to map-matching algorithm"};
        }
        break;
      // If non-exact shape points are used, then we need to correct this shape by sending them
      // through the map-matching algorithm to snap the points to the correct shape
      case MAP_SNAP:
        try {
          uint32_t best_paths = request.get<uint32_t>("best_paths", 1);
          map_match_results = map_match(controller, true, best_paths);
        } catch (const std::exception& e) {
          throw valhalla_exception_t{444, shape_match->first + " algorithm failed to snap the shape points to the correct shape."};
        }
        break;
      //If we think that we have the exact shape but there ends up being no Valhalla route match, then
      // then we want to fallback to try and use meili map matching to match to local route network.
      //No shortcuts are used and detailed information at every intersection becomes available.
      case WALK_OR_SNAP:
        trip_path = route_match(controller);
        if (trip_path.node().size() == 0) {
          LOG_WARN(shape_match->first + " algorithm failed to find exact route match; Falling back to map_match...");
          try {
            map_match_results = map_match(controller, true);
          } catch (const std::exception& e) {
            throw valhalla_exception_t{444, shape_match->first + " algorithm failed to snap the shape points to the correct shape."};
          }
        } else {
          map_match_results.emplace_back(1.0f, 0.0f, std::vector<thor::MatchResult>{}, trip_path);
        }
        break;
      }
    }

  auto id = request.get_optional<std::string>("id");
  // Get the directions_options if they are in the request
  DirectionsOptions directions_options;
  auto options = request.get_child_optional("directions_options");
  if(options)
    directions_options = valhalla::odin::GetDirectionsOptions(*options);

  //serialize output to Thor
  json::MapPtr json;
  if (!map_match_results.empty()
      && (std::get<kTripPathIndex>(map_match_results.at(0)).node().size() > 0))
    json = serialize(controller, id, directions_options, map_match_results);
  else
    throw valhalla_exception_t { 442 };

  return json;
}
}
}
