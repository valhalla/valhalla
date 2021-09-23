#include <cstdint>

#include "baldr/graphconstants.h"
#include "odin/enhancedtrippath.h"
#include "proto_conversions.h"
#include "thor/attributes_controller.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;
using namespace valhalla::thor;

namespace {

// <Confidence score, raw score, match results, trip path> tuple indexes
constexpr size_t kConfidenceScoreIndex = 0;
constexpr size_t kRawScoreIndex = 1;
constexpr size_t kMatchResultsIndex = 2;
constexpr size_t kTripLegIndex = 3;

void serialize_admins(const TripLeg& trip_path, rapidjson::writer_wrapper_t& writer) {
  writer.start_array("admins");
  for (const auto& admin : trip_path.admin()) {
    writer.start_object();
    if (admin.has_country_code()) {
      writer("country_code", admin.country_code());
    }
    if (admin.has_country_text()) {
      writer("country_text", admin.country_text());
    }
    if (admin.has_state_code()) {
      writer("state_code", admin.state_code());
    }
    if (admin.has_state_text()) {
      writer("state_text", admin.state_text());
    }
    writer.end_object();
  }
  writer.end_array();
}

void serialize_edges(const AttributesController& controller,
                     const Options& options,
                     const TripLeg& trip_path,
                     rapidjson::writer_wrapper_t& writer) {
  writer.start_array("edges");

  // Length and speed default to kilometers
  double scale = 1;
  if (options.has_units() && options.units() == Options::miles) {
    scale = kMilePerKm;
  }

  // Loop over edges to add attributes
  for (int i = 1; i < trip_path.node().size(); i++) {
    if (trip_path.node(i - 1).has_edge()) {
      const auto& edge = trip_path.node(i - 1).edge();

      writer.start_object();
      if (edge.has_truck_route()) {
        writer("truck_route", static_cast<bool>(edge.truck_route()));
      }
      if (edge.has_truck_speed() && (edge.truck_speed() > 0)) {
        writer("truck_speed", static_cast<uint64_t>(std::round(edge.truck_speed() * scale)));
      }
      if (edge.has_speed_limit() && (edge.speed_limit() > 0)) {
        if (edge.speed_limit() == kUnlimitedSpeedLimit) {
          writer("speed_limit", std::string("unlimited"));
        } else {
          writer("speed_limit", static_cast<uint64_t>(std::round(edge.speed_limit() * scale)));
        }
      }
      if (edge.has_density()) {
        writer("density", static_cast<uint64_t>(edge.density()));
      }
      if (edge.has_sac_scale()) {
        writer("sac_scale", static_cast<uint64_t>(edge.sac_scale()));
      }
      if (edge.has_shoulder()) {
        writer("shoulder", static_cast<bool>(edge.shoulder()));
      }
      if (edge.has_sidewalk()) {
        writer("sidewalk", to_string(edge.sidewalk()));
      }
      if (edge.has_bicycle_network()) {
        writer("bicycle_network", static_cast<uint64_t>(edge.bicycle_network()));
      }
      if (edge.has_cycle_lane()) {
        writer("cycle_lane", to_string(static_cast<CycleLane>(edge.cycle_lane())));
      }
      if (edge.has_lane_count()) {
        writer("lane_count", static_cast<uint64_t>(edge.lane_count()));
      }
      if (edge.lane_connectivity_size()) {
        writer.start_array("lane_connectivity");
        for (const auto& l : edge.lane_connectivity()) {
          writer.start_object();
          writer("from", l.from_way_id());
          writer("to_lanes", l.to_lanes());
          writer("from_lanes", l.from_lanes());
          writer.end_object();
        }
        writer.end_array();
      }
      if (edge.has_max_downward_grade()) {
        writer("max_downward_grade", static_cast<int64_t>(edge.max_downward_grade()));
      }
      if (edge.has_max_upward_grade()) {
        writer("max_upward_grade", static_cast<int64_t>(edge.max_upward_grade()));
      }
      if (edge.has_weighted_grade()) {
        writer.set_precision(3);
        writer("weighted_grade", edge.weighted_grade());
      }
      if (edge.has_mean_elevation()) {
        // Convert to feet if a valid elevation and units are miles
        float mean = edge.mean_elevation();
        if (mean != kNoElevationData && options.has_units() && options.units() == Options::miles) {
          mean *= kFeetPerMeter;
        }
        writer("mean_elevation", static_cast<int64_t>(mean));
      }
      if (edge.has_way_id()) {
        writer("way_id", static_cast<uint64_t>(edge.way_id()));
      }
      if (edge.has_id()) {
        writer("id", static_cast<uint64_t>(edge.id()));
      }
      if (edge.has_travel_mode()) {
        writer("travel_mode", to_string(edge.travel_mode()));
      }
      if (edge.has_vehicle_type()) {
        writer("vehicle_type", to_string(edge.vehicle_type()));
      }
      if (edge.has_pedestrian_type()) {
        writer("pedestrian_type", to_string(edge.pedestrian_type()));
      }
      if (edge.has_bicycle_type()) {
        writer("bicycle_type", to_string(edge.bicycle_type()));
      }
      if (edge.has_surface()) {
        writer("surface", to_string(static_cast<baldr::Surface>(edge.surface())));
      }
      if (edge.has_drive_on_right()) {
        writer("drive_on_right", static_cast<bool>(edge.drive_on_right()));
      }
      if (edge.has_internal_intersection()) {
        writer("internal_intersection", static_cast<bool>(edge.internal_intersection()));
      }
      if (edge.has_roundabout()) {
        writer("roundabout", static_cast<bool>(edge.roundabout()));
      }
      if (edge.has_bridge()) {
        writer("bridge", static_cast<bool>(edge.bridge()));
      }
      if (edge.has_tunnel()) {
        writer("tunnel", static_cast<bool>(edge.tunnel()));
      }
      if (edge.has_unpaved()) {
        writer("unpaved", static_cast<bool>(edge.unpaved()));
      }
      if (edge.has_toll()) {
        writer("toll", static_cast<bool>(edge.toll()));
      }
      if (edge.has_use()) {
        writer("use", to_string(static_cast<baldr::Use>(edge.use())));
      }
      if (edge.has_traversability()) {
        writer("traversability", to_string(edge.traversability()));
      }
      if (edge.has_end_shape_index()) {
        writer("end_shape_index", static_cast<uint64_t>(edge.end_shape_index()));
      }
      if (edge.has_begin_shape_index()) {
        writer("begin_shape_index", static_cast<uint64_t>(edge.begin_shape_index()));
      }
      if (edge.has_end_heading()) {
        writer("end_heading", static_cast<uint64_t>(edge.end_heading()));
      }
      if (edge.has_begin_heading()) {
        writer("begin_heading", static_cast<uint64_t>(edge.begin_heading()));
      }
      if (edge.has_road_class()) {
        writer("road_class", to_string(static_cast<baldr::RoadClass>(edge.road_class())));
      }
      if (edge.has_speed()) {
        writer("speed", static_cast<uint64_t>(std::round(edge.speed() * scale)));
      }
      if (edge.has_length_km()) {
        writer.set_precision(3);
        writer("length", edge.length_km() * scale);
      }
      // TODO: do we want to output 'is_route_number'?
      if (edge.name_size() > 0) {
        writer.start_array("names");
        for (const auto& name : edge.name()) {
          writer(name.value());
        }
        writer.end_array();
      }
      if (edge.traffic_segment().size() > 0) {
        writer.start_array("traffic_segments");
        for (const auto& segment : edge.traffic_segment()) {
          writer.start_object();
          writer.set_precision(3);
          writer("segment_id", segment.segment_id());
          writer("begin_percent", segment.begin_percent());
          writer("end_percent", segment.end_percent());
          writer("starts_segment", segment.starts_segment());
          writer("ends_segment", segment.ends_segment());
          writer.end_object();
        }
        writer.end_array();
      }

      // Process edge sign
      // TODO: do we want to output 'is_route_number'?
      if (edge.has_sign()) {
        writer.start_object("sign");

        // Populate exit number array
        if (edge.sign().exit_numbers_size() > 0) {
          writer.start_array("exit_number");
          for (const auto& exit_number : edge.sign().exit_numbers()) {
            writer(exit_number.text());
          }
          writer.end_array();
        }

        // Populate exit branch array
        if (edge.sign().exit_onto_streets_size() > 0) {
          writer.start_array("exit_branch");
          for (const auto& exit_onto_street : edge.sign().exit_onto_streets()) {
            writer(exit_onto_street.text());
          }
          writer.end_array();
        }

        // Populate exit toward array
        if (edge.sign().exit_toward_locations_size() > 0) {
          writer.start_array("exit_toward");
          for (const auto& exit_toward_location : edge.sign().exit_toward_locations()) {
            writer(exit_toward_location.text());
          }
          writer.end_array();
        }

        // Populate exit name array
        if (edge.sign().exit_names_size() > 0) {
          writer.start_array("exit_name");
          for (const auto& exit_name : edge.sign().exit_names()) {
            writer(exit_name.text());
          }
          writer.end_array();
        }
        writer.end_object();
      }

      // Process edge end node only if any node items are enabled
      if (controller.category_attribute_enabled(kNodeCategory)) {
        const auto& node = trip_path.node(i);
        if (node.intersecting_edge_size() > 0) {
          writer.start_array("intersecting_edges");
          for (const auto& xedge : node.intersecting_edge()) {
            writer.start_object();
            if (xedge.has_walkability() && (xedge.walkability() != TripLeg_Traversability_kNone)) {
              writer("walkability", to_string(xedge.walkability()));
            }
            if (xedge.has_cyclability() && (xedge.cyclability() != TripLeg_Traversability_kNone)) {
              writer("cyclability", to_string(xedge.cyclability()));
            }
            if (xedge.has_driveability() && (xedge.driveability() != TripLeg_Traversability_kNone)) {
              writer("driveability", to_string(xedge.driveability()));
            }
            writer("from_edge_name_consistency", static_cast<bool>(xedge.prev_name_consistency()));
            writer("to_edge_name_consistency", static_cast<bool>(xedge.curr_name_consistency()));
            writer("begin_heading", static_cast<uint64_t>(xedge.begin_heading()));
            if (xedge.has_use()) {
              writer("use", to_string(static_cast<baldr::Use>(xedge.use())));
            }
            if (xedge.has_road_class()) {
              writer("road_class", to_string(static_cast<baldr::RoadClass>(xedge.road_class())));
            }
            writer.end_object();
          }
          writer.end_array();
        }

        if (node.has_cost() && node.cost().has_elapsed_cost() &&
            node.cost().elapsed_cost().has_seconds()) {
          writer.set_precision(3);
          writer("elapsed_time", node.cost().elapsed_cost().seconds());
        }
        if (node.has_admin_index()) {
          writer("admin_index", static_cast<uint64_t>(node.admin_index()));
        }
        if (node.has_type()) {
          writer("type", to_string(static_cast<baldr::NodeType>(node.type())));
        }
        if (node.has_fork()) {
          writer("fork", static_cast<bool>(node.fork()));
        }
        if (node.has_time_zone()) {
          writer("time_zone", node.time_zone());
        }
        if (node.has_cost() && node.cost().has_transition_cost() &&
            node.cost().transition_cost().has_seconds()) {
          writer.set_precision(3);
          writer("transition_time", node.cost().transition_cost().seconds());
        }

        // TODO transit info at node
        // kNodeTransitStopInfoType = "node.transit_stop_info.type";
        // kNodeTransitStopInfoOnestopId = "node.transit_stop_info.onestop_id";
        // kNodetransitStopInfoName = "node.transit_stop_info.name";
        // kNodeTransitStopInfoArrivalDateTime = "node.transit_stop_info.arrival_date_time";
        // kNodeTransitStopInfoDepartureDateTime = "node.transit_stop_info.departure_date_time";
        // kNodeTransitStopInfoIsParentStop = "node.transit_stop_info.is_parent_stop";
        // kNodeTransitStopInfoAssumedSchedule = "node.transit_stop_info.assumed_schedule";
        // kNodeTransitStopInfoLatLon = "node.transit_stop_info.lat_lon";

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

      writer.end_object();
    }
  }
  writer.end_array();
}

void serialize_matched_points(const AttributesController& controller,
                              const std::vector<meili::MatchResult>& match_results,
                              rapidjson::writer_wrapper_t& writer) {
  writer.start_array("matched_points");
  for (const auto& match_result : match_results) {
    writer.start_object();

    // Process matched point
    if (controller.attributes.at(kMatchedPoint)) {
      writer.set_precision(6);
      writer("lon", match_result.lnglat.first);
      writer("lat", match_result.lnglat.second);
    }

    // Process matched type
    if (controller.attributes.at(kMatchedType)) {
      switch (match_result.GetType()) {
        case meili::MatchResult::Type::kMatched:
          writer("type", std::string("matched"));
          break;
        case meili::MatchResult::Type::kInterpolated:
          writer("type", std::string("interpolated"));
          break;
        default:
          writer("type", std::string("unmatched"));
          break;
      }
    }

    // TODO: need to keep track of the index of the edge in the global set of edges a given
    // TODO: match result belongs/correlated to
    // Process matched point edge index
    if (controller.attributes.at(kMatchedEdgeIndex) && match_result.edgeid.Is_Valid()) {
      writer("edge_index", static_cast<uint64_t>(match_result.edge_index));
    }

    // Process matched point begin route discontinuity
    if (controller.attributes.at(kMatchedBeginRouteDiscontinuity) &&
        match_result.begins_discontinuity) {
      writer("begin_route_discontinuity", static_cast<bool>(match_result.begins_discontinuity));
    }

    // Process matched point end route discontinuity
    if (controller.attributes.at(kMatchedEndRouteDiscontinuity) && match_result.ends_discontinuity) {
      writer("end_route_discontinuity", static_cast<bool>(match_result.ends_discontinuity));
    }

    // Process matched point distance along edge
    if (controller.attributes.at(kMatchedDistanceAlongEdge) &&
        (match_result.GetType() != meili::MatchResult::Type::kUnmatched)) {
      writer.set_precision(6);
      writer("distance_along_edge", match_result.distance_along);
    }

    // Process matched point distance from trace point
    if (controller.attributes.at(kMatchedDistanceFromTracePoint) &&
        (match_result.GetType() != meili::MatchResult::Type::kUnmatched)) {
      writer.set_precision(6);
      writer("distance_from_trace_point", match_result.distance_from);
    }
    writer.end_object();
  }
  writer.end_array();
}

void serialize_shape_attributes(const AttributesController& controller,
                                const TripLeg& trip_path,
                                rapidjson::writer_wrapper_t& writer) {
  writer.start_object("shape_attributes");
  if (controller.attributes.at(kShapeAttributesTime)) {
    writer.start_array("time");
    writer.set_precision(3);
    for (const auto& time : trip_path.shape_attributes().time()) {
      // milliseconds (ms) to seconds (sec)
      writer(time * kSecPerMillisecond);
    }
    writer.end_array();
  }
  if (controller.attributes.at(kShapeAttributesLength)) {
    writer.start_array("length");
    writer.set_precision(3);
    for (const auto& length : trip_path.shape_attributes().length()) {
      // decimeters (dm) to kilometer (km)
      writer(length * kKmPerDecimeter);
    }
    writer.end_array();
  }
  if (controller.attributes.at(kShapeAttributesSpeed)) {
    writer.start_array("speed");
    writer.set_precision(3);
    for (const auto& speed : trip_path.shape_attributes().speed()) {
      // dm/s to km/h
      writer(speed * kDecimeterPerSectoKPH);
    }
    writer.end_array();
  }
  writer.end_object();
}

void append_trace_info(
    rapidjson::writer_wrapper_t& writer,
    const AttributesController& controller,
    const Options& options,
    const std::tuple<float, float, std::vector<meili::MatchResult>>& map_match_result,
    const TripLeg& trip_path) {
  // Set trip path and match results
  const auto& match_results = std::get<kMatchResultsIndex>(map_match_result);

  // Add osm_changeset
  if (trip_path.has_osm_changeset()) {
    writer("osm_changeset", trip_path.osm_changeset());
  }

  // Add shape
  if (trip_path.has_shape()) {
    writer("shape", trip_path.shape());
  }

  // Add confidence_score
  writer.set_precision(3);
  if (controller.attributes.at(kConfidenceScore)) {
    writer("confidence_score", std::get<kConfidenceScoreIndex>(map_match_result));
  }

  // Add raw_score
  if (controller.attributes.at(kRawScore)) {
    writer("raw_score", std::get<kRawScoreIndex>(map_match_result));
  }

  // Add admins list
  if (trip_path.admin_size() > 0) {
    serialize_admins(trip_path, writer);
  }

  // Add edges
  serialize_edges(controller, options, trip_path, writer);

  // Add matched points, if requested
  if (controller.category_attribute_enabled(kMatchedCategory) && !match_results.empty()) {
    serialize_matched_points(controller, match_results, writer);
  }

  // Add shape_attributes, if requested
  if (controller.category_attribute_enabled(kShapeAttributesCategory)) {
    serialize_shape_attributes(controller, trip_path, writer);
  }
}
} // namespace

namespace valhalla {
namespace tyr {

std::string serializeTraceAttributes(
    const Api& request,
    const AttributesController& controller,
    std::vector<std::tuple<float, float, std::vector<meili::MatchResult>>>& map_match_results) {

  // build up the json object, reserve 4k bytes
  rapidjson::writer_wrapper_t writer(4096);
  writer.start_object();

  // Add result id, if supplied
  if (request.options().has_id()) {
    writer("id", request.options().id());
  }

  // Add units, if specified
  if (request.options().has_units()) {
    writer("units", valhalla::Options_Units_Enum_Name(request.options().units()));
  }

  // Loop over all results to process the best path
  // and the alternate paths (if alternates exist)
  bool best_path = true;
  auto route = request.trip().routes().begin();
  for (const auto& map_match_result : map_match_results) {
    if (best_path) {
      // Append the best path trace info
      append_trace_info(writer, controller, request.options(), map_match_result, route->legs(0));
      best_path = false;
      writer.start_array("alternate_paths");
    } else {
      // Append alternate path trace info to alternate path array
      writer.start_object();
      append_trace_info(writer, controller, request.options(), map_match_result, route->legs(0));
      writer.end_object();
    }
    ++route;
  }
  writer.end_array();
  writer.end_object();
  return writer.get_buffer();
}

} // namespace tyr
} // namespace valhalla
