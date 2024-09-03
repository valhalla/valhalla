#include <cstdint>

#include "baldr/attributes_controller.h"
#include "baldr/graphconstants.h"
#include "odin/enhancedtrippath.h"
#include "proto_conversions.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;

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
    if (!admin.country_code().empty()) {
      writer("country_code", admin.country_code());
    }
    if (!admin.country_text().empty()) {
      writer("country_text", admin.country_text());
    }
    if (!admin.state_code().empty()) {
      writer("state_code", admin.state_code());
    }
    if (!admin.state_text().empty()) {
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
  if (options.units() == Options::miles) {
    scale = kMilePerKm;
  }

  // Loop over edges to add attributes
  for (int i = 1; i < trip_path.node().size(); i++) {
    if (trip_path.node(i - 1).has_edge()) {
      const auto& edge = trip_path.node(i - 1).edge();

      writer.start_object();
      if (controller(kEdgeTruckRoute)) {
        writer("truck_route", static_cast<bool>(edge.truck_route()));
      }
      if (controller(kEdgeTruckSpeed) && (edge.truck_speed() > 0)) {
        writer("truck_speed", static_cast<uint64_t>(std::round(edge.truck_speed() * scale)));
      }
      if (controller(kEdgeSpeedLimit) && (edge.speed_limit() > 0)) {
        if (edge.speed_limit() == kUnlimitedSpeedLimit) {
          writer("speed_limit", std::string("unlimited"));
        } else {
          writer("speed_limit", static_cast<uint64_t>(std::round(edge.speed_limit() * scale)));
        }
      }
      if (controller(kEdgeDensity)) {
        writer("density", static_cast<uint64_t>(edge.density()));
      }
      if (controller(kEdgeSacScale)) {
        writer("sac_scale", static_cast<uint64_t>(edge.sac_scale()));
      }
      if (controller(kEdgeShoulder)) {
        writer("shoulder", static_cast<bool>(edge.shoulder()));
      }
      if (controller(kEdgeSidewalk)) {
        writer("sidewalk", to_string(edge.sidewalk()));
      }
      if (controller(kEdgeBicycleNetwork)) {
        writer("bicycle_network", static_cast<uint64_t>(edge.bicycle_network()));
      }
      if (controller(kEdgeCycleLane)) {
        writer("cycle_lane", to_string(static_cast<CycleLane>(edge.cycle_lane())));
      }
      if (controller(kEdgeLaneCount)) {
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
      if (controller(kEdgeMaxDownwardGrade)) {
        writer("max_downward_grade", static_cast<int64_t>(edge.max_downward_grade()));
      }
      if (controller(kEdgeMaxUpwardGrade)) {
        writer("max_upward_grade", static_cast<int64_t>(edge.max_upward_grade()));
      }
      if (controller(kEdgeWeightedGrade)) {
        writer.set_precision(3);
        writer("weighted_grade", edge.weighted_grade());
      }
      if (controller(kEdgeMeanElevation)) {
        float mean = edge.mean_elevation();
        if (mean != kNoElevationData) {
          // Convert to feet if a valid elevation and units are miles
          if (static_cast<int64_t>(options.units() == Options::miles)) {
            mean *= kFeetPerMeter;
          }
          writer("mean_elevation", static_cast<int64_t>(mean));
        }
      }
      if (controller(kEdgeWayId)) {
        writer("way_id", static_cast<uint64_t>(edge.way_id()));
      }
      if (controller(kEdgeId)) {
        writer("id", static_cast<uint64_t>(edge.id()));
      }
      if (controller(kEdgeTravelMode)) {
        writer("travel_mode", to_string(edge.travel_mode()));
      }
      if (controller(kEdgeVehicleType) && edge.travel_mode() == valhalla::kDrive) {
        writer("vehicle_type", to_string(edge.vehicle_type()));
      }
      if (controller(kEdgePedestrianType) && edge.travel_mode() == valhalla::kPedestrian) {
        writer("pedestrian_type", to_string(edge.pedestrian_type()));
      }
      if (controller(kEdgeBicycleType) && edge.travel_mode() == valhalla::kBicycle) {
        writer("bicycle_type", to_string(edge.bicycle_type()));
      }
      if (controller(kEdgeSurface)) {
        writer("surface", to_string(static_cast<baldr::Surface>(edge.surface())));
      }
      if (controller(kEdgeDriveOnRight)) {
        writer("drive_on_right", static_cast<bool>(!edge.drive_on_left()));
      }
      if (controller(kEdgeInternalIntersection)) {
        writer("internal_intersection", static_cast<bool>(edge.internal_intersection()));
      }
      if (controller(kEdgeRoundabout)) {
        writer("roundabout", static_cast<bool>(edge.roundabout()));
      }
      if (controller(kEdgeBridge)) {
        writer("bridge", static_cast<bool>(edge.bridge()));
      }
      if (controller(kEdgeTunnel)) {
        writer("tunnel", static_cast<bool>(edge.tunnel()));
      }
      if (controller(kEdgeUnpaved)) {
        writer("unpaved", static_cast<bool>(edge.unpaved()));
      }
      if (controller(kEdgeToll)) {
        writer("toll", static_cast<bool>(edge.toll()));
      }
      if (controller(kEdgeUse)) {
        writer("use", to_string(static_cast<baldr::Use>(edge.use())));
      }
      if (controller(kEdgeTraversability)) {
        writer("traversability", to_string(edge.traversability()));
      }
      if (controller(kEdgeEndShapeIndex)) {
        writer("end_shape_index", static_cast<uint64_t>(edge.end_shape_index()));
      }
      if (controller(kEdgeBeginShapeIndex)) {
        writer("begin_shape_index", static_cast<uint64_t>(edge.begin_shape_index()));
      }
      if (controller(kEdgeEndHeading)) {
        writer("end_heading", static_cast<uint64_t>(edge.end_heading()));
      }
      if (controller(kEdgeBeginHeading)) {
        writer("begin_heading", static_cast<uint64_t>(edge.begin_heading()));
      }
      if (controller(kEdgeRoadClass)) {
        writer("road_class", to_string(static_cast<baldr::RoadClass>(edge.road_class())));
      }
      if (controller(kEdgeSpeed)) {
        writer("speed", static_cast<uint64_t>(std::round(edge.speed() * scale)));
      }
      if (controller(kEdgeCountryCrossing)) {
        writer("country_crossing", static_cast<bool>(edge.country_crossing()));
      }
      if (controller(kEdgeForward)) {
        writer("forward", static_cast<bool>(edge.forward()));
      }
      if (controller(kEdgeLength)) {
        writer.set_precision(3);
        writer("length", edge.length_km() * scale);
        if (edge.source_along_edge() != 0.f) {
          writer("source_percent_along", edge.source_along_edge());
        }
        if (edge.target_along_edge() != 1.f) {
          writer("target_percent_along", edge.target_along_edge());
        }
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
        writer.start_object("end_node");
        if (node.intersecting_edge_size() > 0) {
          writer.start_array("intersecting_edges");
          for (const auto& xedge : node.intersecting_edge()) {
            writer.start_object();
            if (controller(kNodeIntersectingEdgeWalkability) &&
                (xedge.walkability() != TripLeg_Traversability_kNone)) {
              writer("walkability", to_string(xedge.walkability()));
            }
            if (controller(kNodeIntersectingEdgeCyclability) &&
                (xedge.cyclability() != TripLeg_Traversability_kNone)) {
              writer("cyclability", to_string(xedge.cyclability()));
            }
            if (controller(kNodeIntersectingEdgeDriveability) &&
                (xedge.driveability() != TripLeg_Traversability_kNone)) {
              writer("driveability", to_string(xedge.driveability()));
            }
            if (controller(kNodeIntersectingEdgeFromEdgeNameConsistency)) {
              writer("from_edge_name_consistency", static_cast<bool>(xedge.prev_name_consistency()));
            }
            if (controller(kNodeIntersectingEdgeToEdgeNameConsistency)) {
              writer("to_edge_name_consistency", static_cast<bool>(xedge.curr_name_consistency()));
            }
            if (controller(kNodeIntersectingEdgeBeginHeading)) {
              writer("begin_heading", static_cast<uint64_t>(xedge.begin_heading()));
            }
            if (controller(kNodeIntersectingEdgeUse)) {
              writer("use", to_string(static_cast<baldr::Use>(xedge.use())));
            }
            if (controller(kNodeIntersectingEdgeRoadClass)) {
              writer("road_class", to_string(static_cast<baldr::RoadClass>(xedge.road_class())));
            }
            writer.end_object();
          }
          writer.end_array();
        }

        if (controller(kNodeElapsedTime)) {
          writer.set_precision(3);
          writer("elapsed_time", node.cost().elapsed_cost().seconds());
          writer("elapsed_cost", node.cost().elapsed_cost().cost());
        }
        if (controller(kNodeAdminIndex)) {
          writer("admin_index", static_cast<uint64_t>(node.admin_index()));
        }
        if (controller(kNodeType)) {
          writer("type", to_string(static_cast<baldr::NodeType>(node.type())));
        }
        if (controller(kNodeFork)) {
          writer("fork", static_cast<bool>(node.fork()));
        }
        if (controller(kNodeTimeZone) && !node.time_zone().empty()) {
          writer("time_zone", node.time_zone());
        }
        if (controller(kNodeTransitionTime)) {
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
        writer.end_object();
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
    if (controller(kMatchedPoint)) {
      writer.set_precision(6);
      writer("lon", match_result.lnglat.first);
      writer("lat", match_result.lnglat.second);
    }

    // Process matched type
    if (controller(kMatchedType)) {
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
    if (controller(kMatchedEdgeIndex) && match_result.edgeid.Is_Valid()) {
      writer("edge_index", static_cast<uint64_t>(match_result.edge_index));
    }

    // Process matched point begin route discontinuity
    if (controller(kMatchedBeginRouteDiscontinuity) && match_result.begins_discontinuity) {
      writer("begin_route_discontinuity", static_cast<bool>(match_result.begins_discontinuity));
    }

    // Process matched point end route discontinuity
    if (controller(kMatchedEndRouteDiscontinuity) && match_result.ends_discontinuity) {
      writer("end_route_discontinuity", static_cast<bool>(match_result.ends_discontinuity));
    }

    // Process matched point distance along edge
    if (controller(kMatchedDistanceAlongEdge) &&
        (match_result.GetType() != meili::MatchResult::Type::kUnmatched)) {
      writer.set_precision(6);
      writer("distance_along_edge", match_result.distance_along);
    }

    // Process matched point distance from trace point
    if (controller(kMatchedDistanceFromTracePoint) &&
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
  writer.set_precision(3);
  if (controller(kShapeAttributesTime)) {
    writer.start_array("time");
    for (const auto& time : trip_path.shape_attributes().time()) {
      // milliseconds (ms) to seconds (sec)
      writer(time * kSecPerMillisecond);
    }
    writer.end_array();
  }
  if (controller(kShapeAttributesLength)) {
    writer.start_array("length");
    for (const auto& length : trip_path.shape_attributes().length()) {
      // decimeters (dm) to kilometer (km)
      writer(length * kKmPerDecimeter);
    }
    writer.end_array();
  }
  if (controller(kShapeAttributesSpeed)) {
    writer.start_array("speed");
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
  if (controller(kOsmChangeset)) {
    writer("osm_changeset", trip_path.osm_changeset());
  }

  // Add shape
  if (controller(kShape)) {
    writer("shape", trip_path.shape());
  }

  // Add confidence_score
  if (controller(kConfidenceScore)) {
    writer.set_precision(3);
    writer("confidence_score", std::get<kConfidenceScoreIndex>(map_match_result));
  }

  // Add raw_score
  if (controller(kRawScore)) {
    writer.set_precision(3);
    writer("raw_score", std::get<kRawScoreIndex>(map_match_result));
  }

  // Add admins list
  if (trip_path.admin_size() > 0) {
    serialize_admins(trip_path, writer);
  }

  // Add edges
  serialize_edges(controller, options, trip_path, writer);

  // Add elevation at the specified interval
  if (options.elevation_interval() > 0.0f) {
    float unit_factor = options.units() == Options::miles ? kFeetPerMeter : 1.0f;
    float interval = options.elevation_interval();
    writer.set_precision(1);
    writer("elevation_interval", interval * unit_factor);
    writer.start_array("elevation");
    auto elevation = tyr::get_elevation(trip_path, interval);
    for (const auto& h : elevation) {
      writer(h * unit_factor);
    }
    writer.end_array();
  }

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
    Api& request,
    const AttributesController& controller,
    std::vector<std::tuple<float, float, std::vector<meili::MatchResult>>>& map_match_results) {

  // If its pbf format just return the trip
  if (request.options().format() == Options_Format_pbf)
    return serializePbf(request);

  // build up the json object, reserve 4k bytes
  rapidjson::writer_wrapper_t writer(4096);
  writer.start_object();

  // Add result id, if supplied
  if (!request.options().id().empty()) {
    writer("id", request.options().id());
  }

  // Add units
  writer("units", valhalla::Options_Units_Enum_Name(request.options().units()));

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

  // add warnings to json response
  if (request.info().warnings_size() >= 1) {
    valhalla::tyr::serializeWarnings(request, writer);
  }

  writer.end_object();
  return writer.get_buffer();
}

} // namespace tyr
} // namespace valhalla
