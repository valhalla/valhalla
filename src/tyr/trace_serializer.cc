#include <cstdint>

#include "baldr/attributes_controller.h"
#include "baldr/graphconstants.h"
#include "baldr/json.h"
#include "odin/enhancedtrippath.h"
#include "proto_conversions.h"
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

json::ArrayPtr serialize_admins(const TripLeg& trip_path) {
  auto admin_array = json::array({});
  for (const auto& admin : trip_path.admin()) {
    auto admin_map = json::map({});
    if (!admin.country_code().empty()) {
      admin_map->emplace("country_code", admin.country_code());
    }
    if (!admin.country_text().empty()) {
      admin_map->emplace("country_text", admin.country_text());
    }
    if (!admin.state_code().empty()) {
      admin_map->emplace("state_code", admin.state_code());
    }
    if (!admin.state_text().empty()) {
      admin_map->emplace("state_text", admin.state_text());
    }

    admin_array->push_back(admin_map);
  }

  return admin_array;
}

json::ArrayPtr serialize_edges(const AttributesController& controller,
                               const Options& options,
                               const TripLeg& trip_path) {
  json::ArrayPtr edge_array = json::array({});

  // Length and speed default to kilometers
  double scale = 1;
  if (options.units() == Options::miles) {
    scale = kMilePerKm;
  }

  // Loop over edges to add attributes
  for (int i = 1; i < trip_path.node().size(); i++) {

    if (trip_path.node(i - 1).has_edge()) {
      const auto& edge = trip_path.node(i - 1).edge();

      // Process each edge
      auto edge_map = json::map({});
      if (controller(kEdgeTruckRoute)) {
        edge_map->emplace("truck_route", static_cast<bool>(edge.truck_route()));
      }
      if (controller(kEdgeTruckSpeed) && (edge.truck_speed() > 0)) {
        edge_map->emplace("truck_speed",
                          static_cast<uint64_t>(std::round(edge.truck_speed() * scale)));
      }
      if (controller(kEdgeSpeedLimit) && (edge.speed_limit() > 0)) {
        if (edge.speed_limit() == kUnlimitedSpeedLimit) {
          edge_map->emplace("speed_limit", std::string("unlimited"));
        } else {
          edge_map->emplace("speed_limit",
                            static_cast<uint64_t>(std::round(edge.speed_limit() * scale)));
        }
      }
      if (controller(kEdgeDensity)) {
        edge_map->emplace("density", static_cast<uint64_t>(edge.density()));
      }
      if (controller(kEdgeSacScale)) {
        edge_map->emplace("sac_scale", static_cast<uint64_t>(edge.sac_scale()));
      }
      if (controller(kEdgeShoulder)) {
        edge_map->emplace("shoulder", static_cast<bool>(edge.shoulder()));
      }
      if (controller(kEdgeSidewalk)) {
        edge_map->emplace("sidewalk", to_string(edge.sidewalk()));
      }
      if (controller(kEdgeBicycleNetwork)) {
        edge_map->emplace("bicycle_network", static_cast<uint64_t>(edge.bicycle_network()));
      }
      if (controller(kEdgeCycleLane)) {
        edge_map->emplace("cycle_lane", to_string(static_cast<CycleLane>(edge.cycle_lane())));
      }
      if (controller(kEdgeLaneCount)) {
        edge_map->emplace("lane_count", static_cast<uint64_t>(edge.lane_count()));
      }
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
      if (controller(kEdgeMaxDownwardGrade)) {
        if (edge.max_downward_grade() == kNoElevationData) {
          edge_map->emplace("max_downward_grade", static_cast<int64_t>(edge.max_downward_grade()));
        } else {
          edge_map->emplace("max_downward_grade", nullptr);
        }
      }
      if (controller(kEdgeMaxUpwardGrade)) {
        if (edge.max_downward_grade() == kNoElevationData) {
          edge_map->emplace("max_upward_grade", static_cast<int64_t>(edge.max_upward_grade()));
        } else {
          edge_map->emplace("max_upward_grade", nullptr);
        }
      }
      if (controller(kEdgeWeightedGrade)) {
        edge_map->emplace("weighted_grade", json::fixed_t{edge.weighted_grade(), 3});
      }
      if (controller(kEdgeMeanElevation)) {
        // Convert to feet if a valid elevation and units are miles
        float mean = edge.mean_elevation();
        if (mean == kNoElevationData) {
          edge_map->emplace("mean_elevation", nullptr);
        } else {
          edge_map->emplace("mean_elevation", static_cast<int64_t>(options.units() == Options::miles
                                                                       ? mean
                                                                       : mean * kFeetPerMeter));
        }
      }
      if (controller(kEdgeWayId)) {
        edge_map->emplace("way_id", static_cast<uint64_t>(edge.way_id()));
      }
      if (controller(kEdgeId)) {
        edge_map->emplace("id", static_cast<uint64_t>(edge.id()));
      }
      if (controller(kEdgeTravelMode)) {
        edge_map->emplace("travel_mode", to_string(edge.travel_mode()));
      }
      if (controller(kEdgeVehicleType) && edge.travel_mode() == valhalla::kDrive) {
        edge_map->emplace("vehicle_type", to_string(edge.vehicle_type()));
      }
      if (controller(kEdgePedestrianType) && edge.travel_mode() == valhalla::kPedestrian) {
        edge_map->emplace("pedestrian_type", to_string(edge.pedestrian_type()));
      }
      if (controller(kEdgeBicycleType) && edge.travel_mode() == valhalla::kBicycle) {
        edge_map->emplace("bicycle_type", to_string(edge.bicycle_type()));
      }
      if (controller(kEdgeSurface)) {
        edge_map->emplace("surface", to_string(static_cast<baldr::Surface>(edge.surface())));
      }
      if (controller(kEdgeDriveOnRight)) {
        edge_map->emplace("drive_on_right", static_cast<bool>(!edge.drive_on_left()));
      }
      if (controller(kEdgeInternalIntersection)) {
        edge_map->emplace("internal_intersection", static_cast<bool>(edge.internal_intersection()));
      }
      if (controller(kEdgeRoundabout)) {
        edge_map->emplace("roundabout", static_cast<bool>(edge.roundabout()));
      }
      if (controller(kEdgeBridge)) {
        edge_map->emplace("bridge", static_cast<bool>(edge.bridge()));
      }
      if (controller(kEdgeTunnel)) {
        edge_map->emplace("tunnel", static_cast<bool>(edge.tunnel()));
      }
      if (controller(kEdgeUnpaved)) {
        edge_map->emplace("unpaved", static_cast<bool>(edge.unpaved()));
      }
      if (controller(kEdgeToll)) {
        edge_map->emplace("toll", static_cast<bool>(edge.toll()));
      }
      if (controller(kEdgeUse)) {
        edge_map->emplace("use", to_string(static_cast<baldr::Use>(edge.use())));
      }
      if (controller(kEdgeTraversability)) {
        edge_map->emplace("traversability", to_string(edge.traversability()));
      }
      if (controller(kEdgeEndShapeIndex)) {
        edge_map->emplace("end_shape_index", static_cast<uint64_t>(edge.end_shape_index()));
      }
      if (controller(kEdgeBeginShapeIndex)) {
        edge_map->emplace("begin_shape_index", static_cast<uint64_t>(edge.begin_shape_index()));
      }
      if (controller(kEdgeEndHeading)) {
        edge_map->emplace("end_heading", static_cast<uint64_t>(edge.end_heading()));
      }
      if (controller(kEdgeBeginHeading)) {
        edge_map->emplace("begin_heading", static_cast<uint64_t>(edge.begin_heading()));
      }
      if (controller(kEdgeRoadClass)) {
        edge_map->emplace("road_class", to_string(static_cast<baldr::RoadClass>(edge.road_class())));
      }
      if (controller(kEdgeSpeed)) {
        edge_map->emplace("speed", static_cast<uint64_t>(std::round(edge.speed() * scale)));
      }
      if (controller(kEdgeLength)) {
        edge_map->emplace("length", json::fixed_t{edge.length_km() * scale, 3});
      }
      // TODO: do we want to output 'is_route_number'?
      if (edge.name_size() > 0) {
        auto names_array = json::array({});
        for (const auto& name : edge.name()) {
          names_array->push_back(name.value());
        }
        edge_map->emplace("names", names_array);
      }
      if (edge.traffic_segment().size() > 0) {
        auto segments_array = json::array({});
        for (const auto& segment : edge.traffic_segment()) {
          json::MapPtr segmap =
              json::map({{"segment_id", segment.segment_id()},
                         {"begin_percent", json::fixed_t{segment.begin_percent(), 3}},
                         {"end_percent", json::fixed_t{segment.end_percent(), 3}},
                         {"starts_segment", segment.starts_segment()},
                         {"ends_segment", segment.ends_segment()}});
          segments_array->emplace_back(segmap);
        }
        edge_map->emplace("traffic_segments", segments_array);
      }

      // Process edge sign
      // TODO: do we want to output 'is_route_number'?
      if (edge.has_sign()) {
        auto sign_map = json::map({});

        // Populate exit number array
        if (edge.sign().exit_numbers_size() > 0) {
          auto exit_number_array = json::array({});
          for (const auto& exit_number : edge.sign().exit_numbers()) {
            exit_number_array->push_back(exit_number.text());
          }
          sign_map->emplace("exit_number", exit_number_array);
        }

        // Populate exit branch array
        if (edge.sign().exit_onto_streets_size() > 0) {
          auto exit_branch_array = json::array({});
          for (const auto& exit_onto_street : edge.sign().exit_onto_streets()) {
            exit_branch_array->push_back(exit_onto_street.text());
          }
          sign_map->emplace("exit_branch", exit_branch_array);
        }

        // Populate exit toward array
        if (edge.sign().exit_toward_locations_size() > 0) {
          auto exit_toward_array = json::array({});
          for (const auto& exit_toward_location : edge.sign().exit_toward_locations()) {
            exit_toward_array->push_back(exit_toward_location.text());
          }
          sign_map->emplace("exit_toward", exit_toward_array);
        }

        // Populate exit name array
        if (edge.sign().exit_names_size() > 0) {
          auto exit_name_array = json::array({});
          for (const auto& exit_name : edge.sign().exit_names()) {
            exit_name_array->push_back(exit_name.text());
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
            if (controller(kNodeIntersectingEdgeWalkability) &&
                (xedge.walkability() != TripLeg_Traversability_kNone)) {
              xedge_map->emplace("walkability", to_string(xedge.walkability()));
            }
            if (controller(kNodeIntersectingEdgeCyclability) &&
                (xedge.cyclability() != TripLeg_Traversability_kNone)) {
              xedge_map->emplace("cyclability", to_string(xedge.cyclability()));
            }
            if (controller(kNodeIntersectingEdgeDriveability) &&
                (xedge.driveability() != TripLeg_Traversability_kNone)) {
              xedge_map->emplace("driveability", to_string(xedge.driveability()));
            }
            if (controller(kNodeIntersectingEdgeFromEdgeNameConsistency)) {
              xedge_map->emplace("from_edge_name_consistency",
                                 static_cast<bool>(xedge.prev_name_consistency()));
            }
            if (controller(kNodeIntersectingEdgeToEdgeNameConsistency)) {
              xedge_map->emplace("to_edge_name_consistency",
                                 static_cast<bool>(xedge.curr_name_consistency()));
            }
            if (controller(kNodeIntersectingEdgeBeginHeading)) {
              xedge_map->emplace("begin_heading", static_cast<uint64_t>(xedge.begin_heading()));
            }
            if (controller(kNodeIntersectingEdgeUse)) {
              xedge_map->emplace("use", to_string(static_cast<baldr::Use>(xedge.use())));
            }
            if (controller(kNodeIntersectingEdgeRoadClass)) {
              xedge_map->emplace("road_class",
                                 to_string(static_cast<baldr::RoadClass>(xedge.road_class())));
            }
            intersecting_edge_array->emplace_back(xedge_map);
          }
          end_node_map->emplace("intersecting_edges", intersecting_edge_array);
        }

        if (controller(kNodeElapsedTime)) {
          end_node_map->emplace("elapsed_time",
                                json::fixed_t{node.cost().elapsed_cost().seconds(), 3});
        }
        if (controller(kNodeAdminIndex)) {
          end_node_map->emplace("admin_index", static_cast<uint64_t>(node.admin_index()));
        }
        if (controller(kNodeType)) {
          end_node_map->emplace("type", to_string(static_cast<baldr::NodeType>(node.type())));
        }
        if (controller(kNodeFork)) {
          end_node_map->emplace("fork", static_cast<bool>(node.fork()));
        }
        if (controller(kNodeTimeZone) && !node.time_zone().empty()) {
          end_node_map->emplace("time_zone", node.time_zone());
        }
        if (controller(kNodeTransitionTime)) {
          end_node_map->emplace("transition_time",
                                json::fixed_t{node.cost().transition_cost().seconds(), 3});
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
                                        const std::vector<meili::MatchResult>& match_results) {
  auto match_points_array = json::array({});
  for (const auto& match_result : match_results) {
    auto match_points_map = json::map({});

    // Process matched point
    if (controller(kMatchedPoint)) {
      match_points_map->emplace("lon", json::fixed_t{match_result.lnglat.first, 6});
      match_points_map->emplace("lat", json::fixed_t{match_result.lnglat.second, 6});
    }

    // Process matched type
    if (controller(kMatchedType)) {
      switch (match_result.GetType()) {
        case meili::MatchResult::Type::kMatched:
          match_points_map->emplace("type", std::string("matched"));
          break;
        case meili::MatchResult::Type::kInterpolated:
          match_points_map->emplace("type", std::string("interpolated"));
          break;
        default:
          match_points_map->emplace("type", std::string("unmatched"));
          break;
      }
    }

    // TODO: need to keep track of the index of the edge in the global set of edges a given
    // TODO: match result belongs/correlated to
    // Process matched point edge index
    if (controller(kMatchedEdgeIndex) && match_result.edgeid.Is_Valid()) {
      match_points_map->emplace("edge_index", static_cast<uint64_t>(match_result.edge_index));
    }

    // Process matched point begin route discontinuity
    if (controller(kMatchedBeginRouteDiscontinuity) && match_result.begins_discontinuity) {
      match_points_map->emplace("begin_route_discontinuity",
                                static_cast<bool>(match_result.begins_discontinuity));
    }

    // Process matched point end route discontinuity
    if (controller(kMatchedEndRouteDiscontinuity) && match_result.ends_discontinuity) {
      match_points_map->emplace("end_route_discontinuity",
                                static_cast<bool>(match_result.ends_discontinuity));
    }

    // Process matched point distance along edge
    if (controller(kMatchedDistanceAlongEdge) &&
        (match_result.GetType() != meili::MatchResult::Type::kUnmatched)) {
      match_points_map->emplace("distance_along_edge", json::fixed_t{match_result.distance_along, 3});
    }

    // Process matched point distance from trace point
    if (controller(kMatchedDistanceFromTracePoint) &&
        (match_result.GetType() != meili::MatchResult::Type::kUnmatched)) {
      match_points_map->emplace("distance_from_trace_point",
                                json::fixed_t{match_result.distance_from, 3});
    }

    match_points_array->push_back(match_points_map);
  }
  return match_points_array;
}

json::MapPtr serialize_shape_attributes(const AttributesController& controller,
                                        const TripLeg& trip_path) {
  auto attributes_map = json::map({});
  if (controller(kShapeAttributesTime)) {
    auto times_array = json::array({});
    for (const auto& time : trip_path.shape_attributes().time()) {
      // milliseconds (ms) to seconds (sec)
      times_array->push_back(json::fixed_t{time * kSecPerMillisecond, 3});
    }
    attributes_map->emplace("time", times_array);
  }
  if (controller(kShapeAttributesLength)) {
    auto lengths_array = json::array({});
    for (const auto& length : trip_path.shape_attributes().length()) {
      // decimeters (dm) to kilometer (km)
      lengths_array->push_back(json::fixed_t{length * kKmPerDecimeter, 3});
    }
    attributes_map->emplace("length", lengths_array);
  }
  if (controller(kShapeAttributesSpeed)) {
    auto speeds_array = json::array({});
    for (const auto& speed : trip_path.shape_attributes().speed()) {
      // dm/s to km/h
      speeds_array->push_back(json::fixed_t{speed * kDecimeterPerSectoKPH, 3});
    }
    attributes_map->emplace("speed", speeds_array);
  }
  return attributes_map;
}

void append_trace_info(
    const json::MapPtr& json,
    const AttributesController& controller,
    const Options& options,
    const std::tuple<float, float, std::vector<meili::MatchResult>>& map_match_result,
    const TripLeg& trip_path) {
  // Set trip path and match results
  const auto& match_results = std::get<kMatchResultsIndex>(map_match_result);

  // Add osm_changeset
  if (controller(kOsmChangeset)) {
    json->emplace("osm_changeset", trip_path.osm_changeset());
  }

  // Add shape
  if (controller(kShape)) {
    json->emplace("shape", trip_path.shape());
  }

  // Add confidence_score
  if (controller(kConfidenceScore)) {
    json->emplace("confidence_score",
                  json::fixed_t{std::get<kConfidenceScoreIndex>(map_match_result), 3});
  }

  // Add raw_score
  if (controller(kRawScore)) {
    json->emplace("raw_score", json::fixed_t{std::get<kRawScoreIndex>(map_match_result), 3});
  }

  // Add admins list
  if (trip_path.admin_size() > 0) {
    json->emplace("admins", serialize_admins(trip_path));
  }

  // Add edges
  json->emplace("edges", serialize_edges(controller, options, trip_path));

  // Add matched points, if requested
  if (controller.category_attribute_enabled(kMatchedCategory) && !match_results.empty()) {
    json->emplace("matched_points", serialize_matched_points(controller, match_results));
  }

  // Add shape_attributes, if requested
  if (controller.category_attribute_enabled(kShapeAttributesCategory)) {
    json->emplace("shape_attributes", serialize_shape_attributes(controller, trip_path));
  }
}
} // namespace

namespace valhalla {
namespace tyr {

std::string serializeTraceAttributes(
    Api& request,
    const AttributesController& controller,
    std::vector<std::tuple<float, float, std::vector<meili::MatchResult>>>& map_match_results) {

  // TODO: this is missing the matched points, we should add this to the Trip object upstream
  // If its pbf format just return the trip
  if (request.options().format() == Options_Format_pbf)
    return serializePbf(request);

  // Create json map to return
  auto json = json::map({});

  // Add result id, if supplied
  if (!request.options().id().empty()) {
    json->emplace("id", request.options().id());
  }

  // Add units
  json->emplace("units", valhalla::Options_Units_Enum_Name(request.options().units()));

  // Loop over all results to process the best path
  // and the alternate paths (if alternates exist)
  bool best_path = true;
  auto alt_paths_array = json::array({});
  json->emplace("alternate_paths", alt_paths_array);
  auto route = request.trip().routes().begin();
  for (const auto& map_match_result : map_match_results) {
    if (best_path) {
      // Append the best path trace info
      append_trace_info(json, controller, request.options(), map_match_result, route->legs(0));
      best_path = false;
    } else {
      // Append alternate path trace info to alternate path array
      auto alt_path_json = json::map({});
      append_trace_info(alt_path_json, controller, request.options(), map_match_result,
                        route->legs(0));
      alt_paths_array->push_back(alt_path_json);
    }
    ++route;
  }
  std::stringstream ss;
  ss << *json;
  return ss.str();
}

} // namespace tyr
} // namespace valhalla
