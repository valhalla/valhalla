#include "tyr/serializers.h"
#include "baldr/datetime.h"
#include "baldr/json.h"
#include "baldr/openlr.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/encoded.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "proto/incidents.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"
#include "proto_conversions.h"

#include <boost/algorithm/string/replace.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::tyr;
using namespace std;

namespace {
using FormOfWay = valhalla::baldr::OpenLR::LocationReferencePoint::FormOfWay;

FormOfWay road_class_to_fow(const valhalla::TripLeg::Edge& edge) {
  if (edge.roundabout()) {
    return FormOfWay::ROUNDABOUT;
  } else if (edge.use() == valhalla::TripLeg::kRampUse ||
             edge.use() == valhalla::TripLeg::kTurnChannelUse) {
    return FormOfWay::SLIPROAD;
  } else if (edge.road_class() == valhalla::RoadClass::kMotorway) {
    return FormOfWay::MOTORWAY;
  } else if (edge.traversability() == valhalla::TripLeg::kBoth) {
    return FormOfWay::MULTIPLE_CARRIAGEWAY;
  } else if (edge.traversability() != valhalla::TripLeg::kNone) {
    return FormOfWay::SINGLE_CARRIAGEWAY;
  } else {
    return FormOfWay::OTHER;
  }
}

std::vector<std::string> openlr_edges(const TripLeg& leg) {
  // TODO: can we get the uncompressed shape when we have it in other serialization steps
  const std::vector<midgard::PointLL>& shape =
      midgard::decode<std::vector<midgard::PointLL>>(leg.shape());
  std::vector<std::string> openlrs;
  openlrs.reserve(leg.node_size());
  for (const TripLeg::Node& node : leg.node()) {
    // the last trip node is the end, we shouldnt have an openlr there
    if (!node.has_edge())
      break;

    const auto& edge = node.edge();

    const FormOfWay fow = road_class_to_fow(edge);
    const auto frc = static_cast<uint8_t>(edge.road_class());

    const auto begin_index = edge.begin_shape_index();
    const auto end_index = edge.end_shape_index();

    const auto& start = shape[begin_index];
    float forward_heading =
        midgard::tangent_angle(begin_index, start, shape, 20.f, true, begin_index, end_index);
    const auto& end = shape[end_index];
    float reverse_heading =
        midgard::tangent_angle(end_index, end, shape, 20.f, false, begin_index, end_index);

    std::vector<baldr::OpenLR::LocationReferencePoint> lrps;
    lrps.emplace_back(start.lng(), start.lat(), forward_heading, frc, fow, nullptr,
                      edge.length_km() * valhalla::midgard::kMetersPerKm, frc);
    lrps.emplace_back(end.lng(), end.lat(), reverse_heading, frc, fow, &lrps.back());
    openlrs.emplace_back(baldr::OpenLR::OpenLr{lrps, 0, 0}.toBase64());
  }
  return openlrs;
}
} // namespace
namespace valhalla {
namespace tyr {
std::string serializeStatus(Api& request) {

  if (request.options().format() == Options_Format_pbf)
    return serializePbf(request);

  rapidjson::Document status_doc;
  status_doc.SetObject();
  auto& alloc = status_doc.GetAllocator();

  status_doc.AddMember("version", rapidjson::Value().SetString(request.status().version(), alloc),
                       alloc);
  status_doc.AddMember("tileset_last_modified",
                       rapidjson::Value().SetInt(request.status().tileset_last_modified()), alloc);

  rapidjson::Value actions_list(rapidjson::kArrayType);
  for (const auto& action : request.status().available_actions()) {
    actions_list.GetArray().PushBack(rapidjson::Value{}.SetString(action.c_str(), alloc), alloc);
  }
  status_doc.AddMember("available_actions", actions_list, alloc);

  if (request.status().has_has_tiles_case())
    status_doc.AddMember("has_tiles", rapidjson::Value().SetBool(request.status().has_tiles()),
                         alloc);
  if (request.status().has_has_admins_case())
    status_doc.AddMember("has_admins", rapidjson::Value().SetBool(request.status().has_admins()),
                         alloc);
  if (request.status().has_has_timezones_case())
    status_doc.AddMember("has_timezones",
                         rapidjson::Value().SetBool(request.status().has_timezones()), alloc);
  if (request.status().has_has_live_traffic_case())
    status_doc.AddMember("has_live_traffic",
                         rapidjson::Value().SetBool(request.status().has_live_traffic()), alloc);
  if (request.status().has_has_transit_tiles_case())
    status_doc.AddMember("has_transit_tiles",
                         rapidjson::Value().SetBool(request.status().has_transit_tiles()), alloc);
  // a 0 changeset indicates there's none, so don't write in the output
  // TODO: currently this can't be tested as gurka isn't adding changeset IDs to OSM objects (yet)
  if (request.status().has_osm_changeset_case() && request.status().osm_changeset())
    status_doc.AddMember("osm_changeset",
                         rapidjson::Value().SetUint64(request.status().osm_changeset()), alloc);

  rapidjson::Document bbox_doc;
  if (request.status().has_bbox_case()) {
    bbox_doc.Parse(request.status().bbox());
    rapidjson::SetValueByPointer(status_doc, "/bbox", bbox_doc, alloc);
  }

  return rapidjson::to_string(status_doc);
}

void route_references(json::MapPtr& route_json, const TripRoute& route, const Options& options) {
  const bool linear_reference =
      options.linear_references() &&
      (options.action() == Options::trace_route || options.action() == Options::route);
  if (!linear_reference) {
    return;
  }
  json::ArrayPtr references = json::array({});
  for (const TripLeg& leg : route.legs()) {
    auto edge_references = openlr_edges(leg);
    references->reserve(references->size() + edge_references.size());
    for (const std::string& openlr : edge_references) {
      references->emplace_back(openlr);
    }
  }
  route_json->emplace("linear_references", references);
}

void openlr(const valhalla::Api& api, int route_index, rapidjson::writer_wrapper_t& writer) {
  // you have to have requested it and you have to be some kind of route response
  if (!api.options().linear_references() ||
      (api.options().action() != Options::trace_route && api.options().action() != Options::route))
    return;

  writer.start_array("linear_references");
  for (const TripLeg& leg : api.trip().routes(route_index).legs()) {
    for (const std::string& openlr : openlr_edges(leg)) {
      writer(openlr);
    }
  }
  writer.end_array();
}

void serializeWarnings(const valhalla::Api& api, rapidjson::writer_wrapper_t& writer) {
  writer.start_array("warnings");
  for (const auto& warning : api.info().warnings()) {
    writer.start_object();
    writer("code", warning.code());
    writer("text", warning.description());
    writer.end_object();
  }
  writer.end_array();
}

json::ArrayPtr serializeWarnings(const valhalla::Api& api) {
  auto warnings = json::array({});
  for (const auto& warning : api.info().warnings()) {
    warnings->emplace_back(json::map({{"code", warning.code()}, {"text", warning.description()}}));
  }
  return warnings;
}

std::string serializePbf(Api& request) {
  // if they dont want to select the parts just pick the obvious thing they would want based on action
  PbfFieldSelector selection = request.options().pbf_field_selector();
  if (!request.options().has_pbf_field_selector()) {
    switch (request.options().action()) {
      // route like requests
      case Options::route:
      case Options::centroid:
      case Options::optimized_route:
      case Options::trace_route:
        selection.set_directions(true);
        break;
      // meta data requests
      case Options::trace_attributes:
        selection.set_trip(true);
        break;
      // service stats
      case Options::status:
        selection.set_status(true);
        break;
      case Options::sources_to_targets:
        selection.set_matrix(true);
        break;
      case Options::isochrone:
        selection.set_isochrone(true);
        break;
      case Options::expansion:
        selection.set_expansion(true);
        break;
      // should never get here, actions which dont have pbf yet return json
      default:
        throw std::logic_error("Requested action is not yet serializable as pbf");
    }
  }

  // if they dont want the options object but its a service request we have to work around it
  bool skip_options = !request.options().pbf_field_selector().options() && request.has_info() &&
                      request.info().is_service();
  Options dummy;
  if (skip_options) {
    request.mutable_options()->Swap(&dummy);
  }

  // disable all the stuff we need to disable, options must be last since we are referencing it
  if (!selection.trip())
    request.clear_trip();
  if (!selection.directions())
    request.clear_directions();
  if (!selection.status())
    request.clear_status();
  if (!selection.options())
    request.clear_options();
  if (!selection.matrix())
    request.clear_matrix();
  if (!selection.isochrone())
    request.clear_isochrone();
  if (!selection.expansion())
    request.clear_expansion();

  // serialize the bytes
  auto bytes = request.SerializeAsString();

  // we do need to keep the options object though because downstream request handling relies on it
  if (skip_options) {
    request.mutable_options()->Swap(&dummy);
  }

  return bytes;
}

// Generate leg shape in geojson format.
baldr::json::MapPtr geojson_shape(const std::vector<midgard::PointLL>& shape) {
  auto geojson = baldr::json::map({});
  auto coords = baldr::json::array({});
  coords->reserve(shape.size());
  for (const auto& p : shape) {
    coords->emplace_back(
        baldr::json::array({baldr::json::fixed_t{p.lng(), 6}, baldr::json::fixed_t{p.lat(), 6}}));
  }
  geojson->emplace("type", std::string("LineString"));
  geojson->emplace("coordinates", coords);
  return geojson;
}

void geojson_shape(const std::vector<midgard::PointLL>& shape, rapidjson::writer_wrapper_t& writer) {
  writer("type", "LineString");
  writer.start_array("coordinates");
  writer.set_precision(tyr::kCoordinatePrecision);
  for (const auto& p : shape) {
    writer.start_array();
    writer(p.lng());
    writer(p.lat());
    writer.end_array();
  }
  writer.set_precision(kDefaultPrecision);
  writer.end_array();
}
} // namespace tyr
} // namespace valhalla

namespace osrm {

// Serialize a location (waypoint) in OSRM compatible format. Waypoint format is described here:
//     http://project-osrm.org/docs/v5.5.1/api/#waypoint-object
valhalla::baldr::json::MapPtr
waypoint(const valhalla::Location& location, bool is_tracepoint, bool is_optimized) {
  // Create a waypoint to add to the array
  auto waypoint = json::map({});

  // Output location as a lon,lat array. Note this is the projected
  // lon,lat on the nearest road.
  auto loc = json::array({});
  loc->emplace_back(json::fixed_t{location.correlation().edges(0).ll().lng(), 6});
  loc->emplace_back(json::fixed_t{location.correlation().edges(0).ll().lat(), 6});
  waypoint->emplace("location", loc);

  // Add street name.
  std::string name =
      location.correlation().edges_size() && location.correlation().edges(0).names_size()
          ? location.correlation().edges(0).names(0)
          : "";
  waypoint->emplace("name", name);

  // Add distance in meters from the input location to the nearest
  // point on the road used in the route
  // TODO: since distance was normalized in thor - need to recalculate here
  //       in the future we shall have store separately from score
  waypoint->emplace("distance",
                    json::fixed_t{to_ll(location.ll())
                                      .Distance(to_ll(location.correlation().edges(0).ll())),
                                  3});

  // If the location was used for a tracepoint we trigger extra serialization
  if (is_tracepoint) {
    waypoint->emplace("alternatives_count",
                      static_cast<uint64_t>(location.correlation().edges_size() - 1));
    if (location.correlation().waypoint_index() == numeric_limits<uint32_t>::max()) {
      // when tracepoint is neither a break nor leg's starting/ending
      // point (shape_index is uint32_t max), we assign null to its waypoint_index
      waypoint->emplace("waypoint_index", static_cast<std::nullptr_t>(nullptr));
    } else {
      waypoint->emplace("waypoint_index",
                        static_cast<uint64_t>(location.correlation().waypoint_index()));
    }
    waypoint->emplace("matchings_index", static_cast<uint64_t>(location.correlation().route_index()));
  }

  // If the location was used for optimized route we add trips_index and waypoint
  // index (index of the waypoint in the trip)
  if (is_optimized) {
    int trips_index = 0; // TODO
    waypoint->emplace("trips_index", static_cast<uint64_t>(trips_index));
    waypoint->emplace("waypoint_index",
                      static_cast<uint64_t>(location.correlation().waypoint_index()));
  }

  return waypoint;
}
void waypoint(const valhalla::Location& location,
              rapidjson::writer_wrapper_t& writer,
              bool is_tracepoint,
              bool is_optimized) {
  // Output location as a lon,lat array. Note this is the projected
  // lon,lat on the nearest road.
  writer.start_object();
  writer.start_array("location");
  writer.set_precision(kCoordinatePrecision);
  writer(location.correlation().edges(0).ll().lng());
  writer(location.correlation().edges(0).ll().lat());
  writer.end_array();
  writer.set_precision(kDefaultPrecision);

  // Add street name.
  std::string name =
      location.correlation().edges_size() && location.correlation().edges(0).names_size()
          ? location.correlation().edges(0).names(0)
          : "";
  writer("name", name);

  // Add distance in meters from the input location to the nearest
  // point on the road used in the route
  // TODO: since distance was normalized in thor - need to recalculate here
  //       in the future we shall have store separately from score
  writer("distance", to_ll(location.ll()).Distance(to_ll(location.correlation().edges(0).ll())));

  // If the location was used for a tracepoint we trigger extra serialization
  if (is_tracepoint) {
    writer("alternatives_count", location.correlation().edges_size() - 1);
    if (location.correlation().waypoint_index() == numeric_limits<uint32_t>::max()) {
      // when tracepoint is neither a break nor leg's starting/ending
      // point (shape_index is uint32_t max), we assign null to its waypoint_index
      writer("waypoint_index", nullptr);
    } else {
      writer("waypoint_index", location.correlation().waypoint_index());
    }
    writer("matchings_index", location.correlation().route_index());
  }

  // If the location was used for optimized route we add trips_index and waypoint
  // index (index of the waypoint in the trip)
  if (is_optimized) {
    int trips_index = 0; // TODO
    writer("trips_index", trips_index);
    writer("waypoint_index", location.correlation().waypoint_index());
  }
  writer.end_object();
}

// Serialize locations (called waypoints in OSRM). Waypoints are described here:
//     http://project-osrm.org/docs/v5.5.1/api/#waypoint-object
json::ArrayPtr waypoints(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                         bool is_tracepoint) {
  auto waypoints = json::array({});
  for (const auto& location : locations) {
    if (location.correlation().edges().size() == 0) {
      waypoints->emplace_back(static_cast<std::nullptr_t>(nullptr));
    } else {
      waypoints->emplace_back(waypoint(location, is_tracepoint));
    }
  }
  return waypoints;
}

void waypoints(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
               rapidjson::writer_wrapper_t& writer,
               bool is_tracepoint) {
  for (const auto& location : locations) {
    if (location.correlation().edges().size() == 0) {
      writer(nullptr);
    } else {
      waypoint(location, writer, is_tracepoint, false);
    }
  }
}

json::ArrayPtr waypoints(const valhalla::Trip& trip) {
  auto waypoints = json::array({});
  // For multi-route the same waypoints are used for all routes.
  for (const auto& leg : trip.routes(0).legs()) {
    for (int i = 0; i < leg.location_size(); ++i) {
      // we skip the first location of legs > 0 because that would duplicate waypoints
      if (i == 0 && !waypoints->empty()) {
        continue;
      }
      waypoints->emplace_back(waypoint(leg.location(i), false));
    }
  }
  return waypoints;
}

/*
 * This function takes any waypoints (excluding origin and destination) and gets
 * the associated leg shape index (geometry index) from the location.  We use
 * that geometry index to calculate the distance_from_leg_start.
 * Then we serialize the via_waypoints object.
 *
 */
json::ArrayPtr intermediate_waypoints(const valhalla::TripLeg& leg) {
  // Create a vector of indexes based on the number of locations.
  auto via_waypoints = json::array({});
  // only loop thru the locations that are not origin or destinations
  for (const auto& loc : leg.location()) {
    // Only create via_waypoints object if the locations are via or through types
    if (loc.type() == valhalla::Location::kVia || loc.type() == valhalla::Location::kThrough) {
      auto via_waypoint = json::map({});
      via_waypoint->emplace("geometry_index",
                            static_cast<uint64_t>(loc.correlation().leg_shape_index()));
      via_waypoint->emplace("distance_from_start",
                            json::fixed_t{loc.correlation().distance_from_leg_origin(), 3});
      via_waypoint->emplace("waypoint_index",
                            static_cast<uint64_t>(loc.correlation().original_index()));
      via_waypoints->emplace_back(via_waypoint);
    }
  }
  return via_waypoints;
}

void serializeIncidentProperties(rapidjson::writer_wrapper_t& writer,
                                 const valhalla::IncidentsTile::Metadata& incident_metadata,
                                 const int begin_shape_index,
                                 const int end_shape_index,
                                 const std::string& road_class,
                                 const std::string& key_prefix) {
  writer(key_prefix + "id", std::to_string(incident_metadata.id()));
  // Type is mandatory
  writer(key_prefix + "type", valhalla::incidentTypeToString(incident_metadata.type()));
  if (!incident_metadata.iso_3166_1_alpha2().empty()) {
    writer(key_prefix + "iso_3166_1_alpha2", incident_metadata.iso_3166_1_alpha2());
  }
  if (!incident_metadata.iso_3166_1_alpha3().empty()) {
    writer(key_prefix + "iso_3166_1_alpha3", incident_metadata.iso_3166_1_alpha3());
  }
  if (!incident_metadata.description().empty()) {
    writer(key_prefix + "description", incident_metadata.description());
  }
  if (!incident_metadata.long_description().empty()) {
    writer(key_prefix + "long_description", incident_metadata.long_description());
  }
  if (incident_metadata.creation_time()) {
    writer(key_prefix + "creation_time",
           baldr::DateTime::seconds_to_date_utc(incident_metadata.creation_time()));
  }
  if (incident_metadata.start_time() > 0) {
    writer(key_prefix + "start_time",
           baldr::DateTime::seconds_to_date_utc(incident_metadata.start_time()));
  }
  if (incident_metadata.end_time()) {
    writer(key_prefix + "end_time",
           baldr::DateTime::seconds_to_date_utc(incident_metadata.end_time()));
  }
  if (incident_metadata.impact()) {
    writer(key_prefix + "impact",
           std::string(valhalla::incidentImpactToString(incident_metadata.impact())));
  }
  if (!incident_metadata.sub_type().empty()) {
    writer(key_prefix + "sub_type", incident_metadata.sub_type());
  }
  if (!incident_metadata.sub_type_description().empty()) {
    writer(key_prefix + "sub_type_description", incident_metadata.sub_type_description());
  }
  if (incident_metadata.alertc_codes_size() > 0) {
    writer(key_prefix + "alertc_codes");
    writer.start_array();
    for (const auto& alertc_code : incident_metadata.alertc_codes()) {
      writer(static_cast<uint64_t>(alertc_code));
    }
    writer.end_array();
  }
  {
    writer(key_prefix + "lanes_blocked");
    writer.start_array();
    for (const auto& blocked_lane : incident_metadata.lanes_blocked()) {
      writer(blocked_lane);
    }
    writer.end_array();
  }
  if (incident_metadata.num_lanes_blocked()) {
    writer(key_prefix + "num_lanes_blocked", incident_metadata.num_lanes_blocked());
  }
  if (!incident_metadata.clear_lanes().empty()) {
    writer(key_prefix + "clear_lanes", incident_metadata.clear_lanes());
  }

  if (incident_metadata.length() > 0) {
    writer(key_prefix + "length", incident_metadata.length());
  }

  if (incident_metadata.road_closed()) {
    writer(key_prefix + "closed", incident_metadata.road_closed());
  }
  if (!road_class.empty()) {
    writer(key_prefix + "class", road_class);
  }

  if (incident_metadata.has_congestion()) {
    writer(key_prefix + "congestion");
    writer.start_object();
    writer("value", incident_metadata.congestion().value());
    writer.end_object();
  }

  if (begin_shape_index >= 0) {
    writer(key_prefix + "geometry_index_start", begin_shape_index);
  }
  if (end_shape_index >= 0) {
    writer(key_prefix + "geometry_index_end", end_shape_index);
  }
  // TODO Add test of lanes blocked and add missing properties
}

} // namespace osrm
