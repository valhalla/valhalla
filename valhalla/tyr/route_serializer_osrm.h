#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "baldr/json.h"
#include "midgard/encoded.h"
#include "midgard/pointll.h"
#include "midgard/polyline2.h"
#include "midgard/util.h"
#include "odin/enhancedtrippath.h"
#include "odin/util.h"
#include "tyr/route_summary_cache.h"
#include "tyr/serializer_constants.h"
#include "tyr/serializers.h"
#include "worker.h"

#include "proto/directions.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"
#include "proto_conversions.h"
#ifdef INLINE_TEST
#include "test.h"
#endif

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;
using namespace valhalla::tyr;
using namespace std;

namespace {
const std::string kSignElementDelimiter = ", ";
#if 0
const std::string kDestinationsDelimiter = ": ";
const std::string kSpeedLimitSignVienna = "vienna";
const std::string kSpeedLimitSignMutcd = "mutcd";
const std::string kSpeedLimitUnitsKph = "km/h";
const std::string kSpeedLimitUnitsMph = "mph";

constexpr std::size_t MAX_USED_SEGMENTS = 2;

struct Coordinate {
  std::int32_t lng;
  std::int32_t lat;

  Coordinate(const std::int32_t lng_, const std::int32_t lat_) : lng(lng_), lat(lat_) {
  }
};

inline std::int32_t toFixed(const float floating) {
  const auto d = static_cast<double>(floating);
  const auto fixed = static_cast<std::int32_t>(std::round(d * ENCODE_PRECISION));
  return fixed;
}

inline double toFloating(const std::int32_t fixed) {
  const auto i = static_cast<std::int32_t>(fixed);
  const auto floating = static_cast<double>(i) * DECODE_PRECISION;
  return floating;
}

const constexpr double TILE_SIZE = 256.0;
static constexpr unsigned MAX_ZOOM = 18;
static constexpr unsigned MIN_ZOOM = 1;
// this is an upper bound to current display sizes
static constexpr double VIEWPORT_WIDTH = 8 * TILE_SIZE;
static constexpr double VIEWPORT_HEIGHT = 5 * TILE_SIZE;
static double INV_LOG_2 = 1. / std::log(2);
const constexpr double DEGREE_TO_RAD = 0.017453292519943295769236907684886;
const constexpr double RAD_TO_DEGREE = 1. / DEGREE_TO_RAD;
const constexpr double EPSG3857_MAX_LATITUDE = 85.051128779806592378; // 90(4*atan(exp(pi))/pi-1)

const constexpr PointLL::first_type DOUGLAS_PEUCKER_THRESHOLDS[19] = {
    703125.0, // z0
    351562.5, // z1
    175781.2, // z2
    87890.6,  // z3
    43945.3,  // z4
    21972.6,  // z5
    10986.3,  // z6
    5493.1,   // z7
    2746.5,   // z8
    1373.2,   // z9
    686.6,    // z10
    343.3,    // z11
    171.6,    // z12
    85.8,     // z13
    42.9,     // z14
    21.4,     // z15
    10.7,     // z16
    5.3,      // z17
    2.6,      // z18
};

inline double clamp(const double lat) {
  return std::max(std::min(lat, double(EPSG3857_MAX_LATITUDE)), double(-EPSG3857_MAX_LATITUDE));
}

inline double latToY(const double latitude) {
  // apparently this is the (faster) version of the canonical log(tan()) version
  const auto clamped_latitude = clamp(latitude);
  const double f = std::sin(DEGREE_TO_RAD * static_cast<double>(clamped_latitude));
  return RAD_TO_DEGREE * 0.5 * std::log((1 + f) / (1 - f));
}

inline double lngToPixel(double lon, unsigned zoom) {
  const double shift = (1u << zoom) * TILE_SIZE;
  const double b = shift / 2.0;
  const double x = b * (1 + static_cast<double>(lon) / 180.0);
  return x;
}

inline double latToPixel(double lat, unsigned zoom) {
  const double shift = (1u << zoom) * TILE_SIZE;
  const double b = shift / 2.0;
  const double y = b * (1. - latToY(lat) / 180.);
  return y;
}

inline unsigned getFittedZoom(Coordinate south_west, Coordinate north_east) {
  const auto min_x = lngToPixel(toFloating(south_west.lng), MAX_ZOOM);
  const auto max_y = latToPixel(toFloating(south_west.lat), MAX_ZOOM);
  const auto max_x = lngToPixel(toFloating(north_east.lng), MAX_ZOOM);
  const auto min_y = latToPixel(toFloating(north_east.lat), MAX_ZOOM);
  const double width_ratio = (max_x - min_x) / VIEWPORT_WIDTH;
  const double height_ratio = (max_y - min_y) / VIEWPORT_HEIGHT;
  const auto zoom = MAX_ZOOM - std::max(std::log(width_ratio), std::log(height_ratio)) * INV_LOG_2;

  if (std::isfinite(zoom))
    return std::max<unsigned>(MIN_ZOOM, zoom);
  else
    return MIN_ZOOM;
}

// Sign style and unit conventions for speed limit signs by country.
// Most countries use Vienna style and km/h, but the countries below
// use MUTCD and/or mph conventions.
std::unordered_map<std::string, std::pair<std::string, std::string>> speed_limit_info = {
    {"AG", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"AI", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"AS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"BS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"BZ", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"CA", {kSpeedLimitSignMutcd, kSpeedLimitUnitsKph}},
    {"DM", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"FK", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GB", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GD", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GG", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GU", {kSpeedLimitSignMutcd, kSpeedLimitUnitsMph}},
    {"IM", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"JE", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"KN", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"KY", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"LC", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"LR", {kSpeedLimitSignMutcd, kSpeedLimitUnitsKph}},
    {"MP", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"MS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"PR", {kSpeedLimitSignMutcd, kSpeedLimitUnitsMph}},
    {"SH", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"TC", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"US", {kSpeedLimitSignMutcd, kSpeedLimitUnitsMph}},
    {"VC", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"VG", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"VI", {kSpeedLimitSignMutcd, kSpeedLimitUnitsMph}},
    {"WS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
};
#endif
namespace osrm_serializers {
/*
OSRM output is described in: http://project-osrm.org/docs/v5.5.1/api/
{
    "code":"Ok"
    "waypoints": [{ }, { }...],
    "routes": [
        {
            "geometry":"....."
            "distance":xxx.y
            "duration":yyy.z
            "legs":[
                {
                    "steps":[
                        "intersections":[
                        ]
                        "geometry":" "
                        "maneuver":{
                        }
                    ]
                }
            ]
        },
        ...
    ]
}
*/

std::string destinations(const valhalla::TripSign& sign);

// Add OSRM route summary information: distance, duration
void route_summary(json::MapPtr& route, const valhalla::Api& api, bool imperial, int route_index);

// Generate full shape of the route.
std::vector<PointLL> full_shape(const valhalla::DirectionsRoute& directions,
                                const valhalla::Options& options);

// Generate simplified shape of the route.
std::vector<PointLL> simplified_shape(const valhalla::DirectionsRoute& directions);

json::MapPtr serialize_annotations(const valhalla::TripLeg& trip_leg);

// Serialize waypoints for optimized route. Note that OSRM retains the
// original location order, and stores an index for the waypoint index in
// the optimized sequence.
json::ArrayPtr waypoints(google::protobuf::RepeatedPtrField<valhalla::Location>& locs);

// Simple structure for storing intersection data
struct IntersectionEdges {
  uint32_t bearing;
  bool routeable;
  bool in_edge;
  bool out_edge;

  IntersectionEdges(const uint32_t brg, const bool rte, const bool edge_in, const bool edge_out)
      : bearing(brg), routeable(rte), in_edge(edge_in), out_edge(edge_out) {
  }

  bool operator<(const IntersectionEdges& other) const {
    return bearing < other.bearing;
  }
};

// Process 'indications' array - add indications from left to right
json::ArrayPtr lane_indications(const bool drive_on_right, const uint16_t mask);

// Add intersections along a step/maneuver.
json::ArrayPtr intersections(const valhalla::DirectionsLeg::Maneuver& maneuver,
                             valhalla::odin::EnhancedTripLeg* etp,
                             const std::vector<PointLL>& shape,
                             uint32_t& count,
                             const bool arrive_maneuver,
                             const baldr::AttributesController& controller);

// Add exits (exit numbers) along a step/maneuver.
std::string exits(const valhalla::TripSign& sign);

valhalla::baldr::json::RawJSON serializeIncident(const TripLeg::Incident& incident);

// Serializes incidents and adds to json-document
void serializeIncidents(const google::protobuf::RepeatedPtrField<TripLeg::Incident>& incidents,
                        json::Jmap& doc);

// Add destinations along a step/maneuver. Constructs a destinations string.
// Here are the destinations formats:
//   1. <ref>
//   2. <non-ref>
//   3. <ref>: <non-ref>
// Each <ref> or <non-ref> could have one or more items and will separated with ", "
//   for example: "I 99, US 220, US 30: Altoona, Johnstown"
std::string destinations(const valhalla::TripSign& sign);

const ::google::protobuf::RepeatedPtrField<::valhalla::StreetName>&
get_maneuver_street_names(const valhalla::DirectionsLeg::Maneuver& maneuver);

// Get the names and ref names
std::pair<std::string, std::string> names_and_refs(const valhalla::DirectionsLeg::Maneuver& maneuver);

// Get the pronunciations string
std::string get_pronunciations(const valhalla::DirectionsLeg::Maneuver& maneuver);

// Serialize each leg
json::ArrayPtr serialize_legs(const google::protobuf::RepeatedPtrField<valhalla::DirectionsLeg>& legs,
                              const std::vector<std::string>& leg_summaries,
                              google::protobuf::RepeatedPtrField<valhalla::TripLeg>& path_legs,
                              bool imperial,
                              const valhalla::Options& options,
                              const baldr::AttributesController& controller);

std::vector<std::vector<std::string>>
summarize_route_legs(const google::protobuf::RepeatedPtrField<DirectionsRoute>& routes);

// Serialize route response in OSRM compatible format.
// Inputs are:
//     directions options
//     TripLeg protocol buffer
//     DirectionsLeg protocol buffer
std::string serialize(valhalla::Api& api);

} // namespace osrm_serializers

#ifdef INLINE_TEST

using namespace osrm_serializers;

/// Assert equality of two json documents
//
// TODO Improve the diffed view of mismatching documents
void assert_json_equality(const rapidjson::Document& doc1, const rapidjson::Document& doc2) {
  if (doc1 != doc2) {
    ASSERT_STREQ(rapidjson::serialize(doc1).c_str(), rapidjson::serialize(doc2).c_str());
  }
}

TEST(RouteSerializerOsrm, testserializeIncidents) {
  // Test that an incident is added correctly to the intersections-json

  rapidjson::Document serialized_to_json;
  {
    auto intersection_doc = json::Jmap();
    auto leg = TripLeg();
    // Sets up the incident
    auto incidents = leg.mutable_incidents();
    auto* incident = incidents->Add();
    incident->set_begin_shape_index(42);
    incident->set_end_shape_index(42);

    valhalla::IncidentsTile::Metadata meta;
    meta.set_id(
        // Set a large id that exercises the uint64 serialization
        18446744073709551615u);
    uint64_t creation_time = 1597241829;
    meta.set_creation_time(creation_time);
    meta.set_start_time(creation_time + 100);
    meta.set_end_time(creation_time + 1800);
    meta.set_type(valhalla::IncidentsTile::Metadata::WEATHER);
    meta.set_impact(valhalla::IncidentsTile::Metadata::MAJOR);
    meta.set_description("fooing foo");
    meta.set_long_description("long fooing foo");
    meta.set_sub_type("foo");
    meta.set_sub_type_description("foobar");
    meta.set_road_closed(true);
    meta.set_num_lanes_blocked(2);
    meta.set_length(1337);
    meta.set_clear_lanes("many lanes clear");
    meta.mutable_congestion()->set_value(33);
    meta.add_alertc_codes(11);
    meta.set_iso_3166_1_alpha2("AU");
    meta.set_iso_3166_1_alpha3("AUS");
    *incident->mutable_metadata() = meta;

    // Finally call the function under test to serialize to json
    serializeIncidents(*incidents, intersection_doc);

    // Lastly, convert to rapidjson
    std::stringstream ss;
    ss << intersection_doc;
    serialized_to_json.Parse(ss.str().c_str());
  }

  rapidjson::Document expected_json;
  {
    expected_json.Parse(R"({
      "incidents": [
        {
          "id": "18446744073709551615",
          "type": "weather",
          "iso_3166_1_alpha2": "AU",
          "iso_3166_1_alpha3": "AUS",
          "creation_time": "2020-08-12T14:17:09Z",
          "start_time": "2020-08-12T14:18:49Z",
          "end_time": "2020-08-12T14:47:09Z",
          "impact": "major",
          "description": "fooing foo",
          "long_description": "long fooing foo",
          "sub_type": "foo",
          "sub_type_description": "foobar",
          "alertc_codes": [ 11 ],
          "lanes_blocked": [],
          "num_lanes_blocked": 2,
          "clear_lanes": "many lanes clear",
          "length": 1337,
          "closed": true,
          "congestion": {
            "value": 33
          },
          "geometry_index_start": 42,
          "geometry_index_end": 42
        }
      ]
    })");
    ASSERT_TRUE(expected_json.IsObject());
  }

  assert_json_equality(serialized_to_json, expected_json);
}

TEST(RouteSerializerOsrm, testserializeIncidentsMultipleIncidentsSingleEdge) {
  // Test that multiple incidents on an edge are serialized correctly
  // that only the incident-id is stored in subsequent intersections
  // after the first

  rapidjson::Document serialized_to_json;
  {
    auto intersection_doc = json::Jmap();
    auto leg = TripLeg();
    // Sets up the incident
    auto* incidents = leg.mutable_incidents();
    {
      // First incident
      auto incident = incidents->Add();
      uint64_t creation_time = 1597241829;
      incident->set_begin_shape_index(87);
      incident->set_end_shape_index(92);

      valhalla::IncidentsTile::Metadata meta;
      meta.set_id(1337);
      meta.set_description("Fooo");
      meta.set_creation_time(creation_time);
      meta.set_type(valhalla::IncidentsTile::Metadata::WEATHER);
      meta.set_iso_3166_1_alpha2("SE");
      meta.set_iso_3166_1_alpha3("SWE");
      *incident->mutable_metadata() = meta;
    }
    {
      // second incident
      auto incident = incidents->Add();
      uint64_t creation_time = 1597241800;
      incident->set_begin_shape_index(21);
      incident->set_end_shape_index(104);

      valhalla::IncidentsTile::Metadata meta;
      meta.set_id(2448);
      meta.set_creation_time(creation_time);
      meta.set_start_time(creation_time + 100);
      meta.set_end_time(creation_time + 1800);
      meta.set_type(valhalla::IncidentsTile::Metadata::ACCIDENT);
      meta.set_iso_3166_1_alpha2("SE");
      meta.set_iso_3166_1_alpha3("SWE");
      *incident->mutable_metadata() = meta;
    }

    // Finally call the function under test to serialize to json
    serializeIncidents(*incidents, intersection_doc);

    // Lastly, convert to rapidjson
    std::stringstream ss;
    ss << intersection_doc;
    serialized_to_json.Parse(ss.str().c_str());
  }

  rapidjson::Document expected_json;
  {
    expected_json.Parse(R"({
      "incidents": [
        {
          "id": "1337",
          "description": "Fooo",
          "creation_time": "2020-08-12T14:17:09Z",
          "type": "weather",
          "iso_3166_1_alpha2": "SE",
          "iso_3166_1_alpha3": "SWE",
          "lanes_blocked": [],
          "geometry_index_start": 87,
          "geometry_index_end": 92
        },
        {
          "id": "2448",
          "creation_time": "2020-08-12T14:16:40Z",
          "start_time": "2020-08-12T14:18:20Z",
          "end_time": "2020-08-12T14:46:40Z",
          "type": "accident",
          "iso_3166_1_alpha2": "SE",
          "iso_3166_1_alpha3": "SWE",
          "lanes_blocked": [],
          "geometry_index_start": 21,
          "geometry_index_end": 104
        }
      ]
    })");
    // Ensure the json was parsed
    ASSERT_TRUE(expected_json.IsObject());
  }

  assert_json_equality(serialized_to_json, expected_json);
}

TEST(RouteSerializerOsrm, testserializeIncidentsNothingToAdd) {

  rapidjson::Document serialized_to_json;
  {
    auto intersection_doc = json::Jmap();
    auto leg = TripLeg();

    // Finally call the function under test to serialize to json
    serializeIncidents(leg.incidents(), intersection_doc);

    // Lastly, convert to rapidjson
    std::stringstream ss;
    ss << intersection_doc;
    serialized_to_json.Parse(ss.str().c_str());
  }

  rapidjson::Document expected_json;
  {
    expected_json.Parse("{}");
    ASSERT_TRUE(expected_json.IsObject());
  }

  assert_json_equality(serialized_to_json, expected_json);
}

TEST(RouteSerializerOsrm, testserializeAnnotationsEmpty) {
  rapidjson::Document serialized_to_json;
  {
    auto leg = TripLeg();
    json::MapPtr annotations = serialize_annotations(leg);

    std::stringstream ss;
    ss << *annotations;
    serialized_to_json.Parse(ss.str().c_str());
    std::cout << *annotations << std::endl;
  }
  rapidjson::Document expected_json;
  { expected_json.Parse(R"({})"); }

  assert_json_equality(serialized_to_json, expected_json);
}

TEST(RouteSerializerOsrm, testserializeAnnotations) {
  rapidjson::Document serialized_to_json;
  {
    auto leg = TripLeg();
    leg.mutable_shape_attributes()->add_time(1);
    leg.mutable_shape_attributes()->add_length(2);
    leg.mutable_shape_attributes()->add_speed(3);
    auto annotations = serialize_annotations(leg);

    std::stringstream ss;
    ss << *annotations;
    serialized_to_json.Parse(ss.str().c_str());
  }
  rapidjson::Document expected_json;
  {
    expected_json.Parse(R"({
      "duration": [0.001],
      "distance": [0.2],
      "speed": [0.3]
    })");
    ASSERT_TRUE(expected_json.IsObject());
  }

  assert_json_equality(serialized_to_json, expected_json);
}

TEST(RouteSerializerOsrm, testserializeAnnotationsSpeedLimits) {
  rapidjson::Document serialized_to_json;
  {
    auto leg = TripLeg();
    leg.mutable_shape_attributes()->add_speed_limit(30);
    leg.mutable_shape_attributes()->add_speed_limit(255);
    leg.mutable_shape_attributes()->add_speed_limit(0);
    auto annotations = serialize_annotations(leg);

    std::stringstream ss;
    ss << *annotations;
    serialized_to_json.Parse(ss.str().c_str());
  }
  rapidjson::Document expected_json;
  {
    expected_json.Parse(R"({
      "maxspeed": [
        { "speed": 30, "unit": "km/h" },
        { "none": true },
        { "unknown": true }
      ]
    })");
    ASSERT_TRUE(expected_json.IsObject());
  }

  assert_json_equality(serialized_to_json, expected_json);
}

TEST(RouteSerializerOsrm, testlaneIndications) {
  json::ArrayPtr indications_1 = lane_indications(true, kTurnLaneReverse | kTurnLaneSharpLeft);
  json::ArrayPtr indications_2 =
      lane_indications(true, kTurnLaneThrough | kTurnLaneRight | kTurnLaneSharpRight);

  ASSERT_EQ(indications_1->size(), 2);
  ASSERT_STREQ(boost::get<std::string>(indications_1->at(0)).c_str(), "uturn");
  ASSERT_STREQ(boost::get<std::string>(indications_1->at(1)).c_str(), "sharp left");

  ASSERT_EQ(indications_2->size(), 3);
  ASSERT_STREQ(boost::get<std::string>(indications_2->at(0)).c_str(), "straight");
  ASSERT_STREQ(boost::get<std::string>(indications_2->at(1)).c_str(), "right");
  ASSERT_STREQ(boost::get<std::string>(indications_2->at(2)).c_str(), "sharp right");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#else

} // namespace

#endif
