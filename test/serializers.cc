#include <string>
#include "proto/route.pb.h"

#include "midgard/logging.h"
#include "tyr/serializers.h"
#include "test.h"

using namespace valhalla;
using namespace tyr;

namespace {

void emptyJsonTest () {
  const std::string empty = "{}";
  Route route;
  jsonToProtoRoute(empty, route);
  if (route.has_trip())
    throw std::runtime_error("Proto Route has_trip() returned true even though there was nothing in the JSON.");
}

void emptyMessageTest () {
  const std::string emptyTrip = R"({"trip":{}})";
  Route route;
  jsonToProtoRoute (emptyTrip, route);
  if (!route.has_trip()) {
    throw std::runtime_error("Proto Route does not have a trip object.");
  }

  Route::Trip trip = route.trip();
  if (trip.locations_size() != 0) {
    throw std::runtime_error ("Proto trip locations size is: " + std::to_string(trip.locations_size()) + " | Expected size: 0");
  }

  if (trip.has_summary()) {
    throw std::runtime_error ("Proto trip has summary when there is none");
  }

  if (trip.legs_size() != 0) {
    throw std::runtime_error ("Proto trip legs size is: " + std::to_string(trip.legs_size()) + " | Expected size: 0");
  }

  if (trip.has_status_message()) {
    throw std::runtime_error ("Proto trip has status_message when there is none");
  }

  if (trip.has_status()) {
    throw std::runtime_error ("Proto trip has status when there is none");
  }

  if (trip.has_units()) {
    throw std::runtime_error ("Proto trip has units when there is none");
  }

  if (trip.has_language()) {
    throw std::runtime_error ("Proto trip has language when there is none");
  }

  if (trip.has_id()) {
    throw std::runtime_error ("Proto trip has id when there is none");
  }
}

void invalidFields () {
  const std::string invalidFields =
      R"({"trip":{
            "bad field 1":125,
            "status_message":"abcdef",
            "bad field 2":"Hello",
            "units":"km",
            "bad field 3":true
        }})";
  Route route;
  jsonToProtoRoute(invalidFields, route);

  if (!route.has_trip()) {
    throw std::runtime_error ("Proto route has no trip when it should");
  }
  Route::Trip trip = route.trip();

  if (trip.locations_size() != 0) {
    throw std::runtime_error ("Proto trip locations size is: " + std::to_string(trip.locations_size()) + " | Expected size: 0");
  }

  if (trip.has_summary()) {
    throw std::runtime_error ("Proto trip has summary when there is none");
  }

  if (trip.legs_size() != 0) {
    throw std::runtime_error ("Proto trip legs size is: " + std::to_string(trip.legs_size()) + " | Expected size: 0");
  }

  if (!trip.has_status_message()) {
    throw std::runtime_error ("Proto trip is missing status message");
  }
  if (trip.status_message() != "abcdef") {
    throw std::runtime_error ("Status message parsed incorrectly. status_message: " + trip.status_message() +
        " | Expected: abcdef");
  }

  if (trip.has_status()) {
    throw std::runtime_error ("Proto trip has status when there is none");
  }

  if (!trip.has_units()) {
    throw std::runtime_error ("Proto trip is missing units");
  }
  if (trip.units() != "km") {
    throw std::runtime_error ("Units parsed incorrectly. units: " + trip.units() + " | Expected: km");
  }

  if (trip.has_language()) {
    throw std::runtime_error ("Proto trip has language when there is none");
  }

  if (trip.has_id()) {
    throw std::runtime_error ("Proto trip has id when there is none");
  }
}

void testTrip() {
  const std::string tripTest =
      R"({
        "trip":{
          "language":"en-US",
          "summary":{},
          "locations":[{},{}],
          "units":"miles",
          "legs":[{},{},{}],
          "status_message":"Found route between points",
          "status":1
        }})";
  Route route;
  jsonToProtoRoute (tripTest, route);
  if (!route.has_trip()) {
    throw std::runtime_error ("Proto route has no trip when it should");
  }

  Route::Trip trip = route.trip();

  if (trip.language() != "en-US") {
    throw std::runtime_error ("language is: " + trip.language() + " | Expected: en-US");
  }

  if (!trip.has_summary()) {
    throw std::runtime_error ("Missing summary");
  }

  if (trip.locations_size() != 2) {
    throw std::runtime_error("locations size: " + std::to_string(trip.locations_size()) + " | Expected: 2");
  }

  if (trip.units() != "miles") {
    throw std::runtime_error("units is" + trip.units() + " | Expected: miles");
  }

  if (trip.legs_size() != 3) {
    throw std::runtime_error("legs size: " + std::to_string(trip.legs_size()) + " | Expected: 3");
  }

  if (trip.status_message() != "Found route between points") {
    throw std::runtime_error ("status message is: " + trip.status_message() +
        " | Expected: Found route between points");
  }

  if (trip.status() != 1) {
    throw std::runtime_error ("status is: " + std::to_string(trip.status()) + " | Expected: 0");
  }

  // Test throwing runtime errors
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::string tripErrorTest =
      R"({trip:20})";
  try {
    jsonToProtoRoute (tripErrorTest, route);
    throw std::runtime_error ("Did not throw parse error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw parse error") {
      throw e;
    }
  }

  tripErrorTest =
      R"({"trip":20})";
  try {
    jsonToProtoRoute (tripErrorTest, route);
    throw std::runtime_error ("Did not throw trip error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw trip error") {
      throw e;
    }
  }

  tripErrorTest =
      R"({"trip":{"language":20}})";
  try {
    jsonToProtoRoute (tripErrorTest, route);
    throw std::runtime_error ("Did not throw language error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw language error") {
      throw e;
    }
  }

  tripErrorTest =
      R"({"trip":{"summary":20}})";
  try {
    jsonToProtoRoute (tripErrorTest, route);
    throw std::runtime_error ("Did not throw summary error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw summary error") {
      throw e;
    }
  }

  tripErrorTest =
      R"({"trip":{"locations":20}})";
  try {
    jsonToProtoRoute (tripErrorTest, route);
    throw std::runtime_error ("Did not throw locations error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw locations error") {
      throw e;
    }
  }

  tripErrorTest =
      R"({"trip":{"locations":[20]}})";
  try {
    jsonToProtoRoute (tripErrorTest, route);
    throw std::runtime_error ("Did not throw location error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw location error") {
      throw e;
    }
  }

  tripErrorTest =
      R"({"trip":{"units":20}})";
  try {
    jsonToProtoRoute (tripErrorTest, route);
    throw std::runtime_error ("Did not throw units error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw units error") {
      throw e;
    }
  }

  tripErrorTest =
      R"({"trip":{"legs":20}})";
  try {
    jsonToProtoRoute (tripErrorTest, route);
    throw std::runtime_error ("Did not throw legs error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw legs error") {
      throw e;
    }
  }

  tripErrorTest =
      R"({"trip":{"legs":[20]}})";
  try {
    jsonToProtoRoute (tripErrorTest, route);
    throw std::runtime_error ("Did not throw leg error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw leg error") {
      throw e;
    }
  }

  tripErrorTest =
      R"({"trip":{"status_message":20}})";
  try {
    jsonToProtoRoute (tripErrorTest, route);
    throw std::runtime_error ("Did not throw status_message error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw status_message error") {
      throw e;
    }
  }

  tripErrorTest =
      R"({"trip":{"status":"1"}})";
  try {
    jsonToProtoRoute (tripErrorTest, route);
    throw std::runtime_error ("Did not throw status error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw status error") {
      throw e;
    }
  }
}

void testLocation() {
  const std::string locationTest =
      R"({
        "trip":{"locations":[{
          "state":"PA",
          "type":"break",
          "side_of_street":"right",
          "lat":12.24153,
          "lon":-12.733452,
          "city":"Lancaster",
          "street":"123 Random Street",
          "heading":25,
          "name":"start",
          "postal_code":"12345",
          "country":"USA",
          "date_time":"03/09/2017",
          "original_index":5
        }]}})";
  Route route;
  jsonToProtoRoute (locationTest, route);
  if (!route.has_trip()) {
    throw std::runtime_error ("Proto route has no trip when it should");
  }

  Route::Trip trip = route.trip();
  if (trip.locations_size() != 1) {
    throw std::runtime_error ("Wrong amount of locations. size: " + std::to_string(trip.locations_size()) +
                              " | Expected: 1");
  }

  Route::Location location = trip.locations(0);
  if (location.lat() != 12.24153f) {
    throw std::runtime_error ("lat is: " + std::to_string(location.lat()) + " | Expected: 12.24153");
  }

  if (location.lon() != -12.733452f) {
    throw std::runtime_error ("lon is: " + std::to_string(location.lon()) + " | Expected: -12.733452");
  }

  if (location.type() != "break") {
    throw std::runtime_error ("type is: " + location.type() + " | Expected: break");
  }

  if (location.heading() != 25) {
    throw std::runtime_error ("heading is: " + std::to_string (location.heading()) + " | Expected: 25");
  }

  if (location.name() != "start") {
    throw std::runtime_error ("name is: " + location.name() + " | Expected: start");
  }

  if (location.street() != "123 Random Street") {
    throw std::runtime_error ("street is: " + location.street() + " | Expected: 123 Random Street");
  }

  if (location.city() != "Lancaster") {
    throw std::runtime_error ("city is: " + location.street() + " | Expected: Lancaster");
  }

  if (location.state() != "PA") {
    throw std::runtime_error ("state is: " + location.state() + " | Expected: PA");
  }

  if (location.postal_code() != "12345") {
    throw std::runtime_error ("postal_code is: " + location.postal_code() + " | Expected: 12345");
  }

  if (location.country() != "USA") {
    throw std::runtime_error ("country is: " + location.country() + " | Expected: USA");
  }

  if (location.date_time() != "03/09/2017") {
    throw std::runtime_error ("date_time is: " + location.date_time() + " | Expected: 03/09/2017");
  }

  if (location.side_of_street() != "right") {
    throw std::runtime_error ("side_of_street is: " + location.side_of_street() + " | Expected: right");
  }

  if (location.original_index() != 5) {
    throw std::runtime_error ("original_index is: " + std::to_string(location.original_index()) + " | Expected: 5");
  }

  // Test throwing runtime errors
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::string locationErrorTest =
      R"({"trip":{"locations":[{"state":20}]}})";
  try {
    jsonToProtoRoute (locationErrorTest, route);
    throw std::runtime_error ("Did not throw state error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw state error") {
      throw e;
    }
  }

  locationErrorTest =
      R"({"trip":{"locations":[{"type":20}]}})";
  try {
    jsonToProtoRoute (locationErrorTest, route);
    throw std::runtime_error ("Did not throw type error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw type error") {
      throw e;
    }
  }

  locationErrorTest =
      R"({"trip":{"locations":[{"side_of_street":20}]}})";
  try {
    jsonToProtoRoute (locationErrorTest, route);
    throw std::runtime_error ("Did not throw side_of_street error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw side_of_street error") {
      throw e;
    }
  }

  locationErrorTest =
      R"({"trip":{"locations":[{"lat":"12.24153"}]}})";
  try {
    jsonToProtoRoute (locationErrorTest, route);
    throw std::runtime_error ("Did not throw lat error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw lat error") {
      throw e;
    }
  }

  locationErrorTest =
      R"({"trip":{"locations":[{"lon":"12.24153"}]}})";
  try {
    jsonToProtoRoute (locationErrorTest, route);
    throw std::runtime_error ("Did not throw lon error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw lon error") {
      throw e;
    }
  }

  locationErrorTest =
      R"({"trip":{"locations":[{"city":1}]}})";
  try {
    jsonToProtoRoute (locationErrorTest, route);
    throw std::runtime_error ("Did not throw city error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw city error") {
      throw e;
    }
  }

  locationErrorTest =
      R"({"trip":{"locations":[{"street":123}]}})";
  try {
    jsonToProtoRoute (locationErrorTest, route);
    throw std::runtime_error ("Did not throw street error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw street error") {
      throw e;
    }
  }

  locationErrorTest =
      R"({"trip":{"locations":[{"heading":"25"}]}})";
  try {
    jsonToProtoRoute (locationErrorTest, route);
    throw std::runtime_error ("Did not throw heading error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw heading error") {
      throw e;
    }
  }

  locationErrorTest =
      R"({"trip":{"locations":[{"name":20}]}})";
  try {
    jsonToProtoRoute (locationErrorTest, route);
    throw std::runtime_error ("Did not throw name error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw name error") {
      throw e;
    }
  }

  locationErrorTest =
      R"({"trip":{"locations":[{"postal_code":12345}]}})";
  try {
    jsonToProtoRoute (locationErrorTest, route);
    throw std::runtime_error ("Did not throw postal_code error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw postal_code error") {
      throw e;
    }
  }

  locationErrorTest =
      R"({"trip":{"locations":[{"country":20}]}})";
  try {
    jsonToProtoRoute (locationErrorTest, route);
    throw std::runtime_error ("Did not throw country error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw country error") {
      throw e;
    }
  }

  locationErrorTest =
      R"({"trip":{"locations":[{"date_time":20}]}})";
  try {
    jsonToProtoRoute (locationErrorTest, route);
    throw std::runtime_error ("Did not throw date_time error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw date_time error") {
      throw e;
    }
  }

  locationErrorTest =
      R"({"trip":{"locations":[{"original_index":"5"}]}})";
  try {
    jsonToProtoRoute (locationErrorTest, route);
    throw std::runtime_error ("Did not throw original_index error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw original_index error") {
      throw e;
    }
  }
}

void testSummary () {
  const std::string summaryTest =
      R"({"trip":{"summary":{
          "max_lon":-12.733452,
          "max_lat":12.24153,
          "time":8435,
          "length":147.371,
          "min_lat":11.24153,
          "min_lon":-13.733452
        }}})";
  Route route;
  jsonToProtoRoute (summaryTest, route);
  if (!route.has_trip()) {
    throw std::runtime_error ("Proto route has no trip when it should");
  }

  Route::Trip trip = route.trip();
  if (!trip.has_summary()) {
    throw std::runtime_error ("Missing summary");
  }

  Route::Summary summary = trip.summary();
  if (summary.length() != 147.371f) {
    throw std::runtime_error ("length is: " + std::to_string(summary.length()) + " | Expected: 147.371");
  }

  if (summary.time() != 8435) {
    throw std::runtime_error ("time is: " + std::to_string(summary.time()) + " | Expected: 8435");
  }

  if (summary.min_lat() != 11.24153f) {
    throw std::runtime_error ("min_lat is: " + std::to_string(summary.min_lat()) + " | Expected: 11.24153");
  }

  if (summary.min_lon() != -13.733452f) {
    throw std::runtime_error ("min_lon is: " + std::to_string(summary.min_lon()) + " | Expected: -13.733452");
  }

  if (summary.max_lat() != 12.24153f) {
    throw std::runtime_error ("max_lat is: " + std::to_string(summary.max_lat()) + " | Expected: 12.24153");
  }

  if (summary.max_lon() != -12.733452f) {
    throw std::runtime_error ("max_lon is: " + std::to_string(summary.max_lon()) + " | Expected: -12.733452");
  }

  // Test throwing runtime errors
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::string summaryErrorTest =
      R"({"trip":{"summary":{"max_lon":"-12.733452"}}})";
  try {
    jsonToProtoRoute (summaryErrorTest, route);
    throw std::runtime_error ("Did not throw max_lon error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw max_lon error") {
      throw e;
    }
  }

  summaryErrorTest =
      R"({"trip":{"summary":{"max_lat":"-12.733452"}}})";
  try {
    jsonToProtoRoute (summaryErrorTest, route);
    throw std::runtime_error ("Did not throw max_lat error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw max_lat error") {
      throw e;
    }
  }

  summaryErrorTest =
      R"({"trip":{"summary":{"time":"8435"}}})";
  try {
    jsonToProtoRoute (summaryErrorTest, route);
    throw std::runtime_error ("Did not throw time error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw time error") {
      throw e;
    }
  }

  summaryErrorTest =
      R"({"trip":{"summary":{"length":"147.371"}}})";
  try {
    jsonToProtoRoute (summaryErrorTest, route);
    throw std::runtime_error ("Did not throw length error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw length error") {
      throw e;
    }
  }

  summaryErrorTest =
      R"({"trip":{"summary":{"min_lat":"11.24153"}}})";
  try {
    jsonToProtoRoute (summaryErrorTest, route);
    throw std::runtime_error ("Did not throw min_lat error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw min_lat error") {
      throw e;
    }
  }

  summaryErrorTest =
      R"({"trip":{"summary":{"min_lat":"11.24153"}}})";
  try {
    jsonToProtoRoute (summaryErrorTest, route);
    throw std::runtime_error ("Did not throw min_lat error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw min_lat error") {
      throw e;
    }
  }

  summaryErrorTest =
      R"({"trip":{"summary":{"min_lon":"11.24153"}}})";
  try {
    jsonToProtoRoute (summaryErrorTest, route);
    throw std::runtime_error ("Did not throw min_lon error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw min_lon error") {
      throw e;
    }
  }
}

void testLeg () {
  const std::string legTest =
      R"({"trip":{"legs":[{
          "shape":"q{ewkA|bliqC|DlJ",
          "maneuvers":[{},{}],
          "summary":{}
        }]}})";
  Route route;
  jsonToProtoRoute (legTest, route);
  if (!route.has_trip()) {
    throw std::runtime_error ("Proto route has no trip when it should");
  }

  Route::Trip trip = route.trip();
  if (trip.legs_size() != 1) {
    throw std::runtime_error("legs size: " + std::to_string(trip.legs_size()) + " | Expected: 1");
  }

  Route::Leg leg = trip.legs(0);
  if (!leg.has_summary()) {
    throw std::runtime_error ("Missing summary");
  }

  if (leg.maneuvers_size() != 2) {
    throw std::runtime_error ("maneuver size: " + std::to_string(leg.maneuvers_size()) + " | Expected: 2");
  }

  if (leg.shape() != "q{ewkA|bliqC|DlJ") {
    throw std::runtime_error ("shape: " + leg.shape() + " | Expected: q{ewkA|bliqC|DlJ");
  }

  // Test throwing runtime errors
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::string legErrorTest =
      R"({"trip":{"legs":[{"shape":20}]}})";
  try {
    jsonToProtoRoute (legErrorTest, route);
    throw std::runtime_error ("Did not throw shape error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw shape error") {
      throw e;
    }
  }

  legErrorTest =
      R"({"trip":{"legs":[{"maneuvers":20}]}})";
  try {
    jsonToProtoRoute (legErrorTest, route);
    throw std::runtime_error ("Did not throw maneuvers error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw maneuvers error") {
      throw e;
    }
  }

  legErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[20]}]}})";
  try {
    jsonToProtoRoute (legErrorTest, route);
    throw std::runtime_error ("Did not throw maneuver error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw maneuver error") {
      throw e;
    }
  }

  legErrorTest =
      R"({"trip":{"legs":[{"summary":20}]}})";
  try {
    jsonToProtoRoute (legErrorTest, route);
    throw std::runtime_error ("Did not throw summary error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw summary error") {
      throw e;
    }
  }
}

void testManeuver () {
  const std::string maneuverTest =
      R"({"trip":{"legs":[{"maneuvers":[{
          "travel_type":"car",
          "travel_mode":"drive",
          "verbal_pre_transition_instruction":"Take exit 1 on the right.",
          "verbal_transition_alert_instruction":"Take exit 1 on the right.",
          "toll":true,
          "instruction":"Take exit 1 on the right.",
          "end_shape_index":2111,
          "type":20,
          "time":33,
          "begin_shape_index":2086,
          "length":0.266,
          "sign":{},
          "street_names":["street 1","street 2"],
          "begin_cardinal_direction":"north",
          "begin_heading":7,
          "rough":true,
          "verbal_post_transition_instruction":"end",
          "begin_street_names":["begin 1","begin 2"],
          "roundabout_exit_count":2,
          "depart_instruction":"leave",
          "verbal_depart_instruction":"leave now",
          "arrive_instruction":"arrived",
          "verbal_arrive_instruction":"arrived at spot",
          "transit_info":{},
          "verbal_multi_cue":true
        }]}]}})";
  Route route;
  jsonToProtoRoute (maneuverTest, route);
  if (!route.has_trip()) {
    throw std::runtime_error ("Proto route has no trip when it should");
  }

  Route::Trip trip = route.trip();
  if (trip.legs_size() != 1) {
    throw std::runtime_error("legs size: " + std::to_string(trip.legs_size()) + " | Expected: 1");
  }

  Route::Leg leg = trip.legs(0);
  if (leg.maneuvers_size() != 1) {
    throw std::runtime_error ("maneuver size: " + std::to_string(leg.maneuvers_size()) + " | Expected: 1");
  }

  Route::Maneuver maneuver = leg.maneuvers(0);
  if (maneuver.type() != 20) {
    throw std::runtime_error ("type is: " + std::to_string(maneuver.type()) + " | Expected: 20");
  }

  if (maneuver.instruction() != "Take exit 1 on the right.") {
    throw std::runtime_error ("instruction is: " + maneuver.instruction() +
                              " | Expected: Take exit 1 on the right.");
  }

  if (maneuver.street_names_size() != 2) {
    throw std::runtime_error ("street_names size: " + std::to_string(maneuver.street_names_size()) + " | Expected: 2");
  }
  if (maneuver.street_names(0) != "street 1") {
    throw std::runtime_error ("street_name 0 is: " + maneuver.street_names(0) + " | Expected: street 1");
  }
  if (maneuver.street_names(1) != "street 2") {
    throw std::runtime_error ("street_name 0 is: " + maneuver.street_names(1) + " | Expected: street 2");
  }

  if (maneuver.length() != 0.266f) {
    throw std::runtime_error ("length is: " + std::to_string(maneuver.length()) + " | Expected: 0.266");
  }

  if (maneuver.time() != 33) {
    throw std::runtime_error ("time is: " + std::to_string(maneuver.time()) + " | Expected: 33");
  }

  if (maneuver.begin_cardinal_direction() != "north") {
    throw std::runtime_error ("begin_cardinal_direction is: " + maneuver.begin_cardinal_direction() + " | Expected: north");
  }

  if (maneuver.begin_heading() != 7) {
    throw std::runtime_error ("begin_heading is: " + std::to_string(maneuver.begin_heading()) + " | Expected: 7");
  }

  if (maneuver.begin_shape_index() != 2086) {
    throw std::runtime_error ("begin_shape_index is: " + std::to_string(maneuver.begin_shape_index()) + " | Expected: 2086");
  }

  if (maneuver.end_shape_index() != 2111) {
    throw std::runtime_error ("end_shape_index is: " + std::to_string(maneuver.end_shape_index()) + " | Expected: 2111");
  }

  if (!maneuver.toll()) {
    throw std::runtime_error ("toll failed to parse");
  }

  if (!maneuver.rough()) {
    throw std::runtime_error ("rough failed to parse");
  }

  if (maneuver.verbal_transition_alert_instruction() != "Take exit 1 on the right.") {
    throw std::runtime_error ("verbal_transition_alert_instruction is: " + maneuver.verbal_transition_alert_instruction() +
                              " | Expected: Take exit 1 on the right.");
  }

  if (maneuver.verbal_pre_transition_instruction() != "Take exit 1 on the right.") {
    throw std::runtime_error ("verbal_pre_transition_instruction is: " + maneuver.verbal_pre_transition_instruction() +
                              " | Expected: Take exit 1 on the right.");
  }

  if (maneuver.verbal_post_transition_instruction() != "end") {
    throw std::runtime_error ("verbal_post_transition_instruction is: " + maneuver.verbal_post_transition_instruction() +
                              " | Expected: end");
  }

  if (maneuver.begin_street_names_size() != 2) {
    throw std::runtime_error ("begin_street_names size: " + std::to_string(maneuver.begin_street_names_size()) +
                              " | Expected: 2");
  }
  if (maneuver.begin_street_names(0) != "begin 1") {
    throw std::runtime_error ("street_name 0 is: " + maneuver.begin_street_names(0) + " | Expected: begin 1");
  }
  if (maneuver.begin_street_names(1) != "begin 2") {
    throw std::runtime_error ("street_name 0 is: " + maneuver.begin_street_names(1) + " | Expected: begin 2");
  }

  if (!maneuver.has_sign()) {
    throw std::runtime_error ("sign missing");
  }

  if (maneuver.roundabout_exit_count() != 2) {
    throw std::runtime_error ("roundabout_exit_count is: " + std::to_string(maneuver.roundabout_exit_count()) +
        " | Expected: 2");
  }

  if (maneuver.roundabout_exit_count() != 2) {
    throw std::runtime_error ("roundabout_exit_count is: " + std::to_string(maneuver.roundabout_exit_count()) +
        " | Expected: 2");
  }

  if (maneuver.depart_instruction() != "leave") {
    throw std::runtime_error ("depart_instruction is: " + maneuver.depart_instruction() + " | Expected: leave");
  }

  if (maneuver.verbal_depart_instruction() != "leave now") {
    throw std::runtime_error ("verbal_depart_instruction is: " + maneuver.verbal_depart_instruction() +
                              " | Expected: leave now");
  }

  if (maneuver.arrive_instruction() != "arrived") {
    throw std::runtime_error ("arrive_instruction is: " + maneuver.arrive_instruction() + " | Expected: arrived");
  }

  if (maneuver.verbal_arrive_instruction() != "arrived at spot") {
    throw std::runtime_error ("verbal_arrive_instruction is: " + maneuver.verbal_arrive_instruction() +
                              " | Expected: arrived at spot");
  }

  if (!maneuver.has_transit_info()) {
    throw std::runtime_error ("transit_info missing");
  }

  if (!maneuver.verbal_multi_cue()) {
    throw std::runtime_error ("verbal_multi_cue failed to parse");
  }

  if (maneuver.travel_mode() != "drive") {
    throw std::runtime_error ("travel_mode is: " + maneuver.travel_mode() + " | Expected: drive");
  }

  if (maneuver.travel_type() != "car") {
    throw std::runtime_error ("travel_type is: " + maneuver.travel_type() + " | Expected: car");
  }

  // Test throwing runtime errors
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::string maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"travel_type":20}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw travel_type error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw travel_type error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"travel_mode":20}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw travel_mode error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw travel_mode error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"verbal_pre_transition_instruction":20}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw verbal_pre_transition_instruction error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw verbal_pre_transition_instruction error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"verbal_transition_alert_instruction":20}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw verbal_transition_alert_instruction error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw verbal_transition_alert_instruction error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"toll":"true"}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw toll error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw toll error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"instruction":20}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw instruction error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw instruction error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"end_shape_index":"2111"}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw end_shape_index error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw end_shape_index error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"type":"20"}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw type error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw type error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"time":"33"}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw time error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw time error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"begin_shape_index":"2086"}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw begin_shape_index error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw begin_shape_index error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"length":"0.266"}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw length error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw length error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"sign":20}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw sign error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw sign error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"street_names":"street"}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw street_names error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw street_names error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"street_names":[20]}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw street_name error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw street_name error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"begin_cardinal_direction":20}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw begin_cardinal_direction error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw begin_cardinal_direction error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"begin_heading":"7"}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw begin_heading error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw begin_heading error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"rough":"true"}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw rough error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw rough error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"verbal_post_transition_instruction":20}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw verbal_post_transition_instruction error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw verbal_post_transition_instruction error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"begin_street_names":"begin"}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw begin_street_names error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw begin_street_names error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"begin_street_names":[20]}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw begin_street_name error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw begin_street_name error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"roundabout_exit_count":"2"}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw roundabout_exit_count error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw roundabout_exit_count error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"depart_instruction":20}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw depart_instruction error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw depart_instruction error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"verbal_depart_instruction":20}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw verbal_depart_instruction error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw verbal_depart_instruction error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"arrive_instruction":20}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw arrive_instruction error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw arrive_instruction error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"verbal_arrive_instruction":20}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw verbal_arrive_instruction error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw verbal_arrive_instruction error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":"hello"}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw transit_info error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw transit_info error") {
      throw e;
    }
  }

  maneuverErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"verbal_multi_cue":"true"}]}]}})";
  try {
    jsonToProtoRoute (maneuverErrorTest, route);
    throw std::runtime_error ("Did not throw verbal_multi_cue error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw verbal_multi_cue error") {
      throw e;
    }
  }
}

void testTransitInfo () {
  const std::string transitInfoTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{
          "onestop_id":"id",
          "short_name":"bus",
          "long_name":"buusss",
          "headsign":"go",
          "color":255,
          "text_color":133,
          "description":"goin away",
          "operator_onestop_id":"id2",
          "operator_name":"bob",
          "operator_url":"www.com",
          "transit_stops":[{},{}]
        }}]}]}})";
  Route route;
  jsonToProtoRoute (transitInfoTest, route);
  if (!route.has_trip()) {
    throw std::runtime_error ("Proto route has no trip when it should");
  }

  Route::Trip trip = route.trip();
  if (trip.legs_size() != 1) {
    throw std::runtime_error("legs size: " + std::to_string(trip.legs_size()) + " | Expected: 1");
  }

  Route::Leg leg = trip.legs(0);
  if (leg.maneuvers_size() != 1) {
    throw std::runtime_error ("maneuver size: " + std::to_string(leg.maneuvers_size()) + " | Expected: 1");
  }

  Route::Maneuver maneuver = leg.maneuvers(0);
  if (!maneuver.has_transit_info()) {
    throw std::runtime_error ("transit_info missing");
  }

  Route::TransitInfo transit_info = maneuver.transit_info();
  if (transit_info.onestop_id() != "id") {
    throw std::runtime_error ("onestop_id is: " + transit_info.onestop_id() + " | Expected: id");
  }

  if (transit_info.short_name() != "bus") {
    throw std::runtime_error ("short_name is: " + transit_info.short_name() + " | Expected: bus");
  }

  if (transit_info.long_name() != "buusss") {
    throw std::runtime_error ("long_name is: " + transit_info.long_name() + " | Expected: buusss");
  }

  if (transit_info.headsign() != "go") {
    throw std::runtime_error ("headsign is: " + transit_info.headsign() + " | Expected: go");
  }

  if (transit_info.color() != 255) {
    throw std::runtime_error ("color is: " + std::to_string(transit_info.color()) + " | Expected: 255");
  }

  if (transit_info.text_color() != 133) {
    throw std::runtime_error ("text_color is: " + std::to_string(transit_info.text_color()) + " | Expected: 133");
  }

  if (transit_info.description() != "goin away") {
    throw std::runtime_error ("description is: " + transit_info.description() + " | Expected: goin away");
  }

  if (transit_info.operator_onestop_id() != "id2") {
    throw std::runtime_error ("operator_onestop_id is: " + transit_info.operator_onestop_id() + " | Expected: id2");
  }

  if (transit_info.operator_name() != "bob") {
    throw std::runtime_error ("operator_name is: " + transit_info.operator_name() + " | Expected: bob");
  }

  if (transit_info.operator_url() != "www.com") {
    throw std::runtime_error ("operator_url is: " + transit_info.operator_url() + " | Expected: www.com");
  }

  if (transit_info.transit_stops_size() != 2) {
    throw std::runtime_error ("transit_stops size: " + std::to_string(transit_info.transit_stops_size()) +
                              " | Expected: 2");
  }

  // Test throwing runtime errors
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::string transitInfoErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"onestop_id":20}}]}]}})";
  try {
    jsonToProtoRoute (transitInfoErrorTest, route);
    throw std::runtime_error ("Did not throw onestop_id error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw onestop_id error") {
      throw e;
    }
  }

  transitInfoErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"short_name":20}}]}]}})";
  try {
    jsonToProtoRoute (transitInfoErrorTest, route);
    throw std::runtime_error ("Did not throw short_name error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw short_name error") {
      throw e;
    }
  }

  transitInfoErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"long_name":20}}]}]}})";
  try {
    jsonToProtoRoute (transitInfoErrorTest, route);
    throw std::runtime_error ("Did not throw long_name error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw long_name error") {
      throw e;
    }
  }

  transitInfoErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"headsign":20}}]}]}})";
  try {
    jsonToProtoRoute (transitInfoErrorTest, route);
    throw std::runtime_error ("Did not throw headsign error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw headsign error") {
      throw e;
    }
  }

  transitInfoErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"color":"255"}}]}]}})";
  try {
    jsonToProtoRoute (transitInfoErrorTest, route);
    throw std::runtime_error ("Did not throw color error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw color error") {
      throw e;
    }
  }

  transitInfoErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"text_color":"133"}}]}]}})";
  try {
    jsonToProtoRoute (transitInfoErrorTest, route);
    throw std::runtime_error ("Did not throw text_color error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw text_color error") {
      throw e;
    }
  }

  transitInfoErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"description":20}}]}]}})";
  try {
    jsonToProtoRoute (transitInfoErrorTest, route);
    throw std::runtime_error ("Did not throw description error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw description error") {
      throw e;
    }
  }

  transitInfoErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"operator_onestop_id":20}}]}]}})";
  try {
    jsonToProtoRoute (transitInfoErrorTest, route);
    throw std::runtime_error ("Did not throw operator_onestop_id error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw operator_onestop_id error") {
      throw e;
    }
  }

  transitInfoErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"operator_name":20}}]}]}})";
  try {
    jsonToProtoRoute (transitInfoErrorTest, route);
    throw std::runtime_error ("Did not throw operator_name error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw operator_name error") {
      throw e;
    }
  }

  transitInfoErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"operator_url":20}}]}]}})";
  try {
    jsonToProtoRoute (transitInfoErrorTest, route);
    throw std::runtime_error ("Did not throw operator_url error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw operator_url error") {
      throw e;
    }
  }

  transitInfoErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"transit_stops":20}}]}]}})";
  try {
    jsonToProtoRoute (transitInfoErrorTest, route);
    throw std::runtime_error ("Did not throw transit_stops error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw transit_stops error") {
      throw e;
    }
  }

  transitInfoErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"transit_stops":[20]}}]}]}})";
  try {
    jsonToProtoRoute (transitInfoErrorTest, route);
    throw std::runtime_error ("Did not throw transit_stop error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw transit_stop error") {
      throw e;
    }
  }
}

void testTransitStop() {
  const std::string transitStopTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"transit_stops":[{
          "type":"simple stop",
          "onestop_id":"id",
          "name":"bus stop",
          "arrival_date_time":"2017-06-12T08:49",
          "departure_date_time":"2017-06-12T07:49",
          "is_parent_stop":true,
          "assumed_schedule":true,
          "lat":13.892,
          "lon":19.12
        }]}}]}]}})";
  Route route;
  jsonToProtoRoute (transitStopTest, route);
  if (!route.has_trip()) {
    throw std::runtime_error ("Proto route has no trip when it should");
  }

  Route::Trip trip = route.trip();
  if (trip.legs_size() != 1) {
    throw std::runtime_error("legs size: " + std::to_string(trip.legs_size()) + " | Expected: 1");
  }

  Route::Leg leg = trip.legs(0);
  if (leg.maneuvers_size() != 1) {
    throw std::runtime_error ("maneuver size: " + std::to_string(leg.maneuvers_size()) + " | Expected: 1");
  }

  Route::Maneuver maneuver = leg.maneuvers(0);
  if (!maneuver.has_transit_info()) {
    throw std::runtime_error ("transit_info missing");
  }

  Route::TransitInfo transit_info = maneuver.transit_info();
  if (transit_info.transit_stops_size() != 1) {
    throw std::runtime_error ("transit_stops size: " + std::to_string(transit_info.transit_stops_size()) +
                              " | Expected: 1");
  }

  Route::TransitStop transit_stop = transit_info.transit_stops(0);
  if (transit_stop.type() != "simple stop") {
    throw std::runtime_error ("type is: " + transit_stop.type() + " | Expected: simple stop");
  }

  if (transit_stop.onestop_id() != "id") {
    throw std::runtime_error ("onestop_id is: " + transit_stop.onestop_id() + " | Expected: id");
  }

  if (transit_stop.name() != "bus stop") {
    throw std::runtime_error ("name is: " + transit_stop.name() + " | Expected: bus stop");
  }

  if (transit_stop.arrival_date_time() != "2017-06-12T08:49") {
    throw std::runtime_error ("arrival_date_time is: " + transit_stop.arrival_date_time() +
                              " | Expected: 2017-06-12T08:49");
  }

  if (transit_stop.departure_date_time() != "2017-06-12T07:49") {
    throw std::runtime_error ("departure_date_time is: " + transit_stop.departure_date_time() +
                              " | Expected: 2017-06-12T07:49");
  }

  if (!transit_stop.is_parent_stop()) {
    throw std::runtime_error ("is_parent_stop failed to parse");
  }

  if (!transit_stop.assumed_schedule()) {
    throw std::runtime_error ("assumed_schedule failed to parse");
  }

  if (transit_stop.lat() != 13.892f) {
    throw std::runtime_error ("lat is: " + std::to_string(transit_stop.lat()) + " | Expected: 13.892");
  }

  if (transit_stop.lon() != 19.12f) {
    throw std::runtime_error ("lon is: " + std::to_string(transit_stop.lon()) + " | Expected: 19.12");
  }

  // Test throwing runtime errors
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::string transitStopErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"transit_stops":[{"type":20}]}}]}]}})";
  try {
    jsonToProtoRoute (transitStopErrorTest, route);
    throw std::runtime_error ("Did not throw type error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw type error") {
      throw e;
    }
  }

  transitStopErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"transit_stops":[{"onestop_id":20}]}}]}]}})";
  try {
    jsonToProtoRoute (transitStopErrorTest, route);
    throw std::runtime_error ("Did not throw onestop_id error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw onestop_id error") {
      throw e;
    }
  }

  transitStopErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"transit_stops":[{"name":20}]}}]}]}})";
  try {
    jsonToProtoRoute (transitStopErrorTest, route);
    throw std::runtime_error ("Did not throw name error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw name error") {
      throw e;
    }
  }

  transitStopErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"transit_stops":[{"arrival_date_time":20}]}}]}]}})";
  try {
    jsonToProtoRoute (transitStopErrorTest, route);
    throw std::runtime_error ("Did not throw arrival_date_time error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw arrival_date_time error") {
      throw e;
    }
  }

  transitStopErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"transit_stops":[{"departure_date_time":20}]}}]}]}})";
  try {
    jsonToProtoRoute (transitStopErrorTest, route);
    throw std::runtime_error ("Did not throw departure_date_time error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw departure_date_time error") {
      throw e;
    }
  }

  transitStopErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"transit_stops":[{"is_parent_stop":"true"}]}}]}]}})";
  try {
    jsonToProtoRoute (transitStopErrorTest, route);
    throw std::runtime_error ("Did not throw is_parent_stop error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw is_parent_stop error") {
      throw e;
    }
  }

  transitStopErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"transit_stops":[{"assumed_schedule":"true"}]}}]}]}})";
  try {
    jsonToProtoRoute (transitStopErrorTest, route);
    throw std::runtime_error ("Did not throw assumed_schedule error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw assumed_schedule error") {
      throw e;
    }
  }

  transitStopErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"transit_stops":[{"lat":"13.892"}]}}]}]}})";
  try {
    jsonToProtoRoute (transitStopErrorTest, route);
    throw std::runtime_error ("Did not throw lat error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw lat error") {
      throw e;
    }
  }

  transitStopErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"transit_info":{"transit_stops":[{"lon":"13.892"}]}}]}]}})";
  try {
    jsonToProtoRoute (transitStopErrorTest, route);
    throw std::runtime_error ("Did not throw lon error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw lon error") {
      throw e;
    }
  }
}

void testSignElements() {
  const std::string signTest =
      R"({"trip":{"legs":[{"maneuvers":[{"sign":{
          "exit_number_elements":[{},{}],
          "exit_toward_elements":[{},{}],
          "exit_branch_elements":[{},{}],
          "exit_name_elements":[{"consecutive_count":2,"text":"Town"},{}]
        }}]}]}})";
  Route route;
  jsonToProtoRoute (signTest, route);
  if (!route.has_trip()) {
    throw std::runtime_error ("Proto route has no trip when it should");
  }

  Route::Trip trip = route.trip();
  if (trip.legs_size() != 1) {
    throw std::runtime_error("legs size: " + std::to_string(trip.legs_size()) + " | Expected: 1");
  }

  Route::Leg leg = trip.legs(0);
  if (leg.maneuvers_size() != 1) {
    throw std::runtime_error ("maneuver size: " + std::to_string(leg.maneuvers_size()) + " | Expected: 1");
  }

  Route::Maneuver maneuver = leg.maneuvers(0);
  if (!maneuver.has_sign()) {
    throw std::runtime_error ("sign missing");
  }

  Route::Maneuver::Sign sign = maneuver.sign();
  if(sign.exit_number_elements_size() != 2) {
    throw std::runtime_error ("exit_number_elements size: " + std::to_string(sign.exit_number_elements_size()) +
                              " | Expected: 2");
  }

  if(sign.exit_branch_elements_size() != 2) {
    throw std::runtime_error ("exit_branch_elements size: " + std::to_string(sign.exit_branch_elements_size()) +
                              " | Expected: 2");
  }

  if(sign.exit_toward_elements_size() != 2) {
    throw std::runtime_error ("exit_toward_elements size: " + std::to_string(sign.exit_toward_elements_size()) +
                              " | Expected: 2");
  }

  if(sign.exit_name_elements_size() != 2) {
    throw std::runtime_error ("exit_name_elements size: " + std::to_string(sign.exit_name_elements_size()) +
                              " | Expected: 2");
  }

  Route::Maneuver::Sign::Element element = sign.exit_name_elements(0);
  if (element.text() != "Town") {
    throw std::runtime_error ("text is: " + element.text() + " | Expected: Town");
  }

  if (element.consecutive_count() != 2) {
    throw std::runtime_error ("consecutive_count is: " + std::to_string(element.consecutive_count()) + " | Expected: 2");
  }

  // Test throwing runtime errors
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::string signElementsErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"sign":{"exit_number_elements":20}}]}]}})";
  try {
    jsonToProtoRoute (signElementsErrorTest, route);
    throw std::runtime_error ("Did not throw exit_number_elements error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw exit_number_elements error") {
      throw e;
    }
  }

  signElementsErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"sign":{"exit_number_elements":[20]}}]}]}})";
  try {
    jsonToProtoRoute (signElementsErrorTest, route);
    throw std::runtime_error ("Did not throw exit_number_element error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw exit_number_element error") {
      throw e;
    }
  }

  signElementsErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"sign":{"exit_toward_elements":20}}]}]}})";
  try {
    jsonToProtoRoute (signElementsErrorTest, route);
    throw std::runtime_error ("Did not throw exit_toward_elements error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw exit_toward_elements error") {
      throw e;
    }
  }

  signElementsErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"sign":{"exit_toward_elements":[20]}}]}]}})";
  try {
    jsonToProtoRoute (signElementsErrorTest, route);
    throw std::runtime_error ("Did not throw exit_toward_element error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw exit_toward_element error") {
      throw e;
    }
  }

  signElementsErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"sign":{"exit_branch_elements":20}}]}]}})";
  try {
    jsonToProtoRoute (signElementsErrorTest, route);
    throw std::runtime_error ("Did not throw exit_branch_elements error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw exit_branch_elements error") {
      throw e;
    }
  }

  signElementsErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"sign":{"exit_branch_elements":[20]}}]}]}})";
  try {
    jsonToProtoRoute (signElementsErrorTest, route);
    throw std::runtime_error ("Did not throw exit_branch_element error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw exit_branch_element error") {
      throw e;
    }
  }

  signElementsErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"sign":{"exit_name_elements":20}}]}]}})";
  try {
    jsonToProtoRoute (signElementsErrorTest, route);
    throw std::runtime_error ("Did not throw exit_name_elements error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw exit_name_elements error") {
      throw e;
    }
  }

  signElementsErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"sign":{"exit_name_elements":[20]}}]}]}})";
  try {
    jsonToProtoRoute (signElementsErrorTest, route);
    throw std::runtime_error ("Did not throw exit_name_element error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw exit_name_element error") {
      throw e;
    }
  }

  signElementsErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"sign":{"exit_name_elements":[{"consecutive_count":"2"}]}}]}]}})";
  try {
    jsonToProtoRoute (signElementsErrorTest, route);
    throw std::runtime_error ("Did not throw consecutive_count error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw consecutive_count error") {
      throw e;
    }
  }

  signElementsErrorTest =
      R"({"trip":{"legs":[{"maneuvers":[{"sign":{"exit_name_elements":[{"text":20}]}}]}]}})";
  try {
    jsonToProtoRoute (signElementsErrorTest, route);
    throw std::runtime_error ("Did not throw text error");
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) == "Did not throw text error") {
      throw e;
    }
  }

}

}

int main() {
  test::suite suite("serailizers");

  // Empty JSON
  suite.test(TEST_CASE(emptyJsonTest));

  // Empty trip
  suite.test(TEST_CASE(emptyMessageTest));

  // Invalid fields amongst valid ones
  suite.test(TEST_CASE(invalidFields));

  // Test trip message parsing
  suite.test(TEST_CASE(testTrip));

  // Test location message parsing
  suite.test(TEST_CASE(testLocation));

  // Test summary message parsing
  suite.test(TEST_CASE(testSummary));

  // Test maneuver message parsing
  suite.test(TEST_CASE(testManeuver));

  // Test transitInfo message parsing
  suite.test(TEST_CASE(testTransitInfo));

  // Test transitStop message parsing
  suite.test(TEST_CASE(testTransitStop));

  // Test sign message parsing
  suite.test(TEST_CASE(testSignElements));
}
