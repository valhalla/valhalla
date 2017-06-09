#include <string>
#include "route.pb.h"

#include "midgard/logging.h"
#include "tyr/util.h"
#include "test.h"

using namespace valhalla;
using namespace tyr;

namespace {

const std::string testStrings[] = {
  R"({
    "trip":{
      "units":"km",
      "locations":[
         {"lat":2.3,"lon":3.4,"type":"break","heading":320,"name":"Location 1","street":"random dr.","city":"Lancaster",
          "state":"PA","postal_code":"17603","country":"USA","date_time":"05/03/2017","side_of_street":"left",
          "orignial_index":0},
         {"lat":2.7,"lon":3.7,"postal_code":23545,"original_index":1,"non Field":1734,"name":"trip"}],
      "language":"en-US",
      "id":"abc"
    }
  })"
};

void emptyJsonTest () {
  std::string empty = "{}";
  Route route;
  jsonToProtoRoute(empty, route);
  if (route.has_trip())
    throw std::runtime_error("Proto Route has_trip() returned true even though there was nothing in the JSON.");
}

void emptyMessageTest () {
  std::string emptyTrip = R"({"trip":{}})";
  Route route;
  jsonToProtoRoute (emptyTrip, route);
  if (!route.has_trip()) {
    throw std::runtime_error("Proto Route does not have a trip object.");
  }

  Route::Trip* trip = route.mutable_trip();
  if (trip->locations_size() != 0) {
    throw std::runtime_error ("Proto trip locations size is: " + std::to_string(trip->locations_size()) + " | Expected size: 0");
  }

  if (trip->has_summary()) {
    throw std::runtime_error ("Proto trip has summary when there is none");
  }

  if (trip->legs_size() != 0) {
    throw std::runtime_error ("Proto trip legs size is: " + std::to_string(trip->legs_size()) + " | Expected size: 0");
  }

  if (trip->has_status_message()) {
    throw std::runtime_error ("Proto trip has status_message when there is none");
  }

  if (trip->has_status()) {
    throw std::runtime_error ("Proto trip has status when there is none");
  }

  if (trip->has_units()) {
    throw std::runtime_error ("Proto trip has units when there is none");
  }

  if (trip->has_language()) {
    throw std::runtime_error ("Proto trip has language when there is none");
  }

  if (trip->has_id()) {
    throw std::runtime_error ("Proto trip has id when there is none");
  }
}

void invalidFields () {
  std::string invalidFields =
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
  Route::Trip* trip = route.mutable_trip();

  if (trip->locations_size() != 0) {
    throw std::runtime_error ("Proto trip locations size is: " + std::to_string(trip->locations_size()) + " | Expected size: 0");
  }

  if (trip->has_summary()) {
    throw std::runtime_error ("Proto trip has summary when there is none");
  }

  if (trip->legs_size() != 0) {
    throw std::runtime_error ("Proto trip legs size is: " + std::to_string(trip->legs_size()) + " | Expected size: 0");
  }

  if (!trip->has_status_message()) {
    throw std::runtime_error ("Proto trip is missing status message");
  }
  if (*trip->mutable_status_message() != "abcdef") {
    throw std::runtime_error ("Status message parsed incorrectly. status_message: " + *trip->mutable_status_message() +
        " | Expected: abcdef");
  }

  if (trip->has_status()) {
    throw std::runtime_error ("Proto trip has status when there is none");
  }

  if (!trip->has_units()) {
    throw std::runtime_error ("Proto trip is missing units");
  }
  if (*trip->mutable_units() != "km") {
    throw std::runtime_error ("Units parsed incorrectly. units: " + *trip->mutable_units() + " | Expected: km");
  }

  if (trip->has_language()) {
    throw std::runtime_error ("Proto trip has language when there is none");
  }

  if (trip->has_id()) {
    throw std::runtime_error ("Proto trip has id when there is none");
  }
}

}

int main() {
  test::suite suite("util_tyr");

  // Empty JSON
  suite.test(TEST_CASE(emptyJsonTest));

  // Empty trip
  suite.test(TEST_CASE(emptyMessageTest));

  // Invalid fields amongst valid ones
  suite.test(TEST_CASE(invalidFields));
}
