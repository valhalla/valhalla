#include <string>
#include <iostream>

#include <boost/algorithm/string/replace.hpp>

#include "proto/route.pb.h"
#include "baldr/location.h"
#include "tyr/navigator.h"

#include "test.h"

using namespace std;
using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::tyr;

namespace {
const std::string kLanguageTag = "<LANGUAGE>";
const std::string kUnitsTag = "<UNITS>";
const std::string kStatusMessageTag = "<STATUS_MESSAGE>";
const std::string kStatusTag = "<STATUS>";
const std::string kIdTag = "<ID>";
const std::string kSummaryLengthTag = "<SUMMARY_LENGTH>";
const std::string kSummaryTimeTag = "<SUMMARY_TIME>";
const std::string kSummaryMinLatTag = "<SUMMARY_MIN_LAT>";
const std::string kSummaryMinLonTag = "<SUMMARY_MIN_LON>";
const std::string kSummaryMaxLatTag = "<SUMMARY_MAX_LAT>";
const std::string kSummaryMaxLonTag = "<SUMMARY_MAX_LON>";
const std::string kLocationOrigLatTag = "<LOCATION_ORIG_LAT>";
const std::string kLocationOrigLonTag = "<LOCATION_ORIG_LON>";
const std::string kLocationOrigTypeTag = "<LOCATION_ORIG_TYPE>";
const std::string kLocationOrigHeadingTag = "<LOCATION_ORIG_HEADING>";
const std::string kLocationOrigNameTag = "<LOCATION_ORIG_NAME>";
const std::string kLocationOrigStreetTag = "<LOCATION_ORIG_STREET>";
const std::string kLocationOrigCityTag = "<LOCATION_ORIG_CITY>";
const std::string kLocationOrigStateTag = "<LOCATION_ORIG_STATE>";
const std::string kLocationOrigPostalCodeTag = "<LOCATION_ORIG_POSTAL_CODE>";
const std::string kLocationOrigCountryTag = "<LOCATION_ORIG_COUNTRY>";
const std::string kLocationOrigDateTimeTag = "<LOCATION_ORIG_DATE_TIME>";
const std::string kLocationOrigSideOfStreetTag = "<LOCATION_ORIG_SIDE_OF_STREET>";
const std::string kLocationOrigOriginalIndexTag = "<LOCATION_ORIG_ORIGINAL_INDEX>";
const std::string kLocationDestLatTag = "<LOCATION_DEST_LAT>";
const std::string kLocationDestLonTag = "<LOCATION_DEST_LON>";
const std::string kLocationDestTypeTag = "<LOCATION_DEST_TYPE>";
const std::string kLocationDestHeadingTag = "<LOCATION_DEST_HEADING>";
const std::string kLocationDestNameTag = "<LOCATION_DEST_NAME>";
const std::string kLocationDestStreetTag = "<LOCATION_DEST_STREET>";
const std::string kLocationDestCityTag = "<LOCATION_DEST_CITY>";
const std::string kLocationDestStateTag = "<LOCATION_DEST_STATE>";
const std::string kLocationDestPostalCodeTag = "<LOCATION_DEST_POSTAL_CODE>";
const std::string kLocationDestCountryTag = "<LOCATION_DEST_COUNTRY>";
const std::string kLocationDestDateTimeTag = "<LOCATION_DEST_DATE_TIME>";
const std::string kLocationDestSideOfStreetTag = "<LOCATION_DEST_SIDE_OF_STREET>";
const std::string kLocationDestOriginalIndexTag = "<LOCATION_DEST_ORIGINAL_INDEX>";
const std::string kLeg1ShapeTag = "<LEG1_SHAPE_TAG>";

// Sub class to test protected methods
class NavigatorTest : public Navigator {
 public:
  NavigatorTest(const std::string& trip_json_str)
      : Navigator(trip_json_str) {
  }

  const valhalla::Route& route() const {
    return Navigator::route();
  }

  void OnLocationChanged(const Location& location) {
    return Navigator::OnLocationChanged(location);
  }
};

void TryTopLevelCtor(const std::string& route_json_str,
    const std::string& expected_language,
    const std::string& expected_units,
    const std::string& expected_status_message,
    uint32_t expected_status,
    const std::string& expected_id) {
  NavigatorTest nt(route_json_str);

  if (nt.route().trip().language() != expected_language)
    throw std::runtime_error("Incorrect language - found: " + nt.route().trip().language() + " | expected: " + expected_language);

  if (nt.route().trip().units() != expected_units)
    throw std::runtime_error("Incorrect units - found: " + nt.route().trip().units() + " | expected: " + expected_units);

  if (nt.route().trip().status_message() != expected_status_message)
    throw std::runtime_error("Incorrect status_message - found: " + nt.route().trip().status_message() + " | expected: " + expected_status_message);

  if (nt.route().trip().status() != expected_status)
    throw std::runtime_error("Incorrect status - found: " + std::to_string(nt.route().trip().status()) + " | expected: " + std::to_string(expected_status));

  if (nt.route().trip().id() != expected_id)
    throw std::runtime_error("Incorrect id - found: " + nt.route().trip().id() + " | expected: " + expected_id);

}

void TestTopLevelCtor() {
  // Tagged route json string
  std::string route_json_str = "{\"trip\":{\"language\":\"" + kLanguageTag
      + "\",\"units\":\"" + kUnitsTag
      + "\",\"status_message\":\"" + kStatusMessageTag
      + "\",\"status\":" + kStatusTag
      + ",\"id\":\"" + kIdTag
      + "\"}}";

  // Expected values
  std::string expected_language = "en-US";
  std::string expected_units = "miles";
  std::string expected_status_message = "Found route between points";
  uint32_t expected_status = 0;
  std::string expected_id = "SIMPLE_TEST";

  // Replace tags with values
  boost::replace_all(route_json_str, kLanguageTag, expected_language);
  boost::replace_all(route_json_str, kUnitsTag, expected_units);
  boost::replace_all(route_json_str, kStatusMessageTag, expected_status_message);
  boost::replace_all(route_json_str, kStatusTag, std::to_string(expected_status));
  boost::replace_all(route_json_str, kIdTag, expected_id);

  TryTopLevelCtor(route_json_str, expected_language, expected_units,
      expected_status_message, expected_status, expected_id);
}

void TryTopLevelSummaryCtor(const std::string& route_json_str,
    float expected_summary_length,
    uint32_t expected_summary_time,
    float expected_summary_min_lat,
    float expected_summary_min_lon,
    float expected_summary_max_lat,
    float expected_summary_max_lon)
{
  NavigatorTest nt(route_json_str);

  if (nt.route().trip().summary().length() != expected_summary_length)
    throw std::runtime_error("Incorrect status - found: " + std::to_string(nt.route().trip().summary().length()) + " | expected: " + std::to_string(expected_summary_length));

  if (nt.route().trip().summary().time() != expected_summary_time)
    throw std::runtime_error("Incorrect status - found: " + std::to_string(nt.route().trip().summary().time()) + " | expected: " + std::to_string(expected_summary_time));

  if (nt.route().trip().summary().min_lat() != expected_summary_min_lat)
    throw std::runtime_error("Incorrect status - found: " + std::to_string(nt.route().trip().summary().min_lat()) + " | expected: " + std::to_string(expected_summary_min_lat));

  if (nt.route().trip().summary().min_lon() != expected_summary_min_lon)
    throw std::runtime_error("Incorrect status - found: " + std::to_string(nt.route().trip().summary().min_lon()) + " | expected: " + std::to_string(expected_summary_min_lon));

  if (nt.route().trip().summary().max_lat() != expected_summary_max_lat)
    throw std::runtime_error("Incorrect status - found: " + std::to_string(nt.route().trip().summary().max_lat()) + " | expected: " + std::to_string(expected_summary_max_lat));

  if (nt.route().trip().summary().max_lon() != expected_summary_max_lon)
    throw std::runtime_error("Incorrect status - found: " + std::to_string(nt.route().trip().summary().max_lon()) + " | expected: " + std::to_string(expected_summary_max_lon));

}

void TestTopLevelSummaryCtor() {
  // Tagged route json string
  std::string route_json_str = "{\"trip\":{\"language\":\"" + kLanguageTag
      + "\",\"units\":\"" + kUnitsTag
      + "\",\"status_message\":\"" + kStatusMessageTag
      + "\",\"status\":" + kStatusTag
      + ",\"id\":\"" + kIdTag
      + "\",\"summary\":{\"max_lon\":" + kSummaryMaxLonTag
      + ",\"max_lat\":" + kSummaryMaxLatTag
      + ",\"time\":" + kSummaryTimeTag
      + ",\"length\":" + kSummaryLengthTag
      + ",\"min_lat\":" + kSummaryMinLatTag
      + ",\"min_lon\":" + kSummaryMinLonTag
      + "}"
      + "}}";

  // Expected values
  std::string expected_language = "en-US";
  std::string expected_units = "miles";
  std::string expected_status_message = "Found route between points";
  uint32_t expected_status = 0;
  std::string expected_id = "SIMPLE_TEST";
  float expected_summary_length = 2.168;
  uint32_t expected_summary_time = 287;
  float expected_summary_min_lat = 40.042500;
  float expected_summary_min_lon = -76.299324;
  float expected_summary_max_lat = 40.063713;
  float expected_summary_max_lon = -76.276863;

  // Replace tags with values
  boost::replace_all(route_json_str, kLanguageTag, expected_language);
  boost::replace_all(route_json_str, kUnitsTag, expected_units);
  boost::replace_all(route_json_str, kStatusMessageTag, expected_status_message);
  boost::replace_all(route_json_str, kStatusTag, std::to_string(expected_status));
  boost::replace_all(route_json_str, kIdTag, expected_id);
  boost::replace_all(route_json_str, kSummaryLengthTag, std::to_string(expected_summary_length));
  boost::replace_all(route_json_str, kSummaryTimeTag, std::to_string(expected_summary_time));
  boost::replace_all(route_json_str, kSummaryMinLatTag, std::to_string(expected_summary_min_lat));
  boost::replace_all(route_json_str, kSummaryMinLonTag, std::to_string(expected_summary_min_lon));
  boost::replace_all(route_json_str, kSummaryMaxLatTag, std::to_string(expected_summary_max_lat));
  boost::replace_all(route_json_str, kSummaryMaxLonTag, std::to_string(expected_summary_max_lon));

  TryTopLevelCtor(route_json_str, expected_language, expected_units,
      expected_status_message, expected_status, expected_id);

  TryTopLevelSummaryCtor(route_json_str, expected_summary_length, expected_summary_time,
      expected_summary_min_lat, expected_summary_min_lon,
      expected_summary_max_lat, expected_summary_max_lon);
}

void TryTopLevelLocationCtor(const std::string& route_json_str,
    size_t expected_location_count,
    const std::string& expected_orig_street,
    float expected_orig_lat,
    float expected_orig_lon,
    const std::string& expected_orig_type,
    const std::string& expected_dest_street,
    float expected_dest_lat,
    float expected_dest_lon,
    const std::string& expected_dest_type,
    const std::string& expected_dest_side_of_street)
{
  NavigatorTest nt(route_json_str);

  int location_count = nt.route().trip().locations_size();
  if (location_count != expected_location_count)
    throw std::runtime_error("Incorrect location count - found: " + std::to_string(location_count) + " | expected: " + std::to_string(expected_location_count));

  auto& orig = nt.route().trip().locations(0);
  if (orig.street() != expected_orig_street)
    throw std::runtime_error("Incorrect orig street - found: " + orig.street() + " | expected: " + expected_orig_street);

  if (orig.lat() != expected_orig_lat)
    throw std::runtime_error("Incorrect orig lat - found: " + std::to_string(orig.lat()) + " | expected: " + std::to_string(expected_orig_lat));

  if (orig.lon() != expected_orig_lon)
    throw std::runtime_error("Incorrect orig lon - found: " + std::to_string(orig.lon()) + " | expected: " + std::to_string(expected_orig_lon));

  if (orig.type() != expected_orig_type)
    throw std::runtime_error("Incorrect orig type - found: " + orig.type() + " | expected: " + expected_orig_type);

  auto& dest = nt.route().trip().locations(1);
  if (dest.street() != expected_dest_street)
    throw std::runtime_error("Incorrect dest street - found: " + dest.street() + " | expected: " + expected_dest_street);

  if (dest.lat() != expected_dest_lat)
    throw std::runtime_error("Incorrect dest lat - found: " + std::to_string(dest.lat()) + " | expected: " + std::to_string(expected_dest_lat));

  if (dest.lon() != expected_dest_lon)
    throw std::runtime_error("Incorrect dest lon - found: " + std::to_string(dest.lon()) + " | expected: " + std::to_string(expected_dest_lon));

  if (dest.type() != expected_dest_type)
    throw std::runtime_error("Incorrect dest type - found: " + dest.type() + " | expected: " + expected_dest_type);

  if (dest.side_of_street() != expected_dest_side_of_street)
    throw std::runtime_error("Incorrect dest side_of_street - found: " + dest.side_of_street() + " | expected: " + expected_dest_side_of_street);
}

void TestTopLevelLocationCtor() {
  // Tagged route json string
  std::string route_json_str = "{\"trip\":{\"language\":\"" + kLanguageTag
      + "\",\"units\":\"" + kUnitsTag
      + "\",\"status_message\":\"" + kStatusMessageTag
      + "\",\"status\":" + kStatusTag
      + ",\"id\":\"" + kIdTag
      + "\",\"summary\":{\"max_lon\":" + kSummaryMaxLonTag
      + ",\"max_lat\":" + kSummaryMaxLatTag
      + ",\"time\":" + kSummaryTimeTag
      + ",\"length\":" + kSummaryLengthTag
      + ",\"min_lat\":" + kSummaryMinLatTag
      + ",\"min_lon\":" + kSummaryMinLonTag
      + "}"
      + ",\"locations\":[{\"street\":\"" + kLocationOrigStreetTag
      + "\",\"lon\":" + kLocationOrigLonTag
      + ",\"lat\":" + kLocationOrigLatTag
      + ",\"type\":\"" + kLocationOrigTypeTag
      + "\"},{\"type\":\"" + kLocationDestTypeTag
      + "\",\"side_of_street\":\"" + kLocationDestSideOfStreetTag
      + "\",\"lat\":" + kLocationDestLatTag
      + ",\"lon\":" + kLocationDestLonTag
      + ",\"street\":\"" + kLocationDestStreetTag
      + "\"}]"
      + "}}";

  ////////////////////////////////////////////////////////////////////////////
  // Expected top level values
  std::string expected_language = "en-US";
  std::string expected_units = "miles";
  std::string expected_status_message = "Found route between points";
  uint32_t expected_status = 0;
  std::string expected_id = "SIMPLE_TEST";

  // Expected top level summary values
  float expected_summary_length = 2.168;
  uint32_t expected_summary_time = 287;
  float expected_summary_min_lat = 40.042500;
  float expected_summary_min_lon = -76.299324;
  float expected_summary_max_lat = 40.063713;
  float expected_summary_max_lon = -76.276863;

  // Expected top level location values
  size_t expected_location_count = 2;
  std::string expected_orig_street = "East Fulton Street";
  float expected_orig_lat = 40.042538;
  float expected_orig_lon = -76.299332;
  std::string expected_orig_type = "break";
  std::string expected_dest_street = "US 30 West";
  float expected_dest_lat = 40.063747;
  float expected_dest_lon = -76.282028;
  std::string expected_dest_type = "break";
  std::string expected_dest_side_of_street = "right";

  ////////////////////////////////////////////////////////////////////////////
  // Replace tags with values
  boost::replace_all(route_json_str, kLanguageTag, expected_language);
  boost::replace_all(route_json_str, kUnitsTag, expected_units);
  boost::replace_all(route_json_str, kStatusMessageTag, expected_status_message);
  boost::replace_all(route_json_str, kStatusTag, std::to_string(expected_status));
  boost::replace_all(route_json_str, kIdTag, expected_id);
  boost::replace_all(route_json_str, kSummaryLengthTag, std::to_string(expected_summary_length));
  boost::replace_all(route_json_str, kSummaryTimeTag, std::to_string(expected_summary_time));
  boost::replace_all(route_json_str, kSummaryMinLatTag, std::to_string(expected_summary_min_lat));
  boost::replace_all(route_json_str, kSummaryMinLonTag, std::to_string(expected_summary_min_lon));
  boost::replace_all(route_json_str, kSummaryMaxLatTag, std::to_string(expected_summary_max_lat));
  boost::replace_all(route_json_str, kSummaryMaxLonTag, std::to_string(expected_summary_max_lon));
  boost::replace_all(route_json_str, kLocationOrigStreetTag, expected_orig_street);
  boost::replace_all(route_json_str, kLocationOrigLatTag, std::to_string(expected_orig_lat));
  boost::replace_all(route_json_str, kLocationOrigLonTag, std::to_string(expected_orig_lon));
  boost::replace_all(route_json_str, kLocationOrigTypeTag, expected_orig_type);
  boost::replace_all(route_json_str, kLocationDestStreetTag, expected_dest_street);
  boost::replace_all(route_json_str, kLocationDestLatTag, std::to_string(expected_dest_lat));
  boost::replace_all(route_json_str, kLocationDestLonTag, std::to_string(expected_dest_lon));
  boost::replace_all(route_json_str, kLocationDestTypeTag, expected_dest_type);
  boost::replace_all(route_json_str, kLocationDestSideOfStreetTag, expected_dest_side_of_street);

  TryTopLevelCtor(route_json_str, expected_language, expected_units,
      expected_status_message, expected_status, expected_id);

  TryTopLevelSummaryCtor(route_json_str, expected_summary_length, expected_summary_time,
      expected_summary_min_lat, expected_summary_min_lon,
      expected_summary_max_lat, expected_summary_max_lon);

  TryTopLevelLocationCtor(route_json_str, expected_location_count,
      expected_orig_street, expected_orig_lat, expected_orig_lon,
      expected_orig_type, expected_dest_street, expected_dest_lat,
      expected_dest_lon, expected_dest_type, expected_dest_side_of_street);
}

void TryTopLevelLegCtor(const std::string& route_json_str,
    size_t expected_leg_maneuver_count,
    const std::string& expected_leg_shape)
{
  NavigatorTest nt(route_json_str);

  auto& leg = nt.route().trip().legs(0);

  int maneuver_count = leg.maneuvers_size();
  if (maneuver_count != expected_leg_maneuver_count)
    throw std::runtime_error("Incorrect maneuver count - found: " + std::to_string(maneuver_count) + " | expected: " + std::to_string(expected_leg_maneuver_count));

  if (leg.shape() != expected_leg_shape)
    throw std::runtime_error("Incorrect leg shape - found: " + leg.shape() + " | expected: " + expected_leg_shape);
}

void TestTopLevelLegCtor() {
  // Tagged route json string
  std::string route_json_str = "{\"trip\":{\"language\":\"" + kLanguageTag
      + "\",\"units\":\"" + kUnitsTag
      + "\",\"status_message\":\"" + kStatusMessageTag
      + "\",\"status\":" + kStatusTag
      + ",\"id\":\"" + kIdTag
      + "\",\"summary\":{\"max_lon\":" + kSummaryMaxLonTag
      + ",\"max_lat\":" + kSummaryMaxLatTag
      + ",\"time\":" + kSummaryTimeTag
      + ",\"length\":" + kSummaryLengthTag
      + ",\"min_lat\":" + kSummaryMinLatTag
      + ",\"min_lon\":" + kSummaryMinLonTag
      + "}"
      + ",\"locations\":[{\"street\":\"" + kLocationOrigStreetTag
      + "\",\"lon\":" + kLocationOrigLonTag
      + ",\"lat\":" + kLocationOrigLatTag
      + ",\"type\":\"" + kLocationOrigTypeTag
      + "\"},{\"type\":\"" + kLocationDestTypeTag
      + "\",\"side_of_street\":\"" + kLocationDestSideOfStreetTag
      + "\",\"lat\":" + kLocationDestLatTag
      + ",\"lon\":" + kLocationDestLonTag
      + ",\"street\":\"" + kLocationDestStreetTag
      + "\"}]"
      + ",\"legs\":[{\"shape\":\"" + kLeg1ShapeTag
      + "\",\"summary\":{\"max_lon\":-76.276863,\"max_lat\":40.063713,\"time\":287,\"length\":2.168,\"min_lat\":40.042500,\"min_lon\":-76.299324}"
      + ",\"maneuvers\":[{\"travel_mode\":\"drive\",\"begin_shape_index\":0,\"length\":0.081,\"time\":16,\"type\":1,\"end_shape_index\":2,\"instruction\":\"Drive east on East Fulton Street.\",\"verbal_pre_transition_instruction\":\"Drive east on East Fulton Street for 400 feet.\",\"travel_type\":\"car\",\"street_names\":[\"East Fulton Street\"]},{\"travel_type\":\"car\",\"travel_mode\":\"drive\",\"verbal_pre_transition_instruction\":\"Turn left onto North Plum Street.\",\"verbal_transition_alert_instruction\":\"Turn left onto North Plum Street.\",\"length\":0.176,\"instruction\":\"Turn left onto North Plum Street.\",\"end_shape_index\":5,\"type\":15,\"time\":58,\"verbal_post_transition_instruction\":\"Continue for 2 tenths of a mile.\",\"street_names\":[\"North Plum Street\"],\"begin_shape_index\":2},{\"travel_type\":\"car\",\"travel_mode\":\"drive\",\"verbal_pre_transition_instruction\":\"Bear right onto New Holland Avenue.\",\"verbal_transition_alert_instruction\":\"Bear right onto New Holland Avenue.\",\"length\":1.276,\"instruction\":\"Bear right onto New Holland Avenue.\",\"end_shape_index\":29,\"type\":9,\"time\":124,\"verbal_post_transition_instruction\":\"Continue for 1.3 miles.\",\"street_names\":[\"New Holland Avenue\"],\"begin_shape_index\":5},{\"travel_type\":\"car\",\"verbal_pre_transition_instruction\":\"Continue on New Holland Pike for 3 tenths of a mile.\",\"verbal_transition_alert_instruction\":\"Continue on New Holland Pike.\",\"length\":0.284,\"instruction\":\"Continue on New Holland Pike.\",\"end_shape_index\":39,\"type\":8,\"time\":35,\"street_names\":[\"New Holland Pike\"],\"begin_shape_index\":29,\"travel_mode\":\"drive\"},{\"travel_type\":\"car\",\"verbal_pre_transition_instruction\":\"Turn left to take the U.S. 30 West ramp toward York, Harrisburg.\",\"verbal_transition_alert_instruction\":\"Turn left to take the U.S. 30 West ramp.\",\"instruction\":\"Turn left to take the US 30 West ramp toward York/Harrisburg.\",\"end_shape_index\":53,\"type\":19,\"time\":48,\"begin_shape_index\":39,\"length\":0.327,\"sign\":{\"exit_toward_elements\":[{\"text\":\"York\"},{\"text\":\"Harrisburg\"}],\"exit_branch_elements\":[{\"consecutive_count\":1,\"text\":\"US 30 West\"}]},\"travel_mode\":\"drive\"},{\"travel_type\":\"car\",\"travel_mode\":\"drive\",\"verbal_pre_transition_instruction\":\"Merge onto U.S. 30 West. Then U.S. 30 West will be on the right.\",\"verbal_post_transition_instruction\":\"Continue for 100 feet.\",\"instruction\":\"Merge onto US 30 West.\",\"end_shape_index\":54,\"type\":25,\"time\":6,\"verbal_multi_cue\":true,\"street_names\":[\"US 30 West\"],\"length\":0.023,\"begin_shape_index\":53},{\"travel_type\":\"car\",\"travel_mode\":\"drive\",\"begin_shape_index\":54,\"time\":0,\"type\":5,\"end_shape_index\":54,\"instruction\":\"US 30 West is on the right.\",\"length\":0.000,\"verbal_transition_alert_instruction\":\"U.S. 30 West will be on the right.\",\"verbal_pre_transition_instruction\":\"U.S. 30 West is on the right.\"}]"
      + "}]"
      + "}}";

  ////////////////////////////////////////////////////////////////////////////
  // Expected top level values
  std::string expected_language = "en-US";
  std::string expected_units = "miles";
  std::string expected_status_message = "Found route between points";
  uint32_t expected_status = 0;
  std::string expected_id = "SIMPLE_TEST";

  // Expected top level summary values
  float expected_summary_length = 2.168;
  uint32_t expected_summary_time = 287;
  float expected_summary_min_lat = 40.042500;
  float expected_summary_min_lon = -76.299324;
  float expected_summary_max_lat = 40.063713;
  float expected_summary_max_lon = -76.276863;

  // Expected top level location values
  size_t expected_location_count = 2;
  std::string expected_orig_street = "East Fulton Street";
  float expected_orig_lat = 40.042538;
  float expected_orig_lon = -76.299332;
  std::string expected_orig_type = "break";
  std::string expected_dest_street = "US 30 West";
  float expected_dest_lat = 40.063747;
  float expected_dest_lon = -76.282028;
  std::string expected_dest_type = "break";
  std::string expected_dest_side_of_street = "right";

  // Expected top level leg values
  size_t expected_leg_maneuver_count = 7;
  std::string leg_shape = "e__kkAxb}opCcBq\\\\sFk`Aqp@`HqRxBexAfNwHk@y`AgwA{KePed@qp@iCwDwM_S{z@snAemA}gByoAekBqGoJgJwMiGmJiv@}hAil@q{@eU_]s_@{j@gYwc@mi@}}@qMiW{F}JcPwX}Ymi@}Zmi@yoAqxBiHwNqk@gbA_Xqf@aDkKyVee@}OiWqBwCwI{Ks[a\\\\iByB}D`HkGjKoHfNmEnIuE|IsGfOkUhl@{KvWeK~SaBvDaNhVwRp]c[li@eUrd@sL`S";
  std::string expected_leg_shape = "e__kkAxb}opCcBq\\sFk`Aqp@`HqRxBexAfNwHk@y`AgwA{KePed@qp@iCwDwM_S{z@snAemA}gByoAekBqGoJgJwMiGmJiv@}hAil@q{@eU_]s_@{j@gYwc@mi@}}@qMiW{F}JcPwX}Ymi@}Zmi@yoAqxBiHwNqk@gbA_Xqf@aDkKyVee@}OiWqBwCwI{Ks[a\\iByB}D`HkGjKoHfNmEnIuE|IsGfOkUhl@{KvWeK~SaBvDaNhVwRp]c[li@eUrd@sL`S";

  ////////////////////////////////////////////////////////////////////////////
  // Replace tags with values
  boost::replace_all(route_json_str, kLanguageTag, expected_language);
  boost::replace_all(route_json_str, kUnitsTag, expected_units);
  boost::replace_all(route_json_str, kStatusMessageTag, expected_status_message);
  boost::replace_all(route_json_str, kStatusTag, std::to_string(expected_status));
  boost::replace_all(route_json_str, kIdTag, expected_id);
  boost::replace_all(route_json_str, kSummaryLengthTag, std::to_string(expected_summary_length));
  boost::replace_all(route_json_str, kSummaryTimeTag, std::to_string(expected_summary_time));
  boost::replace_all(route_json_str, kSummaryMinLatTag, std::to_string(expected_summary_min_lat));
  boost::replace_all(route_json_str, kSummaryMinLonTag, std::to_string(expected_summary_min_lon));
  boost::replace_all(route_json_str, kSummaryMaxLatTag, std::to_string(expected_summary_max_lat));
  boost::replace_all(route_json_str, kSummaryMaxLonTag, std::to_string(expected_summary_max_lon));
  boost::replace_all(route_json_str, kLocationOrigStreetTag, expected_orig_street);
  boost::replace_all(route_json_str, kLocationOrigLatTag, std::to_string(expected_orig_lat));
  boost::replace_all(route_json_str, kLocationOrigLonTag, std::to_string(expected_orig_lon));
  boost::replace_all(route_json_str, kLocationOrigTypeTag, expected_orig_type);
  boost::replace_all(route_json_str, kLocationDestStreetTag, expected_dest_street);
  boost::replace_all(route_json_str, kLocationDestLatTag, std::to_string(expected_dest_lat));
  boost::replace_all(route_json_str, kLocationDestLonTag, std::to_string(expected_dest_lon));
  boost::replace_all(route_json_str, kLocationDestTypeTag, expected_dest_type);
  boost::replace_all(route_json_str, kLocationDestSideOfStreetTag, expected_dest_side_of_street);
  boost::replace_all(route_json_str, kLeg1ShapeTag, leg_shape);

  std::cout << "route_json_str=" << route_json_str << std::endl;

  TryTopLevelCtor(route_json_str, expected_language, expected_units,
      expected_status_message, expected_status, expected_id);

  TryTopLevelSummaryCtor(route_json_str, expected_summary_length, expected_summary_time,
      expected_summary_min_lat, expected_summary_min_lon,
      expected_summary_max_lat, expected_summary_max_lon);

  TryTopLevelLocationCtor(route_json_str, expected_location_count,
      expected_orig_street, expected_orig_lat, expected_orig_lon,
      expected_orig_type, expected_dest_street, expected_dest_lat,
      expected_dest_lon, expected_dest_type, expected_dest_side_of_street);

  TryTopLevelLegCtor(route_json_str, expected_leg_maneuver_count,
      expected_leg_shape);
}

}

int main() {
  test::suite suite("navigator");

  // TopLevelCtor
  suite.test(TEST_CASE(TestTopLevelCtor));

  // TopLevelSummaryCtor
  suite.test(TEST_CASE(TestTopLevelSummaryCtor));

  // TopLevelLocationCtor
  suite.test(TEST_CASE(TestTopLevelLocationCtor));

  // TopLevelLegCtor
  suite.test(TEST_CASE(TestTopLevelLegCtor));

  return suite.tear_down();
}
