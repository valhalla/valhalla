#include <string>
#include <iostream>

#include <boost/algorithm/string/replace.hpp>

#include "proto/route.pb.h"
#include "proto/navigator.pb.h"
#include "baldr/location.h"
#include "tyr/navigator.h"
#include "midgard/pointll.h"
#include "midgard/util.h"

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

  const Route& route() const {
    return Navigator::route();
  }

  const std::vector<std::pair<float, uint32_t>>& remaining_leg_values() const {
    return Navigator::remaining_leg_values_;
  }

  const size_t leg_index() const {
    return Navigator::leg_index_;
  }

  NavigationStatus OnLocationChanged(const FixLocation& fix_location) {
    return Navigator::OnLocationChanged(fix_location);
  }

  bool HasKilometerUnits() const {
    return Navigator::HasKilometerUnits();
  }

  bool IsDestinationShapeIndex(size_t idx) const {
    return Navigator::IsDestinationShapeIndex(idx);
  }

  void SnapToRoute(const FixLocation& fix_location,
       NavigationStatus& nav_status) {
    return Navigator::SnapToRoute(fix_location, nav_status);
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

void TryRouteLegCount(NavigatorTest& nav, int expected_leg_count) {

  int leg_count = nav.route().trip().legs_size();

  if (leg_count != expected_leg_count)
    throw std::runtime_error("Incorrect leg count for route - found: " + std::to_string(leg_count) + " | expected: " + std::to_string(expected_leg_count));
}

void TryRouteManeuverCount(NavigatorTest& nav, uint32_t leg_index,
    int expected_leg_maneuver_count) {

  int maneuver_count = nav.route().trip().legs(leg_index).maneuvers_size();

  if (maneuver_count != expected_leg_maneuver_count)
    throw std::runtime_error("Incorrect maneuver count for route - found: " + std::to_string(maneuver_count) + " | expected: " + std::to_string(expected_leg_maneuver_count));
}

void TryRouteUnits(NavigatorTest& nav, std::string expected_units, bool expected_units_boolean) {

  std::string units = nav.route().trip().units();
  bool units_boolean = nav.HasKilometerUnits();

  if (units != expected_units)
    throw std::runtime_error("Incorrect units for route - found: " + units + " | expected: " + expected_units);

  if (units_boolean != expected_units_boolean)
    throw std::runtime_error("Incorrect units boolean for route - found: " + (units_boolean ? std::string("true") : std::string("false")) + " | expected: " + (expected_units_boolean ? std::string("true") : std::string("false")));
}

void TryRouteLanguage(NavigatorTest& nav, std::string expected_language) {

  std::string language = nav.route().trip().language();

  if (language != expected_language)
    throw std::runtime_error("Incorrect language for route - found: " + language + " | expected: " + expected_language);
}

void TryRemainingLegValues(NavigatorTest& nav, uint32_t index,
    float expected_remaining_leg_length, uint32_t expected_remaining_leg_time) {

  float remaining_leg_length = nav.remaining_leg_values().at(index).first;
  uint32_t remaining_leg_time = nav.remaining_leg_values().at(index).second;

  if (!valhalla::midgard::equal<float>(remaining_leg_length, expected_remaining_leg_length, 0.005f))
    throw std::runtime_error("Incorrect remaining leg length - found: " + std::to_string(remaining_leg_length) + " | expected: " + std::to_string(expected_remaining_leg_length));

  if (!valhalla::midgard::equal<float>(remaining_leg_time, expected_remaining_leg_time, (expected_remaining_leg_time*0.0066f)))
    throw std::runtime_error("Incorrect remaining leg time - found: " + std::to_string(remaining_leg_time) + " | expected: " + std::to_string(expected_remaining_leg_time));
}

FixLocation GetFixLocation(float lon, float lat, uint64_t time) {
  FixLocation fix;
  fix.set_lon(lon);
  fix.set_lat(lat);
  fix.set_time(time);
  return fix;
}

FixLocation GetFixLocation(float lon, float lat, uint64_t time, float speed) {
  FixLocation fix;
  fix.set_lon(lon);
  fix.set_lat(lat);
  fix.set_time(time);
  fix.set_speed(speed);
  return fix;
}

FixLocation GetFixLocation(float lon, float lat, uint64_t time, float speed,
    float bearing, float altitude, float accuracy, std::string provider) {
  FixLocation fix;
  fix.set_lon(lon);
  fix.set_lat(lat);
  fix.set_time(time);
  fix.set_speed(speed);
  fix.set_bearing(bearing);
  fix.set_altitude(altitude);
  fix.set_accuracy(accuracy);
  fix.set_provider(provider);
  return fix;
}

NavigationStatus GetNavigationStatus(NavigationStatus_RouteState route_state) {
  NavigationStatus nav_status;
  nav_status.set_route_state(route_state);
  return nav_status;
}

NavigationStatus GetNavigationStatus(NavigationStatus_RouteState route_state,
    float lon, float lat, uint32_t leg_index, float remaining_leg_length,
    uint32_t remaining_leg_time, uint32_t maneuver_index,
    float remaining_maneuver_length, uint32_t remaining_maneuver_time) {
  NavigationStatus nav_status;
  nav_status.set_route_state(route_state);
  nav_status.set_lon(lon);
  nav_status.set_lat(lat);
  nav_status.set_leg_index(leg_index);
  nav_status.set_remaining_leg_length(remaining_leg_length);
  nav_status.set_remaining_leg_time(remaining_leg_time);
  nav_status.set_maneuver_index(maneuver_index);
  nav_status.set_remaining_maneuver_length(remaining_maneuver_length);
  nav_status.set_remaining_maneuver_time(remaining_maneuver_time);
  return nav_status;
}

void ValidateNavigationStatus(const NavigationStatus& nav_status,
    const NavigationStatus& expected_nav_status) {

  if (nav_status.route_state() != expected_nav_status.route_state())
    throw std::runtime_error("Incorrect route state for NavigationStatus - found: " + NavigationStatus_RouteState_Name(nav_status.route_state()) + " | expected: " + NavigationStatus_RouteState_Name(expected_nav_status.route_state()));

  if (!valhalla::midgard::equal<float>(nav_status.lat(), expected_nav_status.lat(), 0.00002f))
    throw std::runtime_error("Incorrect lat for NavigationStatus - found: " + std::to_string(nav_status.lat()) + " | expected: " + std::to_string(expected_nav_status.lat()));

  if (!valhalla::midgard::equal<float>(nav_status.lon(), expected_nav_status.lon(), 0.00002f))
    throw std::runtime_error("Incorrect lon for NavigationStatus - found: " + std::to_string(nav_status.lon()) + " | expected: " + std::to_string(expected_nav_status.lon()));

  if (nav_status.leg_index() != expected_nav_status.leg_index())
    throw std::runtime_error("Incorrect leg_index for NavigationStatus - found: " + std::to_string(nav_status.leg_index()) + " | expected: " + std::to_string(expected_nav_status.leg_index()));

  if (!valhalla::midgard::equal<float>(nav_status.remaining_leg_length(), expected_nav_status.remaining_leg_length(), 0.005f))
    throw std::runtime_error("Incorrect remaining_leg_length for NavigationStatus - found: " + std::to_string(nav_status.remaining_leg_length()) + " | expected: " + std::to_string(expected_nav_status.remaining_leg_length()));

  if (!valhalla::midgard::equal<float>(nav_status.remaining_leg_time(), expected_nav_status.remaining_leg_time(), (expected_nav_status.remaining_leg_time() * 0.0066f)))
    throw std::runtime_error("Incorrect remaining_leg_time for NavigationStatus - found: " + std::to_string(nav_status.remaining_leg_time()) + " | expected: " + std::to_string(expected_nav_status.remaining_leg_time()));

  if (nav_status.maneuver_index() != expected_nav_status.maneuver_index())
    throw std::runtime_error("Incorrect maneuver_index for NavigationStatus - found: " + std::to_string(nav_status.maneuver_index()) + " | expected: " + std::to_string(expected_nav_status.maneuver_index()));

  if (!valhalla::midgard::equal<float>(nav_status.remaining_maneuver_length(), expected_nav_status.remaining_maneuver_length(), 0.005f))
    throw std::runtime_error("Incorrect remaining_maneuver_length for NavigationStatus - found: " + std::to_string(nav_status.remaining_maneuver_length()) + " | expected: " + std::to_string(expected_nav_status.remaining_maneuver_length()));

  if (!valhalla::midgard::equal<float>(nav_status.remaining_maneuver_time(), expected_nav_status.remaining_maneuver_time(), (expected_nav_status.remaining_maneuver_time() * 0.0066f)))
    throw std::runtime_error("Incorrect remaining_maneuver_time for NavigationStatus - found: " + std::to_string(nav_status.remaining_maneuver_time()) + " | expected: " + std::to_string(expected_nav_status.remaining_maneuver_time()));
}

void TrySnapToRoute(NavigatorTest& nav, const FixLocation& fix_location,
    NavigationStatus expected_nav_status) {
  NavigationStatus nav_status;
  nav.SnapToRoute(fix_location, nav_status);
  ValidateNavigationStatus(nav_status, expected_nav_status);
}

void TryRouteOnLocationChanged(NavigatorTest& nav, const FixLocation& fix,
    const NavigationStatus& expected_nav_status) {
  NavigationStatus nav_status = nav.OnLocationChanged(fix);
  ValidateNavigationStatus(nav_status, expected_nav_status);
}

void TestLancasterToHershey() {
  std::string route_json_str = "{\"trip\":{\"language\":\"en-US\",\"summary\":{\"max_lon\":-76.266228,\"max_lat\":40.283943,\"time\":2438,\"length\":31.322,\"min_lat\":40.042015,\"min_lon\":-76.664360},\"locations\":[{\"lon\":-76.299179,\"lat\":40.042572,\"type\":\"break\"},{\"lon\":-76.654625,\"lat\":40.283924,\"type\":\"break\"}],\"units\":\"miles\",\"legs\":[{\"shape\":\"k`_kkAfy|opC}@_SsFk`A~g@cFeDmt@{FkiAkLisBmJcxBsKcxBaHytAWaIqBm^u@wNsAo]yAyMsAqGyBsFiCeEeZs[iBiBsA{AgDyC{QsPaMiMcFaH}@yA{@{AgDoIgDyLU}@eAcFcAmJm@oIe@_JiBwXOaGm@mKu@{K{@oHcBkLgCeOiC}JoDyLaCaIuDgMeF}J}IuOaMqRcLwNgDcFiHcGoNkLke@y`@uDyCu@MeEgDuOqHsKuEuEiBcu@gX_EkB{~@a\\\\}JeEc[yM}IgDoNcFsFyBcGgDqGeEiHcG{FcGcGoHmEqHsEmJoD}JqC{KaCiLyAkLeKo|@aCsPaCuOoCuO_DePgI_^uy@ejDaGiWoTi`AkKgc@mZapAaWucAmEzAcB\\\\iBl@qLdFqXzKkFvCuEtDmDtEyCrFoCpH{A|Iu@lJEnJj@zJbBlJ`C`HhHrQtD|JxBnIbBhLt@pGd@~IDpHElJe@zJsA|JaCzKqCnIsZ~r@{KrZow@nfB}JhW{Vzi@cVbf@sZjk@s`@`q@{iArlB}c@ju@iHxLoHhMg}@`{AiWx`@_r@jjAya@~q@ePxWcGxLwCdF_r@bnAkLzUkLhWkFjLgSli@aHjVkQzt@wCdO{Fb\\\\cF~]iCjUeFxk@{@dPkAhWu@tZe@`\\\\Nz_Ad@fkAd@liAGxgCkAxkAoDhjA{F`fA{F|s@mEdn@gb@znFyHhjAaGl}@qRjhCuUjeDaRpyByBdZgNhtB{@z_@cBfc@cBdn@eEvbA{FdwA{@nScBtOgCnTaDbQs@rF_DhWcGtc@aXh_BwC`RcB|JoM`q@yHb[kFlU}Jj`@aG|TiHzVaHzTiHpSwNx`@}IxW}bAhrCqW`r@mPbe@cK`]gIlT_Oha@gD~IgNz_@mx@n{Bi\\\\z~@yRli@i`AbmC}EtNoHlUqMvb@}Jl_@qGzUkB~HeEbQgI|_@wHx`@{AlKeOpeAcBvNaH`p@iCtZqB~SgDdc@aCvXcUpnCeAvMiGju@gYjqDaCzUmEjj@mc@ffFoIpeAgNx_B_JjiAoH~|@aHpq@oDfYuJ|r@kFb[yHja@uIxa@kGfXuTrz@uJp[cQ|i@erAjpDsGtOsEjL_Yfw@_Oz_@s@zB{Ll^s@hCoSzi@gEjLiGdPwm@pdB{d@rnAaSjj@oNj`@oMb\\\\iRzi@{Lb\\\\{JfXakApbDyWls@ib@hjAed@dnAgg@huA_Tvm@qMfb@wNli@iLde@m@fCgNrp@eJ`g@}EtYoIro@aHhk@G|@_Dp\\\\oDbe@}Dre@yBb\\\\oSdkCcL~zAeOtlB_J|hA_Idx@iCzUwH~h@iCtOoNvw@aMvl@qGfXwI`]oHxWmFpQ]jA_Srp@uUhv@iL~]mZbdAwIr[u^dmAgS~q@ke@|}A__@`oAeOrf@aWvw@sKz_@_JdZgDxLgCzK{nAldEwqA|mEiRhl@{K~]_Ozi@mc@pzAenA~bEeEfN{F`R{e@x_B{EvN}Y~{@gOz`@uc@jiAe~@~yBmO~^s`@~eAeKzViQdd@cRbe@aa@tcAscBdiEkLdYyVxl@_IpR{Qre@y[by@sVbp@y{@rxB}m@l|AmTjk@sxBdrFqQdc@eKxWs`@dcA{KhXmOz_@w|@nzBei@jtAya@rcAkBdE{EzK}_@xaA}Ovb@erAjfDcKvWyH|TqQ|i@gItYcQ`q@wI|^cGfYuJ|h@aG`]_J`q@{Ftd@sFli@aBnSaC`\\\\wCrf@m@vMu@`Se@nScAz_@u@li@OlUN~]G~\\\\NbQbApg@hCby@pB|_@vRx|CjBtYlEdo@pGbeAnDli@hCvb@vHzhAtJj~AxG`fApChb@bArPrArPj[naFzj@rzInCjj@|@|TbAl_@l@l_@TlTNfc@?z`@e@tYcBz~@kAtYaBn^oDfm@iCdZiHxu@gw@jwGor@n`G_TpdB{o@~uFuTjiBsPtwAiGrd@{F~^oIbe@oD`RwIha@eJl`@_N~f@oIvYyMxa@yL~\\\\sPvc@wNn]{P|^{Pn]wX`g@yQtZ{QdYqoFjaIqMpRaoAfkBoXvb@cRvXaGlJ{hB`mCwNnTs_@hk@wqArnB{z@rnAopApnBe_@|i@a]~g@_StYaMrQusA`nB_vApoBckIrlLcyAhtB_Yj`@g]`f@md@to@gYx`@gpAxiBqaAlsAqG|Iom@p{@ioB|oC{JvNqHjKwCtE}s@tcA{j@vw@m_@|h@uYhb@evBlzCucAvvAqzArwByGbGanCdtD}IlJ_Y`]mTxVgI~HoT`SoNhLgN|J}s@pf@yGtEgIdFwCz@cBN{ELiCMqR{AyGkAmJ{AeKyB{K{AoMm@yWm@uON{P?wSNyRl@oXhBmThBeKzAiGjA_c@lKsV~HwMrFaNpGmn@tZ{UhLucAz`@}iDtlBkKrFytK`tFuqBreAitA|r@wm@fY}T|Jsj@`Q{[~Ice@jLm_IjgB}i@~Sqp@rQuy@pQuuBbp@m}Ahv@u^r[}hAbdAqq@~]w_Bp{@_n@`HyoEeFy`AtFauBj`@ggEdmA{cAtY}mA`]ePdE}kBnh@kGhBmIvCm~@zVk[lJk[nI}IhBeo@rQ_IxBmU`GyGjBeJhCkxBtm@i\\\\|J}wAl_@}JxBmJxBmJhBuJzAeJjAmJjAcVjBsf@jAw\\\\Lud@N}d@l@iu@l@e_@z@ca@Nun@jAgNNcf@?{o@z@_]^ib@pGob@xLsQpGcP`IcVbPo`CliAiLrEwNrGyf@l^_d@`g@kPlUqM|TcQ|^sZ|}@{Vzt@yQzj@oNfc@kKzUaIhMaLvNqMhLyMxLsPpRiq@haAe}A|eC}IvMyQvXaM`SsLdOog@pg@uOfNmUzUsz@zt@{n@jj@aw@vw@uDtE_JjLkF`HwC`HgE`Hye@z~@cyAr`Dq]ns@aLvXs{D`qIkKlToIxM}JhLqLbGaNbFyu@p]i\\\\bo@mJbQ_{BzdE{nAn{B_Ylh@y`@bz@gTfd@qjAgbAy\\\\iXif@ya@cLwM_ImJoIyLmE_JkFkKaI}TsK}_@}EwM{EmJqb@k~AuOkk@e^ksAgIuYcLwb@eJq\\\\mFaS\",\"summary\":{\"max_lon\":-76.266228,\"max_lat\":40.283943,\"time\":2438,\"length\":31.322,\"min_lat\":40.042015,\"min_lon\":-76.664360},\"maneuvers\":[{\"travel_mode\":\"drive\",\"begin_shape_index\":0,\"length\":0.073,\"time\":14,\"type\":1,\"end_shape_index\":2,\"instruction\":\"Drive east on East Fulton Street.\",\"verbal_pre_transition_instruction\":\"Drive east on East Fulton Street for 400 feet.\",\"travel_type\":\"car\",\"street_names\":[\"East Fulton Street\"]},{\"travel_type\":\"car\",\"travel_mode\":\"drive\",\"verbal_multi_cue\":true,\"verbal_pre_transition_instruction\":\"Turn right onto North Plum Street. Then Turn left onto East Chestnut Street.\",\"verbal_transition_alert_instruction\":\"Turn right onto North Plum Street.\",\"length\":0.046,\"instruction\":\"Turn right onto North Plum Street.\",\"end_shape_index\":3,\"type\":10,\"time\":34,\"verbal_post_transition_instruction\":\"Continue for 200 feet.\",\"street_names\":[\"North Plum Street\"],\"begin_shape_index\":2},{\"travel_type\":\"car\",\"travel_mode\":\"drive\",\"end_shape_index\":89,\"verbal_pre_transition_instruction\":\"Turn left onto East Chestnut Street, Pennsylvania 23 East.\",\"begin_street_names\":[\"East Chestnut Street\",\"PA 23 East\"],\"verbal_transition_alert_instruction\":\"Turn left onto East Chestnut Street.\",\"length\":2.054,\"instruction\":\"Turn left onto East Chestnut Street/PA 23 East. Continue on PA 23 East.\",\"type\":15,\"time\":213,\"verbal_post_transition_instruction\":\"Continue on Pennsylvania 23 East for 2.1 miles.\",\"street_names\":[\"PA 23 East\"],\"begin_shape_index\":3},{\"travel_type\":\"car\",\"travel_mode\":\"drive\",\"verbal_pre_transition_instruction\":\"Turn left to take the U.S. 30 West ramp toward New Holland, Harrisburg.\",\"verbal_transition_alert_instruction\":\"Turn left to take the U.S. 30 West ramp.\",\"instruction\":\"Turn left to take the US 30 West ramp toward New Holland/Harrisburg.\",\"end_shape_index\":119,\"type\":19,\"time\":45,\"street_names\":[\"PA 23 East\"],\"begin_shape_index\":89,\"length\":0.375,\"sign\":{\"exit_toward_elements\":[{\"text\":\"New Holland\"},{\"text\":\"Harrisburg\"}],\"exit_branch_elements\":[{\"consecutive_count\":1,\"text\":\"US 30 West\"},{\"text\":\"PA 23 East\"}]}},{\"travel_type\":\"car\",\"verbal_pre_transition_instruction\":\"Merge onto U.S. 30 West.\",\"verbal_post_transition_instruction\":\"Continue for 2.7 miles.\",\"instruction\":\"Merge onto US 30 West.\",\"end_shape_index\":169,\"type\":25,\"time\":164,\"street_names\":[\"US 30 West\"],\"length\":2.746,\"begin_shape_index\":119,\"travel_mode\":\"drive\"},{\"travel_type\":\"car\",\"travel_mode\":\"drive\",\"verbal_pre_transition_instruction\":\"Keep left to take Pennsylvania 2 83 West toward Harrisburg.\",\"verbal_transition_alert_instruction\":\"Keep left to take Pennsylvania 2 83 West.\",\"sign\":{\"exit_toward_elements\":[{\"text\":\"Harrisburg\"}],\"exit_branch_elements\":[{\"text\":\"PA 283 West\"}]},\"length\":16.845,\"instruction\":\"Keep left to take PA 283 West toward Harrisburg.\",\"end_shape_index\":475,\"type\":24,\"time\":956,\"verbal_post_transition_instruction\":\"Continue for 16.8 miles.\",\"street_names\":[\"PA 283 West\"],\"begin_shape_index\":169},{\"travel_type\":\"car\",\"verbal_pre_transition_instruction\":\"Take the Pennsylvania 7 43 exit on the right toward Hershey.\",\"verbal_transition_alert_instruction\":\"Take the Pennsylvania 7 43 exit on the right.\",\"instruction\":\"Take the PA 743 exit on the right toward Hershey.\",\"end_shape_index\":485,\"type\":20,\"time\":31,\"begin_shape_index\":475,\"length\":0.469,\"sign\":{\"exit_toward_elements\":[{\"consecutive_count\":1,\"text\":\"Hershey\"},{\"text\":\"Elizabethtown\"}],\"exit_branch_elements\":[{\"text\":\"PA 743\"}]},\"travel_mode\":\"drive\"},{\"travel_type\":\"car\",\"travel_mode\":\"drive\",\"verbal_pre_transition_instruction\":\"Keep right to take Pennsylvania 7 43 North toward Hershey. Then Continue on Pennsylvania 7 43 North.\",\"verbal_transition_alert_instruction\":\"Keep right to take Pennsylvania 7 43 North.\",\"instruction\":\"Keep right to take PA 743 North toward Hershey.\",\"end_shape_index\":492,\"type\":23,\"time\":4,\"verbal_multi_cue\":true,\"begin_shape_index\":485,\"length\":0.067,\"sign\":{\"exit_toward_elements\":[{\"consecutive_count\":1,\"text\":\"Hershey\"}],\"exit_branch_elements\":[{\"consecutive_count\":1,\"text\":\"PA 743 North\"}]}},{\"travel_type\":\"car\",\"travel_mode\":\"drive\",\"verbal_pre_transition_instruction\":\"Continue on Pennsylvania 7 43 North for 4.9 miles.\",\"begin_street_names\":[\"Hershey Road\",\"PA 743 North\",\"PA 341 Truck\"],\"verbal_transition_alert_instruction\":\"Continue on Pennsylvania 7 43 North.\",\"length\":4.885,\"instruction\":\"Continue on PA 743 North.\",\"end_shape_index\":572,\"type\":8,\"time\":522,\"street_names\":[\"PA 743 North\"],\"begin_shape_index\":492},{\"travel_type\":\"car\",\"travel_mode\":\"drive\",\"verbal_pre_transition_instruction\":\"Continue on Fishburn Road for 2.5 miles.\",\"begin_street_names\":[\"Fishburn Road\",\"PA 743\"],\"verbal_transition_alert_instruction\":\"Continue on Fishburn Road.\",\"length\":2.526,\"instruction\":\"Continue on Fishburn Road.\",\"end_shape_index\":626,\"type\":8,\"time\":261,\"street_names\":[\"Fishburn Road\"],\"begin_shape_index\":572},{\"travel_type\":\"car\",\"travel_mode\":\"drive\",\"verbal_pre_transition_instruction\":\"Bear left onto Hockersville Road.\",\"verbal_transition_alert_instruction\":\"Bear left onto Hockersville Road.\",\"length\":0.572,\"instruction\":\"Bear left onto Hockersville Road.\",\"end_shape_index\":633,\"type\":16,\"time\":92,\"verbal_post_transition_instruction\":\"Continue for 6 tenths of a mile.\",\"street_names\":[\"Hockersville Road\"],\"begin_shape_index\":626},{\"travel_type\":\"car\",\"travel_mode\":\"drive\",\"verbal_pre_transition_instruction\":\"Turn right onto U.S. 4 22, West Chocolate Avenue.\",\"verbal_transition_alert_instruction\":\"Turn right onto U.S. 4 22.\",\"length\":0.664,\"instruction\":\"Turn right onto US 422/West Chocolate Avenue.\",\"end_shape_index\":652,\"type\":10,\"time\":102,\"verbal_post_transition_instruction\":\"Continue for 7 tenths of a mile.\",\"street_names\":[\"US 422\",\"West Chocolate Avenue\"],\"begin_shape_index\":633},{\"travel_type\":\"car\",\"travel_mode\":\"drive\",\"begin_shape_index\":652,\"time\":0,\"type\":4,\"end_shape_index\":652,\"instruction\":\"You have arrived at your destination.\",\"length\":0.000,\"verbal_transition_alert_instruction\":\"You will arrive at your destination.\",\"verbal_pre_transition_instruction\":\"You have arrived at your destination.\"}]}],\"status_message\":\"Found route between points\",\"status\":0}}";
  NavigatorTest nav(route_json_str);
  uint32_t leg_index = 0;

  TryRouteLegCount(nav, 1);
  TryRouteManeuverCount(nav, leg_index, 13);
  TryRouteUnits(nav, "miles", false);
  TryRouteLanguage(nav, "en-US");

  TryRemainingLegValues(nav, 652, 0.0f, 0);       // 0
  TryRemainingLegValues(nav, 633, 0.664f, 102);   // 102
  TryRemainingLegValues(nav, 626, 1.236f, 194);   // 193
  TryRemainingLegValues(nav, 572, 3.762f, 455);   // 452
  TryRemainingLegValues(nav, 492, 8.647f, 977);   // 973
  TryRemainingLegValues(nav, 485, 8.714f, 981);   // 976
  TryRemainingLegValues(nav, 475, 9.183f, 1012);  // 1007
  TryRemainingLegValues(nav, 169, 26.028f, 1968); // 1963
  TryRemainingLegValues(nav, 119, 28.774f, 2132); // 2124
  TryRemainingLegValues(nav, 89, 29.149f, 2177);  // 2164
  TryRemainingLegValues(nav, 3, 31.203f, 2390);   // 2375
  TryRemainingLegValues(nav, 2, 31.249f, 2424);   // 2409
  TryRemainingLegValues(nav, 0, 31.322f, 2438);   // 2423

  ////////////////////////////////////////////////////////////////////////////
  uint32_t maneuver_index = 0;
  ////////////////////////////////////////////////////////////////////////////

  // trace_pt[0] | segment index 0 | begin of maneuver index 0
  TrySnapToRoute(nav, GetFixLocation(-76.299179f, 40.042572f, 0),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.299171f, 40.042519f, leg_index, 31.322f, 2438,
          maneuver_index, 0.073f, 14));

  // off route | segment index 1 | partial maneuver 0
  TrySnapToRoute(nav, GetFixLocation(-76.29875f, 40.04316f, 0),
      GetNavigationStatus(NavigationStatus_RouteState_kInvalid));

  // trace_pt[6] | segment index 1 | partial maneuver 0
  TrySnapToRoute(nav, GetFixLocation(-76.298363f, 40.042652f, 0),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.298355f, 40.042606f, leg_index, 31.278385f, 2415,
          maneuver_index, 0.029385f, 6));

  // trace_pt[9] | segment index 1 | near end of maneuver index 0
  TrySnapToRoute(nav, GetFixLocation(-76.297966f, 40.042698f, 12),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.297958f, 40.042652f, leg_index, 31.257146f, 2411,
          maneuver_index, 0.008146f, 2));

  // trace_pt[10] | segment index 1 | near end of maneuver index 0
  TrySnapToRoute(nav, GetFixLocation(-76.297844f, 40.042709f, 13),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.297836f, 40.042667f, leg_index, 31.250611f, 2409,
          maneuver_index, 0.001611f, 0));


  ////////////////////////////////////////////////////////////////////////////
  maneuver_index = 1;
  ////////////////////////////////////////////////////////////////////////////

  // snap to shape_index[2] | segment index 1/2 | begin maneuver index 1
  TrySnapToRoute(nav, GetFixLocation(-76.297820f, 40.042671f, 14),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.297806f, 40.042671f, leg_index, 31.249f, 2409,
          maneuver_index, 0.046f, 34));

  // near shape_index[2] | segment index 1/2 | begin maneuver index 1
  TrySnapToRoute(nav, GetFixLocation(-76.297810f, 40.042671f, 14),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.297806f, 40.042671f, leg_index, 31.249f, 2409,
          maneuver_index, 0.046f, 34));

  // shape_index[2] | segment index 1/2 | begin maneuver index 1
  TrySnapToRoute(nav, GetFixLocation(-76.297806f, 40.042671f, 14),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.297806f, 40.042671f, leg_index, 31.249f, 2409,
          maneuver_index, 0.046f, 34));


  ////////////////////////////////////////////////////////////////////////////
  maneuver_index = 2;
  ////////////////////////////////////////////////////////////////////////////

  // trace_pt[132] | segment index 15 | middle of maneuver index 2
  TrySnapToRoute(nav, GetFixLocation(-76.279152f, 40.048409f, 154),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.279091f, 40.048393f, leg_index, 30.046f, 2255,
          maneuver_index, 0.8978f, 91));

  ////////////////////////////////////////////////////////////////////////////
  maneuver_index = 3;
  ////////////////////////////////////////////////////////////////////////////

  // trace_pt[211] | segment index 23 | middle of maneuver index 3
  TrySnapToRoute(nav, GetFixLocation(-76.267502f, 40.056950f, 283),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.267517f, 40.056873f, leg_index, 29.024f, 2150,
          maneuver_index, 0.251f, 26));

  ////////////////////////////////////////////////////////////////////////////
  maneuver_index = 4;
  ////////////////////////////////////////////////////////////////////////////

  // trace_pt[315] | segment index 29 | middle of maneuver index 4
  TrySnapToRoute(nav, GetFixLocation(-76.293968f, 40.068066f, 388),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.293968f, 40.068073f, leg_index, 27.341f, 2042,
          maneuver_index, 1.3148f, 79));

  ////////////////////////////////////////////////////////////////////////////
  maneuver_index = 5;
  ////////////////////////////////////////////////////////////////////////////

  // trace_pt[850] | segment index 60 | middle of maneuver index 5
  TrySnapToRoute(nav, GetFixLocation(-76.471893f, 40.125996f, 948),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.471878f, 40.126011f, leg_index, 16.9535f, 1450,
          maneuver_index, 7.771f, 443));

  ////////////////////////////////////////////////////////////////////////////
  maneuver_index = 6;
  ////////////////////////////////////////////////////////////////////////////

  // trace_pt[1270] | segment index 74 | middle of maneuver index 6
  TrySnapToRoute(nav, GetFixLocation(-76.598633f, 40.176716f, 1441),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.598648f, 40.176708f, leg_index, 8.86744f, 985,
          maneuver_index, 0.1534f, 9));

  ////////////////////////////////////////////////////////////////////////////
  maneuver_index = 7;
  ////////////////////////////////////////////////////////////////////////////

  // trace_pt[1288] | segment index 75 | middle of maneuver index 7
  TrySnapToRoute(nav, GetFixLocation(-76.600441f, 40.178944f, 1459),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.600441f, 40.178944f, leg_index, 8.6843f, 974,
          maneuver_index, 0.03765f, 1));

  ////////////////////////////////////////////////////////////////////////////
  maneuver_index = 8;
  ////////////////////////////////////////////////////////////////////////////

  // trace_pt[1522] | segment index 86 | middle of maneuver index 8
  TrySnapToRoute(nav, GetFixLocation(-76.618904f, 40.219437f, 1722),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.618668f, 40.219433f, leg_index, 5.6829f, 654,
          maneuver_index, 1.9216f, 202));

  ////////////////////////////////////////////////////////////////////////////
  maneuver_index = 9;
  ////////////////////////////////////////////////////////////////////////////

  // trace_pt[1870] | segment index 105 | middle of maneuver index 9
  TrySnapToRoute(nav, GetFixLocation(-76.652367f, 40.270046f, 2113),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.652374f, 40.270039f, leg_index, 1.5266f, 224,
          maneuver_index, 0.2904f, 31));

  ////////////////////////////////////////////////////////////////////////////
  maneuver_index = 10;
  ////////////////////////////////////////////////////////////////////////////

  // trace_pt[1936] | segment index 111 | middle of maneuver index 10
  TrySnapToRoute(nav, GetFixLocation(-76.660156f, 40.275745f, 2290),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.660187f, 40.275719f, leg_index, 0.9464f, 147,
          maneuver_index, 0.2826f, 45));

  ////////////////////////////////////////////////////////////////////////////
  maneuver_index = 11;
  ////////////////////////////////////////////////////////////////////////////

  // trace_pt[2003] | segment index 119 | middle of maneuver index 11
  TrySnapToRoute(nav, GetFixLocation(-76.659927f, 40.281940f, 2386),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.659943f, 40.281963f, leg_index, 0.3119f, 47,
          maneuver_index, 0.3119f, 47));

  ////////////////////////////////////////////////////////////////////////////
  maneuver_index = 12;
  ////////////////////////////////////////////////////////////////////////////

  // trace_pt[2038] | segment index 125 | destination maneuver index 12
  TrySnapToRoute(nav, GetFixLocation(-76.654625f, 40.283924f, 2438),
      GetNavigationStatus(NavigationStatus_RouteState_kTracking,
          -76.654633f, 40.283943f, leg_index, 0.0f, 0,
          maneuver_index, 0.0f, 0));

  ////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  //  TryRouteOnLocationChanged(nav, GetFixLocation(-76.299179, 40.042572, 0),
//      GetNavigationStatus(NavigationStatus_RouteState_kPreTransition,
//          -76.299171, 40.042519, leg_index, maneuver_index));
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

  // TestLancasterToHershey
  suite.test(TEST_CASE(TestLancasterToHershey));

  return suite.tear_down();
}
