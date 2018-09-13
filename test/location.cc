#include "test.h"
#include <cstdint>

#include "baldr/location.h"
#include "midgard/logging.h"
#include "midgard/util.h"

#include <unordered_map>

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

std::string make_json(float lat,
                      float lng,
                      std::string type = "",
                      boost::optional<int> heading = boost::none,
                      std::string name = "",
                      std::string street = "",
                      std::string city = "",
                      std::string state = "",
                      std::string postal_code = "",
                      std::string country = "",
                      boost::optional<uint64_t> way_id = boost::none,
                      boost::optional<std::string> date_time = boost::none) {
  std::string json = "{";
  json += "\"lat\":" + std::to_string(lat);
  json += ",\"lon\":" + std::to_string(lng);

  if (!type.empty())
    json += ",\"type\":\"" + type + "\"";
  if (heading)
    json += ",\"heading\":" + std::to_string(*heading);
  if (way_id)
    json += ",\"way_id\":" + std::to_string(*way_id);
  if (date_time)
    json += ",\"date_time\":\"" + *date_time + "\"";
  if (!name.empty())
    json += ",\"name\":\"" + name + "\"";
  if (!street.empty())
    json += ",\"street\":\"" + street + "\"";
  if (!city.empty())
    json += ",\"city\":\"" + city + "\"";
  if (!state.empty())
    json += ",\"state\":\"" + state + "\"";
  if (!postal_code.empty())
    json += ",\"postal_code\":\"" + postal_code + "\"";
  if (!country.empty())
    json += ",\"country\":\"" + country + "\"";
  json += "}";

  // LOG_INFO("json=" + json);
  return json;
}

Location from_json(const std::string& json) {
  rapidjson::Document d;
  d.Parse(json.c_str());
  if (d.HasParseError())
    throw std::runtime_error("Parse Error");
  return Location::FromRapidJson(d);
}

void test_from_json() {

  float lat = 40.748174;
  float lng = -73.984984;
  std::string type_default = "break";
  std::string type_specified = "through";
  int heading = 200;
  std::string name = "Empire State Building";
  std::string street = "350 5th Avenue";
  std::string city = "New York";
  std::string state = "NY";
  std::string postal_code = "10118-0110";
  std::string country = "US";
  uint64_t way_id = 12987234107;
  std::string date_time = "2015-05-06T08:00";

  bool exception_caught = false;
  try {
    Location loc = from_json("something");
  } catch (...) { exception_caught = true; }
  if (!exception_caught)
    throw std::runtime_error("This should have been malformed json");

  // Test Lat/Lng and default type
  Location loc = from_json(make_json(lat, lng));
  if (!valhalla::midgard::equal<float>(loc.latlng_.lat(), lat) ||
      !valhalla::midgard::equal<float>(loc.latlng_.lng(), lng) ||
      (loc.stoptype_ != Location::StopType::BREAK))
    throw std::runtime_error("Json location parsing failed: Test Lat/Lng and default type");

  // Test Lat/Lng and specified type
  loc = from_json(make_json(lat, lng, type_specified));
  if (!valhalla::midgard::equal<float>(loc.latlng_.lat(), lat) ||
      !valhalla::midgard::equal<float>(loc.latlng_.lng(), lng) ||
      (loc.stoptype_ != Location::StopType::THROUGH))
    throw std::runtime_error("Json location parsing failed: Test Lat/Lng and specified type");

  // Test Lat/Lng, specified type, and heading
  loc = from_json(make_json(lat, lng, type_default, heading));
  if (!valhalla::midgard::equal<float>(loc.latlng_.lat(), lat) ||
      !valhalla::midgard::equal<float>(loc.latlng_.lng(), lng) ||
      (loc.stoptype_ != Location::StopType::BREAK) || (*loc.heading_ != heading))
    throw std::runtime_error(
        "Json location parsing failed: Test Lat/Lng, specified type, and heading");

  // Test name
  loc = from_json(make_json(lat, lng, type_default, boost::none, name));
  if ((loc.name_ != name))
    throw std::runtime_error("Json location parsing failed: Test name name_");
  if (loc.heading_)
    throw std::runtime_error("Json location parsing failed: Test name heading_");

  // Test street
  loc = from_json(make_json(lat, lng, type_default, boost::none, name, street));
  if ((loc.street_ != street))
    throw std::runtime_error("Json location parsing failed: Test street");

  // Test city
  loc = from_json(make_json(lat, lng, type_default, boost::none, name, street, city));
  if ((loc.city_ != city))
    throw std::runtime_error("Json location parsing failed: Test city");

  // Test state
  loc = from_json(make_json(lat, lng, type_default, boost::none, name, street, city, state));
  if ((loc.state_ != state))
    throw std::runtime_error("Json location parsing failed: Test state");

  // Test postal code
  loc = from_json(
      make_json(lat, lng, type_default, boost::none, name, street, city, state, postal_code));
  if ((loc.zip_ != postal_code))
    throw std::runtime_error("Json location parsing failed: Test postal code postal_code");
  if (loc.way_id_)
    throw std::runtime_error("Json location parsing failed: Test postal code way_id");
  if (loc.date_time_)
    throw std::runtime_error("Json location parsing failed: Test postal code date_time");

  // Test country, way_id, and date_time.
  loc = from_json(make_json(lat, lng, type_default, boost::none, name, street, city, state,
                            postal_code, country, way_id, date_time));
  if ((loc.country_ != country))
    throw std::runtime_error(
        "Json location parsing failed: Test country, way_id, and date_time country");
  if (*loc.way_id_ != way_id)
    throw std::runtime_error(
        "Json location parsing failed: Test country, way_id, and date_time way_id");
  if (*loc.date_time_ != date_time)
    throw std::runtime_error(
        "Json location parsing failed: Test country, way_id, and date_time date_time");

  // Test wrong thing is always break
  loc = from_json(make_json(lat, lng, "this isnt valid"));
  if ((loc.stoptype_ != Location::StopType::BREAK))
    throw std::runtime_error("Json location parsing failed: Test wrong thing is always break");

  // Test everything
  loc = from_json(make_json(lat, lng, type_default, heading, name, street, city, state, postal_code,
                            country, boost::none, date_time));
  if (!valhalla::midgard::equal<float>(loc.latlng_.lat(), lat) ||
      !valhalla::midgard::equal<float>(loc.latlng_.lng(), lng) ||
      (loc.stoptype_ != Location::StopType::BREAK) || (*loc.heading_ != heading) ||
      (loc.name_ != name) || (loc.street_ != street) || (loc.city_ != city) ||
      (loc.state_ != state) || (loc.zip_ != postal_code) || (loc.country_ != country) ||
      (loc.date_time_ != date_time))
    throw std::runtime_error("Json location parsing failed: Test everything");

  // Test large way_id
  loc = from_json(R"({"lat":0,"lon":0,"way_id":18446744073709551614})");
  if (*loc.way_id_ != 18446744073709551614ULL)
    throw std::runtime_error("Wrong way id");

  // Test floating point heading
  loc = from_json(R"({"lat":0,"lon":0,"heading":37.1})");
  if (*loc.heading_ != 37)
    throw std::runtime_error("Wrong heading");
}

void test_hashing() {

  Location a({123.456789, 9.87654321}, Location::StopType::BREAK);
  a.name_ = "name";
  a.street_ = "street";
  a.city_ = "city";
  a.state_ = "state";
  a.zip_ = "zip";
  a.country_ = "country";
  a.date_time_ = "date";
  a.heading_ = 1;
  a.way_id_ = 2;
  Location b = a;
  b.name_ = "nameb";
  Location c = a;
  c.way_id_.reset();
  Location d = a;
  d.latlng_.first = 123.4567;

  std::unordered_map<Location, int> m{{a, 1}};
  if (m.find(a) == m.cend())
    throw std::logic_error("Should have found a");
  if (!m.insert({b, 2}).second)
    throw std::logic_error("Should not have found b");
  if (!m.insert({c, 3}).second)
    throw std::logic_error("Should not have found c");
  if (!m.insert({d, 4}).second)
    throw std::logic_error("Should not have found d");
}

} // namespace

int main(void) {
  test::suite suite("location");

  suite.test(TEST_CASE(test_from_json));

  suite.test(TEST_CASE(test_hashing));

  return suite.tear_down();
}
