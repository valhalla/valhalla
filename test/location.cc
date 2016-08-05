#include "test.h"

#include "baldr/location.h"
#include <boost/property_tree/json_parser.hpp>
#include <valhalla/midgard/util.h>
#include <valhalla/midgard/logging.h>

#include <unordered_map>

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

void test_from_csv() {
  try {
    Location csv = Location::FromCsv("somestring");
    throw std::runtime_error("This should have been malformed csv");
  } catch (...) {
  }

  Location csv = Location::FromCsv("1.452,3.45");
  if (csv.latlng_.y() != std::stof("1.452")
      || csv.latlng_.x() != std::stof("3.45")
      || csv.stoptype_ != Location::StopType::BREAK)
    throw std::runtime_error("Csv location parsing failed");

  csv = Location::FromCsv("1.452,-3.45,through");
  if (csv.latlng_.y() != std::stof("1.452")
      || csv.latlng_.x() != std::stof("-3.45")
      || csv.stoptype_ != Location::StopType::THROUGH)
    throw std::runtime_error("Csv location parsing failed");

  csv = Location::FromCsv("1.452,-3.45,break");
  if (csv.latlng_.y() != std::stof("1.452")
      || csv.latlng_.x() != std::stof("-3.45")
      || csv.stoptype_ != Location::StopType::BREAK)
    throw std::runtime_error("Csv location parsing failed");

  Location b(PointLL { 1, 2 });
  if (b.latlng_.x() != 1 || b.latlng_.y() != 2)
    throw std::runtime_error("Location's latlng object should be set");

}

std::string make_json(float lat, float lng,
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

  if(!type.empty())
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

  //LOG_INFO("json=" + json);
  return json;
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

  try {
    Location loc = Location::FromJson("something");
    throw std::runtime_error("This should have been malformed json");
  } catch (...) {
  }

  // Test Lat/Lng and default type
  Location loc = Location::FromJson(make_json(lat, lng));
  if (!valhalla::midgard::equal<float>(loc.latlng_.lat(), lat)
      || !valhalla::midgard::equal<float>(loc.latlng_.lng(), lng)
      || (loc.stoptype_ != Location::StopType::BREAK))
    throw std::runtime_error("Json location parsing failed");

  // Test Lat/Lng and specified type
  loc = Location::FromJson(make_json(lat, lng, type_specified));
  if (!valhalla::midgard::equal<float>(loc.latlng_.lat(), lat)
      || !valhalla::midgard::equal<float>(loc.latlng_.lng(), lng)
      || (loc.stoptype_ != Location::StopType::THROUGH))
    throw std::runtime_error("Json location parsing failed");

  // Test Lat/Lng, specified type, and heading
  loc = Location::FromJson(make_json(lat, lng, type_default, heading));
  if (!valhalla::midgard::equal<float>(loc.latlng_.lat(), lat)
      || !valhalla::midgard::equal<float>(loc.latlng_.lng(), lng)
      || (loc.stoptype_ != Location::StopType::BREAK)
      || (*loc.heading_ != heading))
    throw std::runtime_error("Json location parsing failed");

  // Test name
  loc = Location::FromJson(
      make_json(lat, lng, type_default, boost::none, name));
  if ((loc.name_ != name))
    throw std::runtime_error("Json location parsing failed");
  if (loc.heading_)
    throw std::runtime_error("Json location parsing failed");

  // Test street
  loc = Location::FromJson(
      make_json(lat, lng, type_default, boost::none, name, street));
  if ((loc.street_ != street))
    throw std::runtime_error("Json location parsing failed");

  // Test city
  loc = Location::FromJson(
      make_json(lat, lng, type_default, boost::none, name, street, city));
  if ((loc.city_ != city))
    throw std::runtime_error("Json location parsing failed");

  // Test state
  loc = Location::FromJson(
      make_json(lat, lng, type_default, boost::none, name, street, city, state));
  if ((loc.state_ != state))
    throw std::runtime_error("Json location parsing failed");

  // Test postal code
  loc = Location::FromJson(
      make_json(lat, lng, type_default, boost::none, name, street, city, state, postal_code));
  if ((loc.zip_ != postal_code))
    throw std::runtime_error("Json location parsing failed");
  if (loc.way_id_)
    throw std::runtime_error("Json location parsing failed");
  if (loc.date_time_)
    throw std::runtime_error("Json location parsing failed");

  // Test country, way_id, and date_time.
  loc = Location::FromJson(
      make_json(lat, lng, type_default, boost::none, name, street, city, state, postal_code, country, way_id, date_time));
  if ((loc.country_ != country))
    throw std::runtime_error("Json location parsing failed");
  if (*loc.way_id_ != way_id)
    throw std::runtime_error("Json location parsing failed");
  if (*loc.date_time_ != date_time)
    throw std::runtime_error("Json location parsing failed");

  // Test wrong thing is always break
  loc = Location::FromJson(
      make_json(lat, lng, "this isnt valid"));
  if ((loc.stoptype_ != Location::StopType::BREAK))
    throw std::runtime_error("Json location parsing failed");

  // Test everything
  loc = Location::FromJson(
      make_json(lat, lng, type_default, heading, name, street, city, state,
                 postal_code, country,boost::none, date_time));
  if (!valhalla::midgard::equal<float>(loc.latlng_.lat(), lat)
      || !valhalla::midgard::equal<float>(loc.latlng_.lng(), lng)
      || (loc.stoptype_ != Location::StopType::BREAK)
      || (*loc.heading_ != heading)
      || (loc.name_ != name)
      || (loc.street_ != street)
      || (loc.city_ != city)
      || (loc.state_ != state)
      || (loc.zip_ != postal_code)
      || (loc.country_ != country)
      || (loc.date_time_ != date_time))
    throw std::runtime_error("Json location parsing failed");
}

void test_hashing() {

  Location a({123.456789,9.87654321},Location::StopType::BREAK);
  a.name_="name"; a.street_="street"; a.city_="city"; a.state_="state"; a.zip_="zip"; a.country_="country"; a.date_time_="date"; a.heading_=1; a.way_id_=2;
  Location b = a; b.name_="nameb";
  Location c = a; c.way_id_.reset();
  Location d = a; d.latlng_.first = 123.4567;

  std::unordered_map<Location, int> m{{a,1}};
  if(m.find(a) == m.cend())
    throw std::logic_error("Should have found a");
  if(!m.insert({b,2}).second)
    throw std::logic_error("Should not have found b");
  if(!m.insert({c,3}).second)
    throw std::logic_error("Should not have found c");
  if(!m.insert({d,4}).second)
    throw std::logic_error("Should not have found d");
}

}

int main(void) {
  test::suite suite("location");

  suite.test(TEST_CASE(test_from_csv));

  suite.test(TEST_CASE(test_from_json));

  suite.test(TEST_CASE(test_hashing));

  return suite.tear_down();
}

