#include "test.h"

#include "baldr/location.h"
#include <boost/property_tree/json_parser.hpp>
#include <valhalla/midgard/util.h>
#include <valhalla/midgard/logging.h>

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

  csv = Location::FromCsv("1.452,-3.45,stop");
  if (csv.latlng_.y() != std::stof("1.452")
      || csv.latlng_.x() != std::stof("-3.45")
      || csv.stoptype_ != Location::StopType::BREAK)
    throw std::runtime_error("Csv location parsing failed");

  Location b(PointLL { 1, 2 });
  if (b.latlng_.x() != 1 || b.latlng_.y() != 2)
    throw std::runtime_error("Location's latlng object should be set");

  csv = Location::FromCsv("1.452,-3.45,stop,Just a name");
  if (!(csv.name_ == "Just a name"))
    throw std::runtime_error("Csv location parsing failed");

  csv = Location::FromCsv(
      "1.452,-3.45,stop,Home,123 Main Street,Lancaster,PA,90210");
  if (!(csv.name_ == "Home" && csv.street_ == "123 Main Street"
      && csv.city_ == "Lancaster" && csv.state_ == "PA" && csv.zip_ == "90210"))
    throw std::runtime_error("Csv location parsing failed");

  csv = Location::FromCsv(
      "1.452,-3.45,stop,,123 Main Street,Lancaster,PA,90210");
  if (!(csv.name_ == "" && csv.street_ == "123 Main Street"
      && csv.city_ == "Lancaster" && csv.state_ == "PA" && csv.zip_ == "90210"))
    throw std::runtime_error("Csv location parsing failed");

  csv = Location::FromCsv("1.452,-3.45,stop,Home,,Lancaster,PA,90210");
  if (!(csv.name_ == "Home" && csv.street_ == "" && csv.city_ == "Lancaster"
      && csv.state_ == "PA" && csv.zip_ == "90210"))
    throw std::runtime_error("Csv location parsing failed");

  csv = Location::FromCsv("1.452,-3.45,stop,Home,123 Main Street,,PA,90210");
  if (!(csv.name_ == "Home" && csv.street_ == "123 Main Street"
      && csv.city_ == "" && csv.state_ == "PA" && csv.zip_ == "90210"))
    throw std::runtime_error("Csv location parsing failed");

  csv = Location::FromCsv(
      "1.452,-3.45,stop,Home,123 Main Street,Lancaster,,90210");
  if (!(csv.name_ == "Home" && csv.street_ == "123 Main Street"
      && csv.city_ == "Lancaster" && csv.state_ == "" && csv.zip_ == "90210"))
    throw std::runtime_error("Csv location parsing failed");

  csv = Location::FromCsv(
      "1.452,-3.45,stop,Home,123 Main Street,Lancaster,PA,");
  if (!(csv.name_ == "Home" && csv.street_ == "123 Main Street"
      && csv.city_ == "Lancaster" && csv.state_ == "PA" && csv.zip_ == ""))
    throw std::runtime_error("Csv location parsing failed");

  csv = Location::FromCsv(
      "1.452,-3.45,stop,Home,123 Main Street,Lancaster,PA,90210,US");
  if (!(csv.name_ == "Home" && csv.street_ == "123 Main Street"
      && csv.city_ == "Lancaster" && csv.state_ == "PA" && csv.zip_ == "90210"
      && csv.country_ == "US"))
    throw std::runtime_error("Csv location parsing failed");

  csv = Location::FromCsv("1.452,-3.45,stop,,,,,,DE");
  if (!(csv.name_ == "" && csv.street_ == "" && csv.city_ == ""
      && csv.state_ == "" && csv.zip_ == "" && csv.country_ == "DE"))
    throw std::runtime_error("Csv location parsing failed");

  csv = Location::FromCsv(
      "1.452,-3.45,stop,Home,123 Main Street,Lancaster,PA,90210,US,270");
  if (!(csv.name_ == "Home" && csv.street_ == "123 Main Street"
      && csv.city_ == "Lancaster" && csv.state_ == "PA" && csv.zip_ == "90210"
      && csv.country_ == "US" && csv.heading_ == "270"))
    throw std::runtime_error("Csv location parsing failed");

  csv = Location::FromCsv("1.452,-3.45,stop,Lancaster Brewing Company,,,,,,65");
  if (!(csv.name_ == "Lancaster Brewing Company" && csv.street_ == ""
      && csv.city_ == "" && csv.state_ == "" && csv.zip_ == ""
      && csv.country_ == "" && csv.heading_ == "65"))
    throw std::runtime_error("Csv location parsing failed");
}

std::string make_json(float lat, float lng,
                      std::string type = "break",
                      std::string heading = "",
                      std::string name = "",
                      std::string street = "",
                      std::string city = "",
                      std::string state = "",
                      std::string postal_code = "",
                      std::string country = "") {
  std::string json = "{";
  json += "\"latitude\":" + std::to_string(lat);
  json += ",\"longitude\":" + std::to_string(lng);
  json += ",\"type\":\"" + type + "\"";

  if (!heading.empty())
    json += ",\"heading\":" + heading;
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

boost::property_tree::ptree make_ptree(float lat, float lng,
                                std::string type = "break",
                                std::string heading = "",
                                std::string name = "",
                                std::string street = "",
                                std::string city = "",
                                std::string state = "",
                                std::string postal_code = "",
                                std::string country = "") {
  std::string json = make_json(lat, lng, type, heading, name, street, city,
                               state, postal_code, country);
  //LOG_INFO("json=" + json);
  std::stringstream stream;
  stream << json.c_str();
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(stream, pt);
  return pt;
}

void test_from_ptree() {

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

  try {
    boost::property_tree::ptree pt;
    Location loc = Location::FromPtree(pt);
    throw std::runtime_error("This should have been malformed ptree");
  } catch (...) {
  }

  // Test Lat/Lng and default type
  Location loc = Location::FromPtree(make_ptree(lat, lng));
  if (!valhalla::midgard::equal<float>(loc.latlng_.lat(), lat)
      || !valhalla::midgard::equal<float>(loc.latlng_.lng(), lng)
      || (loc.stoptype_ != Location::StopType::BREAK))
    throw std::runtime_error("Ptree location parsing failed");

  // Test Lat/Lng and specified type
  loc = Location::FromPtree(make_ptree(lat, lng, type_specified));
  if (!valhalla::midgard::equal<float>(loc.latlng_.lat(), lat)
      || !valhalla::midgard::equal<float>(loc.latlng_.lng(), lng)
      || (loc.stoptype_ != Location::StopType::THROUGH))
    throw std::runtime_error("Ptree location parsing failed");

  // Test Lat/Lng, specified type, and heading
  std::string heading_str = std::to_string(heading);
  loc = Location::FromPtree(make_ptree(lat, lng, type_default, heading_str));
  if (!valhalla::midgard::equal<float>(loc.latlng_.lat(), lat)
      || !valhalla::midgard::equal<float>(loc.latlng_.lng(), lng)
      || (loc.stoptype_ != Location::StopType::BREAK)
      || (loc.heading_ != heading_str))
    throw std::runtime_error("Ptree location parsing failed");

  // Test name
  loc = Location::FromPtree(
      make_ptree(lat, lng, type_default, "", name));
  if ((loc.name_ != name))
    throw std::runtime_error("Ptree location parsing failed");

  // Test street
  loc = Location::FromPtree(
      make_ptree(lat, lng, type_default, "", name, street));
  if ((loc.street_ != street))
    throw std::runtime_error("Ptree location parsing failed");

  // Test city
  loc = Location::FromPtree(
      make_ptree(lat, lng, type_default, "", name, street, city));
  if ((loc.city_ != city))
    throw std::runtime_error("Ptree location parsing failed");

  // Test state
  loc = Location::FromPtree(
      make_ptree(lat, lng, type_default, "", name, street, city, state));
  if ((loc.state_ != state))
    throw std::runtime_error("Ptree location parsing failed");

  // Test postal code
  loc = Location::FromPtree(
      make_ptree(lat, lng, type_default, "", name, street, city, state, postal_code));
  if ((loc.zip_ != postal_code))
    throw std::runtime_error("Ptree location parsing failed");

  // Test country
  loc = Location::FromPtree(
      make_ptree(lat, lng, type_default, "", name, street, city, state, postal_code, country));
  if ((loc.country_ != country))
    throw std::runtime_error("Ptree location parsing failed");

  // Test everything
  loc = Location::FromPtree(
      make_ptree(lat, lng, type_default, heading_str, name, street, city, state,
                 postal_code, country));
  if (!valhalla::midgard::equal<float>(loc.latlng_.lat(), lat)
      || !valhalla::midgard::equal<float>(loc.latlng_.lng(), lng)
      || (loc.stoptype_ != Location::StopType::BREAK)
      || (loc.heading_ != heading_str)
      || (loc.name_ != name)
      || (loc.street_ != street)
      || (loc.city_ != city)
      || (loc.state_ != state)
      || (loc.zip_ != postal_code)
      || (loc.country_ != country))
    throw std::runtime_error("Ptree location parsing failed");
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
  std::string heading_str = std::to_string(heading);
  loc = Location::FromJson(make_json(lat, lng, type_default, heading_str));
  if (!valhalla::midgard::equal<float>(loc.latlng_.lat(), lat)
      || !valhalla::midgard::equal<float>(loc.latlng_.lng(), lng)
      || (loc.stoptype_ != Location::StopType::BREAK)
      || (loc.heading_ != heading_str))
    throw std::runtime_error("Json location parsing failed");

  // Test name
  loc = Location::FromJson(
      make_json(lat, lng, type_default, "", name));
  if ((loc.name_ != name))
    throw std::runtime_error("Json location parsing failed");

  // Test street
  loc = Location::FromJson(
      make_json(lat, lng, type_default, "", name, street));
  if ((loc.street_ != street))
    throw std::runtime_error("Json location parsing failed");

  // Test city
  loc = Location::FromJson(
      make_json(lat, lng, type_default, "", name, street, city));
  if ((loc.city_ != city))
    throw std::runtime_error("Json location parsing failed");

  // Test state
  loc = Location::FromJson(
      make_json(lat, lng, type_default, "", name, street, city, state));
  if ((loc.state_ != state))
    throw std::runtime_error("Json location parsing failed");

  // Test postal code
  loc = Location::FromJson(
      make_json(lat, lng, type_default, "", name, street, city, state, postal_code));
  if ((loc.zip_ != postal_code))
    throw std::runtime_error("Json location parsing failed");

  // Test country
  loc = Location::FromJson(
      make_json(lat, lng, type_default, "", name, street, city, state, postal_code, country));
  if ((loc.country_ != country))
    throw std::runtime_error("Json location parsing failed");

  // Test everything
  loc = Location::FromJson(
      make_json(lat, lng, type_default, heading_str, name, street, city, state,
                 postal_code, country));
  if (!valhalla::midgard::equal<float>(loc.latlng_.lat(), lat)
      || !valhalla::midgard::equal<float>(loc.latlng_.lng(), lng)
      || (loc.stoptype_ != Location::StopType::BREAK)
      || (loc.heading_ != heading_str)
      || (loc.name_ != name)
      || (loc.street_ != street)
      || (loc.city_ != city)
      || (loc.state_ != state)
      || (loc.zip_ != postal_code)
      || (loc.country_ != country))
    throw std::runtime_error("Json location parsing failed");
}

}

int main(void) {
  test::suite suite("location");

  suite.test(TEST_CASE(test_from_csv));

  suite.test(TEST_CASE(test_from_ptree));

  suite.test(TEST_CASE(test_from_json));

  return suite.tear_down();
}

