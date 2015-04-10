#include "test.h"

#include "baldr/location.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {
  void test_construction() {
    Location l();
    try{
      Location geojson = Location::FromGeoJson("somestring");
      throw std::runtime_error("Location from geojson serialization should throw as its unimplemented");
    }
    catch(...) {
    }
    try{
      Location csv = Location::FromCsv("somestring");
      throw std::runtime_error("This should have been malformed csv");
    }
    catch(...) {
    }

    Location csv = Location::FromCsv("1.452,3.45");
    if(csv.latlng_.y() != std::stof("1.452") || csv.latlng_.x() != std::stof("3.45")|| csv.stoptype_ != Location::StopType::BREAK)
      throw std::runtime_error("Csv location parsing failed");

    csv = Location::FromCsv("1.452,-3.45,through");
    if(csv.latlng_.y() != std::stof("1.452") || csv.latlng_.x() != std::stof("-3.45") || csv.stoptype_ != Location::StopType::THROUGH)
      throw std::runtime_error("Csv location parsing failed");

    csv = Location::FromCsv("1.452,-3.45,stop");
    if(csv.latlng_.y() != std::stof("1.452") || csv.latlng_.x() != std::stof("-3.45") || csv.stoptype_ != Location::StopType::BREAK)
      throw std::runtime_error("Csv location parsing failed");

    Location b(PointLL{1,2});
    if(b.latlng_.x() !=1 || b.latlng_.y() != 2)
      throw std::runtime_error("Location's latlng object should be set");

    csv = Location::FromCsv("1.452,-3.45,stop,Just a name");
    if(!(csv.name_=="Just a name"))
      throw std::runtime_error("Csv location parsing failed");

    csv = Location::FromCsv("1.452,-3.45,stop,Home,123 Main Street,Lancaster,PA,90210");
    if(!(csv.name_=="Home" && csv.street_=="123 Main Street" && csv.city_=="Lancaster" && csv.state_=="PA" && csv.zip_=="90210"))
      throw std::runtime_error("Csv location parsing failed");

    csv = Location::FromCsv("1.452,-3.45,stop,,123 Main Street,Lancaster,PA,90210");
    if(!(csv.name_=="" && csv.street_=="123 Main Street" && csv.city_=="Lancaster" && csv.state_=="PA" && csv.zip_=="90210"))
      throw std::runtime_error("Csv location parsing failed");

    csv = Location::FromCsv("1.452,-3.45,stop,Home,,Lancaster,PA,90210");
    if(!(csv.name_=="Home" && csv.street_=="" && csv.city_=="Lancaster" && csv.state_=="PA" && csv.zip_=="90210"))
      throw std::runtime_error("Csv location parsing failed");

    csv = Location::FromCsv("1.452,-3.45,stop,Home,123 Main Street,,PA,90210");
    if(!(csv.name_=="Home" && csv.street_=="123 Main Street" && csv.city_=="" && csv.state_=="PA" && csv.zip_=="90210"))
      throw std::runtime_error("Csv location parsing failed");

    csv = Location::FromCsv("1.452,-3.45,stop,Home,123 Main Street,Lancaster,,90210");
    if(!(csv.name_=="Home" && csv.street_=="123 Main Street" && csv.city_=="Lancaster" && csv.state_=="" && csv.zip_=="90210"))
      throw std::runtime_error("Csv location parsing failed");

    csv = Location::FromCsv("1.452,-3.45,stop,Home,123 Main Street,Lancaster,PA,");
    if(!(csv.name_=="Home" && csv.street_=="123 Main Street" && csv.city_=="Lancaster" && csv.state_=="PA" && csv.zip_==""))
      throw std::runtime_error("Csv location parsing failed");

    csv = Location::FromCsv("1.452,-3.45,stop,Home,123 Main Street,Lancaster,PA,90210,US");
    if(!(csv.name_=="Home" && csv.street_=="123 Main Street" && csv.city_=="Lancaster" && csv.state_=="PA" && csv.zip_=="90210" && csv.country_=="US"))
      throw std::runtime_error("Csv location parsing failed");

    csv = Location::FromCsv("1.452,-3.45,stop,Home,123 Main Street,Lancaster,PA,90210,US,717-555-1212");
    if(!(csv.name_=="Home" && csv.street_=="123 Main Street" && csv.city_=="Lancaster" && csv.state_=="PA" && csv.zip_=="90210" && csv.country_=="US" && csv.phone_=="717-555-1212"))
      throw std::runtime_error("Csv location parsing failed");

    csv = Location::FromCsv("1.452,-3.45,stop,Home,123 Main Street,Lancaster,PA,90210,US,717-555-1212,www.url.com");
    if(!(csv.name_=="Home" && csv.street_=="123 Main Street" && csv.city_=="Lancaster" && csv.state_=="PA" && csv.zip_=="90210" && csv.country_=="US" && csv.phone_=="717-555-1212" && csv.url_=="www.url.com"))
      throw std::runtime_error("Csv location parsing failed");

    csv = Location::FromCsv("1.452,-3.45,stop,,,,,,,,www.url.com");
    if(!(csv.name_=="" && csv.street_=="" && csv.city_=="" && csv.state_=="" && csv.zip_=="" && csv.country_=="" && csv.phone_=="" && csv.url_=="www.url.com"))
      throw std::runtime_error("Csv location parsing failed");

    csv = Location::FromCsv("1.452,-3.45,stop,Home,123 Main Street,Lancaster,PA,90210,US,717-555-1212,www.url.com,270");
    if(!(csv.name_=="Home" && csv.street_=="123 Main Street" && csv.city_=="Lancaster" && csv.state_=="PA" && csv.zip_=="90210" && csv.country_=="US" && csv.phone_=="717-555-1212" && csv.url_=="www.url.com" && csv.heading_=="270"))
      throw std::runtime_error("Csv location parsing failed");

    csv = Location::FromCsv("1.452,-3.45,stop,Lancaster Brewing Company,,,,,,,,65");
    if(!(csv.name_=="Lancaster Brewing Company" && csv.street_=="" && csv.city_=="" && csv.state_=="" && csv.zip_=="" && csv.country_=="" && csv.phone_=="" && csv.url_=="" && csv.heading_=="65"))
      throw std::runtime_error("Csv location parsing failed");

  }
}

int main(void)
{
  test::suite suite("location");

  suite.test(TEST_CASE(test_construction));
  //TODO: many more

  return suite.tear_down();
}
