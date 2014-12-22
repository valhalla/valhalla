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

    Location b(PointLL{1,2});
    if(b.latlng_.y() !=1 || b.latlng_.x() != 2)
      throw std::runtime_error("Location's latlng object should be set");
  }
}

int main(void)
{
  test::suite suite("location");

  suite.test(TEST_CASE(test_construction));
  //TODO: many more

  return suite.tear_down();
}
