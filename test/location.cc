#include "test.h"

#include "config.h"
#include "baldr/location.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {
  void test_construction() {
    Location l();
    try{
      Location("somestring");
      throw std::runtime_error("Location from geojson serialization should throw as its unimplemented");
    }
    catch(...) {
    }
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
