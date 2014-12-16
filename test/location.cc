#include "test.h"

#include "include/config.h"
#include "include/baldr/location.h"

using namespace std;
using namespace valhalla::baldr;

namespace {
  void test_construction() {
    Location l();
    try{
      Location("somestring");
      throw std::runtime_error("Location from geojson serialization should throw as its unimplemented");
    }
    catch(...) {
    }
  }
}

int main(void)
{
  test::suite suite("location");

  suite.test(TEST_CASE(test_construction));
  //TODO: many more

  return suite.tear_down();
}
