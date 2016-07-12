#include "test.h"
#include "midgard/gridded_data.h"
#include <limits>

using namespace valhalla::midgard;

namespace {

  void test_gridded() {
    //fill this as distance from center
    GriddedData<PointLL> g({-5,-5,5,5}, 1, std::numeric_limits<float>::max());
    for(float i = -4.5; i < 5; i += 1) {
      for(float j = -4.5; j < 5; j += 1) {
        if(!g.Set({i,j}, PointLL(0,0).Distance({i,j})))
          throw std::logic_error("Should have been able to set this cell");
      }
    }
    /*auto contours = */g.GenerateContourLines({100000,200000,300000,400000,500000,600000,700000});

    //are these right?

  }

}

int main() {
  test::suite suite("gridded");

  suite.test(TEST_CASE(test_gridded));

  return suite.tear_down();
}
