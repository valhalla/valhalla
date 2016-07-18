#include "test.h"
#include "midgard/gridded_data.h"
#include "midgard/pointll.h"
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
    auto contours = g.GenerateContourLines({100000,200000,300000,400000,500000,600000,700000});

    //need to be the same size and all of them have to have a single ring
    if(contours.size() != 7)
      throw std::logic_error("There should be 7 iso lines");
    for(const auto& contour : contours)
      if(contour.size() != 1)
        throw std::logic_error("Each contour should be a single ring");

    //because of the pattern above we shoul end up with concentric circles
    //every ring should have all smaller rings inside it
    for(size_t i = 0; i < contours.size(); ++i) {
      for(size_t j = i + 1; j < contours.size(); ++j) {
        for(const auto& p : contours[j])
          if(!p.WithinConvexPolygon(contours[i]))
            throw std::logic_error("Ring " + std::to_string(i) + " should contain ring " + std::to_string(j));
      }
    }
  }

}

int main() {
  test::suite suite("gridded");

  suite.test(TEST_CASE(test_gridded));

  return suite.tear_down();
}
