#include "test.h"
#include "midgard/gridded_data.h"
#include "midgard/pointll.h"
#include <limits>
#include <iostream>

using namespace valhalla::midgard;

namespace {

  void test_gridded() {
    //fill this as distance from center
    GriddedData<PointLL> g({-5,-5,5,5}, 1, std::numeric_limits<float>::max());
    for(int i = 0; i < 10; ++i) {
      for(int j = 0; j < 10; ++j) {
        Tiles<PointLL> t({-5,-5,5,5}, 1);
        //NOTE: we aren't setting the center because the contour algorithm uses bottom left
        auto b = t.Base(t.TileId(i,j));
        if(!g.Set(b, PointLL(0,0).Distance(b)))
          throw std::logic_error("Should have been able to set this cell");
      }
    }

    std::vector<float> iso_markers{100000,200000,300000,400000,500000,600000,700000};
    auto contours = g.GenerateContours(iso_markers);

    //need to be the same size and all of them have to have a single ring
    if(contours.size() != iso_markers.size())
      throw std::logic_error("There should be 7 iso lines");

    //because of the pattern above we should end up with concentric circles
    //every ring should have all smaller rings inside it
    for(size_t i = 1; i < contours.size(); ++i) {
      //not looking at a ring any more so we are done
      if(contours[i].front().front() != contours[i].front().back())
        break;
      //if this is a ring the iso lines with lesser units should be contained within it
      for(const auto& p : contours[i - 1].front()) {
        if(!p.WithinConvexPolygon(contours[i].front()))
          throw std::logic_error("Ring " + std::to_string(i) + " should contain ring " + std::to_string(i - 1));
      }
    }
  }

}

int main() {
  test::suite suite("gridded");

  suite.test(TEST_CASE(test_gridded));

  return suite.tear_down();
}
