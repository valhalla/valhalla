#include "midgard/gridded_data.h"
#include "midgard/pointll.h"
#include "test.h"
#include <limits>
//#include <iostream>

using namespace valhalla::midgard;

namespace {

void test_gridded() {
  // fill this as distance from center
  GriddedData<PointLL> g({-5, -5, 5, 5}, 1, std::numeric_limits<float>::max());
  for (int i = 0; i < 10; ++i) {
    for (int j = 0; j < 10; ++j) {
      Tiles<PointLL> t({-5, -5, 5, 5}, 1);
      // NOTE: we aren't setting the center because the contour algorithm uses bottom left
      auto b = t.Base(t.TileId(i, j));
      if (!g.Set(b, PointLL(0, 0).Distance(b)))
        throw std::logic_error("Should have been able to set this cell");
    }
  }

  // make the contours
  std::vector<float> iso_markers{100000, 200000, 300000, 400000, 500000, 600000};
  auto contours = g.GenerateContours(iso_markers, true);

  // need to be the same size and all of them have to have a single ring
  if (contours.size() != iso_markers.size())
    throw std::logic_error("There should be 7 iso lines");

  // because of the pattern above we should end up with concentric circles
  // every ring should have all smaller rings inside it
  size_t rings = 0;
  for (auto collection = std::next(contours.rbegin()); collection != contours.rend(); ++collection) {
    // nothing here
    auto& contour = collection->second.front();
    if (contour.empty())
      continue;
    ++rings;
    // if this is a ring the iso lines with lesser units should be contained within it
    for (const auto& p : std::prev(collection)->second.front().front()) {
      if (!p.WithinPolygon(contour.front()))
        throw std::logic_error("Ring " + std::to_string(collection->first) + " should contain ring " +
                               std::to_string(std::prev(collection)->first));
    }
  }

  // there should be quite a few rings here
  if (rings == 0)
    throw std::logic_error("There should be at least a few rings here");

  /*
  std::cout << "{\"type\":\"FeatureCollection\",\"features\":[";
  for(const auto& feature : contours) {
    std::cout << "{\"type\":\"Feature\",\"properties\":{\"iso\": " << feature.first << "},";
    std::cout << "\"geometry\":{\"type\":\"MultiLineString\",\"coordinates\":[";
    for(const auto& line : feature.second) {
      std::cout << "[";
      for(const auto& coord : line) {
        std::cout << "[" << coord.first << "," << coord.second << "]" << (&coord != &line.back() ?
  "," : "");
      }
      std::cout << "]"  << (&line != &feature.second.back() ? "," : "");
    }
    std::cout << "]}}" << (&feature != &*std::prev(contours.cend()) ? "," : "");
  }
  std::cout << "]}";*/
}

} // namespace

int main() {
  test::suite suite("gridded");

  suite.test(TEST_CASE(test_gridded));

  return suite.tear_down();
}
