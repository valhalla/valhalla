#include "midgard/gridded_data.h"
#include "midgard/pointll.h"
#include <limits>
//#include <iostream>

#include "test.h"

using namespace valhalla::midgard;

namespace {

TEST(GriddedData, Basic) {
  // fill this as distance from center
  GriddedData<PointLL> g({-5, -5, 5, 5}, 1, std::numeric_limits<float>::max());
  for (int i = 0; i < 10; ++i) {
    for (int j = 0; j < 10; ++j) {
      Tiles<PointLL> t({-5, -5, 5, 5}, 1);
      // NOTE: we aren't setting the center because the contour algorithm uses bottom left
      auto b = t.Base(t.TileId(i, j));
      ASSERT_TRUE(g.Set(b, PointLL(0, 0).Distance(b)));
    }
  }

  // make the contours
  std::vector<float> iso_markers{100000, 200000, 300000, 400000, 500000, 600000};
  auto contours = g.GenerateContours(iso_markers, true);

  // need to be the same size and all of them have to have a single ring
  ASSERT_EQ(contours.size(), iso_markers.size()) << "There should be 7 iso lines";

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
      ASSERT_TRUE(p.WithinPolygon(contour.front()))
          << "Ring " + std::to_string(collection->first) + " should contain ring " +
                 std::to_string(std::prev(collection)->first);
    }
  }

  // there should be quite a few rings here
  ASSERT_NE(rings, 0) << "There should be at least a few rings here";

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

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}