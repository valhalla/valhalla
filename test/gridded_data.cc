#include "midgard/gridded_data.h"
#include "midgard/pointll.h"

#include <limits>
// #include <iostream>

#include "test.h"

using namespace valhalla::midgard;

namespace {

TEST(GriddedData, Basic) {
  // fill this as distance from center
  GriddedData<1> g({-7, -7, 7, 7}, 1, {std::numeric_limits<float>::max()});
  for (int i = 0; i < 14; ++i) {
    for (int j = 0; j < 14; ++j) {
      Tiles<PointLL> t({-7, -7, 7, 7}, 1);
      // NOTE: we aren't setting the center because the contour algorithm uses bottom left
      auto b = t.Base(t.TileId(i, j));
      float d = PointLL(0, 0).Distance(b);
      g.SetIfLessThan(t.TileId(i, j), {d});
    }
  }

  // make the contours
  std::vector<GriddedData<1>::contour_interval_t> iso_markers{
      {0, 100000, "dist", ""}, {0, 200000, "dist", ""}, {0, 300000, "dist", ""},
      {0, 400000, "dist", ""}, {0, 500000, "dist", ""}, {0, 600000, "dist", ""},
  };
  auto contours = g.GenerateContours(iso_markers, true);

  // need to be the same size and all of them have to have a single ring
  ASSERT_EQ(contours.size(), iso_markers.size()) << "There should be 7 iso lines";

  // because of the pattern above we should end up with concentric circles
  // every ring should have all smaller rings inside it
  size_t rings = 0;
  for (auto collection = contours.rbegin(); collection != contours.rend(); ++collection) {
    auto& contour = collection->front();
    ASSERT_FALSE(contour.empty());
    ++rings;
    // skip the first (smallest ring)
    if (collection == contours.rbegin())
      continue;
    // if this is a ring the iso lines with lesser units should be contained within it
    for (const auto& p : std::prev(collection)->front().front()) {
      ASSERT_TRUE(p.WithinPolygon(contour.front())) << "Ring should contain smaller ring";
    }
  }

  // there should be quite a few rings here
  ASSERT_EQ(rings, 6) << "There should be at least a few rings here";
  /*
  auto iso_marker = iso_markers.begin();
  std::cout << "{\"type\":\"FeatureCollection\",\"features\":[";
  for (const auto& feature_collection : contours) {
    for (const auto& multilinestring : feature_collection) {
      std::cout << "{\"type\":\"Feature\",\"properties\":{\"iso\": " << std::get<1>(*iso_marker)
                << "},";
      std::cout << "\"geometry\":{\"type\":\"MultiLineString\",\"coordinates\":[";
      for (const auto& linestring : multilinestring) {
        std::cout << "[";
        for (const auto& coord : linestring) {
          std::cout << "[" << coord.first << "," << coord.second << "]"
                    << (&coord != &linestring.back() ? "," : "");
        }
        std::cout << "]" << (&linestring != &multilinestring.back() ? "," : "");
      }
      std::cout << "]}}"
                << (&multilinestring != &feature_collection.back() ||
                            &feature_collection != &contours.back()
                        ? ","
                        : "");
    }
    ++iso_marker;
  }
  std::cout << "]}";
  */
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
