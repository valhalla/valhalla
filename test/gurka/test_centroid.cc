#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(centroid, minimal) {
  const std::string ascii_map = R"(A-----B--1--C-----D)";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_centroid_minimal");
  auto api = gurka::do_action(Options::centroid, map, {"A", "D"}, "pedestrian");
  ASSERT_EQ(api.trip().routes_size(), 2);
  ASSERT_EQ(api.trip().routes(0).legs_size(), 1);
  ASSERT_EQ(api.trip().routes(1).legs_size(), 1);
  ASSERT_EQ(api.trip().routes(0).legs(0).location(1).ll().lat(),
            api.trip().routes(1).legs(0).location(1).ll().lat());
  ASSERT_EQ(api.trip().routes(0).legs(0).location(1).ll().lng(),
            api.trip().routes(1).legs(0).location(1).ll().lng());
  ASSERT_NEAR(map.nodes["1"].lat(), api.trip().routes(0).legs(0).location(1).ll().lat(), 0.0000001);
  ASSERT_NEAR(map.nodes["1"].lng(), api.trip().routes(0).legs(0).location(1).ll().lng(), 0.0000001);
}
