#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, RestServiceArea) {
  const std::string ascii_map = R"(
                          A
                          |
                          | 
                          B----C
                          |
                          |
                          D
                          |
                          |
                          E----F
  )";

  const gurka::ways ways =
      {{"AB", {{"highway", "motorway"}}},
       {"BC", {{"highway", "motorway"}, {"service", "rest_area"}}},
       {"BD", {{"highway", "motorway"}}},
       {"DE", {{"highway", "motorway"}}},
       {"EF", {{"highway", "motorway"}, {"service", "rest_area"}, {"amenity", "yes"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_rest_service_area");

  auto result = gurka::route(map, "A", "C", "auto");
  // rest_area
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(1).edge().use(), TripLeg::Use::TripLeg_Use_kRestAreaUse); // BC

  // service_area
  result = gurka::route(map, "A", "F", "auto");
  leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(3).edge().use(), TripLeg::Use::TripLeg_Use_kServiceAreaUse); // EF
}
