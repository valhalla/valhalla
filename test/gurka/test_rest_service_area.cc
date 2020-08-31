#include "gurka.h"
#include <gtest/gtest.h>

#include "proto/trip.pb.h"
#include "odin/enhancedtrippath.h"

using namespace valhalla;

TEST(Standalone, RestServiceArea) {
  const std::string ascii_map = R"(
                          A
                          | 
                     C----B
                          |
                          D
                          |
                          E----F
  )";

  const gurka::ways ways = {
      {"AB", {"highway", "motorway"}},
      {"BC", {"service", "rest_area"}},
      {"BD", {"highway", "motorway"}},
      {"DE", {"highway", "motorway"}},
      {"EF", {"service", "rest_area"}, {"amenity", "yes"}}
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_rest_service_area");
  auto result = gurka::route(map, "A", "F", "auto");

  auto maneuver = result.directions().routes(0).legs(0).maneuver(1);

  odin::EnhancedTripLeg etl(*result.mutable_trip()->mutable_routes(0)->mutable_legs(0));
  auto curr_edge = etl.GetCurrEdge(maneuver.begin_path_index());

  ASSERT_TRUE(curr_edge);
  EXPECT_EQ(curr_edge->use(), TripLeg::Use::TripLeg_Use_kRestAreaUse);  // BC

  maneuver = result.directions().routes(0).legs(0).maneuver(4);
  curr_edge = etl.GetCurrEdge(maneuver.begin_path_index());
  EXPECT_EQ(curr_edge->use(), TripLeg::Use::TripLeg_Use_kServiceAreaUse);  // EF
}
