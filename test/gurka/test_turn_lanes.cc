#include "gurka.h"
#include <gtest/gtest.h>

#include "odin/enhancedtrippath.h"

using namespace valhalla;

/*************************************************************/
TEST(Standalone, TurnLanes) {

  const std::string ascii_map = R"(
    A----B----C
         |
         D)";

  const gurka::ways ways = {{"ABC", {{"highway", "primary"}, {"lanes", "4"}}},
                            {"DB",
                             {{"highway", "primary"},
                              {"lanes:forward", "3"},
                              {"lanes:backward", "0"},
                              {"turn:lanes", "left|none|none"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turn_lanes_1");

  auto result = gurka::route(map, "D", "C", "auto");

  gurka::assert::osrm::expect_steps(result, {"DB", "ABC"});
  gurka::assert::raw::expect_path(result, {"DB", "ABC"});

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  gurka::assert::raw::expect_path_length(result, 0.7, 0.001);

  auto maneuver = result.directions().routes(0).legs(0).maneuver(1);
  EXPECT_EQ(maneuver.type(), DirectionsLeg_Maneuver_Type_kRight);

  odin::EnhancedTripLeg etl(*result.mutable_trip()->mutable_routes(0)->mutable_legs(0));
  auto prev_edge = etl.GetPrevEdge(maneuver.begin_path_index());

  ASSERT_TRUE(prev_edge);
  EXPECT_EQ(prev_edge->turn_lanes_size(), 3);
  EXPECT_EQ(prev_edge->TurnLanesToString(), "[ left | through | through;right ACTIVE ]");
}
