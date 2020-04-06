#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class UseDirectionOnWays : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
         A----B----C----D
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}, {"ref", "US 30;US 222"}, {"direction", "East;North"}}},
        {"BC", {{"highway", "primary"}, {"int_ref", "I-95;US 40"}, {"int_direction", "North;East"}}},
        {"CD",
         {{"highway", "primary"},
          {"ref", "US 15"},
          {"direction", "South"},
          {"int_ref", "I-80;US 15"},
          {"int_direction", "West;South"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/use_direction_on_ways",
                            {{"mjolnir.data_processing.use_direction_on_ways", "true"}});
  }
};

gurka::map UseDirectionOnWays::map = {};

/*************************************************************/

TEST_F(UseDirectionOnWays, CheckNamesAndRefs) {
  auto result = gurka::route(map, "A", "D", "auto");
  gurka::assert::osrm::expect_route(result, {"AB", "BC", "CD"});

  // test direction is used correctly
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name_size(), 3);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "AB");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(1).value(), "US 30 East");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(2).value(), "US 222 North");

  // int_refs are copied to refs
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).street_name_size(), 3);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).street_name(0).value(), "BC");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).street_name(1).value(), "I-95 North");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).street_name(2).value(), "US 40 East");

  // ensure dups are removed
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(2).street_name_size(), 3);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(2).street_name(0).value(), "CD");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(2).street_name(1).value(), "US 15 South");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(2).street_name(2).value(), "I-80 West");
}
