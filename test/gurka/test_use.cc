#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class Use : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
                          A
                          |
                          |
                          B
                          | \
                          |  C
                          D
                          |
                          |
                          E
                          | \
                          |  F
                          G
  )";

    const gurka::ways ways = {{"AB", {{"highway", "motorway"}}},
                              {"BC",
                               {{"highway", "motorway_link"},
                                {"service", "rest_area"},
                                {"destination", "Rest Area"}}},
                              {"BD", {{"highway", "motorway"}}},
                              {"DE", {{"highway", "motorway"}}},
                              {"EF",
                               {{"highway", "motorway_link"},
                                {"service", "rest_area"},
                                {"amenity", "yes"},
                                {"destination", "Service Area"}}},
                              {"EG", {{"highway", "motorway"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_use",
                            {{"mjolnir.data_processing.use_rest_area", "true"}});
  }
};
gurka::map Use::map = {};

/*************************************************************/

TEST_F(Use, EdgeUse) {
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
