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
         A----B----C   
    )";

    const gurka::ways ways = {
        {"AB",
         {{"highway", "primary"},
          {"ref", "US 30;US 222"},
          {"direction", "East;North"}}},
        {"BC",
         {{"highway", "primary"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map =
        gurka::buildtiles(layout, ways, {}, {}, "test/data/use_direction_on_ways",
                          {{"mjolnir.data_processing.use_direction_on_ways", true}});
  }
};

gurka::map UseDirectionOnWays::map = {};

/*************************************************************/

TEST_F(UseDirectionOnWays, Route) {
  auto result = gurka::route(map, "A", "C", "auto");
  gurka::assert::osrm::expect_route(result, {"WE", "BC"});
}

