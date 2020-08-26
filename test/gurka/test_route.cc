#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

/*************************************************************/
TEST(Standalone, TruckRegression) {

  const std::string ascii_map = R"(A---1---------------B--------2)";

  const gurka::ways ways = {
      {"AB", {{"highway", "service"}}},
  };

  const double gridsize = 10;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/truck_regression");
  auto result = gurka::route(map, {"1", "2", "1"}, "truck", {{"/locations/1/type", "break_through"}});
  // Annoyingly because its a node snap at a break through, it starts on AB ends a leg at the end of
  // AB and then starts the next leg at the end of AB and then uturns and goes back on BA
  gurka::assert::raw::expect_path(result, {"AB", "AB", "AB"});
}

// ignoreaccess, ignorerestriction
TEST(Standalone, AutoIgnoreOneway) {

  const std::string ascii_map = R"(A---1---------------B        2)";

  const gurka::ways ways = {
      {"AB", {{"highway", "service"}, {"oneway", "yes"}}},
  };

  const double gridsize = 10;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/ignore_oneway");

  EXPECT_THROW(gurka::route(map, {"2", "1"}, "auto"), std::runtime_error);
}