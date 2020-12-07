#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, Maxspeed) {
  const std::string ascii_map = R"(
      A----B----C----D----E
           |         |    |
           |         |    |
           I----H----G----F
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}, {"maxwidth", "1.9"}}},
      {"CD", {{"highway", "residential"}, {"maxwidth", "1.9m"}}},
      {"DE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}}},
      {"FG", {{"highway", "residential"}}},
      {"GH", {{"highway", "residential"}}},
      {"HI", {{"highway", "residential"}}},
      {"DG", {{"highway", "residential"}, {"maxwidth", "1.8"}}},
      {"BI", {{"highway", "residential"}, {"maxwidth", "1.8m"}}},

  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_maxwidth");
  auto result = gurka::route(map, "A", "I", "auto");

  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI"});
}
