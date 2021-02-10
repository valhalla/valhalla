#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

/*************************************************************/
TEST(Standalone, avoid_service) {

  // Test to make sure bicycle routes do not use highway:service to cut
  // corners near intersections
  const std::string ascii_map = R"(
    A----B----C
         |    |
         D----E
              |
              F)";

  const gurka::ways ways = {{"AB", {{"highway", "secondary"}}}, {"BC", {{"highway", "secondary"}}},
                            {"CE", {{"highway", "tertiary"}}},  {"EF", {{"highway", "tertiary"}}},
                            {"BD", {{"highway", "service"}}},   {"DE", {{"highway", "service"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_avoid_service");

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "bicycle");

  // Make sure the path doesn't take service road "B-D-E"
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CE", "EF"});
}
