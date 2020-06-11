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

  const gurka::ways ways = {{"ABC", {{"highway", "secondary"}}},
                            {"CEF", {{"highway", "tertiary"}}},
                            {"BDE", {{"highway", "service"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_avoid_service");

  auto result = gurka::route(map, "A", "F", "bicycle");

  gurka::assert::osrm::expect_route(result, {"ABC", "CEF"});
  //  gurka::assert::raw::expect_path_length(result, 1.0, .001);
}
