#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {

TEST(Standalone, ExcludeFerry) {
  const std::string ascii_map = R"(
    A----1---B----C--D------E
  )";

  const gurka::ways ways = {
      {"AB", {{"route", "ferry"}}},
      {"BC", {{"highway", "secondary"}}},
      {"CD", {{"highway", "secondary"}}},
      {"DE", {{"highway", "secondary"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500);
  const auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/exclude_ferry");
  const auto default_result =
      gurka::do_action(valhalla::Options::route, map, {"1", "D"}, "pedestrian");
  gurka::assert::raw::expect_path(default_result, {"AB", "BC", "CD"});

  const auto result = gurka::do_action(valhalla::Options::route, map, {"1", "D"}, "pedestrian",
                                       {{"/costing_options/pedestrian/exclude_ferry", "1"}});
  gurka::assert::raw::expect_path(result, {"BC", "CD"});
}
} // namespace