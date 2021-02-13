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
      {"BC", {{"highway", "residential"}, {"maxwidth", "2.0"}}},
      {"CD", {{"highway", "residential"}, {"maxwidth", "2.0m"}}},
      {"DE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}}},
      {"FG", {{"highway", "residential"}}},
      {"GH", {{"highway", "residential"}}},
      {"HI", {{"highway", "residential"}}},
      {"DG", {{"highway", "residential"}, {"maxwidth", "1.9"}}},
      {"BI", {{"highway", "residential"}, {"maxwidth", "1.9m"}}},

  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_maxwidth");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "auto");

  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI"});

  auto from = "A";
  auto to = "I";
  // set odd width just to confirm we get the correct route.
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"truck","costing_options":{"truck":{"width":1.8}}})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();

  result = gurka::do_action(valhalla::Options::route, map, request);
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI"});

  result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "bus");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI"});

  result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "taxi");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI"});

  result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "hov");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI"});

  result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "bicycle");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BI"});

  result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "pedestrian");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BI"});

  result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "motorcycle");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BI"});

  result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "motor_scooter");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BI"});
}
