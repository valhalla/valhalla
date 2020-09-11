#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, Shortest) {
  const std::string ascii_map = R"(
      A--B----C--D
         |    |
         E----F
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}, {"maxspeed", "25"}}},
      {"BC", {{"highway", "motorway"}, {"maxspeed", "25"}}},
      {"CD", {{"highway", "motorway"}, {"maxspeed", "25"}}},
      {"BE", {{"highway", "motorway"}, {"maxspeed", "120"}}},
      {"EF", {{"highway", "motorway"}, {"maxspeed", "120"}}},
      {"FC", {{"highway", "motorway"}, {"maxspeed", "120"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_shortest");

  // fastest (default) route
  valhalla::Api fastest = gurka::route(map, "A", "D", "auto");
  auto summary_fastest = fastest.directions().routes(0).legs(0).summary();
  double length_fastest = summary_fastest.length();
  double time_fastest = summary_fastest.time();

  gurka::assert::raw::expect_path(fastest, {"AB", "BE", "EF", "FC", "CD"});

  // shortest route 
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto", "costing_options": {"auto": {"shortest": true}}})") %
       std::to_string(map.nodes.at("A").lat()) % std::to_string(map.nodes.at("A").lng()) %
       std::to_string(map.nodes.at("D").lat()) % std::to_string(map.nodes.at("D").lng()))
          .str();
  valhalla::Api shortest = gurka::route(map, request);
  auto summary_shortest = shortest.directions().routes(0).legs(0).summary();
  double length_shortest = summary_shortest.length();
  double time_shortest = summary_shortest.time();

  gurka::assert::raw::expect_path(shortest, {"AB", "BC", "CD"});

  // Much more time, much less distance for shortest: true
  ASSERT_GT(length_fastest, length_shortest);
  ASSERT_LT(time_fastest, time_shortest);
}
