#include "gurka.h"
#include "test.h"

using namespace valhalla;

TEST(warnings, routes_endpoint) {
  const std::string ascii_map = R"(
          A----------B-------------C-----P
          |          |                   |
          |          |                   |
          |          3--------U     G----S
          |          |        |     |    |
          D----------E--------4-----F----L
     )";
  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"name", "RT 1"}}},
      {"BC", {{"highway", "motorway"}, {"name", "RT 2"}}},
      {"CP", {{"highway", "motorway"}, {"name", "RT 3"}}},
      {"AD", {{"highway", "motorway"}, {"name", "RT 4"}}},
      {"B3", {{"highway", "motorway"}, {"name", "RT 5"}}},
      {"3E", {{"highway", "motorway"}, {"name", "RT 6"}}},
      {"3U", {{"highway", "motorway"}, {"name", "RT 7"}}},
      {"E4", {{"highway", "motorway"}, {"name", "RT 8"}}},
      {"U4", {{"highway", "motorway"}, {"name", "RT 9"}}},
      {"4F", {{"highway", "motorway"}, {"name", "RT 10"}}},
      {"GF", {{"highway", "motorway"}, {"name", "RT 11"}}},
      {"GS", {{"highway", "motorway"}, {"name", "RT 12"}}},
      {"PS", {{"highway", "motorway"}, {"name", "RT 13"}}},
      {"FL", {{"highway", "motorway"}, {"name", "RT 14"}}},
      {"SL", {{"highway", "motorway"}, {"name", "RT 15"}}},
  };
  const double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/routes_warnings");
  valhalla::Api result =
      gurka::do_action(valhalla::Options::route, map, {"A", "L"}, "hov", {{"/best_paths", "2"}});
  ASSERT_TRUE(result.info().warnings_size() != 0);
  EXPECT_EQ(result.info().warnings_size(), 2);
}
