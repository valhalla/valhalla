#include "gurka.h"
#include "mjolnir/osmway.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;
using namespace valhalla::baldr;

TEST(area_routing, square_skipped_routes_around) {
  const std::string ascii_map = R"(
    F--------G
    |        |
    A----B   |
    |    |   |
    D----C   |
         |   |
         E---H
  )";

  const gurka::ways ways = {
      {"ABCDA", {{"highway", "pedestrian"}, {"area", "yes"}, {"name", "square"}}},
      {"FG", {{"highway", "footway"}, {"name", "top"}}},
      {"GH", {{"highway", "footway"}, {"name", "right"}}},
      {"EH", {{"highway", "footway"}, {"name", "bottom"}}},
      {"FA", {{"highway", "footway"}, {"name", "entry"}}},
      {"CE", {{"highway", "footway"}, {"name", "exit"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_area_square",
                               {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}});

  const auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));

  // we don't want to route the perimeter of an area, so if it is a pedestrian area, do not make edges
  EXPECT_THROW(gurka::findEdgeByNodes(*reader, map.nodes, "A", "B"), std::runtime_error);
  EXPECT_THROW(gurka::findEdgeByNodes(*reader, map.nodes, "B", "C"), std::runtime_error);
  EXPECT_THROW(gurka::findEdgeByNodes(*reader, map.nodes, "C", "D"), std::runtime_error);
  EXPECT_THROW(gurka::findEdgeByNodes(*reader, map.nodes, "D", "A"), std::runtime_error);

  // if the square is correctly skipped it should route around the outside (F -> G -> H -> E).
  // once area routing is implemented the route should cross the square
  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "E"}, "pedestrian");
  gurka::assert::raw::expect_path(result, {"top", "right", "bottom"});
}

TEST(area_routing, area_bit_set_when_enabled) {
  const std::string ascii_map = R"(
    A----B
    |    |
    D----C
  )";
  const gurka::ways ways = {
      {"ABCDA", {{"highway", "pedestrian"}, {"area", "yes"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

  // build with pedestrian_areas enabled, stop after parsing ways
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_area_bit_on",
                               {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}},
                               mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kParseWays);

  auto way = gurka::findWay(map, "ABCDA");
  // we expect the area_bit to be true
  EXPECT_TRUE(way.area());
}

TEST(area_routing, area_bit_not_set_when_disabled) {
  const std::string ascii_map = R"(
    A----B
    |    |
    D----C
  )";
  const gurka::ways ways = {
      {"ABCDA", {{"highway", "pedestrian"}, {"area", "yes"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

  // build without the flag (defaults to false), stop after parsing ways
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_area_bit_off",
                               {{"mjolnir.concurrency", "1"}}, mjolnir::BuildStage::kInitialize,
                               mjolnir::BuildStage::kParseWays);

  auto way = gurka::findWay(map, "ABCDA");
  // we expect the area_bit to be false
  EXPECT_FALSE(way.area());
}