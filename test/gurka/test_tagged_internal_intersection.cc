#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;

// Makes sure '"junction=intersection" forces the edges
// without influence of the infer_internal_intersection
TEST(Standalone, junction_intersection_tag_infer_enabled) {
  const std::string ascii_map = R"(
    A----B----C
         |    |
    D----E----F----G
   )";

  const gurka::ways ways =
      {{"AB", {{"highway", "trunk"}, {"oneway", "yes"}}},
       {"BC", {{"highway", "trunk"}, {"oneway", "yes"}}},
       {"FE", {{"highway", "trunk"}, {"oneway", "yes"}}},
       {"ED", {{"highway", "trunk"}, {"oneway", "yes"}}},
       {"CF", {{"highway", "trunk_link"}, {"oneway", "yes"}}},
       {"BE", {{"highway", "service"}, {"junction", "intersection"}, {"oneway", "yes"}}},
       {"FG", {{"highway", "trunk"}, {"oneway", "yes"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

  const auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_tagged_internal_intersection_0",
                        {{"mjolnir.data_processing.infer_internal_intersections", "true"}});

  const auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  const auto edge = gurka::findEdgeByNodes(*reader, map.nodes, "B", "E");
  EXPECT_TRUE(std::get<1>(edge)->internal());

  const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");
  gurka::assert::raw::expect_path(result, {"AB", "BE", "ED"});
}

TEST(Standalone, junction_intersection_tag_infer_disabled) {
  const std::string ascii_map = R"(
    A----B----C
             |       |
    D----E----F----G
   )";

  const gurka::ways ways =
      {{"AB", {{"highway", "trunk"}, {"oneway", "yes"}}},
       {"BC", {{"highway", "trunk"}, {"oneway", "yes"}}},
       {"FE", {{"highway", "trunk"}, {"oneway", "yes"}}},
       {"ED", {{"highway", "trunk"}, {"oneway", "yes"}}},
       {"CF", {{"highway", "trunk_link"}, {"oneway", "yes"}}},
       {"BE", {{"highway", "service"}, {"junction", "intersection"}, {"oneway", "yes"}}},
       {"FG", {{"highway", "trunk"}, {"oneway", "yes"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

  const auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_tagged_intersection_0",
                        {{"mjolnir.data_processing.infer_internal_intersections", "false"}});

  const auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  const auto edge = gurka::findEdgeByNodes(*reader, map.nodes, "B", "E");
  EXPECT_TRUE(std::get<1>(edge)->internal());

  const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");
  gurka::assert::raw::expect_path(result, {"AB", "BE", "ED"});
}
