#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;

/* A test that forces 1 side of the bidirectional A* searcjh to exhaust
 * before the other directionm has a chance to run, incorrectly causing
 * a no route.
 */
TEST(StandAlone, not_thru) {
  const std::string ascii_map = R"(
  A--B--C--D--E--F
  )";

  // Make the 3 not_thru edges slow
  const gurka::ways ways = {{"AB", {{"highway", "secondary"}, {"maxspeed", "10"}}},
                            {"BC", {{"highway", "secondary"}, {"maxspeed", "10"}}},
                            {"CD", {{"highway", "secondary"}, {"maxspeed", "10"}}},
                            {"DE", {{"highway", "motorway"}}},
                            {"EF", {{"highway", "motorway"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const std::string tile_dir = "test/data/not_thru";
  auto map = gurka::buildtiles(layout, ways, {}, {}, tile_dir);
  std::shared_ptr<baldr::GraphReader> reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));

  // mark AB, BC & CD not_thru in both directions
  std::vector<GraphId> not_thru_edgeids;
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, "A", "B")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, "B", "A")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, "B", "C")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, "C", "B")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, "C", "D")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, "D", "C")));

  test::customize_edges(map.config, [&not_thru_edgeids](const GraphId& edgeid, DirectedEdge& edge) {
    if (std::find(not_thru_edgeids.begin(), not_thru_edgeids.end(), edgeid) != not_thru_edgeids.end()) {
      edge.set_not_thru(true);
    }
  });

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto", {});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF"});
}
