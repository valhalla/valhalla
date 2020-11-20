#include "gurka.h"
#include <gtest/gtest.h>

#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "midgard/pointll.h"

using namespace valhalla;

// Here 2 shortcuts should be created: from A to C all edges have speed 80 and from C to A all have
// speed 90
TEST(Shortcuts, test_create_valid) {
  constexpr double gridsize = 50;

  const std::string ascii_map = R"(
      A---B--C
  )";
  const gurka::ways ways = {
      {"AB",
       {{"highway", "primary"},
        {"name", "Independence Avenue"},
        {"maxspeed:forward", "80"},
        {"maxspeed:backward", "90"}}},
      {"BC",
       {{"highway", "primary"},
        {"name", "Independence Avenue"},
        {"maxspeed:forward", "80"},
        {"maxspeed:backward", "90"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_openlrjoiner_shortcut_speed");

  baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));

  // find shortcut edges
  auto shortcut_edge = std::get<1>(gurka::findEdgeByNodes(graph_reader, layout, "A", "C"));
  auto opp_shortcut_edged = std::get<1>(gurka::findEdgeByNodes(graph_reader, layout, "C", "A"));

  EXPECT_EQ(shortcut_edge->speed(), 80);
  EXPECT_EQ(opp_shortcut_edged->speed(), 90);
}

// Here no shortcuts are created. There could be one from A to C with speed 80 but in the opposite
// direction speeds differ which blocks CA creation.
TEST(Shortcuts, test_create_invalid) {
  constexpr double gridsize = 50;

  const std::string ascii_map = R"(
      A--B--C
  )";

  const gurka::ways ways = {
      {"AB",
       {{"highway", "primary"},
        {"name", "Independence Avenue"},
        {"maxspeed:forward", "80"},
        {"maxspeed:backward", "80"}}},
      {"BC",
       {{"highway", "primary"},
        {"name", "Independence Avenue"},
        {"maxspeed:forward", "80"},
        {"maxspeed:backward", "90"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_openlrjoiner_shortcut_speed");

  baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));

  // check that there are no shortcut edges
  EXPECT_ANY_THROW(gurka::findEdgeByNodes(graph_reader, layout, "A", "C"));
  EXPECT_ANY_THROW(gurka::findEdgeByNodes(graph_reader, layout, "C", "A"));
}
