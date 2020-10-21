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

TEST(shortcuts, test_shortcut_speed) {
  // At C node turn duration is present. As a result an average speed for AE shortcut
  // is decreased from 100 kph to 93 kph and for EA shortcut - from 100 kph to 98 kph
  // in the test case below
  const std::string ascii_map = R"(A-----B\
                                   |       \C
                                   |        |\
                                   G        | \
                                            F  \
                                                |
                                                |
                                                |
                                                D
                                                |
                                                |
                                                |
                                                E
                                                |
                                                I)";
  const gurka::ways ways = {
      {"ABCDE", {{"highway", "motorway"}, {"maxspeed", "100"}}},
      {"CF", {{"highway", "service"}}},
      {"AG", {{"highway", "service"}}},
      {"EI", {{"highway", "service"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_shortcut");
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  auto route = gurka::route(map, "A", "E", "auto");
  std::cout << route.trip().routes(0).legs(0).shape() << std::endl;

  std::vector<std::pair<baldr::GraphId, int>> shortcut_infos;
  auto tileset = reader.GetTileSet(0);
  for (const auto tileid : tileset) {
    if (reader.OverCommitted())
      reader.Trim();

    // for each edge in the tile
    const auto* tile = reader.GetGraphTile(tileid);
    for (size_t j = 0; j < tile->header()->directededgecount(); ++j) {
      // skip it if its not a shortcut or the shortcut is one we will never traverse
      const auto* edge = tile->directededge(j);
      if (!edge->is_shortcut() || !(edge->forwardaccess() & baldr::kAutoAccess))
        continue;

      // make a graph id out of the shortcut to send to recover
      auto shortcutid = tileid;
      shortcutid.set_id(j);
      shortcut_infos.push_back(std::make_pair(shortcutid, edge->speed()));
    }
  }

  ASSERT_EQ(shortcut_infos.size(), 2);

  for (auto const shortcut_info : shortcut_infos) {
    auto shortcutid = shortcut_info.first;
    auto const shortcut_speed = shortcut_info.second;
    auto edgeids = reader.RecoverShortcut(shortcutid);

    // if it gave us back the shortcut we failed
    ASSERT_FALSE(edgeids.front() == shortcutid);

    // Compare the speed on the recovered edges to the speed on the shortcut
    // Shortcut speed should be lower because it is calculated including turn duration
    std::vector<midgard::PointLL> recovered_shape;
    for (auto edgeid : edgeids) {
      const auto* tile = reader.GetGraphTile(edgeid);
      const auto* de = tile->directededge(edgeid);
      EXPECT_GT(de->speed(), shortcut_speed);
    }
  }
}
