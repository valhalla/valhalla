#include "gurka.h"
#include <gtest/gtest.h>

#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "midgard/pointll.h"
#include "mjolnir/graphtilebuilder.h"

using namespace valhalla;

// Here 2 shortcuts should be created: from A to C all edges have speed 80 and from C to A all have
// speed 90
TEST(Shortcuts, CreateValid) {
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
TEST(Shortcuts, CreateInvalid) {
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

TEST(Shortcuts, ShortcutSpeed) {
  // At C node turn duration is present. As a result the speed for AE shortcut is decreased
  // from 100 kph to 93 kph and for EA shortcut - from 100 kph to 98 kph in the test case below.
  // Similarly truck speed is decreased form 90 kph to 85 kph for AE and from 90 to 89 for EA.
  const std::string ascii_map = R"(A-----B\
                                           \C
                                            |\
                                            | \
                                            F  \
                                                |
                                                |
                                                |
                                                D
                                                |
                                                |
                                                |
                                                E)";
  const gurka::ways ways = {
      {"AB",
       {{"highway", "motorway"},
        {"maxspeed", "100"},
        {"maxspeed:hgv", "90"},
        {"name", "High street"}}},
      {"BC",
       {{"highway", "motorway"},
        {"maxspeed", "100"},
        {"maxspeed:hgv", "90"},
        {"name", "High street"}}},
      {"CD",
       {{"highway", "motorway"},
        {"maxspeed", "100"},
        {"maxspeed:hgv", "90"},
        {"name", "High street"}}},
      {"DE",
       {{"highway", "motorway"},
        {"maxspeed", "100"},
        {"maxspeed:hgv", "90"},
        {"name", "High street"}}},
      {"CF", {{"highway", "service"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_shortcut");
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  std::vector<std::tuple<baldr::GraphId, int, int>> shortcut_infos;
  auto const tileset = reader.GetTileSet(0);
  for (const auto tileid : tileset) {
    if (reader.OverCommitted())
      reader.Trim();

    // for each edge in the tile
    auto tile = reader.GetGraphTile(tileid);
    for (size_t j = 0; j < tile->header()->directededgecount(); ++j) {
      // skip it if its not a shortcut or the shortcut is one we will never traverse
      const auto* edge = tile->directededge(j);
      if (!edge->is_shortcut() || !(edge->forwardaccess() & baldr::kAutoAccess))
        continue;

      // make a graph id out of the shortcut to send to recover
      auto shortcutid = tileid;
      shortcutid.set_id(j);
      shortcut_infos.push_back(std::make_tuple(shortcutid, edge->speed(), edge->truck_speed()));
      // For current test case the values should be different
      EXPECT_NE(edge->speed(), edge->truck_speed());
    }
  }

  ASSERT_EQ(shortcut_infos.size(), 2);

  for (auto const shortcut_info : shortcut_infos) {
    auto const shortcutid = std::get<0>(shortcut_info);
    auto const shortcut_speed = std::get<1>(shortcut_info);
    auto const shortcut_truck_speed = std::get<2>(shortcut_info);
    auto const edgeids = reader.RecoverShortcut(shortcutid);

    // if it gave us back the shortcut we failed
    ASSERT_FALSE(edgeids.front() == shortcutid);

    // Compare the speed on the recovered edges to the speed on the shortcut
    // Shortcut speed should be lower because it is calculated including turn duration
    std::vector<midgard::PointLL> recovered_shape;
    for (auto const edgeid : edgeids) {
      auto tile = reader.GetGraphTile(edgeid);
      const auto* de = tile->directededge(edgeid);
      EXPECT_GT(de->speed(), shortcut_speed);
      EXPECT_GT(de->truck_speed(), shortcut_truck_speed);
    }
  }
}

TEST(Shortcuts, TruckSpeedNotSet) {
  // When truck speed is not set normal speed is used to calculate shortcut truck speed.
  // As a result it should be equal to normal shortcut speed.
  const std::string ascii_map = R"(A-----B\
                                           \C
                                            |\
                                            | \
                                            F  \
                                                |
                                                |
                                                |
                                                D
                                                |
                                                |
                                                |
                                                E)";
  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}, {"maxspeed", "100"}, {"name", "High street"}}},
      {"BC", {{"highway", "motorway"}, {"maxspeed", "100"}, {"name", "High street"}}},
      {"CD", {{"highway", "motorway"}, {"maxspeed", "100"}, {"name", "High street"}}},
      {"DE", {{"highway", "motorway"}, {"maxspeed", "100"}, {"name", "High street"}}},
      {"CF", {{"highway", "service"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_shortcut");
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  bool found_shortcut = false;
  auto const tileset = reader.GetTileSet(0);
  for (const auto tileid : tileset) {
    if (reader.OverCommitted())
      reader.Trim();

    // for each edge in the tile
    auto tile = reader.GetGraphTile(tileid);
    for (size_t j = 0; j < tile->header()->directededgecount(); ++j) {
      // skip it if its not a shortcut or the shortcut is one we will never traverse
      const auto* edge = tile->directededge(j);
      if (!edge->is_shortcut() || !(edge->forwardaccess() & baldr::kAutoAccess))
        continue;

      EXPECT_EQ(edge->speed(), edge->truck_speed());
      found_shortcut = true;
    }
  }
  EXPECT_TRUE(found_shortcut) << "No shortcuts found. Check the map.";
}

TEST(Shortcuts, TruckSpeedPartiallySet) {
  // When truck speed is not set normal speed is used to calculate shortcut truck speed.
  // As a result, when truck speed is set only for some of constituent edges, resulting speed is
  // range from truck speed to normal speed
  const std::string ascii_map = R"(A---B---C---D---E)";
  const gurka::ways ways = {
      {"AB",
       {{"highway", "motorway"},
        {"maxspeed", "100"},
        {"maxspeed:hgv", "90"},
        {"name", "High street"}}},
      {"BC",
       {{"highway", "motorway"},
        {"maxspeed", "100"},
        {"maxspeed:hgv", "90"},
        {"name", "High street"}}},
      {"CD", {{"highway", "motorway"}, {"maxspeed", "100"}, {"name", "High street"}}},
      {"DE", {{"highway", "motorway"}, {"maxspeed", "100"}, {"name", "High street"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_shortcut");
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  bool found_shortcut = false;
  auto const tileset = reader.GetTileSet(0);
  for (const auto tileid : tileset) {
    if (reader.OverCommitted())
      reader.Trim();

    // for each edge in the tile
    auto tile = reader.GetGraphTile(tileid);
    for (size_t j = 0; j < tile->header()->directededgecount(); ++j) {
      // skip it if its not a shortcut or the shortcut is one we will never traverse
      const auto* edge = tile->directededge(j);
      if (!edge->is_shortcut() || !(edge->forwardaccess() & baldr::kAutoAccess))
        continue;

      EXPECT_GT(100, edge->truck_speed());
      EXPECT_LT(90, edge->truck_speed());
      found_shortcut = true;
    }
  }
  EXPECT_TRUE(found_shortcut) << "No shortcuts found. Check the map.";
}

TEST(Shortcuts, ShortcutsInBins) {
  const std::string ascii_map = R"(A---B---C)";
  const gurka::ways ways = {{"AB", {{"highway", "motorway"}, {"name", "High Street"}}},
                            {"BC", {{"highway", "motorway"}, {"name", "High Street"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_shortcut");
  GraphReader reader(map.config.get_child("mjolnir"));

  std::vector<GraphId> edge_ids;
  auto bin_tileset = reader.GetTileSet(2);
  for (auto tileid : bin_tileset) {
    if (reader.OverCommitted())
      reader.Trim();
    auto tile = reader.GetGraphTile(tileid);
    for (size_t j = 0; j < kBinCount; ++j) {
      auto bin = tile->GetBin(j);
      for (auto edge_id : bin) {
        edge_ids.push_back(edge_id);
      }
    }
  }

  size_t shortcut_cnt = 0;
  for (auto edge_id : edge_ids) {
    auto* edge = reader.directededge(edge_id);
    if (edge->is_shortcut()) {
      shortcut_cnt += 1;
    }
  }

  EXPECT_EQ(shortcut_cnt, 1);
}

TEST(Shortcuts, ShortcutRestrictions) {
  const std::string ascii_map = R"(
  A--B--C--D--E--F

  G--H--I--J--K--L
  )";

  // the first line should produce only one shortcut, the second two
  gurka::ways ways = {
      {"AB", {{"highway", "motorway"}, {"name", "highway"}}},
      {"BC", {{"highway", "motorway"}, {"name", "highway"}}},
      {"CD", {{"highway", "motorway"}, {"name", "highway"}, {"maxweight", "3.5"}}},
      {"DE", {{"highway", "motorway"}, {"name", "highway"}, {"maxweight", "8"}}},
      {"EF", {{"highway", "motorway"}, {"name", "highway"}}},

      {"GH", {{"highway", "motorway"}, {"name", "highway"}}},
      {"HI", {{"highway", "motorway"}, {"name", "highway"}, {"hazmat", "yes"}}},
      {"IJ", {{"highway", "motorway"}, {"name", "highway"}, {"hazmat", "yes"}}},
      {"JK", {{"highway", "motorway"}, {"name", "highway"}}},
      {"KL", {{"highway", "motorway"}, {"name", "highway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500);
  auto map = gurka::buildtiles(layout, ways, {}, {},
                               VALHALLA_BUILD_DIR "test/data/gurka_shortcut_restrictions");
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  // test we got the right shortcuts edges, implicitly means they were broken properly
  for (const auto& end_node : {"C", "J", "L"}) {
    const auto shortcut =
        gurka::findEdge(reader, layout, "highway", end_node, baldr::GraphId{}, 0, true);
    EXPECT_TRUE(std::get<1>(shortcut)->is_shortcut());
    EXPECT_TRUE(std::get<3>(shortcut)->is_shortcut());
    EXPECT_EQ(std::get<1>(shortcut)->length(), 3000);
    EXPECT_EQ(std::get<3>(shortcut)->length(), 3000);
  }

  // test the right edges are really superseded by a shortcut
  // forward
  for (const auto& end_node : {"B", "I", "K"}) {
    const auto edge = gurka::findEdge(reader, layout, "highway", end_node);
    EXPECT_NE(std::get<1>(edge)->superseded(), 0);
  }
  // reverse
  for (const auto& end_node : {"C", "J", "L"}) {
    const auto edge = gurka::findEdge(reader, layout, "highway", end_node);
    EXPECT_NE(std::get<3>(edge)->superseded(), 0);
  }

  // test that without those restrictions we're still building all shortcuts

  // remove those access restrictions
  ways["CD"].erase("maxweight");
  ways["DE"].erase("maxweight");
  ways["HI"].erase("hazmat");
  ways["IJ"].erase("hazmat");
  auto map2 = gurka::buildtiles(layout, ways, {}, {},
                                VALHALLA_BUILD_DIR "test/data/gurka_shortcut_without_restrictions");
  baldr::GraphReader reader2(map2.config.get_child("mjolnir"));

  // we don't have those small shortcuts anymore
  for (const auto& end_node : {"C", "J"}) {
    const auto shortcut =
        gurka::findEdge(reader2, layout, "highway", end_node, baldr::GraphId{}, 0, true);
    EXPECT_EQ(std::get<1>(shortcut), nullptr);
    EXPECT_EQ(std::get<3>(shortcut), nullptr);
  }

  // we did build the long shorcuts across all edges
  for (const auto& end_node : {"F", "L"}) {
    const auto shortcut =
        gurka::findEdge(reader2, layout, "highway", end_node, baldr::GraphId{}, 0, true);
    EXPECT_TRUE(std::get<1>(shortcut)->is_shortcut());
    EXPECT_TRUE(std::get<3>(shortcut)->is_shortcut());
    EXPECT_EQ(std::get<1>(shortcut)->length(), 7500);
    EXPECT_EQ(std::get<3>(shortcut)->length(), 7500);
  }
}
