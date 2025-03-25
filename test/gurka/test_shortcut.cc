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

TEST(Shortcuts, LoopWithoutShortcut) {
  constexpr double gridsize = 50;

  const std::string ascii_map = R"(
      A--B
      |  |
      |  |
      C--D
  )";
  const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                            {"BD", {{"highway", "primary"}}},
                            {"DC", {{"highway", "primary"}}},
                            {"CA", {{"highway", "primary"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_shortcut");

  baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));

  auto loopEdge = std::get<0>(gurka::findEdgeByNodes(graph_reader, layout, "A", "B"));
  auto shortcut = graph_reader.GetShortcut(loopEdge);

  EXPECT_FALSE(shortcut.Is_Valid()) << "Shortcuts found. Check the map.";
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

  for (auto const& shortcut_info : shortcut_infos) {
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
  // When truck speed is not set normal speed is used to calculate shortcut truck speed
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

      // truck speed should be equal to edge speed by default
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
        {"maxspeed:hgv", "100"},
        {"name", "High street"}}},
      {"BC",
       {{"highway", "motorway"},
        {"maxspeed", "100"},
        {"maxspeed:hgv", "100"},
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

TEST(Shortcuts, TruckSpeedPartiallySetLow) {
  // When truck speed is not set normal speed is used to calculate shortcut truck speed.
  // As a result, when truck speed is set only for some of constituent edges, resulting speed is
  // range from truck speed to normal speed
  const std::string ascii_map = R"(A---B---C---D---E)";
  const gurka::ways ways = {
      {"AB",
       {{"highway", "motorway"},
        {"maxspeed", "100"},
        {"maxspeed:hgv", "60"},
        {"name", "High street"}}},
      {"BC",
       {{"highway", "motorway"},
        {"maxspeed", "100"},
        {"maxspeed:hgv", "60"},
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

      EXPECT_LT(60, edge->truck_speed());
      EXPECT_GT(85, edge->truck_speed());
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
  using node_pairs = std::vector<std::pair<std::string, std::string>>;

  // the first line should produce only one HUGE shortcut, the second line one small one
  const std::string ascii_map = R"(
  A--B--C--D--E--F

  G--H--I--J--K--L
  )";

  std::map<std::string, std::string> high_access_res = {{"highway", "motorway"}, {"hazmat", "yes"},
                                                        {"maxweight", "30"},     {"maxheight", "6"},
                                                        {"maxlength", "10"},     {"maxaxles", "10"}};
  std::map<std::string, std::string> low_access_res = {{"highway", "motorway"}, {"hazmat", "no"},
                                                       {"maxweight", "3"},      {"maxheight", "3"},
                                                       {"maxlength", "4"},      {"maxaxles", "4"}};

  gurka::ways ways = {
      {"AB", {{"highway", "motorway"}}},
      {"BC", {{"highway", "motorway"}}},
      {"CD", high_access_res},
      {"DE", low_access_res},
      {"EF", {{"highway", "motorway"}}},

      {"GH", {{"highway", "motorway"}}},
      {"HI", {{"highway", "motorway"}, {"motorcar:conditional", "yes @ 00:00-07:00"}}},
      {"IJ", {{"highway", "motorway"}, {"motorcar:conditional", "no @ 00:00-07:00"}}},
      {"JK", {{"highway", "motorway"}}},
      {"KL", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500);
  auto map = gurka::buildtiles(layout, ways, {}, {},
                               VALHALLA_BUILD_DIR "test/data/gurka_shortcut_restrictions");
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  // test we got the right shortcuts edges for the second line of the map
  // implicitly means they were broken properly
  for (const auto& pair : node_pairs{{"A", "F"}, {"F", "A"}, {"J", "L"}, {"L", "J"}}) {
    const auto shortcut = gurka::findEdgeByNodes(reader, layout, pair.first, pair.second);
    EXPECT_TRUE(std::get<1>(shortcut)->is_shortcut());
  }

  // test that the long shortcut has the strictest non-conditional access restrictions
  const auto AF = gurka::findEdgeByNodes(reader, layout, "A", "F");
  const auto AF_res =
      reader.GetGraphTile(std::get<0>(AF))->GetAccessRestrictions(std::get<0>(AF).id(), kAllAccess);
  EXPECT_EQ(AF_res.size(), 5);
  for (const auto& res : AF_res) {
    uint64_t expected_value = 0;
    switch (res.type()) {
      case AccessType::kHazmat:
        // should be false/0
        break;
      case AccessType::kMaxWeight:
        expected_value = strtoull(low_access_res["maxweight"].c_str(), nullptr, 10) * 100;
        break;
      case AccessType::kMaxHeight:
        expected_value = strtoull(low_access_res["maxheight"].c_str(), nullptr, 10) * 100;
        break;
      case AccessType::kMaxLength:
        expected_value = strtoull(low_access_res["maxlength"].c_str(), nullptr, 10) * 100;
        break;
      case AccessType::kMaxAxles:
        expected_value = strtoull(low_access_res["maxaxles"].c_str(), nullptr, 10);
        break;
      default:
        break;
    }
    EXPECT_EQ(res.value(), expected_value);
  }

  // test the right edges are really superseded by a shortcut
  // forward
  for (const auto& pair : node_pairs{{"A", "B"}, {"J", "K"}}) {
    const auto edge = gurka::findEdgeByNodes(reader, layout, pair.first, pair.second);
    EXPECT_NE(std::get<1>(edge)->superseded(), 0);
  }
  // reverse
  for (const auto& pair : node_pairs{{"F", "E"}, {"L", "K"}}) {
    const auto edge = gurka::findEdgeByNodes(reader, layout, pair.first, pair.second);
    EXPECT_NE(std::get<1>(edge)->superseded(), 0);
  }

  // test that without those restrictions we're still building all shortcuts

  // remove those access restrictions
  ways["CD"] = {{"highway", "motorway"}};
  ways["DE"] = {{"highway", "motorway"}};
  ways["HI"] = {{"highway", "motorway"}};
  ways["IJ"] = {{"highway", "motorway"}};
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
  for (const auto& pair : node_pairs{{"A", "F"}, {"F", "A"}, {"G", "L"}, {"L", "G"}}) {
    const auto shortcut = gurka::findEdgeByNodes(reader2, layout, pair.first, pair.second);
    EXPECT_TRUE(std::get<1>(shortcut)->is_shortcut());
    EXPECT_NEAR(std::get<1>(shortcut)->length(), 7500, 1);
  }
}
