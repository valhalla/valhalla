#include "baldr/graphreader.h"
#include "gurka.h"
#include "mjolnir/osmdata.h"
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

  // the route crosses the square through the generated traversal
  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "E"}, "pedestrian");
  auto names = gurka::detail::get_paths(result).front();
  ASSERT_FALSE(names.empty());
  EXPECT_EQ(names.front(), "entry");
  EXPECT_EQ(names.back(), "exit");
  for (const auto& n : names) {
    EXPECT_NE(n, "top");
    EXPECT_NE(n, "right");
    EXPECT_NE(n, "bottom");
  }
}

TEST(area_routing, area_bit_is_set) {
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

TEST(area_routing, area_member_ways_collected) {
  const std::string ascii_map = R"(
    A----B
    |    |
    D----C
  )";

  // the member way has no routable tags, so lua discards it during ParseWays
  const gurka::ways ways = {
      {"ABCDA", {}},
  };

  // the relation carries the area tags
  const gurka::relations relations = {
      {{{
           {gurka::way_member, "ABCDA", "outer"},
       }},
       {{"type", "multipolygon"}, {"highway", "pedestrian"}, {"area", "yes"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const std::string workdir = "test/data/gurka_area_members";

  // run up to ParseRelations, the member way was discarded by lua
  auto map =
      gurka::buildtiles(layout, ways, {}, relations, workdir,
                        {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}},
                        mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kParseRelations);

  EXPECT_THROW(gurka::findWay(map, "ABCDA"), std::runtime_error);

  // run the new stage on top of the partial build
  gurka::buildtiles(layout, ways, {}, relations, workdir,
                    {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}},
                    mjolnir::BuildStage::kParseAreaWays, mjolnir::BuildStage::kParseAreaWays);

  // now the member way was collected with the area bit set
  auto way = gurka::findWay(map, "ABCDA");
  EXPECT_TRUE(way.area());

  // its geometry was collected too. Coords aren't yet since ParseNodes runs after ParseAreaWays
  auto b_nodes = gurka::findWayNodes(map, "B");
  ASSERT_FALSE(b_nodes.empty());
  EXPECT_FALSE(b_nodes.front().node.latlng().IsValid());
}

TEST(area_routing, area_relations_collected) {
  const std::string ascii_map = R"(
    A----B
    |    |
    D----C
  )";

  // the member way has no routable tags, so lua discards it during ParseWays
  const gurka::ways ways = {
      {"ABCDA", {}},
  };

  // the relation carries the area tags
  const gurka::relations relations = {
      {{{
           {gurka::way_member, "ABCDA", "outer"},
       }},
       {{"type", "multipolygon"}, {"highway", "pedestrian"}, {"area", "yes"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const std::string workdir = "test/data/gurka_area_relations";

  // run up to ParseRelations so area_relations is filled and written to temp files
  auto map =
      gurka::buildtiles(layout, ways, {}, relations, workdir,
                        {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}},
                        mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kParseRelations);

  // load the OSMData from the temp files and check the area relation members
  const auto tile_dir = map.config.get<std::string>("mjolnir.tile_dir");
  mjolnir::OSMData osmdata{};
  osmdata.read_from_temp_files(tile_dir);

  // the area relation collected its member way as an outer
  ASSERT_FALSE(osmdata.area_relations.empty());
  const auto way_id = map.way_osm_ids.at("ABCDA");
  bool found_outer = false;
  for (const auto& [rel_id, member] : osmdata.area_relations) {
    if (member.way_id == way_id && member.is_outer) {
      found_outer = true;
    }
  }
  EXPECT_TRUE(found_outer);
}

TEST(area_routing, relation_routes_through) {
  const std::string ascii_map = R"(
    W---------X------Y---------Z
    |         |      |         |
    A---------B      M---------N
    |         |      |         |
    |  E---F  |      |  Q---R  |
    |  |   |  |      |  |   |  |
    |  G---H  |      |  S---T  |
    |         |      |         |
    D---------C      P---------O
  )";

  const gurka::ways ways = {
      {"ABCDA", {}},
      {"EFHGE", {}},
      {"MNO", {}},
      {"OPM", {}},
      {"QRTSQ", {}},
      {"WX", {{"highway", "footway"}, {"name", "wx"}}},
      {"XY", {{"highway", "footway"}, {"name", "xy"}}},
      {"YZ", {{"highway", "footway"}, {"name", "yz"}}},
      {"WA", {{"highway", "footway"}, {"name", "wa"}}},
      {"XB", {{"highway", "footway"}, {"name", "xb"}}},
      {"YM", {{"highway", "footway"}, {"name", "ym"}}},
      {"ZN", {{"highway", "footway"}, {"name", "zn"}}},
  };

  const gurka::relations relations = {
      {{{
           {gurka::way_member, "ABCDA", "outer"},
           {gurka::way_member, "EFHGE", "inner"},
           {gurka::way_member, "MNO", "outer"},
           {gurka::way_member, "OPM", "outer"},
           {gurka::way_member, "QRTSQ", "inner"},
       }},
       {{"type", "multipolygon"}, {"highway", "pedestrian"}, {"area", "yes"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);

  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/gurka_area_relation_route",
                               {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}});

  const auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));

  // a route from one entrance of the left plaza to the other should cross it
  // through the traversal instead of going around via the top street if it is shorter
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "pedestrian");
  auto names = gurka::detail::get_paths(result).front();
  ASSERT_FALSE(names.empty());
  for (const auto& n : names) {
    EXPECT_NE(n, "wx");
    EXPECT_NE(n, "xy");
    EXPECT_NE(n, "yz");
    EXPECT_NE(n, "wa");
    EXPECT_NE(n, "xb");
  }
}

TEST(area_routing, hole_is_avoided) {
  const std::string ascii_map = R"(
    F------------G
    |            |
    A------------B
    |            |
    |   I----J   |
    |   |    |   |
    |   L----K   |
    |            |
    D------------C
    |            |
    E------------H
  )";

  const gurka::ways ways = {
      {"ABCDA", {}},
      {"IJKLI", {}},
      {"FG", {{"highway", "footway"}, {"name", "top"}}},
      {"EH", {{"highway", "footway"}, {"name", "bottom"}}},
      {"FA", {{"highway", "footway"}, {"name", "entry"}}},
      {"DE", {{"highway", "footway"}, {"name", "exit"}}},
  };

  const gurka::relations relations = {
      {{{
           {gurka::way_member, "ABCDA", "outer"},
           {gurka::way_member, "IJKLI", "inner"},
       }},
       {{"type", "multipolygon"}, {"highway", "pedestrian"}, {"area", "yes"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 20);

  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/gurka_area_hole",
                               {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}});

  // the route crosses the square but the shape must go around the fountain,
  // never through it: no shape point may fall inside the inner ring
  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "E"}, "pedestrian");
  auto shape =
      midgard::decode<std::vector<midgard::PointLL>>(result.trip().routes(0).legs(0).shape());

  const double min_lng = map.nodes.at("L").lng(), max_lng = map.nodes.at("J").lng();
  const double min_lat = map.nodes.at("L").lat(), max_lat = map.nodes.at("J").lat();
  for (const auto& p : shape) {
    const bool inside_hole =
        p.lng() > min_lng && p.lng() < max_lng && p.lat() > min_lat && p.lat() < max_lat;
    EXPECT_FALSE(inside_hole) << "route shape crosses the fountain at " << p.lng() << "," << p.lat();
  }
}

TEST(area_routing, tiny_area_keeps_perimeter) {
  const std::string ascii_map = R"(
    F---G
    A-B |
    | | |
    D-C |
    E---H
  )";

  const gurka::ways ways = {
      {"ABCDA", {{"highway", "pedestrian"}, {"area", "yes"}, {"name", "tiny"}}},
      {"FG", {{"highway", "footway"}, {"name", "top"}}},
      {"GH", {{"highway", "footway"}, {"name", "right"}}},
      {"EH", {{"highway", "footway"}, {"name", "bottom"}}},
      {"FA", {{"highway", "footway"}, {"name", "entry"}}},
      {"CE", {{"highway", "footway"}, {"name", "exit"}}},
  };

  // grid size below the 100 m2 threshold
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 2);

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_area_tiny",
                               {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}});

  const auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));

  // too small for a traversal, so the perimeter must be routable again: the area
  // bit was cleared and the perimeter generates edges between the entrances
  // (A and C are the graph nodes; B and D are just shape points of those edges)
  EXPECT_NO_THROW(gurka::findEdgeByNodes(*reader, map.nodes, "A", "C"));
  // and a route between the entrances uses the perimeter instead of a traversal
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "pedestrian");
  auto names = gurka::detail::get_paths(result).front();
  for (const auto& n : names) {
    EXPECT_EQ(n, "tiny");
  }
}

TEST(area_routing, mapped_paths_inside_skip_generation) {
  const std::string ascii_map = R"(
    F--------G
    |        |
    A----B   |
    | \  |   |
    |  M |   |
    |   \|   |
    D----C   |
         |   |
         E---H
  )";

  const gurka::ways ways = {
      {"ABCDA", {{"highway", "pedestrian"}, {"area", "yes"}, {"name", "square"}}},
      // a footway already mapped across the square, entering at A and leaving at C
      {"AMC", {{"highway", "footway"}, {"name", "shortcut"}}},
      {"FG", {{"highway", "footway"}, {"name", "top"}}},
      {"GH", {{"highway", "footway"}, {"name", "right"}}},
      {"EH", {{"highway", "footway"}, {"name", "bottom"}}},
      {"FA", {{"highway", "footway"}, {"name", "entry"}}},
      {"CE", {{"highway", "footway"}, {"name", "exit"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_area_mapped_paths",
                               {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}});

  // the existing shortcut is used, no traversal was generated on top of it
  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "E"}, "pedestrian");
  auto names = gurka::detail::get_paths(result).front();
  bool used_shortcut = false;
  for (const auto& n : names) {
    if (n == "shortcut") {
      used_shortcut = true;
    }
  }
  EXPECT_TRUE(used_shortcut);
}
