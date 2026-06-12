#include "baldr/graphconstants.h"
#include "gurka.h"
#include "mjolnir/osmdata.h"
#include "mjolnir/osmway.h"

#include <gtest/gtest.h>

using namespace valhalla;

TEST(ParseWays, OsmWayMarshalling) {
  const std::string ascii_map = R"(A----B----C----D----E----F)";
  const gurka::ways ways = {
      {"AB",
       {{"highway", "residential"},
        {"maxspeed", "120"},
        {"lanes", "4"},
        {"surface", "gravel"},
        {"tunnel", "yes"},
        {"osm_id", "100"}}},
      {"BC", {{"highway", "motorway"}, {"maxspeed", "141"}, {"osm_id", "101"}}},
      {"CD", {{"highway", "motorway"}, {"maxspeed", "254"}, {"osm_id", "102"}}},
      {"DE", {{"highway", "motorway"}, {"maxspeed", "5"}, {"osm_id", "103"}}},
      {"EF", {{"highway", "motorway"}, {"maxspeed", "255"}, {"osm_id", "104"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

  // stop after parsing ways so we can assert on the raw OSMWay structs
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_parse_ways",
                               {{"mjolnir.concurrency", "1"}}, mjolnir::BuildStage::kInitialize,
                               mjolnir::BuildStage::kParseWays);

  auto way = gurka::findWay(map, 100);
  EXPECT_EQ(way.road_class(), baldr::RoadClass::kResidential);
  EXPECT_EQ(way.speed_limit(), 120);
  EXPECT_EQ(way.speed(), 120);
  EXPECT_EQ(way.lanes(), 4);
  EXPECT_EQ(way.surface(), baldr::Surface::kGravel);
  EXPECT_TRUE(way.tunnel());

  // speeds above kMaxOSMSpeed are clamped by OSMWay::set_speed/set_speed_limit
  auto clamped = gurka::findWay(map, 101);
  EXPECT_EQ(clamped.speed_limit(), 140);
  EXPECT_EQ(clamped.speed(), 140);

  auto clamped_high = gurka::findWay(map, 102);
  EXPECT_EQ(clamped_high.speed_limit(), 140);
  EXPECT_EQ(clamped_high.speed(), 140);

  // lua's normalize_speed tosses speeds below 10 kph before they reach C++
  auto tossed = gurka::findWay(map, 103);
  EXPECT_EQ(tossed.speed_limit(), 0);

  // it also tosses speeds >= 255 kph which would collide with the kUnlimitedSpeedLimit sentinel
  auto sentinel = gurka::findWay(map, 104);
  EXPECT_EQ(sentinel.speed_limit(), 0);
  EXPECT_EQ(sentinel.speed(), tossed.speed());
}

TEST(ParseNodes, TwoPhaseWayNodesFill) {
  const std::string ascii_map = R"(
    A----B----C
         |
         D
  )";
  const gurka::ways ways = {
      {"ABC", {{"highway", "residential"}, {"osm_id", "100"}}},
      {"BD", {{"highway", "residential"}, {"osm_id", "101"}}},
  };
  const gurka::nodes nodes = {
      {"A", {{"osm_id", "10"}}},
      {"B", {{"barrier", "gate"}, {"osm_id", "20"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const std::string workdir = "test/data/gurka_parse_way_nodes";

  auto map = gurka::buildtiles(layout, ways, nodes, {}, workdir, {{"mjolnir.concurrency", "1"}},
                               mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kParseWays);

  // after parsing ways the way nodes only carry the node id, the position within the way and
  // whether the node is a way endpoint - the node pass hasn't seen coords or node tags yet
  {
    auto b_nodes = gurka::findWayNodes(map, 20);
    ASSERT_EQ(b_nodes.size(), 2);
    for (const auto& way_node : b_nodes) {
      EXPECT_FALSE(way_node.node.latlng().IsValid());
      EXPECT_NE(way_node.node.type(), baldr::NodeType::kGate);
      // endpoint of BD but interior shape point of ABC
      EXPECT_EQ(way_node.node.intersection_, way_node.way_shape_node_index == 0);
    }
  }

  // run only the node parsing stage on top of the partial build
  gurka::buildtiles(layout, ways, nodes, {}, workdir, {{"mjolnir.concurrency", "1"}},
                    mjolnir::BuildStage::kParseNodes, mjolnir::BuildStage::kParseNodes);

  // now both copies of B have coords, the gate type and are flagged as an intersection since
  // two ways reference the node
  {
    auto b_nodes = gurka::findWayNodes(map, 20);
    ASSERT_EQ(b_nodes.size(), 2);
    for (const auto& way_node : b_nodes) {
      EXPECT_NEAR(way_node.node.latlng().lat(), map.nodes["B"].lat(), 1e-6);
      EXPECT_NEAR(way_node.node.latlng().lng(), map.nodes["B"].lng(), 1e-6);
      EXPECT_EQ(way_node.node.type(), baldr::NodeType::kGate);
      EXPECT_TRUE(way_node.node.intersection());
    }

    // dead ends stay flagged as intersections
    auto a_nodes = gurka::findWayNodes(map, 10);
    ASSERT_EQ(a_nodes.size(), 1);
    EXPECT_TRUE(a_nodes.front().node.intersection());
    EXPECT_TRUE(a_nodes.front().node.latlng().IsValid());
  }
}
