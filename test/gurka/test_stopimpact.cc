#include "gurka.h"

#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;

class StopImpact : public ::testing::Test {
protected:
  // Helper: get stopimpact for the transition through a node.
  static uint32_t GetStopImpact(GraphReader& reader,
                                const nodelayout& nodes,
                                const std::string& from_node,
                                const std::string& via_node,
                                const std::string& to_node) {

    auto [inbound_id, inbound_edge] = findEdgeByNodes(reader, nodes, from_node, via_node);
    auto [outbound_id, outbound_edge] = findEdgeByNodes(reader, nodes, via_node, to_node);

    return outbound_edge->stopimpact(inbound_edge->opp_local_idx());
  }
};

TEST_F(StopImpact, MotorwayWayChangeNoIntersection) {
  const std::string ascii_map = R"(
      A-----------B-----------C
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}, {"oneway", "yes"}}},
      {"BC", {{"highway", "motorway"}, {"oneway", "yes"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_stopimpact_motorway");

  GraphReader reader(map.config.get_child("mjolnir"));
  EXPECT_EQ(GetStopImpact(reader, map.nodes, "A", "B", "C"), 0);
}

TEST_F(StopImpact, ResidentialWayChangeNoIntersection) {
  const std::string ascii_map = R"(
      A-----------B-----------C
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_stopimpact_residential");

  GraphReader reader(map.config.get_child("mjolnir"));

  EXPECT_EQ(GetStopImpact(reader, map.nodes, "A", "B", "C"), 0);
}

TEST_F(StopImpact, RealIntersection) {
  const std::string ascii_map = R"(
               D        F
               |        |
      A--------B--------C
               |        |
               E        G
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"BD", {{"highway", "cycleway"}}},
      {"BE", {{"highway", "cycleway"}}},
      {"CF", {{"highway", "residential"}}},
      {"CG", {{"highway", "residential"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_stopimpact_intersection");

  GraphReader reader(map.config.get_child("mjolnir"));

  EXPECT_EQ(GetStopImpact(reader, map.nodes, "A", "B", "C"), 0);
  // Some positive stopimpact expected
  EXPECT_GT(GetStopImpact(reader, map.nodes, "D", "B", "E"), 0);
  EXPECT_GT(GetStopImpact(reader, map.nodes, "F", "C", "G"), 0);
}

TEST_F(StopImpact, RealIntersectionSameRoadClass) {
  const std::string ascii_map = R"(
               D
               |
      A--------B--------C
               |
               E
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},
      {"BD", {{"highway", "residential"}}},
      {"BE", {{"highway", "residential"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_stopimpact_intersection_same_roadclass");

  GraphReader reader(map.config.get_child("mjolnir"));

  EXPECT_GT(GetStopImpact(reader, map.nodes, "A", "B", "C"), 0);
  EXPECT_GT(GetStopImpact(reader, map.nodes, "D", "B", "E"), 0);
}

TEST_F(StopImpact, OneWayCrossStreet) {
  const std::string ascii_map = R"(
      A-----------B-----------C
                  |
                  D
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},
      {"BD", {{"highway", "residential"}, {"oneway", "yes"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_stopimpact_oneway_cross");

  GraphReader reader(map.config.get_child("mjolnir"));

  // It isn't clear that this _should_ be true - in fact A-B-C could probably have 0 stopimpact;
  // this test is to verify existing behavior hasn't changed.
  EXPECT_GT(GetStopImpact(reader, map.nodes, "A", "B", "C"), 0);
}
