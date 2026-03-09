#include "gurka.h"

#include <gtest/gtest.h>

#include <random>

using namespace valhalla;

class SimpleRestrictions : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {

    const std::string ascii_map = R"(
    A---1B----C
    |    |
    D----E----F
    |
    G----H---2I)";

    // generate random 64bit ids for these ways
    std::mt19937_64 gen(1);
    std::uniform_int_distribution<uint64_t> dist(100, std::numeric_limits<uint64_t>::max());
    std::vector<std::string> ids{
        "121", "122", "123", "124", "6472927700900931484", "125",
    };
    for (int i = 0; i < 6; ++i) {
      ids.push_back(std::to_string(dist(gen)));
    }

    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}, {"osm_id", ids[0]}}},
        {"BC", {{"highway", "primary"}, {"osm_id", ids[1]}}},
        {"DEF", {{"highway", "primary"}, {"osm_id", ids[2]}}},
        {"GHI", {{"highway", "primary"}, {"osm_id", ids[3]}}},
        {"ADG", {{"highway", "primary"}, {"osm_id", ids[4]}}},
        {"BE", {{"highway", "primary"}, {"osm_id", ids[5]}}},
    };

    const gurka::relations relations = {
        {{
             {gurka::way_member, "BC", "from"},
             {gurka::way_member, "BE", "to"},
             {gurka::node_member, "B", "via"},
         },
         {
             {"type", "restriction"},
             {"restriction", "no_left_turn"},
         }},
        {{
             {gurka::way_member, "GHI", "from"},
             {gurka::way_member, "AB", "to"},
             {gurka::way_member, "ADG", "via"},
         },
         {
             {"type", "restriction"},
             {"restriction", "no_entry"},
         }},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map = gurka::buildtiles(layout, ways, {}, relations, "test/data/simple_restrictions",
                            {{"mjolnir.hierarchy", "false"}, {"mjolnir.concurrency", "1"}});
  }
};

gurka::map SimpleRestrictions::map = {};

/*************************************************************/
TEST_F(SimpleRestrictions, ForceDetour) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "F"}, "auto");
  gurka::assert::osrm::expect_steps(result, {"BC", "ADG", "DEF"});
  gurka::assert::raw::expect_path(result, {"BC", "AB", "ADG", "DEF", "DEF"});
}
TEST_F(SimpleRestrictions, NoDetourWhenReversed) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "C"}, "auto");
  gurka::assert::osrm::expect_steps(result, {"DEF", "BE", "BC"});
  gurka::assert::raw::expect_path(result, {"DEF", "BE", "BC"});
}
TEST_F(SimpleRestrictions, NoDetourFromDifferentStart) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"1", "F"}, "auto");
  gurka::assert::osrm::expect_steps(result, {"AB", "BE", "DEF"});
  gurka::assert::raw::expect_path(result, {"AB", "BE", "DEF"});
}
TEST_F(SimpleRestrictions, ForceDetourComplex) {
  // this test fails if you remove the time dependence because
  // bidirectional a* has a problem with complex restrictions
  auto result = gurka::do_action(valhalla::Options::route, map,
                                 R"({"locations":[{"lon":0.008084,"lat":-0.003593},
    {"lon":0.00359,"lat":0.0}],"costing":"auto","date_time":{"type":0}})");
  gurka::assert::raw::expect_path(result, {"GHI", "ADG", "DEF", "BE", "AB"});
}
TEST_F(SimpleRestrictions, IgnoreRestriction) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "F"}, "auto",
                                 {{"/costing_options/auto/ignore_restrictions", "1"}});
  gurka::assert::osrm::expect_steps(result, {"BC", "BE", "DEF"});
  gurka::assert::raw::expect_path(result, {"BC", "BE", "DEF"});
}
TEST_F(SimpleRestrictions, IgnoreRestrictionPedestrian) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "F"}, "pedestrian");
  gurka::assert::osrm::expect_steps(result, {"BC", "BE", "DEF"});
  gurka::assert::raw::expect_path(result, {"BC", "BE", "DEF"});
}

TEST_F(SimpleRestrictions, IgnoreRestrictionMatching) {
  auto result =
      gurka::do_action(valhalla::Options::trace_route, map, {"C", "B", "E", "F"}, "auto",
                       {{"/costing_options/auto/ignore_restrictions", "1"}}, {}, nullptr, "break");
  gurka::assert::osrm::expect_steps(result, {"BC", "BE", "DEF"}, true, "matchings");
  gurka::assert::raw::expect_path(result, {"BC", "BE", "DEF"});
}

// Verify that the restriction builder handles a way split into many edges without
// stack overflow. Before the iterative DFS fix, ExpandFromNode used mutual recursion
// to follow same-way-id chains, causing stack overflow on long ways (200+ edges).
// Uses way-type via to create a complex restriction routed through
// restrictionbuilder.cc's ExpandFromNode (node-type via would create a simple
// restriction handled in graphbuilder.cc, which wouldn't exercise the fix).
TEST(LongWayRestriction, NoStackOverflow) {
  const std::string ascii_map = R"(
    A-B-C-D-E-F-G-H-I-J-K-L-M-N-O-P-Q-R-S-T-U-V-W-X
                                                      |
                                                      Y
                                                      |
    a-b-c-d-e-f-g-h-i-j-k-l-m-n-o-p-q-r-s-t-u-v-w-x
  )";

  const gurka::ways ways = {
      {"ABCDEFGHIJKLMNOPQRSTUVWX", {{"highway", "primary"}}},
      {"XY", {{"highway", "primary"}}},
      {"Yx", {{"highway", "primary"}}},
      {"xwvutsrqponmlkjihgfedcba", {{"highway", "primary"}}},
  };

  // Complex restriction with way-type via: from long way, via XY, no entry to Yx.
  // Forces ExpandFromNode to traverse all 23 edges of ABCDEFGHIJKLMNOPQRSTUVWX.
  const gurka::relations relations = {
      {{
           {gurka::way_member, "ABCDEFGHIJKLMNOPQRSTUVWX", "from"},
           {gurka::way_member, "XY", "via"},
           {gurka::way_member, "Yx", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_entry"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  // The key assertion: buildtiles must complete without stack overflow
  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/long_way_restriction",
                               {{"mjolnir.hierarchy", "false"}, {"mjolnir.concurrency", "1"}});

  // Verify tiles are valid by routing along the long way
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "X"}, "auto");
  EXPECT_EQ(result.trip().routes_size(), 1);
}
