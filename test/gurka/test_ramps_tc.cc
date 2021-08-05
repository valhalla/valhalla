#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
TEST(RampsTCs, test_tc_no_infer) {

  constexpr double gridsize_metres = 100;

  const std::string ascii_map = R"(
            E D
            | |
            Q P
           /| |\
          / | | \
      L--R--F-C--O--K
            | |
      I-----G-B--N--J
            | | /
            | |/
            | M
            | |
            H A
)";

  const gurka::ways ways = {{"AM",
                             {{"highway", "primary"},
                              {"oneway", "yes"},
                              {"lanes", "5"},
                              {"turn:lanes", "through|through|right"}}},
                            {"MB",
                             {{"highway", "primary"},
                              {"oneway", "yes"},
                              {"lanes", "5"},
                              {"internal_intersection", "true"},
                              {"turn:lanes", "left|through|through"}}},
                            {"MN",
                             {{"highway", "primary_link"},
                              {"oneway", "yes"},
                              {"internal_intersection", "true"},
                              {"turn_channel", "true"}}},
                            {"OP",
                             {{"highway", "primary"},
                              {"oneway", "yes"},
                              {"internal_intersection", "true"},
                              {"turn_channel", "true"}}},
                            {"QR",
                             {{"highway", "primary_link"},
                              {"oneway", "yes"},
                              {"internal_intersection", "false"},
                              {"turn_channel", "false"}}},
                            {"EQ",
                             {{"highway", "primary"},
                              {"oneway", "yes"},
                              {"lanes", "4"},
                              {"turn:lanes", "reverse|left|through|right"}}},
                            {"QF",
                             {{"highway", "primary"},
                              {"oneway", "yes"},
                              {"lanes", "4"},
                              {"turn:lanes", "reverse|left|through|right"}}},
                            {"BC",
                             {
                                 {"highway", "primary"},
                                 {"oneway", "yes"},
                                 {"internal_intersection", "true"},
                             }},
                            {"CP",
                             {
                                 {"highway", "primary"},
                                 {"oneway", "yes"},
                             }},
                            {"PD",
                             {
                                 {"highway", "primary"},
                                 {"oneway", "yes"},
                             }},
                            {"FG",
                             {
                                 {"highway", "primary"},
                                 {"oneway", "yes"},
                                 {"internal_intersection", "true"},
                             }},
                            {"GH",
                             {
                                 {"highway", "primary"},
                                 {"oneway", "yes"},
                             }},
                            {"IG",
                             {
                                 {"highway", "primary"},
                                 {"oneway", "yes"},
                             }},
                            {"GB",
                             {
                                 {"highway", "primary"},
                                 {"oneway", "yes"},
                                 {"internal_intersection", "true"},
                             }},
                            {"BN",
                             {
                                 {"highway", "primary"},
                                 {"oneway", "yes"},
                                 {"internal_intersection", "true"},
                             }},
                            {"NJ",
                             {
                                 {"highway", "primary"},
                                 {"oneway", "yes"},
                             }},
                            {"KO",
                             {
                                 {"highway", "primary"},
                                 {"oneway", "yes"},
                             }},
                            {"OC",
                             {
                                 {"highway", "primary"},
                                 {"oneway", "yes"},
                             }},
                            {"CF",
                             {
                                 {"highway", "primary"},
                                 {"oneway", "yes"},
                                 {"internal_intersection", "true"},
                             }},
                            {"FR",
                             {
                                 {"highway", "primary"},
                                 {"oneway", "yes"},
                             }},
                            {"RL",
                             {
                                 {"highway", "primary"},
                                 {"oneway", "yes"},
                             }}};

  const gurka::relations relations = {
      {{
           {gurka::way_member, "MB", "from"},
           {gurka::way_member, "BN", "to"},
           {gurka::node_member, "B", "via"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_right_turn"},
       }},
      {{
           {gurka::way_member, "BC", "from"},
           {gurka::way_member, "CF", "to"},
           {gurka::node_member, "C", "via"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_left_turn"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/gurka_ramps_tc_no_infer",
                               {{"mjolnir.data_processing.infer_internal_intersections", "false"},
                                {"mjolnir.data_processing.infer_turn_channels", "false"},
                                {"mjolnir.reclassify_links", "false"}});

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, "auto");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AM", "MN", "NJ"});
  EXPECT_EQ(leg.node(1).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kRampUse);
  EXPECT_EQ(leg.node(1).edge().internal_intersection(), 0);

  result = gurka::do_action(valhalla::Options::route, map, {"K", "D"}, "auto");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"KO", "OP", "PD"});
  EXPECT_EQ(leg.node(1).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kTurnChannelUse);
  EXPECT_EQ(leg.node(1).edge().internal_intersection(), 0);

  result = gurka::do_action(valhalla::Options::route, map, {"E", "L"}, "auto");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"EQ", "QR", "RL"});
  EXPECT_EQ(leg.node(1).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kRampUse);
  EXPECT_EQ(leg.node(1).edge().internal_intersection(), 0);

  result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AM", "MB", "BC", "CP", "PD"});
  EXPECT_EQ(leg.node(1).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kRoadUse);
  EXPECT_EQ(leg.node(1).edge().internal_intersection(), 1);
}

TEST(RampsTCs, test_tc_infer) {

  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(

      A---B-------------------------------C------D
           \                             /
            \            F J            /
             \           | |           /
              \          | |          /
               \         | |         /
                \        | |        /
                 --E-----G-K-----N--
                    \    | |    /
                     \   | |   /
                      \  | |  /
                       \ | | /
                         H L
                         | |
                         | |
                         | |
                         I M


)";

  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}, {"oneway", "yes"}}},
      {"BC", {{"highway", "motorway"}, {"oneway", "yes"}}},
      {"CD", {{"highway", "motorway"}, {"oneway", "yes"}}},

      {"BE", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"EG", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"EH", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"GK", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"FG", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"GH", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"HI", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"ML", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"LK", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"KJ", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"KN", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"LN", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"NC", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
  };

  const gurka::nodes nodes = {{"B", {{"highway", "motorway_junction"}, {"ref", "4"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_ramps_tc_infer",
                               {{"mjolnir.data_processing.infer_internal_intersections", "true"},
                                {"mjolnir.data_processing.infer_turn_channels", "true"},
                                {"mjolnir.reclassify_links", "true"}});

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, "auto");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BE", "EG", "GK", "KJ"});

  EXPECT_EQ(leg.node(0).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kRoadUse);
  EXPECT_EQ(leg.node(0).edge().internal_intersection(), 0);

  EXPECT_EQ(leg.node(1).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kRampUse);
  EXPECT_EQ(leg.node(1).edge().internal_intersection(), 0);

  EXPECT_EQ(leg.node(2).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kRampUse);
  EXPECT_EQ(leg.node(2).edge().internal_intersection(), 0);

  EXPECT_EQ(leg.node(3).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kTurnChannelUse);
  EXPECT_EQ(leg.node(3).edge().internal_intersection(), 1);

  EXPECT_EQ(leg.node(4).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kRoadUse);
  EXPECT_EQ(leg.node(4).edge().internal_intersection(), 0);

  gurka::assert::raw::expect_maneuver_begin_path_indexes(result, {0, 1, 4, 5});

  int maneuver_index = 2;

  // Verify that we are marking the internal edge.  If not, there will be a continue instruction
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn left onto KJ.",
      "Turn left. Then You will arrive at your destination.", "Turn left onto KJ.",
      "Turn left onto KJ. Then You will arrive at your destination.", "Continue for 50 meters.");
}

namespace {
void check_edge_classification(baldr::GraphReader& graph_reader,
                               const gurka::nodelayout& nodes,
                               const std::string& b,
                               const std::string& e,
                               baldr::RoadClass rc) {
  const auto edge = std::get<1>(gurka::findEdgeByNodes(graph_reader, nodes, b, e));
  EXPECT_EQ(edge->classification(), rc);
}
} // namespace

TEST(LinkReclassification, test_use_refs) {
  // Check that linkreclassification algorithm takes into account 'ref' tag
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
  A------------B-------------C-----------D
                \     G     /
                 \    |    K----------L
     I------------J   |   /
                   \  |  /
                    \ | /
                     \|/
                      F
                      |
                      |
                      E
  )";

  const gurka::ways ways = {
      // motorway road M1
      {"ABCD", {{"highway", "motorway"}, {"ref", "M1"}}},
      // trunk road M2
      {"EFG", {{"highway", "trunk"}, {"ref", "M2"}}},
      // link from M1 to M2
      {"BJF", {{"highway", "motorway_link"}, {"ref", "M2"}, {"oneway", "yes"}}},
      // some tertiary road
      {"IJ", {{"highway", "tertiary"}}},
      // link from M2 to M1
      {"FKC", {{"highway", "motorway_link"}, {"ref", "M2"}, {"oneway", "yes"}}},
      // some tertiary road
      {"KL", {{"highway", "tertiary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_ramps_use_refs",
                               {{"mjolnir.reclassify_links", "true"}});
  baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));

  // check that ramp classification from M1 to M2 wasn't confused by the tertiary road
  check_edge_classification(graph_reader, layout, "B", "J", baldr::RoadClass::kTrunk);
  check_edge_classification(graph_reader, layout, "J", "F", baldr::RoadClass::kTrunk);

  // check that ramp classification from M2 to M1 wasn't confused by the tertiary road
  check_edge_classification(graph_reader, layout, "F", "K", baldr::RoadClass::kTrunk);
  check_edge_classification(graph_reader, layout, "K", "C", baldr::RoadClass::kTrunk);
}

TEST(LinkReclassification, test_use_dest_refs) {
  // Check that linkreclassification algorithm takes into account 'destination:ref' tag
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
  A------------B-------------C-----------D
                \     G     /
                 \    |    K----------L
     I------------J   |   /
                   \  |  /
                    \ | /
                     \|/
                      F
                      |
                      |
                      E
  )";

  const gurka::ways ways = {
      // motorway road M1
      {"ABCD", {{"highway", "motorway"}, {"ref", "M1"}}},
      // trunk road M2
      {"EFG", {{"highway", "trunk"}, {"ref", "M2"}}},
      // link from M1 to M2
      {"BJF", {{"highway", "motorway_link"}, {"destination:ref", "M2"}, {"oneway", "yes"}}},
      // some tertiary road
      {"IJ", {{"highway", "tertiary"}}},
      // link from M2 to M1
      {"FKC", {{"highway", "motorway_link"}, {"destination:ref", "M1"}, {"oneway", "yes"}}},
      // some tertiary road
      {"KL", {{"highway", "tertiary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_ramps_use_dest_refs",
                               {{"mjolnir.reclassify_links", "true"}});
  baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));

  // check that ramp classification from M1 to M2 wasn't confused by the tertiary road
  check_edge_classification(graph_reader, layout, "B", "J", baldr::RoadClass::kTrunk);
  check_edge_classification(graph_reader, layout, "J", "F", baldr::RoadClass::kTrunk);

  // check that ramp classification from M2 to M1 wasn't confused by the tertiary road
  check_edge_classification(graph_reader, layout, "F", "K", baldr::RoadClass::kTrunk);
  check_edge_classification(graph_reader, layout, "K", "C", baldr::RoadClass::kTrunk);
}

TEST(LinkReclassification, test_hierarchical_reclass) {
  // Check that final road classes of links will be higher or equal to the classes of their children
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
  A--------B-------------------------------------C
            \                             D
             \                            |
              G-------------H-------------E
              |             |             |
          I---J---K         |             F
                        L---M---N
  )";

  const gurka::ways ways = {
      {"ABC", {{"highway", "motorway"}}},
      {"DEF", {{"highway", "trunk"}}},
      {"BGHE", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"GJ", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"HM", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"IJK", {{"highway", "secondary"}}},
      {"LMN", {{"highway", "tertiary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_ramps_hierarchical_reclass",
                               {{"mjolnir.reclassify_links", "true"}});
  baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));

  // BG leads to 3 roads, make sure that the best classification was chosen
  check_edge_classification(graph_reader, layout, "B", "G", baldr::RoadClass::kTrunk);
  // GJ leads to only one secondary road
  check_edge_classification(graph_reader, layout, "G", "J", baldr::RoadClass::kSecondary);
  // GH lead to 2 road, make sure that the best classification was chosen
  check_edge_classification(graph_reader, layout, "G", "H", baldr::RoadClass::kTrunk);
  // HM leads to only one tertiary road
  check_edge_classification(graph_reader, layout, "H", "M", baldr::RoadClass::kTertiary);
  // HE leads to only one trunk road
  check_edge_classification(graph_reader, layout, "H", "E", baldr::RoadClass::kTrunk);
}

TEST(LinkReclassification, test_acyclic_graph) {
  // Check that all links will be reclassified correctly in case when some nodes have
  // several inbound links and some nodes have several outbound links (when link graph
  // is an acyclic graph that is not a tree)
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
                                         A------B-----C
                                                |         D
                   G                M           |         |
                   |               / \          J---------E
                   |              /   \        /          |
                   H-----N-------L-----K-------           F
                   |     |
                   |   O-P-R
                   I
  )";

  const gurka::ways ways = {
      {"GHI", {{"highway", "motorway"}}},
      {"ABC", {{"highway", "primary"}}},
      {"DEF", {{"highway", "secondary"}}},
      {"OPR", {{"highway", "tertiary"}}},

      {"BJ", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"EJ", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"JK", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"KL", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"KM", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"ML", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"LN", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"NH", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
      {"NP", {{"highway", "motorway_link"}, {"oneway", "yes"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_ramps_acyclic_graph",
                               {{"mjolnir.reclassify_links", "true"}});
  baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));

  check_edge_classification(graph_reader, layout, "B", "J", baldr::RoadClass::kPrimary);
  check_edge_classification(graph_reader, layout, "J", "K", baldr::RoadClass::kPrimary);
  check_edge_classification(graph_reader, layout, "K", "L", baldr::RoadClass::kPrimary);
  check_edge_classification(graph_reader, layout, "K", "M", baldr::RoadClass::kPrimary);
  check_edge_classification(graph_reader, layout, "M", "L", baldr::RoadClass::kPrimary);
  check_edge_classification(graph_reader, layout, "L", "N", baldr::RoadClass::kPrimary);
  check_edge_classification(graph_reader, layout, "N", "H", baldr::RoadClass::kPrimary);
  check_edge_classification(graph_reader, layout, "N", "P", baldr::RoadClass::kTertiary);
  check_edge_classification(graph_reader, layout, "E", "J", baldr::RoadClass::kSecondary);
}

TEST(RampsTCs, SlipLane) {
  // slip lane logic applies if link is long enough(> 200m at the moment)
  // so gridsize should be super small
  constexpr double gridsize_metres = 100;
  const std::string ascii_map = R"(
                Z
                |
  K__M_____W____C____D___E____N
      \         |        /
        \__Y    |      ^
            \   B     /
            |   |   F
            |   | /
             \  A
              \ |
                X
    )";

  const gurka::ways ways = {{"XABCZ", {{"highway", "primary"}, {"name", "M1"}}},
                            {"KMWCDEN", {{"highway", "primary"}, {"name", "M2"}}},
                            {"AFE", {{"highway", "primary_link"}, {"oneway", "yes"}}},
                            {"XYM", {{"highway", "primary_link"}, {"oneway", "yes"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);

  std::string test_dir = "test/data/gurka_ramp_vs_turn_channel_slip_lane";
  auto map = gurka::buildtiles(layout, ways, {}, {}, test_dir);

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  {
    auto edge = gurka::findEdgeByNodes(reader, layout, "A", "E");
    const baldr::DirectedEdge* directed_edge = std::get<1>(edge);
    EXPECT_EQ(directed_edge->use(), valhalla::baldr::Use::kTurnChannel);
  }
  {
    auto edge = gurka::findEdgeByNodes(reader, layout, "X", "M");
    const baldr::DirectedEdge* directed_edge = std::get<1>(edge);
    EXPECT_EQ(directed_edge->use(), valhalla::baldr::Use::kTurnChannel);
  }
}
