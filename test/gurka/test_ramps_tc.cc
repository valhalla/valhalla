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

  auto result = gurka::route(map, "A", "J", "auto");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AM", "MN", "NJ"});
  EXPECT_EQ(leg.node(1).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kRampUse);
  EXPECT_EQ(leg.node(1).edge().internal_intersection(), 0);

  result = gurka::route(map, "K", "D", "auto");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"KO", "OP", "PD"});
  EXPECT_EQ(leg.node(1).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kTurnChannelUse);
  EXPECT_EQ(leg.node(1).edge().internal_intersection(), 0);

  result = gurka::route(map, "E", "L", "auto");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"EQ", "QR", "RL"});
  EXPECT_EQ(leg.node(1).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kRampUse);
  EXPECT_EQ(leg.node(1).edge().internal_intersection(), 0);

  result = gurka::route(map, "A", "D", "auto");
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

  auto result = gurka::route(map, "A", "J", "auto");
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
      result, maneuver_index, "Turn left onto KJ.", "Turn left onto KJ.",
      "Turn left onto KJ. Then You will arrive at your destination.", "Continue for 50 meters.");
}
