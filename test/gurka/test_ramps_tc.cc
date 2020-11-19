#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class RampsTCs : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
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
    map = gurka::buildtiles(layout, ways, {}, relations, "test/data/gurka_ramps_tc",
                            {{"mjolnir.data_processing.infer_internal_intersections", "false"},
                             {"mjolnir.data_processing.infer_turn_channels", "false"},
                             {"mjolnir.reclassify_links", "false"}});
  }
};
gurka::map RampsTCs::map = {};
Api api;
rapidjson::Document d;

/*************************************************************/

TEST_F(RampsTCs, test_tc_use) {
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
