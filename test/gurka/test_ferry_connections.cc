#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

class FerryTest : public ::testing::TestWithParam<std::string> {
protected:
  static gurka::map ferry_map;

public:
  bool edges_were_reclassified(const std::map<std::string, std::string>& custom_tags,
                               const std::string& allowed = "motorcar") {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
          A--B--b--C-----D--E--e
          F--G--g--H-----I--J--j
    )";

    // only allow the passed profile on the roads
    std::map<std::string, std::string> way_props = {{"motorcar", "no"}, {"motorcycle", "no"},
                                                    {"moped", "no"},    {"hgv", "no"},
                                                    {"taxi", "no"},     {"bus", "no"}};
    way_props[allowed] = "yes";
    way_props.insert(custom_tags.begin(), custom_tags.end());

    const gurka::ways ways = {{"AB", {{"highway", "trunk"}}},
                              {"Bb", way_props},
                              {"bC", way_props},
                              {"CD",
                               {{"motorcar", "yes"},
                                {"motorcycle", "yes"},
                                {"moped", "yes"},
                                {"hgv", "yes"},
                                {"taxi", "yes"},
                                {"bus", "yes"},
                                {"route", "ferry"}}},
                              {"DE", way_props},
                              {"FG", {{"highway", "trunk"}}},
                              {"Gg", way_props},
                              {"gH", way_props},
                              {"HI",
                               {{"motorcar", "yes"},
                                {"motorcycle", "yes"},
                                {"moped", "yes"},
                                {"hgv", "yes"},
                                {"taxi", "yes"},
                                {"bus", "yes"},
                                {"route", "shuttle_train"}}},
                              {"IJ", way_props},
                              {"Ee", {{"highway", "trunk"}}},
                              {"Jj", {{"highway", "trunk"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);

    std::cerr << "Model " + allowed << std::endl;

    auto map =
        gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_reclassify_ferry_connections");
    baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));

    std::vector<std::vector<std::string>> node_pairs = {{"B", "b"},
                                                        {"G", "g"},
                                                        {"D", "E"},
                                                        {"I", "J"}};
    for (const auto& node_pair : node_pairs) {
      auto edge =
          std::get<1>(gurka::findEdgeByNodes(graph_reader, layout, node_pair[0], node_pair[1]));
      if (edge->classification() > valhalla::baldr::RoadClass::kPrimary) {
        return false;
      }
    }

    return true;
  }
};

gurka::map FerryTest::ferry_map = {};

TEST(Standalone, ShortFerry) {
  const std::string ascii_map = R"(
          A--B--C--F--D--E--G
                   |
                   N
    )";

  const gurka::ways ways = {{"AB", {{"highway", "trunk"}}},
                            {"BC", {{"highway", "service"}}},
                            {"CF",
                             {{"motor_vehicle", "yes"},
                              {"motorcar", "yes"},
                              {"bicycle", "yes"},
                              {"foot", "no"},
                              {"duration", "35"},
                              {"route", "shuttle_train"},
                              {"name", "Eurotunnel Shuttle"}}},
                            {"FD",
                             {{"motor_vehicle", "yes"},
                              {"motorcar", "yes"},
                              {"bicycle", "yes"},
                              {"foot", "no"},
                              {"duration", "35"},
                              {"route", "shuttle_train"},
                              {"name", "Eurotunnel Shuttle"}}},
                            {"DE", {{"highway", "service"}}},
                            {"EG", {{"highway", "trunk"}}},
                            {"FN", {{"highway", "service"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_reclassify_ferry_connections");
  baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));

  auto BC_edge = std::get<1>(gurka::findEdgeByNodes(graph_reader, layout, "B", "C"));
  auto DE_edge = std::get<1>(gurka::findEdgeByNodes(graph_reader, layout, "D", "E"));

  EXPECT_EQ(BC_edge->classification(), baldr::RoadClass::kPrimary);
  EXPECT_EQ(DE_edge->classification(), baldr::RoadClass::kPrimary);
}

TEST(Standalone, TruckFerryDuration) {
  // corresponds to 21.4 m/sec (~77 km/h) in the below map
  uint32_t ferry_secs = 14;

  const std::string ascii_map = R"(
          A--B--C--D
          |        |
          |        |
          |        |
          |        |
          E--------F
    )";

  const gurka::ways ways = {{"AB", {{"highway", "secondary"}}},
                            {"BC",
                             {{"motor_vehicle", "yes"},
                              {"motorcar", "yes"},
                              {"bicycle", "yes"},
                              {"hgv", "yes"},
                              {"duration", "00:00:0" + std::to_string(ferry_secs)},
                              {"route", "ferry"},
                              {"name", "Random ferry"}}},
                            {"CD", {{"highway", "secondary"}}},
                            {"AEFD", {{"highway", "secondary"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_ferry_duration");

  valhalla::Api fastest = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "truck",
                                           {{"/costing_options/truck/use_ferry", "1"},
                                            {"/costing_options/truck/ferry_cost", "0"}});

  // verify we took the ferry edge and the duration tag was respected
  auto ferry_edge = fastest.trip().routes(0).legs(0).node(1).edge();
  ASSERT_EQ(ferry_edge.use(), valhalla::TripLeg_Use::TripLeg_Use_kFerryUse);
  ASSERT_NEAR(ferry_edge.speed(), ferry_edge.length_km() / (ferry_secs * kHourPerSec), 0.2);
  ASSERT_NEAR(fastest.directions().routes(0).legs(0).summary().time(), 50, 0.1);

  // pass a higher ferry cost and make sure it's added to the time
  valhalla::Api higher_ferry_cost =
      gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "truck",
                       {{"/costing_options/truck/use_ferry", "1"},
                        {"/costing_options/truck/ferry_cost", "10"}});

  ASSERT_NEAR(higher_ferry_cost.directions().routes(0).legs(0).summary().time(), 60, 0.1);
}

TEST_F(FerryTest, DoNotReclassifyFerryConnection) {
  // roads with these values of 'highway' tag do not participate in search for ferry connection so
  // edge class remains low
  const std::vector<std::string> not_reclassifiable_ways = {"track", "living_street"};
  for (const auto& cls : not_reclassifiable_ways) {
    EXPECT_FALSE(edges_were_reclassified({{"highway", cls}}));
  }

  // roads with these values of 'service' tag do not participate in search for ferry connection so
  // edge class remains low
  const std::vector<std::string> not_reclassifiable_use = {"parking_aisle", "driveway", "alley",
                                                           "emergency_access", "drive-through"};
  for (const auto& use : not_reclassifiable_use) {
    std::map<std::string, std::string> desc = {{"highway", "service"}, {"service", use}};
    EXPECT_FALSE(edges_were_reclassified(desc));
  }
}

TEST(Standalone, ReclassifyFerryConnectionRouteModes) {
  const std::string ascii_map = R"(
    A--B--C--D------E--G--H--I
  )";

  std::map<std::string, std::string> no_hgv_car = {{"highway", "secondary"},
                                                   {"moped", "no"},
                                                   {"hgv", "no"}};

  const gurka::ways ways = {{"AB", {{"highway", "trunk"}}},
                            {"BC", no_hgv_car},
                            {"CD", no_hgv_car},
                            {"DE",
                             {{"motor_vehicle", "yes"},
                              {"motorcar", "yes"},
                              {"bicycle", "yes"},
                              {"moped", "yes"},
                              {"bus", "yes"},
                              {"hov", "yes"},
                              {"taxi", "yes"},
                              {"motorcycle", "yes"},
                              {"route", "ferry"}}},
                            {"EG", no_hgv_car},
                            {"GH", no_hgv_car},
                            {"HI", no_hgv_car}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 1000);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_reclassify_ferry_connections");

  // working modes
  for (const auto& mode : {"auto", "motorcycle", "taxi", "bus", "hov"}) {
    gurka::do_action(valhalla::Options::route, map, {"A", "I"}, mode);
  }

  // non-working modes
  for (const auto& mode : {"motor_scooter", "truck"}) {
    try {
      gurka::do_action(Options::route, map, {"A", "I"}, mode);
    } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
      FAIL() << "Expected valhalla_exception_t." << std::endl;
    };
  }
}
TEST_P(FerryTest, ReclassifyFerryConnectionPerMode) {
  // for these values of 'highway' tag edge class is upgraded in order to connect ferry to a
  // high-class road
  const std::vector<std::string> reclassifiable_ways = {"secondary", "unclassified", "service",
                                                        "secondary_link"};
  for (const auto& cls : reclassifiable_ways) {
    EXPECT_TRUE(edges_were_reclassified({{"highway", cls}}, GetParam()));
  }
}

TEST(Standalone, ReclassifyFerryUntagDestOnly) {
  // access=customers should be untagged if it's present on connecting edge(s)
  // see https://github.com/valhalla/valhalla/issues/3941#issuecomment-1407713742
  // CD is destonly -> should untag BC, CD and take the faster path
  // EF is not destonly -> shouldn't untag FG and take the detour
  const std::string ascii_map = R"(
    A--B--C--D------E--F---G--H--M
       |  |            |   |
       I--J            K---L
  )";

  std::map<std::string, std::string> trunk = {{"highway", "trunk"}};
  std::map<std::string, std::string> destonly = {
      {"highway", "residential"},
      {"access", "customers"},
  };
  std::map<std::string, std::string> not_destonly = {{"highway", "residential"}};

  const gurka::ways ways = {
      {"AB", trunk},
      {"BC", destonly}, // destonly should be removed when building the graph
      {"BIJC", not_destonly},
      {"CD", destonly},
      {"DE",
       {{"motor_vehicle", "yes"},
        {"motorcar", "yes"},
        {"bicycle", "yes"},
        {"moped", "yes"},
        {"bus", "yes"},
        {"hov", "yes"},
        {"taxi", "yes"},
        {"motorcycle", "yes"},
        {"route", "ferry"}}},
      {"EF", not_destonly},
      {"FG", destonly}, // should stay destonly and low-class bcs connecting edge isn't and this will
                        // be penalized
      {"FKLG", not_destonly},
      {"GH", not_destonly},
      {"HM", trunk},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_reclassify_destonly");
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  // see if BC was untagged and upclassed
  auto tagged = gurka::findEdge(reader, layout, "BC", "C");
  EXPECT_FALSE(std::get<1>(tagged)->destonly()) << "Edge BC shouldn't be destonly";
  EXPECT_TRUE(std::get<1>(tagged)->classification() == valhalla::baldr::RoadClass::kPrimary);

  // see if FX & XG are still tagged and low class
  auto untagged = gurka::findEdge(reader, layout, "FG", "G");
  EXPECT_TRUE(std::get<1>(untagged)->destonly()) << "Edge FG should be destonly";
  EXPECT_FALSE(std::get<1>(untagged)->classification() == valhalla::baldr::RoadClass::kPrimary);

  // see if FKLG is upclassed
  auto upclassed = gurka::findEdge(reader, layout, "FKLG", "G");
  EXPECT_TRUE(std::get<1>(upclassed)->classification() == valhalla::baldr::RoadClass::kPrimary);

  // we expect to take the shorter route on the left and the detour on the right
  std::vector<std::string> modes = {"auto", "motorcycle", "taxi", "bus", "hov", "truck"};
  for (const auto& mode : modes) {
    auto res = gurka::do_action(valhalla::Options::route, map, {"A", "M"}, mode);
    gurka::assert::raw::expect_path(res, {"AB", "BC", "CD", "DE", "EF", "FKLG", "GH", "HM"},
                                    mode + " failed.");
  }
}

TEST(Standalone, ReclassifyFerryNodePair) {
  // Test to validate that the shortest path between 2 edges between a node
  // pair takes the shorter, higher class road (not the longer, lower class)

  const std::string ascii_map = R"(
    A--B--C--D------E
       |  |
       I--J
  )";

  std::map<std::string, std::string> trunk = {{"highway", "trunk"}};
  std::map<std::string, std::string> secondary = {{"highway", "secondary"}};
  std::map<std::string, std::string> residential = {{"highway", "residential"}};

  const gurka::ways ways = {
      {"AB", trunk},
      {"BC", secondary},
      {"BIJC", residential},
      {"CD", secondary},
      {"DE",
       {{"motor_vehicle", "yes"},
        {"motorcar", "yes"},
        {"bicycle", "yes"},
        {"moped", "yes"},
        {"bus", "yes"},
        {"hov", "yes"},
        {"taxi", "yes"},
        {"motorcycle", "yes"},
        {"route", "ferry"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_reclassify_nodepair");
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  // make sure BC and CD are upclassed
  auto upclassed = gurka::findEdge(reader, layout, "BC", "C");
  EXPECT_TRUE(std::get<1>(upclassed)->classification() == valhalla::baldr::RoadClass::kPrimary);
  auto upclassed2 = gurka::findEdge(reader, layout, "CD", "D");
  EXPECT_TRUE(std::get<1>(upclassed2)->classification() == valhalla::baldr::RoadClass::kPrimary);

  // make sure edge BIJC is not upclassed
  auto not_upclassed = gurka::findEdge(reader, layout, "BIJC", "C");
  EXPECT_TRUE(std::get<1>(not_upclassed)->classification() ==
              valhalla::baldr::RoadClass::kResidential);
}

TEST(Standalone, ReclassifyCorrectPath) {
  // Test to validate that the shortest path between 2 edges between a node
  // pair takes the shorter, higher class road (not the longer, lower class)

  const std::string ascii_map = R"(
    A--Z--B--C--D------E
          |  |
       F--G  J--K
          |  |
       I--H  L--M
  )";

  std::map<std::string, std::string> trunk = {{"highway", "trunk"}};
  std::map<std::string, std::string> secondary = {{"highway", "secondary"}};
  std::map<std::string, std::string> residential = {{"highway", "residential"}};
  std::map<std::string, std::string> driveway = {{"highway", "service"}, {"service", "driveway"}};

  const gurka::ways ways = {
      {"AZ", trunk},
      {"ZB", secondary},
      {"BC", secondary},
      {"CD", secondary},
      {"BG", residential},
      {"GH", residential},
      {"GF", driveway},
      {"HI", driveway},
      {"CJ", residential},
      {"JL", residential},
      {"JK", driveway},
      {"LM", driveway},
      {"DE",
       {{"motor_vehicle", "yes"},
        {"motorcar", "yes"},
        {"bicycle", "yes"},
        {"moped", "yes"},
        {"bus", "yes"},
        {"hov", "yes"},
        {"taxi", "yes"},
        {"motorcycle", "yes"},
        {"route", "ferry"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_reclassify_correct_path");
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  // make sure ZB, BC and CD are upclassed
  auto upclassed1 = gurka::findEdge(reader, layout, "ZB", "B");
  EXPECT_TRUE(std::get<1>(upclassed1)->classification() == valhalla::baldr::RoadClass::kPrimary);
  auto upclassed2 = gurka::findEdge(reader, layout, "BC", "C");
  EXPECT_TRUE(std::get<1>(upclassed2)->classification() == valhalla::baldr::RoadClass::kPrimary);
  auto upclassed3 = gurka::findEdge(reader, layout, "CD", "D");
  EXPECT_TRUE(std::get<1>(upclassed3)->classification() == valhalla::baldr::RoadClass::kPrimary);

  // make sure other edges are not upclassed
  auto not_upclassed1 = gurka::findEdge(reader, layout, "BG", "G");
  EXPECT_TRUE(std::get<1>(not_upclassed1)->classification() ==
              valhalla::baldr::RoadClass::kResidential);
  auto not_upclassed2 = gurka::findEdge(reader, layout, "GH", "H");
  EXPECT_TRUE(std::get<1>(not_upclassed2)->classification() ==
              valhalla::baldr::RoadClass::kResidential);
  auto not_upclassed3 = gurka::findEdge(reader, layout, "CJ", "J");
  EXPECT_TRUE(std::get<1>(not_upclassed3)->classification() ==
              valhalla::baldr::RoadClass::kResidential);
  auto not_upclassed4 = gurka::findEdge(reader, layout, "JL", "L");
  EXPECT_TRUE(std::get<1>(not_upclassed4)->classification() ==
              valhalla::baldr::RoadClass::kResidential);
  auto not_upclassed5 = gurka::findEdge(reader, layout, "GF", "F");
  EXPECT_TRUE(std::get<1>(not_upclassed5)->classification() ==
              valhalla::baldr::RoadClass::kServiceOther);
  auto not_upclassed6 = gurka::findEdge(reader, layout, "HI", "I");
  EXPECT_TRUE(std::get<1>(not_upclassed6)->classification() ==
              valhalla::baldr::RoadClass::kServiceOther);
  auto not_upclassed7 = gurka::findEdge(reader, layout, "JK", "K");
  EXPECT_TRUE(std::get<1>(not_upclassed7)->classification() ==
              valhalla::baldr::RoadClass::kServiceOther);
  auto not_upclassed8 = gurka::findEdge(reader, layout, "LM", "M");
  EXPECT_TRUE(std::get<1>(not_upclassed8)->classification() ==
              valhalla::baldr::RoadClass::kServiceOther);
}

TEST(Standalone, ReclassifySeparateInboundAndOutbound) {
  // Sometimes ferry path is connected to the separate inbound and outbound pathways.
  // Both of them should be reclassified if both of them lead to the high road class.

  const std::string ascii_map = R"(
            A
            |
            |
            B
           / \
          /   \
         C     D
        /      |
    E--F---------------G
    H-----I------------J
           \  /
            --
  )";

  std::map<std::string, std::string> primary = {{"highway", "primary"}, {"oneway", "yes"}};
  std::map<std::string, std::string> service = {{"highway", "service"}, {"oneway", "yes"}};

  const gurka::ways ways = {
      {"AB",
       {{"motor_vehicle", "yes"},
        {"motorcar", "yes"},
        {"bicycle", "yes"},
        {"moped", "yes"},
        {"bus", "yes"},
        {"hov", "yes"},
        {"taxi", "yes"},
        {"motorcycle", "yes"},
        {"route", "ferry"}}},
      {"BCF", service},
      {"IDB", service},
      {"EFG", primary},
      {"JIH", primary},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_reclassify_inbound_outbound");
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  // make sure that both service roads get reclassified
  auto outbound_service = gurka::findEdge(reader, layout, "BCF", "F");
  EXPECT_EQ(std::get<1>(outbound_service)->classification(), valhalla::baldr::RoadClass::kPrimary);
  auto inbound_service = gurka::findEdge(reader, layout, "IDB", "B");
  EXPECT_EQ(std::get<1>(inbound_service)->classification(), valhalla::baldr::RoadClass::kPrimary);
}

TEST(Standalone, ReclassifyInboundOnly) {
  const std::string ascii_map = R"(
            A
            |
            |
            B
           /
          /
         C
        /
    E--F---------------G
    H-----I------------J
  )";

  std::map<std::string, std::string> primary = {{"highway", "primary"}, {"oneway", "yes"}};
  std::map<std::string, std::string> service = {{"highway", "service"}, {"oneway", "yes"}};

  const gurka::ways ways = {
      {"AB",
       {{"motor_vehicle", "yes"},
        {"motorcar", "yes"},
        {"bicycle", "yes"},
        {"moped", "yes"},
        {"bus", "yes"},
        {"hov", "yes"},
        {"taxi", "yes"},
        {"motorcycle", "yes"},
        {"route", "ferry"}}},
      {"BCF", service},
      {"EFG", primary},
      {"JIH", primary},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_reclassify_outbound_only");
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  auto outbound_service = gurka::findEdge(reader, layout, "BCF", "F");
  EXPECT_EQ(std::get<1>(outbound_service)->classification(), valhalla::baldr::RoadClass::kPrimary);
}

TEST(Standalone, ReclassifyOutboundOnly) {
  const std::string ascii_map = R"(
            A
            |
            |
            B
             \
              \
               D
               |
    E--F---------------G
    H-----I------------J
           \  /
            --
  )";

  std::map<std::string, std::string> primary = {{"highway", "primary"}, {"oneway", "yes"}};
  std::map<std::string, std::string> service = {{"highway", "service"}, {"oneway", "yes"}};

  const gurka::ways ways = {
      {"AB",
       {{"motor_vehicle", "yes"},
        {"motorcar", "yes"},
        {"bicycle", "yes"},
        {"moped", "yes"},
        {"bus", "yes"},
        {"hov", "yes"},
        {"taxi", "yes"},
        {"motorcycle", "yes"},
        {"route", "ferry"}}},
      {"IDB", service},
      {"EFG", primary},
      {"JIH", primary},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_reclassify_inbound_only");
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  auto inbound_service = gurka::findEdge(reader, layout, "IDB", "B");
  EXPECT_EQ(std::get<1>(inbound_service)->classification(), valhalla::baldr::RoadClass::kPrimary);
}

TEST(Standalone, ConsiderBlockedRoads) {
  // Nodes that block the path should be considered when the fastest way from ferry to high class road
  // is evaluated, as otherwise found shortest path might be not traversable

  const std::string ascii_map = R"(
    A-------B   M
            |   |
            |   |
        D---C-1-N
       /        |
      E         |
       \        |
        F-------O
                |
                P

  )";

  const gurka::ways ways = {
      {"AB",
       {{"motor_vehicle", "yes"},
        {"motorcar", "yes"},
        {"bicycle", "yes"},
        {"moped", "yes"},
        {"bus", "yes"},
        {"hov", "yes"},
        {"taxi", "yes"},
        {"motorcycle", "yes"},
        {"route", "ferry"}}},

      // shorter road that is blocked by block
      {"C1N", {{"highway", "service"}}},

      // longer unblocked road
      {"BC", {{"highway", "service"}}},
      {"CD", {{"highway", "service"}}},
      {"DE", {{"highway", "service"}}},
      {"EF", {{"highway", "service"}}},
      {"FO", {{"highway", "service"}}},

      {"MNOP", {{"highway", "primary"}}},
  };

  const gurka::nodes nodes = {
      {"1",
       {
           {"barrier", "block"},
           {"motor_vehicle", "no"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500);
  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_reclassify_block");

  // sanity check that the longer route is taken
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "M"}, "auto");
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FO", "MNOP", "MNOP"});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  // long not blocked road should be reclassified
  for (const auto& name : {"BC", "CD", "DE", "EF", "FO"}) {
    const auto way = gurka::findEdge(reader, layout, name, name + 1);
    EXPECT_EQ(std::get<1>(way)->classification(), valhalla::baldr::RoadClass::kPrimary);
  }

  // short blocked route is not reclassified
  auto blocked = gurka::findEdge(reader, layout, "C1N", "N");
  EXPECT_EQ(std::get<1>(blocked)->classification(), valhalla::baldr::RoadClass::kServiceOther);
}

TEST(Standalone, ReclassifyNothingReclassified) {
  // Test to validate that if no edges are found with the target classification
  // nothing gets reclassified.

  const std::string ascii_map = R"(
    A--B--C--D------E
  )";

  std::map<std::string, std::string> secondary = {{"highway", "secondary"}};
  std::map<std::string, std::string> tertiary = {{"highway", "tertiary"}};

  const gurka::ways ways = {
      {"AB", secondary},
      {"BC", secondary},
      {"CD", tertiary},
      {"DE",
       {{"motor_vehicle", "yes"},
        {"motorcar", "yes"},
        {"bicycle", "yes"},
        {"moped", "yes"},
        {"bus", "yes"},
        {"hov", "yes"},
        {"taxi", "yes"},
        {"motorcycle", "yes"},
        {"route", "ferry"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_reclassify_nothing");
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  // make sure no edges are upclassed
  auto not_upclassed1 = gurka::findEdge(reader, layout, "AB", "B");
  EXPECT_TRUE(std::get<1>(not_upclassed1)->classification() ==
              valhalla::baldr::RoadClass::kSecondary);
  auto not_upclassed2 = gurka::findEdge(reader, layout, "BC", "C");
  EXPECT_TRUE(std::get<1>(not_upclassed2)->classification() ==
              valhalla::baldr::RoadClass::kSecondary);
  auto not_upclassed3 = gurka::findEdge(reader, layout, "CD", "D");
  EXPECT_TRUE(std::get<1>(not_upclassed3)->classification() == valhalla::baldr::RoadClass::kTertiary);
}

class ExcludeFerryTest : public ::testing::TestWithParam<std::string> {
protected:
  static gurka::map map;
  static void SetUpTestSuite() {

    const std::string ascii_map = R"(
    A----1---B----C--D------E
  )";

    const gurka::ways ways = {
        {"AB", {{"highway", "secondary"}}},
        {"BC", {{"route", "ferry"}}},
        {"CD", {{"highway", "secondary"}}},
        {"DE", {{"highway", "secondary"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500);

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/exclude_ferry");
    baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));
  }
};

gurka::map ExcludeFerryTest::map = {};

TEST_P(ExcludeFerryTest, ExcludeFerry) {
  const auto default_result = gurka::do_action(valhalla::Options::route, map, {"1", "D"}, GetParam());
  gurka::assert::raw::expect_path(default_result, {"AB", "BC", "CD"});

  try {
    const auto result =
        gurka::do_action(valhalla::Options::route, map, {"1", "D"}, GetParam(),
                         {{"/costing_options/" + GetParam() + "/exclude_ferries", "1"}});
    FAIL() << "Expected no path to be found";
  } catch (valhalla_exception_t& e) { EXPECT_EQ(e.code, 442); } catch (...) {
    FAIL() << "Failed with unexpected error code";
  }
}
INSTANTIATE_TEST_SUITE_P(ExcludeFerry,
                         ExcludeFerryTest,
                         ::testing::Values("auto",
                                           "truck",
                                           "motor_scooter",
                                           "pedestrian",
                                           "bicycle",
                                           "motorcycle",
                                           "taxi",
                                           "bus"));

INSTANTIATE_TEST_SUITE_P(FerryConnectionTest,
                         FerryTest,
                         ::testing::Values("motorcar", "hgv", "moped", "motorcycle", "taxi", "bus"));