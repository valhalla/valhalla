#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

std::pair<baldr::RoadClass, baldr::RoadClass>
ReclassifyFerryConnectionEdge(std::map<std::string, std::string> const& way_description) {
  constexpr double gridsize_metres = 1000;

  const std::string ascii_map = R"(
          A--B--b--C----D--E
          F--G--g--H----I--J
    )";

  const gurka::ways ways = {{"AB", {{"highway", "trunk"}, {"name", ""}}},
                            {"Bb", way_description},
                            {"bC", way_description},
                            {"CD",
                             {{"motor_vehicle", "yes"},
                              {"motorcar", "yes"},
                              {"bicycle", "yes"},
                              {"foot", "yes"},
                              {"horse", "no"},
                              {"duration", "01:25"},
                              {"route", "ferry"},
                              {"operator", "Cape May-Lewes Ferry"},
                              {"name", "Cape May-Lewes Ferry"},
                              {"destination:forward", "Cape May"},
                              {"destination:backward", "Lewes"}}},
                            {"DE", {{"highway", "trunk"}, {"name", ""}}},
                            {"FG", {{"highway", "trunk"}, {"name", ""}}},
                            {"Gg", way_description},
                            {"gH", way_description},
                            {"HI",
                             {{"motor_vehicle", "yes"},
                              {"motorcar", "yes"},
                              {"bicycle", "yes"},
                              {"foot", "no"},
                              {"duration", "35"},
                              {"route", "shuttle_train"},
                              {"name", "Eurotunnel Shuttle"}}},
                            {"IJ", {{"highway", "trunk"}, {"name", ""}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_reclassify_ferry_connections");
  baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));

  auto Bb_edge = std::get<1>(gurka::findEdgeByNodes(graph_reader, layout, "B", "b"));
  auto Gg_edge = std::get<1>(gurka::findEdgeByNodes(graph_reader, layout, "G", "g"));
  return {Bb_edge->classification(), Gg_edge->classification()};
}

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
  // corresponds to 33.3 m/sec (120 km/h) in the below map
  uint32_t ferry_secs = 9;

  const std::string ascii_map = R"(
          A--B--C--D
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
  ASSERT_NEAR(ferry_edge.speed(), ferry_edge.length_km() / (ferry_secs * kHourPerSec), 0.1);
}

TEST(Standalone, ReclassifyFerryConnection) {
  // for these values of 'highway' tag edge class is upgraded in order to connect ferry to a
  // high-class road
  const std::vector<std::string> reclassifiable_ways = {"secondary",      "tertiary",
                                                        "unclassified",   "service",
                                                        "secondary_link", "tertiary_link"};
  for (const auto& cls : reclassifiable_ways) {
    std::map<std::string, std::string> desc = {{"highway", cls}, {"name", ""}};
    baldr::RoadClass bclass, gclass;
    std::tie(bclass, gclass) = ReclassifyFerryConnectionEdge(desc);
    EXPECT_EQ(bclass, baldr::RoadClass::kPrimary);
    EXPECT_EQ(gclass, baldr::RoadClass::kPrimary);
  }
}

TEST(Standalone, DoNotReclassifyFerryConnection) {
  // roads with these values of 'highway' tag do not participate in search for ferry connection so
  // edge class remains low
  const std::vector<std::string> not_reclassifiable_ways = {"track", "living_street"};
  for (const auto& cls : not_reclassifiable_ways) {
    std::map<std::string, std::string> desc = {{"highway", cls}, {"name", ""}};
    baldr::RoadClass bclass, gclass;
    std::tie(bclass, gclass) = ReclassifyFerryConnectionEdge(desc);
    EXPECT_GT(bclass, baldr::RoadClass::kPrimary);
    EXPECT_GT(gclass, baldr::RoadClass::kPrimary);
  }

  // roads with these values of 'service' tag do not participate in search for ferry connection so
  // edge class remains low
  const std::vector<std::string> not_reclassifiable_use = {"parking_aisle", "driveway", "alley",
                                                           "emergency_access", "drive-through"};
  for (const auto& use : not_reclassifiable_use) {
    std::map<std::string, std::string> desc = {{"highway", "tertiary"},
                                               {"service", use},
                                               {"name", ""}};
    baldr::RoadClass bclass, gclass;
    std::tie(bclass, gclass) = ReclassifyFerryConnectionEdge(desc);
    EXPECT_GT(bclass, baldr::RoadClass::kPrimary);
    EXPECT_GT(gclass, baldr::RoadClass::kPrimary);
  }
}
