#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

baldr::RoadClass
ReclassifyFerryConnectionEdge(std::map<std::string, std::string> const& way_description) {
  constexpr double gridsize_metres = 1000;

  const std::string ascii_map = R"(
          A--B--b--C----D--E
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
                            {"DE", {{"highway", "trunk"}, {"name", ""}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_reclassify_ferry_connections");
  baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));

  auto edge = std::get<1>(gurka::findEdgeByNodes(graph_reader, layout, "B", "b"));
  return edge->classification();
}

TEST(Standalone, ReclassifyFerryConnection) {
  // for these values of 'highway' tag edge class is upgraded in order to connect ferry to a
  // high-class road
  const std::vector<std::string> reclassifiable_ways = {"secondary",      "tertiary",
                                                        "unclassified",   "service",
                                                        "secondary_link", "tertiary_link"};
  for (const auto& cls : reclassifiable_ways) {
    std::map<std::string, std::string> desc = {{"highway", cls}, {"name", ""}};
    auto edge_class = ReclassifyFerryConnectionEdge(desc);
    EXPECT_EQ(edge_class, baldr::RoadClass::kPrimary);
  }
}

TEST(Standalone, DoNotReclassifyFerryConnection) {
  // roads with these values of 'highway' tag do not participate in search for ferry connection so
  // edge class remains low
  const std::vector<std::string> not_reclassifiable_ways = {"track", "living_street"};
  for (const auto& cls : not_reclassifiable_ways) {
    std::map<std::string, std::string> desc = {{"highway", cls}, {"name", ""}};
    auto edge_class = ReclassifyFerryConnectionEdge(desc);
    EXPECT_GE(edge_class, baldr::RoadClass::kPrimary);
  }

  // roads with these values of 'service' tag do not participate in search for ferry connection so
  // edge class remains low
  const std::vector<std::string> not_reclassifiable_use = {"parking_aisle", "driveway", "alley",
                                                           "emergency_access", "drive-through"};
  for (const auto& use : not_reclassifiable_use) {
    std::map<std::string, std::string> desc = {{"highway", "tertiary"},
                                               {"service", use},
                                               {"name", ""}};
    auto edge_class = ReclassifyFerryConnectionEdge(desc);
    EXPECT_GE(edge_class, baldr::RoadClass::kPrimary);
  }
}
