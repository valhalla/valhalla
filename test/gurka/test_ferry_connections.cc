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

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
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

INSTANTIATE_TEST_SUITE_P(FerryConnectionTest,
                         FerryTest,
                         ::testing::Values("motorcar", "hgv", "moped", "motorcycle", "taxi", "bus"));
