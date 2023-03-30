#include "gurka.h"
#include "test.h"

using namespace valhalla;

/*************************************************************/
TEST(Standalone, TruckRegression) {

  const std::string ascii_map = R"(A---1---------------B--------2)";

  const gurka::ways ways = {
      {"AB", {{"highway", "service"}}},
  };

  const double gridsize = 10;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/truck_regression");
  auto result = gurka::do_action(valhalla::Options::route, map, {"1", "2", "1"}, "truck",
                                 {{"/locations/1/type", "break_through"}});
  // Annoyingly because its a node snap at a break through, it starts on AB ends a leg at the end of
  // AB and then starts the next leg at the end of AB and then uturns and goes back on BA
  gurka::assert::raw::expect_path(result, {"AB", "AB", "AB"});
}

/*************************************************************/
class IgnoreAccessTest : public ::testing::Test {
protected:
  static gurka::map ignore_access_map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
          B----------C
         /            \
        A<-------------D
         \            /
          F----------E
    )";

    const gurka::ways ways = {
        // oneway road without access for pedestrians
        {"DA", {{"highway", "service"}, {"oneway", "yes"}, {"foot", "no"}}},
        // allowed only for pedestrians
        {"AB", {{"highway", "footway"}, {"bicycle", "no"}}},
        // allowed only for bicycles
        {"AF", {{"highway", "cycleway"}, {"foot", "no"}}},

        {"BC", {{"highway", "service"}}},
        {"CD", {{"highway", "service"}, {"bicycle", "no"}}},
        {"FE", {{"highway", "service"}}},
        {"ED", {{"highway", "service"}}},
    };

    const gurka::nodes nodes = {
        // gate is opened only for pedestrians
        {"E", {{"barrier", "gate"}, {"access", "no"}, {"foot", "yes"}}},
        // gate is opened only for bicycles
        {"C", {{"barrier", "gate"}, {"access", "no"}, {"bicycle", "yes"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    ignore_access_map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/ignore_access");
  }
};

gurka::map IgnoreAccessTest::ignore_access_map = {};

namespace {
inline std::string IgnoreOneWaysParam(const std::string& cost_name) {
  return "/costing_options/" + cost_name + "/ignore_oneways";
}

inline std::string IgnoreAccessParam(const std::string& cost_name) {
  return "/costing_options/" + cost_name + "/ignore_access";
}
} // namespace

// check 'ignore oneways' parameter

TEST_F(IgnoreAccessTest, BicycleIgnoreOneWay) {
  const std::string cost = "bicycle";
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "D"}, cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "D"}, cost,
                                   {{IgnoreOneWaysParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, AutoIgnoreOneWay) {
  const std::string cost = "auto";
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "D"}, cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "D"}, cost,
                                   {{IgnoreOneWaysParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, BusIgnoreOneWay) {
  const std::string cost = "bus";
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "D"}, cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "D"}, cost,
                                   {{IgnoreOneWaysParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, TaxiIgnoreOneWay) {
  const std::string cost = "taxi";
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "D"}, cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "D"}, cost,
                                   {{IgnoreOneWaysParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, MotorcycleIgnoreOneWay) {
  const std::string cost = "motorcycle";
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "D"}, cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "D"}, cost,
                                   {{IgnoreOneWaysParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, MotorScooterIgnoreOneWay) {
  const std::string cost = "motor_scooter";
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "D"}, cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "D"}, cost,
                                   {{IgnoreOneWaysParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, TruckIgnoreOneWay) {
  const std::string cost = "truck";
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "D"}, cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "D"}, cost,
                                   {{IgnoreOneWaysParam(cost), "1"}}));
}

// check 'ignore access' parameter

TEST_F(IgnoreAccessTest, BicycleIgnoreAccess) {
  const std::string cost = "bicycle";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "B", "D"}, cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "B", "D"}, cost,
                                   {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, AutoIgnoreAccess) {
  const std::string cost = "auto";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "B", "D"}, cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "B", "D"}, cost,
                                   {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, BusIgnoreAccess) {
  const std::string cost = "bus";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "B", "D"}, cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "B", "D"}, cost,
                                   {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, TaxiIgnoreAccess) {
  const std::string cost = "taxi";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "B", "D"}, cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "B", "D"}, cost,
                                   {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, MotorcycleIgnoreAccess) {
  const std::string cost = "motorcycle";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "B", "D"}, cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "B", "D"}, cost,
                                   {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, MotorScooterIgnoreAccess) {
  const std::string cost = "motor_scooter";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "B", "D"}, cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "B", "D"}, cost,
                                   {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, TruckIgnoreAccess) {
  const std::string cost = "truck";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "B", "D"}, cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "B", "D"}, cost,
                                   {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, PedestrianIgnoreAccess) {
  const std::string cost = "pedestrian";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "F", "D", "B"},
                                cost),
               std::runtime_error);
  EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, ignore_access_map, {"A", "F", "D", "B"},
                                   cost, {{IgnoreAccessParam(cost), "1"}}));
}

TEST(AutoDataFix, deprecation) {
  // if both auto & auto_shorter costing options were provided, auto costing should be overridden
  Api request;
  std::string request_str =
      R"({"locations":[{"lat":52.114622,"lon":5.131816},{"lat":52.048267,"lon":5.074825}],"costing":"auto_data_fix",)"
      R"("costing_options":{"auto":{"use_ferry":0.8}, "auto_data_fix":{"use_ferry":0.1, "use_tolls": 0.77}}})";
  ParseApi(request_str, Options::route, request);

  EXPECT_EQ(request.options().costing_type(), Costing::auto_);
  const auto& co = request.options().costings().find(Costing::auto_)->second.options();
  EXPECT_EQ(co.ignore_access(), true);
  EXPECT_EQ(co.ignore_closures(), true);
  EXPECT_EQ(co.ignore_oneways(), true);
  EXPECT_EQ(co.ignore_restrictions(), true);
  EXPECT_EQ(co.use_ferry(), 0.1f);
  EXPECT_EQ(co.use_tolls(), 0.77f);
}

/*************************************************************/
class AlgorithmTest : public ::testing::Test {
protected:
  static gurka::map map;
  static uint32_t current, historical, constrained, freeflow;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
      b----1----c----2----d
      |         |         |
      |         |         |
      |         |         |
      |         |         |
      3         4         5
      |         |         |
      |         |         |
      |         |         |
      |         |         |
      a----6----f----7----e
      |         |         |
      |         |         |
      |         |         |
      |         |         |
      8         9         0
      |         |         |
      |         |         |
      |         |         |
      |         |         |
      g----A----h----B----i
    )";

    const gurka::ways ways = {
        {"abcdefaghie", {{"highway", "residential"}}},
        {"cfh", {{"highway", "tertiary"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/algorithm_selection",
                            {
                                {"mjolnir.shortcuts", "false"},
                                {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"},
                            });
    map.config.put("mjolnir.traffic_extract", "test/data/algorithm_selection/traffic.tar");

    // add live traffic
    test::build_live_traffic_data(map.config);
    test::customize_live_traffic_data(map.config, [&](baldr::GraphReader&, baldr::TrafficTile&, int,
                                                      valhalla::baldr::TrafficSpeed* traffic_speed) {
      traffic_speed->overall_encoded_speed = 50 >> 1;
      traffic_speed->encoded_speed1 = 50 >> 1;
      traffic_speed->breakpoint1 = 255;
    });

    test::customize_historical_traffic(map.config, [](DirectedEdge& e) {
      e.set_constrained_flow_speed(25);
      e.set_free_flow_speed(75);

      // speeds for every 5 min bucket of the week
      std::array<float, kBucketsPerWeek> historical;
      historical.fill(7);
      for (size_t i = 0; i < historical.size(); ++i) {
        // TODO: if we are in morning or evening set a different speed and add another test
      }
      return historical;
    });

    // tests below use saturday at 9am and thursday 4am (27 if for leap seconds)
    size_t second_of_week_constrained = 5 * 24 * 60 * 60 + 9 * 60 * 60 + 27;
    size_t second_of_week_freeflow = 3 * 24 * 60 * 60 + 4 * 60 * 60 + 27;
    auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
    for (auto tile_id : reader->GetTileSet()) {
      auto tile = reader->GetGraphTile(tile_id);
      for (const auto& e : tile->GetDirectedEdges()) {
        current = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0);
        EXPECT_EQ(current, 50);
        historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, second_of_week_constrained);
        EXPECT_EQ(historical, 7);
        constrained = tile->GetSpeed(&e, baldr::kConstrainedFlowMask, second_of_week_constrained);
        EXPECT_EQ(constrained, 25);
        freeflow = tile->GetSpeed(&e, baldr::kFreeFlowMask, second_of_week_freeflow);
        EXPECT_EQ(freeflow, 75);
      }
    }
  }
};

gurka::map AlgorithmTest::map = {};
uint32_t AlgorithmTest::current = 0, AlgorithmTest::historical = 0, AlgorithmTest::constrained = 0,
         AlgorithmTest::freeflow = 0;

uint32_t speed_from_edge(const valhalla::Api& api, bool compare_with_previous_edge = true) {
  uint32_t kmh = -1;
  const auto& nodes = api.trip().routes(0).legs(0).node();
  for (int i = 0; i < nodes.size() - 1; ++i) {
    const auto& node = nodes.Get(i);
    if (!node.has_edge())
      break;
    auto km = node.edge().length_km();
    auto h = (nodes.Get(i + 1).cost().elapsed_cost().seconds() -
              node.cost().elapsed_cost().seconds() - node.cost().transition_cost().seconds()) /
             3600.0;
    auto new_kmh = static_cast<uint32_t>(km / h + .5);
    if (kmh != -1 && compare_with_previous_edge) {
      EXPECT_EQ(kmh, new_kmh);
    }
    kmh = new_kmh;
  }
  return kmh;
}

// this happens with depart_at routes trivial or not and trivial invariant routes
TEST_F(AlgorithmTest, TDForward) {
  {
    auto api = gurka::do_action(valhalla::Options::route, map, {"0", "3"}, "auto",
                                {{"/date_time/type", "1"}, {"/date_time/value", "2020-10-30T09:00"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  }

  {
    auto api = gurka::do_action(valhalla::Options::route, map, {"8", "A"}, "auto",
                                {{"/date_time/type", "1"}, {"/date_time/value", "2020-10-30T09:00"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  }

  {
    auto api = gurka::do_action(valhalla::Options::route, map, {"2", "5"}, "auto",
                                {{"/date_time/type", "3"}, {"/date_time/value", "2020-10-30T09:00"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  }
}

// this happens with arrive_by routes trivial or not
TEST_F(AlgorithmTest, TDReverse) {
  {
    auto api = gurka::do_action(valhalla::Options::route, map, {"6", "B"}, "auto",
                                {{"/date_time/type", "2"}, {"/date_time/value", "2020-10-30T09:00"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "time_dependent_reverse_a*");
  }

  {
    auto api = gurka::do_action(valhalla::Options::route, map, {"9", "7"}, "auto",
                                {{"/date_time/type", "2"}, {"/date_time/value", "2020-10-30T09:00"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "time_dependent_reverse_a*");
  }
}

// this happens only with non-trivial routes with no date_time or invariant date_time
TEST_F(AlgorithmTest, Bidir) {
  {
    auto api = gurka::do_action(valhalla::Options::route, map, {"4", "0"}, "auto");
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    EXPECT_EQ(speed_from_edge(api), constrained);
  }

  {
    auto api = gurka::do_action(valhalla::Options::route, map, {"A", "2"}, "auto",
                                {{"/date_time/type", "3"},
                                 {"/date_time/value", "2020-10-30T09:00"},
                                 {"/costing_options/auto/speed_types/0", "constrained"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    EXPECT_EQ(speed_from_edge(api), constrained);
  }

  {
    auto api = gurka::do_action(valhalla::Options::route, map, {"A", "2"}, "auto",
                                {{"/date_time/type", "3"},
                                 {"/date_time/value", "2020-10-30T09:00"},
                                 {"/costing_options/auto/speed_types/0", "predicted"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    EXPECT_EQ(speed_from_edge(api), historical);
  }

  {
    auto api = gurka::do_action(valhalla::Options::route, map, {"A", "2"}, "auto",
                                {{"/date_time/type", "3"},
                                 {"/date_time/value", "2020-10-30T09:00"},
                                 {"/costing_options/auto/speed_types/0", "current"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    // Because of live-traffic smoothing, speed will be mixed with default edge speed in the end of
    // the route.
    EXPECT_LE(speed_from_edge(api, false), current);
  }

  {
    auto api = gurka::do_action(valhalla::Options::route, map, {"A", "2"}, "auto",
                                {{"/date_time/type", "3"},
                                 {"/date_time/value", "2020-10-28T04:00"},
                                 {"/costing_options/auto/speed_types/0", "freeflow"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
    EXPECT_EQ(speed_from_edge(api), freeflow);
  }
}

/*************************************************************/
TEST(Standalone, LegWeightRegression) {

  const std::string ascii_map = R"(A-1---B----------C-3-----D
                                          \        /
                                           E----2-F)";

  const gurka::ways ways = {{"ABCD", {{"highway", "motorway"}}},
                            {"BE", {{"highway", "motorway_link"}}},
                            {"FC", {{"highway", "motorway_link"}}},
                            {"EF", {{"highway", "service"}}}};

  const double gridsize = 30;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/leg_weights");
  auto result = gurka::do_action(valhalla::Options::route, map, {"1", "E", "3"}, "auto",
                                 {{"/locations/1/type", "via"}});

  auto doc = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  // "weight" was negative due to failing to properly update elapsed_cost.cost
  EXPECT_GT(doc["routes"][0]["legs"][0]["steps"][2]["weight"].GetDouble(), 0);
}

TEST(Standalone, DontIgnoreRestriction) {
  const std::string ascii_map = R"(
                D
               /
      A----B--C
           |
           E
)";

  const gurka::ways ways = {
      {"DC", {{"highway", "secondary"}, {"oneway", "yes"}}},
      {"CB", {{"highway", "secondary"}, {"oneway", "yes"}}},
      {"BE", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"BA", {{"highway", "primary"}, {"oneway", "yes"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "DC", "from"},
           {gurka::way_member, "CB", "via"},
           {gurka::way_member, "BE", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_entry"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/dont_ignore_restriction",
                               {{"mjolnir.concurrency", "1"}});

  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"D", "E"}, "auto");
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
}

TEST(Standalone, BridgingEdgeIsRestricted) {
  const std::string ascii_map = R"(
   A----B----C--------D----E----F-----G
)";

  const gurka::ways ways = {
      {"AB", {{"highway", "secondary"}, {"oneway", "yes"}}},
      {"BC", {{"highway", "secondary"}, {"oneway", "yes"}}},
      {"CD", {{"highway", "secondary"}, {"oneway", "yes"}}},
      {"DE", {{"highway", "secondary"}, {"oneway", "yes"}}},
      {"EF", {{"highway", "secondary"}, {"oneway", "yes"}}},
      {"FG", {{"highway", "primary"}, {"oneway", "yes"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "BC", "from"},
           {gurka::way_member, "CD", "via"},
           {gurka::way_member, "DE", "via"},
           {gurka::way_member, "EF", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_entry"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/bridging_edge_is_restricted",
                               {{"mjolnir.concurrency", "1"}});

  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto");
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
}

TEST(Standalone, AvoidExtraDetours) {
  // Check that we don't take extra detours to get the target point. Here is
  // a special usecase that breaks previous logic about connection edges in bidirectional astar.
  const std::string ascii_map = R"(
                             F
                             2
                             |
                             E
                             |
      A-1--B--------C--------D---------H---------------------------------------I
                                       |                                       |
                                       K---------------------------------------J
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}},

      {"DE", {{"highway", "service"}, {"service", "driveway"}}},
      {"EF", {{"highway", "service"}, {"service", "driveway"}}},

      {"DH", {{"highway", "primary"}}},
      {"HI", {{"highway", "primary"}}},
      {"IJ", {{"highway", "primary"}}},
      {"JK", {{"highway", "primary"}, {"maxspeed", "10"}}},
      {"KH", {{"highway", "primary"}}},
  };

  const auto nodes = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(nodes, ways, {}, {}, "test/data/avoid_extra_detours");

  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));

  std::vector<GraphId> not_thru_edgeids;
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, nodes, "D", "C")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, nodes, "C", "B")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, nodes, "B", "A")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, nodes, "D", "E")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, nodes, "E", "F")));

  test::customize_edges(map.config, [&not_thru_edgeids](const GraphId& edgeid, DirectedEdge& edge) {
    if (std::find(not_thru_edgeids.begin(), not_thru_edgeids.end(), edgeid) != not_thru_edgeids.end())
      edge.set_not_thru(true);
  });

  auto result = gurka::do_action(valhalla::Options::route, map, {"1", "2"}, "auto");
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF"});
}

TEST(Standalone, DoNotAllowDoubleUturns) {
  // This test checks that bidirectional astar doesn't allow double uturns.
  // It might happens if we handle connection edges incorrectly.
  const std::string ascii_map = R"(
  J---------------2---------I-------------------------------F
                            |
                            |
  A---1-------B-------------C------D------------------------E
              |                    |
              K--------------------L
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"BC", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"CI", {{"highway", "secondary"}}},
      {"CD", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"DE", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"FI", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"IJ", {{"highway", "primary"}, {"oneway", "yes"}}},

      {"BK", {{"highway", "secondary"}}},
      {"KL", {{"highway", "secondary"}}},
      {"LD", {{"highway", "secondary"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "BC", "from"},
           {gurka::way_member, "CI", "via"},
           {gurka::way_member, "IJ", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_u_turn"},
       }},
  };

  const auto nodes = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(nodes, ways, {}, relations, "test/data/do_not_allow_double_uturns");

  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"1", "2"}, "auto");
    const auto names = gurka::detail::get_paths(result).front();
    if (std::count(names.begin(), names.end(), "CI") == 3)
      FAIL() << "Double uturn detected!";
    else
      FAIL() << "Unexpected path found!";
  } catch (const std::exception& e) { EXPECT_STREQ(e.what(), "No path could be found for input"); }
}

TEST(Standalone, OneWayIdToManyEdgeIds) {
  const std::string ascii_map = R"(
        A<--B<--C<---D
                ^
                |
                E
                ^
                |
                F
)";

  const gurka::ways ways = {
      {"FE", {{"highway", "secondary"}, {"oneway", "yes"}, {"osm_id", "1"}}},
      {"EC", {{"highway", "secondary"}, {"oneway", "yes"}, {"osm_id", "2"}}},
      {"CB", {{"highway", "primary"}, {"oneway", "yes"}, {"osm_id", "3"}}},
      {"BA", {{"highway", "primary"}, {"oneway", "yes"}, {"osm_id", "3"}}},
      {"DC", {{"highway", "primary"}, {"oneway", "yes"}, {"osm_id", "3"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "FE", "from"},
           {gurka::way_member, "EC", "via"},
           {gurka::way_member, "CB", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_left_turn"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/one_way_id_to_many_edge_ids",
                               {{"mjolnir.concurrency", "1"}});

  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"F", "B"}, "auto");
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
}

TEST(Standalone, HonorAccessPropertyWhenConstructingRestriction) {
  const std::string ascii_map = R"(
        A<-----B<----D
               ^
               |
        E----->F---->G
)";

  const gurka::ways ways = {
      {"EF", {{"highway", "secondary"}, {"oneway", "yes"}, {"osm_id", "1"}}},
      {"FG", {{"highway", "secondary"}, {"oneway", "yes"}, {"osm_id", "1"}}},
      {"FB", {{"highway", "primary"}, {"oneway", "yes"}, {"osm_id", "2"}}},
      {"DB", {{"highway", "primary"}, {"oneway", "yes"}, {"osm_id", "3"}}},
      {"BA", {{"highway", "primary"}, {"oneway", "yes"}, {"osm_id", "3"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "EF", "from"},
           {gurka::way_member, "FB", "via"},
           {gurka::way_member, "BA", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_u_turn"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/honor_restriction_access_mode",
                               {{"mjolnir.concurrency", "1"}});

  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"E", "A"}, "auto");
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
}

TEST(MultipointRoute, WithIsolatedPoints) {
  /*
   * Locations 2 and 3 lie on isolated island, but location 1 belongs to a big connectivity component.
   * This test checks that we snap points on isolated island to the main road.
   */
  const std::string ascii_map = R"(
                                   C------------------D
                                   |   X---2---3--Y   |
  A-1------------------------------B                  |
                                   |                  |
                                   |                  |
                                   F------------------E
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},   {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}},   {"DE", {{"highway", "primary"}}},
      {"EF", {{"highway", "primary"}}},   {"FB", {{"highway", "primary"}}},

      {"XY", {{"highway", "secondary"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 25);

  std::unordered_map<std::string, std::string> config_opts = {
      {"mjolnir.concurrency", "1"},
      {"loki.service_defaults.minimum_reachability", "5"},
  };

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/multipoint_with_isolated_points",
                               config_opts);

  // Locations 2 and 3 should be snapped to the main road CD.
  // no datetime, check bidirectional astar
  auto result = gurka::do_action(valhalla::Options::route, map, {"3", "2", "1"}, "auto", {}, {},
                                 nullptr, "break_through");
  gurka::assert::raw::expect_path(result, {"CD", "CD", "BC", "AB"});

  // with depart_at, check timedep forward astar
  result = gurka::do_action(valhalla::Options::route, map, {"3", "2", "1"}, "auto",
                            {{"/date_time/type", "1"}, {"/date_time/value", "2021-03-12T19:00"}}, {},
                            nullptr, "break_through");
  gurka::assert::raw::expect_path(result, {"CD", "CD", "BC", "AB"});

  // with arrive_by, check timedep reverse astar
  result = gurka::do_action(valhalla::Options::route, map, {"1", "2", "3"}, "auto",
                            {{"/date_time/type", "2"}, {"/date_time/value", "2021-03-12T19:00"}}, {},
                            nullptr, "break_through");
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "CD"});
}

TEST(MultipointRoute, WithPointsOnDeadends) {
  const std::string ascii_map = R"(
    A-1--2-B-3--4-C
          / \
         D   E
  )";

  // BD and BE are oneways that lead away, and toward the main road A-B-C
  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"BD", {{"highway", "secondary"}, {"oneway", "yes"}}},
      {"BE", {{"highway", "secondary"}, {"oneway", "yes"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 25);

  std::unordered_map<std::string, std::string> config_opts = {
      {"mjolnir.concurrency", "1"},
      {"loki.service_defaults.minimum_reachability", "3"},
  };

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/multipoint_with_points_on_deadends",
                               config_opts);

  {
    // Verify simple waypoint routing works
    auto result1 = gurka::do_action(valhalla::Options::route, map, {"1", "2", "4"}, "auto", {}, {},
                                    nullptr, "break_through");
    gurka::assert::raw::expect_path(result1, {"AB", "AB", "BC"});

    auto result2 = gurka::do_action(valhalla::Options::route, map, {"1", "3", "4"}, "auto", {}, {},
                                    nullptr, "break_through");
    gurka::assert::raw::expect_path(result2, {"AB", "BC", "BC"});
  }
  {
    // BD is a oneway leading away from ABC with no escape
    // input coordinate at D should get snapped to the ABC way
    // Should give the same result as 1->2->4
    auto result = gurka::do_action(valhalla::Options::route, map, {"1", "D", "4"}, "auto", {}, {},
                                   nullptr, "break_through");
    gurka::assert::raw::expect_path(result, {"AB", "AB", "BC"});
  }
  {
    // BE is a oneway leading towards ABC with no way in
    // input coordinate at E should get snapped to the ABC way
    // Should give the same result as 1->3->4
    auto result = gurka::do_action(valhalla::Options::route, map, {"1", "E", "4"}, "auto", {}, {},
                                   nullptr, "break_through");
    gurka::assert::raw::expect_path(result, {"AB", "BC", "BC"});
  }
}

// prove that non-bidir A* algorithms allow destination-only routing in their first pass
TEST(AlgorithmTestDest, TestAlgoSwapAndDestOnly) {
  constexpr double gridsize = 100;

  const std::string ascii_map = R"(
      B---------------C
      |               |
      |               8
      |               ↑
      |               |
      Y---------------Z
      |               |
      7               9
      |               |
      A---------------D
           )";
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  const gurka::ways ways = {{"AB", {{"highway", "motorway"}}},
                            {"BC", {{"highway", "primary"}}},
                            {"CZ",
                             {{"highway", "primary"}, {"oneway", "-1"}, {"access", "destination"}}},
                            {"ZD", {{"highway", "primary"}, {"access", "destination"}}},
                            {"DA", {{"highway", "primary"}}},
                            {"AY", {{"highway", "primary"}}},
                            {"YZ", {{"highway", "primary"}}}};
  gurka::map map = gurka::buildtiles(layout, ways, {}, {}, "test/data/algo_swap_dest_only");

  // Notes on this test:
  // * We want the first leg to choose bidir A*:
  // == The path from 7 to 8 cannot be directly/trivially connected, hence YZ exists. This ensures
  //    the first leg of the route chooses bidir A*.
  // * We want the second leg to choose unidir A*:
  // == The path between 8 & 9 needs to be directly/trivially connected so unidir A* is chosen for
  //    second leg.
  // == However, to ensure the 8->9 route cannot be solved without destination_only=true, we have
  //    "oneway" attribution on ZC. This forces a trip "around the block" for the second leg.
  // == ZD & CZ need access=destination or second leg will use bidir A*.
  // == Also, CZ needs access access=destination so we prove that unidir A* solves the route
  //    from 8->9 in its first pass (we are trying to prove that non-bidir A* algorithms allow
  //    destination-only routing in their first pass).
  //
  // * Below, see that a heading is specified. This ensures filtered-edges are added to the final
  //   destination node (9). Another important part of the test, see more comments below.
  auto from = "7";
  auto mid = "8";
  auto to = "9";
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"heading":180,"heading_tolerance":45}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(mid).lat()) % std::to_string(map.nodes.at(mid).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();

  auto api = gurka::do_action(valhalla::Options::route, map, request);

  ASSERT_EQ(api.trip().routes(0).legs_size(), 2);

  EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
  EXPECT_EQ(api.trip().routes(0).legs(1).algorithms(0), "time_dependent_forward_a*");

  EXPECT_EQ(api.trip().routes(0).legs(0).node(0).edge().destination_only(), false);
  EXPECT_EQ(api.trip().routes(0).legs(1).node(0).edge().destination_only(), true);

  // These "expected_path_edge_sizes" are from the perspective of each node (7, 8, 9).
  // Without the fix, the path-edge sizes are {2, 1, 2}. The third int is 2 because unidir A*
  // would previously enter fallback logic to solve the second leg (8->9) and consequently
  // add a "filtered edge". We prove we aren't entering fallback logic by asserting that the
  // third int is 1.
  //
  // Honestly, this is a slightly convoluted way to test that "non-bidir A* algorithms allow
  // destination only routing on their first-pass". This relies on some internal counts that
  // imply some knowledge about the inner workings of Valhalla. Valhalla could evolve for
  // the better, these numbers could change and this test could fail. But that doesn't mean
  // your code is bad. Just consider this test's intent and whether what its testing still makes
  // sense relative to your changes.
  std::vector<int> expected_path_edge_sizes = {2, 1, 1};
  std::vector<int> actual_path_edge_sizes;
  actual_path_edge_sizes.reserve(api.options().locations_size());
  for (int i = 0; i < api.options().locations_size(); i++) {
    actual_path_edge_sizes.emplace_back(api.options().locations(i).correlation().edges().size());
  }
  ASSERT_EQ(expected_path_edge_sizes, actual_path_edge_sizes);
}

TEST(AlgorithmTestTrivial, unidirectional_regression) {
  constexpr double gridsize_metres = 10;
  const std::string ascii_map = R"(A1234B5678C)";
  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {0.00, 0.00});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_trivial_regression");

  // the code used to remove one of the origin edge candidates which then forced a uturn
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "3"}, "auto");
  gurka::assert::raw::expect_path(result, {"AB"});

  // again with reverse a* search direction
  result = gurka::do_action(valhalla::Options::route, map, {"3", "A"}, "auto",
                            {
                                {"/date_time/type", "2"},
                                {"/date_time/value", "2111-11-11T11:11"},
                            });
  gurka::assert::raw::expect_path(result, {"AB"});
}

// check that non-bidir A* algorithms behave well with multiple origin and destination edges
TEST(AlgorithmTestDest, TestAlgoMultiOriginDestination) {
  constexpr double gridsize = 10;
  constexpr double radius = gridsize * 20;

  const std::string ascii_map = R"(
       A-----B-----C-----D
     1   2 3   4 5   6 7   8
           )";
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                            {"BC", {{"highway", "primary"}}},
                            {"CD", {{"highway", "primary"}}}};
  gurka::map map = gurka::buildtiles(layout, ways, {}, {}, "test/data/algo_multi_o_d");

  auto check = [&](const char* from, const char* to, const std::vector<std::string>& expected_names) {
    for (int type = 1; type <= 2; type++) {
      const std::string& request =
          (boost::format(
               R"({"locations":[{"lat":%s,"lon":%s,"radius":%s,"node_snap_tolerance":0},{"lat":%s,"lon":%s,"radius":%s,"node_snap_tolerance":0}],"costing":"auto","date_time":{"type":%s,"value":"2111-11-11T11:11"}})") %
           std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
           std::to_string(radius) % std::to_string(map.nodes.at(to).lat()) %
           std::to_string(map.nodes.at(to).lng()) % std::to_string(radius) % std::to_string(type))
              .str();

      auto result = gurka::do_action(valhalla::Options::route, map, request);

      EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0),
                type == 1 ? "time_dependent_forward_a*" : "time_dependent_reverse_a*");
      EXPECT_EQ(result.options().locations(0).correlation().edges().size(), 6);
      EXPECT_EQ(result.options().locations(1).correlation().edges().size(), 6);

      gurka::assert::raw::expect_path(result, expected_names);
    }
  };

  check("1", "1", {"AB"});
  check("1", "2", {"AB"});
  check("1", "3", {"AB"});
  check("1", "4", {"AB", "BC"});
  check("1", "5", {"AB", "BC"});
  check("1", "6", {"AB", "BC", "CD"});
  check("1", "7", {"AB", "BC", "CD"});
  check("1", "8", {"AB", "BC", "CD"});

  check("2", "2", {"AB"});
  check("2", "3", {"AB"});
  check("2", "4", {"AB", "BC"});
  check("2", "5", {"AB", "BC"});
  check("2", "6", {"AB", "BC", "CD"});
  check("2", "7", {"AB", "BC", "CD"});
  check("2", "8", {"AB", "BC", "CD"});

  check("3", "3", {"AB"});
  check("3", "4", {"AB", "BC"});
  check("3", "5", {"AB", "BC"});
  check("3", "6", {"AB", "BC", "CD"});
  check("3", "7", {"AB", "BC", "CD"});
  check("3", "8", {"AB", "BC", "CD"});

  check("4", "4", {"BC"});
  check("4", "5", {"BC"});
  check("4", "6", {"BC", "CD"});
  check("4", "7", {"BC", "CD"});
  check("4", "8", {"BC", "CD"});

  check("5", "5", {"BC"});
  check("5", "6", {"BC", "CD"});
  check("5", "7", {"BC", "CD"});
  check("5", "8", {"BC", "CD"});

  check("6", "6", {"CD"});
  check("6", "7", {"CD"});
  check("6", "8", {"CD"});

  check("7", "7", {"CD"});
  check("7", "8", {"CD"});

  check("8", "8", {"CD"});
}