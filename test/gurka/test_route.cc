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

TEST_F(IgnoreAccessTest, HOVIgnoreOneWay) {
  const std::string cost = "hov";
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

TEST_F(IgnoreAccessTest, HOVIgnoreAccess) {
  const std::string cost = "hov";
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

  EXPECT_EQ(request.options().costing(), valhalla::auto_);
  EXPECT_EQ(request.options().costing_options(valhalla::auto_).ignore_access(), true);
  EXPECT_EQ(request.options().costing_options(valhalla::auto_).ignore_closures(), true);
  EXPECT_EQ(request.options().costing_options(valhalla::auto_).ignore_oneways(), true);
  EXPECT_EQ(request.options().costing_options(valhalla::auto_).ignore_restrictions(), true);
  EXPECT_EQ(request.options().costing_options(valhalla::auto_).use_ferry(), 0.1f);
  EXPECT_EQ(request.options().costing_options(valhalla::auto_).use_tolls(), 0.77f);
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
      traffic_speed->overall_speed = 50 >> 1;
      traffic_speed->speed1 = 50 >> 1;
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

uint32_t speed_from_edge(const valhalla::Api& api) {
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
    if (kmh != -1)
      EXPECT_EQ(kmh, new_kmh);
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
    EXPECT_EQ(speed_from_edge(api), current);
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
