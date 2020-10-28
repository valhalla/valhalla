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
  auto result = gurka::route(map, {"1", "2", "1"}, "truck", {{"/locations/1/type", "break_through"}});
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
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, AutoIgnoreOneWay) {
  const std::string cost = "auto";
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, BusIgnoreOneWay) {
  const std::string cost = "bus";
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, HOVIgnoreOneWay) {
  const std::string cost = "hov";
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, TaxiIgnoreOneWay) {
  const std::string cost = "taxi";
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, MotorcycleIgnoreOneWay) {
  const std::string cost = "motorcycle";
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, MotorScooterIgnoreOneWay) {
  const std::string cost = "motor_scooter";
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, TruckIgnoreOneWay) {
  const std::string cost = "truck";
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}}));
}

// check 'ignore access' parameter

TEST_F(IgnoreAccessTest, BicycleIgnoreAccess) {
  const std::string cost = "bicycle";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "B", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "B", "D"}, cost, {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, AutoIgnoreAccess) {
  const std::string cost = "auto";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "B", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "B", "D"}, cost, {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, BusIgnoreAccess) {
  const std::string cost = "bus";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "B", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "B", "D"}, cost, {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, HOVIgnoreAccess) {
  const std::string cost = "hov";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "B", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "B", "D"}, cost, {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, TaxiIgnoreAccess) {
  const std::string cost = "taxi";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "B", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "B", "D"}, cost, {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, MotorcycleIgnoreAccess) {
  const std::string cost = "motorcycle";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "B", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "B", "D"}, cost, {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, MotorScooterIgnoreAccess) {
  const std::string cost = "motor_scooter";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "B", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "B", "D"}, cost, {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, TruckIgnoreAccess) {
  const std::string cost = "truck";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "B", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "B", "D"}, cost, {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, PedestrianIgnoreAccess) {
  const std::string cost = "pedestrian";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "F", "D", "B"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "F", "D", "B"}, cost, {{IgnoreAccessParam(cost), "1"}}));
}

TEST(AutoDataFix, deprecation) {
  // if both auto & auto_shorter costing options were provided, auto costing should be overridden
  Api request;
  std::string request_str =
      R"({"locations":[{"lat":52.114622,"lon":5.131816},{"lat":52.048267,"lon":5.074825}],"costing":"auto_data_fix",)"
      R"("costing_options":{"auto":{"use_ferry":0.8}, "auto_data_fix":{"use_ferry":0.1, "use_tolls": 0.77}}})";
  ParseApi(request_str, Options::route, request);

  ASSERT_EQ(request.options().costing(), valhalla::auto_);
  ASSERT_EQ(request.options().costing_options(valhalla::auto_).ignore_access(), true);
  ASSERT_EQ(request.options().costing_options(valhalla::auto_).ignore_closures(), true);
  ASSERT_EQ(request.options().costing_options(valhalla::auto_).ignore_oneways(), true);
  ASSERT_EQ(request.options().costing_options(valhalla::auto_).ignore_restrictions(), true);
  ASSERT_EQ(request.options().costing_options(valhalla::auto_).use_ferry(), 0.1f);
  ASSERT_EQ(request.options().costing_options(valhalla::auto_).use_tolls(), 0.77f);
}

/*************************************************************/
class AlgorithmTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

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

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/algorithm_selection");
    map.config.put("mjolnir.traffic_extract", "test/data/algorithm_selection/traffic.tar");

    // add live traffic
    test::build_live_traffic_data(map.config);
    test::customize_live_traffic_data(map.config,
                                      [&](baldr::GraphReader& reader,
                                          baldr::TrafficTile& traffic_tile, int edge_index,
                                          valhalla::baldr::TrafficSpeed* traffic_speed) {
                                        baldr::GraphId edge_id(traffic_tile.header->tile_id);
                                        edge_id.set_id(edge_index);
                                      });

    // TODO: add historical traffic

    // std::cout << test::dump_geojson_graph(map.config) << std::endl;
  }
};

gurka::map AlgorithmTest::map = {};

// this only happens if with trivial routes that have no date_time
TEST_F(AlgorithmTest, Astar) {
  {
    auto api = gurka::route(map, {"3", "1"}, "auto");
    ASSERT_EQ(api.trip().routes(0).legs(0).algorithms(0), "a*");
  }
}

// this happens with depart_at routes trivial or not and trivial invariant routes
TEST_F(AlgorithmTest, TDForward) {
  {
    auto api = gurka::route(map, {"0", "3"}, "auto",
                            {{"/date_time/type", "1"}, {"/date_time/value", "2020-10-29T09:00"}});
    ASSERT_EQ(api.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  }

  {
    auto api = gurka::route(map, {"8", "A"}, "auto",
                            {{"/date_time/type", "1"}, {"/date_time/value", "2020-10-29T09:00"}});
    ASSERT_EQ(api.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  }

  {
    auto api = gurka::route(map, {"2", "5"}, "auto",
                            {{"/date_time/type", "3"}, {"/date_time/value", "2020-10-29T09:00"}});
    ASSERT_EQ(api.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  }
}

// this happens with arrive_by routes trivial or not
TEST_F(AlgorithmTest, TDReverse) {
  {
    auto api = gurka::route(map, {"6", "B"}, "auto",
                            {{"/date_time/type", "2"}, {"/date_time/value", "2020-10-29T09:00"}});
    ASSERT_EQ(api.trip().routes(0).legs(0).algorithms(0), "time_dependent_reverse_a*");
  }

  {
    auto api = gurka::route(map, {"9", "7"}, "auto",
                            {{"/date_time/type", "2"}, {"/date_time/value", "2020-10-29T09:00"}});
    ASSERT_EQ(api.trip().routes(0).legs(0).algorithms(0), "time_dependent_reverse_a*");
  }
}

// this happens only with non-trivial routes with no date_time or invariant date_time
TEST_F(AlgorithmTest, Bidir) {
  /*{
    auto api = gurka::route(map, {"4", "0"}, "auto");
    ASSERT_EQ(api.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
  }

  {
    auto api = gurka::route(map, {"A", "2"}, "auto",
                            {{"/date_time/type", "4"}, {"/date_time/value", "2020-10-29T09:00"}});
    ASSERT_EQ(api.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
  }*/
}
