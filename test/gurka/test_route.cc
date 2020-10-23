#include "gurka.h"
#include <gtest/gtest.h>

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

TEST_F(IgnoreAccessTest, AutoDataFixIgnoreOneWay) {
  const std::string cost = "auto_data_fix";
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, AutoShorterIgnoreOneWay) {
  const std::string cost = "auto_shorter";
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

TEST_F(IgnoreAccessTest, AutoDataFixIgnoreAccess) {
  const std::string cost = "auto_data_fix";
  // ignore edges and nodes access restriction
  EXPECT_THROW(gurka::route(ignore_access_map, {"A", "B", "D"}, cost), std::runtime_error);
  EXPECT_NO_THROW(
      gurka::route(ignore_access_map, {"A", "B", "D"}, cost, {{IgnoreAccessParam(cost), "1"}}));
}

TEST_F(IgnoreAccessTest, AutoShorterIgnoreAccess) {
  const std::string cost = "auto_shorter";
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

/*************************************************************/
class AlgorithmTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
      b--------c--------d
      |        |        |
      |        |        |
      |        |        |
      a--------f--------e
      |        |        |
      |        |        |
      |        |        |
      g--------h--------i
    )";

    const gurka::ways ways = {
        {"abcdefaghie", {{"highway", "residential"}}},
        {"cfh", {{"highway", "tertiary"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/ignore_access");
  }
};

gurka::map AlgorithmTest::map = {};

TEST_F(AlgorithmTest, Astar) {
}

TEST_F(AlgorithmTest, TDForward) {
}

TEST_F(AlgorithmTest, TDReverse) {
}

TEST_F(AlgorithmTest, Bidir) {
}
