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
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
          B----------C
         /            \
        A<-------------D
         \            /
          F----------E
    )";

    const gurka::ways ways = {
        {"DA", {{"highway", "service"}, {"oneway", "yes"}}},
        // allowed only for pedestrians
        {"AB", {{"highway", "footway"}, {"bicycle", "no"}}},
        // allowed only for bicycles
        {"AF", {{"highway", "cycleway"}, {"foot", "no"}}},

        {"BC", {{"highway", "service"}}},
        {"CD", {{"highway", "service"}}},
        {"FE", {{"highway", "service"}}},
        {"ED", {{"highway", "service"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    ignore_access_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/ignore_access");
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
  auto result = gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"DA"});
  gurka::assert::raw::expect_path(result, {"DA"});
}

TEST_F(IgnoreAccessTest, AutoIgnoreOneWay) {
  const std::string cost = "auto";
  auto result = gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"DA"});
  gurka::assert::raw::expect_path(result, {"DA"});
}

TEST_F(IgnoreAccessTest, AutoDataFixIgnoreOneWay) {
  const std::string cost = "auto_data_fix";
  auto result = gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"DA"});
  gurka::assert::raw::expect_path(result, {"DA"});
}

TEST_F(IgnoreAccessTest, AutoShorterIgnoreOneWay) {
  const std::string cost = "auto_shorter";
  auto result = gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"DA"});
  gurka::assert::raw::expect_path(result, {"DA"});
}

TEST_F(IgnoreAccessTest, BusIgnoreOneWay) {
  const std::string cost = "bus";
  auto result = gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"DA"});
  gurka::assert::raw::expect_path(result, {"DA"});
}

TEST_F(IgnoreAccessTest, HOVIgnoreOneWay) {
  const std::string cost = "hov";
  auto result = gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"DA"});
  gurka::assert::raw::expect_path(result, {"DA"});
}

TEST_F(IgnoreAccessTest, TaxiIgnoreOneWay) {
  const std::string cost = "taxi";
  auto result = gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"DA"});
  gurka::assert::raw::expect_path(result, {"DA"});
}

TEST_F(IgnoreAccessTest, MotorcycleIgnoreOneWay) {
  const std::string cost = "motorcycle";
  auto result = gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"DA"});
  gurka::assert::raw::expect_path(result, {"DA"});
}

TEST_F(IgnoreAccessTest, MotorScooterIgnoreOneWay) {
  const std::string cost = "motor_scooter";
  auto result = gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"DA"});
  gurka::assert::raw::expect_path(result, {"DA"});
}

TEST_F(IgnoreAccessTest, TruckIgnoreOneWay) {
  const std::string cost = "truck";
  auto result = gurka::route(ignore_access_map, {"A", "D"}, cost, {{IgnoreOneWaysParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"DA"});
  gurka::assert::raw::expect_path(result, {"DA"});
}

// check 'ignore access' parameter

TEST_F(IgnoreAccessTest, BicycleIgnoreAccess) {
  const std::string cost = "bicycle";
  auto result = gurka::route(ignore_access_map, {"A", "B"}, cost, {{IgnoreAccessParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB"});
}

TEST_F(IgnoreAccessTest, AutoIgnoreAccess) {
  const std::string cost = "auto";
  auto result = gurka::route(ignore_access_map, {"A", "B"}, cost, {{IgnoreAccessParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB"});
}

TEST_F(IgnoreAccessTest, AutoDataFixIgnoreAccess) {
  const std::string cost = "auto_data_fix";
  auto result = gurka::route(ignore_access_map, {"A", "B"}, cost, {{IgnoreAccessParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB"});
}

TEST_F(IgnoreAccessTest, AutoShorterIgnoreAccess) {
  const std::string cost = "auto_shorter";
  auto result = gurka::route(ignore_access_map, {"A", "B"}, cost, {{IgnoreAccessParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB"});
}

TEST_F(IgnoreAccessTest, BusIgnoreAccess) {
  const std::string cost = "bus";
  auto result = gurka::route(ignore_access_map, {"A", "B"}, cost, {{IgnoreAccessParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB"});
}

TEST_F(IgnoreAccessTest, HOVIgnoreAccess) {
  const std::string cost = "hov";
  auto result = gurka::route(ignore_access_map, {"A", "B"}, cost, {{IgnoreAccessParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB"});
}

TEST_F(IgnoreAccessTest, TaxiIgnoreAccess) {
  const std::string cost = "taxi";
  auto result = gurka::route(ignore_access_map, {"A", "B"}, cost, {{IgnoreAccessParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB"});
}

TEST_F(IgnoreAccessTest, MotorcycleIgnoreAccess) {
  const std::string cost = "motorcycle";
  auto result = gurka::route(ignore_access_map, {"A", "B"}, cost, {{IgnoreAccessParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB"});
}

TEST_F(IgnoreAccessTest, MotorScooterIgnoreAccess) {
  const std::string cost = "motor_scooter";
  auto result = gurka::route(ignore_access_map, {"A", "B"}, cost, {{IgnoreAccessParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB"});
}

TEST_F(IgnoreAccessTest, PedestrianIgnoreAccess) {
  const std::string cost = "pedestrian";
  auto result = gurka::route(ignore_access_map, {"A", "F"}, cost, {{IgnoreAccessParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"AF"});
  gurka::assert::raw::expect_path(result, {"AF"});
}

TEST_F(IgnoreAccessTest, TruckIgnoreAccess) {
  const std::string cost = "truck";
  auto result = gurka::route(ignore_access_map, {"A", "B"}, cost, {{IgnoreAccessParam(cost), "1"}});
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB"});
}
