#include "gurka.h"
#include <baldr/graphconstants.h>
#include <gtest/gtest.h>
#include <valhalla/sif/costconstraints.h>
using namespace valhalla;

class Constraints : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
    A----B----C----D
    |         |  /
    E----F----G)";

    // BE and EH are highway=path, so no cars
    // EI is a shortcut that's not accessible to bikes
    const gurka::ways ways = {{"AB", {{"highway", "motorway"}}},  {"BC", {{"highway", "motorway"}}},
                              {"CD", {{"highway", "motorway"}}},  {"CG", {{"highway", "secondary"}}},
                              {"AE", {{"highway", "secondary"}}}, {"EF", {{"highway", "secondary"}}},
                              {"FG", {{"highway", "secondary"}}}, {"GD", {{"highway", "secondary"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/constraints");
  }

  static double GetConstraint(uint32_t idx,
                              const baldr::DirectedEdge* edge,
                              const baldr::GraphTile* tile,
                              const uint32_t secs,
                              const float end_time,
                              const bool is_forward) {
    if (edge->classification() == baldr::RoadClass::kMotorway)
      return 11;
    return 5;
  }
};

gurka::map Constraints::map = {};

/*************************************************************/

// A simple test to make sure we didn't break routing without constraints...
TEST_F(Constraints, NoConstraints) {
  auto result = gurka::route(map, "A", "D", "auto");
  gurka::assert::osrm::expect_route(result, {"AB", "BC", "CD"});
}

// test whether we take the longer route to meet our constraint...
TEST_F(Constraints, WithConstraints) {
  sif::CostConstraints::StaticComputeConstraintForEdgeF = Constraints::GetConstraint;
  auto result = gurka::route_with_costing_options(
      map, std::vector<std::string>{"A", "D"}, "auto",
      "{\"auto\":{ \"constraints\":[{\"max_inclusive\":true,\"min_inclusive\":true,\"max_value\":20.0}]}}");
  gurka::assert::osrm::expect_route(result, {"AE", "EF", "FG", "GD"});
  result = gurka::route_with_costing_options(
      map, std::vector<std::string>{"A", "D"}, "auto",
      "{\"auto\":{ \"constraints\":[{\"max_inclusive\":true,\"min_inclusive\":true,\"max_value\":32.0}]}}");
  gurka::assert::osrm::expect_route(result, {"AB", "BC", "CG", "GD"});
}

// Finally test that routing fails when a secondary constraint is exceeded for all possible routes...
TEST_F(Constraints, WithConstraintsNoSolution) {
  sif::CostConstraints::StaticComputeConstraintForEdgeF = Constraints::GetConstraint;
  EXPECT_THROW(
      gurka::route(
          map, std::vector<std::string>{"A", "D"}, "auto",
          "{\"auto\":{ \"constraints\":[{\"max_inclusive\":true,\"min_inclusive\":true,\"max_value\":3.0}]}}"),
      std::exception);
}