#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla;

const std::vector<std::string>& costing = {"auto",  "taxi",          "bus",        "bicycle",
                                           "truck", "motor_scooter", "motorcycle", "pedestrian"};

void validate_path(const valhalla::Api& result, const std::vector<std::string>& expected_names) {
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, expected_names);
}

class LivingStreetTest : public ::testing::Test {
protected:
  static gurka::map use_living_streets_map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
    A-1----B-b----e-E-----2-F
           |        |       |
           |        |       |
           |        |       3
           |        |       |
           C--------D       G
                     \
                      I
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "living_street"}}}, {"Bb", {{"highway", "residential"}}},
        {"be", {{"highway", "living_street"}}}, {"eE", {{"highway", "residential"}}},
        {"EF", {{"highway", "living_street"}}}, {"FG", {{"highway", "residential"}}},
        {"BC", {{"highway", "residential"}}},   {"CD", {{"highway", "residential"}}},
        {"DE", {{"highway", "residential"}}},   {"DI", {{"highway", "secondary"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    use_living_streets_map =
        gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_use_living_streets");

    // set the same historical speeds for all roads
    test::customize_historical_traffic(use_living_streets_map.config, [](baldr::DirectedEdge& e) {
      e.set_free_flow_speed(40);
      e.set_constrained_flow_speed(40);
      return std::array<float, kBucketsPerWeek>{};
    });
  }
};

gurka::map LivingStreetTest::use_living_streets_map = {};

TEST_F(LivingStreetTest, test_default_value) {
  for (const auto& c : costing)
    if (c == "bicycle" || c == "pedestrian")
      // living_street tag shouldn't affect these costings too much
      validate_path(gurka::do_action(valhalla::Options::route, use_living_streets_map, {"1", "2"}, c),
                    {"AB", "Bb", "be", "eE", "EF"});
    else
      // avoid living_streets by default; use living_streets only if the route starts or ends at
      // 'living_street' edge
      validate_path(gurka::do_action(valhalla::Options::route, use_living_streets_map, {"1", "2"}, c),
                    {"AB", "BC", "CD", "DE", "EF"});
}

TEST_F(LivingStreetTest, test_use_living_streets) {
  // use routes with living_streets
  for (const auto& c : costing)
    validate_path(gurka::do_action(valhalla::Options::route, use_living_streets_map, {"1", "2"}, c,
                                   {{"/costing_options/" + c + "/use_living_streets", "1"}}),
                  {"AB", "Bb", "be", "eE", "EF"});
}

TEST_F(LivingStreetTest, test_avoid_living_streets) {
  // avoid living_streets
  for (const auto& c : costing)
    validate_path(gurka::do_action(valhalla::Options::route, use_living_streets_map, {"1", "2"}, c,
                                   {{"/costing_options/" + c + "/use_living_streets", "0"}}),
                  {"AB", "BC", "CD", "DE", "EF"});
}

TEST_F(LivingStreetTest, test_use_living_streets_if_no_other_roads) {
  // use living_street if it's needed to complete the route
  for (const auto& c : costing) {
    validate_path(gurka::do_action(valhalla::Options::route, use_living_streets_map, {"D", "3"}, c,
                                   {{"/costing_options/" + c + "/use_living_streets", "0"}}),
                  {"DE", "EF", "FG"});
  }
}

TEST(LivingStreetStandaloneTest, test_living_streets_cheapest_route) {
  const std::string ascii_map = R"(
    A-1----B--------E-----2-F
           |        |       |
           |        |       |
           |        |       3
           |        |       |
           C--------D       G
                     \
                      I
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "living_street"}}}, {"BE", {{"highway", "living_street"}}},
      {"EF", {{"highway", "living_street"}}}, {"FG", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},   {"CD", {{"highway", "residential"}}},
      {"DE", {{"highway", "residential"}}},   {"DI", {{"highway", "secondary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_use_living_streets");

  // This test is to note the following corner case:
  // When both start and end locations are on living streets AND there is a relatively short path
  // between them that takes only living_street edges it may turn out to be cheaper than
  // the one that leaves and enters living street edges later. This happens due to transition penalty
  // usage. However, living_streets_factor generally strives to discourage such cases.
  validate_path(gurka::do_action(valhalla::Options::route, map, {"1", "2"}, "auto",
                                 {{"/costing_options/auto/use_living_streets", "0"}}),
                {"AB", "BE", "EF"});
}
