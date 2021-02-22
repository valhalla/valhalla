#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

/*************************************************************/
TEST(Standalone, avoid_service) {

  // Test to make sure bicycle routes do not use highway:service to cut
  // corners near intersections
  const std::string ascii_map = R"(
    A----B----C
         |    |
         D----E
              |
              F)";

  const gurka::ways ways = {{"AB", {{"highway", "secondary"}}}, {"BC", {{"highway", "secondary"}}},
                            {"CE", {{"highway", "tertiary"}}},  {"EF", {{"highway", "tertiary"}}},
                            {"BD", {{"highway", "service"}}},   {"DE", {{"highway", "service"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_avoid_service");

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "bicycle");

  // Make sure the path doesn't take service road "B-D-E"
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CE", "EF"});
}

void validate_path(const valhalla::Api& result, const std::vector<std::string>& expected_names) {
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, expected_names);
}

const std::vector<std::string>& costing = {"auto", "hov", "taxi", "bus", "truck", "motorcycle"};

class ServiceRoadsTest : public ::testing::Test {
protected:
  static gurka::map service_streets_map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
    A-1----B--------E-----2-F
           |        |       |
           |        |       |
           |        |       G
           |        |
           |        |
           C--------D
                     \
                      I
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "secondary"}}}, {"BE", {{"highway", "service"}}},
        {"EF", {{"highway", "secondary"}}}, {"FG", {{"highway", "secondary"}}},
        {"BC", {{"highway", "secondary"}}}, {"CD", {{"highway", "secondary"}}},
        {"DE", {{"highway", "secondary"}}}, {"DI", {{"highway", "residential"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    service_streets_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_service_streets");
  }
};

gurka::map ServiceRoadsTest::service_streets_map = {};

TEST_F(ServiceRoadsTest, test_default_value) {
  for (const auto& c : costing)
    validate_path(gurka::do_action(valhalla::Options::route, service_streets_map, {"1", "2"}, c),
                  {"AB", "BC", "CD", "DE", "EF"});
}

TEST_F(ServiceRoadsTest, test_use_service_roads) {
  for (const auto& c : costing)
    if (c == "truck")
      // truck is also impacted by 'low_class_penalty' option which is non-zero by default
      validate_path(gurka::do_action(valhalla::Options::route, service_streets_map, {"1", "2"}, c,
                                     {{"/costing_options/" + c + "/service_penalty", "0"},
                                      {"/costing_options/" + c + "/service_factor", "0.1"},
                                      {"/costing_options/" + c + "/low_class_penalty", "0"}}),
                    {"AB", "BE", "EF"});
    else
      validate_path(gurka::do_action(valhalla::Options::route, service_streets_map, {"1", "2"}, c,
                                     {{"/costing_options/" + c + "/service_penalty", "0"},
                                      {"/costing_options/" + c + "/service_factor", "0.1"}}),
                    {"AB", "BE", "EF"});
}

TEST_F(ServiceRoadsTest, test_avoid_service_roads) {
  for (const auto& c : costing)
    validate_path(gurka::do_action(valhalla::Options::route, service_streets_map, {"1", "2"}, c,
                                   {{"/costing_options/" + c + "/service_penalty", "300"},
                                    {"/costing_options/" + c + "/service_factor", "8"}}),
                  {"AB", "BC", "CD", "DE", "EF"});
}
