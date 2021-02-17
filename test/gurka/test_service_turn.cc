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

const std::vector<std::string>& costing = {"auto", "hov", "taxi", "bus", "truck"};

TEST(ServiceStandalone, test_avoid_service_default) {
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
      {"AB", {{"highway", "secondary"}}}, {"BE", {{"highway", "service"}}},
      {"EF", {{"highway", "secondary"}}}, {"FG", {{"highway", "secondary"}}},
      {"BC", {{"highway", "secondary"}}}, {"CD", {{"highway", "secondary"}}},
      {"DE", {{"highway", "secondary"}}}, {"DI", {{"highway", "residential"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_avois_service_default");

  for (const auto& c : costing)
    validate_path(gurka::do_action(valhalla::Options::route, map, {"1", "2"}, c),
                  {"AB", "BC", "CD", "DE", "EF"});
}
