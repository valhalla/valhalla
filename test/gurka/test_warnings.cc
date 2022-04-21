#include "gurka.h"
#include <gtest/gtest.h>
#include <vector>

using namespace valhalla;

std::vector<std::string> deprecated_costing_methods = {"auto_shorter", "hov", "auto_data_fix"};

class Warning : public ::testing::TestWithParam<valhalla::Options::Action> {
public:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(A----B----C)";

    const gurka::ways ways = {{"AB", {{"highway", "primary"}}}, {"BC", {{"highway", "primary"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/warning");
  }
};

gurka::map Warning::map = {};

/*************************************************************/

// test for route, locate, height, trace_route, trace_attributes, transit_available
TEST_P(Warning, valhalla__api_endpoints) {
  const auto& endpoint = GetParam();
  for (auto& costing : deprecated_costing_methods) {
    valhalla::Api result = gurka::do_action((valhalla::Options::Action)endpoint, map, {"A", "B", "C"},
                                            costing, {{"/best_paths", "2"}});
    ASSERT_TRUE(result.info().warnings_size() != 0);
    EXPECT_EQ(result.info().warnings_size(), 2);
  }
}

// test for isochrone endpoint
TEST_P(Warning, isochrone) {
  Api request;
  for (auto& costing : deprecated_costing_methods) {
    valhalla::Api result =
        gurka::do_action(valhalla::Options::isochrone, map, {"B"}, costing,
                         {{"/contours/0/time", "10"}, {"/denoise", "0"}, {"/generalize", "0"}});
    ASSERT_FALSE(result.info().warnings_size() == 0);
    EXPECT_EQ(result.info().warnings_size(), 1);
  }
}

// test for matrix endpoint
TEST_P(Warning, matrix) {
  Api request;
  for (auto& costing : deprecated_costing_methods) {
    std::string request_str = R"({"sources":[{"lat":)" + std::to_string(map.nodes["A"].lat()) +
                              R"(,"lon":)" + std::to_string(map.nodes["A"].lng()) +
                              R"(}],"targets":[{"lat":)" + std::to_string(map.nodes["C"].lat()) +
                              R"(,"lon":)" + std::to_string(map.nodes["C"].lng()) +
                              R"(}],"costing":")" + costing + R"("})";
    ParseApi(request_str, Options::sources_to_targets, request);
    ASSERT_FALSE(request.info().warnings_size() == 0);
    EXPECT_EQ(request.info().warnings_size(), 1);
  }
}

/*
 * Endpoints defined in proto/options.proto
 * route = 1
 * locate = 2
 * trace_route = 6
 * trace_attributes = 7
 * transit_available = 9
 * height = 8
 */

INSTANTIATE_TEST_CASE_P(WarningsTests, Warning, ::testing::Values(1, 2, 6, 7, 8, 9));
