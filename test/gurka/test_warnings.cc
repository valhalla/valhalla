#include "gurka.h"
#include <gtest/gtest.h>
#include <vector>

using namespace valhalla;

std::vector<std::string> deprecated_costing_methods = {"auto_shorter", "hov", "auto_data_fix"};

class Warning : public ::testing::Test {
protected:
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
// test for route endpoint
TEST_F(Warning, routes) {
  for (auto& costing : deprecated_costing_methods) {
    valhalla::Api result =
        gurka::do_action(valhalla::Options::route, map, {"A", "C"}, costing, {{"/best_paths", "2"}});
    ASSERT_TRUE(result.info().warnings_size() != 0);
    EXPECT_EQ(result.info().warnings_size(), 2);
  }
}

// test for matrix endpoint
TEST_F(Warning, matrix) {
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

// test for locate endpoint
TEST_F(Warning, locate) {
  for (auto& costing : deprecated_costing_methods) {
    valhalla::Api result = gurka::do_action(valhalla::Options::locate, map, {"B"}, costing);
    ASSERT_TRUE(result.info().warnings_size() != 0);
    EXPECT_EQ(result.info().warnings_size(), 1);
  }
}

// test for isochrone endpoint
TEST_F(Warning, isochrone) {
  for (auto& costing : deprecated_costing_methods) {
    valhalla::Api result =
        gurka::do_action(valhalla::Options::isochrone, map, {"B"}, costing,
                         {{"/contours/0/time", "10"}, {"/denoise", "0"}, {"/generalize", "0"}});
    ASSERT_FALSE(result.info().warnings_size() == 0);
    EXPECT_EQ(result.info().warnings_size(), 1);
  }
}

// test for transit_available endpoint
TEST_F(Warning, transit_available) {
  valhalla::Api result = gurka::do_action(valhalla::Options::transit_available, map, {"A"}, "",
                                          {{"/locations/0/radius", "5"}, {"/best_paths", "2"}});
  ASSERT_FALSE(result.info().warnings_size() == 0);
  EXPECT_EQ(result.info().warnings_size(), 1);
}

// test for height endpoint
TEST_F(Warning, height) {
  valhalla::Api result = gurka::do_action(valhalla::Options::height, map, {"A", "C"}, "",
                                          {{"/resample_distance", "15"}, {"/best_paths", "2"}});
  ASSERT_FALSE(result.info().warnings_size() == 0);
  EXPECT_EQ(result.info().warnings_size(), 1);
}

// test for map_matching endpoint
TEST_F(Warning, map_matching) {
  for (auto& costing : deprecated_costing_methods) {
    valhalla::Api trace_route_result =
        gurka::do_action(valhalla::Options::trace_route, map, {"A", "B", "C"}, costing);
    valhalla::Api trace_attributes_result =
        gurka::do_action(valhalla::Options::trace_attributes, map, {"A", "B", "C"}, costing);
    ASSERT_FALSE(trace_route_result.info().warnings_size() == 0);
    EXPECT_EQ(trace_route_result.info().warnings_size(), 1);
    ASSERT_TRUE(trace_attributes_result.info().warnings_size() != 0);
    EXPECT_EQ(trace_attributes_result.info().warnings_size(), 1);
  }
}
