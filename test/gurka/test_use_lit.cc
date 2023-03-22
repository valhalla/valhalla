#include "baldr/graphconstants.h"
#include "gurka.h"
#include "test.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

class UseLitTest : public ::testing::Test {
protected:
  static gurka::map speed_map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
      A------B
      |      |
      |      |
      |      |
      |      |
      |      C
      |      |
      E------D
    )";

    const gurka::ways ways = {{"AB", {{"highway", "residential"}}},
                              {"AE", {{"highway", "residential"}, {"lit", "yes"}}},
                              {"BC", {{"highway", "residential"}}},
                              {"CD", {{"highway", "residential"}}},
                              {"ED", {{"highway", "residential"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    speed_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/uselit");
  }

  inline float getDuration(const valhalla::Api& route) {
    return route.directions().routes(0).legs(0).summary().time();
  }
};

gurka::map UseLitTest::speed_map = {};

TEST_F(UseLitTest, LitHighway) {

  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/pedestrian/use_lit", "1"}};

  valhalla::Api default_route =
      gurka::do_action(valhalla::Options::route, speed_map, {"A", "C"}, "pedestrian");
  float default_time = getDuration(default_route);

  valhalla::Api lit_route =
      gurka::do_action(valhalla::Options::route, speed_map, {"A", "C"}, "pedestrian", options);

  gurka::assert::raw::expect_path(default_route, {"AB", "BC"});
  gurka::assert::raw::expect_path(lit_route, {"AE", "ED", "CD"});
}
