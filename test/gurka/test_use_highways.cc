
#include "baldr/graphconstants.h"
#include "gurka.h"
#include "test.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

class useHighwayTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
      B------C
      |      |
      |      |
      A      D
      |      |
      |      |
      E      F
        \  /
         G
    )";

    const gurka::ways ways = {{"ABCD", {{"highway", "motorway"}}},      // kHighwayFactor 1.0f
                              {"AEGFD", {{"highway", "residential"}}}}; // kHighwayFactor 0.0f

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/highwayopt");
  }

  void doTests(const std::string& costing,
               const std::unordered_map<std::string, std::string>& options) {

    valhalla::Api default_route =
        gurka::do_action(valhalla::Options::route, map, {"A", "D"}, costing);

    valhalla::Api capped_route =
        gurka::do_action(valhalla::Options::route, map, {"A", "D"}, costing, options);

    gurka::assert::raw::expect_path(default_route, {"ABCD"});
    // initially use_highway = 0.5f
    // so highway_factor_ becomes 0 for both and shorter route taken

    gurka::assert::raw::expect_path(capped_route, {"AEGFD"});
    // when use_highways = 0 then highway_factor_ = 8
    //
    // Residential : contribution in overall factor is still 0 (kHighwayFactor 0.0f)
    // Motorway : It makes a significant contribution to overall factor (kHighwayFactor 1.0f)
    //
    // hence the longer residential route is taken due to lower costing.
  }
};

gurka::map useHighwayTest::map = {};

TEST_F(useHighwayTest, Autocost) {
  doTests("auto", {{"/costing_options/auto/use_highways", "0"}});
}

TEST_F(useHighwayTest, Buscost) {
  doTests("bus", {{"/costing_options/bus/use_highways", "0"}});
}

TEST_F(useHighwayTest, Motorcyclecost) {
  doTests("motorcycle", {{"/costing_options/motorcycle/use_highways", "0"}});
}

TEST_F(useHighwayTest, Truckcost) {
  doTests("truck", {{"/costing_options/truck/use_highways", "0"}});
}
