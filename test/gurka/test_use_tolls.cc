
#include "baldr/graphconstants.h"
#include "gurka.h"
#include "test.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

class useTollTest : public ::testing::Test {
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

    const gurka::ways ways = {{"ABCD", {{"highway", "motorway"}, {"toll", "yes"}}},
                              {"AEGFD", {{"highway", "motorway"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/tollopt");
  }

  void doTests(const std::string& costing,
               const std::unordered_map<std::string, std::string>& options) {

    valhalla::Api default_route =
        gurka::do_action(valhalla::Options::route, map, {"A", "D"}, costing);

    valhalla::Api capped_route =
        gurka::do_action(valhalla::Options::route, map, {"A", "D"}, costing, options);

    gurka::assert::raw::expect_path(default_route, {"ABCD"});
    gurka::assert::raw::expect_feature(default_route, "toll", true);
    // initially use_toll = 0.5f
    // so toll_factor_ becomes 0 for both and shorter route taken

    gurka::assert::raw::expect_path(capped_route, {"AEGFD"});
    gurka::assert::raw::expect_feature(capped_route, "toll", false);
    // when use_toll = 0 then toll_factor_ = 8
    //
    // No toll : contribution in overall factor is still 0
    // With toll : It makes a significant contribution to overall factor
    //
    // hence the longer residential route is taken due to lower costing.
  }
};

gurka::map useTollTest::map = {};

TEST_F(useTollTest, Autocost) {
  doTests("auto", {{"/costing_options/auto/use_tolls", "0"}});
}

TEST_F(useTollTest, Buscost) {
  doTests("bus", {{"/costing_options/bus/use_tolls", "0"}});
}

TEST_F(useTollTest, Motorcyclecost) {
  doTests("motorcycle", {{"/costing_options/motorcycle/use_tolls", "0"}});
}

TEST_F(useTollTest, Truckcost) {
  doTests("truck", {{"/costing_options/truck/use_tolls", "0"}});
}
