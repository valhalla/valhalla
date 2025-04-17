#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

class CarRestrictionTest : public ::testing::TestWithParam<std::pair<std::string, std::string>> {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
      A---B---C---D
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "residential"}}},
        // all restrictions should be higher than our defaults, so we can actually see the impact of
        // any single one
        {"BC",
         {
             {"highway", "residential"},
             {"maxheight", "5"},
             {"maxlength", "25"},
             {"maxwidth", "3"},
             {"maxweight", "1"},
         }},
        {"CD", {{"highway", "residential"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/car_restrictions");
    std::cout << gurka::dump_geojson_graph(map) << std::endl;
  }
};

gurka::map CarRestrictionTest::map = {};

TEST_P(CarRestrictionTest, NotAllowed) {
  std::string option, v;
  std::tie(option, v) = GetParam();

  // "no path could be found for input" should be raised if we exceed this costing option
  try {
    EXPECT_THROW(gurka::do_action(Options::route, map, {"A", "D"}, "auto",
                                  {{"/costing_options/auto/" + option, v}}),
                 valhalla_exception_t);

  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_F(CarRestrictionTest, Allowed) {
  // without setting a costing option, we should get a path
  auto res = gurka::do_action(Options::route, map, {"A", "D"}, "auto");
  gurka::assert::raw::expect_path(res, {"AB", "BC", "CD"});
}

INSTANTIATE_TEST_SUITE_P(CarRestrictions,
                         CarRestrictionTest,
                         ::testing::Values(std::pair<std::string, std::string>{"height", "7.0"},
                                           std::pair<std::string, std::string>{"width", "7.0"},
                                           std::pair<std::string, std::string>{"length", "30.0"},
                                           std::pair<std::string, std::string>{"weight", "3.0"}));
