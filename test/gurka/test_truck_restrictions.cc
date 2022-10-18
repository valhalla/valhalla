#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

class TruckRestrictionTest : public ::testing::TestWithParam<std::pair<std::string, std::string>> {
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
         {{"highway", "residential"},
          {"maxheight", "5"},
          {"maxlength", "25"},
          {"maxwidth", "3"},
          {"hazmat", "no"},
          {"maxaxles", "8"},
          {"maxaxleload", "10"}}},
        {"CD", {{"highway", "residential"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/truck_restrictions");
  }
};

gurka::map TruckRestrictionTest::map = {};

TEST_P(TruckRestrictionTest, NotAllowed) {
  std::string option, v;
  std::tie(option, v) = GetParam();

  // "no path could be found for input" should be raised if we exceed this costing option
  try {
    gurka::do_action(Options::route, map, {"A", "D"}, "truck",
                     {{"/costing_options/truck/" + option, v}});
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_F(TruckRestrictionTest, Allowed) {
  // without setting a costing option, we should get a path
  auto res = gurka::do_action(Options::route, map, {"A", "D"}, "truck");
  gurka::assert::raw::expect_path(res, {"AB", "BC", "CD"});
}

INSTANTIATE_TEST_SUITE_P(TruckRestrictions,
                         TruckRestrictionTest,
                         ::testing::Values(std::pair<std::string, std::string>{"height", "6"},
                                           std::pair<std::string, std::string>{"width", "4"},
                                           std::pair<std::string, std::string>{"length", "30"},
                                           std::pair<std::string, std::string>{"hazmat", "true"},
                                           std::pair<std::string, std::string>{"axle_load", "11"},
                                           std::pair<std::string, std::string>{"axle_count", "10"}));
