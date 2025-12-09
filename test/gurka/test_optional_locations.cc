

#include "gurka.h"

#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

void add_optional_locations(const gurka::nodelayout& layout,
                            const std::vector<std::string>& optional_locations,
                            std::unordered_map<std::string, std::string>& request_params) {
  size_t i = 0;
  for (const auto& node : optional_locations) {
    auto pt = layout.at(node);
    request_params["/optional_locations/" + std::to_string(i) + "/lat"] = std::to_string(pt.lat());
    request_params["/optional_locations/" + std::to_string(i) + "/lon"] = std::to_string(pt.lng());
    ++i;
  }
}

class OptionalLocationsTestBase : public ::testing::Test {
protected:
  static gurka::map map;
  static gurka::nodelayout layout;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
    A---x-B------y-C-------D------E
                              z
  )";

    const gurka::ways ways = {
        {"AB", {{"highway", "corridor"}}},
        {"BC", {{"highway", "corridor"}}},
        {"CD", {{"highway", "corridor"}}},
        {"DE", {{"highway", "corridor"}}},
    };

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/optional_locations", {});
  }
};
gurka::map OptionalLocationsTestBase::map = {};
gurka::nodelayout OptionalLocationsTestBase::layout = {};

TEST_F(OptionalLocationsTestBase, Simple) {
  std::unordered_map<std::string, std::string> params{};
  add_optional_locations(layout, {"y"}, params);

  // base case: no opptional locations
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "pedestrian", {});
  EXPECT_EQ(result.trip().routes(0).legs_size(), 1);

  // now pass along y, expect to get 2 legs
  result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "pedestrian", params);
  EXPECT_EQ(result.trip().routes(0).legs_size(), 2);
}
