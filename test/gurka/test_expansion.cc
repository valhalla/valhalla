#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

class ExpansionTest : public ::testing::Test {
protected:
  static gurka::map test_map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
            B   F--G
            |   |  |
          E--A--C--H
            |
            D
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}, {"name", "AB"}}},
        {"AD", {{"highway", "primary"}, {"name", "AD"}}},
        {"AE", {{"highway", "primary"}, {"name", "AE"}}},
        {"AC", {{"highway", "primary"}, {"name", "AC"}}},
        {"CF", {{"highway", "primary"}, {"name", "CF"}}},
        {"FG", {{"highway", "primary"}, {"name", "FG"}}},
        {"GH", {{"highway", "primary"}, {"name", "GH"}}},
        {"HC", {{"highway", "primary"}, {"name", "HC"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    test_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/shortest");
  }
};

gurka::map ExpansionTest::test_map = {};

TEST_F(ExpansionTest, IsochroneSuccess) {
  // test Dijkstra expansion
  const auto& center_node = test_map.nodes["A"];
  const std::string req = R"({"locations":[{"lat":)" + std::to_string(center_node.lat()) +
                          R"(,"lon":)" + std::to_string(center_node.lng()) +
                          R"(}],"costing":"auto","contours":[{"distance":0.12}]})";
  std::string res;
  auto api = gurka::do_action(Options::expansion, test_map, req, nullptr, res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  std::cout << res << std::endl;

  // ASSERT_EQ(res_doc["features"][0]["geometry"]["type"].GetString(), "MultiLineString");
  // ASSERT_EQ(res_doc["features"][0]["geometry"]["coordinates"].GetArray().Size(), 10);

  // no action given in request: should default to isochrone action
  ASSERT_EQ(api.options().expansion_action(), Options::isochrone);
}
