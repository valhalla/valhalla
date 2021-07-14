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
            B  F--G
            |  |  |
         E--A--C--H
            |
            D
    )";

    const gurka::ways ways = {
        {"DAB", {{"highway", "primary"}}},
        {"EACH", {{"highway", "primary"}}},
        {"CFG", {{"highway", "primary"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    test_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/shortest");
  }
};

gurka::map ExpansionTest::test_map = {};

TEST_F(ExpansionTest, IsochroneSuccess) {
  // test Dijkstra expansion
  const auto& center_node = test_map.nodes["A"];
  // remember there's no way to get less than 10 km or mins expansion (buffer in isochrones)
  const std::string req = R"({"locations":[{"lat":)" + std::to_string(center_node.lat()) +
                          R"(,"lon":)" + std::to_string(center_node.lng()) +
                          R"(}],"costing":"auto","contours":[{"distance":0.05}]})";
  std::string res;
  auto api = gurka::do_action(Options::expansion, test_map, req, nullptr, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  ASSERT_EQ(res_doc["features"][0]["geometry"]["type"].GetString(), std::string("MultiLineString"));
  ASSERT_EQ(res_doc["features"][0]["geometry"]["coordinates"].GetArray().Size(), 12);
  ASSERT_TRUE(res_doc["features"][0]["properties"].HasMember("durations"));
  ASSERT_TRUE(res_doc["features"][0]["properties"].HasMember("distances"));
  ASSERT_TRUE(res_doc["features"][0]["properties"].HasMember("costs"));
}

TEST_F(ExpansionTest, RoutingSuccess) {
  // test AStar expansion
  const std::unordered_map<std::string, std::string> options = {{"/action", "route"}};
  std::vector<midgard::PointLL> lls;
  for (const auto& node_name : {"E", "H"}) {
    lls.push_back(test_map.nodes.at(node_name));
  }
  const auto req =
      gurka::detail::build_valhalla_request(std::string("locations"), lls, "auto", options);
  std::string res;
  auto api = gurka::do_action(Options::expansion, test_map, req, nullptr, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  ASSERT_EQ(res_doc["features"][0]["geometry"]["type"].GetString(), std::string("MultiLineString"));
  ASSERT_TRUE(res_doc["features"][0]["properties"].HasMember("statuses"));
}

TEST_F(ExpansionTest, UnsupportedActionFailure) {
  const auto& center_node = test_map.nodes["A"];
  const std::string req = R"({"action":"locate","locations":[{"lat":)" +
                          std::to_string(center_node.lat()) + R"(,"lon":)" +
                          std::to_string(center_node.lng()) +
                          R"(}],"costing":"auto","contours":[{"distance":0.05}]})";

  try {
    auto api = gurka::do_action(Options::expansion, test_map, req);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 144); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}
