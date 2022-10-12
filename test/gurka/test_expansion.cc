#include "gurka.h"
#include <boost/algorithm/string/join.hpp>
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

class ExpansionTest : public ::testing::TestWithParam<std::vector<std::string>> {
protected:
  static gurka::map expansion_map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    // 5 birectional edges, 1 oneway = 11 directed edges
    const std::string ascii_map = R"(
            B  F--G
            |  |
         E--A--C--H
            |
            D
    )";

    const gurka::ways ways = {
        {"DAB", {{"highway", "primary"}}},
        {"EACH", {{"highway", "primary"}}},
        {"CFG", {{"highway", "primary"}, {"oneway", "yes"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    expansion_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/expansion");
  }

  void check_result(const std::string& action,
                    const std::vector<std::string>& waypoints,
                    bool skip_opps,
                    unsigned exp_feats,
                    const std::vector<std::string>& props = {}) {

    std::unordered_map<std::string, std::string> options = {{"/skip_opposites",
                                                             skip_opps ? "1" : "0"},
                                                            {"/action", action}};
    for (uint8_t i = 0; i < props.size(); i++) {
      options.insert({{"/expansion_properties/" + std::to_string(i), props[i]}});
    }
    if (action == "isochrone") {
      options.insert({{"/contours/0/time", "10"}, {"/contours/1/time", "20"}});
    }

    // get the response
    std::string res;
    auto api =
        gurka::do_action(Options::expansion, expansion_map, waypoints, "auto", options, {}, &res);

    // get the MultiLineString feature
    rapidjson::Document res_doc;
    res_doc.Parse(res.c_str());
    auto feat = res_doc["features"][0].GetObject();
    ASSERT_EQ(feat["geometry"]["type"].GetString(), std::string("MultiLineString"));

    auto coords_size = feat["geometry"]["coordinates"].GetArray().Size();
    ASSERT_EQ(coords_size, exp_feats);

    for (const auto& prop : props) {
      ASSERT_EQ(feat["properties"][prop].GetArray().Size(), coords_size);
    }
  }
};

gurka::map ExpansionTest::expansion_map = {};

// TODO: tons of " C++ exception with description "IsObject()" thrown in the test body."

TEST_P(ExpansionTest, Isochrone) {
  // test Dijkstra expansion
  // 11 because there's a one-way
  check_result("isochrone", {"A"}, false, 11, GetParam());
}

TEST_P(ExpansionTest, IsochroneNoOpposites) {
  // test Dijkstra expansion and skip collecting more expensive opposite edges
  check_result("isochrone", {"A"}, true, 6, GetParam());
}

TEST_P(ExpansionTest, Routing) {
  // test AStar expansion
  check_result("route", {"E", "H"}, false, 23, GetParam());
}

TEST_P(ExpansionTest, RoutingNoOpposites) {
  // test AStar expansion and no opposite edges
  check_result("route", {"E", "H"}, true, 16, GetParam());
}

TEST_F(ExpansionTest, UnsupportedAction) {
  try {
    check_result("status", {"E", "H"}, true, 16);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 144); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_F(ExpansionTest, UnsupportedPropType) {
  try {
    check_result("route", {"E", "H"}, true, 16, {"foo", "bar"});
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 168); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_F(ExpansionTest, NoAction) {
  const auto& center_node = expansion_map.nodes["A"];
  const std::string req = R"({"locations":[{"lat":)" + std::to_string(center_node.lat()) +
                          R"(,"lon":)" + std::to_string(center_node.lng()) +
                          R"(}],"costing":"auto","contours":[{"distance":0.05}]})";

  try {
    auto api = gurka::do_action(Options::expansion, expansion_map, req);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 115); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

INSTANTIATE_TEST_SUITE_P(ExpandPropsTest,
                         ExpansionTest,
                         ::testing::Values(std::vector<std::string>{"statuses"},
                                           std::vector<std::string>{"distances", "durations"},
                                           std::vector<std::string>{"edge_ids", "costs"},
                                           std::vector<std::string>{}));
