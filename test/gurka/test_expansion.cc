#include "gurka.h"
#include <boost/algorithm/string/join.hpp>
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

  valhalla::Api do_expansion_action(std::string* res,
                                    bool skip_opps,
                                    bool dedupe,
                                    std::string action,
                                    const std::vector<std::string>& props,
                                    const std::vector<std::string>& waypoints,
                                    const bool pbf) {
    std::unordered_map<std::string, std::string> options = {{"/skip_opposites",
                                                             skip_opps ? "1" : "0"},
                                                            {"/action", action},
                                                            {"/dedupe", dedupe ? "1" : "0"},
                                                            {"/format", pbf ? "pbf" : "json"}};
    for (uint8_t i = 0; i < props.size(); i++) {
      options.insert({{"/expansion_properties/" + std::to_string(i), props[i]}});
    }
    if (action == "isochrone") {
      options.insert({{"/contours/0/time", "10"}, {"/contours/1/time", "20"}});
    }

    // get the response
    valhalla::Api api;

    if (action == "sources_to_targets") {
      api = gurka::do_action(Options::expansion, expansion_map, {waypoints[0]}, {waypoints[1]},
                             "auto", options, {}, res);
    } else {
      api = gurka::do_action(Options::expansion, expansion_map, waypoints, "auto", options, {}, res);
    }

    return api;
  }
  void check_results(const std::string& action,
                     const std::vector<std::string>& waypoints,
                     bool skip_opps,
                     unsigned exp_feats,
                     const std::vector<std::string>& props = {},
                     bool dedupe = false) {
    check_result_json(action, waypoints, skip_opps, dedupe, exp_feats, props);
    check_result_pbf(action, waypoints, skip_opps, dedupe, exp_feats, props);
  }
  void check_result_pbf(const std::string& action,
                        const std::vector<std::string>& waypoints,
                        bool skip_opps,
                        bool dedupe,
                        unsigned exp_feats,
                        const std::vector<std::string>& props) {
    std::string res;
    auto api = do_expansion_action(&res, skip_opps, dedupe, action, props, waypoints, true);

    Api parsed_api;
    parsed_api.ParseFromString(res);

    ASSERT_EQ(parsed_api.expansion().geometries_size(), exp_feats);

    bool edge_status = false;
    bool distance = false;
    bool duration = false;
    bool pred_edge_id = false;
    bool edge_id = false;
    bool cost = false;
    bool expansion_type = false;

    const std::unordered_map<std::string, int> prop_count;
    for (const auto& prop : props) {
      if (prop == "edge_status") {
        edge_status = true;
      }
      if (prop == "distance") {
        distance = true;
      }
      if (prop == "duration") {
        duration = true;
      }
      if (prop == "pred_edge_id") {
        pred_edge_id = true;
      }
      if (prop == "edge_id") {
        edge_id = true;
      }
      if (prop == "cost") {
        cost = true;
      }
      if (prop == "expansion_type") {
        expansion_type = true;
      }
    }
    ASSERT_EQ(parsed_api.expansion().geometries_size(), exp_feats);
    ASSERT_EQ(parsed_api.expansion().edge_status_size(), edge_status ? exp_feats : 0);
    ASSERT_EQ(parsed_api.expansion().distances_size(), distance ? exp_feats : 0);
    ASSERT_EQ(parsed_api.expansion().durations_size(), duration ? exp_feats : 0);
    ASSERT_EQ(parsed_api.expansion().pred_edge_id_size(), pred_edge_id ? exp_feats : 0);
    ASSERT_EQ(parsed_api.expansion().edge_id_size(), edge_id ? exp_feats : 0);
    ASSERT_EQ(parsed_api.expansion().costs_size(), cost ? exp_feats : 0);
    ASSERT_EQ(parsed_api.expansion().expansion_type_size(), expansion_type ? exp_feats : 0);
  }
  void check_result_json(const std::string& action,
                         const std::vector<std::string>& waypoints,
                         bool skip_opps,
                         bool dedupe,
                         unsigned exp_feats,
                         const std::vector<std::string>& props) {

    std::string res;
    auto api = do_expansion_action(&res, skip_opps, dedupe, action, props, waypoints, false);
    // get the MultiLineString feature
    rapidjson::Document res_doc;
    res_doc.Parse(res.c_str());
    ASSERT_EQ(res_doc["features"].GetArray().Size(), exp_feats);
    auto feat = res_doc["features"][0].GetObject();
    ASSERT_EQ(feat["geometry"]["type"].GetString(), std::string("LineString"));
    ASSERT_TRUE(feat.HasMember("properties"));

    ASSERT_EQ(feat["properties"].MemberCount(), props.size());
    for (const auto& prop : props) {
      ASSERT_TRUE(feat["properties"].HasMember(prop));
    }
  }
};

gurka::map ExpansionTest::expansion_map = {};

// TODO: tons of " C++ exception with description "IsObject()" thrown in the test body."

TEST_P(ExpansionTest, Isochrone) {
  // test Dijkstra expansion
  // 11 because there's a one-way
  check_results("isochrone", {"A"}, false, 11, GetParam());
}

TEST_P(ExpansionTest, IsochroneNoOpposites) {
  // test Dijkstra expansion and skip collecting more expensive opposite edges
  check_results("isochrone", {"A"}, true, 6, GetParam());
}

TEST_P(ExpansionTest, Routing) {
  // test AStar expansion
  check_results("route", {"E", "H"}, false, 23, GetParam());
}

TEST_P(ExpansionTest, RoutingNoOpposites) {
  // test AStar expansion and no opposite edges
  check_results("route", {"E", "H"}, true, 16, GetParam());
}

TEST_P(ExpansionTest, Matrix) {
  check_results("sources_to_targets", {"E", "H"}, false, 48, GetParam());
}

TEST_P(ExpansionTest, MatrixNoOpposites) {
  check_results("sources_to_targets", {"E", "H"}, true, 23, GetParam());
}

TEST_P(ExpansionTest, IsochroneDedupe) {
  // test Dijkstra expansion
  // 11 because there's a one-way
  check_results("isochrone", {"A"}, false, 11, GetParam(), true);
}
TEST_P(ExpansionTest, IsochroneNoOppositesDedupe) {
  // test Dijkstra expansion and skip collecting more expensive opposite edges
  check_results("isochrone", {"A"}, true, 6, GetParam(), true);
}

TEST_P(ExpansionTest, RoutingDedupe) {
  // test AStar expansion
  check_results("route", {"E", "H"}, false, 7, GetParam(), true);
}

TEST_P(ExpansionTest, RoutingNoOppositesDedupe) {
  // test AStar expansion and no opposite edges
  check_results("route", {"E", "H"}, true, 5, GetParam(), true);
}

TEST_P(ExpansionTest, MatrixDedupe) {
  check_results("sources_to_targets", {"E", "H"}, false, 11, GetParam(), true);
}

TEST_P(ExpansionTest, MatrixNoOppositesDedupe) {
  check_results("sources_to_targets", {"E", "H"}, true, 6, GetParam(), true);
}

TEST_F(ExpansionTest, UnsupportedAction) {
  try {
    check_results("status", {"E", "H"}, true, 16);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 144); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_F(ExpansionTest, UnsupportedPropType) {
  try {
    check_results("route", {"E", "H"}, true, 16, {"foo", "bar"});
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

TEST_F(ExpansionTest, UnsupportedActionDedupe) {
  try {
    check_results("status", {"E", "H"}, true, 5, {}, true);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 144); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_F(ExpansionTest, UnsupportedPropTypeDedupe) {
  try {
    check_results("route", {"E", "H"}, true, 5, {"foo", "bar"}, true);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 168); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

INSTANTIATE_TEST_SUITE_P(ExpandPropsTest,
                         ExpansionTest,
                         ::testing::Values(std::vector<std::string>{"edge_status"},
                                           std::vector<std::string>{"distance", "duration",
                                                                    "pred_edge_id", "expansion_type"},
                                           std::vector<std::string>{"edge_id", "cost"},
                                           std::vector<std::string>{}));
