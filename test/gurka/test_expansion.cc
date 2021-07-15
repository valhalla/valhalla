#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

class ExpansionTest : public ::testing::Test {
protected:
  static gurka::map expansion_map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    // 5 birectional edges, 1 oneway = 11 directed edges
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
        {"CFG", {{"highway", "primary"}, {"oneway", "yes"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    expansion_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/expansion");
  }

  void
  check_props(const rapidjson::Value& doc, unsigned exp_feats, const std::vector<std::string> props) {

    auto feat = doc["features"][0].GetObject();
    ASSERT_EQ(feat["geometry"]["type"].GetString(), std::string("MultiLineString"));
    ASSERT_EQ(feat["geometry"]["coordinates"].GetArray().Size(), exp_feats);

    auto coords_size = feat["geometry"]["coordinates"].GetArray().Size();
    for (const auto& prop : props) {
      ASSERT_EQ(feat["properties"][prop].GetArray().Size(), coords_size);
    }
  }
};

gurka::map ExpansionTest::expansion_map = {};

TEST_F(ExpansionTest, Isochrone) {
  // test Dijkstra expansion
  const auto& center_node = expansion_map.nodes["A"];
  // remember there's no way to get less than 10 km or mins expansion (buffer in isochrones)
  const std::string req = R"({"locations":[{"lat":)" + std::to_string(center_node.lat()) +
                          R"(,"lon":)" + std::to_string(center_node.lng()) +
                          R"(}],"costing":"auto","contours":[{"distance":0.05}]})";
  std::string res;
  auto api = gurka::do_action(Options::expansion, expansion_map, req, nullptr, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  // 11 because there's a one-way
  check_props(res_doc, 11, {"distances", "durations", "costs"});
}

TEST_F(ExpansionTest, IsochroneNoOpposites) {
  // test Dijkstra expansion and skip collecting more expensive opposite edges
  const auto& center_node = expansion_map.nodes["A"];
  // remember there's no way to get less than 10 km or mins expansion (buffer in isochrones)
  const std::string req = R"({"skip_opposites":true,"locations":[{"lat":)" +
                          std::to_string(center_node.lat()) + R"(,"lon":)" +
                          std::to_string(center_node.lng()) +
                          R"(}],"costing":"auto","contours":[{"distance":0.05}]})";
  std::string res;
  auto api = gurka::do_action(Options::expansion, expansion_map, req, nullptr, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  check_props(res_doc, 6, {"distances", "durations", "costs"});
}

TEST_F(ExpansionTest, Routing) {
  // test AStar expansion
  const std::unordered_map<std::string, std::string> options = {{"/action", "route"}};
  std::vector<midgard::PointLL> lls;
  for (const auto& node_name : {"E", "H"}) {
    lls.push_back(expansion_map.nodes.at(node_name));
  }
  const auto req =
      gurka::detail::build_valhalla_request(std::string("locations"), lls, "auto", options);
  std::string res;
  auto api = gurka::do_action(Options::expansion, expansion_map, req, nullptr, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  check_props(res_doc, 23, {"statuses", "edge_ids"});
}

TEST_F(ExpansionTest, RoutingNoOpposites) {
  // test AStar expansion
  const std::unordered_map<std::string, std::string> options = {{"/action", "route"},
                                                                {"/skip_opposites", "true"}};
  std::vector<midgard::PointLL> lls;
  for (const auto& node_name : {"E", "H"}) {
    lls.push_back(expansion_map.nodes.at(node_name));
  }
  const auto req =
      gurka::detail::build_valhalla_request(std::string("locations"), lls, "auto", options);
  std::string res;
  auto api = gurka::do_action(Options::expansion, expansion_map, req, nullptr, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  check_props(res_doc, 23, {"statuses", "edge_ids"});
}

TEST_F(ExpansionTest, UnsupportedAction) {
  const auto& center_node = expansion_map.nodes["A"];
  const std::string req = R"({"action":"locate","locations":[{"lat":)" +
                          std::to_string(center_node.lat()) + R"(,"lon":)" +
                          std::to_string(center_node.lng()) +
                          R"(}],"costing":"auto","contours":[{"distance":0.05}]})";

  try {
    auto api = gurka::do_action(Options::expansion, expansion_map, req);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 144); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}
