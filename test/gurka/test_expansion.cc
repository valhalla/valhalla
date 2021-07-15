#include "gurka.h"
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

  std::string build_prop_string(const std::vector<std::string> props) {
    std::stringstream req_props;
    req_props << "[";
    for (const auto& prop : props) {
      req_props << R"(")" + prop + R"(")";
    }
    req_props << "]";

    return R"("expansion_props":)" + req_props.str();
  }

  void
  check_props(const rapidjson::Value& doc, unsigned exp_feats, const std::vector<std::string> props) {

    auto feat = doc["features"][0].GetObject();
    ASSERT_EQ(feat["geometry"]["type"].GetString(), std::string("MultiLineString"));

    auto coords_size = feat["geometry"]["coordinates"].GetArray().Size();
    ASSERT_EQ(coords_size, exp_feats);
    for (const auto& prop : props) {
      ASSERT_EQ(feat["properties"][prop].GetArray().Size(), coords_size);
    }
  }
};

gurka::map ExpansionTest::expansion_map = {};

TEST_F(ExpansionTest, Isochrone) {
  // test Dijkstra expansion
  const auto& center_node = expansion_map.nodes["A"];
  const std::vector<std::string> props = {"distances", "durations", "costs"};
  const auto props_str = build_prop_string(props);

  // remember there's no way to get less than 10 km or mins expansion (buffer in isochrones)
  const std::string req = R"({"locations":[{"lat":)" + std::to_string(center_node.lat()) +
                          R"(,"lon":)" + std::to_string(center_node.lng()) +
                          R"(}],"costing":"auto","contours":[{"distance":0.05}],)" + props_str +
                          R"(})";
  std::string res;
  auto api = gurka::do_action(Options::expansion, expansion_map, req, nullptr, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  // 11 because there's a one-way
  check_props(res_doc, 11, props);
}

TEST_F(ExpansionTest, IsochroneNoOpposites) {
  // test Dijkstra expansion and skip collecting more expensive opposite edges
  const auto& center_node = expansion_map.nodes["A"];
  const std::vector<std::string> props = {"distances", "statuses", "edge_ids"};
  const auto props_str = build_prop_string(props);

  // remember there's no way to get less than 10 km or mins expansion (buffer in isochrones)
  const std::string req =
      R"({"skip_opposites":true,"locations":[{"lat":)" + std::to_string(center_node.lat()) +
      R"(,"lon":)" + std::to_string(center_node.lng()) +
      R"(}],"costing":"auto","contours":[{"distance":0.05}],)" + props_str + R"(})";
  std::string res;
  auto api = gurka::do_action(Options::expansion, expansion_map, req, nullptr, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  check_props(res_doc, 6, props);
}

TEST_P(ExpansionTest, IsochroneDefaultProps) {
  // test Dijkstra expansion with varying properties in the request
  const auto& center_node = expansion_map.nodes["A"];
  const std::vector<std::string> default_props = {"durations", "distances"};

  const std::string req = R"({"locations":[{"lat":)" + std::to_string(center_node.lat()) +
                          R"(,"lon":)" + std::to_string(center_node.lng()) +
                          R"(}],"costing":"auto","contours":[{"distance":0.05}]})";
  std::string res;
  auto api = gurka::do_action(Options::expansion, expansion_map, req, nullptr, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  const auto& feat = res_doc["features"][0];

  for (const auto& res_prop : default_props) {
    ASSERT_TRUE(feat["properties"].HasMember(res_prop));
  }

  check_props(res_doc, 11, default_props);
}

TEST_P(ExpansionTest, IsochroneVaryingProps) {
  // test Dijkstra expansion with varying properties in the request
  const auto& center_node = expansion_map.nodes["A"];

  auto props_string = build_prop_string(GetParam());

  const std::string req = R"({"locations":[{"lat":)" + std::to_string(center_node.lat()) +
                          R"(,"lon":)" + std::to_string(center_node.lng()) +
                          R"(}],"costing":"auto","contours":[{"distance":0.05}],)" +
                          (props_string.size() ? props_string : "") + R"(})";
  std::string res;
  auto api = gurka::do_action(Options::expansion, expansion_map, req, nullptr, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  const auto& feat = res_doc["features"][0];

  ASSERT_EQ(feat["properties"].GetArray().Size(), GetParam().size());
  for (const auto& res_prop : GetParam()) {
    ASSERT_TRUE(feat["properties"].HasMember(res_prop));
  }

  check_props(res_doc, 11, GetParam());
}

INSTANTIATE_TEST_SUITE_P(ExpandPropsTest,
                         ExpansionTest,
                         ::testing::Values(std::vector<std::string>{"statuses"},
                                           std::vector<std::string>{"distances", "durations"},
                                           std::vector<std::string>{"edge_ids", "costs"},
                                           std::vector<std::string>{}));

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

  check_props(res_doc, 23, {"duratons", "distances"});
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

  check_props(res_doc, 23, {"duratons", "distances"});
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
