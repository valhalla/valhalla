#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.data_processing.use_admin_db", "true"}};

class NodeType : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
                          A
                          |
                          |     
                          B----E
                          |
                          |
                          C
                          |
                          |
                          D----F   
  )";

    const gurka::ways ways = {{"AB", {{"highway", "motorway"}}},
                              {"BC", {{"highway", "motorway"}}},
                              {"CD", {{"highway", "motorway"}}},
                              {"BE", {{"highway", "motorway"}}},
                              {"DF", {{"highway", "motorway"}}}};

    const gurka::nodes nodes = {{"B", {{"barrier", "toll_booth"}}},
                                {"D", {{"highway", "toll_gantry"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_node_type", build_config);
  }
};
gurka::map NodeType::map = {};

/*************************************************************/

TEST_F(NodeType, Toll) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");

  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(1).type(), TripLeg::Node::Type::TripLeg_Node_Type_kTollBooth); // AE

  result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");
  leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(3).type(), TripLeg::Node::Type::TripLeg_Node_Type_kTollGantry); // AF
}

TEST_F(NodeType, test_toll_response) {
  std::string locations = R"({"lon":)" + std::to_string(map.nodes["A"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["A"].lat()) + R"(},{"lon":)" +
                          std::to_string(map.nodes["E"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["E"].lat()) + "}";

  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);
  auto json = actor.route(R"({"costing":"auto","format":"osrm","locations":[)" + locations + R"(]})");

  // get the osrm json
  rapidjson::Document d;
  d.Parse(json);
  EXPECT_FALSE(d.HasParseError());

  for (const auto& route : d["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          if (intersection.HasMember("toll_collection")) {
            EXPECT_EQ(intersection.HasMember("toll_collection"), true);
            const auto& tc = intersection["toll_collection"];
            EXPECT_TRUE(tc.IsObject());
            EXPECT_TRUE(tc["type"].IsString());
            EXPECT_TRUE(tc["type"] == "toll_booth");
            EXPECT_FALSE(tc["type"] == "toll_gantry");
          }
        }
      }
    }
  }
}

TEST_F(NodeType, test_toll_response2) {
  std::string locations = R"({"lon":)" + std::to_string(map.nodes["C"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["C"].lat()) + R"(},{"lon":)" +
                          std::to_string(map.nodes["F"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["F"].lat()) + "}";

  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);
  auto json = actor.route(R"({"costing":"auto","format":"osrm","locations":[)" + locations + R"(]})");

  // get the osrm json
  rapidjson::Document d;
  d.Parse(json);
  EXPECT_FALSE(d.HasParseError());

  for (const auto& route : d["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          if (intersection.HasMember("toll_collection")) {
            EXPECT_EQ(intersection.HasMember("toll_collection"), true);
            const auto& tc = intersection["toll_collection"];
            EXPECT_TRUE(tc.IsObject());
            EXPECT_TRUE(tc["type"].IsString());
            EXPECT_FALSE(tc["type"] == "toll_booth");
            EXPECT_TRUE(tc["type"] == "toll_gantry");
          }
        }
      }
    }
  }
}
