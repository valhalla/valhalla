#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{{}};

class TaggedValues : public ::testing::Test {
protected:
  static gurka::map map;
  static std::string ascii_map;
  static gurka::nodelayout layout;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    ascii_map = R"(
                          A
                          |
                          |
                          B         E----F
                          | \      /
                          |  C----D
                          G
                          |
                          |
                          H
    )";

    // TODO: does the graphparser make sure we dont ever try to set layer to 0 since its the default
    const gurka::ways ways = {{"AB", {{"highway", "motorway"}}},
                              {"BC",
                               {{"highway", "motorway"},
                                {"layer", "-3"},
                                {"tunnel", "yes"},
                                {"tunnel:name", "Fort McHenry Tunnel"}}},
                              {"CD",
                               {{"highway", "motorway"},
                                {"layer", "-2"},
                                {"tunnel", "yes"},
                                {"tunnel:name", "Fort McHenry Tunnel"}}},
                              {"DE",
                               {{"highway", "motorway"},
                                {"layer", "-1"},
                                {"tunnel", "yes"},
                                {"tunnel:name", "Fort McHenry Tunnel"}}},
                              {"EF", {{"highway", "motorway"}}},
                              {"BG", {{"highway", "motorway"}, {"layer", "2"}}},
                              {"GH", {{"highway", "motorway"}}}};

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_tagged_values", build_config);
  }
};
gurka::map TaggedValues::map = {};
std::string TaggedValues::ascii_map = {};
gurka::nodelayout TaggedValues::layout = {};

Api api;
rapidjson::Document d;

/*************************************************************/

TEST_F(TaggedValues, Layer) {
  baldr::GraphReader graphreader(map.config.get_child("mjolnir"));

  auto get_layer = [&](auto from, auto to) {
    auto edgeId = std::get<0>(gurka::findEdgeByNodes(graphreader, layout, from, to));
    return graphreader.edgeinfo(edgeId).layer();
  };
  EXPECT_EQ(get_layer("A", "B"), 0);
  EXPECT_EQ(get_layer("B", "C"), -3);
  EXPECT_EQ(get_layer("C", "D"), -2);
  EXPECT_EQ(get_layer("D", "E"), -1);
  EXPECT_EQ(get_layer("B", "G"), 2);
}

TEST_F(TaggedValues, Tunnel) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(0).edge().tunnel(), false);

  EXPECT_EQ(leg.node(1).edge().tunnel(), true);
  EXPECT_EQ(leg.node(1).edge().tagged_value().Get(0).type(), TaggedValue_Type_kLayer);
  EXPECT_EQ(leg.node(1).edge().tagged_value().Get(0).value(), std::string(1, char(-3)));
  EXPECT_EQ(leg.node(1).edge().tagged_value().Get(1).type(), TaggedValue_Type_kTunnel);
  EXPECT_EQ(leg.node(1).edge().tagged_value().Get(1).value(), "Fort McHenry Tunnel");

  EXPECT_EQ(leg.node(2).edge().tunnel(), true);
  EXPECT_EQ(leg.node(2).edge().tagged_value().Get(0).type(), TaggedValue_Type_kLayer);
  EXPECT_EQ(leg.node(2).edge().tagged_value().Get(0).value(), std::string(1, char(-2)));
  EXPECT_EQ(leg.node(2).edge().tagged_value().Get(1).type(), TaggedValue_Type_kTunnel);
  EXPECT_EQ(leg.node(2).edge().tagged_value().Get(1).value(), "Fort McHenry Tunnel");

  EXPECT_EQ(leg.node(3).edge().tunnel(), true);
  EXPECT_EQ(leg.node(3).edge().tagged_value().Get(0).type(), TaggedValue_Type_kLayer);
  EXPECT_EQ(leg.node(3).edge().tagged_value().Get(0).value(), std::string(1, char(-1)));
  EXPECT_TRUE(leg.node(3).edge().tagged_value().Get(1).type() == TaggedValue_Type_kTunnel);
  EXPECT_TRUE(leg.node(3).edge().tagged_value().Get(1).value() == "Fort McHenry Tunnel");

  EXPECT_EQ(leg.node(4).edge().tunnel(), false);

  EXPECT_EQ(leg.node(5).edge().tunnel(), false);
}

TEST_F(TaggedValues, NoTunnel) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");

  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(0).edge().tunnel(), false);
  EXPECT_EQ(leg.node(1).edge().tunnel(), false);
  EXPECT_EQ(leg.node(2).edge().tunnel(), false);
}

TEST_F(TaggedValues, test_taking_tunnel) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");
  rapidjson::Document d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  auto steps = d["routes"][0]["legs"][0]["steps"].GetArray();
  EXPECT_EQ(steps.Size(), 6);

  int step_index = 0;
  // First step AB has no tunnels
  for (const auto& intersection : steps[step_index]["intersections"].GetArray()) {
    EXPECT_FALSE(intersection.HasMember("tunnel_name"));
    EXPECT_STREQ(intersection["classes"][0].GetString(), "motorway");
  }

  // Second step BC is tunnel
  step_index++;
  for (const auto& intersection : steps[step_index]["intersections"].GetArray()) {
    EXPECT_TRUE(intersection.HasMember("tunnel_name"));
    EXPECT_STREQ(intersection["tunnel_name"].GetString(), "Fort McHenry Tunnel");
    EXPECT_STREQ(intersection["classes"][0].GetString(), "tunnel");
    EXPECT_STREQ(intersection["classes"][1].GetString(), "motorway");
  }

  // Third step CD is tunnel
  step_index++;
  for (const auto& intersection : steps[step_index]["intersections"].GetArray()) {
    EXPECT_TRUE(intersection.HasMember("tunnel_name"));
    EXPECT_STREQ(intersection["tunnel_name"].GetString(), "Fort McHenry Tunnel");
    EXPECT_STREQ(intersection["classes"][0].GetString(), "tunnel");
    EXPECT_STREQ(intersection["classes"][1].GetString(), "motorway");
  }

  // Fourth step DE is tunnel
  step_index++;
  for (const auto& intersection : steps[step_index]["intersections"].GetArray()) {
    EXPECT_TRUE(intersection.HasMember("tunnel_name"));
    EXPECT_STREQ(intersection["tunnel_name"].GetString(), "Fort McHenry Tunnel");
    EXPECT_STREQ(intersection["classes"][0].GetString(), "tunnel");
    EXPECT_STREQ(intersection["classes"][1].GetString(), "motorway");
  }

  // Fifth step EF is not tunnel
  step_index++;
  for (const auto& intersection : steps[step_index]["intersections"].GetArray()) {
    EXPECT_FALSE(intersection.HasMember("tunnel_name"));
    EXPECT_STREQ(intersection["classes"][0].GetString(), "motorway");
  }

  // Sixth step is arrival
  step_index++;
  for (const auto& intersection : steps[step_index]["intersections"].GetArray()) {
    EXPECT_FALSE(intersection.HasMember("tunnel_name"));
  }
}

TEST_F(TaggedValues, test_bypass_tunnel) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");
  rapidjson::Document d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  for (const auto& route : d["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          EXPECT_FALSE(intersection.HasMember("tunnel_name"));
        }
      }
    }
  }
}
