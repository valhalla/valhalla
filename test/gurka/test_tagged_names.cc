#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{{}};

class TaggedNames : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
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

    const gurka::ways ways =
        {{"AB", {{"highway", "motorway"}}},
         {"BC", {{"highway", "motorway"}, {"tunnel:name", "Fort McHenry Tunnel"}}},
         {"CD", {{"highway", "motorway"}, {"tunnel:name", "Fort McHenry Tunnel"}}},
         {"DE", {{"highway", "motorway"}, {"tunnel:name", "Fort McHenry Tunnel"}}},
         {"EF", {{"highway", "motorway"}}},
         {"BG", {{"highway", "motorway"}}},
         {"GH", {{"highway", "motorway"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_tagged_names", build_config);
  }
};
gurka::map TaggedNames::map = {};
Api api;
rapidjson::Document d;

/*************************************************************/

TEST_F(TaggedNames, Tunnel) {
  auto result = gurka::route(map, "A", "F", "auto");

  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(0).edge().has_tunnel(), false);
  EXPECT_TRUE(leg.node(1).edge().tagged_name().Get(0).type() == TaggedName_Type_kTunnel);
  EXPECT_TRUE(leg.node(1).edge().tagged_name().Get(0).value() == "Fort McHenry Tunnel");
  EXPECT_TRUE(leg.node(2).edge().tagged_name().Get(0).type() == TaggedName_Type_kTunnel);
  EXPECT_TRUE(leg.node(2).edge().tagged_name().Get(0).value() == "Fort McHenry Tunnel");
  EXPECT_TRUE(leg.node(3).edge().tagged_name().Get(0).type() == TaggedName_Type_kTunnel);
  EXPECT_TRUE(leg.node(3).edge().tagged_name().Get(0).value() == "Fort McHenry Tunnel");
  EXPECT_EQ(leg.node(4).edge().has_tunnel(), false);
  EXPECT_EQ(leg.node(5).edge().has_tunnel(), false);
}

TEST_F(TaggedNames, NoTunnel) {
  auto result = gurka::route(map, "A", "H", "auto");

  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(0).edge().has_tunnel(), false);
  EXPECT_EQ(leg.node(1).edge().has_tunnel(), false);
  EXPECT_EQ(leg.node(2).edge().has_tunnel(), false);
}

/*TEST_F(TaggedNames, test_taking_tunnel) {
  std::string locations = R"({"lon":)" + std::to_string(map.nodes["A"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["A"].lat()) + R"(},{"lon":)" +
                          std::to_string(map.nodes["F"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["F"].lat()) + "}";

  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);
  auto result = gurka::route(map, "A", "F", "auto");
  rapidjson::Document response = gurka::convert_to_json(result, valhalla::Options_Format_osrm);
  auto intersections = response["routes"][0]["legs"][0]["steps"][0].GetArray();
  EXPECT_FALSE(intersections[0]["tunnel_name"].GetString() == "Fort McHenry Tunnel");
  EXPECT_EQ(intersections[0]["classes"][0].GetString(), std::string("motorway"));
  EXPECT_EQ(intersections[1]["tunnel_name"].GetString(), "Fort McHenry Tunnel");
  EXPECT_EQ(intersections[1]["classes"][0].GetString(), std::string("tunnel"));
  EXPECT_EQ(intersections[1]["classes"][1].GetString(), std::string("motorway"));
  EXPECT_EQ(intersections[2]["tunnel_name"].GetString(), "Fort McHenry Tunnel");
  EXPECT_EQ(intersections[2]["classes"][0].GetString(), std::string("tunnel"));
  EXPECT_EQ(intersections[2]["classes"][1].GetString(), std::string("motorway"));
  EXPECT_EQ(intersections[3]["tunnel_name"].GetString(), "Fort McHenry Tunnel");
  EXPECT_EQ(intersections[3]["classes"][0].GetString(), std::string("tunnel"));
  EXPECT_EQ(intersections[3]["classes"][1].GetString(), std::string("motorway"));
  EXPECT_EQ(intersections[4]["tunnel_name"].GetString(), "Fort McHenry Tunnel");
  EXPECT_EQ(intersections[4]["classes"][0].GetString(), std::string("tunnel"));
  EXPECT_EQ(intersections[4]["classes"][1].GetString(), std::string("motorway"));
  EXPECT_FALSE(intersections[5]["tunnel_name"].GetString() == "Fort McHenry Tunnel");
  EXPECT_EQ(intersections[5]["classes"][0].GetString(), std::string("motorway"));
}

TEST_F(TaggedNames, test_bypass_tunnel) {
  std::string locations = R"({"lon":)" + std::to_string(map.nodes["A"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["A"].lat()) + R"(},{"lon":)" +
                          std::to_string(map.nodes["H"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["H"].lat()) + "}";

  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);
  auto result = gurka::route(map, "A", "H", "auto");
  rapidjson::Document response = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  auto intersections = response["routes"][0]["legs"][0]["steps"][0].GetArray();
  EXPECT_EQ(intersections[0].HasMember("tunnel_name"), false);
}*/
