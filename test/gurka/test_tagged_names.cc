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
         {"BC", {{"highway", "motorway"}, {"tunnel", "yes"}, {"tunnel:name", "Fort McHenry Tunnel"}}},
         {"CD", {{"highway", "motorway"}, {"tunnel", "yes"}, {"tunnel:name", "Fort McHenry Tunnel"}}},
         {"DE", {{"highway", "motorway"}, {"tunnel", "yes"}, {"tunnel:name", "Fort McHenry Tunnel"}}},
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
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(0).edge().has_tunnel(), false);

  EXPECT_EQ(leg.node(1).edge().tunnel(), true);
  EXPECT_TRUE(leg.node(1).edge().tagged_name().Get(0).type() == TaggedName_Type_kTunnel);
  EXPECT_TRUE(leg.node(1).edge().tagged_name().Get(0).value() == "Fort McHenry Tunnel");

  EXPECT_EQ(leg.node(2).edge().tunnel(), true);
  EXPECT_TRUE(leg.node(2).edge().tagged_name().Get(0).type() == TaggedName_Type_kTunnel);
  EXPECT_TRUE(leg.node(2).edge().tagged_name().Get(0).value() == "Fort McHenry Tunnel");

  EXPECT_EQ(leg.node(3).edge().tunnel(), true);
  EXPECT_TRUE(leg.node(3).edge().tagged_name().Get(0).type() == TaggedName_Type_kTunnel);
  EXPECT_TRUE(leg.node(3).edge().tagged_name().Get(0).value() == "Fort McHenry Tunnel");

  EXPECT_EQ(leg.node(4).edge().has_tunnel(), false);

  EXPECT_EQ(leg.node(5).edge().has_tunnel(), false);
}

TEST_F(TaggedNames, NoTunnel) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");

  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(0).edge().has_tunnel(), false);
  EXPECT_EQ(leg.node(1).edge().has_tunnel(), false);
  EXPECT_EQ(leg.node(2).edge().has_tunnel(), false);
}

TEST_F(TaggedNames, test_taking_tunnel) {
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

TEST_F(TaggedNames, test_bypass_tunnel) {
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
