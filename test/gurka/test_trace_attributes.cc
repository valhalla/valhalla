#include "gurka.h"
#include "midgard/encoded.h"
#include "midgard/util.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;

/*************************************************************/
TEST(Standalone, SacScaleAttributes) {

  const std::string ascii_map = R"(
      1
    A---2B-3-4C
              |
              |5
              D
         )";

  const gurka::ways ways = {{"AB", {{"highway", "track"}, {"sac_scale", "hiking"}}},
                            {"BC", {{"highway", "track"}, {"sac_scale", "alpine_hiking"}}},
                            {"CD", {{"highway", "track"}}}};

  const double gridsize = 10;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/sac_scale_attributes");

  std::string trace_json;
  auto api =
      gurka::do_action(valhalla::Options::trace_attributes, map, {"1", "2", "3", "4", "5"},
                       "pedestrian", {{"/costing_options/pedestrian/max_hiking_difficulty", "5"}}, {},
                       &trace_json, "via");

  rapidjson::Document result;
  result.Parse(trace_json.c_str());

  auto edges = result["edges"].GetArray();
  ASSERT_EQ(edges.Size(), 3);

  EXPECT_TRUE(edges[0].HasMember("sac_scale"));
  EXPECT_EQ(edges[0]["sac_scale"].GetInt(), 1);
  EXPECT_TRUE(edges[1].HasMember("sac_scale"));
  EXPECT_EQ(edges[1]["sac_scale"].GetInt(), 4);
  EXPECT_TRUE(edges[2].HasMember("sac_scale"));
  EXPECT_EQ(edges[2]["sac_scale"].GetInt(), 0);
}

TEST(Standalone, ShoulderAttributes) {

  const std::string ascii_map = R"(
      1
    A---2B-3-4C)";

  const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"shoulder", "both"}}},
                            {"BC", {{"highway", "primary"}}}};

  const double gridsize = 10;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/shoulder_attributes");

  std::string trace_json;
  auto api = gurka::do_action(valhalla::Options::trace_attributes, map, {"1", "2", "3", "4"},
                              "bicycle", {}, {}, &trace_json, "via");

  rapidjson::Document result;
  result.Parse(trace_json.c_str());

  auto edges = result["edges"].GetArray();
  ASSERT_EQ(edges.Size(), 2);
  EXPECT_TRUE(edges[0].HasMember("shoulder"));
  EXPECT_TRUE(edges[0]["shoulder"].GetBool());
  EXPECT_TRUE(edges[1].HasMember("shoulder"));
  EXPECT_FALSE(edges[1]["shoulder"].GetBool());
}

TEST(Standalone, InterpolatedPoints) {
  const std::string ascii_map = R"(
         3
    A--12B4--56C)";

  const gurka::ways ways = {{"AB", {{"highway", "primary"}}}, {"BC", {{"highway", "secondary"}}}};

  const double gridsize = 2;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/shoulder_attributes");

  std::string trace_json;
  auto api = gurka::do_action(valhalla::Options::trace_attributes, map,
                              {"1", "2", "3", "4", "5", "6"}, "bicycle", {}, {}, &trace_json, "via");

  // confirm one of the interpolated points has the right edge index
  rapidjson::Document result_doc;
  result_doc.Parse(trace_json);
  ASSERT_EQ(result_doc["matched_points"].GetArray().Size(), 6);
  ASSERT_EQ(result_doc["edges"].GetArray().Size(), 2);

  // we have all the right points set as interpolated & matched
  const std::unordered_map<std::string, std::vector<int>> wp_pairs{{"matched", {0, 4, 5}},
                                                                   {"interpolated", {1, 2, 3}}};
  for (const auto& wp_pair : wp_pairs) {
    for (const auto& wp : wp_pair.second) {
      ASSERT_EQ(static_cast<std::string>(result_doc["matched_points"][wp]["type"].GetString()),
                wp_pair.first);
    }
  }

  // make sure the relation of points to edge is correct
  ASSERT_EQ(result_doc["matched_points"][0]["edge_index"].GetInt(), 0);
  ASSERT_EQ(result_doc["matched_points"][1]["edge_index"].GetInt(), 0);

  // since WP 3 projects on the last edge, it should have distance_along_edge = 0
  ASSERT_EQ(result_doc["matched_points"][2]["edge_index"].GetInt(), 1);
  ASSERT_EQ(result_doc["matched_points"][2]["distance_along_edge"].GetFloat(), 0.f);

  ASSERT_EQ(result_doc["matched_points"][3]["edge_index"].GetInt(), 1);
  ASSERT_EQ(result_doc["matched_points"][4]["edge_index"].GetInt(), 1);
  ASSERT_EQ(result_doc["matched_points"][5]["edge_index"].GetInt(), 1);
}

TEST(Standalone, RetrieveTrafficSignal) {
  const std::string ascii_map = R"(
    A---B---C
        |
        D
  )";

  const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                            {"BC", {{"highway", "primary"}}},
                            {"BD", {{"highway", "primary"}}}};

  const gurka::nodes nodes = {{"B", {{"highway", "traffic_signals"}}}};

  const double gridsize = 10;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/traffic_signal_attributes");

  std::string trace_json;
  auto api = gurka::do_action(valhalla::Options::trace_attributes, map, {"A", "B", "C"}, "auto", {},
                              {}, &trace_json, "via");

  rapidjson::Document result;
  result.Parse(trace_json.c_str());

  auto edges = result["edges"].GetArray();
  ASSERT_EQ(edges.Size(), 2);

  EXPECT_TRUE(edges[0]["end_node"].HasMember("traffic_signal"));
  EXPECT_TRUE(edges[0]["end_node"]["traffic_signal"].GetBool());

  EXPECT_TRUE(edges[1]["end_node"].HasMember("traffic_signal"));
  EXPECT_FALSE(edges[1]["end_node"]["traffic_signal"].GetBool());
}

TEST(Standalone, AdditionalSpeedAttributes) {
  const std::string ascii_map = R"(
    A---B---C---D---E---F---G
  )";

  // Simulate two edges: one with faded speed, one with non-faded speed
  gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"maxspeed", "60"}}},
      {"BC", {{"highway", "primary"}, {"maxspeed", "60"}}},
      {"CD", {{"highway", "primary"}, {"maxspeed", "60"}}},
      {"DE", {{"highway", "primary"}, {"maxspeed", "60"}}},
      {"EF", {{"highway", "primary"}, {"maxspeed", "60"}}},
      {"FG", {{"highway", "primary"}, {"maxspeed", "60"}}},
  };

  const double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/speed_attributes");
  map.config.put("mjolnir.traffic_extract", "test/data/speed_attributes/traffic.tar");

  // add live traffic
  test::build_live_traffic_data(map.config);
  test::customize_live_traffic_data(map.config, [&](baldr::GraphReader&, baldr::TrafficTile&, int,
                                                    valhalla::baldr::TrafficSpeed* traffic_speed) {
    traffic_speed->overall_encoded_speed = 2 >> 1;
    traffic_speed->encoded_speed1 = 2 >> 1;
    traffic_speed->breakpoint1 = 255;
  });
  test::customize_historical_traffic(map.config, [](DirectedEdge& e) {
    e.set_constrained_flow_speed(40);
    e.set_free_flow_speed(100);

    // speeds for every 5 min bucket of the week
    std::array<float, kBucketsPerWeek> historical;
    historical.fill(10);
    return historical;
  });

  std::string trace_json;
  auto api = gurka::do_action(valhalla::Options::trace_attributes, map, {"A", "B", "C", "D", "E", "F", "G"}, "auto",
                              {{"/date_time/type", "0"}, {"/date_time/value", "current"}}, {},
                              &trace_json, "via");
                              
  rapidjson::Document result;
  result.Parse(trace_json.c_str());

  auto edges = result["edges"].GetArray();
  ASSERT_EQ(edges.Size(), 6);

  // Check for kEdgeSpeedFaded (should be true for AB, false for BC)
  EXPECT_TRUE(edges[0]["speeds_non_faded"].HasMember("current"));
  EXPECT_EQ(edges[0]["speeds_non_faded"]["current"].GetInt(), 2);

  EXPECT_TRUE(edges[0]["speeds_non_faded"].HasMember("constrained"));
  EXPECT_EQ(edges[0]["speeds_non_faded"]["constrained"].GetInt(), 40);

  EXPECT_TRUE(edges[0]["speeds_non_faded"].HasMember("free"));
  EXPECT_EQ(edges[0]["speeds_non_faded"]["free"].GetInt(), 100);

  EXPECT_TRUE(edges[0]["speeds_non_faded"].HasMember("base"));
  EXPECT_EQ(edges[0]["speeds_non_faded"]["base"].GetInt(), 60);

  EXPECT_TRUE(edges[0]["speeds_non_faded"].HasMember("predicted"));
  EXPECT_EQ(edges[0]["speeds_non_faded"]["predicted"].GetInt(), 10);

  EXPECT_TRUE(edges[0]["speeds_faded"].HasMember("current"));
  EXPECT_EQ(edges[0]["speeds_faded"]["current"].GetInt(), 2);

  EXPECT_TRUE(edges[0]["speeds_faded"].HasMember("constrained"));
  EXPECT_EQ(edges[0]["speeds_faded"]["constrained"].GetInt(), 2);

  EXPECT_TRUE(edges[0]["speeds_faded"].HasMember("free"));
  EXPECT_EQ(edges[0]["speeds_faded"]["free"].GetInt(), 2);

  EXPECT_TRUE(edges[0]["speeds_faded"].HasMember("base"));
  EXPECT_EQ(edges[0]["speeds_faded"]["base"].GetInt(), 2);

  EXPECT_TRUE(edges[0]["speeds_faded"].HasMember("predicted"));
  EXPECT_EQ(edges[0]["speeds_faded"]["predicted"].GetInt(), 2);
}
