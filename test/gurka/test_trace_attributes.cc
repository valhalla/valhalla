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

TEST(Standalone, RetrieveNodeTrafficSignal) {
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
  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/traffic_signal_node_attributes");

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

TEST(Standalone, SpeedTypes) {
  const std::string ascii_map = R"(
    A---B---C
  )";

  gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}, {"maxspeed", "60"}}},
  };

  const double gridsize = 10;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/speed_types");

  std::string trace_json;
  auto api = gurka::do_action(valhalla::Options::trace_attributes, map, {"A", "B", "C"}, "auto", {},
                              {}, &trace_json, "via");

  rapidjson::Document result;
  result.Parse(trace_json.c_str());

  auto edges = result["edges"].GetArray();
  ASSERT_EQ(edges.Size(), 2);

  EXPECT_EQ(edges[0]["speed_type"].GetString(), to_string(baldr::SpeedType::kClassified));
  EXPECT_EQ(edges[1]["speed_type"].GetString(), to_string(baldr::SpeedType::kTagged));
}

TEST(Standalone, AdditionalSpeedAttributes) {
  // set all speeds in kph
  uint64_t current = 20;
  uint64_t constrained = 40;
  uint64_t free = 100;
  uint64_t predicted = 10;
  uint64_t base = 60;

  const std::string ascii_map = R"(
    A---B---C
  )";

  gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}, {"maxspeed", std::to_string(base)}}},
  };

  // gridsize 2500 = 10km length per edge
  const double gridsize = 2500;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/speed_attributes");
  map.config.put("mjolnir.traffic_extract", "test/data/speed_attributes/traffic.tar");

  // add live traffic
  test::build_live_traffic_data(map.config);
  test::customize_live_traffic_data(map.config, [&](baldr::GraphReader&, baldr::TrafficTile&, int,
                                                    valhalla::baldr::TrafficSpeed* traffic_speed) {
    traffic_speed->overall_encoded_speed = current >> 1;
    traffic_speed->encoded_speed1 = current >> 1;
    traffic_speed->breakpoint1 = 255;
  });
  // set all historical speed buckets to 10 to simulate uniform historical traffic speeds for testing.
  test::customize_historical_traffic(map.config, [&](DirectedEdge& e) {
    e.set_constrained_flow_speed(constrained);
    e.set_free_flow_speed(free);

    // speeds for every 5 min bucket of the week
    std::array<float, kBucketsPerWeek> historical;
    historical.fill(predicted);
    return historical;
  });

  std::string trace_json;
  auto api = gurka::do_action(valhalla::Options::trace_attributes, map, {"A", "B", "C"}, "auto",
                              {{"/shape_match", "edge_walk"},
                               {"/date_time/type", "0"},
                               {"/date_time/value", "current"},
                               {"/costing_options/auto/speed_types/0", "current"},
                               {"/costing_options/auto/speed_types/1", "predicted"},
                               {"/trace_options/breakage_distance", "10000"}},
                              {}, &trace_json, "via");
  rapidjson::Document result;
  result.Parse(trace_json.c_str());
  auto edges = result["edges"].GetArray();
  ASSERT_EQ(edges.Size(), 2);

  EXPECT_EQ(edges[0]["speeds_non_faded"]["current_flow"].GetInt(), current);
  EXPECT_EQ(edges[0]["speeds_non_faded"]["constrained_flow"].GetInt(), constrained);
  EXPECT_EQ(edges[0]["speeds_non_faded"]["free_flow"].GetInt(), free);
  EXPECT_EQ(edges[0]["speeds_non_faded"]["predicted_flow"].GetInt(), predicted);
  EXPECT_EQ(edges[0]["speeds_non_faded"]["no_flow"].GetInt(),
            75); // speed is 75 because its inferred by the primary road class

  EXPECT_EQ(edges[0]["speeds_faded"]["current_flow"].GetInt(), current);
  EXPECT_EQ(edges[0]["speeds_faded"]["constrained_flow"].GetInt(), current);
  EXPECT_EQ(edges[0]["speeds_faded"]["free_flow"].GetInt(), current);
  EXPECT_EQ(edges[0]["speeds_faded"]["predicted_flow"].GetInt(), current);
  EXPECT_EQ(edges[0]["speeds_faded"]["no_flow"].GetInt(), current);

  EXPECT_EQ(edges[1]["speeds_non_faded"]["current_flow"].GetInt(), current);
  EXPECT_EQ(edges[1]["speeds_non_faded"]["constrained_flow"].GetInt(), constrained);
  EXPECT_EQ(edges[1]["speeds_non_faded"]["free_flow"].GetInt(), free);
  EXPECT_EQ(edges[1]["speeds_non_faded"]["predicted_flow"].GetInt(), predicted);
  EXPECT_EQ(edges[1]["speeds_non_faded"]["no_flow"].GetInt(), base);

  // current_flow fades to predicted flow because its next up from the speed types in the request
  EXPECT_NEAR(edges[1]["speeds_faded"]["current_flow"].GetInt(), current * 0.5 + predicted * 0.5, 1);
  EXPECT_NEAR(edges[1]["speeds_faded"]["constrained_flow"].GetInt(),
              current * 0.5 + constrained * 0.5, 1);
  EXPECT_NEAR(edges[1]["speeds_faded"]["free_flow"].GetInt(), current * 0.5 + free * 0.5, 1);
  EXPECT_NEAR(edges[1]["speeds_faded"]["predicted_flow"].GetInt(), current * 0.5 + predicted * 0.5,
              1);
  EXPECT_NEAR(edges[1]["speeds_faded"]["no_flow"].GetInt(), current * 0.5 + base * 0.5, 1);

  api = gurka::do_action(valhalla::Options::trace_attributes, map, {"A", "B", "C"}, "auto",
                         {{"/shape_match", "edge_walk"},
                          {"/date_time/type", "1"},
                          {"/date_time/value", "2025-06-27T08:00"},
                          {"/trace_options/breakage_distance", "10000"}},
                         {}, &trace_json, "via");
  result.Parse(trace_json.c_str());
  edges = result["edges"].GetArray();

  for (rapidjson::SizeType i = 0; i < edges.Size(); i++) {
    EXPECT_FALSE(edges[i].HasMember("speeds_faded"));
    EXPECT_TRUE(edges[i]["speeds_non_faded"].HasMember("current_flow"));
    EXPECT_TRUE(edges[i]["speeds_non_faded"].HasMember("predicted_flow"));
    EXPECT_TRUE(edges[i]["speeds_non_faded"].HasMember("constrained_flow"));
    EXPECT_TRUE(edges[i]["speeds_non_faded"].HasMember("free_flow"));
    EXPECT_TRUE(edges[i]["speeds_non_faded"].HasMember("no_flow"));
  }

  api =
      gurka::do_action(valhalla::Options::trace_attributes, map, {"A", "B", "C"}, "auto",
                       {{"/shape_match", "edge_walk"}, {"/trace_options/breakage_distance", "10000"}},
                       {}, &trace_json, "via");
  result.Parse(trace_json.c_str());
  edges = result["edges"].GetArray();

  for (rapidjson::SizeType i = 0; i < edges.Size(); i++) {
    EXPECT_FALSE(edges[i].HasMember("speeds_faded"));
    EXPECT_FALSE(edges[i]["speeds_non_faded"].HasMember("predicted_flow"));
    EXPECT_TRUE(edges[i]["speeds_non_faded"].HasMember("current_flow"));
    EXPECT_TRUE(edges[i]["speeds_non_faded"].HasMember("constrained_flow"));
    EXPECT_TRUE(edges[i]["speeds_non_faded"].HasMember("free_flow"));
    EXPECT_TRUE(edges[i]["speeds_non_faded"].HasMember("no_flow"));
  }

  // reset historical traffic
  test::customize_historical_traffic(map.config, [&](DirectedEdge& e) {
    e.set_constrained_flow_speed(0);
    e.set_free_flow_speed(0);

    return std::nullopt;
  });
  // invalidate traffic speed
  test::customize_live_traffic_data(map.config, [&](baldr::GraphReader&, baldr::TrafficTile&, int,
                                                    valhalla::baldr::TrafficSpeed* traffic_speed) {
    traffic_speed->overall_encoded_speed = UNKNOWN_TRAFFIC_SPEED_RAW;
    traffic_speed->encoded_speed1 = UNKNOWN_TRAFFIC_SPEED_RAW;
    traffic_speed->breakpoint1 = 0;
  });

  // this request should not have current, predicted, constrained nor free flows
  api = gurka::do_action(valhalla::Options::trace_attributes, map, {"A", "B", "C"}, "auto",
                         {{"/shape_match", "edge_walk"},
                          {"/date_time/type", "0"},
                          {"/date_time/value", "current"},
                          {"/trace_options/breakage_distance", "10000"}},
                         {}, &trace_json, "via");

  result.Parse(trace_json.c_str());
  edges = result["edges"].GetArray();

  for (rapidjson::SizeType i = 0; i < edges.Size(); i++) {
    // no faded speeds because edges don't have traffic speed anymore
    EXPECT_FALSE(edges[i].HasMember("speeds_faded"));
    EXPECT_FALSE(edges[i]["speeds_non_faded"].HasMember("current_flow"));
    EXPECT_FALSE(edges[i]["speeds_non_faded"].HasMember("predicted_flow"));
    EXPECT_FALSE(edges[i]["speeds_non_faded"].HasMember("constrained_flow"));
    EXPECT_FALSE(edges[i]["speeds_non_faded"].HasMember("free_flow"));
  }
}

TEST(Standalone, RetrieveEdgeTrafficSignal) {
  const std::string ascii_map = R"(
    A--B--C--D
  )";

  const gurka::ways ways = {{"ABC", {{"highway", "primary"}}}, {"CD", {{"highway", "primary"}}}};

  const gurka::nodes nodes = {
      {"B", {{"highway", "traffic_signals"}, {"traffic_signals:direction", "forward"}}}};

  const double gridsize = 10;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/traffic_signal_edge_attributes");

  std::string trace_json;
  auto api = gurka::do_action(valhalla::Options::trace_attributes, map, {"A", "B", "C", "D"}, "auto",
                              {}, {}, &trace_json, "via");

  rapidjson::Document result;
  result.Parse(trace_json.c_str());

  auto edges = result["edges"].GetArray();
  ASSERT_EQ(edges.Size(), 2);

  EXPECT_TRUE(edges[0].HasMember("traffic_signal"));
  EXPECT_TRUE(edges[0]["traffic_signal"].GetBool());

  EXPECT_TRUE(edges[1].HasMember("traffic_signal"));
  EXPECT_FALSE(edges[1]["traffic_signal"].GetBool());
}