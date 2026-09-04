#include "baldr/rapidjson_utils.h"
#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, Maxspeed) {
  const std::string ascii_map = R"(
      A----B----C----D----E----F
                               |
                               G
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}, {"maxspeed", "50"}}},
      {"BC", {{"highway", "motorway"}, {"maxspeed", "60mph"}}},
      {"CD", {{"highway", "motorway"}, {"maxspeed", ""}}},
      {"DE", {{"highway", "motorway"}}},
      {"EF", {{"highway", "motorway"}, {"maxspeed", "none"}}},
      {"FG", {{"highway", "motorway"}, {"maxspeed", "40"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_maxspeed");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto",
                                 {{"/filters/action", "include"},
                                  {"/filters/attributes/0", "shape_attributes.speed_limit"}});

  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);

  EXPECT_EQ(leg.node(0).edge().speed_limit(), 50);  // AB
  EXPECT_EQ(leg.node(1).edge().speed_limit(), 97);  // BC
  EXPECT_EQ(leg.node(2).edge().speed_limit(), 0);   // CD
  EXPECT_EQ(leg.node(3).edge().speed_limit(), 0);   // DE
  EXPECT_EQ(leg.node(4).edge().speed_limit(), 255); // EF
  EXPECT_EQ(leg.node(5).edge().speed_limit(), 40);  // FG

  // Test osrm output
  //
  // "maxspeed": [
  //  {"speed":50,"unit":"km\/h"},
  //  {"speed":97,"unit":"km\/h"},
  //  {"unknown":true},
  //  {"unknown":true},
  //  {"none":true},
  //  {"speed":40,"unit":"km\/h"}
  // ]
  auto d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);
  EXPECT_EQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"].Size(), 6);
  EXPECT_EQ(d["routes"][0]["legs"][0]["steps"].Size(), 3);

  EXPECT_EQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][0]["speed"].GetInt(), 50);
  EXPECT_STREQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][0]["unit"].GetString(), "km/h");
  EXPECT_EQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][1]["speed"].GetInt(), 97);
  EXPECT_STREQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][1]["unit"].GetString(), "km/h");
  EXPECT_EQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][2]["unknown"].GetBool(), true);
  EXPECT_EQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][3]["unknown"].GetBool(), true);
  EXPECT_EQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][4]["none"].GetBool(), true);
  EXPECT_EQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][5]["speed"].GetInt(), 40);
  EXPECT_STREQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][5]["unit"].GetString(), "km/h");

  EXPECT_STREQ(d["routes"][0]["legs"][0]["steps"][0]["speedLimitSign"].GetString(), "vienna");
  EXPECT_STREQ(d["routes"][0]["legs"][0]["steps"][0]["speedLimitUnit"].GetString(), "km/h");
  EXPECT_STREQ(d["routes"][0]["legs"][0]["steps"][1]["speedLimitSign"].GetString(), "vienna");
  EXPECT_STREQ(d["routes"][0]["legs"][0]["steps"][1]["speedLimitUnit"].GetString(), "km/h");
  EXPECT_STREQ(d["routes"][0]["legs"][0]["steps"][2]["speedLimitSign"].GetString(), "vienna");
  EXPECT_STREQ(d["routes"][0]["legs"][0]["steps"][2]["speedLimitUnit"].GetString(), "km/h");
}

TEST(Standalone, ReverseSpeedLimit) {
  const std::string ascii_map = R"(
      A----B----C----D
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"maxspeed:forward", "100"}, {"maxspeed:backward", "80"}}},
      {"BC",
       {{"highway", "primary"},
        {"maxspeed", "90"},
        {"maxspeed:forward", "100"},
        {"maxspeed:backward", "90"}}},
      {"CD", {{"highway", "primary"}, {"maxspeed", "70"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_reverse_speed_limit");

  // the tag is only written when the limits differ and is read relative to the stored shape
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  for (const auto& [begin, end, limit, has_reverse] :
       std::vector<std::tuple<std::string, std::string, uint32_t, bool>>{
           {"A", "B", 100, true},
           {"B", "A", 80, true},
           {"B", "C", 100, true},
           {"C", "B", 90, true},
           {"C", "D", 70, false},
           {"D", "C", 70, false},
       }) {
    auto [edge_id, edge] = gurka::findEdgeByNodes(reader, layout, begin, end);
    auto edgeinfo = reader.GetGraphTile(edge_id)->edgeinfo(edge);
    EXPECT_EQ(edgeinfo.speed_limit(edge->forward()), limit) << begin << end;
    EXPECT_EQ(edgeinfo.GetTags().count(baldr::TaggedValue::kReverseSpeedLimit) > 0, has_reverse)
        << begin << end;
  }

  // the trip carries the limit for the direction of travel
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");
  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(0).edge().speed_limit(), 100); // AB
  EXPECT_EQ(leg.node(1).edge().speed_limit(), 100); // BC
  EXPECT_EQ(leg.node(2).edge().speed_limit(), 70);  // CD

  result = gurka::do_action(valhalla::Options::route, map, {"D", "A"}, "auto");
  leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(0).edge().speed_limit(), 70); // DC
  EXPECT_EQ(leg.node(1).edge().speed_limit(), 90); // CB
  EXPECT_EQ(leg.node(2).edge().speed_limit(), 80); // BA

  // locate reports both limits, the reverse one only when it is stored
  std::string json;
  gurka::do_action(valhalla::Options::locate, map, {"A", "D"}, "auto", {}, nullptr, &json);
  rapidjson::Document root;
  root.Parse(json);
  ASSERT_FALSE(root.HasParseError()) << json;
  ASSERT_EQ(root.GetArray().Size(), 2);

  for (const auto& edge : root[0]["edges"].GetArray()) {
    const auto& edge_info = edge["edge_info"];
    EXPECT_EQ(edge_info["speed_limit"], 100);
    ASSERT_TRUE(edge_info.HasMember("reverse_speed_limit"));
    EXPECT_EQ(edge_info["reverse_speed_limit"], 80);
  }
  for (const auto& edge : root[1]["edges"].GetArray()) {
    const auto& edge_info = edge["edge_info"];
    EXPECT_EQ(edge_info["speed_limit"], 70);
    EXPECT_FALSE(edge_info.HasMember("reverse_speed_limit"));
  }
}
