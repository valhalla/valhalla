
#include "baldr/graphreader.h"
#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, AcrossHierarchies) {
  const std::string ascii_map = R"(
      q  1                   2       o
      |  |                   |       |
      m--A----B----C-x--D----E----F--n
      |  |                   |       |
      p  3                   4       r
  )";

  // clang-format off
  const gurka::ways ways = {
    {"AB", {{"highway", "residential"}}},
    {"BC", {{"highway", "residential"}}},
    {"CD", {{"highway", "residential"}}},
    {"DE", {{"highway", "residential"}}},
    {"EF", {{"highway", "residential"}}}, 
    {"1A", {{"highway", "service"}}},
    {"3A", {{"highway", "service"}}},
    {"2E", {{"highway", "service"}}},
    {"4E", {{"highway", "service"}}},
    {"Am", {{"highway", "residential"}}},
    {"mq", {{"highway", "service"}}},
    // some bogus ones to avoid deadends
    {"mp", {{"highway", "service"}}},
    {"Fn", {{"highway", "residential"}}},
    {"no", {{"highway", "service"}}},
    {"nr", {{"highway", "service"}}},
  };
  // clang-format on

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_road_segments");
  std::string json;
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  auto r = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  auto result = gurka::do_action(valhalla::Options::locate, map, {"x"}, "auto",
                                 {{"/verbose", "0"}, {"/road_segments", "0"}}, r, &json);
  rapidjson::Document response;
  response.Parse(json);
  if (response.HasParseError())
    throw std::runtime_error("bad json response");
  auto expected_outer_id_1 = gurka::findNode(reader, layout, "m");
  auto expected_outer_id_2 = gurka::findNode(reader, layout, "n");
  const auto* expected_outer_1 = reader.nodeinfo(expected_outer_id_1);
  const auto* expected_outer_2 = reader.nodeinfo(expected_outer_id_2);
  auto start_node_id = static_cast<uint64_t>(
      rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/start_node/id/value")
          .Get(response)
          ->GetInt64());

  auto end_node_id = static_cast<uint64_t>(
      rapidjson::Pointer("/0/edges/0/full_road_segment/intersections/end_node/id/value")
          .Get(response)
          ->GetInt64());

  EXPECT_TRUE(expected_outer_id_1.value == start_node_id ||
              expected_outer_id_2.value == start_node_id);
  EXPECT_TRUE(expected_outer_id_1.value == end_node_id || expected_outer_id_2.value == end_node_id);
}