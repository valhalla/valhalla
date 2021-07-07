#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla;

const std::unordered_map<std::string, std::string> build_config{{"mjolnir.shortcuts", "false"}};

void check_opposing(const gurka::map& map, uint32_t expected_edge_count) {
  // then we test the invariant that any edge should be the opposing edge of its opposing edge
  // edge == opposing(opposing(edge))
  valhalla::baldr::GraphReader reader(map.config.get_child("mjolnir"));
  uint32_t actual_edge_count = 0;
  for (auto tile_id : reader.GetTileSet()) {
    auto tile = reader.GetGraphTile(tile_id);
    for (auto edge = tile_id; edge.id() < tile->header()->directededgecount(); ++edge) {
      ++actual_edge_count;
      auto opposing = reader.GetOpposingEdgeId(edge);
      auto opposing_opposing = reader.GetOpposingEdgeId(opposing);
      EXPECT_EQ(edge, opposing_opposing);
    }
  }

  // it should have the right number of edges
  EXPECT_EQ(expected_edge_count, actual_edge_count);
}

TEST(loops, flat_loop) {
  const std::string ascii_map = R"(A-----B)";
  const gurka::ways ways = {
      {"ABA", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_flat_loop", build_config);
  check_opposing(map, 2);
}

TEST(loops, long_loop) {
  const std::string ascii_map = R"(A--B--C--D--E)";
  const gurka::ways ways = {
      {"ABCDEDCBA", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_long_loop", build_config);
  check_opposing(map, 8);
}

TEST(loops, lollipop) {
  const std::string ascii_map = R"(
              C----D
             /      \
      A-----B        |
             \      /
              F----E)";
  const gurka::ways ways = {
      {"ABCDEFBA", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_lollipop", build_config);
  check_opposing(map, 6);
}

TEST(loops, split_hair) {
  const std::string ascii_map = R"(
  F--E
      \
       C-----D
      /
  A--B)";
  const gurka::ways ways = {
      {"ABCDCEF", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_split_hair", build_config);
  check_opposing(map, 6);
}

TEST(loops, split_lolli) {
  const std::string ascii_map = R"(
  K--J         I----H
      \       /      \
       C--D--E        |
      /       \      /
  A--B         F----G)";
  const gurka::ways ways = {
      {"ABCDEFGHIEDCJK", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_split_lolli", build_config);
  check_opposing(map, 12);
}

TEST(loops, eye) {
  const std::string ascii_map = R"(
          J----I
         /      \
  A--B--C        F--G--H
         \      /
          D----E)";
  const gurka::ways ways = {
      {"ABCDEFGHGFIJC", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_eye", build_config);
  check_opposing(map, 10);
}

TEST(loops, bubble) {
  const std::string ascii_map = R"(
          H----G
         /      \
  A--B--C--------D--E--F)";
  const gurka::ways ways = {
      {"ABCDEFEDGHC", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_bubble", build_config);
  check_opposing(map, 10);
}

TEST(loops, phi) {
  const std::string ascii_map = R"(
              C----D
             /      \
      A-----B--------E
             \      /
              G----F)";
  const gurka::ways ways = {
      {"ABCDEFGBE", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_phi", build_config);
  check_opposing(map, 10);
}

TEST(loops, bow_tie) {
  const std::string ascii_map = R"(
              B   F
             / \ / \
            A   C   E
             \ / \ /
              G   D)";
  const gurka::ways ways = {
      {"ABCDEFCGA", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_bow_tie", build_config);
  check_opposing(map, 8);
}

TEST(loops, please_god_why) {
  const std::string ascii_map = R"(
              F----G
             /      \
      E--D--C---B----A
          \         /
           I-------H)";
  const gurka::ways ways = {
      {"ABCDEDCFGAHID", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_please_god_why", build_config);
  check_opposing(map, 12);
}

TEST(loops, use_kCuldesac_false) {
  const std::string ascii_map = R"(
          J----I
         /      \
  A--B--C        F--G--H
         \      /\
          D----E  K--L
)";
  const gurka::ways ways = {
      {"CDEFIJC", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"ABC", {{"highway", "residential"}}},
      {"FGH", {{"highway", "residential"}}},
      {"LKF", {{"highway", "living_street"}, {"oneway", "yes"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_use_kCuldesac_false", build_config);
  const auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  const auto edge = gurka::findEdgeByNodes(*reader, map.nodes, "F", "C");
  EXPECT_NE(std::get<1>(edge)->use(), Use::kCuldesac);
}

TEST(loops, use_kCuldesac_true) {
  const std::string ascii_map = R"(
          J----I
         /      \
  A--B--C        F
         \      /
          D----E
)";
  const gurka::ways ways = {
      {"CDEFIJC", {{"highway", "residential"}}},
      {"ABC", {{"highway", "residential"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_use_kCuldesac_true", build_config);
  const auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  const auto edge = gurka::findEdgeByNodes(*reader, map.nodes, "C", "F");
  EXPECT_EQ(std::get<1>(edge)->use(), Use::kCuldesac);
}

TEST(loops, use_kCuldesac_foot_path) {
  const std::string ascii_map = R"(
          J----I
         /      \
  A--B--C        F--G--H
         \      /
          D----E
)";
  const gurka::ways ways = {
      {"CDEFIJC", {{"highway", "residential"}}},
      {"ABC", {{"highway", "residential"}}},
      {"FGH", {{"highway", "foot_path"}, {"bicycle", "yes"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_use_kCuldesac_foot_path",
                                     build_config);
  const auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  const auto edge = gurka::findEdgeByNodes(*reader, map.nodes, "C", "F");
  EXPECT_EQ(std::get<1>(edge)->use(), Use::kCuldesac);
}

TEST(loops, use_kCuldesac_several_roads_from_one_node) {
  const std::string ascii_map = R"(
    J----I
   /      \
  C        F--G--H
   \      /\
    D----E  K--L
)";
  const gurka::ways ways = {
      {"CDEFIJC", {{"highway", "residential"}}},
      {"FGH", {{"highway", "residential"}}},
      {"FKL", {{"highway", "residential"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const auto map =
      gurka::buildtiles(layout, ways, {}, {},
                        "test/data/gurka_use_kCuldesac_several_roads_from_one_node", build_config);
  const auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  const auto edge = gurka::findEdgeByNodes(*reader, map.nodes, "C", "F");
  EXPECT_EQ(std::get<1>(edge)->use(), Use::kCuldesac);
}
