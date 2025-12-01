#include "gurka.h"
#include "mjolnir/util.h"

#include <gtest/gtest.h>

#include <filesystem>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}};

TEST(Standalone, Separate) {
  const std::string ascii_map = R"(
                      A
                     / \
                    /   \
                   B-----C
                   |     |
                   |     |
                   |     |
                   |     |
                   D-----E
                    \   /
                     \ /
                      F

	  )";

  const gurka::ways ways = {
      {"FD", {{"highway", "primary"}, {"cycleway", "separate"}, {"oneway", "yes"}}},
      {"DB", {{"highway", "primary"}, {"cycleway", "separate"}, {"oneway", "yes"}}},
      {"BA", {{"highway", "primary"}, {"cycleway", "separate"}, {"oneway", "yes"}}},
      {"DE", {{"highway", "cycleway"}, {"cycleway", "crossing"}}},
      {"FE", {{"highway", "cycleway"}, {"cycleway", "sidwalk"}, {"oneway", "yes"}}},
      {"EC", {{"highway", "cycleway"}, {"cycleway", "sidwalk"}, {"oneway", "yes"}}},
      {"CA", {{"highway", "cycleway"}, {"cycleway", "sidwalk"}, {"oneway", "yes"}}},
      {"BC", {{"highway", "cycleway"}, {"cycleway", "crossing"}}},
  };

  constexpr double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.1079374, 52.0887174});
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_cycleway_separate", build_config);

  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "A"}, "bicycle");
  gurka::assert::raw::expect_path(result, {"FE", "EC", "CA"});

  GraphReader graph_reader(map.config.get_child("mjolnir"));

  {
    GraphId FD_edge_id;
    const DirectedEdge* FD_edge = nullptr;
    GraphId DF_edge_id;
    const DirectedEdge* DF_edge = nullptr;
    std::tie(FD_edge_id, FD_edge, DF_edge_id, DF_edge) = findEdge(graph_reader, map.nodes, "FD", "D");
    EXPECT_NE(FD_edge, nullptr);
    EXPECT_NE(DF_edge, nullptr);

    EXPECT_EQ(static_cast<uint8_t>(FD_edge->cyclelane()), static_cast<uint8_t>(CycleLane::kSeparated));
    EXPECT_EQ(static_cast<uint8_t>(DF_edge->cyclelane()), static_cast<uint8_t>(CycleLane::kNone));
  }
}

TEST(Standalone, Cycleway_Both) {
  const std::string ascii_map = R"(
                      A
                     /|\
                    / | \
                   B--C--D
                   |  |  |
                   |  |  |
                   |  |  |
                   |  |  |
                   E--F--G
                    \ | /
                     \|/
                      H

	  )";

  const gurka::ways ways = {
      {"HF", {{"highway", "primary"}, {"cycleway:both", "separate"}}},
      {"FC", {{"highway", "primary"}, {"cycleway:both", "separate"}}},
      {"CA", {{"highway", "primary"}, {"cycleway:both", "separate"}}},
      {"EFG", {{"highway", "cycleway"}, {"cycleway", "crossing"}}},
      {"HG", {{"highway", "cycleway"}, {"cycleway", "sidwalk"}, {"oneway", "yes"}}},
      {"GD", {{"highway", "cycleway"}, {"cycleway", "sidwalk"}, {"oneway", "yes"}}},
      {"DA", {{"highway", "cycleway"}, {"cycleway", "sidwalk"}, {"oneway", "yes"}}},
      {"AB", {{"highway", "cycleway"}, {"cycleway", "sidwalk"}, {"oneway", "yes"}}},
      {"BE", {{"highway", "cycleway"}, {"cycleway", "sidwalk"}, {"oneway", "yes"}}},
      {"EH", {{"highway", "cycleway"}, {"cycleway", "sidwalk"}, {"oneway", "yes"}}},
      {"BCD", {{"highway", "cycleway"}, {"cycleway", "crossing"}}},
  };

  constexpr double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.1079374, 52.0887174});
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_cycleway_both", build_config);

  auto result = gurka::do_action(valhalla::Options::route, map, {"H", "A"}, "bicycle");
  gurka::assert::raw::expect_path(result, {"HG", "GD", "DA"});

  GraphReader graph_reader(map.config.get_child("mjolnir"));

  {
    GraphId FC_edge_id;
    const DirectedEdge* FC_edge = nullptr;
    GraphId CF_edge_id;
    const DirectedEdge* CF_edge = nullptr;
    std::tie(FC_edge_id, FC_edge, CF_edge_id, CF_edge) = findEdge(graph_reader, map.nodes, "FC", "C");
    EXPECT_NE(FC_edge, nullptr);
    EXPECT_NE(CF_edge, nullptr);

    EXPECT_EQ(static_cast<uint8_t>(FC_edge->cyclelane()), static_cast<uint8_t>(CycleLane::kSeparated));
    EXPECT_EQ(static_cast<uint8_t>(CF_edge->cyclelane()), static_cast<uint8_t>(CycleLane::kSeparated));
  }

  {
    GraphId GD_edge_id;
    const DirectedEdge* GD_edge = nullptr;
    GraphId DG_edge_id;
    const DirectedEdge* DG_edge = nullptr;
    std::tie(GD_edge_id, GD_edge, DG_edge_id, DG_edge) = findEdge(graph_reader, map.nodes, "GD", "D");
    EXPECT_NE(GD_edge, nullptr);
    EXPECT_NE(DG_edge, nullptr);

    EXPECT_EQ(static_cast<uint8_t>(GD_edge->cyclelane()), static_cast<uint8_t>(CycleLane::kSeparated));
    EXPECT_EQ(static_cast<uint8_t>(DG_edge->cyclelane()), static_cast<uint8_t>(CycleLane::kNone));
  }

  {
    GraphId BE_edge_id;
    const DirectedEdge* BE_edge = nullptr;
    GraphId EB_edge_id;
    const DirectedEdge* EB_edge = nullptr;
    std::tie(BE_edge_id, BE_edge, EB_edge_id, EB_edge) = findEdge(graph_reader, map.nodes, "BE", "E");
    EXPECT_NE(BE_edge, nullptr);
    EXPECT_NE(EB_edge, nullptr);

    EXPECT_EQ(static_cast<uint8_t>(BE_edge->cyclelane()), static_cast<uint8_t>(CycleLane::kSeparated));
    EXPECT_EQ(static_cast<uint8_t>(EB_edge->cyclelane()), static_cast<uint8_t>(CycleLane::kNone));
  }
}

TEST(Standalone, Cycleway_Right_Oneway) {
  const std::string ascii_map = R"(
                      A
                      |
                      |
                      |
                      |
                      |
                      |
                      |
                      B                   
	  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}, {"cycleway:right", "track"}, {"oneway", "yes"},
    		  {"cycleway:right:oneway", "-1"}, {"cycleway:right", "track"}, {"oneway:bicycle", "no"}}},
  };

  constexpr double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.1079374, 52.0887174});
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_cycleway_route_oneway", build_config);

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
  gurka::assert::raw::expect_path(result, {"AB"});

  try{
	  result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "bicycle");
	  FAIL();
  } catch (const std::exception&) { SUCCEED(); }

  try{
	  result = gurka::do_action(valhalla::Options::route, map, {"B", "A"}, "auto");
	  FAIL();
  } catch (const std::exception&) { SUCCEED(); }

  result = gurka::do_action(valhalla::Options::route, map, {"B", "A"}, "bicycle");
  gurka::assert::raw::expect_path(result, {"AB"});

  GraphReader graph_reader(map.config.get_child("mjolnir"));

  {
    GraphId AB_edge_id;
    const DirectedEdge* AB_edge = nullptr;
    GraphId BA_edge_id;
    const DirectedEdge* BA_edge = nullptr;
    std::tie(AB_edge_id, AB_edge, BA_edge_id, BA_edge) = findEdge(graph_reader, map.nodes, "AB", "B");
    EXPECT_NE(AB_edge, nullptr);
    EXPECT_NE(BA_edge, nullptr);

    EXPECT_EQ(static_cast<uint8_t>(AB_edge->cyclelane()), static_cast<uint8_t>(CycleLane::kNone));
    EXPECT_EQ(static_cast<uint8_t>(BA_edge->cyclelane()), static_cast<uint8_t>(CycleLane::kSeparated));
  }
}
