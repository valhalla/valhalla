#include "gurka.h"
#include "midgard/sequence.h"
#include "mjolnir/osmrestriction.h"

#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

// TEMP diagnostic: run only through kParseRelations and dump the complex restriction
// sequence files, so we can tell whether the parse stage produced the restriction
// records correctly (before graphbuilder/restrictionbuilder run).
TEST(OnlyRestrictions, DumpParsedRestrictions) {
  const std::string ascii_map = R"(
         1
        /
     B-C---D---E
)";
  const gurka::ways ways = {
      {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"DE", {{"highway", "primary"}}},
      {"C1", {{"highway", "primary"}}},
  };
  const gurka::relations relations = {
      {{
           {gurka::way_member, "BC", "from"},
           {gurka::way_member, "CD", "via"},
           {gurka::way_member, "DE", "to"},
       },
       {{"type", "restriction"}, {"restriction", "only_left_turn"}}},
  };
  const std::string workdir = "test/data/only_restrictions_parse";
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, relations, workdir, {{"mjolnir.concurrency", "1"}},
                               mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kParseRelations);

  auto tile_dir = map.config.get<std::string>("mjolnir.tile_dir");
  auto dump = [&](const std::string& fname) {
    midgard::sequence<mjolnir::OSMRestriction> seq(tile_dir + "/" + fname, false);
    printf("[PARSE] %s count=%zu\n", fname.c_str(), seq.size());
    for (const auto& r : seq) {
      std::string vias;
      for (auto v : r.vias())
        vias += std::to_string(v) + " ";
      printf("[PARSE]   from=%llu to=%llu type=%d modes=%u vias=[%s]\n",
             (unsigned long long)r.from(), (unsigned long long)r.to(), (int)r.type(), r.modes(),
             vias.c_str());
    }
  };
  dump("complex_from_restrictions.bin");
  dump("complex_to_restrictions.bin");
}

// TEMP diagnostic: build a map with one complex restriction and dump what actually
// landed in the tiles (restriction byte-sizes + per-edge restriction flags). used to
// localize the windows restriction failures.
TEST(OnlyRestrictions, DumpGraphStats) {
  const std::string ascii_map = R"(
         1
        /
     B-C---D---E
)";
  const gurka::ways ways = {
      {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"DE", {{"highway", "primary"}}},
      {"C1", {{"highway", "primary"}}},
  };
  const gurka::relations relations = {
      {{
           {gurka::way_member, "BC", "from"},
           {gurka::way_member, "CD", "via"},
           {gurka::way_member, "DE", "to"},
       },
       {{"type", "restriction"}, {"restriction", "only_left_turn"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/only_restrictions_dump",
                               {{"mjolnir.concurrency", "1"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  for (const auto& tile_id : reader.GetTileSet()) {
    auto tile = reader.GetGraphTile(tile_id);
    if (!tile)
      continue;
    const auto* h = tile->header();
    size_t fwd_bytes = h->complex_restriction_reverse_offset() - h->complex_restriction_forward_offset();
    size_t rev_bytes = h->edgeinfo_offset() - h->complex_restriction_reverse_offset();
    int n_start = 0, n_end = 0, n_complex = 0;
    for (uint32_t i = 0; i < h->directededgecount(); ++i) {
      const auto* de = tile->directededge(i);
      n_start += de->start_restriction() != 0;
      n_end += de->end_restriction() != 0;
      n_complex += de->part_of_complex_restriction();
    }
    printf("[STATS] tile %u edges=%u fwd_cr_bytes=%zu rev_cr_bytes=%zu "
           "start_restr_edges=%d end_restr_edges=%d complex_edges=%d\n",
           tile_id.tileid(), h->directededgecount(), fwd_bytes, rev_bytes, n_start, n_end, n_complex);
  }
}

TEST(OnlyRestrictions, Straight) {
  const std::string ascii_map = R"(
   A-B-C-D-E-F
)";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}}, {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}}, {"DE", {{"highway", "primary"}}},
      {"EF", {{"highway", "primary"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "BC", "from"},
           {gurka::way_member, "CD", "via"},
           {gurka::way_member, "DE", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "only_straight_on"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/only_restrictions_straight",
                               {{"mjolnir.concurrency", "1"}});

  for (const auto& from : map.nodes) {
    for (const auto& to : map.nodes) {
      if (from == to)
        continue;
      auto result = gurka::do_action(valhalla::Options::route, map, {from.first, to.first}, "auto");
      EXPECT_EQ(result.trip().routes_size(), 1);
    }
  }
}

TEST(OnlyRestrictions, OneNeighbour) {
  const std::string ascii_map = R"(
         1
        /
     B-C---D---E
)";

  const gurka::ways ways = {
      {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"DE", {{"highway", "primary"}}},
      {"C1", {{"highway", "primary"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "BC", "from"},
           {gurka::way_member, "CD", "via"},
           {gurka::way_member, "DE", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "only_left_turn"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map =
      gurka::buildtiles(layout, ways, {}, relations, "test/data/only_restrictions_one_neighbour",
                        {{"mjolnir.concurrency", "1"}});

  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"B", "1"}, "auto");
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
}

TEST(OnlyRestrictions, ManyNeighbours) {
  const std::string ascii_map = R"(
           1   3--5
           |   |
  A----B---C---D---E-----F
           |   |
           2   4
)";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}},
      {"DE", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"EF", {{"highway", "primary"}}},
      {"C1", {{"highway", "primary"}}},
      {"C2", {{"highway", "primary"}}},
      {"D3", {{"highway", "primary"}}},
      {"D4", {{"highway", "primary"}}},
      {"35", {{"highway", "primary"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "BC", "from"},
           {gurka::way_member, "CD", "via"},
           {gurka::way_member, "DE", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "only_right_turn"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map =
      gurka::buildtiles(layout, ways, {}, relations, "test/data/only_restrictions_many_neighbours",
                        {{"mjolnir.concurrency", "1"}});

  for (const auto& from : {"A", "B"}) {
    for (const auto& to : {"1", "2", "3", "4", "5"}) {
      try {
        auto result = gurka::do_action(valhalla::Options::route, map, {from, to}, "auto");
        gurka::assert::raw::expect_path(result, {std::string("Unexpected path found for request ") +
                                                 from + " -> " + to});

      } catch (const std::runtime_error& e) {
        EXPECT_STREQ(e.what(), "No path could be found for input");
      }
    }
  }
}
