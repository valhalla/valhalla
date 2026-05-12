#include "gurka.h"
#include "test.h"
#include "valhalla/mjolnir/way_edges_processor.h"

#include <gtest/gtest.h>

using namespace valhalla;
using namespace valhalla::baldr;

class MappingTest : public ::testing::Test {
protected:
  static gurka::map map;
  static std::string tile_dir;
  static gurka::nodelayout layout;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
  A--B--C--D   O-------R---T
  |     |  |   |       |
  E     G--K   |       |
  |     |  |   |       |
  |     |  |   |       |
  F---H-I--J   P-------Q---S

      h      
      |      
  a---g----b  i--------j  m--------n---r
  |        |  |        |  |        |
  |        |  |        |  |        |
  |        |  |        |  |        |
  e---d----c  l--------k  p--------o
      |      
      f      
  )";

    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}}},
        {"BC", {{"highway", "primary"}}},
        {"CD", {{"highway", "primary"}}},
        {"AEF", {{"highway", "primary"}}},
        {"FH", {{"highway", "primary"}}},
        {"HI", {{"highway", "primary"}}},
        {"IJ", {{"highway", "primary"}}},
        {"GK", {{"highway", "living_street"}}},
        {"CGI", {{"highway", "living_street"}}},
        {"DK", {{"highway", "living_street"}}},
        {"KJ", {{"highway", "living_street"}}},
        // a loop that is broken up during graph build and has different
        // start and end nodes
        {"QROPQS", {{"highway", "living_street"}, {"osm_id", "120"}}},
        {"RT", {{"highway", "living_street"}}},
        // a loop that's broken up during graph build and starts and
        // ends at the same node
        {"agbcdea", {{"highway", "living_street"}, {"osm_id", "140"}}},
        {"gh", {{"highway", "living_street"}}},
        {"df", {{"highway", "living_street"}}},
        // a loop not broken up with matching start and end nodes
        {"ijkli", {{"highway", "living_street"}}},
        // a loop not broken up with non-matching start and end nodes
        {"rnmpon", {{"highway", "living_street"}}},
    };

    layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    tile_dir = "test/data/ways_to_edges";
    map = gurka::buildtiles(layout, ways, {}, {}, tile_dir,
                            {
                                {"mjolnir.concurrency", "1"},
                                {"mjolnir.keep_osm_node_ids", "1"},
                                {"mjolnir.keep_all_osm_node_ids", "1"},
                            });
  }
};

gurka::map MappingTest::map = {};
std::string MappingTest::tile_dir;
gurka::nodelayout MappingTest::layout;

TEST_F(MappingTest, ways_to_edges_sorted) {
  baldr::GraphReader reader = valhalla::baldr::GraphReader(map.config.get_child("mjolnir"));
  auto config = test::make_config(tile_dir, {{
                                                 "tile_dir",
                                                 "test/data/ways_to_edges",
                                             },
                                             {"tile_extract", "test/data/ways_to_edges.tar"}});

  std::string input = "test/data/ways_to_edges/map.pbf";
  auto ways_to_edges = mjolnir::collect_way_edges_sorted(input, reader, "");

  std::unordered_set<std::string> expected_names{"AB", "BC", "CD",     "AEF",    "FH",      "HI",
                                                 "IJ", "GK", "CGI",    "DK",     "KJ",      "RT",
                                                 "gh", "df", "QROPQS", "rnmpon", "agbcdea", "ijkli"};

  for (const auto& [key, edges] : ways_to_edges) {
    for (const auto& edge : edges) {
      // get the names
      auto ei = reader.edgeinfo(valhalla::baldr::GraphId(edge.edgeid));
      auto name = ei.GetNames()[0];
      auto it = expected_names.find(name);
      EXPECT_NE(it, expected_names.end()) << "Unexpected way in mapping: " << name;
    }
    auto ei = reader.edgeinfo(GraphId(edges[0].edgeid));
    auto name = ei.GetNames()[0];
    if (name == "CGI") {
      std::vector<decltype(edges)::value_type> fwd_edges;
      std::vector<decltype(edges)::value_type> bwd_edges;
      for (const auto& edge : edges) {
        if (edge.forward) {
          fwd_edges.push_back(edge);
        } else {
          bwd_edges.push_back(edge);
        }
      }

      // ensure the order is correct
      ASSERT_TRUE(fwd_edges[0].length < fwd_edges[1].length);
      ASSERT_TRUE(bwd_edges[1].length < bwd_edges[0].length);
    }
  }
  // make a copy that we'll remove found items from
  // to make sure all of our expected edge names were found
  auto expected_names_copy = expected_names;
  for (const auto& [key, edges] : ways_to_edges) {
    EXPECT_GT(edges.size(), 1); // we expect at least two edges for each way
    for (const auto& edge : edges) {
      auto ei = reader.edgeinfo(valhalla::baldr::GraphId(edge.edgeid));
      auto name = ei.GetNames()[0];
      auto it = expected_names_copy.find(name);
      if (it != expected_names_copy.end()) {
        expected_names_copy.erase(it);
      }
    }
  }
  if (!expected_names_copy.empty()) {
    std::string names;
    std::string sep;

    for (const auto& name : expected_names_copy) {
      names += sep + name;
      sep = ", ";
    }
    FAIL() << "Umatched edges: " << names;
  }

  EXPECT_EQ(ways_to_edges.size(), expected_names.size()) << "Wrong number of ways in mapping";

  GraphId AEF_edge_id;
  const DirectedEdge* AEF_edge = nullptr;
  std::tie(AEF_edge_id, AEF_edge) = valhalla::gurka::findEdgeByNodes(reader, map.nodes, "A", "F");
  EXPECT_NE(AEF_edge, nullptr);

  GraphId node_id = AEF_edge->endnode();
  auto tile = reader.GetGraphTile(node_id);
  auto edgeinfo = tile->edgeinfo(AEF_edge);

  auto osm_id = edgeinfo.wayid();
  EXPECT_NE(ways_to_edges.find(osm_id), ways_to_edges.end());
  EXPECT_EQ(ways_to_edges[osm_id].size(), 2);

  // CGI should have been split up
  GraphId CG_edge_id;
  const DirectedEdge* CG_edge = nullptr;
  std::tie(CG_edge_id, CG_edge) = valhalla::gurka::findEdgeByNodes(reader, map.nodes, "C", "G");
  EXPECT_NE(CG_edge, nullptr);

  node_id = CG_edge->endnode();
  tile = reader.GetGraphTile(CG_edge_id);
  edgeinfo = tile->edgeinfo(CG_edge);

  osm_id = edgeinfo.wayid();
  EXPECT_NE(ways_to_edges.find(osm_id), ways_to_edges.end());
  auto edges_CGI = ways_to_edges.find(osm_id);
  size_t forward_count = 0, backward_count = 0;
  for (const auto edge : edges_CGI->second) {
    if (edge.forward)
      forward_count++;
    if (!edge.forward)
      backward_count++;
  }
  EXPECT_EQ(edges_CGI->second.size(), 4);
  EXPECT_EQ(forward_count, 2);
  EXPECT_EQ(backward_count, 2);

  // Also test the order of the loop edges
  // QROPQS
  {
    auto it = ways_to_edges.find(120);
    EXPECT_NE(it, ways_to_edges.end());

    // QROPQS gets split up at node O
    // because otherwise it would form the
    // loop edge RQ
    std::vector<std::pair<std::string, std::string>> expected_edges{
        {"Q", "R"}, {"R", "O"}, {"O", "Q"}, {"Q", "S"},
        {"S", "Q"}, {"Q", "O"}, {"O", "R"}, {"R", "Q"},
    };

    EXPECT_EQ(it->second.size(), expected_edges.size());
    for (size_t i = 0; i < expected_edges.size(); ++i) {
      auto& fromto = expected_edges[i];
      GraphId edgeid;
      const DirectedEdge* de = nullptr;

      std::tie(edgeid, de) =
          valhalla::gurka::findEdgeByNodes(reader, map.nodes, fromto.first, fromto.second);

      auto names_1 = reader.edgeinfo(edgeid).GetNames()[0];
      auto names_2 = reader.edgeinfo(GraphId(it->second[i].edgeid)).GetNames()[0];
      EXPECT_EQ(edgeid, it->second[i].edgeid)
          << "Unexpected order of edges: " << std::to_string(edgeid) << " (" << names_1 << ") "
          << " vs. " << std::to_string(GraphId(it->second[i].edgeid)) << " (" << names_2
          << ") at index " << i;
    }
  }

  // agbcdea
  {
    auto it = ways_to_edges.find(140);
    EXPECT_NE(it, ways_to_edges.end());

    // abgcdea gets split up at node c
    // in the graph parser to prevent loop edges
    std::vector<std::pair<std::string, std::string>> expected_edges{
        {"a", "g"}, {"g", "c"}, {"c", "d"}, {"d", "a"},
        {"a", "d"}, {"d", "c"}, {"c", "g"}, {"g", "a"},
    };

    EXPECT_EQ(it->second.size(), expected_edges.size());
    for (size_t i = 0; i < expected_edges.size(); ++i) {
      auto& fromto = expected_edges[i];
      GraphId edgeid;
      const DirectedEdge* de = nullptr;
      std::tie(edgeid, de) =
          valhalla::gurka::findEdgeByNodes(reader, map.nodes, fromto.first, fromto.second);

      auto names_1 = reader.edgeinfo(edgeid).GetNames()[0];
      auto names_2 = reader.edgeinfo(GraphId(it->second[i].edgeid)).GetNames()[0];
      EXPECT_EQ(edgeid, it->second[i].edgeid)
          << "Unexpected order of edges: " << std::to_string(edgeid) << " (" << names_1 << ") "
          << " vs. " << std::to_string(GraphId(it->second[i].edgeid)) << " (" << names_2
          << ") at index " << i;
    }
  }
}
