#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(GraphReader, ModifyTilePointerByReference) {
  // 2km long way crosses a tile boundary at the prime meridian due to 1km offset from null island
  const std::string ascii_map = R"(A------------------B)";
  const gurka::ways ways = {{"AB", {{"highway", "motorway"}, {"name", "81"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {-.01, -.01});
  const auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/graphreader_tileboundary",
                        {
                            {"mjolnir.incident_dir", "test/data/graphreader_tileboundary"},
                            {"mjolnir.incident_log", "test/data/graphreader_tileboundary/log"},
                        });

  // both sets of edges that cross the tile boundary from left to right and right to left
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  auto ltr = gurka::findEdgeByNodes(reader, layout, "A", "B");
  auto rtl = gurka::findEdgeByNodes(reader, layout, "B", "A");
  auto l = gurka::findNode(reader, layout, "A");
  auto r = gurka::findNode(reader, layout, "B");

  // all methods that use/modify a reference to tile pointer with two calls for cache miss and hit
  for (size_t i = 0; i < 2; ++i) {
    baldr::GraphId edge_id, opp_edge_id;
    const baldr::DirectedEdge *edge, *opp_edge;
    std::tie(edge_id, edge) = ltr;
    std::tie(opp_edge_id, opp_edge) = rtl;
    auto begin_node_id = l;
    auto end_node_id = r;

    {
      baldr::graph_tile_ptr tile;
      reader.GetGraphTile(begin_node_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
      reader.GetGraphTile(begin_node_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
    }

    {
      baldr::graph_tile_ptr tile;
      reader.GetOpposingEdgeId(edge_id, tile);
      ASSERT_EQ(opp_edge_id.Tile_Base(), tile->id());
      reader.GetOpposingEdgeId(edge_id, tile);
      ASSERT_EQ(opp_edge_id.Tile_Base(), tile->id());
    }

    {
      baldr::graph_tile_ptr tile;
      const baldr::DirectedEdge* actual_op_edge;
      reader.GetOpposingEdgeId(edge_id, actual_op_edge, tile);
      ASSERT_EQ(opp_edge_id.Tile_Base(), tile->id());
      ASSERT_EQ(opp_edge, actual_op_edge);
      reader.GetOpposingEdgeId(edge_id, tile);
      ASSERT_EQ(opp_edge_id.Tile_Base(), tile->id());
      ASSERT_EQ(opp_edge, actual_op_edge);
    }

    {
      baldr::graph_tile_ptr tile;
      reader.GetOpposingEdge(edge_id, tile);
      ASSERT_EQ(opp_edge_id.Tile_Base(), tile->id());
      reader.GetOpposingEdge(edge_id, tile);
      ASSERT_EQ(opp_edge_id.Tile_Base(), tile->id());
    }

    {
      baldr::graph_tile_ptr tile;
      reader.GetOpposingEdge(edge, tile);
      ASSERT_EQ(opp_edge_id.Tile_Base(), tile->id());
      reader.GetOpposingEdge(edge, tile);
      ASSERT_EQ(opp_edge_id.Tile_Base(), tile->id());
    }

    {
      baldr::graph_tile_ptr tile;
      reader.GetEndNode(edge, tile);
      ASSERT_EQ(end_node_id.Tile_Base(), tile->id());
      reader.GetEndNode(edge, tile);
      ASSERT_EQ(end_node_id.Tile_Base(), tile->id());
    }

    {
      baldr::graph_tile_ptr tile;
      reader.GetBeginNodeId(edge, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
      reader.GetBeginNodeId(edge, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
    }

    {
      baldr::graph_tile_ptr tile;
      reader.AreEdgesConnectedForward(edge_id, opp_edge_id, tile);
      ASSERT_EQ(end_node_id.Tile_Base(), tile->id());
      reader.AreEdgesConnectedForward(edge_id, opp_edge_id, tile);
      ASSERT_EQ(end_node_id.Tile_Base(), tile->id());
    }

    {
      baldr::graph_tile_ptr tile;
      reader.nodeinfo(begin_node_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
      reader.nodeinfo(begin_node_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
    }

    {
      baldr::graph_tile_ptr tile;
      reader.directededge(edge_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
      reader.directededge(edge_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
    }

    {
      baldr::graph_tile_ptr tile;
      reader.GetDirectedEdgeNodes(edge_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
      reader.GetDirectedEdgeNodes(edge_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
    }

    {
      baldr::graph_tile_ptr tile;
      reader.edge_endnode(edge_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
      reader.edge_endnode(edge_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
    }

    {
      baldr::graph_tile_ptr tile;
      reader.edge_startnode(edge_id, tile);
      ASSERT_EQ(end_node_id.Tile_Base(), tile->id());
      reader.edge_startnode(edge_id, tile);
      ASSERT_EQ(end_node_id.Tile_Base(), tile->id());
    }

    {
      baldr::graph_tile_ptr tile;
      reader.edgeinfo(edge_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
      reader.edgeinfo(edge_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
    }

    {
      baldr::graph_tile_ptr tile;
      reader.GetTimezone(begin_node_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
      reader.GetTimezone(begin_node_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
    }

    {
      baldr::graph_tile_ptr tile;
      reader.GetIncidents(begin_node_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
      reader.GetIncidents(begin_node_id, tile);
      ASSERT_EQ(begin_node_id.Tile_Base(), tile->id());
    }

    // swap left and right
    std::swap(ltr, rtl);
    std::swap(l, r);
  }
}