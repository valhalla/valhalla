#include "test.h"
#include "loki/search.h"

#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/mjolnir/graphtilebuilder.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/exitsigninfo.h>
#include <valhalla/mjolnir/nodeinfobuilder.h>
#include <valhalla/mjolnir/directededgebuilder.h>

namespace {

boost::property_tree::ptree make_tile() {
  using namespace valhalla::mjolnir;
  using namespace valhalla::baldr;

  //make the config file
  std::stringstream json; json << "{ \
    \"tile_dir\": \"test/tiles\", \
    \"levels\": [ \
      {\"name\": \"local\", \"level\": 2, \"size\": 0.25}, \
      {\"name\": \"arterial\", \"level\": 1, \"size\": 1, \"importance_cutoff\": \"TertiaryUnclassified\"}, \
      {\"name\": \"highway\", \"level\": 0, \"size\": 4, \"importance_cutoff\": \"Trunk\"} \
    ] \
  }";
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(json, conf);

  //basic tile information
  TileHierarchy h(conf);
  GraphId tile_id = h.GetGraphId({.125,.125}, 2);

  /* all paths leave a and arrive at d
  b
  |\
 2| \0
  |  \
  a-3-d
  |  /
 1| /4
  |/
  c
  */

  GraphTileBuilder tile;
  GraphId b(tile_id.tileid(), tile_id.level(), 0),
          a(tile_id.tileid(), tile_id.level(), 1),
          c(tile_id.tileid(), tile_id.level(), 2),
          d(tile_id.tileid(), tile_id.level(), 3);
  bool add;

  //B
  {
    NodeInfoBuilder node_builder;
    node_builder.set_latlng({.01, .2});
    node_builder.set_bestrc(RoadClass::kSecondary);
    node_builder.set_edge_count(1);
    node_builder.set_edge_index(0);
    //3
    DirectedEdgeBuilder edge_builder;
    edge_builder.set_length(PointLL().Length({{.01, .2}, {.2, .1}}));
    edge_builder.set_endnode(d);
    uint32_t edge_info_offset = tile.AddEdgeInfo(0, b, d, {{.01, .2}, {.2, .1}}, {"0"}, {}, add);
    edge_builder.set_edgedataoffset(edge_info_offset);
    //add
    tile.AddNodeAndDirectedEdges(node_builder, {edge_builder});
  }

  //A
  {
    NodeInfoBuilder node_builder;
    node_builder.set_latlng({.01, .1});
    node_builder.set_bestrc(RoadClass::kSecondary);
    node_builder.set_edge_count(3);
    node_builder.set_edge_index(1);
    std::vector<DirectedEdgeBuilder> edges;
    //0
    {
      DirectedEdgeBuilder edge_builder;
      edge_builder.set_length(PointLL().Length({{.01, .1}, {.01, .01}}));
      edge_builder.set_endnode(c);
      uint32_t edge_info_offset = tile.AddEdgeInfo(0, a, c, {{.01, .1}, {.01, .01}}, {"1"}, {}, add);
      edge_builder.set_edgedataoffset(edge_info_offset);
      edges.emplace_back(std::move(edge_builder));
    }
    //1
    {
      DirectedEdgeBuilder edge_builder;
      edge_builder.set_length(PointLL().Length({{.01, .1}, {.01, .2}}));
      edge_builder.set_endnode(b);
      uint32_t edge_info_offset = tile.AddEdgeInfo(0, a, b, {{.01, .1}, {.01, .2}}, {"2"}, {}, add);
      edge_builder.set_edgedataoffset(edge_info_offset);
      edges.emplace_back(std::move(edge_builder));
    }
    //2
    {
      DirectedEdgeBuilder edge_builder;
      edge_builder.set_length(PointLL().Length({{.01, .1}, {.2, .1}}));
      edge_builder.set_endnode(d);
      uint32_t edge_info_offset = tile.AddEdgeInfo(0, a, d, {{.01, .1}, {.2, .1}}, {"3"}, {}, add);
      edge_builder.set_edgedataoffset(edge_info_offset);
      edges.emplace_back(std::move(edge_builder));
    }
    //add
    tile.AddNodeAndDirectedEdges(node_builder, edges);
  }

  //C
  {
    NodeInfoBuilder node_builder;
    node_builder.set_latlng({.01, .01});
    node_builder.set_bestrc(RoadClass::kSecondary);
    node_builder.set_edge_count(1);
    node_builder.set_edge_index(4);
    //3
    DirectedEdgeBuilder edge_builder;
    edge_builder.set_length(PointLL().Length({{.01, .01}, {.2, .1}}));
    edge_builder.set_endnode(d);
    uint32_t edge_info_offset = tile.AddEdgeInfo(0, c, d, {{.01, .01}, {.2, .1}}, {"4"}, {}, add);
    edge_builder.set_edgedataoffset(edge_info_offset);
    //add
    tile.AddNodeAndDirectedEdges(node_builder, {edge_builder});
  }

  //D
  {
    NodeInfoBuilder node_builder;
    node_builder.set_latlng({.2, .1});
    node_builder.set_bestrc(RoadClass::kSecondary);
    node_builder.set_edge_count(1);
    node_builder.set_edge_index(5);
    //add
    tile.AddNodeAndDirectedEdges(node_builder, {});
  }

  //write the tile
  tile.StoreTileData(h, tile_id);

  return conf;
}

void node_search(valhalla::baldr::GraphReader& reader, const valhalla::baldr::Location& location, const valhalla::midgard::PointLL& expected){
  valhalla::baldr::PathLocation p = valhalla::loki::Search(location, reader, valhalla::loki::SearchStrategy::NODE);
  if(!p.IsNode())
    throw std::runtime_error("Node search should produce node results");
  if(!p.vertex().ApproximatelyEqual(expected))
    throw std::runtime_error("Found wrong node");
}

void TestNodeSearch() {

  auto conf = make_tile();
  valhalla::baldr::GraphReader reader(conf);

  node_search(reader, {{.01, .2}}, {.01, .2});
}

}

int main() {
  test::suite suite("search");

  suite.test(TEST_CASE(TestNodeSearch));

  return suite.tear_down();
}
