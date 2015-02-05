#include "test.h"
#include "loki/search.h"

#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <unordered_set>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/mjolnir/graphtilebuilder.h>
#include <valhalla/baldr/tilehierarchy.h>
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

  /* this is what it looks like
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
  std::pair<GraphId, PointLL>
    b({tile_id.tileid(), tile_id.level(), 0}, {.01, .2}),
    a({tile_id.tileid(), tile_id.level(), 1}, {.01, .1}),
    c({tile_id.tileid(), tile_id.level(), 2}, {.01, .01}),
    d({tile_id.tileid(), tile_id.level(), 3}, {.2, .1});
  uint32_t edge_index = 0;

  auto add_node = [&edge_index] (const std::pair<GraphId, PointLL>& v, const uint32_t edge_count) {
    NodeInfoBuilder node_builder;
    node_builder.set_latlng(v.second);
    node_builder.set_bestrc(RoadClass::kSecondary);
    node_builder.set_edge_count(edge_count);
    node_builder.set_edge_index(edge_index);
    edge_index += edge_count;
    return node_builder;
  };
  auto add_edge = [&tile, &edge_index] (const std::pair<GraphId, PointLL>& u, const std::pair<GraphId, PointLL>& v,
    const uint32_t name, const uint32_t opposing) {

    DirectedEdgeBuilder edge_builder;
    edge_builder.set_length(u.second.Distance(v.second) + .5);
    edge_builder.set_endnode(v.first);
    //this is a brain F
    edge_builder.set_opp_index(opposing);
    bool add;
    uint32_t edge_info_offset = tile.AddEdgeInfo(name, u.first, v.first, {u.second, v.second}, {std::to_string(name)}, add);
    edge_builder.set_edgeinfo_offset(edge_info_offset);
    return edge_builder;
  };

  //B
  {
    auto node = add_node(b, 2);
    auto edges { add_edge(b, d, 0, 0), add_edge(b, a, 2, 0) };
    tile.AddNodeAndDirectedEdges(node, edges);
  }

  //A
  {
    auto node = add_node(a, 3);
    auto edges { add_edge(a, b, 2, 1), add_edge(a, d, 3, 1), add_edge(a, c, 1, 0) };
    tile.AddNodeAndDirectedEdges(node, edges);
  }

  //C
  {
    auto node = add_node(c, 2);
    auto edges { add_edge(c, a, 1, 2), add_edge(c, d, 4, 2) };
    tile.AddNodeAndDirectedEdges(node, edges);
  }

  //D
  {
    auto node = add_node(d, 3);
    auto edges { add_edge(d, b, 0, 0), add_edge(d, a, 3, 1), add_edge(d, c, 4, 1) };
    tile.AddNodeAndDirectedEdges(node, edges);
  }

  //write the tile
  tile.StoreTileData(h, tile_id);

  return conf;
}

void node_search(valhalla::baldr::GraphReader& reader, const valhalla::baldr::Location& location, const valhalla::midgard::PointLL& expected_point,
  const std::string& expected_name){
  valhalla::baldr::PathLocation p = valhalla::loki::Search(location, reader, valhalla::loki::PathThroughFilter, valhalla::loki::SearchStrategy::NODE);
  if(!p.IsCorrelated())
    throw std::runtime_error("Didn't find any node/edges");
  if(!p.IsNode())
    throw std::runtime_error("Node search should produce node results");
  if(!p.vertex().ApproximatelyEqual(expected_point))
    throw std::runtime_error("Found expected_point node");
  if(p.edges().front().dist != 0)
    throw std::runtime_error("Distance along the edge should always be 0 for node search");
  const GraphTile* tile = reader.GetGraphTile(location.latlng_);
  if(expected_name != tile->GetNames(tile->directededge(p.edges().front().id)->edgeinfo_offset())[0])
    throw std::runtime_error("Didn't find expected road name");
}

void TestNodeSearch() {
  auto conf = make_tile();
  valhalla::baldr::GraphReader reader(conf);

  node_search(reader, {{.01, .2}}, {.01, .2}, "0");
}

void edge_search(valhalla::baldr::GraphReader& reader, const valhalla::baldr::Location& location, const valhalla::midgard::PointLL& expected_point,
  const std::vector<std::string>& expected_names, const float expected_distance){
  valhalla::baldr::PathLocation p = valhalla::loki::Search(location, reader, valhalla::loki::PathThroughFilter, valhalla::loki::SearchStrategy::EDGE);
  if(!p.IsCorrelated())
    throw std::runtime_error("Didn't find any node/edges");
  if(p.IsNode() != (expected_distance == 0.f || expected_distance == 1.f))
    throw std::runtime_error("Edge search got unexpected IsNode result");
  if(!p.vertex().ApproximatelyEqual(expected_point))
    throw std::runtime_error("Found wrong point");
  if(expected_distance != 0.f && expected_distance != 1.f && p.edges().size() != 2)
    throw std::runtime_error("Unexpected number of correlated edges");

  const GraphTile* tile = reader.GetGraphTile(location.latlng_);
  std::unordered_set<std::string> found_names;
  for(const auto& edge : p.edges()) {
    if(!equal<float>(edge.dist, expected_distance) && !equal<float>(edge.dist, 1.f - expected_distance))
      throw std::runtime_error("Unexpected distance along the edge");
    auto names = tile->GetNames(tile->directededge(edge.id)->edgeinfo_offset());
    found_names.insert(names.begin(), names.end());
  }

  if(found_names.size() != expected_names.size())
    throw std::runtime_error("Wrong number of names found");
  for(const auto& name : expected_names) {
    auto found = found_names.find(name);
    if(found == found_names.end())
      throw std::runtime_error("Didn't find expected road name");
  }
}

void TestEdgeSearch() {
  auto conf = make_tile();
  valhalla::baldr::GraphReader reader(conf);

  edge_search(reader, {{.105, .1}}, {.105, .1}, {"3"}, .5);
  edge_search(reader, {{.01, .1}}, {.01, .1}, {"1", "2", "3"}, 0);
  edge_search(reader, {{.2, .1}}, {.2, .1}, {"0", "3", "4"}, 1);
  //TODO: add more elaborate shape to test better the nodesnapping and distance along shape segment stuff
}

}

int main() {
  test::suite suite("search");

  suite.test(TEST_CASE(TestNodeSearch));

  suite.test(TEST_CASE(TestEdgeSearch));

  return suite.tear_down();
}
