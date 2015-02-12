#include "test.h"
#include "loki/search.h"

#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <unordered_set>
#include <algorithm>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/mjolnir/graphtilebuilder.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/mjolnir/nodeinfobuilder.h>
#include <valhalla/mjolnir/directededgebuilder.h>

namespace {

std::pair<GraphId, PointLL>
    b({}, {}),
    a({}, {}),
    c({}, {}),
    d({}, {});

boost::property_tree::ptree conf;

void make_tile() {
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
  b.first = {tile_id.tileid(), tile_id.level(), 0}; b.second = {.01, .2};
  a.first = {tile_id.tileid(), tile_id.level(), 1}; a.second = {.01, .1};
  c.first = {tile_id.tileid(), tile_id.level(), 2}; c.second = {.01, .01};
  d.first = {tile_id.tileid(), tile_id.level(), 3}; d.second = {.2, .1};
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
    const uint32_t name, const uint32_t opposing, const bool forward) {

    DirectedEdgeBuilder edge_builder;
    edge_builder.set_length(u.second.Distance(v.second) + .5);
    edge_builder.set_endnode(v.first);
    edge_builder.set_opp_index(opposing);
    edge_builder.set_forward(forward);
    std::vector<PointLL> shape = {u.second, u.second.AffineCombination(.7f, .3f, v.second), u.second.AffineCombination(.3f, .7f, v.second), v.second};
    if(!forward)
      std::reverse(shape.begin(), shape.end());

    bool add;
    //make more complex edge geom so that there are 3 segments, affine combination doesnt properly handle arcs but who cares
    uint32_t edge_info_offset = tile.AddEdgeInfo(name, u.first, v.first, shape, {std::to_string(name)}, add);
    edge_builder.set_edgeinfo_offset(edge_info_offset);
    return edge_builder;
  };

  //B
  {
    auto node = add_node(b, 2);
    auto edges { add_edge(b, d, 0, 0, false), add_edge(b, a, 2, 0, true) };
    tile.AddNodeAndDirectedEdges(node, edges);
  }

  //A
  {
    auto node = add_node(a, 3);
    auto edges { add_edge(a, b, 2, 1, false), add_edge(a, d, 3, 1, true), add_edge(a, c, 1, 0, false) };
    tile.AddNodeAndDirectedEdges(node, edges);
  }

  //C
  {
    auto node = add_node(c, 2);
    auto edges { add_edge(c, a, 1, 2, true), add_edge(c, d, 4, 2, false) };
    tile.AddNodeAndDirectedEdges(node, edges);
  }

  //D
  {
    auto node = add_node(d, 3);
    auto edges { add_edge(d, b, 0, 0, true), add_edge(d, a, 3, 1, false), add_edge(d, c, 4, 1, true) };
    tile.AddNodeAndDirectedEdges(node, edges);
  }

  //write the tile
  tile.StoreTileData(h, tile_id);
}

void node_search(const valhalla::baldr::Location& location, const valhalla::midgard::PointLL& expected_point,
  const std::vector<std::string>& expected_names){

  valhalla::baldr::GraphReader reader(conf);
  valhalla::baldr::PathLocation p = valhalla::loki::Search(location, reader, valhalla::loki::PathThroughFilter, valhalla::loki::SearchStrategy::NODE);

  if(!p.IsCorrelated())
    throw std::runtime_error("Didn't find any node/edges");
  if(!p.IsNode())
    throw std::runtime_error("Node search should produce node results");
  if(!p.vertex().ApproximatelyEqual(expected_point))
    throw std::runtime_error("Found unexpected_point node");
  const GraphTile* tile = reader.GetGraphTile(location.latlng_);
  std::unordered_set<std::string> found_names;
  for(const auto& edge : p.edges()) {
    if(!equal<float>(edge.dist, static_cast<float>(!tile->directededge(edge.id)->forward())))
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

void edge_search(const valhalla::baldr::Location& location, const valhalla::midgard::PointLL& expected_point,
  const std::vector<std::pair<GraphId, float> >& expected_edges){

  valhalla::baldr::GraphReader reader(conf);
  valhalla::baldr::PathLocation p = valhalla::loki::Search(location, reader, valhalla::loki::PathThroughFilter, valhalla::loki::SearchStrategy::EDGE);

  if(!p.IsCorrelated())
    throw std::runtime_error("Didn't find any node/edges");
  if(!p.vertex().ApproximatelyEqual(expected_point))
    throw std::runtime_error("Found wrong point");


  valhalla::baldr::PathLocation answer(location);
  answer.CorrelateVertex(expected_point);
  for(const auto& expected_edge : expected_edges)
    answer.CorrelateEdge(expected_edge.first, expected_edge.second);
  if(!(answer == p)) {
    throw std::runtime_error("Did not find expected edges");
  }
}

void TestNodeSearch() {
  node_search({b.second}, b.second, {"0", "2"});
}

void TestEdgeSearch() {
  auto t = a.first.tileid();
  auto l = a.first.level();

  edge_search({a.second}, a.second, { {{t, l, 2}, 1}, {{t, l, 3}, 0}, {{t, l, 4}, 1} });
  edge_search({d.second}, d.second, { {{t, l, 7}, 0}, {{t, l, 8}, 1}, {{t, l, 9}, 0} });
  edge_search({a.second.MidPoint(d.second)}, a.second.MidPoint(d.second), { {{t, l, 3}, .5f}, {{t, l, 8}, .5f} });
  PointLL answer = a.second.AffineCombination(.6f, .4f, d.second);
  auto ratio = a.second.Distance(answer) / a.second.Distance(d.second);
  edge_search({answer}, answer, { {{t, l, 3}, ratio}, {{t, l, 8}, 1.f - ratio} });
  answer = b.second.AffineCombination(.6f, .4f, d.second);
  ratio = b.second.Distance(answer) / b.second.Distance(d.second);
  edge_search({answer}, answer, { {{t, l, 0}, ratio}, {{t, l, 7}, 1.f - ratio} });
}

}

int main() {
  test::suite suite("search");

  suite.test(TEST_CASE(make_tile));

  suite.test(TEST_CASE(TestNodeSearch));

  suite.test(TEST_CASE(TestEdgeSearch));

  return suite.tear_down();
}
