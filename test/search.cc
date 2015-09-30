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
#include <valhalla/midgard/vector2.h>
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
      {\"name\": \"arterial\", \"level\": 1, \"size\": 1, \"importance_cutoff\": \"Tertiary\"}, \
      {\"name\": \"highway\", \"level\": 0, \"size\": 4, \"importance_cutoff\": \"Trunk\"} \
    ] \
  }";
  boost::property_tree::json_parser::read_json(json, conf);

  //basic tile information
  TileHierarchy h(conf);
  GraphId tile_id = h.GetGraphId({.125,.125}, 2);
  GraphTileBuilder tile;
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
  auto add_edge = [&tile] (const std::pair<GraphId, PointLL>& u, const std::pair<GraphId, PointLL>& v,
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
    uint32_t edge_info_offset = tile.AddEdgeInfo(name, u.first, v.first, 123, shape, {std::to_string(name)}, add);
    edge_builder.set_edgeinfo_offset(edge_info_offset);
    return edge_builder;
  };

  /* this is what it looks like
    b
    |\
  1 | \ 0
    |  \
  2 |   \ 7
    |    \
    a-3-8-d
    |    /
  4 |   / 9
    |  /
  5 | / 6
    |/
    c
  */

  b.first = {tile_id.tileid(), tile_id.level(), 0}; b.second = {.01, .2};
  a.first = {tile_id.tileid(), tile_id.level(), 1}; a.second = {.01, .1};
  c.first = {tile_id.tileid(), tile_id.level(), 2}; c.second = {.01, .01};
  d.first = {tile_id.tileid(), tile_id.level(), 3}; d.second = {.2, .1};

  //NOTE: edge ids are in the order the edges are added, so b->d is 0, b->a is 1, a->b is 2 and so on

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

void search(const valhalla::baldr::Location& location, bool expected_node, const valhalla::midgard::PointLL& expected_point,
  const std::vector<PathLocation::PathEdge>& expected_edges, const valhalla::loki::SearchStrategy& strategy){

  valhalla::baldr::GraphReader reader(conf);
  valhalla::baldr::PathLocation p = valhalla::loki::Search(location, reader, valhalla::loki::PassThroughFilter, strategy);

  if(p.IsNode() != expected_node)
    throw std::runtime_error(expected_node ? "Should've snapped to node" : "Shouldn't've snapped to node");
  if(!p.IsCorrelated())
    throw std::runtime_error("Didn't find any node/edges");
  if(!p.vertex().ApproximatelyEqual(expected_point))
    throw std::runtime_error("Found wrong point");

  valhalla::baldr::PathLocation answer(location);
  answer.CorrelateVertex(expected_point);
  for(const auto& expected_edge : expected_edges)
    answer.CorrelateEdge(PathLocation::PathEdge{expected_edge.id, expected_edge.dist, expected_edge.sos});
  if(!(answer == p)) {
    throw std::runtime_error("Did not find expected edges");
  }
}

void TestNodeSearch() {
  auto t = a.first.tileid();
  auto l = a.first.level();
  using S = PathLocation::SideOfStreet;
  using PE = PathLocation::PathEdge;

  search({b.second}, true, b.second, { PE{{t, l, 0}, 0, S::NONE}, PE{{t, l, 1}, 0, S::NONE}, //leaving edges
                                 PE{{t, l, 7}, 1, S::NONE}, PE{{t, l, 2}, 1, S::NONE} }, valhalla::loki::SearchStrategy::NODE); //arriving edges

  search({a.second}, true, a.second, { PE{{t, l, 2}, 0, S::NONE}, PE{{t, l, 3}, 0, S::NONE}, PE{{t, l, 4}, 0, S::NONE}, //leaving edges
                                 PE{{t, l, 1}, 1, S::NONE}, PE{{t, l, 8}, 1, S::NONE}, PE{{t, l, 5}, 1, S::NONE} }, valhalla::loki::SearchStrategy::NODE); //arriving edges

  search({c.second}, true, c.second, { PE{{t, l, 5}, 0, S::NONE}, PE{{t, l, 6}, 0, S::NONE}, //leaving edges
                                 PE{{t, l, 4}, 1, S::NONE}, PE{{t, l, 9}, 1, S::NONE} }, valhalla::loki::SearchStrategy::NODE); //arriving edges

  search({d.second}, true, d.second, { PE{{t, l, 7}, 0, S::NONE}, PE{{t, l, 8}, 0, S::NONE}, PE{{t, l, 9}, 0, S::NONE}, //leaving edges
                                 PE{{t, l, 0}, 1, S::NONE}, PE{{t, l, 3}, 1, S::NONE}, PE{{t, l, 6}, 1, S::NONE} }, valhalla::loki::SearchStrategy::NODE); //arriving edges

  //slightly off the node
  search({d.second + PointLL(.0001, .0001)}, true, d.second, { PE{{t, l, 7}, 0, S::NONE}, PE{{t, l, 8}, 0, S::NONE}, PE{{t, l, 9}, 0, S::NONE}, //leaving edges
                                                         PE{{t, l, 0}, 1, S::NONE}, PE{{t, l, 3}, 1, S::NONE}, PE{{t, l, 6}, 1, S::NONE} }, valhalla::loki::SearchStrategy::NODE); //arriving edges

  //with heading
  Location x{a.second};
  x.heading_ = 90;    //leaving only
  search(x, true, a.second, { PE{{t, l, 3}, 0, S::NONE} }, valhalla::loki::SearchStrategy::NODE);
  x.heading_ = 270;   //arriving only
  search(x, true, a.second, { PE{{t, l, 8}, 1, S::NONE} }, valhalla::loki::SearchStrategy::NODE);
  x.heading_ = 265;   //leaving                    //arriving
  search(x, true, a.second, { PE{{t, l, 4}, 0, S::NONE}, PE{{t, l, 1}, 1, S::NONE}, PE{{t, l, 8}, 1, S::NONE} }, valhalla::loki::SearchStrategy::NODE);
}

void TestEdgeSearch() {
  auto t = a.first.tileid();
  auto l = a.first.level();
  using S = PathLocation::SideOfStreet;
  using PE = PathLocation::PathEdge;

  //degenerate to node searches
  search({a.second}, true, a.second, { PE{{t, l, 2}, 0, S::NONE}, PE{{t, l, 3}, 0, S::NONE}, PE{{t, l, 4}, 0, S::NONE} }, valhalla::loki::SearchStrategy::EDGE);
  search({d.second}, true, d.second, { PE{{t, l, 7}, 0, S::NONE}, PE{{t, l, 8}, 0, S::NONE}, PE{{t, l, 9}, 0, S::NONE} }, valhalla::loki::SearchStrategy::EDGE);
  //snap to end of edge degenerates to node searches
  search({{d.second.first + .049, d.second.second}}, true, d.second,
    { PE{{t, l, 7}, 0, S::NONE}, PE{{t, l, 8}, 0, S::NONE}, PE{{t, l, 9}, 0, S::NONE}, //leaving edges
      PE{{t, l, 0}, 1, S::NONE}, PE{{t, l, 3}, 1, S::NONE}, PE{{t, l, 6}, 1, S::NONE}  //arriving edges
    }, valhalla::loki::SearchStrategy::EDGE);

  //mid point search
  search({a.second.MidPoint(d.second)}, false, a.second.MidPoint(d.second), { PE{{t, l, 3}, .5f, S::NONE}, PE{{t, l, 8}, .5f, S::NONE} }, valhalla::loki::SearchStrategy::EDGE);

  //set a point 40% along the edge runs with the shape direction
  PointLL answer = a.second.AffineCombination(.6f, .4f, d.second);
  auto ratio = a.second.Distance(answer) / a.second.Distance(d.second);
  search({answer}, false, answer, { PE{{t, l, 3}, ratio, S::NONE}, PE{{t, l, 8}, 1.f - ratio, S::NONE} }, valhalla::loki::SearchStrategy::EDGE);

  //with heading
  Location x{answer};
  x.heading_ = 90;
  search(x, false, answer, { PE{{t, l, 3}, ratio, S::NONE} }, valhalla::loki::SearchStrategy::EDGE);
  x.heading_ = 0;
  search(x, false, answer, { PE{{t, l, 3}, ratio, S::NONE}, PE{{t, l, 8}, 1.f - ratio, S::NONE} }, valhalla::loki::SearchStrategy::EDGE);
  x.heading_ = 269;
  search(x, false, answer, { PE{{t, l, 8}, 1.f - ratio, S::NONE} }, valhalla::loki::SearchStrategy::EDGE);

  //check for side of street by offsetting the test point from the line orthogonally
  auto ortho = (d.second - a.second).GetPerpendicular(true).Normalize() * .01;
  PointLL test{answer.first + ortho.x(), answer.second + ortho.y()};
  search({test}, false, answer, { PE{{t, l, 3}, ratio, S::RIGHT}, PE{{t, l, 8}, 1.f - ratio, S::LEFT} }, valhalla::loki::SearchStrategy::EDGE);

  //set a point 40% along the edge that runs in reverse of the shape
  answer = b.second.AffineCombination(.6f, .4f, d.second);
  ratio = b.second.Distance(answer) / b.second.Distance(d.second);
  search({answer}, false, answer, { PE{{t, l, 0}, ratio, S::NONE}, PE{{t, l, 7}, 1.f - ratio, S::NONE} }, valhalla::loki::SearchStrategy::EDGE);

  //check for side of street by offsetting the test point from the line orthogonally
  ortho = (d.second - b.second).GetPerpendicular(false).Normalize() * .01;
  test.Set(answer.first + ortho.x(), answer.second + ortho.y());
  search({test}, false, answer, { PE{{t, l, 0}, ratio, S::LEFT}, PE{{t, l, 7}, 1.f - ratio, S::RIGHT} }, valhalla::loki::SearchStrategy::EDGE);

  //TODO: add test that has to snap a point not actually on the geometry
}

}

int main() {
  test::suite suite("search");

  suite.test(TEST_CASE(make_tile));

  suite.test(TEST_CASE(TestNodeSearch));

  suite.test(TEST_CASE(TestEdgeSearch));

  return suite.tear_down();
}
