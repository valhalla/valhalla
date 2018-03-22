#include <cstdint>
#include "test.h"
#include "loki/search.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>
#include <unordered_set>

#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/location.h"
#include "midgard/pointll.h"
#include "midgard/vector2.h"
#include "baldr/tilehierarchy.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;

#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/directededgebuilder.h"


namespace {

// this is what it looks like
//    b
//    |\
//  1 | \ 0
//    |  \
//  2 |   \ 7
//    |    \
//    a-3-8-d
//    |    /
//  4 |   / 9
//    |  /
//  5 | / 6
//    |/
//    c
std::string tile_dir = "test/search_tiles";
GraphId tile_id = TileHierarchy::GetGraphId({.125,.125}, 2);
std::pair<GraphId, PointLL> b({tile_id.tileid(), tile_id.level(), 0}, {.01, .2});
std::pair<GraphId, PointLL> a({tile_id.tileid(), tile_id.level(), 1}, {.01, .1});
std::pair<GraphId, PointLL> c({tile_id.tileid(), tile_id.level(), 2}, {.01, .01});
std::pair<GraphId, PointLL> d({tile_id.tileid(), tile_id.level(), 3}, {.2, .1});

void make_tile() {
  using namespace valhalla::mjolnir;
  using namespace valhalla::baldr;

  // make sure that all the old tiles are gone before trying to make new ones.
  if (boost::filesystem::is_directory(tile_dir)) {
    boost::filesystem::remove_all(tile_dir);
  }

  //basic tile information
  GraphTileBuilder tile(tile_dir, tile_id, false);
  uint32_t edge_index = 0;

  auto add_node = [&edge_index] (const std::pair<GraphId, PointLL>& v, const uint32_t edge_count) {
    NodeInfo node_builder;
    node_builder.set_latlng(v.second);
    node_builder.set_edge_count(edge_count);
    node_builder.set_edge_index(edge_index);
    edge_index += edge_count;
    return node_builder;
  };
  auto add_edge = [&tile] (const std::pair<GraphId, PointLL>& u, const std::pair<GraphId, PointLL>& v,
    const uint32_t name, const uint32_t opposing, const bool forward) {

    DirectedEdgeBuilder edge_builder({}, v.first, forward, u.second.Distance(v.second) + .5, 1, 1, 1, {}, {}, 0, false, 0, 0);
    edge_builder.set_opp_index(opposing);
    std::vector<PointLL> shape = {u.second, u.second.MidPoint(v.second), v.second};
    if(!forward)
      std::reverse(shape.begin(), shape.end());

    bool add;
    //make more complex edge geom so that there are 3 segments, affine combination doesnt properly handle arcs but who cares
    uint32_t edge_info_offset = tile.AddEdgeInfo(name, u.first, v.first, 123, shape, {std::to_string(name)}, 0, add);
    edge_builder.set_edgeinfo_offset(edge_info_offset);
    return edge_builder;
  };

  // this is what it looks like
  //    b
  //    |\
  //  1 | \ 0
  //    |  \
  //  2 |   \ 7
  //    |    \
  //    a-3-8-d
  //    |    /
  //  4 |   / 9
  //    |  /
  //  5 | / 6
  //    |/
  //    c

  //NOTE: edge ids are in the order the edges are added, so b->d is 0, b->a is 1, a->b is 2 and so on

  //B
  {
    tile.directededges().emplace_back(add_edge(b, d, 0, 0, false)); //0
    tile.directededges().emplace_back(add_edge(b, a, 2, 0, true));  //1
    tile.nodes().emplace_back(add_node(b, 2));
  }

  //A
  {
    tile.directededges().emplace_back(add_edge(a, b, 2, 1, false)); //2
    tile.directededges().emplace_back(add_edge(a, d, 3, 1, true));  //3
    tile.directededges().emplace_back(add_edge(a, c, 1, 0, false)); //4
    tile.nodes().emplace_back(add_node(a, 3));
  }

  //C
  {
    tile.directededges().emplace_back(add_edge(c, a, 1, 2, true)); //5
    tile.directededges().emplace_back(add_edge(c, d, 4, 2, false));//6
    tile.nodes().emplace_back(add_node(c, 2));
  }

  //D
  {
    tile.directededges().emplace_back(add_edge(d, b, 0, 0, true)); //7
    tile.directededges().emplace_back(add_edge(d, a, 3, 1, false));//8
    tile.directededges().emplace_back(add_edge(d, c, 4, 1, true)); //9
    tile.nodes().emplace_back(add_node(d, 3));
  }

  //write the tile
  tile.StoreTileData();

  //write the bin data
  GraphTileBuilder::tweeners_t tweeners;
  GraphTile reloaded(tile_dir, tile_id);
  auto bins = GraphTileBuilder::BinEdges(&reloaded, tweeners);
  GraphTileBuilder::AddBins(tile_dir, &reloaded, bins);
}

void search(const valhalla::baldr::Location& location, bool expected_node, const valhalla::midgard::PointLL& expected_point,
  const std::vector<PathLocation::PathEdge>& expected_edges, bool exact = false){
  //make the config file
  boost::property_tree::ptree conf;
  conf.put("tile_dir", tile_dir);

  valhalla::baldr::GraphReader reader(conf);
  const auto results = Search({location}, reader, PassThroughEdgeFilter, PassThroughNodeFilter);
  const auto p = results.at(location);

  if((p.edges.front().begin_node() || p.edges.front().end_node()) != expected_node)
    throw std::runtime_error(expected_node ? "Should've snapped to node" : "Shouldn't've snapped to node");
  if(!p.edges.size())
    throw std::runtime_error("Didn't find any node/edges");
  if(!p.edges.front().projected.ApproximatelyEqual(expected_point))
    throw std::runtime_error("Found wrong point");

  valhalla::baldr::PathLocation answer(location);
  for(const auto& expected_edge : expected_edges) {
    answer.edges.emplace_back(PathLocation::PathEdge{expected_edge.id, expected_edge.percent_along,
      expected_point, expected_point.Distance(location.latlng_), expected_edge.sos});
  }
  //note that this just checks that p has the edges that answer has
  //p can have more edges than answer has and that wont fail this check!
  if(!(answer == p))
    throw std::runtime_error("Did not find expected edges");
  //if you want to enforce that the result didnt have more then expected
  if(exact && answer.edges.size() != p.edges.size())
    throw std::logic_error("Got more edges than expected");
}

void search(const valhalla::baldr::Location& location, size_t result_count, int reachability) {
  //make the config file
  boost::property_tree::ptree conf;
  conf.put("tile_dir", tile_dir);

  valhalla::baldr::GraphReader reader(conf);
  const auto p = Search({location}, reader, PassThroughEdgeFilter, PassThroughNodeFilter).at(location);

  if(p.edges.size() != result_count)
    throw std::logic_error("Wrong number of edges");
  for(const auto& e : p.edges)
    if(e.minimum_reachability != reachability)
      throw std::logic_error("Wrong reachability");
}

void test_edge_search() {
  auto t = a.first.tileid();
  auto l = a.first.level();
  using S = PathLocation::SideOfStreet;
  using PE = PathLocation::PathEdge;

  //snap to node searches
  search({a.second}, true, a.second, { PE{{t, l, 2}, 0, a.second, 0, S::NONE}, PE{{t, l, 3}, 0, a.second, 0, S::NONE}, PE{{t, l, 4}, 0, a.second, 0, S::NONE} });
  search({d.second}, true, d.second, { PE{{t, l, 7}, 0, d.second, 0, S::NONE}, PE{{t, l, 8}, 0, d.second, 0, S::NONE}, PE{{t, l, 9}, 0, d.second, 0, S::NONE} });
  //snap to end of edge degenerates to node searches
  search({{d.second.first + .049, d.second.second}}, true, d.second,
    { PE{{t, l, 7}, 0, d.second, 0, S::NONE}, PE{{t, l, 8}, 0, d.second, 0, S::NONE}, PE{{t, l, 9}, 0, d.second, 0, S::NONE}, //leaving edges
      PE{{t, l, 0}, 1, d.second, 0, S::NONE}, PE{{t, l, 3}, 1, d.second, 0, S::NONE}, PE{{t, l, 6}, 1, d.second, 0, S::NONE}  //arriving edges
    });
  //snap to node as through location should be all edges
  Location x{a.second, Location::StopType::THROUGH};
  search(x, true, a.second,
    { PE{{t, l, 2}, 0, a.second, 0, S::NONE}, PE{{t, l, 3}, 0, a.second, 0, S::NONE}, PE{{t, l, 4}, 0, a.second, 0, S::NONE},
      PE{{t, l, 1}, 1, a.second, 0, S::NONE}, PE{{t, l, 8}, 1, a.second, 0, S::NONE}, PE{{t, l, 5}, 1, a.second, 0, S::NONE}
    }, true);
  //with a heading should just be a single outgoing
  x.heading_ = 180;
  search(x, true, a.second, { PE{{t, l, 4}, 0, a.second, 0, S::NONE} }, true);

  //mid point search
  auto answer = a.second.MidPoint(d.second);
  search({a.second.MidPoint(d.second)}, false, a.second.MidPoint(d.second), { PE{{t, l, 3}, .5f, answer, 0, S::NONE}, PE{{t, l, 8}, .5f, answer, 0, S::NONE} });

  //set a point 40% along the edge runs with the shape direction
  answer = a.second.AffineCombination(.6f, .4f, d.second);
  auto ratio = a.second.Distance(answer) / a.second.Distance(d.second);
  x = {answer};
  search(x, false, answer, { PE{{t, l, 3}, ratio, answer, 0, S::NONE}, PE{{t, l, 8}, 1.f - ratio, answer, 0, S::NONE} });

  //with heading
  x.heading_ = 90;
  search(x, false, answer, { PE{{t, l, 3}, ratio, answer, 0, S::NONE} });
  x.heading_ = 0;
  search(x, false, answer, { PE{{t, l, 3}, ratio, answer, 0, S::NONE}, PE{{t, l, 8}, 1.f - ratio, answer, 0, S::NONE} });
  x.heading_ = 269;
  search(x, false, answer, { PE{{t, l, 8}, 1.f - ratio, answer, 0, S::NONE} });

  //check for side of street by offsetting the test point from the line orthogonally
  auto ortho = (d.second - a.second).GetPerpendicular(true).Normalize() * .01;
  PointLL test{answer.first + ortho.x(), answer.second + ortho.y()};
  search({test}, false, answer, { PE{{t, l, 3}, ratio, answer, 0, S::RIGHT}, PE{{t, l, 8}, 1.f - ratio, answer, 0, S::LEFT} });

  //set a point 40% along the edge that runs in reverse of the shape
  answer = b.second.AffineCombination(.6f, .4f, d.second);
  ratio = b.second.Distance(answer) / b.second.Distance(d.second);
  search({answer}, false, answer, { PE{{t, l, 0}, ratio, answer, 0, S::NONE}, PE{{t, l, 7}, 1.f - ratio, answer, 0, S::NONE} });

  //check for side of street by offsetting the test point from the line orthogonally
  ortho = (d.second - b.second).GetPerpendicular(false).Normalize() * .01;
  test.Set(answer.first + ortho.x(), answer.second + ortho.y());
  search({test}, false, answer, { PE{{t, l, 0}, ratio, answer, 0, S::LEFT}, PE{{t, l, 7}, 1.f - ratio, answer, 0, S::RIGHT} });

  //TODO: add more tests
}

void test_reachability_radius() {
  PointLL ob(b.second.first - .001f, b.second.second - .01f);
  unsigned int longest = ob.Distance(d.second);
  unsigned int shortest = ob.Distance(a.second);

  //zero everything should be a single closest result
  search({ob, Location::StopType::BREAK, 0, 0}, 2, 0);

  //set radius high to get them all
  search({b.second,Location::StopType::BREAK, 0, longest + 100}, 10, 0);

  //set radius mid to get just some
  search({b.second,Location::StopType::BREAK, 0, shortest - 100}, 4, 0);

  //set reachability high to see it gets all nodes reachable
  search({ob, Location::StopType::BREAK, 5, 0}, 2, 4);

  //set reachability right on to see we arent off by one
  search({ob, Location::StopType::BREAK, 4, 0}, 2, 4);

  //set reachability lower to see we give up early
  search({ob, Location::StopType::BREAK, 3, 0}, 2, 3);
}

}

int main() {
  test::suite suite("search");

  suite.test(TEST_CASE(make_tile));

  suite.test(TEST_CASE(test_edge_search));

  suite.test(TEST_CASE(test_reachability_radius));

  return suite.tear_down();
}
