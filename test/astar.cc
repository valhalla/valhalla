#include "test.h"
#include "thor/astar.h"

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/vector2.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/sif/pedestriancost.h>
#include <valhalla/sif/costconstants.h>
#include <valhalla/thor/astar.h>

namespace bpt = boost::property_tree;

namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vs = valhalla::sif;
namespace vt = valhalla::thor;

/*
 * to regenerate the test tile you'll want to:
 *  add -I../mjolnir to the compile line
 *  add ../mjolnir/libvalhalla_mjolnir.la to the link line
 *  uncomment the define MAKE_TEST_TILES
 *  and delete the test tile: test/fake_tiles_astar/2/000/519/120.gph
 */

// #define MAKE_TEST_TILES

#ifdef MAKE_TEST_TILES
#include <valhalla/mjolnir/graphtilebuilder.h>
#include <valhalla/mjolnir/directededgebuilder.h>
#endif /* MAKE_TEST_TILES */

namespace {

// ph34r the ASCII art diagram:
//
//       0  2
// a----->--<-----b
// |              |
// v 1          3 v
// |              |
// ^ 4          7 ^
// |              |
// c----->--<-----d
//       5  6
//
vb::TileHierarchy h("test/fake_tiles_astar");
vb::GraphId tile_id = h.GetGraphId({.125, .125}, 2);
std::pair<vb::GraphId, vm::PointLL> a({tile_id.tileid(), tile_id.level(), 0}, {0.01, 0.10});
std::pair<vb::GraphId, vm::PointLL> b({tile_id.tileid(), tile_id.level(), 1}, {0.10, 0.10});
std::pair<vb::GraphId, vm::PointLL> c({tile_id.tileid(), tile_id.level(), 2}, {0.01, 0.01});
std::pair<vb::GraphId, vm::PointLL> d({tile_id.tileid(), tile_id.level(), 3}, {0.10, 0.01});

#ifdef MAKE_TEST_TILES
void make_tile() {
  using namespace valhalla::mjolnir;

  GraphTileBuilder tile(h, tile_id, false);
  uint32_t edge_index = 0;

  auto add_node = [&](const std::pair<vb::GraphId, vm::PointLL>& v, const uint32_t edge_count) {
    NodeInfo node_builder;
    node_builder.set_latlng(v.second);
    //node_builder.set_road_class(RoadClass::kSecondary);
    node_builder.set_edge_count(edge_count);
    node_builder.set_edge_index(edge_index);
    edge_index += edge_count;
    tile.nodes().emplace_back(std::move(node_builder));
  };

  auto add_edge = [&](const std::pair<vb::GraphId, vm::PointLL>& u, const std::pair<vb::GraphId, vm::PointLL>& v,
                      const uint32_t name, const uint32_t opposing, const bool forward) {
    DirectedEdgeBuilder edge_builder({}, v.first, forward, u.second.Distance(v.second) + .5, 1, 1, {}, {}, 0, false, 0, 0);
    edge_builder.set_opp_index(opposing);
    std::vector<vm::PointLL> shape = {u.second, u.second.MidPoint(v.second), v.second};
    if(!forward)
      std::reverse(shape.begin(), shape.end());
    bool add;
    //make more complex edge geom so that there are 3 segments, affine combination doesnt properly handle arcs but who cares
    uint32_t edge_info_offset = tile.AddEdgeInfo(name, u.first, v.first, 123, shape, {std::to_string(name)}, add);
    edge_builder.set_edgeinfo_offset(edge_info_offset);
    tile.directededges().emplace_back(std::move(edge_builder));
  };

  // node a
  add_edge(a, b, 0, 2, true);
  add_edge(a, c, 1, 4, true);
  add_node(a, 2);

  add_edge(b, a, 0, 0, false);
  add_edge(b, d, 2, 7, true);
  add_node(b, 2);

  add_edge(c, a, 1, 1, false);
  add_edge(c, d, 3, 6, true);
  add_node(c, 2);

  add_edge(d, c, 3, 5, false);
  add_edge(d, b, 2, 3, false);
  add_node(d, 2);

  tile.StoreTileData();

  GraphTileBuilder::tweeners_t tweeners;
  GraphTile reloaded(h, tile_id);
  auto bins = GraphTileBuilder::BinEdges(h, &reloaded, tweeners);
  GraphTileBuilder::AddBins(h, &reloaded, bins);
}
#endif /* MAKE_TEST_TILES */

// test that a path from A to B succeeds, even if the edges from A to C and B
// to D appear first in the PathLocation.
void TestTrivialPath() {
  //make the config file
  std::stringstream json;
  json << "{ \"tile_dir\": \"test/fake_tiles_astar\" }";
  bpt::ptree conf;
  bpt::json_parser::read_json(json, conf);

  vb::GraphReader reader(conf);
  auto *tile = reader.GetGraphTile(tile_id);
  if (tile == nullptr) {
    throw std::runtime_error("Unable to load test tile! Please read the comment at the top of this file about generating the test tiles.");
  }

  vb::PathLocation origin(a.second);
  origin.edges.emplace_back(tile_id + uint64_t(1), 0.0f, a.second, 0.0f);
  origin.edges.emplace_back(tile_id + uint64_t(4), 1.0f, a.second, 0.0f);
  origin.edges.emplace_back(tile_id + uint64_t(0), 0.0f, a.second, 0.0f);
  origin.edges.emplace_back(tile_id + uint64_t(2), 1.0f, a.second, 0.0f);

  vb::PathLocation dest(b.second);
  dest.edges.emplace_back(tile_id + uint64_t(3), 0.0f, b.second, 0.0f);
  dest.edges.emplace_back(tile_id + uint64_t(7), 1.0f, b.second, 0.0f);
  dest.edges.emplace_back(tile_id + uint64_t(2), 0.0f, b.second, 0.0f);
  dest.edges.emplace_back(tile_id + uint64_t(0), 1.0f, b.second, 0.0f);

  auto mode = vs::TravelMode::kPedestrian;
  vs::cost_ptr_t costs[int(vs::TravelMode::kMaxTravelMode)];
  auto pedestrian = vs::CreatePedestrianCost(bpt::ptree());
  costs[int(mode)] = pedestrian;
  assert(bool(costs[int(mode)]));

  vt::AStarPathAlgorithm astar;
  auto path = astar.GetBestPath(origin, dest, reader, costs, mode);

  uint32_t time = 0;
  for (auto &p : path) {
    time += p.elapsed_time;
  }

  auto cost = pedestrian->EdgeCost(tile->directededge(0));
  uint32_t expected_time = cost.cost;
  if (time != expected_time) {
    std::ostringstream ostr;
    ostr << "Expected " << expected_time << ", but got " << time;
    throw std::runtime_error(ostr.str());
  }
}

} // anonymous namespace

int main() {
  test::suite suite("astar");

#ifdef MAKE_TEST_TILES
  // TODO: move to mjolnir?
  suite.test(TEST_CASE(make_tile));
#endif /* MAKE_TEST_TILES */

  suite.test(TEST_CASE(TestTrivialPath));

  return suite.tear_down();
}
