#include <cstdint>
#include "test.h"

#include "loki/search.h"
#include "mjolnir/pbfgraphparser.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/graphenhancer.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/graphvalidator.h"
#include "odin/directionsbuilder.h"
#include "midgard/pointll.h"
#include "midgard/vector2.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/location.h"
#include "baldr/tilehierarchy.h"
#include "sif/pedestriancost.h"
#include "sif/costconstants.h"
#include "sif/dynamiccost.h"
#include "thor/astar.h"
#include "thor/trippathbuilder.h"
#include "thor/attributes_controller.h"
#include "proto/trippath.pb.h"
#include "proto/tripdirections.pb.h"
#include "proto/directions_options.pb.h"

#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

namespace bpt = boost::property_tree;

namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vs = valhalla::sif;
namespace vt = valhalla::thor;
namespace vk = valhalla::loki;
namespace vj = valhalla::mjolnir;
namespace vo = valhalla::odin;


/*
 * to regenerate the test tile you'll want to:
 *  add -I../mjolnir to the compile line
 *  add ../mjolnir/libvalhalla_mjolnir.la to the link line
 *  uncomment the define MAKE_TEST_TILES
 *  and delete the test tile: test/fake_tiles_astar/2/000/519/120.gph
 */

// #define MAKE_TEST_TILES

#ifdef MAKE_TEST_TILES
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/directededgebuilder.h"
#endif /* MAKE_TEST_TILES */

namespace {

// ph34r the ASCII art diagram:
//
// first test is just a square set of roads
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
// second test is a triangle set of roads, where the height of the triangle is
// about at third of its width.
//
//      8  10
//  e--->--<---f
//  \         /
// 9 v       v 11
//    \     /
//  12 ^   ^ 13
//      \ /
//       g
//
std::string test_dir = "test/fake_tiles_astar";
vb::GraphId tile_id = vb::TileHierarchy::GetGraphId({.125, .125}, 2);

namespace node {
std::pair<vb::GraphId, vm::PointLL> a({tile_id.tileid(), tile_id.level(), 0}, {0.01, 0.10});
std::pair<vb::GraphId, vm::PointLL> b({tile_id.tileid(), tile_id.level(), 1}, {0.10, 0.10});
std::pair<vb::GraphId, vm::PointLL> c({tile_id.tileid(), tile_id.level(), 2}, {0.01, 0.01});
std::pair<vb::GraphId, vm::PointLL> d({tile_id.tileid(), tile_id.level(), 3}, {0.10, 0.01});

std::pair<vb::GraphId, vm::PointLL> e({tile_id.tileid(), tile_id.level(), 4}, {0.01, 0.14});
std::pair<vb::GraphId, vm::PointLL> f({tile_id.tileid(), tile_id.level(), 5}, {0.10, 0.14});
std::pair<vb::GraphId, vm::PointLL> g({tile_id.tileid(), tile_id.level(), 6}, {0.05, 0.11});
} // namespace node

#ifdef MAKE_TEST_TILES
void make_tile() {
  using namespace valhalla::mjolnir;

  GraphTileBuilder tile(h, tile_id, false);
  uint32_t edge_index = 0;

  auto add_node = [&](const std::pair<vb::GraphId, vm::PointLL>& v, const uint32_t edge_count) {
    NodeInfo node_builder;
    node_builder.set_latlng(v.second);
    //node_builder.set_road_class(RoadClass::kSecondary);
    node_builder.set_access(vb::kAllAccess);
    node_builder.set_edge_count(edge_count);
    node_builder.set_edge_index(edge_index);
    edge_index += edge_count;
    tile.nodes().emplace_back(std::move(node_builder));
  };

  auto add_edge = [&](const std::pair<vb::GraphId, vm::PointLL>& u, const std::pair<vb::GraphId, vm::PointLL>& v,
                      const uint32_t name, const uint32_t opposing, const bool forward) {
    DirectedEdgeBuilder edge_builder({}, v.first, forward, u.second.Distance(v.second) + .5, 1, 1, 1, {}, {}, 0, false, 0, 0);
    edge_builder.set_opp_index(opposing);
    edge_builder.set_forwardaccess(vb::kAllAccess);
    std::vector<vm::PointLL> shape = {u.second, u.second.MidPoint(v.second), v.second};
    if(!forward)
      std::reverse(shape.begin(), shape.end());
    bool add;
    //make more complex edge geom so that there are 3 segments, affine combination doesnt properly handle arcs but who cares
    uint32_t edge_info_offset = tile.AddEdgeInfo(name, u.first, v.first, 123, shape, {std::to_string(name)}, add);
    edge_builder.set_edgeinfo_offset(edge_info_offset);
    tile.directededges().emplace_back(std::move(edge_builder));
  };

  // first set of roads
  add_edge(node::a, node::b, 0, 2, true);
  add_edge(node::a, node::c, 1, 4, true);
  add_node(node::a, 2);

  add_edge(node::b, node::a, 0, 0, false);
  add_edge(node::b, node::d, 2, 7, true);
  add_node(node::b, 2);

  add_edge(node::c, node::a, 1, 1, false);
  add_edge(node::c, node::d, 3, 6, true);
  add_node(node::c, 2);

  add_edge(node::d, node::c, 3, 5, false);
  add_edge(node::d, node::b, 2, 3, false);
  add_node(node::d, 2);

  // second set of roads
  add_edge(node::e, node::f, 4, 10, true);
  add_edge(node::e, node::g, 5, 12, true);
  add_node(node::e, 2);

  add_edge(node::f, node::e, 4,  8, false);
  add_edge(node::f, node::g, 6, 13, true);
  add_node(node::f, 2);

  add_edge(node::g, node::e, 5,  9, false);
  add_edge(node::g, node::f, 6, 11, false);
  add_node(node::g, 2);

  tile.StoreTileData();

  GraphTileBuilder::tweeners_t tweeners;
  GraphTile reloaded(h, tile_id);
  auto bins = GraphTileBuilder::BinEdges(h, &reloaded, tweeners);
  GraphTileBuilder::AddBins(h, &reloaded, bins);
}
#endif /* MAKE_TEST_TILES */

const std::string config_file = "test/test_trivial_path";

void write_config(const std::string& filename) {
  std::ofstream file;
  try {
    file.open(filename, std::ios_base::trunc);
    file << "{ \
      \"mjolnir\": { \
      \"concurrency\": 1, \
       \"tile_dir\": \"test/data/trivial_tiles\", \
        \"admin\": \"test/data/netherlands_admin.sqlite\", \
         \"timezone\": \"test/data/not_needed.sqlite\" \
      } \
    }";
  }
  catch(...) {

  }
  file.close();
}

// check that a path from origin to dest goes along the edge with expected_edge_index
void assert_is_trivial_path(
  vo::Location &origin,
  vo::Location &dest,
  uint32_t expected_edge_index) {

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

  auto cost = pedestrian->EdgeCost(tile->directededge(expected_edge_index));
  uint32_t expected_time = cost.cost;
  if (time != expected_time) {
    std::ostringstream ostr;
    ostr << "Expected " << expected_time << ", but got " << time;
    throw std::runtime_error(ostr.str());
  }
}

void add(GraphId id, float dist, PointLL ll, vo::Location& l) {
  l.mutable_path_edges()->Add()->set_graph_id(id);
  l.mutable_path_edges()->rbegin()->set_percent_along(dist);
  l.mutable_path_edges()->rbegin()->mutable_ll()->set_lng(ll.first);
  l.mutable_path_edges()->rbegin()->mutable_ll()->set_lat(ll.second);
  l.mutable_path_edges()->rbegin()->set_distance(0.0f);
}

// test that a path from A to B succeeds, even if the edges from A to C and B
// to D appear first in the PathLocation.
void TestTrivialPath() {
  using node::a;
  using node::b;

  vo::Location origin;
  origin.mutable_ll()->set_lng(a.second.first);
  origin.mutable_ll()->set_lat(a.second.second);
  add(tile_id + uint64_t(1), 0.0f, a.second, origin);
  add(tile_id + uint64_t(4), 1.0f, a.second, origin);
  add(tile_id + uint64_t(0), 0.0f, a.second, origin);
  add(tile_id + uint64_t(2), 1.0f, a.second, origin);

  vo::Location dest;
  dest.mutable_ll()->set_lng(b.second.first);
  dest.mutable_ll()->set_lat(b.second.second);
  add(tile_id + uint64_t(3), 0.0f, b.second, dest);
  add(tile_id + uint64_t(7), 1.0f, b.second, dest);
  add(tile_id + uint64_t(2), 0.0f, b.second, dest);
  add(tile_id + uint64_t(0), 1.0f, b.second, dest);

  // this should go along the path from A to B
  assert_is_trivial_path(origin, dest, 0);
}

// test that a path from E to F succeeds, even if the edges from E and F
// to G appear first in the PathLocation.
void TestTrivialPathTriangle() {
  using node::e;
  using node::f;

  vo::Location origin;
  origin.mutable_ll()->set_lng(e.second.first);
  origin.mutable_ll()->set_lat(e.second.second);
  add(tile_id + uint64_t(9), 0.0f, e.second, origin);
  add(tile_id + uint64_t(12), 1.0f, e.second, origin);
  add(tile_id + uint64_t(8), 0.0f, e.second, origin);
  add(tile_id + uint64_t(10), 1.0f, e.second, origin);

  vo::Location dest;
  dest.mutable_ll()->set_lng(f.second.first);
  dest.mutable_ll()->set_lat(f.second.second);
  add(tile_id + uint64_t(11), 0.0f, f.second, dest);
  add(tile_id + uint64_t(13), 1.0f, f.second, dest);
  add(tile_id + uint64_t(10), 0.0f, f.second, dest);
  add(tile_id + uint64_t(8), 1.0f, f.second, dest);

  // this should go along the path from E to F
  assert_is_trivial_path(origin, dest, 8);
}

void trivial_path_no_uturns(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);

  //setup and purge
  vb::GraphReader graph_reader(conf.get_child("mjolnir"));
  for(const auto& level : vb::TileHierarchy::levels()) {
    auto level_dir = graph_reader.tile_dir() + "/" + std::to_string(level.first);
    if(boost::filesystem::exists(level_dir) && !boost::filesystem::is_empty(level_dir)) {
      boost::filesystem::remove_all(level_dir);
    }
  }

  std::string ways_file = "test_ways_trivial.bin";
  std::string way_nodes_file = "test_way_nodes_trivial.bin";
  std::string access_file = "test_access_trivial.bin";
  std::string restriction_file = "test_complex_restrictions_trivial.bin";
  auto osmdata = vj::PBFGraphParser::Parse(conf.get_child("mjolnir"), {"test/data/utrecht_netherlands.osm.pbf"},
                                       ways_file, way_nodes_file, access_file, restriction_file);
  // Build the graph using the OSMNodes and OSMWays from the parser
  vj::GraphBuilder::Build(conf, osmdata, ways_file, way_nodes_file, restriction_file);
  // Enhance the local level of the graph. This adds information to the local
  // level that is usable across all levels (density, administrative
  // information (and country based attribution), edge transition logic, etc.
  vj::GraphEnhancer::Enhance(conf, access_file);

  // Validate the graph and add information that cannot be added until
  // full graph is formed.
  vj::GraphValidator::Validate(conf);

  // Locations
  std::vector<valhalla::baldr::Location> locations;
  locations.push_back(valhalla::baldr::Location::FromCsv("52.09595728238367,5.114587247480813,break"));
  locations.push_back(valhalla::baldr::Location::FromCsv("52.096141834552945,5.114506781210365,break"));

  std::string method_options = "costing_options.pedestrian";
  auto costing_options = conf.get_child(method_options, {});

  std::shared_ptr<vs::DynamicCost> mode_costing[4];
  std::shared_ptr<vs::DynamicCost> cost = vs::CreatePedestrianCost(costing_options);
  auto mode = cost->travel_mode();
  mode_costing[static_cast<uint32_t>(mode)] = cost;

  const auto projections = vk::Search(locations, graph_reader, cost->GetEdgeFilter(), cost->GetNodeFilter());
  std::vector<PathLocation> path_location;
  vo::DirectionsOptions directions_options;
  for (auto loc : locations) {
    try {
      path_location.push_back(projections.at(loc));
      PathLocation::toPBF(path_location.back(), directions_options.mutable_locations()->Add(), graph_reader);
    } catch (...) {
      throw std::runtime_error("fail_invalid_origin");
    }
  }

  vt::AStarPathAlgorithm astar;
  auto path = astar.GetBestPath(*directions_options.mutable_locations(0), *directions_options.mutable_locations(1),
      graph_reader, mode_costing, mode);

  vt::AttributesController controller;
  vo::TripPath trip_path = vt::TripPathBuilder::Build(controller, graph_reader, mode_costing,path,
      *directions_options.mutable_locations(0), *directions_options.mutable_locations(1),
                                                      std::list<vo::Location>{});
  //really could of got the total of the elapsed_time.
  vo::DirectionsBuilder directions;
  vo::TripDirections trip_directions = directions.Build(directions_options, trip_path);

  if (trip_directions.summary().time() != 0) {
    std::ostringstream ostr;
    ostr << "Expected 0, but got " << trip_directions.summary().time();
    throw std::runtime_error(ostr.str());
  }

  boost::filesystem::remove(ways_file);
  boost::filesystem::remove(way_nodes_file);
  boost::filesystem::remove(access_file);

}

void TestTrivialPathNoUturns() {
  trivial_path_no_uturns(config_file);
}

void DoConfig() {
  //make a config file
  write_config(config_file);
}

} // anonymous namespace

int main() {
  test::suite suite("astar");

#ifdef MAKE_TEST_TILES
  // TODO: move to mjolnir?
  suite.test(TEST_CASE(make_tile));
#endif /* MAKE_TEST_TILES */

  suite.test(TEST_CASE(TestTrivialPath));
  suite.test(TEST_CASE(TestTrivialPathTriangle));

  suite.test(TEST_CASE(DoConfig));
  suite.test(TEST_CASE(TestTrivialPathNoUturns));

  return suite.tear_down();
}
