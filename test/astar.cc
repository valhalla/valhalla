#include "test.h"
#include <cstdint>
#include <fstream>

#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/location.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "loki/search.h"
#include "midgard/pointll.h"
#include "midgard/vector2.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/graphenhancer.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/graphvalidator.h"
#include "mjolnir/pbfgraphparser.h"
#include "odin/directionsbuilder.h"
#include "sif/costconstants.h"
#include "sif/dynamiccost.h"
#include "sif/pedestriancost.h"
#include "thor/astar.h"
#include "thor/attributes_controller.h"
#include "thor/trippathbuilder.h"

#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/proto/trippath.pb.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

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
 *  uncomment the define MAKE_TEST_TILES
 *  and delete the test tile: test/fake_tiles_astar/2/000/519/120.gph
 *  run make check to generate the test file
 *  copy the generated test file to test/fake_tiles_astar/2/000/519/120.gph
 */

// #define MAKE_TEST_TILES

#ifdef MAKE_TEST_TILES
#include "mjolnir/directededgebuilder.h"
#include "mjolnir/graphtilebuilder.h"
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

  GraphTileBuilder tile(test_dir, tile_id, false);

  // Set the base lat,lon of the tile
  uint32_t id = tile_id.tileid();
  const auto& tl = TileHierarchy::levels().rbegin();
  PointLL base_ll = tl->second.tiles.Base(id);
  tile.header_builder().set_base_ll(base_ll);

  uint32_t edge_index = 0;

  auto add_node = [&](const std::pair<vb::GraphId, vm::PointLL>& v, const uint32_t edge_count) {
    NodeInfo node_builder;
    node_builder.set_latlng(base_ll, v.second);
    // node_builder.set_road_class(RoadClass::kSecondary);
    node_builder.set_access(vb::kAllAccess);
    node_builder.set_edge_count(edge_count);
    node_builder.set_edge_index(edge_index);
    edge_index += edge_count;
    tile.nodes().emplace_back(std::move(node_builder));
  };

  auto add_edge = [&](const std::pair<vb::GraphId, vm::PointLL>& u,
                      const std::pair<vb::GraphId, vm::PointLL>& v, const uint32_t name,
                      const uint32_t opposing, const bool forward) {
    DirectedEdgeBuilder edge_builder({}, v.first, forward, u.second.Distance(v.second) + .5, 1, 1, 1,
                                     {}, {}, 0, false, 0, 0);
    edge_builder.set_opp_index(opposing);
    edge_builder.set_forwardaccess(vb::kAllAccess);
    std::vector<vm::PointLL> shape = {u.second, u.second.MidPoint(v.second), v.second};
    if (!forward)
      std::reverse(shape.begin(), shape.end());
    bool add;
    // make more complex edge geom so that there are 3 segments, affine combination doesnt properly
    // handle arcs but who cares
    uint32_t edge_info_offset =
        tile.AddEdgeInfo(name, u.first, v.first, 123, 0, 0, shape, {std::to_string(name)}, 0, add);
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

  add_edge(node::f, node::e, 4, 8, false);
  add_edge(node::f, node::g, 6, 13, true);
  add_node(node::f, 2);

  add_edge(node::g, node::e, 5, 9, false);
  add_edge(node::g, node::f, 6, 11, false);
  add_node(node::g, 2);

  tile.StoreTileData();

  GraphTileBuilder::tweeners_t tweeners;
  GraphTile reloaded(test_dir, tile_id);
  auto bins = GraphTileBuilder::BinEdges(&reloaded, tweeners);
  GraphTileBuilder::AddBins(test_dir, &reloaded, bins);
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
        \"admin\": \"" VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite\", \
         \"timezone\": \"" VALHALLA_SOURCE_DIR "test/data/not_needed.sqlite\" \
      } \
    }";
  } catch (...) {}
  file.close();
}

void create_costing_options(vo::DirectionsOptions& directions_options) {
  for (const auto costing :
       {vo::auto_, vo::auto_shorter, vo::bicycle, vo::bus, vo::hov, vo::motor_scooter, vo::multimodal,
        vo::pedestrian, vo::transit, vo::truck, vo::motorcycle, vo::auto_data_fix}) {
    if (costing == vo::pedestrian) {
      const rapidjson::Document doc;
      vs::ParsePedestrianCostOptions(doc, "/costing_options/pedestrian",
                                     directions_options.add_costing_options());
    } else {
      directions_options.add_costing_options();
    }
  }
}
// check that a path from origin to dest goes along the edge with expected_edge_index
void assert_is_trivial_path(vo::Location& origin, vo::Location& dest, uint32_t expected_edge_index) {

  // make the config file
  std::stringstream json;
  json << "{ \"tile_dir\": \"" VALHALLA_SOURCE_DIR "test/fake_tiles_astar\" }";
  bpt::ptree conf;
  rapidjson::read_json(json, conf);

  vb::GraphReader reader(conf);
  auto* tile = reader.GetGraphTile(tile_id);
  if (tile == nullptr) {
    throw std::runtime_error("Unable to load test tile! Please read the comment at the top of this "
                             "file about generating the test tiles.");
  }

  vo::DirectionsOptions directions_options;
  create_costing_options(directions_options);
  auto mode = vs::TravelMode::kPedestrian;
  vs::cost_ptr_t costs[int(vs::TravelMode::kMaxTravelMode)];
  auto pedestrian = vs::CreatePedestrianCost(valhalla::odin::Costing::pedestrian, directions_options);
  costs[int(mode)] = pedestrian;
  assert(bool(costs[int(mode)]));

  vt::AStarPathAlgorithm astar;
  auto path = astar.GetBestPath(origin, dest, reader, costs, mode);

  uint32_t time = 0;
  for (auto& p : path) {
    time += p.elapsed_time;
  }

  const DirectedEdge* de = tile->directededge(expected_edge_index);
  auto cost = pedestrian->EdgeCost(de, tile->GetSpeed(de));
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
  rapidjson::read_json(config_file, conf);

  // setup and purge
  vb::GraphReader graph_reader(conf.get_child("mjolnir"));
  for (const auto& level : vb::TileHierarchy::levels()) {
    auto level_dir = graph_reader.tile_dir() + "/" + std::to_string(level.first);
    if (boost::filesystem::exists(level_dir) && !boost::filesystem::is_empty(level_dir)) {
      boost::filesystem::remove_all(level_dir);
    }
  }

  // Set up the temporary (*.bin) files used during processing
  std::string ways_file = "test_ways_trivial.bin";
  std::string way_nodes_file = "test_way_nodes_trivial.bin";
  std::string nodes_file = "test_nodes_trivial.bin";
  std::string edges_file = "test_edges_trivial.bin";
  std::string access_file = "test_access_trivial.bin";
  std::string cr_from_file = "test_from_complex_restrictions_trivial.bin";
  std::string cr_to_file = "test_to_complex_restrictions_trivial.bin";

  // Parse Utrecht OSM data
  auto osmdata =
      vj::PBFGraphParser::Parse(conf.get_child("mjolnir"),
                                {VALHALLA_SOURCE_DIR "test/data/utrecht_netherlands.osm.pbf"},
                                ways_file, way_nodes_file, access_file, cr_from_file, cr_to_file);

  // Build the graph using the OSMNodes and OSMWays from the parser
  vj::GraphBuilder::Build(conf, osmdata, ways_file, way_nodes_file, nodes_file, edges_file,
                          cr_from_file, cr_to_file);

  // Enhance the local level of the graph. This adds information to the local
  // level that is usable across all levels (density, administrative
  // information (and country based attribution), edge transition logic, etc.
  vj::GraphEnhancer::Enhance(conf, access_file);

  // Validate the graph and add information that cannot be added until
  // full graph is formed.
  vj::GraphValidator::Validate(conf);

  // Locations
  std::vector<valhalla::baldr::Location> locations;
  Location origin(valhalla::midgard::PointLL(5.114587f, 52.095957f), Location::StopType::BREAK);
  locations.push_back(origin);
  Location dest(valhalla::midgard::PointLL(5.114506f, 52.096141f), Location::StopType::BREAK);
  locations.push_back(dest);

  vo::DirectionsOptions directions_options;
  create_costing_options(directions_options);
  std::shared_ptr<vs::DynamicCost> mode_costing[4];
  std::shared_ptr<vs::DynamicCost> cost =
      vs::CreatePedestrianCost(valhalla::odin::Costing::pedestrian, directions_options);
  auto mode = cost->travel_mode();
  mode_costing[static_cast<uint32_t>(mode)] = cost;

  const auto projections =
      vk::Search(locations, graph_reader, cost->GetEdgeFilter(), cost->GetNodeFilter());
  std::vector<PathLocation> path_location;

  for (auto loc : locations) {
    try {
      path_location.push_back(projections.at(loc));
      PathLocation::toPBF(path_location.back(), directions_options.mutable_locations()->Add(),
                          graph_reader);
    } catch (...) { throw std::runtime_error("fail_invalid_origin"); }
  }

  vt::AStarPathAlgorithm astar;
  auto path =
      astar.GetBestPath(*directions_options.mutable_locations(0),
                        *directions_options.mutable_locations(1), graph_reader, mode_costing, mode);

  vt::AttributesController controller;
  vo::TripPath trip_path =
      vt::TripPathBuilder::Build(controller, graph_reader, mode_costing, path,
                                 *directions_options.mutable_locations(0),
                                 *directions_options.mutable_locations(1), std::list<vo::Location>{});
  // really could of got the total of the elapsed_time.
  vo::DirectionsBuilder directions;
  vo::TripDirections trip_directions = directions.Build(directions_options, trip_path);

  if (trip_directions.summary().time() != 0) {
    std::ostringstream ostr;
    ostr << "Expected 0, but got " << trip_directions.summary().time();
    throw std::runtime_error(ostr.str());
  }

  boost::filesystem::remove(ways_file);
  boost::filesystem::remove(way_nodes_file);
  boost::filesystem::remove(nodes_file);
  boost::filesystem::remove(edges_file);
  boost::filesystem::remove(access_file);
  boost::filesystem::remove(cr_from_file);
  boost::filesystem::remove(cr_to_file);
}

void TestTrivialPathNoUturns() {
  trivial_path_no_uturns(config_file);
}

void DoConfig() {
  // make a config file
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
