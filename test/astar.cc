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
#include "loki/worker.h"
#include "midgard/pointll.h"
#include "midgard/vector2.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/graphenhancer.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/graphvalidator.h"
#include "mjolnir/pbfgraphparser.h"
#include "odin/directionsbuilder.h"
#include "odin/worker.h"
#include "sif/costconstants.h"
#include "sif/dynamiccost.h"
#include "sif/pedestriancost.h"
#include "thor/astar.h"
#include "thor/attributes_controller.h"
#include "thor/bidirectional_astar.h"
#include "thor/pathalgorithm.h"
#include "thor/timedep.h"
#include "thor/triplegbuilder.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

#include <valhalla/proto/directions.pb.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto/trip.pb.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

namespace bpt = boost::property_tree;

using namespace valhalla;
namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vs = valhalla::sif;
namespace vt = valhalla::thor;
namespace vk = valhalla::loki;
namespace vj = valhalla::mjolnir;
namespace vo = valhalla::odin;

#include "mjolnir/directededgebuilder.h"
#include "mjolnir/graphtilebuilder.h"

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
// about a third of its width.
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
std::string test_dir = "test/data/fake_tiles_astar";
vb::GraphId tile_id = vb::TileHierarchy::GetGraphId({.125, .125}, 2);

namespace node {
std::pair<vb::GraphId, vm::PointLL> a({tile_id.tileid(), tile_id.level(), 0}, {0.01, 0.10});
std::pair<vb::GraphId, vm::PointLL> b({tile_id.tileid(), tile_id.level(), 1}, {0.10, 0.10});
std::pair<vb::GraphId, vm::PointLL> c({tile_id.tileid(), tile_id.level(), 2}, {0.01, 0.01});
std::pair<vb::GraphId, vm::PointLL> d({tile_id.tileid(), tile_id.level(), 3}, {0.10, 0.01});

std::pair<vb::GraphId, vm::PointLL> e({tile_id.tileid(), tile_id.level(), 4}, {0.21, 0.14});
std::pair<vb::GraphId, vm::PointLL> f({tile_id.tileid(), tile_id.level(), 5}, {0.20, 0.14});
std::pair<vb::GraphId, vm::PointLL> g({tile_id.tileid(), tile_id.level(), 6}, {0.25, 0.11});
} // namespace node

void make_tile() {
  // Don't recreate tiles if they already exist (leads to silent corruption of tiles)
  if (filesystem::exists(test_dir + "/2/000/519/120.gph")) {
    return;
  }

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
    node_builder.set_timezone(1);
    edge_index += edge_count;
    tile.nodes().emplace_back(node_builder);
  };

  auto add_edge = [&](const std::pair<vb::GraphId, vm::PointLL>& u,
                      const std::pair<vb::GraphId, vm::PointLL>& v, const uint32_t localedgeidx,
                      const uint32_t opposing_edge_index, const bool forward) {
    DirectedEdgeBuilder edge_builder({}, v.first, forward, u.second.Distance(v.second) + .5, 1, 1,
                                     baldr::Use::kRoad, baldr::RoadClass::kMotorway, localedgeidx,
                                     false, 0, 0, false);
    edge_builder.set_opp_index(opposing_edge_index);
    edge_builder.set_forwardaccess(vb::kAllAccess);
    edge_builder.set_reverseaccess(vb::kAllAccess);
    edge_builder.set_free_flow_speed(100);
    edge_builder.set_constrained_flow_speed(10);
    std::vector<vm::PointLL> shape = {u.second, u.second.MidPoint(v.second), v.second};
    if (!forward) {
      std::reverse(shape.begin(), shape.end());
    }
    bool added;
    // make more complex edge geom so that there are 3 segments, affine combination doesnt properly
    // handle arcs but who cares
    uint32_t edge_info_offset = tile.AddEdgeInfo(localedgeidx, u.first, v.first, 123, // way_id
                                                 0, 0,
                                                 120, // speed limit in kph
                                                 shape, {std::to_string(localedgeidx)}, 0, added);
    // assert(added);
    edge_builder.set_edgeinfo_offset(edge_info_offset);
    tile.directededges().emplace_back(edge_builder);
  };

  // first set of roads - Square
  add_edge(node::a, node::b, 0, 0, true);
  add_edge(node::a, node::c, 1, 0, true);
  add_node(node::a, 2);

  add_edge(node::b, node::a, 2, 0, false);
  add_edge(node::b, node::d, 3, 1, true);
  add_node(node::b, 2);

  add_edge(node::c, node::a, 4, 1, false);
  add_edge(node::c, node::d, 5, 0, true);
  add_node(node::c, 2);

  add_edge(node::d, node::c, 6, 1, false);
  add_edge(node::d, node::b, 7, 1, false);
  add_node(node::d, 2);

  // second set of roads - Triangle
  add_edge(node::e, node::f, 8, 10, true);
  add_edge(node::e, node::g, 9, 12, true);
  add_node(node::e, 2);

  add_edge(node::f, node::e, 10, 8, false);
  add_edge(node::f, node::g, 11, 13, true);
  add_node(node::f, 2);

  add_edge(node::g, node::e, 12, 9, false);
  add_edge(node::g, node::f, 13, 11, false);
  add_node(node::g, 2);

  tile.StoreTileData();

  GraphTileBuilder::tweeners_t tweeners;
  GraphTile reloaded(test_dir, tile_id);
  auto bins = GraphTileBuilder::BinEdges(&reloaded, tweeners);
  GraphTileBuilder::AddBins(test_dir, &reloaded, bins);
  if (!filesystem::exists(test_dir + "/2/000/519/120.gph")) {
    throw std::runtime_error("Still no expected tile, did the actual fname on disk change?");
  }
}

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

void create_costing_options(Options& options) {
  const rapidjson::Document doc;
  sif::ParseAutoCostOptions(doc, "/costing_options/auto", options.add_costing_options());
  sif::ParseAutoShorterCostOptions(doc, "/costing_options/auto_shorter",
                                   options.add_costing_options());
  sif::ParseBicycleCostOptions(doc, "/costing_options/bicycle", options.add_costing_options());
  sif::ParseBusCostOptions(doc, "/costing_options/bus", options.add_costing_options());
  sif::ParseHOVCostOptions(doc, "/costing_options/hov", options.add_costing_options());
  sif::ParseTaxiCostOptions(doc, "/costing_options/taxi", options.add_costing_options());
  sif::ParseMotorScooterCostOptions(doc, "/costing_options/motor_scooter",
                                    options.add_costing_options());
  sif::ParsePedestrianCostOptions(doc, "/costing_options/pedestrian", options.add_costing_options());
  sif::ParseTransitCostOptions(doc, "/costing_options/transit", options.add_costing_options());
  sif::ParseTruckCostOptions(doc, "/costing_options/truck", options.add_costing_options());
  sif::ParseMotorcycleCostOptions(doc, "/costing_options/motorcycle", options.add_costing_options());
  sif::ParseAutoShorterCostOptions(doc, "/costing_options/auto_shorter",
                                   options.add_costing_options());
  sif::ParseAutoDataFixCostOptions(doc, "/costing_options/auto_data_fix",
                                   options.add_costing_options());
  options.add_costing_options();
}

enum class TrivialPathTest {
  MatchesEdge,
  DurationEqualTo,
};

// check that a path from origin to dest goes along the edge with expected_edge_index
void assert_is_trivial_path(vt::PathAlgorithm& astar,
                            valhalla::Location& origin,
                            valhalla::Location& dest,
                            uint32_t expected_num_paths,
                            TrivialPathTest assert_type,
                            int32_t assert_type_value,
                            vs::TravelMode mode = vs::TravelMode::kPedestrian) {

  // make the config file
  std::stringstream json;
  json << "{ \"tile_dir\": \"" << test_dir << "\" }";
  bpt::ptree conf;
  rapidjson::read_json(json, conf);

  vb::GraphReader reader(conf);
  auto* tile = reader.GetGraphTile(tile_id);
  if (tile == nullptr) {
    throw std::runtime_error("Unable to load test tile! Did `make_tile` run succesfully?");
  }
  if (tile->header()->directededgecount() != 14) {
    throw std::runtime_error("test-tiles does not contain expected number of edges");
  }
  const GraphTile* endtile = reader.GetGraphTile(node::b.first);
  if (endtile == nullptr) {
    throw std::runtime_error("bad tile, node::b wasn't found in it");
  }

  Options options;
  create_costing_options(options);
  vs::cost_ptr_t costs[int(vs::TravelMode::kMaxTravelMode)];
  switch (mode) {
    case vs::TravelMode::kPedestrian: {
      auto pedestrian = vs::CreatePedestrianCost(Costing::pedestrian, options);
      costs[int(mode)] = pedestrian;
      break;
    }
    case vs::TravelMode::kDrive: {
      auto car = vs::CreateAutoCost(Costing::auto_, options);
      costs[int(mode)] = car;
      break;
    }
    default:
      throw std::runtime_error("Unhandled case");
  }
  assert(bool(costs[int(mode)]));

  auto paths = astar.GetBestPath(origin, dest, reader, costs, mode);

  int32_t time = 0;
  for (const auto& path : paths) {
    for (const auto& p : path) {
      time += p.elapsed_time;
    }
    for (const vt::PathInfo& subpath : path) {
      LOG_INFO("Got path " + std::to_string(subpath.edgeid.id()));
    }
    if (path.size() != expected_num_paths) {
      std::ostringstream ostr;
      ostr << "Expected number of paths to be " << expected_num_paths << ", but got " << path.size();
      throw std::runtime_error(ostr.str());
    }
    break;
  }

  uint32_t expected_time = 979797;
  switch (assert_type) {
    case TrivialPathTest::DurationEqualTo:
      // Supply duration directly
      expected_time = assert_type_value;
      break;
    case TrivialPathTest::MatchesEdge:
      // Grab time from an edge index
      const DirectedEdge* expected_edge = tile->directededge(assert_type_value);
      auto expected_cost = costs[int(mode)]->EdgeCost(expected_edge, tile);
      expected_time = expected_cost.secs;
      break;
  };
  if (expected_time == 0) {
    throw std::runtime_error("Expected time is 0, your test probably has a logic error");
  }

  if (time != expected_time) {
    std::ostringstream ostr;
    ostr << "Expected time to be " << expected_time << "s, but got " << time << "s";
    throw std::runtime_error(ostr.str());
  }
}

// Adds edge to location
void add(GraphId edge_id, float percent_along, const PointLL& ll, valhalla::Location& location) {
  location.mutable_path_edges()->Add()->set_graph_id(edge_id);
  location.mutable_path_edges()->rbegin()->set_percent_along(percent_along);
  location.mutable_path_edges()->rbegin()->mutable_ll()->set_lng(ll.first);
  location.mutable_path_edges()->rbegin()->mutable_ll()->set_lat(ll.second);
  location.mutable_path_edges()->rbegin()->set_distance(0.0f);
}

// test that a path from A to B succeeds, even if the edges from A to C and B
// to D appear first in the PathLocation.
void TestTrivialPath(vt::PathAlgorithm& astar) {
  using node::a;
  using node::b;
  using node::c;
  using node::d;

  valhalla::Location origin;
  // origin.set_date_time();
  origin.mutable_ll()->set_lng(a.second.first);
  origin.mutable_ll()->set_lat(a.second.second);
  add(tile_id + uint64_t(1), 0.0f, a.second, origin);
  add(tile_id + uint64_t(4), 1.0f, a.second, origin);
  add(tile_id + uint64_t(0), 0.0f, a.second, origin);
  add(tile_id + uint64_t(2), 1.0f, a.second, origin);

  valhalla::Location dest;
  dest.mutable_ll()->set_lng(b.second.first);
  dest.mutable_ll()->set_lat(b.second.second);
  add(tile_id + uint64_t(3), 0.0f, b.second, dest);
  add(tile_id + uint64_t(7), 1.0f, b.second, dest);
  add(tile_id + uint64_t(2), 0.0f, b.second, dest);
  add(tile_id + uint64_t(0), 1.0f, b.second, dest);

  // this should go along the path from A to B
  assert_is_trivial_path(astar, origin, dest, 1, TrivialPathTest::DurationEqualTo, 360,
                         vs::TravelMode::kDrive);
}

void TestTrivialPathForward() {
  auto astar = vt::TimeDepForward();
  TestTrivialPath(astar);
}
void TestTrivialPathReverse() {
  auto astar = vt::TimeDepReverse();
  TestTrivialPath(astar);
}

// test that a path from E to F succeeds, even if the edges from E and F
// to G appear first in the PathLocation.
void TestTrivialPathTriangle() {
  using node::e;
  using node::f;

  valhalla::Location origin;
  origin.mutable_ll()->set_lng(e.second.first);
  origin.mutable_ll()->set_lat(e.second.second);
  add(tile_id + uint64_t(9), 0.0f, e.second, origin);
  add(tile_id + uint64_t(12), 1.0f, e.second, origin);
  add(tile_id + uint64_t(8), 0.0f, e.second, origin);
  add(tile_id + uint64_t(10), 1.0f, e.second, origin);

  valhalla::Location dest;
  dest.mutable_ll()->set_lng(f.second.first);
  dest.mutable_ll()->set_lat(f.second.second);
  add(tile_id + uint64_t(11), 0.0f, f.second, dest);
  add(tile_id + uint64_t(13), 1.0f, f.second, dest);
  add(tile_id + uint64_t(10), 0.0f, f.second, dest);
  add(tile_id + uint64_t(8), 1.0f, f.second, dest);

  // TODO This fails with graphindex out of bounds for Reverse direction, is this
  // related to why we short-circuit trivial routes to AStarPathAlgorithm in route_action.cc?
  //
  vt::AStarPathAlgorithm astar;
  // this should go along the path from E to F
  assert_is_trivial_path(astar, origin, dest, 1, TrivialPathTest::MatchesEdge, 8,
                         vs::TravelMode::kPedestrian);
}

void TestPartialDurationTrivial() {
  // Tests a trivial path with partial edge results in partial duration
  using node::a;
  using node::b;
  using node::d;

  valhalla::Location origin;
  origin.set_date_time("2019-11-21T23:05");
  origin.mutable_ll()->set_lng(a.second.first);
  origin.mutable_ll()->set_lat(a.second.second);
  add(tile_id + uint64_t(0), 0.1f, a.second, origin);
  add(tile_id + uint64_t(2), 0.9f, a.second, origin);

  float partial_dist = 0.1;
  valhalla::Location dest;
  dest.mutable_ll()->set_lng(b.second.first);
  dest.mutable_ll()->set_lat(b.second.second);
  add(tile_id + uint64_t(2), 0. + partial_dist, b.second, dest);
  add(tile_id + uint64_t(0), 1. - partial_dist, b.second, dest);
  add(tile_id + uint64_t(3), 0.0f, b.second, dest);
  add(tile_id + uint64_t(7), 1.0f, b.second, dest);

  uint32_t expected_duration = 288;

  vt::TimeDepForward astar;
  assert_is_trivial_path(astar, origin, dest, 1, TrivialPathTest::DurationEqualTo, expected_duration,
                         vs::TravelMode::kDrive);
}

void TestPartialDuration(vt::PathAlgorithm& astar) {
  // Tests that a partial duration is returned when starting on a partial edge
  using node::a;
  using node::b;
  using node::d;

  float partial_dist = 0.1;

  valhalla::Location origin;
  origin.set_date_time("2019-11-21T23:05");
  origin.mutable_ll()->set_lng(a.second.first);
  origin.mutable_ll()->set_lat(a.second.second);
  add(tile_id + uint64_t(0), 0. + partial_dist, a.second, origin);
  add(tile_id + uint64_t(2), 1. - partial_dist, a.second, origin);

  valhalla::Location dest;
  dest.set_date_time("2019-11-21T23:05");
  dest.mutable_ll()->set_lng(d.second.first);
  dest.mutable_ll()->set_lat(d.second.second);
  add(tile_id + uint64_t(7), 0.0f + partial_dist, d.second, dest);
  add(tile_id + uint64_t(3), 1.0f - partial_dist, d.second, dest);

  uint32_t expected_duration = 973;

  assert_is_trivial_path(astar, origin, dest, 2, TrivialPathTest::DurationEqualTo, expected_duration,
                         vs::TravelMode::kDrive);
}

void TestPartialDurationForward() {
  vt::TimeDepForward astar;
  TestPartialDuration(astar);
}
void TestPartialDurationReverse() {
  vt::TimeDepReverse astar;
  TestPartialDuration(astar);
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
  std::string bss_nodes_file = "test_bss_nodes_file_trivial.bin";

  // Parse Utrecht OSM data
  auto osmdata =
      vj::PBFGraphParser::Parse(conf.get_child("mjolnir"),
                                {VALHALLA_SOURCE_DIR "test/data/utrecht_netherlands.osm.pbf"},
                                ways_file, way_nodes_file, access_file, cr_from_file, cr_to_file,
                                bss_nodes_file);

  // Build the graph using the OSMNodes and OSMWays from the parser
  vj::GraphBuilder::Build(conf, osmdata, ways_file, way_nodes_file, nodes_file, edges_file,
                          cr_from_file, cr_to_file);

  // Enhance the local level of the graph. This adds information to the local
  // level that is usable across all levels (density, administrative
  // information (and country based attribution), edge transition logic, etc.
  vj::GraphEnhancer::Enhance(conf, osmdata, access_file);

  // Validate the graph and add information that cannot be added until
  // full graph is formed.
  vj::GraphValidator::Validate(conf);

  // Locations
  std::vector<valhalla::baldr::Location> locations;
  baldr::Location origin(valhalla::midgard::PointLL(5.114587f, 52.095957f),
                         baldr::Location::StopType::BREAK);
  locations.push_back(origin);
  baldr::Location dest(valhalla::midgard::PointLL(5.114506f, 52.096141f),
                       baldr::Location::StopType::BREAK);
  locations.push_back(dest);

  Api api;
  auto& options = *api.mutable_options();
  create_costing_options(options);
  std::shared_ptr<vs::DynamicCost> mode_costing[4];
  std::shared_ptr<vs::DynamicCost> cost = vs::CreatePedestrianCost(Costing::pedestrian, options);
  auto mode = cost->travel_mode();
  mode_costing[static_cast<uint32_t>(mode)] = cost;

  const auto projections = vk::Search(locations, graph_reader, cost.get());
  std::vector<PathLocation> path_location;

  for (const auto& loc : locations) {
    try {
      path_location.push_back(projections.at(loc));
      PathLocation::toPBF(path_location.back(), options.mutable_locations()->Add(), graph_reader);
    } catch (...) { throw std::runtime_error("fail_invalid_origin"); }
  }

  vt::AStarPathAlgorithm astar;
  auto path = astar
                  .GetBestPath(*options.mutable_locations(0), *options.mutable_locations(1),
                               graph_reader, mode_costing, mode)
                  .front();

  vt::AttributesController controller;
  auto& leg = *api.mutable_trip()->mutable_routes()->Add()->mutable_legs()->Add();
  vt::TripLegBuilder::Build(controller, graph_reader, mode_costing, path.begin(), path.end(),
                            *options.mutable_locations(0), *options.mutable_locations(1),
                            std::list<valhalla::Location>{}, leg);
  // really could of got the total of the elapsed_time.
  odin::DirectionsBuilder::Build(api);
  const auto& trip_directions = api.directions().routes(0).legs(0);

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

std::string ways_file = "test_ways_whitelion.bin";
std::string way_nodes_file = "test_way_nodes_whitelion.bin";
std::string access_file = "test_access_whitelion.bin";
std::string from_restriction_file = "test_from_complex_restrictions_whitelion.bin";
std::string to_restriction_file = "test_to_complex_restrictions_whitelion.bin";
std::string bss_file = "test_bss_nodes_whitelion.bin";

boost::property_tree::ptree get_conf(const char* tiles) {
  std::stringstream ss;
  ss << R"({
      "mjolnir":{"tile_dir":"test/data/)"
     << tiles << R"(", "concurrency": 1},
      "loki":{
        "actions":["route"],
        "logging":{"long_request": 100},
        "service_defaults":{"minimum_reachability": 2,"radius": 10,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "heading_tolerance": 60}
      },
      "thor":{"logging":{"long_request": 100}},
      "odin":{"logging":{"long_request": 100}},
      "skadi":{"actons":["height"],"logging":{"long_request": 5}},
      "meili":{"customizable": ["turn_penalty_factor","max_route_distance_factor","max_route_time_factor","search_radius"],
              "mode":"auto","grid":{"cache_size":100240,"size":500},
              "default":{"beta":3,"breakage_distance":2000,"geometry":false,"gps_accuracy":5.0,"interpolation_distance":10,
              "max_route_distance_factor":5,"max_route_time_factor":5,"max_search_radius":200,"route":true,
              "search_radius":15.0,"sigma_z":4.07,"turn_penalty_factor":200}},
      "service_limits": {
        "auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
        "bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time": 120},
        "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
        "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
        "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
        "skadi": {"max_shape": 750000,"min_resample": 10.0},
        "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
        "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
        "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
      }
    })";
  boost::property_tree::ptree conf;
  rapidjson::read_json(ss, conf);
  return conf;
}

struct route_tester {
  route_tester(const boost::property_tree::ptree& _conf)
      : conf(_conf), reader(new GraphReader(conf.get_child("mjolnir"))), loki_worker(conf, reader),
        thor_worker(conf, reader), odin_worker(conf) {
  }
  Api test(const std::string& request_json) {
    Api request;
    ParseApi(request_json, valhalla::Options::route, request);
    loki_worker.route(request);
    thor_worker.route(request);
    odin_worker.narrate(request);
    return request;
  }
  boost::property_tree::ptree conf;
  std::shared_ptr<GraphReader> reader;
  vk::loki_worker_t loki_worker;
  vt::thor_worker_t thor_worker;
  vo::odin_worker_t odin_worker;
};

void test_oneway() {
  auto conf = get_conf("whitelion_tiles");
  route_tester tester(conf);
  // Test onewayness with this route - oneway works, South-West to North-East
  std::string request =
      R"({"locations":[{"lat":51.455768530466514,"lon":-2.5954368710517883},{"lat":51.456082740244824,"lon":-2.595050632953644}],"costing":"auto"})";

  auto response = tester.test(request);

  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  if (legs.size() != 1) {
    throw std::logic_error("Should have 1 leg");
  }

  std::vector<std::string> names;

  for (const auto& d : directions) {
    for (const auto& m : d.maneuver()) {
      std::string name;
      for (const auto& n : m.street_name()) {
        name += n.value() + " ";
      }
      if (!name.empty()) {
        name.pop_back();
      }
      names.push_back(name);
    }
  }

  auto correct_route = std::vector<std::string>{"Quay Street", "Nelson Street", ""};
  if (names != correct_route) {
    throw std::logic_error("Incorrect route, got: \n" + boost::algorithm::join(names, ", ") +
                           ", expected: \n" + boost::algorithm::join(correct_route, ", "));
  }
}

void test_oneway_wrong_way() {
  auto conf = get_conf("whitelion_tiles");
  route_tester tester(conf);
  // Test onewayness with this route - oneway wrong way, North-east to South-West
  // Should produce no-route
  std::string request =
      R"({"locations":[{"lat":51.456082740244824,"lon":-2.595050632953644},{"lat":51.455768530466514,"lon":-2.5954368710517883}],"costing":"auto"})";

  try {
    auto response = tester.test(request);
  } catch (const std::exception& e) {
    if (std::string(e.what()) != "No path could be found for input") {
      throw std::logic_error("Was expecting 'No path could be found for input'");
    }
  }
}

void test_deadend() {
  auto conf = get_conf("whitelion_tiles");
  route_tester tester(conf);
  std::string request =
      R"({
      "locations":[
        {"lat":51.45562646682483,"lon":-2.5952598452568054},
        {"lat":51.455143447135974,"lon":-2.5958767533302307}
      ],
      "costing":"auto"
      })";

  auto response = tester.test(request);

  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  if (legs.size() != 1) {
    throw std::logic_error("Should have 1 leg");
  }

  std::vector<std::string> names;
  std::string uturn_street;

  for (const auto& d : directions) {
    for (const auto& m : d.maneuver()) {
      std::string name;
      for (const auto& n : m.street_name()) {
        name += n.value() + " ";
      }
      if (!name.empty()) {
        name.pop_back();
      }
      bool is_uturn = false;
      if (m.type() == DirectionsLeg_Maneuver_Type_kUturnRight ||
          m.type() == DirectionsLeg_Maneuver_Type_kUturnLeft) {
        is_uturn = true;
        uturn_street = name;
      }
      names.push_back(name);
    }
  }

  auto correct_route =
      std::vector<std::string>{"Bell Lane",   "Small Street",
                               "Quay Street", // The u-turn on Quay Street is optimized away
                               "Quay Street", "Small Street", "", ""};
  if (names != correct_route) {
    throw std::logic_error("Incorrect route, got: \n" + boost::algorithm::join(names, ", ") +
                           ", expected: \n" + boost::algorithm::join(correct_route, ", "));
  }
  if (uturn_street != "Quay Street") {
    throw std::logic_error("We did not find the expected u-turn");
  }
}

void test_deadend_timedep_forward() {
  auto conf = get_conf("whitelion_tiles_reverse");
  route_tester tester(conf);
  std::string request =
      R"({
      "locations":[
        {"lat":51.45562646682483,"lon":-2.5952598452568054},
        {"lat":51.455143447135974,"lon":-2.5958767533302307}
      ],
      "costing":"auto",
      "date_time":{
        "type":1,
        "value":"2019-11-21T11:05"
      }
    })";

  auto response = tester.test(request);

  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  if (legs.size() != 1) {
    throw std::logic_error("Should have 1 leg");
  }

  std::vector<std::string> names;
  std::string uturn_street;

  for (const auto& d : directions) {
    for (const auto& m : d.maneuver()) {
      std::string name;
      for (const auto& n : m.street_name()) {
        name += n.value() + " ";
      }
      if (!name.empty()) {
        name.pop_back();
      }
      bool is_uturn = false;
      if (m.type() == DirectionsLeg_Maneuver_Type_kUturnRight ||
          m.type() == DirectionsLeg_Maneuver_Type_kUturnLeft) {
        is_uturn = true;
        uturn_street = name;
      }
      names.push_back(name);
    }
  }

  auto correct_route =
      std::vector<std::string>{"Bell Lane",   "Small Street",
                               "Quay Street", // The u-turn on Quay Street is optimized away
                               "Quay Street", "Small Street", "", ""};
  if (names != correct_route) {
    throw std::logic_error("Incorrect route, got: \n" + boost::algorithm::join(names, ", ") +
                           ", expected: \n" + boost::algorithm::join(correct_route, ", "));
  }
  // TODO Why did it not happen on Quay Street?
  // if (uturn_street != "Small Street") {
  if (uturn_street != "Quay Street") {
    throw std::logic_error("We did not find the expected u-turn");
  }
}
void test_deadend_timedep_reverse() {
  auto conf = get_conf("whitelion_tiles");
  route_tester tester(conf);
  std::string request =
      R"({
      "locations":[
        {"lat":51.45562646682483,"lon":-2.5952598452568054},
        {"lat":51.455143447135974,"lon":-2.5958767533302307}
      ],
      "costing":"auto",
      "date_time":{
        "type":2,
        "value":"2019-11-21T11:05"
      }
    })";

  auto response = tester.test(request);

  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  if (legs.size() != 1) {
    throw std::logic_error("Should have 1 leg");
  }

  std::vector<std::string> names;
  std::string uturn_street;

  for (const auto& d : directions) {
    for (const auto& m : d.maneuver()) {
      std::string name;
      for (const auto& n : m.street_name()) {
        name += n.value() + " ";
      }
      if (!name.empty()) {
        name.pop_back();
      }
      bool is_uturn = false;
      if (m.type() == DirectionsLeg_Maneuver_Type_kUturnRight ||
          m.type() == DirectionsLeg_Maneuver_Type_kUturnLeft) {
        is_uturn = true;
        uturn_street = name;
      }
      names.push_back(name);
    }
  }

  auto correct_route =
      std::vector<std::string>{"Bell Lane",   "Small Street",
                               "Quay Street", // The u-turn on Quay Street is optimized away
                               "Quay Street", "Small Street", "", ""};
  if (names != correct_route) {
    throw std::logic_error("Incorrect route, got: \n" + boost::algorithm::join(names, ", ") +
                           ", expected: \n" + boost::algorithm::join(correct_route, ", "));
  }
  if (uturn_street != "Quay Street") {
    throw std::logic_error("We did not find the expected u-turn");
  }
}
void test_time_restricted_road_bidirectional() {
  // Try routing over "Via Montebello" in Rome which is a time restricted road
  // We should receive a route for a time-independent query but have the response
  // note that it is time restricted
  auto conf = get_conf("roma_tiles");
  route_tester tester(conf);
  std::string request =
      R"({"locations":[{"lat":41.90550,"lon":12.50090},{"lat":41.90477,"lon":12.49914}],"costing":"auto"})";

  auto response = tester.test(request);

  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  if (legs.size() != 1) {
    throw std::logic_error("Should have 1 leg");
  }

  std::vector<std::string> names;
  std::vector<std::string> restricted_streets;

  for (const auto& d : directions) {
    for (const auto& m : d.maneuver()) {
      std::string name;
      for (const auto& n : m.street_name()) {
        name += n.value() + " ";
      }
      if (!name.empty()) {
        name.pop_back();
      }
      if (m.has_time_restrictions()) {
        restricted_streets.push_back(name);
      }
      names.push_back(name);
    }
  }

  auto correct_route = std::vector<std::string>{"Via Goito", "Via Montebello", ""};
  if (names != correct_route) {
    throw std::logic_error("Incorrect route, got: \n" + boost::algorithm::join(names, ", ") +
                           ", expected: \n" + boost::algorithm::join(correct_route, ", "));
  }

  if (!response.trip().routes(0).legs(0).node(1).edge().has_time_restrictions()) {
    throw std::logic_error("Expected leg to have time_restriction");
  }

  // Verify JSON payload
  const std::string payload = tyr::serializeDirections(response);
  rapidjson::Document response_json;
  response_json.Parse(payload);
  std::cout << payload << std::endl;
  {
    const char key[] = "/trip/legs/0/maneuvers/0/has_time_restrictions";
    if (GetValueByPointerWithDefault(response_json, key, false) == true) {
      throw std::logic_error(
          std::string("Via Goito is marked as time-restricted which is incorrect! JSON does have ") +
          key + " set to true");
    }
  }
  {
    const char key[] = "/trip/legs/0/maneuvers/1/has_time_restrictions";
    if (GetValueByPointerWithDefault(response_json, key, false) != true) {
      throw std::logic_error(std::string("JSON does not have ") + key + " set to true");
    }
  }
  {
    const char key[] = "/trip/legs/0/summary/has_time_restrictions";
    if (GetValueByPointerWithDefault(response_json, key, false) != true) {
      throw std::logic_error(std::string("JSON does not have ") + key + " set to true");
    }
  }
  {
    const char key[] = "/trip/summary/has_time_restrictions";
    if (GetValueByPointerWithDefault(response_json, key, false) != true) {
      throw std::logic_error(std::string("JSON does not have ") + key + " set to true");
    }
  }
}

Api route_on_timerestricted(std::string& costing_str, int16_t hour) {
  // Try routing over "Via Montebello" in Rome which is a time restricted road
  // The restriction is
  //
  //    <tag k="auto" v="yes"/>
  //    <tag k="motor_vehicle:conditional" v="no @ (Mo-Sa 07:00-16:00)"/>
  //    <tag k="pedestrian" v="no"/>
  //    <tag k="pedestrian:conditional" v="yes @ (Mo-Sa 08:00-15:00)"/>
  //
  // so lets use a timedependent a-star and verify that

  LOG_INFO("Testing " + costing_str + " route at hour " + std::to_string(hour));
  auto conf = get_conf("roma_tiles");
  route_tester tester(conf);
  // The following request results in timedep astar during the restricted hours
  // and should be denied
  std::string request =
      R"({
        "locations":[{"lat":41.90550,"lon":12.50090},{"lat":41.90477,"lon":12.49914}],
        "costing":")" +
      costing_str + R"(",
          "date_time":{
            "type":1,
            "value":"2020-01-16T)" +
      std::to_string(hour) + R"(:05"
          }
        })";

  return tester.test(request);
}

void test_route_restricted(std::string costing_str, int16_t hour) {
  bool found_route = false;
  try {
    auto response = route_on_timerestricted(costing_str, hour);
    found_route = true;
    const auto& leg = response.directions().routes(0).legs(0);
    LOG_INFO("Route that wasn't supposed to happen: " + leg.shape());
  } catch (const std::exception& e) {
    if (std::string(e.what()) != "No path could be found for input") {
      throw std::logic_error("Was expecting 'No path could be found for input'");
    } else {
      return;
    }
  }
  if (found_route) {
    throw std::logic_error("Found a route when no route was expected");
  }
}

void test_time_restricted_road_denied_on_timedep() {
  {
    // A car at hour 11 should be denied
    std::string costing_str("auto");
    test_route_restricted(costing_str, 11);
  }
  {
    // A pedestrian at hour 22 should be denied
    std::string costing_str("pedestrian");
    test_route_restricted(costing_str, 22);
  }
}

void test_route_allowed(std::string costing_str, int16_t hour) {
  auto response = route_on_timerestricted(costing_str, hour);
  const auto& legs = response.trip().routes(0).legs();
  if (legs.size() != 1) {
    throw std::logic_error("Should have 1 leg");
  }
  const auto& directions = response.directions().routes(0).legs(0);
  const auto& summary = directions.summary();
  if (summary.time() == 0) {
    throw std::logic_error("Time shouldn't be 0");
  }
}

void test_time_restricted_road_allowed_on_timedep() {
  {
    // Pedestrian at hour 13 should be allowed
    std::string costing_str("pedestrian");
    test_route_allowed(costing_str, 13);
  }
  {
    // A car at hour 22 should be allowed
    std::string costing_str("auto");
    test_route_allowed(costing_str, 22);
  }
}

} // anonymous namespace

int main() {
  test::suite suite("astar");

  // TODO: move to mjolnir?
  suite.test(TEST_CASE(make_tile));

  suite.test(TEST_CASE(TestTrivialPathForward));
  suite.test(TEST_CASE(TestTrivialPathReverse));
  suite.test(TEST_CASE(TestTrivialPathTriangle));
  suite.test(TEST_CASE(TestPartialDurationTrivial));
  suite.test(TEST_CASE(TestPartialDurationForward));
  suite.test(TEST_CASE(TestPartialDurationReverse));

  suite.test(TEST_CASE(DoConfig));
  suite.test(TEST_CASE(TestTrivialPathNoUturns));

  suite.test(TEST_CASE(test_deadend));
  suite.test(TEST_CASE(test_deadend_timedep_forward));
  suite.test(TEST_CASE(test_deadend_timedep_reverse));
  suite.test(TEST_CASE(test_oneway));
  suite.test(TEST_CASE(test_oneway_wrong_way));
  suite.test(TEST_CASE(test_time_restricted_road_bidirectional));
  suite.test(TEST_CASE(test_time_restricted_road_denied_on_timedep));
  suite.test(TEST_CASE(test_time_restricted_road_allowed_on_timedep));

  return suite.tear_down();
}
