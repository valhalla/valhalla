#include "midgard/logging.h"
#include "test.h"
#include <cstdint>

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
#include "mjolnir/util.h"
#include "odin/directionsbuilder.h"
#include "odin/worker.h"
#include "sif/costconstants.h"
#include "sif/dynamiccost.h"
#include "sif/pedestriancost.h"
#include "thor/bidirectional_astar.h"
#include "thor/pathalgorithm.h"
#include "thor/triplegbuilder.h"
#include "thor/unidirectional_astar.h"
#include "thor/worker.h"
#include "tyr/actor.h"
#include "tyr/serializers.h"

#include "gurka.h"

#include "proto/directions.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"

#include <boost/algorithm/string/join.hpp>
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
namespace vr = valhalla::tyr;

#include "mjolnir/directededgebuilder.h"
#include "mjolnir/graphtilebuilder.h"

namespace {

// ph34r the ASCII art diagram:
//
// first test is just a square set of roads
const std::string map1 = R"(
   a1------------2b
   |              |
   |              |
   |              |
   |              |
   |              3
   c--------------d
)";

const gurka::ways ways1 = {{"ab", {{"highway", "motorway"}}},
                           {"bd", {{"highway", "motorway"}}},
                           {"ac", {{"highway", "motorway"}}},
                           {"dc", {{"highway", "motorway"}}}};

//
// second test is a triangle set of roads, where the height of the triangle is
// about a third of its width.

const std::string map2 = R"(
    e4--------5f
    \         /
     \       /
      \     /
       \   /
        \ /
         g
)";
const gurka::ways ways2 = {{"ef", {{"highway", "residential"}, {"foot", "yes"}}},
                           {"eg", {{"highway", "residential"}, {"foot", "yes"}}},
                           {"fg", {{"highway", "residential"}, {"foot", "yes"}}}};

// Third test has a complex turn restriction preventing K->H->I->L  (marked with R)
// which should force the algorithm to take the detour via the J->M edge
// if starting at K and heading to L
//
const std::string map3 = R"(
   h---V----i--------j
   |        |        |
   |        |        |
   6        7        |
   |        |        |
   |        |        |
   k        l8-------m
   |
   |
   |
   n
)";
const gurka::ways ways3 = {{"kh", {{"highway", "motorway"}}}, {"hi", {{"highway", "motorway"}}},
                           {"ij", {{"highway", "motorway"}}}, {"lm", {{"highway", "motorway"}}},
                           {"mj", {{"highway", "motorway"}}}, {"il", {{"highway", "motorway"}}},
                           {"nk", {{"highway", "motorway"}}}};

const gurka::relations relations3 = {{{gurka::relation_member{gurka::way_member, "kh", "from"},
                                       gurka::relation_member{gurka::way_member, "il", "to"},
                                       gurka::relation_member{gurka::way_member, "hi", "via"}},
                                      {{"type", "restriction"}, {"restriction", "no_right_turn"}}}};

const vb::GraphId tile_id = vb::TileHierarchy::GetGraphId({.125, .125}, 2);
gurka::nodelayout node_locations;
const std::string test_dir = "test/data/fake_tiles_astar";
const auto fake_conf =
    test::make_config(test_dir,
                      {{"mjolnir.admin", VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"},
                       {"mjolnir.timezone", VALHALLA_SOURCE_DIR "test/data/not_needed.sqlite"},
                       {"mjolnir.hierarchy", "false"},
                       {"mjolnir.shortcuts", "false"}});

void make_tile() {

  if (filesystem::exists(test_dir))
    filesystem::remove_all(test_dir);

  filesystem::create_directories(test_dir);

  const double gridsize = 666;

  {
    // Build the maps from the ASCII diagrams, and extract the generated lon,lat values
    auto nodemap = gurka::detail::map_to_coordinates(map1, gridsize, {0, 0.2});
    const int initial_osm_id = 0;
    gurka::detail::build_pbf(nodemap, ways1, {}, {}, test_dir + "/map1.pbf", initial_osm_id);
    for (const auto& n : nodemap)
      node_locations[n.first] = n.second;
  }

  {
    auto nodemap = gurka::detail::map_to_coordinates(map2, gridsize, {0.10, 0.2});
    // Need to use a non-conflicting osm ID range for each map, as they
    // all get merged during tile building, and we don't want a weirdly connected
    // graph because IDs are shared
    const int initial_osm_id = 100;
    gurka::detail::build_pbf(nodemap, ways2, {}, {}, test_dir + "/map2.pbf", initial_osm_id);
    for (const auto& n : nodemap)
      node_locations[n.first] = n.second;
  }

  {
    auto nodemap = gurka::detail::map_to_coordinates(map3, gridsize, {0.1, 0.1});
    const int initial_osm_id = 200;
    gurka::detail::build_pbf(nodemap, ways3, {}, relations3, test_dir + "/map3.pbf", initial_osm_id);
    for (const auto& n : nodemap)
      node_locations[n.first] = n.second;
  }

  {
    constexpr bool release_osmpbf_memory = false;
    mjolnir::build_tile_set(fake_conf,
                            {test_dir + "/map1.pbf", test_dir + "/map2.pbf", test_dir + "/map3.pbf"},
                            mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                            release_osmpbf_memory);
    /** Set the freeflow and constrained flow speeds manually on all edges */
    vj::GraphTileBuilder tile_builder(test_dir, tile_id, false);
    std::vector<DirectedEdge> directededges;
    directededges.reserve(tile_builder.header()->directededgecount());
    for (uint32_t j = 0; j < tile_builder.header()->directededgecount(); ++j) {
      // skip edges for which we dont have speed data
      DirectedEdge& directededge = tile_builder.directededge(j);
      directededge.set_free_flow_speed(100);
      directededge.set_constrained_flow_speed(10);
      directededge.set_forwardaccess(vb::kAllAccess);
      directededge.set_reverseaccess(vb::kAllAccess);
      directededges.emplace_back(std::move(directededge));
    }
    tile_builder.UpdatePredictedSpeeds(directededges);
  }

  auto tile = GraphTile::Create(test_dir, tile_id);
  ASSERT_TRUE(tile);
  ASSERT_EQ(tile->FileSuffix(tile_id), std::string("2/000/519/120.gph"))
      << "Tile ID didn't match the expected filename";

  ASSERT_PRED1(filesystem::exists,
               test_dir + filesystem::path::preferred_separator + tile->FileSuffix(tile_id))
      << "Expected tile file didn't show up on disk - are the fixtures in the right location?";
}

void create_costing_options(Options& options, Costing::Type costing) {
  const rapidjson::Document doc;
  options.set_costing_type(costing);
  sif::ParseCosting(doc, "/costing_options", options);
}
// Convert locations to format needed by PathAlgorithm
std::vector<valhalla::Location> ToPBFLocations(const std::vector<vb::Location>& locations,
                                               vb::GraphReader& graphreader,
                                               const std::shared_ptr<vs::DynamicCost>& costing) {
  const auto projections = loki::Search(locations, graphreader, costing);
  std::vector<valhalla::Location> result;
  for (const auto& loc : locations) {
    valhalla::Location pbfLoc;
    const auto& correlated = projections.at(loc);
    PathLocation::toPBF(correlated, &pbfLoc, graphreader);
    result.emplace_back(std::move(pbfLoc));
  }
  return result;
}

enum class TrivialPathTest {
  MatchesEdge,
  DurationEqualTo,
};

std::unique_ptr<vb::GraphReader> get_graph_reader(const std::string& tile_dir) {
  // make the config file
  std::stringstream json;
  json << "{ \"tile_dir\": \"" << tile_dir << "\" }";
  bpt::ptree conf;
  rapidjson::read_json(json, conf);

  std::unique_ptr<vb::GraphReader> reader(new vb::GraphReader(conf));
  auto tile = reader->GetGraphTile(tile_id);

  EXPECT_NE(tile, nullptr) << "Unable to load test tile! Did `make_tile` run successfully?";
  if (tile->header()->directededgecount() != 28) {
    throw std::logic_error("test-tiles does not contain expected number of edges");
  }

  auto endtile = reader->GetGraphTile(node_locations["b"]);
  EXPECT_NE(endtile, nullptr) << "bad tile, node 'b' wasn't found in it";

  return reader;
}

// check that a path from origin to dest goes along the edge with expected_edge_index
void assert_is_trivial_path(vt::PathAlgorithm& astar,
                            valhalla::Location& origin,
                            valhalla::Location& dest,
                            uint32_t expected_num_paths,
                            TrivialPathTest assert_type,
                            int32_t assert_type_value,
                            vs::TravelMode mode = vs::TravelMode::kPedestrian) {

  auto reader = get_graph_reader(test_dir);

  Options options;
  auto costing = mode == vs::TravelMode::kPedestrian ? Costing::pedestrian : Costing::auto_;
  create_costing_options(options, costing);
  auto mode_costing = sif::CostFactory().CreateModeCosting(options, mode);
  ASSERT_TRUE(bool(mode_costing[int(mode)]));

  auto paths = astar.GetBestPath(origin, dest, *reader, mode_costing, mode);

  int32_t time = 0;
  for (const auto& path : paths) {
    for (const auto& p : path) {
      time += p.elapsed_cost.secs;
    }
    EXPECT_EQ(path.size(), expected_num_paths);
    break;
  }

  auto tile = reader->GetGraphTile(tile_id);
  uint32_t expected_time = 979797;
  switch (assert_type) {
    case TrivialPathTest::DurationEqualTo:
      // Supply duration directly
      expected_time = assert_type_value;
      break;
    case TrivialPathTest::MatchesEdge:
      // Grab time from an edge index
      const DirectedEdge* expected_edge = tile->directededge(assert_type_value);
      auto expected_cost = mode_costing[int(mode)]->EdgeCost(expected_edge, tile);
      expected_time = expected_cost.secs;
      break;
  };
  EXPECT_NE(expected_time, 0) << "Expected time is 0, your test probably has a logic error";
  EXPECT_EQ(time, expected_time) << "time in seconds";
}

// test that a path from A to B succeeds, even if the edges from A to C and B
// to D appear first in the PathLocation.
void TestTrivialPath(vt::PathAlgorithm& astar) {

  Options options;
  create_costing_options(options, Costing::auto_);
  sif::TravelMode mode;
  auto mode_costing = sif::CostFactory().CreateModeCosting(options, mode);

  auto reader = get_graph_reader(test_dir);

  std::vector<valhalla::baldr::Location> locations;
  locations.push_back({node_locations["1"]});
  locations.push_back({node_locations["2"]});

  const auto projections = loki::Search(locations, *reader, mode_costing[static_cast<size_t>(mode)]);
  valhalla::Location origin;
  {
    const auto& correlated = projections.at(locations[0]);
    PathLocation::toPBF(correlated, &origin, *reader);
    origin.set_date_time("2019-11-21T13:05");
  }
  valhalla::Location dest;
  {
    const auto& correlated = projections.at(locations[1]);
    PathLocation::toPBF(correlated, &dest, *reader);
    dest.set_date_time("2019-11-21T13:05");
  }

  // this should go along the path from A to B
  assert_is_trivial_path(astar, origin, dest, 1, TrivialPathTest::DurationEqualTo, 3116,
                         vs::TravelMode::kDrive);
}

TEST(Astar, TestTrivialPathForward) {
  vt::TimeDepForward astar;
  TestTrivialPath(astar);
}

TEST(Astar, TestTrivialPathReverse) {
  vt::TimeDepReverse astar;
  TestTrivialPath(astar);
}

// test that a path from E to F succeeds, even if the edges from E and F
// to G appear first in the PathLocation.
TEST(Astar, TestTrivialPathTriangle) {

  Options options;
  create_costing_options(options, Costing::pedestrian);
  vs::TravelMode mode;
  auto costs = vs::CostFactory().CreateModeCosting(options, mode);

  auto reader = get_graph_reader(test_dir);

  std::vector<valhalla::baldr::Location> locations;
  locations.push_back({node_locations["4"]});
  locations.push_back({node_locations["5"]});

  const auto projections = loki::Search(locations, *reader, costs[static_cast<size_t>(mode)]);
  valhalla::Location origin;
  {
    const auto& correlated = projections.at(locations[0]);
    PathLocation::toPBF(correlated, &origin, *reader);
  }
  valhalla::Location dest;
  {
    const auto& correlated = projections.at(locations[1]);
    PathLocation::toPBF(correlated, &dest, *reader);
  }

  // TODO This fails with graphindex out of bounds for Reverse direction, is this
  // related to why we short-circuit trivial routes to AStarPathAlgorithm in route_action.cc?
  //
  vt::TimeDepForward astar;
  // this should go along the path from E to F
  assert_is_trivial_path(astar, origin, dest, 1, TrivialPathTest::DurationEqualTo, 4231,
                         vs::TravelMode::kPedestrian);
}

void TestPartialDuration(vt::PathAlgorithm& astar) {
  // Tests that a partial duration is returned when starting on a partial edge

  Options options;
  create_costing_options(options, Costing::auto_);
  vs::TravelMode mode;
  auto costs = vs::CostFactory().CreateModeCosting(options, mode);

  auto reader = get_graph_reader(test_dir);

  std::vector<valhalla::baldr::Location> locations;
  locations.push_back({node_locations["1"]});
  locations.push_back({node_locations["3"]});

  auto projections = loki::Search(locations, *reader, costs[static_cast<size_t>(mode)]);
  valhalla::Location origin;
  {
    auto& correlated = projections.at(locations[0]);
    PathLocation::toPBF(correlated, &origin, *reader);
    origin.set_date_time("2019-11-21T13:05");
  }

  valhalla::Location dest;
  {
    auto& correlated = projections.at(locations[1]);
    PathLocation::toPBF(correlated, &dest, *reader);
    dest.set_date_time("2019-11-21T13:05");
  }

  uint32_t expected_duration = 7911;

  assert_is_trivial_path(astar, origin, dest, 2, TrivialPathTest::DurationEqualTo, expected_duration,
                         vs::TravelMode::kDrive);
}

TEST(Astar, TestPartialDurationForward) {
  vt::TimeDepForward astar;
  TestPartialDuration(astar);
}

TEST(Astar, TestPartialDurationReverse) {
  vt::TimeDepReverse astar;
  TestPartialDuration(astar);
}

TEST(Astar, TestTrivialPathNoUturns) {
  boost::property_tree::ptree conf = test::make_config("test/data/utrecht_tiles");
  vr::actor_t actor(conf);
  valhalla::Api api;
  actor.route(
      R"({"costing":"pedestrian","locations":[{"lon":5.114587,"lat":52.095957},{"lon":5.114506,"lat":52.096141}]})",
      {}, &api);
  EXPECT_EQ(api.directions().routes(0).legs(0).summary().time(), 0);
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
    // Cleanup
    loki_worker.cleanup();
    thor_worker.cleanup();
    odin_worker.cleanup();
    return request;
  }
  boost::property_tree::ptree conf;
  std::shared_ptr<GraphReader> reader;
  vk::loki_worker_t loki_worker;
  vt::thor_worker_t thor_worker;
  vo::odin_worker_t odin_worker;
};

TEST(Astar, test_oneway) {
  auto conf = test::make_config("test/data/whitelion_tiles");
  route_tester tester(conf);
  // Test onewayness with this route - oneway works, South-West to North-East
  std::string request =
      R"({"locations":[{"lat":51.455768530466514,"lon":-2.5954368710517883},{"lat":51.456082740244824,"lon":-2.595050632953644}],"costing":"auto"})";

  auto response = tester.test(request);

  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  EXPECT_EQ(legs.size(), 1);

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

  auto correct_route = std::vector<std::string>{"Quay Street", ""};
  EXPECT_EQ(names, correct_route) << "Incorrect route, got: \n" +
                                         boost::algorithm::join(names, ", ") + ", expected: \n" +
                                         boost::algorithm::join(correct_route, ", ");
}

TEST(Astar, test_oneway_wrong_way) {
  auto conf = test::make_config("test/data/whitelion_tiles");
  route_tester tester(conf);
  // Test onewayness with this route - oneway wrong way, North-east to South-West
  // Should produce no-route
  std::string request =
      R"({"locations":[{"lat":51.456082740244824,"lon":-2.595050632953644},{"lat":51.455768530466514,"lon":-2.5954368710517883}],"costing":"auto"})";

  try {
    auto response = tester.test(request);
    FAIL() << "Expected exception!";
  } catch (const std::exception& e) {
    EXPECT_EQ(std::string(e.what()), "No path could be found for input");
  } catch (...) { FAIL() << "Wrong exception type"; }
}

TEST(Astar, test_deadend) {
  auto conf = test::make_config("test/data/whitelion_tiles");
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

  EXPECT_EQ(legs.size(), 1);

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
      if (m.type() == DirectionsLeg_Maneuver_Type_kUturnRight ||
          m.type() == DirectionsLeg_Maneuver_Type_kUturnLeft) {
        uturn_street = name;
      }
      names.push_back(name);
    }
  }

  auto correct_route =
      std::vector<std::string>{"Bell Lane", "Small Street",
                               "Quay Street", // The u-turn on Quay Street is optimized away
                               "", ""};
  EXPECT_EQ(names, correct_route) << "Incorrect route, got: \n" +
                                         boost::algorithm::join(names, ", ") + ", expected: \n" +
                                         boost::algorithm::join(correct_route, ", ");

  EXPECT_EQ(uturn_street, "Quay Street") << "We did not find the expected u-turn";
}
TEST(Astar, test_time_dep_forward_with_current_time) {
  // Test a request with date_time as "current" (type: 0)
  //
  auto conf = test::make_config("test/data/whitelion_tiles_reverse");
  route_tester tester(conf);
  std::string request =
      R"({
      "locations":[
        {"lat":51.45562646682483,"lon":-2.5952598452568054},
        {"lat":51.455143447135974,"lon":-2.5958767533302307}
      ],
      "costing":"auto",
      "date_time":{
        "type":0
      }
    })";

  auto response = tester.test(request);

  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  EXPECT_EQ(legs.size(), 1) << "Should have 1 leg";

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

  auto correct_route =
      std::vector<std::string>{"Bell Lane", "Small Street",
                               "Quay Street", // The u-turn on Quay Street is optimized away
                               "", ""};
  EXPECT_EQ(names, correct_route) << "Incorrect route, got: \n" +
                                         boost::algorithm::join(names, ", ") + ", expected: \n" +
                                         boost::algorithm::join(correct_route, ", ");
}

TEST(Astar, test_deadend_timedep_forward) {
  auto conf = test::make_config("test/data/whitelion_tiles_reverse");
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

  EXPECT_EQ(legs.size(), 1);

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
      if (m.type() == DirectionsLeg_Maneuver_Type_kUturnRight ||
          m.type() == DirectionsLeg_Maneuver_Type_kUturnLeft) {
        uturn_street = name;
      }
      names.push_back(name);
    }
  }

  auto correct_route =
      std::vector<std::string>{"Bell Lane", "Small Street",
                               "Quay Street", // The u-turn on Quay Street is optimized away
                               "", ""};
  EXPECT_EQ(names, correct_route) << "Incorrect route, got: \n" +
                                         boost::algorithm::join(names, ", ") + ", expected: \n" +
                                         boost::algorithm::join(correct_route, ", ");

  EXPECT_EQ(uturn_street, "Quay Street") << "We did not find the expected u-turn";
}
TEST(Astar, test_deadend_timedep_reverse) {
  auto conf = test::make_config("test/data/whitelion_tiles");
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

  EXPECT_EQ(legs.size(), 1);

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
      if (m.type() == DirectionsLeg_Maneuver_Type_kUturnRight ||
          m.type() == DirectionsLeg_Maneuver_Type_kUturnLeft) {
        uturn_street = name;
      }
      names.push_back(name);
    }
  }

  auto correct_route =
      std::vector<std::string>{"Bell Lane", "Small Street",
                               "Quay Street", // The u-turn on Quay Street is optimized away
                               "", ""};
  EXPECT_EQ(names, correct_route) << "Incorrect route, got: \n" +
                                         boost::algorithm::join(names, ", ") + ", expected: \n" +
                                         boost::algorithm::join(correct_route, ", ");

  EXPECT_EQ(uturn_street, "Quay Street") << "We did not find the expected u-turn";
}

TEST(Astar, test_time_restricted_road_bidirectional) {
  // Try routing over "Via Montebello" in Rome which is a time restricted road
  // We should receive a route for a time-independent query but have the response
  // note that it is time restricted
  auto conf = test::make_config("test/data/roma_tiles");
  route_tester tester(conf);
  std::string request =
      R"({"locations":[{"lat":41.90550,"lon":12.50090},{"lat":41.90477,"lon":12.49914}],"costing":"auto"})";

  auto response = tester.test(request);

  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  EXPECT_EQ(legs.size(), 1);

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
  EXPECT_EQ(names, correct_route) << "Incorrect route, got: \n" +
                                         boost::algorithm::join(names, ", ") + ", expected: \n" +
                                         boost::algorithm::join(correct_route, ", ");

  EXPECT_TRUE(response.trip().routes(0).legs(0).node(1).edge().has_time_restrictions())
      << "Expected leg to have time_restriction";

  // Verify JSON payload
  const std::string payload = tyr::serializeDirections(response);
  rapidjson::Document response_json;
  response_json.Parse(payload.c_str());
  std::cout << payload << std::endl;
  {
    const char key[] = "/trip/legs/0/maneuvers/0/has_time_restrictions";
    EXPECT_TRUE(GetValueByPointerWithDefault(response_json, key, false) != true)
        << "Via Goito is marked as time-restricted which is incorrect! JSON does have " << key
        << " set to true";
  }
  {
    const char key[] = "/trip/legs/0/maneuvers/1/has_time_restrictions";
    EXPECT_TRUE(GetValueByPointerWithDefault(response_json, key, false) == true)
        << std::string("JSON does not have ") + key + " set to true";
  }
  {
    const char key[] = "/trip/legs/0/summary/has_time_restrictions";
    EXPECT_TRUE(GetValueByPointerWithDefault(response_json, key, false) == true)
        << std::string("JSON does not have ") + key + " set to true";
  }
  {
    const char key[] = "/trip/summary/has_time_restrictions";
    EXPECT_TRUE(GetValueByPointerWithDefault(response_json, key, false) == true)
        << std::string("JSON does not have ") + key + " set to true";
  }
}

Api route_on_timerestricted(const std::string& costing_str, int16_t hour) {
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
  auto conf = test::make_config("test/data/roma_tiles");
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

void test_route_restricted(const std::string& costing_str, int16_t hour) {
  bool found_route = false;
  try {
    auto response = route_on_timerestricted(costing_str, hour);
    found_route = true;
    const auto& leg = response.directions().routes(0).legs(0);
    LOG_INFO("Route that wasn't supposed to happen: " + leg.shape());
  } catch (const std::exception& e) {
    EXPECT_EQ(std::string(e.what()), "No path could be found for input");
    return;
  }
  EXPECT_FALSE(found_route) << "Found a route when no route was expected";
}

TEST(Astar, test_time_restricted_road_denied_on_timedep) {
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

void test_route_allowed(const std::string& costing_str, int16_t hour) {
  auto response = route_on_timerestricted(costing_str, hour);
  const auto& legs = response.trip().routes(0).legs();
  EXPECT_EQ(legs.size(), 1) << "Should have 1 leg";

  const auto& directions = response.directions().routes(0).legs(0);
  const auto& summary = directions.summary();
  EXPECT_NE(summary.time(), 0) << "Time shouldn't be 0";
}

TEST(Astar, test_time_restricted_road_allowed_on_timedep) {
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

void test_backtrack_complex_restriction(int date_time_type) {
  // Regression test for backtracking complex restriction behaviour.
  //
  // Test-case documented in https://github.com/valhalla/valhalla/issues/2103
  //
  auto conf = test::make_config("test/data/bayfront_singapore_tiles");
  route_tester tester(conf);
  std::string request;
  switch (date_time_type) {
    case 0:
      // Bidir search
      request = R"({
        "locations": [
          {
            "lat":1.282185,
            "lon":103.859650,
            "street":"Sheares Link"
          },
          {
            "lat":1.282493,
            "lon":103.859421,
            "street":"Sheares Link"
          }
        ],
        "costing":"auto"
      })";
      break;
    case 1:
      // Forward search
      request = R"({
        "locations": [
          {
            "lat":1.282185,
            "lon":103.859650,
            "street":"Sheares Link"
          },
          {
            "lat":1.282493,
            "lon":103.859421,
            "street":"Sheares Link"
          }
        ],
        "costing":"auto",
        "date_time": {
          "type": 1,
          "value": "2019-05-02T15:00"
        }
      })";
      break;
    case 2:
      // Backward search with slightly different coordinates
      request = R"({
        "locations": [
          {
            "lat":1.282366,
            "lon":-256.140661,
            "street":"Sheares Link",
            "minimum_reachability": 0
          },
          {
            "lat":1.282355,
            "lon":-256.140414,
            "street":"Sheares Link"
          }
        ],
        "costing":"auto",
        "date_time": {
          "type": 2,
          "value": "2019-05-02T15:00"
        }
      })";
      break;
    default:
      throw std::runtime_error("Unhandled case");
  }

  LOGLN_WARN(request);
  auto response = tester.test(request);

  const auto& leg = response.trip().routes(0).legs(0);
  std::string correct_shape;
  switch (date_time_type) {
    case 0:
    case 1:
      correct_shape =
          R"(kggmA_{abeEyDbIaBtCp@Z|d@pYfGdCp]xRnCzArDlB{C`FqDyBwC_BsYmP}KoGsAw@wJmGcUcLo@[qFaDz@aB)";
      break;
    case 2:
      correct_shape =
          R"(wrgmAsgabeEy@hBqFaDkBcA_WcNkBw@iX_OuJ}EcIgFcVgL}ZgJoVeE^yEy@uBb@PvQfE|b@bLdIpDd`@|Sh\vQ~XxNfB~@pC}E)";
      break;
    default:
      throw std::runtime_error("unhandled case");
  }
  EXPECT_TRUE(test::encoded_shape_equality(leg.shape(), correct_shape))
      << "Did not find expected shape. Found \n" + leg.shape() + "\nbut expected \n" + correct_shape;

  std::vector<std::string> names;
  const auto& directions = response.directions().routes(0).legs();

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
  auto correct_route = std::vector<std::string>{"Sheares Link", "Bayfront Avenue", "Bayfront Avenue",
                                                "Sheares Link", ""};
  if (names != correct_route) {
    throw std::logic_error("Incorrect route, got: \n" + boost::algorithm::join(names, ", ") +
                           ", expected: \n" + boost::algorithm::join(correct_route, ", "));
  }
}

TEST(Astar, TestBacktrackComplexRestrictionForward) {
  test_backtrack_complex_restriction(1);
}

TEST(Astar, TestBacktrackComplexRestrictionReverse) {
  // Reverse direction condition is triggered via use of slightly tweaked start/end coordinates
  test_backtrack_complex_restriction(2);
}

TEST(Astar, TestBacktrackComplexRestrictionBidirectional) {
  // Bidirectional routed before via the reverse direction search
  // So this becomes more of a regression test
  test_backtrack_complex_restriction(0);
}

TEST(Astar, TestBacktrackComplexRestrictionForwardDetourAfterRestriction) {
  // This tests if a detour _after_ a partial complex restriction is found.
  // The other tests with Bayfront Singapore tests with a detour _before_
  // the complex restriction

  Options options;
  create_costing_options(options, Costing::auto_);
  vs::TravelMode mode;
  auto costs = vs::CostFactory().CreateModeCosting(options, mode);
  ASSERT_TRUE(bool(costs[int(mode)]));

  auto reader = get_graph_reader(test_dir);

  auto tile = reader->GetGraphTile(tile_id);

  auto verify_paths = [&](const std::vector<vt::PathInfo>& paths) {
    std::vector<std::string> walked_path;
    for (auto path_info : paths) {
      LOG_INFO("Got pathinfo " + std::to_string(path_info.edgeid.id()));
      auto directededge = tile->directededge(path_info.edgeid);
      auto edgeinfo = tile->edgeinfo(directededge);
      auto names = edgeinfo.GetNames();
      walked_path.push_back(names.front());
    }
    std::vector<std::string> expected_path;
    expected_path.push_back("kh");
    expected_path.push_back("hi");
    expected_path.push_back("ij");
    expected_path.push_back("mj");
    expected_path.push_back("lm");
    expected_path.push_back("il");
    ASSERT_EQ(walked_path, expected_path) << "Wrong path";
  };

  std::vector<valhalla::baldr::Location> locations;
  locations.push_back({node_locations["6"]});
  locations.push_back({node_locations["7"]});

  const auto projections = loki::Search(locations, *reader, costs[int(mode)]);

  std::vector<PathLocation> path_location;
  for (const auto& loc : locations) {
    ASSERT_NO_THROW(
        path_location.push_back(projections.at(loc));
        PathLocation::toPBF(path_location.back(), options.mutable_locations()->Add(), *reader);)
        << "fail_invalid_origin";
  }

  // Set departure date for timedep forward
  options.mutable_locations(0)->set_date_time("2019-11-21T13:05");

  {
    vt::TimeDepForward astar;
    auto paths = astar
                     .GetBestPath(*options.mutable_locations(0), *options.mutable_locations(1),
                                  *reader, costs, mode)
                     .front();

    verify_paths(paths);
  }
  {
    vt::TimeDepReverse astar;
    auto paths = astar
                     .GetBestPath(*options.mutable_locations(0), *options.mutable_locations(1),
                                  *reader, costs, mode)
                     .front();

    verify_paths(paths);
  }
  {
    vt::BidirectionalAStar astar;
    auto paths = astar
                     .GetBestPath(*options.mutable_locations(0), *options.mutable_locations(1),
                                  *reader, costs, mode)
                     .front();

    verify_paths(paths);
  }
}

Api timed_access_restriction_ny(const std::string& mode, const std::string& datetime) {
  // The restriction is <tag k="bicycle:conditional" v="no @ (Su 08:00-18:00)"/>
  // and <tag k="motor_vehicle:conditional" v="no @ (Su 08:00-18:00)"/>
  auto conf = test::make_config("test/data/ny_ar_tiles");
  route_tester tester(conf);
  LOG_INFO("Testing " + mode + " route at " + datetime);

  std::string request =
      R"({
            "locations":[{"lat":40.71835519823214,"lon":-73.99010449658817},{"lat":40.72136384343179,"lon":-73.98817330609745}],
            "costing":")" +
      mode + R"(",
              "date_time":{
                "type":1,
                "value":")" +
      datetime + R"("
          }
        })";
  return tester.test(request);
}

// The following requests results in timedep astar during the non-restricted hours
// and should be allowed
TEST(Astar, test_timed_no_access_restriction_1) {
  auto response = timed_access_restriction_ny("bicycle", "2018-05-13T19:14");
  const auto& legs = response.trip().routes(0).legs();
  EXPECT_EQ(legs.size(), 1) << "Should have 1 leg";
  const auto& directions = response.directions().routes(0).legs(0);
  const auto& maneuvers_size = directions.maneuver_size();
  EXPECT_EQ(maneuvers_size, 3) << "This route should remain on Orchard St.";
}

TEST(Astar, test_timed_no_access_restriction_2) {
  auto response = timed_access_restriction_ny("bicycle", "2018-05-14T17:14");
  const auto& legs = response.trip().routes(0).legs();
  EXPECT_EQ(legs.size(), 1) << "Should have 1 leg";
  const auto& directions = response.directions().routes(0).legs(0);
  const auto& maneuvers_size = directions.maneuver_size();
  EXPECT_EQ(maneuvers_size, 3) << "This route should remain on Orchard St.";
}

TEST(Astar, test_timed_no_access_restriction_3) {
  auto response = timed_access_restriction_ny("pedestrian", "2018-05-13T17:14");
  const auto& legs = response.trip().routes(0).legs();
  EXPECT_EQ(legs.size(), 1) << "Should have 1 leg";
  const auto& directions = response.directions().routes(0).legs(0);
  const auto& maneuvers_size = directions.maneuver_size();
  EXPECT_EQ(maneuvers_size, 3) << "This route should remain on Orchard St.";
}

// The following requests results in timedep astar during the restricted hours
// and should be denied
TEST(Astar, test_timed_access_restriction_1) {
  auto response = timed_access_restriction_ny("bicycle", "2018-05-13T17:14");
  const auto& legs = response.trip().routes(0).legs();
  EXPECT_EQ(legs.size(), 1) << "Should have 1 leg";
  const auto& directions = response.directions().routes(0).legs(0);
  const auto& maneuvers_size = directions.maneuver_size();
  EXPECT_NE(maneuvers_size, 3)
      << "This route should turn L onto Delancey St. because of restriction. ";
}

TEST(Astar, test_timed_access_restriction_2) {
  auto response = timed_access_restriction_ny("auto", "2018-05-13T17:14");
  const auto& legs = response.trip().routes(0).legs();
  EXPECT_EQ(legs.size(), 1) << "Should have 1 leg";
  const auto& directions = response.directions().routes(0).legs(0);
  const auto& maneuvers_size = directions.maneuver_size();
  EXPECT_NE(maneuvers_size, 3)
      << "This route should turn L onto Delancey St. because of restriction. ";
}

Api timed_conditional_restriction_pa(const std::string& mode, const std::string& datetime) {
  // The restriction is <tag k="restriction:conditional" v="no_right_turn @ (Mo-Fr 07:00-09:00)"/>
  auto conf = test::make_config("test/data/pa_ar_tiles");
  route_tester tester(conf);
  LOG_INFO("Testing " + mode + " route at " + datetime);

  std::string request =
      R"({
            "locations":[{"lat":40.234100,"lon":-76.933037},{"lat":40.234734,"lon":-76.932022}],
            "costing":")" +
      mode + R"(",
              "date_time":{
                "type":1,
                "value":")" +
      datetime + R"("
          }
        })";
  return tester.test(request);
}

Api timed_conditional_restriction_nh(const std::string& mode, const std::string& datetime) {
  // The restriction is <tag k="hgv:conditional" v="no @ (19:00-06:00)"/>
  auto conf = test::make_config("test/data/nh_ar_tiles");
  route_tester tester(conf);
  LOG_INFO("Testing " + mode + " route at " + datetime);

  std::string request =
      R"({
            "locations":[{"lat":42.79615642306863,"lon":-71.43550157459686},{"lat":42.79873856769978,"lon":-71.43146753223846}],
            "costing":")" +
      mode +
      R"(","costing_options":{"truck":{"height":"4.11","width":"2.6","length":"21.64","weight":"21.77","axle_load":"9.07","hazmat":false}},
              "date_time":{
                "type":1,
                "value":")" +
      datetime + R"("
          }
        })";
  return tester.test(request);
}

// The following requests results in timedep astar during the non-restricted hours
// and should be allowed
TEST(Astar, test_timed_no_conditional_restriction_1) {
  auto response = timed_conditional_restriction_pa("auto", "2018-11-01T06:30");
  const auto& legs = response.trip().routes(0).legs();
  EXPECT_EQ(legs.size(), 1) << "Should have 1 leg";
  const auto& directions = response.directions().routes(0).legs(0);
  const auto& maneuvers_size = directions.maneuver_size();
  EXPECT_EQ(maneuvers_size, 3) << "This route should turn R onto Dickinson Ave.";
}

TEST(Astar, test_timed_no_conditional_restriction_2) {
  auto response = timed_conditional_restriction_pa("auto", "2018-11-01T10:00");
  const auto& legs = response.trip().routes(0).legs();
  EXPECT_EQ(legs.size(), 1) << "Should have 1 leg";
  const auto& directions = response.directions().routes(0).legs(0);
  const auto& maneuvers_size = directions.maneuver_size();
  EXPECT_EQ(maneuvers_size, 3) << "This route should turn R onto Dickinson Ave.";
}

TEST(Astar, test_timed_no_conditional_restriction_3) {
  auto response = timed_conditional_restriction_nh("truck", "2018-05-02T18:00");
  const auto& legs = response.trip().routes(0).legs();
  EXPECT_EQ(legs.size(), 1) << "Should have 1 leg";
  const auto& directions = response.directions().routes(0).legs(0);
  const auto& maneuvers_size = directions.maneuver_size();
  EXPECT_LE(maneuvers_size, 3) << "This route should turn R onto Old Derry Rd.";
}

// The following requests results in timedep astar during the restricted hours
// and should be denied
TEST(Astar, test_timed_conditional_restriction_1) {
  auto response = timed_conditional_restriction_pa("auto", "2018-11-01T07:00");
  const auto& legs = response.trip().routes(0).legs();
  EXPECT_EQ(legs.size(), 1) << "Should have 1 leg";
  const auto& directions = response.directions().routes(0).legs(0);
  const auto& maneuvers_size = directions.maneuver_size();
  EXPECT_NE(maneuvers_size, 3) << "This route should turn L onto Dickinson Ave.";
}

TEST(Astar, test_timed_conditional_restriction_2) {
  auto response = timed_conditional_restriction_pa("auto", "2018-11-01T09:00");
  const auto& legs = response.trip().routes(0).legs();
  EXPECT_EQ(legs.size(), 1) << "Should have 1 leg";
  const auto& directions = response.directions().routes(0).legs(0);
  const auto& maneuvers_size = directions.maneuver_size();
  EXPECT_NE(maneuvers_size, 3) << "This route should turn L onto Dickinson Ave.";
}

TEST(Astar, test_timed_conditional_restriction_3) {
  bool found_route = false;
  try {
    auto response = timed_conditional_restriction_nh("truck", "2018-05-02T20:00");
    found_route = true;
    const auto& leg = response.directions().routes(0).legs(0);
    LOG_INFO("Route that wasn't supposed to happen: " + leg.shape());
  } catch (const std::exception& e) {
    EXPECT_EQ(std::string(e.what()), "No path could be found for input");
    return;
  }
  EXPECT_FALSE(found_route) << "Found a route when no route was expected";
}

TEST(Astar, test_complex_restriction_short_path_fake) {
  // Tests that Bidirectional can correctly connect the two expanding trees
  // when the connecting edge is part of a complex restriction

  auto reader = get_graph_reader(test_dir);
  Options options;
  create_costing_options(options, Costing::auto_);
  vs::TravelMode mode;
  auto costs = vs::CostFactory().CreateModeCosting(options, mode);
  ASSERT_TRUE(bool(costs[int(mode)]));

  std::vector<valhalla::baldr::Location> locations;
  locations.push_back({node_locations["n"]});
  locations.push_back({node_locations["i"]});

  const auto projections = loki::Search(locations, *reader, costs[int(mode)]);
  valhalla::Location origin;
  {
    const auto& correlated = projections.at(locations[0]);
    PathLocation::toPBF(correlated, &origin, *reader);
  }
  valhalla::Location dest;
  {
    const auto& correlated = projections.at(locations[1]);
    PathLocation::toPBF(correlated, &dest, *reader);
  }

  // Test Bidirectional both for forward and reverse expansion
  vt::BidirectionalAStar astar;

  // Two tests where start and end lives on a partial complex restriction
  //      Under this circumstance the restriction should _not_ trigger

  auto paths = astar.GetBestPath(origin, dest, *reader, costs, mode);

  std::vector<uint32_t> visited;
  for (auto& path_infos : paths) {
    for (auto path_info : path_infos) {
      visited.push_back(path_info.edgeid.id());
    }
  }
  {
    std::vector<uint32_t> expected;
    auto reader = get_graph_reader(test_dir);
    auto e1 = gurka::findEdge(*reader, node_locations, "nk", "k");
    auto e2 = gurka::findEdge(*reader, node_locations, "kh", "h");
    auto e3 = gurka::findEdge(*reader, node_locations, "hi", "i");
    expected.push_back(std::get<0>(e1).id());
    expected.push_back(std::get<0>(e2).id());
    expected.push_back(std::get<0>(e3).id());
    ASSERT_EQ(visited, expected) << "Unexpected edges in case 1 of bidirectional a*";
  }

  // For the second test, just switch origin/destination and reverse expected,
  // result should be the same
  std::cout << "reversed test" << std::endl;
  paths = astar.GetBestPath(dest, origin, *reader, costs, mode);

  visited.clear();
  for (auto& path_infos : paths) {
    for (auto path_info : path_infos) {
      visited.push_back(path_info.edgeid.id());
    }
  }

  {
    std::vector<uint32_t> expected;
    auto reader = get_graph_reader(test_dir);
    auto e1 = gurka::findEdge(*reader, node_locations, "hi", "h");
    auto e2 = gurka::findEdge(*reader, node_locations, "kh", "k");
    auto e3 = gurka::findEdge(*reader, node_locations, "nk", "n");
    expected.push_back(std::get<0>(e1).id());
    expected.push_back(std::get<0>(e2).id());
    expected.push_back(std::get<0>(e3).id());
    ASSERT_EQ(visited, expected) << "Unexpected edges in case 1 of bidirectional a*";
  }

  {
    // TestBacktrackComplexRestrictionBidirectional tests the behaviour with a
    // complex restriction between the two expanding
    // trees actually is a real one and needs to be avoided
  }
}

TEST(Astar, test_complex_restriction_short_path_melborne) {
  // Tests a real live scenario of a short Bidirectional query against "Melborne"
  auto conf = test::make_config("test/data/melborne_tiles");
  route_tester tester(conf);
  {
    // Tests "Route around the block" due to complex restriction,
    // fixed by IsBridgingEdgeRestricted
    std::string request =
        R"({"locations":[{"lat":-37.627860699397075,"lon":145.365825588286},{"lat":-37.62842169939707,"lon":145.36587158828598}],"costing":"auto"})";
    auto response = tester.test(request);
    const auto& leg = response.trip().routes(0).legs(0);
    EXPECT_TRUE(test::encoded_shape_equality(leg.shape(), "~{rwfAmslgtGxNkUvDtDtLjM"));
  }
  {
    // Tests "X-crossing",
    // fixed by IsBridgingEdgeRestricted
    std::string request =
        R"({"locations":[{"lat":-37.62403769939707,"lon":145.360320588286},{"lat":-37.624804699397075,"lon":145.36041758828597}],"costing":"auto"})";
    auto response = tester.test(request);
    const auto& leg = response.trip().routes(0).legs(0);
    EXPECT_TRUE(test::encoded_shape_equality(leg.shape(), "rmkwfAwzagtGlAgCnB}CfHcKzLk@lPbQ"));
  }
}

TEST(Astar, test_IsBridgingEdgeRestricted) {
  // Tests the IsBridgingEdgeRestricted function specifically with known inputs
  // which is simpler than trying to get BidirectionalAStar to call it with
  // a specific setup
  auto reader = get_graph_reader(test_dir);
  Options options;
  create_costing_options(options, Costing::auto_);
  vs::TravelMode mode;
  auto costs = vs::CostFactory().CreateModeCosting(options, mode);
  std::vector<sif::BDEdgeLabel> edge_labels_fwd;
  std::vector<sif::BDEdgeLabel> edge_labels_rev;

  // Lets construct the inputs fed to IsBridgingEdgeRestricted for a situation
  // where it tries to connect edge 14 to edge_labels_fwd from 21 and opposing edges
  // from 18
  DirectedEdge edge_nk;
  {
    auto result = gurka::findEdge(*reader, node_locations, "nk", "k");
    ASSERT_NE(nullptr, std::get<1>(result));
    edge_nk = *std::get<1>(result);
    edge_nk.complex_restriction(true);
    edge_labels_fwd.emplace_back(kInvalidLabel, std::get<0>(result), std::get<2>(result), &edge_nk,
                                 vs::Cost{}, vs::TravelMode::kDrive, vs::Cost{}, 0, false, false,
                                 true, sif::InternalTurn::kNoTurn, false);
  }
  DirectedEdge edge_kh;
  {
    auto result = gurka::findEdge(*reader, node_locations, "kh", "h");
    ASSERT_NE(nullptr, std::get<1>(result));
    edge_kh = *std::get<1>(result);
    edge_kh.complex_restriction(true);
    edge_labels_fwd.emplace_back(edge_labels_fwd.size() - 1, std::get<0>(result), std::get<2>(result),
                                 &edge_kh, vs::Cost{}, vs::TravelMode::kDrive, vs::Cost{}, 0, false,
                                 false, true, sif::InternalTurn::kNoTurn, false);
  }
  // Create our fwd_pred for the bridging check
  DirectedEdge edge_hi;
  auto edge_hi_result = gurka::findEdge(*reader, node_locations, "hi", "i");
  ASSERT_NE(nullptr, std::get<1>(edge_hi_result));
  edge_hi = *std::get<1>(edge_hi_result);
  edge_hi.complex_restriction(true);
  vs::BDEdgeLabel fwd_pred(edge_labels_fwd.size() - 1, // Index to predecessor in edge_labels_fwd
                           std::get<0>(edge_hi_result), std::get<2>(edge_hi_result), &edge_hi,
                           vs::Cost{}, 0.0, 0.0, vs::TravelMode::kDrive, vs::Cost{}, false, false,
                           true, sif::InternalTurn::kNoTurn, false);

  DirectedEdge edge_il;
  {
    auto result = gurka::findEdge(*reader, node_locations, "il", "i");
    ASSERT_NE(nullptr, std::get<1>(result));
    edge_il = *std::get<1>(result);
    edge_il.complex_restriction(true);
    edge_labels_rev.emplace_back(kInvalidLabel, std::get<0>(result), std::get<2>(result), &edge_il,
                                 vs::Cost{}, vs::TravelMode::kDrive, vs::Cost{}, 0, false, false,
                                 true, sif::InternalTurn::kNoTurn, false);
  }
  // Create the rev_pred for the bridging check
  DirectedEdge edge_ih;
  edge_ih = *std::get<3>(edge_hi_result); // use result from earlier, which already got opposing edge
  edge_ih.complex_restriction(true);
  vs::BDEdgeLabel rev_pred(edge_labels_rev.size() - 1, // Index to predecessor in edge_labels_rev
                           std::get<2>(edge_hi_result), std::get<0>(edge_hi_result), &edge_ih,
                           vs::Cost{}, 0.0, 0.0, vs::TravelMode::kDrive, vs::Cost{}, false, false,
                           true, sif::InternalTurn::kNoTurn, false);
  {
    // Test for forward search
    ASSERT_TRUE(vt::IsBridgingEdgeRestricted(*reader, edge_labels_fwd, edge_labels_rev, fwd_pred,
                                             rev_pred, costs[int(mode)]));
  }
}

TEST(ComplexRestriction, WalkVias) {
  // Yes, it's a little odd to have a test of restrictions and vias here, but
  // you need a baked tile to test this functionality which we conveniently
  // have here from `make_tile`.
  // TODO Future improvement would be to make it simpler to quickly generate
  // tiles programmatically
  auto reader = get_graph_reader(test_dir);
  Options options;
  create_costing_options(options, Costing::auto_);
  vs::TravelMode mode;
  auto costs = vs::CostFactory().CreateModeCosting(options, mode);
  auto costing = costs[int(mode)];

  bool is_forward = true;
  auto tile = reader->GetGraphTile(tile_id);

  std::vector<valhalla::baldr::Location> locations;
  locations.push_back({node_locations["7"]});
  const auto projections = loki::Search(locations, *reader, costing);
  const auto& correlated = projections.at(locations[0]);

  ASSERT_EQ(correlated.edges.size(), 2) << "Expected only 2 edges in snapping response";

  // Need to figure out if it's the forward or backward edge that we need to
  // use for walking
  const auto cr = [&]() -> ComplexRestriction* {
    const auto first_id = correlated.edges.front().id;
    auto restrictions = tile->GetRestrictions(is_forward, first_id, costing->access_mode());
    if (!restrictions.empty())
      return restrictions.front();
    const auto second_id = correlated.edges.back().id;
    restrictions = tile->GetRestrictions(is_forward, second_id, costing->access_mode());
    if (!restrictions.empty())
      return restrictions.front();
    return nullptr;
  }();

  ASSERT_NE(cr, nullptr) << "Failed to find the target edge of the test restriction";

  std::vector<GraphId> expected_vias;
  {
    std::vector<valhalla::baldr::Location> via_locations;
    via_locations.push_back({node_locations["V"]});
    const auto via_projections = loki::Search(via_locations, *reader, costing);
    const auto& via_correlated = via_projections.at(via_locations[0]);
    ASSERT_EQ(via_correlated.edges.size(), 2) << "Should've found 2 edges for the via point";

    auto* de1 = tile->directededge(via_correlated.edges.front().id);
    auto* de2 = tile->directededge(via_correlated.edges.back().id);
    if (de1->part_of_complex_restriction()) {
      expected_vias.push_back(via_correlated.edges.front().id);
    }
    if (de2->part_of_complex_restriction()) {
      expected_vias.push_back(via_correlated.edges.back().id);
    }
    ASSERT_LE(expected_vias.size(), 2) << "Found too many edges - max should be 2 (2 DirectedEdges)";
    ASSERT_NE(expected_vias.size(), 0) << "Failed to find the via edge";
  }

  {
    // Walk all vias
    std::vector<GraphId> walked_vias;
    cr->WalkVias([&walked_vias](const GraphId* via) {
      walked_vias.push_back(*via);
      return WalkingVia::KeepWalking;
    });
    EXPECT_EQ(walked_vias.size(), 1);
    EXPECT_NE(std::find(expected_vias.begin(), expected_vias.end(), walked_vias.front()),
              expected_vias.end())
        << "Did not walk expected vias";
  }
}

TEST(Astar, BiDirTrivial) {
  // Normally the service does not allow a trivial path with bidirectional astar because it has some
  // problems with those (oneways that make you go around the block to get to where you started?).
  // However when reviewing the other special case of short bidirectional routes where the forward and
  // backward search paths meet on the destination edge we found that the trivial path edge trimming
  // was wrong. Specifically because the forward expansion only cares about trimming the first edge
  // from the origin and the reverse expansion only cares about trimming the last edge up to the
  // destination but the route is only one edge. This means that both the reverse path label and the
  // forward path label both have trimmed the edge but not enough. So what we have to do is trim the
  // whole edge based on what percentage of the edge is left between the origin and destination.

  // Get access to tiles
  boost::property_tree::ptree conf;
  conf.put("tile_dir", "test/data/utrecht_tiles");
  conf.put<unsigned long>("mjolnir.id_table_size", 1000);
  vb::GraphReader graph_reader(conf);

  // Locations
  std::vector<valhalla::baldr::Location> locations;
  baldr::Location origin(valhalla::midgard::PointLL(5.12696, 52.09701),
                         baldr::Location::StopType::BREAK);
  locations.push_back(origin);
  baldr::Location dest(valhalla::midgard::PointLL(5.12700, 52.09709),
                       baldr::Location::StopType::BREAK);
  locations.push_back(dest);

  // Costing
  Options options;
  create_costing_options(options, Costing::auto_);
  vs::TravelMode mode;
  auto mode_costing = vs::CostFactory().CreateModeCosting(options, mode);
  auto cost = mode_costing[int(mode)];

  // Loki
  const auto projections = vk::Search(locations, graph_reader, cost);
  std::vector<PathLocation> path_location;
  for (const auto& loc : locations) {
    ASSERT_NO_THROW(
        path_location.push_back(projections.at(loc));
        PathLocation::toPBF(path_location.back(), options.mutable_locations()->Add(), graph_reader);)
        << "fail_invalid_origin";
  }

  vt::BidirectionalAStar astar;
  auto path = astar
                  .GetBestPath(*options.mutable_locations(0), *options.mutable_locations(1),
                               graph_reader, mode_costing, mode)
                  .front();

  ASSERT_TRUE(path.size() == 1);
  EXPECT_LT(path.front().elapsed_cost.cost, 1);
  EXPECT_LT(path.front().elapsed_cost.secs, 1);
}

TEST(BiDiAstar, test_recost_path) {
  const std::string ascii_map = R"(
           X-----------Y
          /             \
    1----A               E---2
          \             /
           B--C--------D
           |  |        |
           3  4        5
  )";
  const gurka::ways ways = {
      // make ABC to be a shortcut
      {"ABC", {{"highway", "motorway"}, {"maxspeed", "80"}}},
      // make CDE to be a shortcut
      {"CDE", {{"highway", "primary"}, {"maxspeed", "80"}}},
      {"1A", {{"highway", "secondary"}}},
      {"B3", {{"highway", "secondary"}}},
      {"C4", {{"highway", "secondary"}}},
      {"D5", {{"highway", "secondary"}}},
      // set speeds less than on ABCDE path to force the algorithm
      // to go through ABCDE nodes instead of AXY
      {"AX", {{"highway", "motorway"}, {"maxspeed", "70"}}},
      {"XY", {{"highway", "trunk"}, {"maxspeed", "70"}}},
      {"YE", {{"highway", "primary"}, {"maxspeed", "80"}}},
      {"E2", {{"highway", "secondary"}}},
  };

  auto nodes = gurka::detail::map_to_coordinates(ascii_map, 500);

  const std::string test_dir = "test/data/astar_shortcuts_recosting";
  const auto map = gurka::buildtiles(nodes, ways, {}, {}, test_dir);

  vb::GraphReader graphreader(map.config.get_child("mjolnir"));

  // before continue check that ABC is actually a shortcut
  const auto ABC = gurka::findEdgeByNodes(graphreader, nodes, "A", "C");
  ASSERT_TRUE(std::get<1>(ABC)->is_shortcut()) << "Expected ABC to be a shortcut";
  // before continue check that CDE is actually a shortcut
  const auto CDE = gurka::findEdgeByNodes(graphreader, nodes, "C", "E");
  ASSERT_TRUE(std::get<1>(CDE)->is_shortcut()) << "Expected CDE to be a shortcut";

  auto const set_constrained_speed = [&graphreader, &test_dir](const std::vector<GraphId>& edge_ids) {
    for (const auto& edgeid : edge_ids) {
      GraphId tileid(edgeid.tileid(), edgeid.level(), 0);
      auto tile = graphreader.GetGraphTile(tileid);
      vj::GraphTileBuilder tile_builder(test_dir, tileid, false);
      std::vector<DirectedEdge> edges;
      for (uint32_t j = 0; j < tile->header()->directededgecount(); ++j) {
        DirectedEdge& edge = tile_builder.directededge(j);
        // update only superseded edges
        if (edgeid.id() == j)
          edge.set_constrained_flow_speed(10);
        edges.emplace_back(std::move(edge));
      }
      tile_builder.UpdatePredictedSpeeds(edges);
    }
  };
  // set constrained speed for all superseded edges;
  // this speed will be used for them in costing model
  set_constrained_speed(graphreader.RecoverShortcut(std::get<0>(ABC)));
  set_constrained_speed(graphreader.RecoverShortcut(std::get<0>(CDE)));
  // reset cache to see updated speeds
  graphreader.Clear();

  Options options;
  create_costing_options(options, Costing::auto_);
  vs::TravelMode travel_mode = vs::TravelMode::kDrive;
  const auto mode_costing = vs::CostFactory().CreateModeCosting(options, travel_mode);

  std::vector<vb::Location> locations;
  // set origin location
  locations.push_back({nodes["1"]});
  // set destination location
  locations.push_back({nodes["2"]});
  auto pbf_locations = ToPBFLocations(locations, graphreader, mode_costing[int(travel_mode)]);

  vt::BidirectionalAStar astar;

  // hack hierarchy limits to allow to go through the shortcut
  {
    auto& hierarchy_limits =
        mode_costing[int(travel_mode)]->GetHierarchyLimits(); // access mutable limits
    for (auto& hierarchy : hierarchy_limits) {
      hierarchy.Relax(0.f, 0.f);
    }
  }
  const auto path =
      astar.GetBestPath(pbf_locations[0], pbf_locations[1], graphreader, mode_costing, travel_mode)
          .front();

  // check that final path doesn't contain shortcuts
  for (const auto& info : path) {
    const auto* edge = graphreader.directededge(info.edgeid);
    ASSERT_FALSE(edge->is_shortcut()) << "Final path shouldn't contain shortcuts";
  }
  // check that final path contains right number of edges
  ASSERT_EQ(path.size(), 6) << "Final path has wrong number of edges";

  // calculate edge duration based on length and speed
  const auto get_edge_duration = [&graphreader, &mode_costing,
                                  travel_mode](const vb::GraphId& edgeid,
                                               const vb::DirectedEdge* edge) {
    auto tile = graphreader.GetGraphTile(edgeid);
    const float speed_meters_per_sec =
        (1000.f / 3600.f) * tile->GetSpeed(edge, mode_costing[int(travel_mode)]->flow_mask());
    return edge->length() / speed_meters_per_sec;
  };

  // Check that final path really was recosted. To do that we compare actual
  // edges durations with durations in the final path.
  const std::vector<std::string> superseded_nodes = {"A", "B", "C", "D", "E"};
  for (size_t i = 0; (i + 1) < superseded_nodes.size(); ++i) {
    const auto edge =
        gurka::findEdgeByNodes(graphreader, nodes, superseded_nodes[i], superseded_nodes[i + 1]);
    ASSERT_EQ(path[i + 1].edgeid, std::get<0>(edge)) << "Not expected edge in the path";
    EXPECT_NEAR((path[i + 1].elapsed_cost - path[i].elapsed_cost - path[i + 1].transition_cost).secs,
                get_edge_duration(std::get<0>(edge), std::get<1>(edge)), 0.1f);
  }
}

// TODO(nils): this test fails currently, because bidir A* has a problem with 2 shortcuts between the
// same nodes: https://github.com/valhalla/valhalla/issues/4609
TEST(BiDiAstar, DISABLED_test_recost_path_failing) {
  const std::string ascii_map = R"(
           X-----------Y
          /             \
    1----A               E---2
          \             /
           B--C--------D
  )";
  const gurka::ways ways = {
      // make ABCDE to be a shortcut
      {"ABC", {{"highway", "primary"}, {"maxspeed", "80"}}},
      {"CDE", {{"highway", "primary"}, {"maxspeed", "80"}}},
      {"1A", {{"highway", "secondary"}}},
      // set speeds less than on ABCDE path to force the algorithm
      // to go through ABCDE nodes instead of AXY
      {"AX", {{"highway", "primary"}, {"maxspeed", "70"}}},
      {"XY", {{"highway", "primary"}, {"maxspeed", "70"}}},
      {"YE", {{"highway", "primary"}, {"maxspeed", "80"}}},
      {"E2", {{"highway", "secondary"}}},
  };

  auto nodes = gurka::detail::map_to_coordinates(ascii_map, 500);

  const std::string test_dir = "test/data/astar_shortcuts_recosting";
  const auto map = gurka::buildtiles(nodes, ways, {}, {}, test_dir);

  vb::GraphReader graphreader(map.config.get_child("mjolnir"));

  // before continue check that ABC is actually a shortcut
  const auto ABCDE = gurka::findEdgeByNodes(graphreader, nodes, "A", "E");
  ASSERT_TRUE(std::get<1>(ABCDE)->is_shortcut()) << "Expected ABCDE to be a shortcut";

  Options options;
  create_costing_options(options, Costing::auto_);
  vs::TravelMode travel_mode = vs::TravelMode::kDrive;
  const auto mode_costing = vs::CostFactory().CreateModeCosting(options, travel_mode);

  std::vector<vb::Location> locations;
  // set origin location
  locations.push_back({nodes["1"]});
  // set destination location
  locations.push_back({nodes["2"]});
  auto pbf_locations = ToPBFLocations(locations, graphreader, mode_costing[int(travel_mode)]);

  vt::BidirectionalAStar astar;

  // hack hierarchy limits to allow to go through the shortcut
  {
    auto& hierarchy_limits =
        mode_costing[int(travel_mode)]->GetHierarchyLimits(); // access mutable limits
    for (auto& hierarchy : hierarchy_limits) {
      hierarchy.Relax(0.f, 0.f);
    }
  }
  const auto path =
      astar.GetBestPath(pbf_locations[0], pbf_locations[1], graphreader, mode_costing, travel_mode)
          .front();

  // collect names of base edges
  std::vector<std::string> expected_names = {"1A", "AB", "BC", "CD", "DE", "E2"};
  std::vector<std::string> actual_names;
  for (const auto& info : path) {
    const auto* edge = graphreader.directededge(info.edgeid);
    ASSERT_FALSE(edge->is_shortcut()) << "Final path shouldn't contain shortcuts";

    const auto name = graphreader.edgeinfo(info.edgeid).GetNames()[0];
    actual_names.emplace_back(name);
  }
  // TODO(nils): it gets the wrong path! bidir A* has a problem with 2 shortcuts between the same
  // nodes
  EXPECT_EQ(actual_names, expected_names);
}

class BiAstarTest : public thor::BidirectionalAStar {
public:
  explicit BiAstarTest(const boost::property_tree::ptree& config = {}) : BidirectionalAStar(config) {
  }

  void Clear() {
    BidirectionalAStar::Clear();
    if (clear_reserved_memory_) {
      EXPECT_EQ(edgelabels_forward_.capacity(), 0);
      EXPECT_EQ(edgelabels_reverse_.capacity(), 0);
    } else {
      EXPECT_LE(edgelabels_forward_.capacity(), max_reserved_labels_count_);
      EXPECT_LE(edgelabels_reverse_.capacity(), max_reserved_labels_count_);
    }
  }
};

TEST(BiDiAstar, test_clear_reserved_memory) {
  boost::property_tree::ptree config;
  config.put("clear_reserved_memory", true);

  BiAstarTest astar(config);
  astar.Clear();
}

TEST(BiDiAstar, test_max_reserved_labels_count) {
  boost::property_tree::ptree config;
  config.put("max_reserved_labels_count_bidir_astar", 10);

  BiAstarTest astar(config);
  astar.Clear();
}

class AstarTestEnv : public ::testing::Environment {
public:
  void SetUp() override {
    make_tile();
  }
};

} // anonymous namespace

int main(int argc, char* argv[]) {
  testing::AddGlobalTestEnvironment(new AstarTestEnv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
