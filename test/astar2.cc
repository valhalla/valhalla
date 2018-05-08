#include <cstdint>
#include "test.h"

#include "loki/search.h"
#include "mjolnir/pbfgraphparser.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/graphenhancer.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/graphvalidator.h"
#include "odin/directionsbuilder.h"
#include "baldr/graphreader.h"
#include "baldr/location.h"
#include "sif/pedestriancost.h"
#include "sif/dynamiccost.h"
#include "thor/astar.h"
#include "thor/trippathbuilder.h"
#include "thor/attributes_controller.h"

#include "proto/trippath.pb.h"
#include "proto/directions_options.pb.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>

namespace bpt = boost::property_tree;

namespace vb = valhalla::baldr;
namespace vs = valhalla::sif;
namespace vt = valhalla::thor;
namespace vk = valhalla::loki;
namespace vj = valhalla::mjolnir;
namespace vo = valhalla::odin;

namespace {

void trivial_path_no_uturns() {
  bpt::ptree conf;
  std::stringstream json;
  json << "{ \
  \"mjolnir\": { \
  \"concurrency\": 1, \
  \"tile_dir\": \"test/data/trivial_tiles\", \
  \"admin\": \"test/data/netherlands_admin.sqlite\", \
  \"timezone\": \"test/data/not_needed.sqlite\" \
  } \
  }";
  bpt::json_parser::read_json(json, conf);

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
  std::vector<vb::Location> locations;
  locations.push_back(vb::Location::FromCsv("52.09595728238367,5.114587247480813,break"));
  locations.push_back(vb::Location::FromCsv("52.096141834552945,5.114506781210365,break"));

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
  trivial_path_no_uturns();
}

} // anonymous namespace

int main() {
  test::suite suite("astar2");
  suite.test(TEST_CASE(TestTrivialPathNoUturns));
  return suite.tear_down();
}
