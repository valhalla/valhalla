#include "test.h"
#include "mjolnir/osmnode.h"
#include "mjolnir/pbfgraphparser.h"
#include <valhalla/midgard/sequence.h>

#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/directededge.h>

using namespace std;
using namespace valhalla::mjolnir;
using namespace valhalla::baldr;

namespace {

void write_config(const std::string& filename) {
  std::ofstream file;
  try {
    file.open(filename, std::ios_base::trunc);
    file << "{ \
      \"mjolnir\": { \
       \"tile_dir\": \"test/tiles\" \
      } \
    }";
  }
  catch(...) {

  }
  file.close();
}

const auto node_predicate = [](const OSMWayNode& a, const OSMWayNode& b) {
  return a.node.osmid < b.node.osmid;
};

OSMNode GetNode(uint64_t node_id, sequence<OSMWayNode>& way_nodes) {
  OSMWayNode target{{node_id}};
  if(!way_nodes.find(target, node_predicate))
    throw std::runtime_error("Couldn't find node: " + std::to_string(node_id));
  return target.node;
}

auto way_predicate = [](const OSMWay& a, const OSMWay& b){
  return a.osmwayid_ < b.osmwayid_;
};

OSMWay GetWay(uint64_t way_id, sequence<OSMWay>& ways) {
  OSMWay target{way_id};
  if(!ways.find(target, way_predicate))
    throw std::runtime_error("Couldn't find way: " + std::to_string(way_id));
  return target;
}


void Bike(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  auto osmdata = PBFGraphParser::Parse(conf.get_child("mjolnir"), {"test/data/utrecht_netherlands.osm.pbf"}, ways_file, way_nodes_file);
  sequence<OSMWay> ways(ways_file, false);
  ways.sort(way_predicate);

  auto way = GetWay(127361688, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != true || way.bike_backward() != true || way.bus_backward() != true) {
    throw std::runtime_error("Access is not correct for way 127361688.");
  }

  way = GetWay(7062008, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != false || way.bike_backward() != true || way.bus_backward() != false) {
    throw std::runtime_error("Access is not correct for way 7062008.");
  }

  way = GetWay(48672084, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != false || way.bike_backward() != true || way.bus_backward() != false) {
    throw std::runtime_error("Access is not correct for way 48672084.");
  }

  way = GetWay(7053107, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != false || way.bike_backward() != true || way.bus_backward() != false) {
    throw std::runtime_error("Access is not correct for way 7053107.");
  }

  way = GetWay(7053048, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != false || way.bike_backward() != true || way.bus_backward() != false) {
    throw std::runtime_error("Access is not correct for way 7053048.");
  }

  way = GetWay(221051138, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != false || way.bike_backward() != false || way.bus_backward() != false) {
    throw std::runtime_error("Access is not correct for way 221051138.");
  }

  way = GetWay(23544607, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != true || way.bike_backward() != true || way.bus_backward() != true) {
    throw std::runtime_error("Access is not correct for way 23544607.");
  }

  way = GetWay(221051142, ways);
/*
  std::cout << "way.auto_f() " << std::to_string(way.auto_forward()) << endl;
  std::cout << "way.auto_b() " << std::to_string(way.auto_backward()) << endl;
  std::cout << "way.bike_f() " << std::to_string(way.bike_forward()) << endl;
  std::cout << "way.bike_b() " << std::to_string(way.bike_backward()) << endl;
  std::cout << "way.bus_f() " << std::to_string(way.bus_forward()) << endl;
  std::cout << "way.bus_b() " << std::to_string(way.bus_backward()) << endl;
  std::cout << "way.ped() " << std::to_string(way.pedestrian()) << endl;
*/
  if (way.auto_forward() != false || way.bus_forward() != false || way.bike_forward() != true || way.pedestrian() != false ||
      way.auto_backward() != false || way.bike_backward() != false || way.bus_backward() != false) {
    throw std::runtime_error("Access is not correct for way 221051142.");
  }

  way = GetWay(72906238, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != false || way.bike_backward() != true || way.bus_backward() != false) {
    throw std::runtime_error("Access is not correct for way 72906238.");
  }

  way = GetWay(7010549, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != false || way.bike_backward() != true || way.bus_backward() != false) {
    throw std::runtime_error("Access is not correct for way 7010549.");
  }

  way = GetWay(7007629, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != false || way.bike_backward() != true || way.bus_backward() != false) {
    throw std::runtime_error("Access is not correct for way 7007629.");
  }
}

void Bus(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  auto osmdata = PBFGraphParser::Parse(conf.get_child("mjolnir"), {"test/data/utrecht_netherlands.osm.pbf"}, ways_file, way_nodes_file);
  sequence<OSMWay> ways(ways_file, false);
  ways.sort(way_predicate);

  auto way = GetWay(33648196, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != false || way.bike_backward() != true || way.bus_backward() != true) {
    throw std::runtime_error("Access is not correct for way 33648196.");
  }
}

void TestBike() {
  //write the tiles with it
  Bike("test/test_config");
}

void TestBus() {
  //write the tiles with it
  Bus("test/test_config");
}

void DoConfig() {
  //make a config file
  write_config("test/test_config");
}

}

int main() {

  test::suite suite("utrecht");

  suite.test(TEST_CASE(DoConfig));
  suite.test(TEST_CASE(TestBike));
  suite.test(TEST_CASE(TestBus));

  return suite.tear_down();
}
