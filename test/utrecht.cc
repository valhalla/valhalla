#include "midgard/sequence.h"
#include "mjolnir/osmnode.h"
#include "mjolnir/pbfgraphparser.h"
#include "test.h"
#include <cstdint>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fstream>

#include "baldr/directededge.h"
#include "baldr/graphconstants.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace std;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;
using namespace valhalla::baldr;

namespace {

const std::string config_file = "test/test_config_ut";
std::string ways_file = "test_ways_utrecht.bin";
std::string way_nodes_file = "test_way_nodes_utrecht.bin";
std::string access_file = "test_access_utrecht.bin";
std::string from_restriction_file = "test_from_complex_restrictions_utrecht.bin";
std::string to_restriction_file = "test_to_complex_restrictions_utrecht.bin";
std::string bss_file = "test_bss_nodes_utrecht.bin";

const auto node_predicate = [](const OSMWayNode& a, const OSMWayNode& b) {
  return a.node.osmid_ < b.node.osmid_;
};

OSMNode GetNode(uint64_t node_id, sequence<OSMWayNode>& way_nodes) {
  auto found = way_nodes.find({node_id}, node_predicate);
  if (found == way_nodes.end())
    throw std::runtime_error("Couldn't find node: " + std::to_string(node_id));
  return (*found).node;
}

auto way_predicate = [](const OSMWay& a, const OSMWay& b) { return a.osmwayid_ < b.osmwayid_; };

OSMWay GetWay(uint32_t way_id, sequence<OSMWay>& ways) {
  auto found = ways.find({way_id}, way_predicate);
  if (found == ways.end())
    throw std::runtime_error("Couldn't find way: " + std::to_string(way_id));
  return *found;
}

void Parse() {
  boost::property_tree::ptree conf;
  conf.put<std::string>("mjolnir.tile_dir", "test/data/parser_tiles");
  auto osmdata = PBFGraphParser::Parse(conf.get_child("mjolnir"),
                                       {VALHALLA_SOURCE_DIR "test/data/utrecht_netherlands.osm.pbf"},
                                       ways_file, way_nodes_file, access_file, from_restriction_file,
                                       to_restriction_file, bss_file);
}

void TestBike() {
  boost::property_tree::ptree conf;
  conf.put<std::string>("mjolnir.tile_dir", "test/data/parser_tiles");
  sequence<OSMWay> ways(ways_file, false);
  ways.sort(way_predicate);

  auto way = GetWay(127361688, ways);
  if (way.auto_forward() != true || way.moped_forward() != true || way.bus_forward() != true ||
      way.bike_forward() != true || way.pedestrian() != true || way.auto_backward() != true ||
      way.moped_backward() != true || way.bus_backward() != true || way.bike_backward() != true) {
    throw std::runtime_error("Access is not correct for way 127361688.");
  }

  way = GetWay(7062008, ways);
  if (way.auto_forward() != true || way.moped_forward() != true || way.bus_forward() != true ||
      way.bike_forward() != true || way.pedestrian() != true || way.auto_backward() != false ||
      way.moped_backward() != false || way.bus_backward() != false || way.bike_backward() != true) {
    throw std::runtime_error("Access is not correct for way 7062008.");
  }

  way = GetWay(48672084, ways);
  if (way.auto_forward() != true || way.moped_forward() != true || way.bus_forward() != true ||
      way.bike_forward() != true || way.pedestrian() != true || way.auto_backward() != false ||
      way.moped_backward() != false || way.bus_backward() != false || way.bike_backward() != true) {
    throw std::runtime_error("Access is not correct for way 48672084.");
  }

  way = GetWay(7053107, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.bike_forward() != true ||
      way.pedestrian() != true || way.auto_backward() != false || way.bus_backward() != false ||
      way.bike_backward() != true) {
    throw std::runtime_error("Access is not correct for way 7053107.");
  }

  way = GetWay(7053048, ways);
  if (way.auto_forward() != true || way.moped_forward() != true || way.bus_forward() != true ||
      way.bike_forward() != true || way.pedestrian() != true || way.auto_backward() != false ||
      way.moped_backward() != false || way.bike_backward() != true || way.bus_backward() != false) {
    throw std::runtime_error("Access is not correct for way 7053048.");
  }

  way = GetWay(221051138, ways);
  if (way.auto_forward() != true || way.moped_forward() != true || way.bus_forward() != true ||
      way.bike_forward() != true || way.pedestrian() != true || way.auto_backward() != false ||
      way.moped_backward() != false || way.bus_backward() != false || way.bike_backward() != false) {
    throw std::runtime_error("Access is not correct for way 221051138.");
  }

  way = GetWay(23544607, ways);
  if (way.auto_forward() != true || way.moped_forward() != true || way.bus_forward() != true ||
      way.bike_forward() != true || way.pedestrian() != true || way.auto_backward() != true ||
      way.moped_backward() != true || way.bus_backward() != true || way.bike_backward() != true) {
    throw std::runtime_error("Access is not correct for way 23544607.");
  }

  way = GetWay(221051142, ways);
  if (way.auto_forward() != false || way.moped_forward() != true || way.bus_forward() != false ||
      way.bike_forward() != true || way.pedestrian() != false || way.auto_backward() != false ||
      way.moped_backward() != false || way.bus_backward() != false || way.bike_backward() != false) {
    throw std::runtime_error("Access is not correct for way 221051142.");
  }

  way = GetWay(72906238, ways);
  if (way.auto_forward() != true || way.moped_forward() != true || way.bus_forward() != true ||
      way.bike_forward() != true || way.pedestrian() != true || way.auto_backward() != false ||
      way.moped_backward() != false || way.bus_backward() != false || way.bike_backward() != true) {
    throw std::runtime_error("Access is not correct for way 72906238.");
  }

  way = GetWay(7010549, ways);
  if (way.auto_forward() != true || way.moped_forward() != true || way.bus_forward() != true ||
      way.bike_forward() != true || way.pedestrian() != true || way.auto_backward() != false ||
      way.moped_backward() != false || way.bus_backward() != false || way.bike_backward() != true) {
    throw std::runtime_error("Access is not correct for way 7010549.");
  }

  way = GetWay(7007629, ways);
  if (way.auto_forward() != true || way.moped_forward() != true || way.bus_forward() != true ||
      way.bike_forward() != true || way.pedestrian() != true || way.auto_backward() != false ||
      way.moped_backward() != false || way.bus_backward() != false || way.bike_backward() != true) {
    throw std::runtime_error("Access is not correct for way 7007629.");
  }
}

void TestBus() {
  boost::property_tree::ptree conf;
  conf.put<std::string>("mjolnir.tile_dir", "test/data/parser_tiles");
  sequence<OSMWay> ways(ways_file, false);
  ways.sort(way_predicate);

  auto way = GetWay(33648196, ways);
  if (way.auto_forward() != true || way.moped_forward() != true || way.bus_forward() != true ||
      way.bike_forward() != true || way.pedestrian() != true || way.auto_backward() != false ||
      way.moped_backward() != true || way.bus_backward() != true || way.bike_backward() != true) {
    throw std::runtime_error("Access is not correct for way 33648196.");
  }
}

void TearDown() {
  boost::filesystem::remove(ways_file);
  boost::filesystem::remove(way_nodes_file);
  boost::filesystem::remove(access_file);
  boost::filesystem::remove(from_restriction_file);
  boost::filesystem::remove(to_restriction_file);
}

} // namespace

int main() {

  test::suite suite("utrecht");

  suite.test(TEST_CASE(Parse));
  suite.test(TEST_CASE(TestBike));
  suite.test(TEST_CASE(TestBus));
  suite.test(TEST_CASE(TearDown));

  return suite.tear_down();
}
