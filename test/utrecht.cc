#include "filesystem.h"
#include "midgard/sequence.h"
#include "mjolnir/osmnode.h"
#include "mjolnir/pbfgraphparser.h"
#include <cstdint>

#include <boost/property_tree/ptree.hpp>
#include <fstream>

#include "baldr/directededge.h"
#include "baldr/graphconstants.h"

#include "test.h"

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

TEST(Utrecth, TestBike) {
  boost::property_tree::ptree conf;
  conf.put<std::string>("mjolnir.tile_dir", "test/data/parser_tiles");
  sequence<OSMWay> ways(ways_file, false);
  ways.sort(way_predicate);

  auto way_127361688 = GetWay(127361688, ways);
  EXPECT_TRUE(way_127361688.auto_forward());
  EXPECT_TRUE(way_127361688.moped_forward());
  EXPECT_TRUE(way_127361688.bus_forward());
  EXPECT_TRUE(way_127361688.bike_forward());
  EXPECT_TRUE(way_127361688.pedestrian());
  EXPECT_TRUE(way_127361688.auto_backward());
  EXPECT_TRUE(way_127361688.moped_backward());
  EXPECT_TRUE(way_127361688.bus_backward());
  EXPECT_TRUE(way_127361688.bike_backward());

  auto way_7062008 = GetWay(7062008, ways);
  EXPECT_TRUE(way_7062008.auto_forward());
  EXPECT_TRUE(way_7062008.moped_forward());
  EXPECT_TRUE(way_7062008.bus_forward());
  EXPECT_TRUE(way_7062008.bike_forward());
  EXPECT_TRUE(way_7062008.pedestrian());
  EXPECT_FALSE(way_7062008.auto_backward());
  EXPECT_FALSE(way_7062008.moped_backward());
  EXPECT_FALSE(way_7062008.bus_backward());
  EXPECT_TRUE(way_7062008.bike_backward());

  auto way_48672084 = GetWay(48672084, ways);
  EXPECT_TRUE(way_48672084.auto_forward());
  EXPECT_TRUE(way_48672084.moped_forward());
  EXPECT_TRUE(way_48672084.bus_forward());
  EXPECT_TRUE(way_48672084.bike_forward());
  EXPECT_TRUE(way_48672084.pedestrian());
  EXPECT_FALSE(way_48672084.auto_backward());
  EXPECT_FALSE(way_48672084.moped_backward());
  EXPECT_FALSE(way_48672084.bus_backward());
  EXPECT_TRUE(way_48672084.bike_backward());

  auto way_7053107 = GetWay(7053107, ways);
  EXPECT_TRUE(way_7053107.auto_forward());
  EXPECT_TRUE(way_7053107.bus_forward());
  EXPECT_TRUE(way_7053107.bike_forward());
  EXPECT_TRUE(way_7053107.pedestrian());
  EXPECT_FALSE(way_7053107.auto_backward());
  EXPECT_FALSE(way_7053107.bus_backward());
  EXPECT_TRUE(way_7053107.bike_backward());

  auto way_7053048 = GetWay(7053048, ways);
  EXPECT_TRUE(way_7053048.auto_forward());
  EXPECT_TRUE(way_7053048.moped_forward());
  EXPECT_TRUE(way_7053048.bus_forward());
  EXPECT_TRUE(way_7053048.bike_forward());
  EXPECT_TRUE(way_7053048.pedestrian());
  EXPECT_FALSE(way_7053048.auto_backward());
  EXPECT_FALSE(way_7053048.moped_backward());
  EXPECT_TRUE(way_7053048.bike_backward());
  EXPECT_FALSE(way_7053048.bus_backward());

  auto way_221051138 = GetWay(221051138, ways);
  EXPECT_TRUE(way_221051138.auto_forward());
  EXPECT_TRUE(way_221051138.moped_forward());
  EXPECT_TRUE(way_221051138.bus_forward());
  EXPECT_TRUE(way_221051138.bike_forward());
  EXPECT_TRUE(way_221051138.pedestrian());
  EXPECT_FALSE(way_221051138.auto_backward());
  EXPECT_FALSE(way_221051138.moped_backward());
  EXPECT_FALSE(way_221051138.bus_backward());
  EXPECT_FALSE(way_221051138.bike_backward());

  auto way_23544607 = GetWay(23544607, ways);
  EXPECT_TRUE(way_23544607.auto_forward());
  EXPECT_TRUE(way_23544607.moped_forward());
  EXPECT_TRUE(way_23544607.bus_forward());
  EXPECT_TRUE(way_23544607.bike_forward());
  EXPECT_TRUE(way_23544607.pedestrian());
  EXPECT_TRUE(way_23544607.auto_backward());
  EXPECT_TRUE(way_23544607.moped_backward());
  EXPECT_TRUE(way_23544607.bus_backward());
  EXPECT_TRUE(way_23544607.bike_backward());

  auto way_221051142 = GetWay(221051142, ways);
  EXPECT_FALSE(way_221051142.auto_forward());
  EXPECT_TRUE(way_221051142.moped_forward());
  EXPECT_FALSE(way_221051142.bus_forward());
  EXPECT_TRUE(way_221051142.bike_forward());
  EXPECT_FALSE(way_221051142.pedestrian());
  EXPECT_FALSE(way_221051142.auto_backward());
  EXPECT_FALSE(way_221051142.moped_backward());
  EXPECT_FALSE(way_221051142.bus_backward());
  EXPECT_FALSE(way_221051142.bike_backward());

  auto way_72906238 = GetWay(72906238, ways);
  EXPECT_TRUE(way_72906238.auto_forward());
  EXPECT_TRUE(way_72906238.moped_forward());
  EXPECT_TRUE(way_72906238.bus_forward());
  EXPECT_TRUE(way_72906238.bike_forward());
  EXPECT_TRUE(way_72906238.pedestrian());
  EXPECT_FALSE(way_72906238.auto_backward());
  EXPECT_FALSE(way_72906238.moped_backward());
  EXPECT_FALSE(way_72906238.bus_backward());
  EXPECT_TRUE(way_72906238.bike_backward());

  auto way_7010549 = GetWay(7010549, ways);
  EXPECT_TRUE(way_7010549.auto_forward());
  EXPECT_TRUE(way_7010549.moped_forward());
  EXPECT_TRUE(way_7010549.bus_forward());
  EXPECT_TRUE(way_7010549.bike_forward());
  EXPECT_TRUE(way_7010549.pedestrian());
  EXPECT_FALSE(way_7010549.auto_backward());
  EXPECT_FALSE(way_7010549.moped_backward());
  EXPECT_FALSE(way_7010549.bus_backward());
  EXPECT_TRUE(way_7010549.bike_backward());

  auto way_7007629 = GetWay(7007629, ways);
  EXPECT_TRUE(way_7007629.auto_forward());
  EXPECT_TRUE(way_7007629.moped_forward());
  EXPECT_TRUE(way_7007629.bus_forward());
  EXPECT_TRUE(way_7007629.bike_forward());
  EXPECT_TRUE(way_7007629.pedestrian());
  EXPECT_FALSE(way_7007629.auto_backward());
  EXPECT_FALSE(way_7007629.moped_backward());
  EXPECT_FALSE(way_7007629.bus_backward());
  EXPECT_TRUE(way_7007629.bike_backward());
}

TEST(Utrecht, TestBus) {
  boost::property_tree::ptree conf;
  conf.put<std::string>("mjolnir.tile_dir", "test/data/parser_tiles");
  sequence<OSMWay> ways(ways_file, false);
  ways.sort(way_predicate);

  auto way_33648196 = GetWay(33648196, ways);
  EXPECT_TRUE(way_33648196.auto_forward());
  EXPECT_TRUE(way_33648196.moped_forward());
  EXPECT_TRUE(way_33648196.bus_forward());
  EXPECT_TRUE(way_33648196.bike_forward());
  EXPECT_TRUE(way_33648196.pedestrian());
  EXPECT_FALSE(way_33648196.auto_backward());
  EXPECT_TRUE(way_33648196.moped_backward());
  EXPECT_TRUE(way_33648196.bus_backward());
  EXPECT_TRUE(way_33648196.bike_backward());
}

// Setup and tearown will be called only once for the entire suite
class UtrecthTestSuiteEnv : public ::testing::Environment {
public:
  void SetUp() override {
    boost::property_tree::ptree conf;
    conf.put<std::string>("mjolnir.tile_dir", "test/data/parser_tiles");
    auto osmdata =
        PBFGraphParser::Parse(conf.get_child("mjolnir"),
                              {VALHALLA_SOURCE_DIR "test/data/utrecht_netherlands.osm.pbf"},
                              ways_file, way_nodes_file, access_file, from_restriction_file,
                              to_restriction_file, bss_file);
  }

  void TearDown() override {
    filesystem::remove(ways_file);
    filesystem::remove(way_nodes_file);
    filesystem::remove(access_file);
    filesystem::remove(from_restriction_file);
    filesystem::remove(to_restriction_file);
  }
};

} // namespace

int main(int argc, char* argv[]) {
  testing::AddGlobalTestEnvironment(new UtrecthTestSuiteEnv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
