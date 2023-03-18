#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>
#include "mjolnir/util.h"
#include "mjolnir/osmnode.h"
#include "midgard/pointll.h"
#include "midgard/sequence.h"
#include "mjolnir/pbfgraphparser.h"
#include "filesystem.h"
#include "test.h"

#include <fstream>
#include <iostream>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;

class DisableHierarchyPruningTest : public ::testing::Test {
protected:
  static std::string pbf_filename;
  static std::string workdir;
  static gurka::map shortest_map;
  static std::string source_id;
  static std::string target_id;

  // Set up the map for the test suite. (map of Utrecht Netherlands, utrecht_netherlands.osm.pbf)
  static void SetUpTestSuite() {
    auto config = test::make_config(workdir);
    if (!filesystem::exists(workdir + pbf_filename)) {
      throw std::runtime_error("The PBF file does not exist: " + workdir + pbf_filename);
    }

    midgard::logging::Configure({{"type", ""}});

    mjolnir::build_tile_set(config, {workdir + pbf_filename}, mjolnir::BuildStage::kInitialize,
                          mjolnir::BuildStage::kValidate, false);
    std::string ways_file = "test_ways.bin";
    std::string way_nodes_file = "test_way_nodes.bin";
    std::string access_file = "test_access.bin";
    std::string pronunciation_file = "test_pronunciation.bin";
    std::string bss_nodes_file = "test_bss_nodes.bin";

    auto osmdata =  PBFGraphParser::ParseWays(config.get_child("mjolnir"),
                                  {VALHALLA_SOURCE_DIR "test/data/disable_hierarchy_pruning/map.pbf"},
                                  ways_file, way_nodes_file, access_file, pronunciation_file);
    
    // Parse the nodes and index them by osm_id
    PBFGraphParser::ParseNodes(config.get_child("mjolnir"),
                              {VALHALLA_SOURCE_DIR "test/data/disable_hierarchy_pruning/map.pbf"},
                              way_nodes_file, bss_nodes_file, osmdata);
    sequence<OSMWayNode> way_nodes(way_nodes_file, false);
    shortest_map.config = config;
    for (auto way_node : way_nodes) {
      shortest_map.nodes[std::to_string(way_node.node.osmid_)] = way_node.node.latlng();
    }
  }

  // Compute length of a result route. 
  inline float getLength(const valhalla::Api& route) {
   return route.directions().routes(0).legs(0).summary().length();
  }

  // Do the route action and get a pair of results, 
  // one with disable_hierarchy_pruning = false, and the other = true.
  void RunAction(const std::string& costing, 
                valhalla::Api* default_route, valhalla::Api* new_route) {
    std::unordered_map<std::string, std::string> options_disable = {
      {"/costing_options/" + costing + "/disable_hierarchy_pruning", "1"},
      {"/costing_options/" + costing + "/shortest", "1"}
    };

    std::unordered_map<std::string, std::string> options = {
      {"/costing_options/" + costing + "/shortest", "1"}
    };

    *default_route =
      gurka::do_action(valhalla::Options::route, shortest_map, {source_id, target_id}, costing, options);
    *new_route =
      gurka::do_action(valhalla::Options::route, shortest_map, {source_id, target_id}, costing, options_disable);
  }

  // Compare the length of two results.
  // The one with disable_hierarchy_pruning = true should be shorter.
  void CheckLength(const valhalla::Api default_route, const valhalla::Api new_route) {
    auto default_length = getLength(default_route);
    auto new_length = getLength(new_route);
    EXPECT_LT(new_length, default_length) << "new length is not smaller than default length";
  }

  // Check warning code when input distances exceed the max limit.
  void CheckWarning(const valhalla::Api new_route) {
    EXPECT_EQ(new_route.info().warnings_size(), 1);
    EXPECT_EQ(new_route.info().warnings(0).code(), 205);
  }
};

gurka::map DisableHierarchyPruningTest::shortest_map = {};
std::string DisableHierarchyPruningTest::pbf_filename = "/map.pbf";
std::string DisableHierarchyPruningTest::workdir = "test/data/disable_hierarchy_pruning";
// Source and target locations are specially chosen to show the difference. 
std::string DisableHierarchyPruningTest::source_id = "45012320";
std::string DisableHierarchyPruningTest::target_id = "45429700";
valhalla::Api default_route = {};
valhalla::Api new_route = {};

// Test disable_hierarchy_pruning funtioning correctly in all kinds of costing types.
TEST_F(DisableHierarchyPruningTest, Bus) {
  shortest_map.config.put("service_limits.max_distance_disable_hierarchy_culling", "1000000");
  RunAction("bus", &default_route, &new_route);
  CheckLength(default_route, new_route);
}

TEST_F(DisableHierarchyPruningTest, MotorScooter) {
  shortest_map.config.put("service_limits.max_distance_disable_hierarchy_culling", "1000000");
  RunAction("motor_scooter", &default_route, &new_route);
  CheckLength(default_route, new_route);
}

TEST_F(DisableHierarchyPruningTest, Truck) {
  shortest_map.config.put("service_limits.max_distance_disable_hierarchy_culling", "1000000");
  RunAction("truck", &default_route, &new_route);
  CheckLength(default_route, new_route);
}

TEST_F(DisableHierarchyPruningTest, Motorcycle) {
  shortest_map.config.put("service_limits.max_distance_disable_hierarchy_culling", "1000000");
  RunAction("motorcycle", &default_route, &new_route);
  CheckLength(default_route, new_route);
}

TEST_F(DisableHierarchyPruningTest, Taxi) {
  shortest_map.config.put("service_limits.max_distance_disable_hierarchy_culling", "1000000");
  RunAction("taxi", &default_route, &new_route);
  CheckLength(default_route, new_route);
}

TEST_F(DisableHierarchyPruningTest, Auto) {
  shortest_map.config.put("service_limits.max_distance_disable_hierarchy_culling", "1000000");
  RunAction("auto", &default_route, &new_route);
  CheckLength(default_route, new_route);
}

// Test when the max limit of distance is exceeded, 
// the warning code is correct and two route lengths are equal.
TEST_F(DisableHierarchyPruningTest, AutoExceedMaxDistance) {
  shortest_map.config.put("service_limits.max_distance_disable_hierarchy_culling", "1");
  RunAction("auto", &default_route, &new_route);

  auto default_length = getLength(default_route);
  auto new_length = getLength(new_route);
  EXPECT_EQ(new_length, default_length);

  CheckWarning(new_route);
}
