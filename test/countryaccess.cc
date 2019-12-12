#include "midgard/sequence.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/graphenhancer.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/osmnode.h"
#include "mjolnir/pbfgraphparser.h"
#include "test.h"
#include <cstdint>

#include "baldr/rapidjson_utils.h"
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fstream>

#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace std;
using namespace valhalla::mjolnir;
using namespace valhalla::baldr;

namespace {

const std::string config_file = "test/test_config_country";

// Remove a temporary file if it exists
void remove_temp_file(const std::string& fname) {
  if (boost::filesystem::exists(fname)) {
    boost::filesystem::remove(fname);
  }
}

void write_config(const std::string& filename) {
  std::ofstream file;
  try {
    file.open(filename, std::ios_base::trunc);
    file << "{ \
      \"mjolnir\": { \
      \"concurrency\": 1, \
       \"tile_dir\": \"test/data/amsterdam_tiles\", \
        \"admin\": \"" VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite\", \
         \"timezone\": \"" VALHALLA_SOURCE_DIR "test/data/not_needed.sqlite\" \
      } \
    }";
  } catch (...) {}
  file.close();
}

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

void CountryAccess(const std::string& config_file) {
  boost::property_tree::ptree conf;
  rapidjson::read_json(config_file, conf);

  // setup and purge
  GraphReader graph_reader(conf.get_child("mjolnir"));
  for (const auto& level : TileHierarchy::levels()) {
    auto level_dir = graph_reader.tile_dir() + "/" + std::to_string(level.first);
    if (boost::filesystem::exists(level_dir) && !boost::filesystem::is_empty(level_dir)) {
      boost::filesystem::remove_all(level_dir);
    }
  }

  // Set up the temporary (*.bin) files used during processing
  std::string ways_file = "test_ways_amsterdam.bin";
  std::string way_nodes_file = "test_way_nodes_amsterdam.bin";
  std::string nodes_file = "test_nodes_amsterdam.bin";
  std::string edges_file = "test_edges_amsterdam.bin";
  std::string access_file = "test_access_amsterdam.bin";
  std::string cr_from_file = "test_from_cr_amsterdam.bin";
  std::string cr_to_file = "test_to_cr_amsterdam.bin";
  std::string bss_nodes_file = "test_bss_nodes_amsterdam.bin";
  // Parse Amsterdam OSM data
  auto osmdata =
      PBFGraphParser::Parse(conf.get_child("mjolnir"),
                            {VALHALLA_SOURCE_DIR "test/data/amsterdam.osm.pbf"}, ways_file,
                            way_nodes_file, access_file, cr_from_file, cr_to_file, bss_nodes_file);

  // Build the graph using the OSMNodes and OSMWays from the parser
  GraphBuilder::Build(conf, osmdata, ways_file, way_nodes_file, nodes_file, edges_file, cr_from_file,
                      cr_to_file);

  // load a tile and test the default access.
  GraphId id(820099, 2, 0);
  GraphTile t("test/data/amsterdam_tiles", id);

  GraphTileBuilder tilebuilder(graph_reader.tile_dir(), id, true);

  for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); i++) {
    NodeInfo& nodeinfo = tilebuilder.node_builder(i);
    uint32_t count = nodeinfo.edge_count();
    for (uint32_t j = 0; j < count; j++) {
      DirectedEdge& directededge = tilebuilder.directededge_builder(nodeinfo.edge_index() + j);

      auto e_offset = tilebuilder.edgeinfo(directededge.edgeinfo_offset());

      uint32_t forward = directededge.forwardaccess();
      uint32_t reverse = directededge.reverseaccess();

      // cycleway (not oneway) should have kBicycleAccess and kMopedAccess
      if (e_offset.wayid() == 7047088) {
        if (forward != (kBicycleAccess | kMopedAccess) ||
            reverse != (kBicycleAccess | kMopedAccess)) {
          throw std::runtime_error("Defaults:  Access is not correct for way 7047088.");
        }
        // cycleway (is oneway) should have kPedestrianAccess and kBicycleAccess
      } else if (e_offset.wayid() == 35600238) {
        if (directededge.forward()) {
          if (forward != kBicycleAccess)
            throw std::runtime_error("Defaults:  Forward access is not correct for way 35600238.");
          if (reverse != 0) // no access
            throw std::runtime_error("Defaults:  Reverse access is not correct for way 35600238.");
        } else {
          if (reverse != kBicycleAccess)
            throw std::runtime_error("Defaults:  Reverse access is not correct for way 35600238.");
          if (forward != 0) // no access
            throw std::runtime_error("Defaults:  Forward access is not correct for way 35600238.");
        }
        // trunk that has pedestrian, moped and bike access but motorroad key changes turns off
        // pedestrians and bikes
      } else if (e_offset.wayid() == 139156014) {
        if (directededge.forward()) {
          if (forward !=
              (kAutoAccess | kHOVAccess | kTaxiAccess | kPedestrianAccess | kWheelchairAccess |
               kBicycleAccess | kTruckAccess | kBusAccess | kMopedAccess | kMotorcycleAccess))
            throw std::runtime_error("Defaults:  Forward access is not correct for way 139156014.");
          if (reverse != (kPedestrianAccess | kWheelchairAccess))
            throw std::runtime_error("Defaults:  Reverse access is not correct for way 139156014.");
        } else {
          if (reverse !=
              (kAutoAccess | kHOVAccess | kTaxiAccess | kPedestrianAccess | kWheelchairAccess |
               kBicycleAccess | kTruckAccess | kBusAccess | kMopedAccess | kMotorcycleAccess))
            throw std::runtime_error("Defaults:  Reverse access is not correct for way 139156014.");
          if (forward != (kPedestrianAccess | kWheelchairAccess))
            throw std::runtime_error("Defaults:  Forward access is not correct for way 139156014.");
        }
      } else if (e_offset.wayid() == 512688404) { // motorroad key test
        if (directededge.forward()) {
          if (forward != (kAutoAccess | kHOVAccess | kTaxiAccess | kTruckAccess | kBusAccess))
            throw std::runtime_error("Defaults:  Forward access is not correct for way 512688404.");
          if (reverse != 0)
            throw std::runtime_error("Defaults:  Reverse access is not correct for way 512688404.");
        } else {
          if (reverse != (kAutoAccess | kHOVAccess | kTaxiAccess | kTruckAccess | kBusAccess))
            throw std::runtime_error("Defaults:  Reverse access is not correct for way 512688404.");
          if (forward != 0)
            throw std::runtime_error("Defaults:  Forward access is not correct for way 512688404.");
        }
      }
    }
  }

  // Enhance the local level of the graph. This adds information to the local
  // level that is usable across all levels (density, administrative
  // information (and country based attribution), edge transition logic, etc.
  GraphEnhancer::Enhance(conf, osmdata, access_file);

  // load a tile and test that the country level access is set.
  GraphId id2(820099, 2, 0);
  GraphTile t2("test/data/amsterdam_tiles", id2);

  GraphReader graph_reader2(conf.get_child("mjolnir"));
  GraphTileBuilder tilebuilder2(graph_reader2.tile_dir(), id2, true);

  for (uint32_t i = 0; i < tilebuilder2.header()->nodecount(); i++) {
    NodeInfo& nodeinfo = tilebuilder2.node_builder(i);
    uint32_t count = nodeinfo.edge_count();
    for (uint32_t j = 0; j < count; j++) {
      DirectedEdge& directededge = tilebuilder2.directededge_builder(nodeinfo.edge_index() + j);

      auto e_offset = tilebuilder2.edgeinfo(directededge.edgeinfo_offset());

      uint32_t forward = directededge.forwardaccess();
      uint32_t reverse = directededge.reverseaccess();

      // cycleway (not oneway) should have kPedestrianAccess and kBicycleAccess
      if (e_offset.wayid() == 7047088) {
        if ((forward != (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess)) ||
            (reverse != (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess))) {
          throw std::runtime_error("Enhanced:  Access is not correct for way 7047088.");
        }
        // cycleway (is oneway) should have kPedestrianAccess and kBicycleAccess
      } else if (e_offset.wayid() == 31976259) {
        if (directededge.forward()) {
          if (forward != (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess))
            throw std::runtime_error("Enhanced:  Forward access is not correct for way 31976259.");
          if (reverse !=
              (kPedestrianAccess |
               kWheelchairAccess)) // only pedestrian access because this is a oneway cycleway
            throw std::runtime_error("Enhanced:  Reverse access is not correct for way 31976259.");
        } else {
          if (reverse != (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess))
            throw std::runtime_error("Enhanced:  Reverse access is not correct for way 31976259.");
          if (forward != (kPedestrianAccess | kWheelchairAccess))
            throw std::runtime_error("Enhanced:  Forward access is not correct for way 31976259.");
        }
        // trunk should have no kPedestrianAccess
      } else if (e_offset.wayid() == 139156014) {
        if (directededge.forward()) {
          if (forward != (kAutoAccess | kHOVAccess | kTaxiAccess | kTruckAccess | kBusAccess |
                          kMotorcycleAccess))
            throw std::runtime_error("Enhanced:  Forward access is not correct for way 139156014.");
          if (reverse != 0)
            throw std::runtime_error("Enhanced:  Reverse access is not correct for way 139156014.");
        } else {
          if (reverse != (kAutoAccess | kHOVAccess | kTaxiAccess | kTruckAccess | kBusAccess |
                          kMotorcycleAccess))
            throw std::runtime_error("Enhanced:  Reverse access is not correct for way 139156014.");
          if (forward != 0)
            throw std::runtime_error("Enhanced:  Forward access is not correct for way 139156014.");
        }
      }
    }
  }

  // Remove temporary files
  remove_temp_file(ways_file);
  remove_temp_file(way_nodes_file);
  remove_temp_file(nodes_file);
  remove_temp_file(edges_file);
  remove_temp_file(access_file);
  remove_temp_file(cr_from_file);
  remove_temp_file(cr_to_file);
}

void TestCountryAccess() {
  // write the tiles with it
  CountryAccess(config_file);
}

void DoConfig() {
  // make a config file
  write_config(config_file);
}

} // namespace

int main() {

  test::suite suite("countryaccess");

  suite.test(TEST_CASE(DoConfig));
  suite.test(TEST_CASE(TestCountryAccess));

  return suite.tear_down();
}
