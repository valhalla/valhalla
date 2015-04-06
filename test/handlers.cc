#include "test.h"
#include "tyr/route_handler.h"
#include "tyr/json.h"

#include <valhalla/mjolnir/pbfgraphparser.h>
#include <valhalla/mjolnir/graphbuilder.h>
#include <valhalla/mjolnir/graphenhancer.h>
#include <valhalla/mjolnir/hierarchybuilder.h>
#include <valhalla/mjolnir/graphvalidator.h>
#include <valhalla/midgard/logging.h>

#include <fstream>
#include <sstream>
#include <boost/python/dict.hpp>
#include <boost/python/str.hpp>
#include <boost/python/list.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <Python.h>

namespace {

void write_config(const std::string& filename) {
  std::ofstream file;
  try {
  file.open(filename, std::ios_base::trunc);
  file << "{ \
      \"thor\": {}, \
      \"mjolnir\": { \
        \"input\": { \
          \"type\": \"protocolbuffer\" \
        }, \
        \"hierarchy\": { \
          \"tile_dir\": \"test/tiles\", \
          \"levels\": [ \
            {\"name\": \"local\", \"level\": 2, \"size\": 0.25}, \
            {\"name\": \"arterial\", \"level\": 1, \"size\": 1, \"importance_cutoff\": \"Tertiary\"}, \
            {\"name\": \"highway\", \"level\": 0, \"size\": 4, \"importance_cutoff\": \"Trunk\"} \
          ] \
        }, \
        \"tagtransform\": { \
          \"node_script\": \"test/lua/vertices.lua\", \
          \"node_function\": \"nodes_proc\", \
          \"way_script\": \"test/lua/edges.lua\", \
          \"way_function\": \"ways_proc\", \
          \"relation_script\": \"test/lua/relations.lua\", \
          \"relation_function\": \"rels_proc\" \
        }, \
        \"admin\": { \
          \"admin_dir\": \"/data/valhalla\", \
          \"db_name\": \"admin.sqlite\" \
        } \
      } \
    }";
  }
  catch(...) {

  }
  file.close();
}

void write_tiles(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);

  auto osmdata = valhalla::mjolnir::PBFGraphParser::Parse(conf.get_child("mjolnir"), {"test/data/liechtenstein-latest.osm.pbf"},
      "test_ways_file.bin", "test_way_nodes_file.bin");
  valhalla::mjolnir::GraphBuilder::Build(conf.get_child("mjolnir"), osmdata, "test_ways_file.bin", "test_way_nodes_file.bin");
  valhalla::mjolnir::GraphEnhancer::Enhance(conf);
  valhalla::mjolnir::HierarchyBuilder::Build(conf.get_child("mjolnir.hierarchy"));
  valhalla::mjolnir::GraphValidator::Validate(conf.get_child("mjolnir.hierarchy"));
}

boost::property_tree::ptree make_request(const std::string& loc1, const std::string& loc2,
  const std::string& request_type) {

  using namespace valhalla::tyr;
  auto json = json::map
  ({
    {"loc", json::array({ loc1, loc2 })},
    {"costing_method", request_type},
    {"output", std::string("json")},
    {"z", static_cast<uint64_t>(17)},
    {"instructions", static_cast<bool>(true)},
    {"jsonp", std::string("some_callback")},
  });
  std::stringstream stream; stream << *json;
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(stream, pt);
  return pt;
}

void TestRouteHanlder() {
  //make a config file
  write_config("test/test_config");

  //write the tiles with it
  write_tiles("test/test_config");

  boost::property_tree::ptree config;
  boost::property_tree::read_json("test/test_config", config);

  //make the input
  boost::property_tree::ptree request =
    make_request("47.139815, 9.525708", "47.167321, 9.509609", "auto");

  //run the route
  valhalla::tyr::RouteHandler handler(config, request);
  LOG_DEBUG(handler.Action());
}

void TestCustomRouteHandler() {
  boost::property_tree::ptree config;
  boost::property_tree::read_json("test/test_config", config);

  //make the input
  boost::property_tree::ptree request =
    make_request("47.139815, 9.525708", "47.167321, 9.509609", "auto");

  //run the route
  valhalla::tyr::RouteHandler handler(config, request);
  LOG_DEBUG(handler.Action());
}

}

int main() {
  test::suite suite("handlers");

  suite.test(TEST_CASE(TestRouteHanlder));

  suite.test(TEST_CASE(TestCustomRouteHandler));

  //suite.test(TEST_CASE(TestNearestHanlder));

  //suite.test(TEST_CASE(TestLocateHanlder));

  return suite.tear_down();
}
