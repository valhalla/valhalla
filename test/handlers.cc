#include "test.h"
#include "tyr/route_handler.h"
#include <valhalla/mjolnir/pbfgraphparser.h>
#include <valhalla/mjolnir/graphbuilder.h>
#include <valhalla/mjolnir/graphenhancer.h>
#include <valhalla/mjolnir/graphoptimizer.h>
#include <valhalla/midgard/logging.h>

#include <fstream>
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

  auto osmdata = valhalla::mjolnir::PBFGraphParser::Parse(conf.get_child("mjolnir"), {"test/data/liechtenstein-latest.osm.pbf"});
  valhalla::mjolnir::GraphBuilder builder(conf.get_child("mjolnir"));
  builder.Build(osmdata);
  valhalla::mjolnir::GraphEnhancer enhancer(conf.get_child("mjolnir.hierarchy"));
  enhancer.Enhance();
  valhalla::mjolnir::GraphOptimizer graphoptimizer(conf.get_child("mjolnir.hierarchy"));
  graphoptimizer.Optimize();
}

boost::python::dict make_request(const std::string& loc1, const std::string& loc2,
  const std::string& request_type) {
  //the dict should look something like this:
  /*{
      'loc': ['40.657912,-73.914450', '40.040501,-76.306271'],
      'costing_method': 'auto',
      'output': ['json'],
      'z': ['17'],
      'config': 'conf/pbf2graph.json',
      'instructions': ['true']
    }*/

  namespace bp = boost::python;
  bp::dict request;
  bp::list loc;
  loc.append(loc1);
  loc.append(loc2);
  request["loc"] = loc;
  request["costing_method"] = request_type.c_str();
  bp::list output;  output.append("json");
  request["output"] = output;
  bp::list z; z.append("17");
  request["z"] = z;
  bp::list instructions; instructions.append("true");
  request["instructions"] = instructions;
  return request;
}

void TestRouteHanlder() {
  //make a config file
  write_config("test/test_config");

  //write the tiles with it
  write_tiles("test/test_config");

  //make the input
  boost::python::dict dict =
    make_request("47.139815, 9.525708", "47.167321, 9.509609", "auto");

  //run the route
  valhalla::tyr::RouteHandler handler("test/test_config", dict);
  LOG_DEBUG(handler.Action());
}

}

int main() {
  test::suite suite("handlers");

  Py_Initialize();
  suite.test(TEST_CASE(TestRouteHanlder));

  //suite.test(TEST_CASE(TestNearestHanlder));

  //suite.test(TEST_CASE(TestLocateHanlder));
  Py_Finalize();

  return suite.tear_down();
}
