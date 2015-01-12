#include "test.h"
#include "valhalla/tyr/route_handler.h"
#include <valhalla/mjolnir/graphbuilder.h>

#include <fstream>
#include <boost/python/dict.hpp>
#include <boost/python/str.hpp>
#include <boost/python/list.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace {

void write_config(const std::string& filename) {
  std::ofstream file;
  try {
  file.open(filename, std::ios_base::trunc);
  file << "{ \
      \"input\": { \
        \"type\": \"protocolbuffer\" \
      }, \
      \"output\": { \
        \"tile_dir\": \"test/tiles\", \
        \"levels\": [ \
          {\"name\": \"local\", \"level\": 2, \"size\": 0.25}, \
          {\"name\": \"arterial\", \"level\": 1, \"size\": 1, \"importance_cutoff\": \"TertiaryUnclassified\"}, \
          {\"name\": \"highway\", \"level\": 0, \"size\": 4, \"importance_cutoff\": \"Trunk\"} \
        ] \
      }, \
      \"tagtransform\": { \
        \"node_script\": \"test/lua/vertices.lua\", \
        \"node_function\": \"nodes_proc\", \
        \"way_script\": \"test/lua/edges.lua\", \
        \"way_function\": \"ways_proc\" \
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
  valhalla::mjolnir::GraphBuilder builder(conf, "test/data/liechtenstein-latest.osm.pbf");
  builder.Build();
}

boost::python::dict make_request(const std::string& loc1, const std::string& loc2,
  const std::string& request_type, const std::string& config_file) {
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
  bp::dict dict;
  bp::list loc;
  loc.append(bp::str(loc1));
  loc.append(bp::str(loc2));
  bp::setitem(dict, bp::str("loc"), loc);
  bp::setitem(dict, bp::str("costing_method"), bp::str(request_type));
  bp::setitem(dict, bp::str("output"), bp::list(bp::str("json")));
  bp::setitem(dict, bp::str("z"), bp::list(bp::str("17")));
  bp::setitem(dict, bp::str("config"), bp::str(config_file));
  bp::setitem(dict, bp::str("instructions"), bp::list(bp::str("true")));
  return dict;
}

void TestRouteHanlder() {
  //make a config file
  write_config("test/test_config");

  //write the tiles with it
  write_tiles("test/test_config");

  //make the input
  boost::python::dict dict =
    make_request("40.657912,-73.914450", "40.040501,-76.306271", "auto", "test/test_config");

  //run the route
  valhalla::tyr::RouteHandler handler(dict);
  handler.Action();
}

}


int main() {
  test::suite suite("handlers");

  suite.test(TEST_CASE(TestRouteHanlder));

  //suite.test(TEST_CASE(TestNearestHanlder));

  //suite.test(TEST_CASE(TestLocateHanlder));

  return suite.tear_down();
}
