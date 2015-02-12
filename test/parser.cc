#include "test.h"
#include "mjolnir/pbfparser.h"
#include "mjolnir/osmnode.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/graphoptimizer.h"

#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace std;
using namespace valhalla::mjolnir;

namespace {

class test_graph_builder : public GraphBuilder {
 public:
  using GraphBuilder::GraphBuilder;
  using GraphBuilder::nodes_;
  using GraphBuilder::Build;
};
void write_config(const std::string& filename) {
  std::ofstream file;
  try {
  file.open(filename, std::ios_base::trunc);
  file << "{ \
      \"mjolnir\": { \
        \"input\": { \
          \"type\": \"protocolbuffer\" \
        }, \
        \"hierarchy\": { \
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
          \"way_function\": \"ways_proc\" , \
          \"relation_script\": \"test/lua/edges.lua\", \
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

  valhalla::mjolnir::PBFParser parser(conf.get_child("mjolnir"));
  auto osmdata = parser.Load({"test/data/liechtenstein-latest.osm.pbf"});
//  test_graph_builder* gb = new test_graph_builder(conf.get_child("mjolnir"));
//  gb->Build(osmdata);
  /*
  auto node = osmdata.nodes.find(392700757);
   if (node != osmdata.nodes.end())
     throw std::runtime_error("Bollard at no intersection test failed.");
   else {
     if (!node->second.intersection())
       throw std::runtime_error("Node at Bollard not marked as intersection.");
   }

  //When we split set the uses at bollards and gates, this bollard will be found.
  auto node = gb->nodes_.find(392700757);
  if (node != gb->nodes_.end())
    throw std::runtime_error("Bollard at no intersection test failed.");

  //When we split set the uses at bollards and gates, this gate will be found.
  node = gb->nodes_.find(376947468);
  if (node != gb->nodes_.end())
    throw std::runtime_error("Gate at no intersection test failed.");

  //Is a gate with foot and bike flags set; however, access is private.
  node = gb->nodes_.find(2949666866);
  if (node != gb->nodes_.end() && !node->second.gate() && node->second.modes_mask() != 0)
    throw std::runtime_error("Gate at end of way test failed");
*/
}

void TestRouteParser() {
  //make a config file
  write_config("test/test_config");

  //write the tiles with it
  write_tiles("test/test_config");

}

}

int main() {
  test::suite suite("parser");

  suite.test(TEST_CASE(TestRouteParser));

  return suite.tear_down();
}
