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

void BollardsGates(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);

  valhalla::mjolnir::PBFParser parser(conf.get_child("mjolnir"));
  auto osmdata = parser.Load({"test/data/liechtenstein-latest.osm.pbf"});
  test_graph_builder* gb = new test_graph_builder(conf.get_child("mjolnir"));
  gb->Build(osmdata);

  //When we split set the uses at bollards and gates, this bollard will be found.
  auto node = gb->nodes_.find(392700757);
  if (node != gb->nodes_.end())
    throw std::runtime_error("Bollard not at a intersection test failed.");

  //When we split set the uses at bollards and gates, this gate will be found.
  node = gb->nodes_.find(376947468);
  if (node != gb->nodes_.end())
    throw std::runtime_error("Gate not at a intersection test failed.");

  //Is a gate with foot and bike flags set; however, access is private.
  node = gb->nodes_.find(2949666866);
  if (node == gb->nodes_.end() || !node->second.gate() || node->second.modes_mask() != 6)
    throw std::runtime_error("Gate at end of way test failed");

  //When we split set the uses at bollards and gates, this bollard will be found.
  //Is a bollard with foot and bike flags set.
  node = gb->nodes_.find(569645326);
  if (node != gb->nodes_.end()) //|| !node->second.bollard() || node->second.modes_mask() != 6)
    throw std::runtime_error("Bollard at end of way test failed");

  //When we split set the uses at bollards and gates, this bollard will be found.
  //Is a bollard=block with foot flag set.
  node = gb->nodes_.find(1819036441);
  if (node != gb->nodes_.end()) //|| !node->second.bollard() || node->second.modes_mask() != 4)
    throw std::runtime_error("Block at end of way test failed");

}

void RemovableBollards(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);

  valhalla::mjolnir::PBFParser parser(conf.get_child("mjolnir"));
  auto osmdata = parser.Load({"test/data/Rome.osm.pbf"});
  test_graph_builder* gb = new test_graph_builder(conf.get_child("mjolnir"));
  gb->Build(osmdata);

  //When we split set the uses at bollards and gates, this bollard will be found.
  //Is a bollard=rising with foot flag set.
  auto node = gb->nodes_.find(2425784125);
  if (node != gb->nodes_.end()) //|| !node->second.bollard() || node->second.modes_mask() != 4)
    throw std::runtime_error("Block at end of way test failed");

}

void Bicycle(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);

  valhalla::mjolnir::PBFParser parser(conf.get_child("mjolnir"));
  auto osmdata = parser.Load({"test/data/NYC.osm.pbf"});
  test_graph_builder* gb = new test_graph_builder(conf.get_child("mjolnir"));
  gb->Build(osmdata);

  //When we support finding bike rentals, this test will need updated.
  auto node = gb->nodes_.find(3146484929);
  if (node != gb->nodes_.end())
    throw std::runtime_error("Bike rental test failed.");

  //When we support finding shops that rent bikes, this test will need updated.
  node = gb->nodes_.find(2592264881);
  if (node != gb->nodes_.end())
    throw std::runtime_error("Bike rental at a shop failed.");

}

void DoConfig() {
  //make a config file
  write_config("test/test_config");
}

void TestBollardsGates() {
  //write the tiles with it
  BollardsGates("test/test_config");
}

void TestRemovableBollards() {
  //write the tiles with it
  RemovableBollards("test/test_config");
}

void TestBicycle() {
  //write the tiles with it
  Bicycle("test/test_config");
}

}

int main() {
  test::suite suite("parser");

  suite.test(TEST_CASE(DoConfig));
  suite.test(TEST_CASE(TestBollardsGates));
  suite.test(TEST_CASE(TestRemovableBollards));
  suite.test(TEST_CASE(TestBicycle));

  return suite.tear_down();
}
