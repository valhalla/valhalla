#include "test.h"
#include "mjolnir/osmnode.h"

#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "../valhalla/mjolnir/pbfgraphparser.h"

using namespace std;
using namespace valhalla::mjolnir;

namespace {

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

  valhalla::mjolnir::PBFGraphParser parser(conf.get_child("mjolnir"));
  auto osmdata = parser.Load({"test/data/liechtenstein-latest.osm.pbf"});
/*
  //When we split set the uses at bollards and gates, this bollard will be found.
  osmdata.SortNodes();
  auto node = osmdata.GetNode(392700757);
  if (node == osmdata.nodes.end())
    throw std::runtime_error("Bollard not at a intersection test failed.");
  else {
    if (node.intersection())
      throw std::runtime_error("Bollard not marked as intersection.");
  }

  //When we split set the uses at bollards and gates, this gate will be found.
  node = osmdata.GetNode(376947468);
  if (node == osmdata.nodes.end())
    throw std::runtime_error("Gate not at a intersection test failed.");
  else {
    if (node.intersection())
      throw std::runtime_error("Gate not marked as intersection.");
  }

  //Is a gate with foot and bike flags set; however, access is private.
  node = osmdata.GetNode(2949666866);
  if (node == osmdata.nodes.end())
    throw std::runtime_error("Gate at a intersection test failed.");
  else {
    if (!node.intersection() ||
        !node.gate() || node.modes_mask() != 6)
      throw std::runtime_error("Gate at end of way test failed.");
  }

  //When we split set the uses at bollards and gates, this bollard will be found.
  //Is a bollard with foot and bike flags set.
  node = osmdata.GetNode(569645326);
  if (node == osmdata.nodes.end())
    throw std::runtime_error("Bollard(with flags) not at a intersection test failed.");
  else {
    if (node.intersection()) // ||
       // || !node->second.bollard() || node->second.modes_mask() != 6)
      throw std::runtime_error("Bollard(with flags) not marked as intersection.");
  }

  //When we split set the uses at bollards and gates, this bollard will be found.
  //Is a bollard=block with foot flag set.
  node = osmdata..GetNode(1819036441);
  if (node == osmdata.nodes.end())
    throw std::runtime_error("Bollard=block not at a intersection test failed.");
  else {
    if (node.intersection()) // ||
       // || !node->second.bollard() || node->second.modes_mask() != 4)
      throw std::runtime_error("Bollard=block not marked as intersection.");
  }
**/
}

void RemovableBollards(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);
/**
  valhalla::mjolnir::PBFGraphParser parser(conf.get_child("mjolnir"));
  auto osmdata = parser.Load({"test/data/rome.osm.pbf"});

  //When we split set the uses at bollards and gates, this bollard will be found.
  //Is a bollard=rising with foot flag set.
  osmdata.SortNodes();
  auto node = osmdata.GetNode(2425784125);
   if (node == osmdata.nodes.end())
     throw std::runtime_error("Rising Bollard not at a intersection test failed.");
   else {
     if (node.intersection()) // ||
       // || !node->second.bollard() || node->second.modes_mask() != 4)
       throw std::runtime_error("Rising Bollard not marked as intersection.");
   }
**/
}

void Exits(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);
/**
  valhalla::mjolnir::PBFGraphParser parser(conf.get_child("mjolnir"));
  auto osmdata = parser.Load({"test/data/harrisburg.osm.pbf"});
  osmdata.SortNodes();
  auto node = osmdata.GetNode(33698177);
  if (node == osmdata.nodes.end())
    throw std::runtime_error("Exit node not found.");
  else {
    if (!node.intersection() ||
        !node.ref() || osmdata.node_ref[33698177] != "51A-B")
      throw std::runtime_error("Ref not set correctly .");
  }

  node = osmdata.GetNode(1901353894);
  if (node == osmdata.nodes.end())
    throw std::runtime_error("Exit node not found.");
  else {
    if (!node.intersection() ||
        !node.ref() || osmdata.node_name[1901353894] != "Harrisburg East")
      throw std::runtime_error("Ref not set correctly .");
  }

  node = osmdata.GetNode(462240654);
  if (node == osmdata.nodes.end())
    throw std::runtime_error("Exit node not found.");
  else {
    if (!node.intersection() || osmdata.node_exit_to[462240654] != "PA441")
      throw std::runtime_error("Ref not set correctly .");
  }
**/
}




void BicycleTrafficSignals(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);
/**
  valhalla::mjolnir::PBFGraphParser parser(conf.get_child("mjolnir"));
  auto osmdata = parser.Load({"test/data/nyc.osm.pbf"});
  osmdata.SortNodes();
  //When we support finding bike rentals, this test will need updated.
  auto node = osmdata.GetNode(3146484929);
  if (node != osmdata.nodes.end())
    throw std::runtime_error("Bike rental test failed.");
 // /*else {
    if (node->second.intersection())
      throw std::runtime_error("Bike rental not marked as intersection.");
 // }*/
/**
  //When we support finding shops that rent bikes, this test will need updated.
  node = osmdata.GetNode(2592264881);
  if (node != osmdata.nodes.end())
    throw std::runtime_error("Bike rental at a shop test failed.");
  /*else {
    if (node->second.intersection())
      throw std::runtime_error("Bike rental at a shop not marked as intersection.");
  }*/

/**  node = osmdata.GetNode(42439096);
  if (node == osmdata.nodes.end())
    throw std::runtime_error("Traffic Signal test failed.");
  else {
    if (!node.intersection() || !node.traffic_signal())
      throw std::runtime_error("Traffic Signal test failed.");
  }
  **/
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

void TestBicycleTrafficSignals() {
  //write the tiles with it
  BicycleTrafficSignals("test/test_config");
}

void TestExits() {
  //write the tiles with it
  Exits("test/test_config");
}

}

int main() {
  test::suite suite("parser");

  suite.test(TEST_CASE(DoConfig));
  suite.test(TEST_CASE(TestBollardsGates));
  suite.test(TEST_CASE(TestRemovableBollards));
  suite.test(TEST_CASE(TestBicycleTrafficSignals));
  suite.test(TEST_CASE(TestExits));

  return suite.tear_down();
}
