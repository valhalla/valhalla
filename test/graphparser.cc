#include <cstdint>
#include "test.h"
#include "mjolnir/osmnode.h"
#include "mjolnir/pbfgraphparser.h"
#include "midgard/sequence.h"

#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>

#include "baldr/graphconstants.h"
#include "baldr/directededge.h"

using namespace std;
using namespace valhalla::mjolnir;
using namespace valhalla::baldr;

namespace {

const std::string config_file = "test/test_config_gp";


const auto node_predicate = [](const OSMWayNode& a, const OSMWayNode& b) {
  return a.node.osmid < b.node.osmid;
};

OSMNode GetNode(uint64_t node_id, sequence<OSMWayNode>& way_nodes) {
  auto found = way_nodes.find({node_id}, node_predicate);
  if(found == way_nodes.end())
    throw std::runtime_error("Couldn't find node: " + std::to_string(node_id));
  return (*found).node;
}

auto way_predicate = [](const OSMWay& a, const OSMWay& b){
  return a.osmwayid_ < b.osmwayid_;
};

OSMWay GetWay(uint64_t way_id, sequence<OSMWay>& ways) {
  auto found = ways.find({way_id}, way_predicate);
  if(found == ways.end())
    throw std::runtime_error("Couldn't find way: " + std::to_string(way_id));
  return *found;
}

void BollardsGatesAndAccess(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string access_file = "test_access.bin";
  std::string restriction_file = "test_complex_restrictions.bin";

  auto osmdata = PBFGraphParser::Parse(conf.get_child("mjolnir"), {"test/data/liechtenstein-latest.osm.pbf"},
                                       ways_file, way_nodes_file, access_file, restriction_file);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  way_nodes.sort(node_predicate);

  sequence<OSMWay> ways(ways_file, false);
  ways.sort(way_predicate);

  // bus access tests.
  auto way = GetWay(85744121, ways);
  if (way.auto_forward() != false || way.moped_forward () != false || way.bike_forward() != true || way.bus_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != false || way.moped_backward() != false || way.bike_backward() != false || way.bus_backward() != false) {
    throw std::runtime_error("Access is not set correctly for way 85744121.");
  }

  way = GetWay(86260080, ways);
  if (way.auto_forward() != true || way.bike_forward() != true || way.bus_forward() != true || way.moped_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != true || way.bike_backward() != true || way.bus_backward() != true || way.moped_backward() != true) {
    throw std::runtime_error("Access is not set correctly for way 86260080.");
  }

  way = GetWay(161683833, ways);
  if (way.auto_forward() != true || way.bike_forward() != true || way.bus_forward() != true || way.moped_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != true || way.bike_backward() != true || way.bus_backward() != true || way.moped_backward() != true) {
    throw std::runtime_error("Access is not set correctly for way 161683833.");
  }

  //We split set the uses at bollards and gates.
  auto node = GetNode(392700757, way_nodes);
  if (!node.intersection())
    throw std::runtime_error("Bollard not marked as intersection.");

  //We split set the uses at bollards and gates.
  node = GetNode(376947468, way_nodes);
  if (!node.intersection())
    throw std::runtime_error("Gate not marked as intersection.");

  //Is a gate with foot and bike flags set; however, access is private.
  node = GetNode(2949666866, way_nodes);
  if (!node.intersection() ||
      node.type() != NodeType::kGate || node.access_mask() !=
          (kAutoAccess | kHOVAccess | kTruckAccess | kBusAccess | kEmergencyAccess |
              kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess))
    throw std::runtime_error("Gate at end of way test failed.");

  //block
  node = GetNode(1819036441, way_nodes);
  if (!node.intersection() ||
      node.type() != NodeType::kBollard || node.access_mask() !=
          (kPedestrianAccess | kWheelchairAccess | kBicycleAccess))
    throw std::runtime_error("Block test failed.");

  //border control
  node = GetNode(3256854624, way_nodes);
  if (!node.intersection() ||
      node.type() != NodeType::kBorderControl || node.access_mask() !=
          (kAutoAccess | kHOVAccess | kTruckAccess | kBusAccess | kEmergencyAccess |
              kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess))
    throw std::runtime_error("Border control test failed.");

  //has bike tag but all should have access
  node = GetNode(696222071, way_nodes);
  if (!node.intersection() || node.access_mask() !=
      (kAutoAccess | kHOVAccess | kTruckAccess | kBusAccess | kEmergencyAccess |
          kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess))
    throw std::runtime_error("Bike access only failed.");

  //Is a bollard with no flags set.
  node = GetNode(569645326, way_nodes);
  if (!node.intersection() ||
      node.type() != NodeType::kBollard || node.access_mask() !=
          (kPedestrianAccess | kWheelchairAccess | kBicycleAccess))
    throw std::runtime_error("Bollard(with flags) not marked as intersection.");

  //Is a bollard=block with foot flag set.
  node = GetNode(1819036441, way_nodes);
  if (!node.intersection() ||
      node.type() != NodeType::kBollard || node.access_mask() !=
          (kPedestrianAccess | kWheelchairAccess | kBicycleAccess))
    throw std::runtime_error("Bollard=block not marked as intersection.");

  auto bike = osmdata.bike_relations.equal_range(25452580);
  way = GetWay(25452580, ways);
  uint32_t bike_network = 0;

  for (auto b = bike.first; b != bike.second; ++b)
    bike_network |= b->second.bike_network;

  if (bike_network != kRcn || way.bike_network() != 0)
    throw std::runtime_error("rcn not marked on way 25452580.");

  way = GetWay(74584853, ways);
  bike_network = 0;
  bike = osmdata.bike_relations.equal_range(74584853);

  for (auto b = bike.first; b != bike.second; ++b)
    //mountain bike networks have local, regional, and national networks set too.
    if (b->second.bike_network & kMcn )
      bike_network |= kMcn;
    else bike_network |= b->second.bike_network;

  if ((!(bike_network & kMcn) || !(bike_network & kLcn)) || way.bike_network() != 0)
    throw std::runtime_error("lcn and mtb not marked on way 74584853.");

  way = GetWay(75786176, ways);
  bike_network = 0;
  bike = osmdata.bike_relations.equal_range(75786176);

  for (auto b = bike.first; b != bike.second; ++b) {
     //mountain bike networks have local, regional, and national networks set too.
     if (b->second.bike_network & kMcn )
       bike_network |= kMcn;
     else bike_network |= b->second.bike_network;
  }

  if ((!(bike_network & kMcn) || !(bike_network & kRcn)) || way.bike_network() != 0)
    throw std::runtime_error("rcn and mtb not marked on way 75786176.");

  boost::filesystem::remove(ways_file);
  boost::filesystem::remove(way_nodes_file);
  boost::filesystem::remove(access_file);
}

void RemovableBollards(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string access_file = "test_access.bin";
  std::string restriction_file = "test_complex_restrictions.bin";

  auto osmdata = PBFGraphParser::Parse(conf.get_child("mjolnir"), {"test/data/rome.osm.pbf"},
                                       ways_file, way_nodes_file, access_file, restriction_file);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  way_nodes.sort(node_predicate);

  //Is a bollard=rising is saved as a gate...with foot flag and bike set.
  auto node = GetNode(2425784125, way_nodes);
  if (!node.intersection() ||
    node.type() != NodeType::kGate || node.access_mask() !=
      (kAutoAccess | kHOVAccess | kTruckAccess | kBusAccess | kEmergencyAccess |
          kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess))
    throw std::runtime_error("Rising Bollard not marked as intersection.");

  boost::filesystem::remove(ways_file);
  boost::filesystem::remove(way_nodes_file);
  boost::filesystem::remove(access_file);
}

void Exits(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string access_file = "test_access.bin";
  std::string restriction_file = "test_complex_restrictions.bin";

  auto osmdata = PBFGraphParser::Parse(conf.get_child("mjolnir"), {"test/data/harrisburg.osm.pbf"},
                                       ways_file, way_nodes_file, access_file, restriction_file);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  way_nodes.sort(node_predicate);

  auto node = GetNode(33698177, way_nodes);

  if (!node.intersection() ||
      !node.ref() || osmdata.node_ref[33698177] != "51A-B")
    throw std::runtime_error("Ref not set correctly .");


  node = GetNode(1901353894, way_nodes);

  if (!node.intersection() ||
      !node.ref() || osmdata.node_name[1901353894] != "Harrisburg East")
    throw std::runtime_error("Ref not set correctly .");


  node = GetNode(462240654, way_nodes);

  if (!node.intersection() || osmdata.node_exit_to[462240654] != "PA441")
    throw std::runtime_error("Ref not set correctly .");

  boost::filesystem::remove(ways_file);
  boost::filesystem::remove(way_nodes_file);
  boost::filesystem::remove(access_file);

}

void Baltimore(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string access_file = "test_access.bin";
  std::string restriction_file = "test_complex_restrictions.bin";

  auto osmdata = PBFGraphParser::Parse(conf.get_child("mjolnir"), {"test/data/baltimore.osm.pbf"},
                                       ways_file, way_nodes_file, access_file, restriction_file);
  sequence<OSMWay> ways(ways_file, false);
  ways.sort(way_predicate);

  // bike_forward and reverse is set to false by default.  Meaning defaults for
  // highway = pedestrian.  Bike overrides bicycle=designated and/or cycleway=shared_lane
  // make it bike_forward and reverse = true
  auto way = GetWay(216240466, ways);
  if (way.auto_forward() != false || way.bus_forward() != false || way.moped_forward() != false || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != false || way.bus_backward() != false || way.moped_backward() != false || way.bike_backward() != true) {
    throw std::runtime_error("Access is not set correctly for way 216240466.");
  }
  // access for all
  way = GetWay(138388359, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.moped_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != true || way.bus_backward() != true || way.moped_backward() != true ||  way.bike_backward() != true) {
    throw std::runtime_error("Access is not set correctly for way 138388359.");
  }
  // footway...pedestrian only
  way = GetWay(133689121, ways);
  if (way.auto_forward() != false || way.bus_forward() != false || way.moped_forward() != false || way.bike_forward() != false || way.pedestrian() != true ||
      way.auto_backward() != false || way.bus_backward() != false || way.moped_backward() != false || way.bike_backward() != false) {
    throw std::runtime_error("Access is not set correctly for way 133689121.");
  }
  // oneway
  way = GetWay(49641455, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.moped_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != false || way.bus_backward() != false || way.moped_backward() != false || way.bike_backward() != false) {
    throw std::runtime_error("Access is not set correctly for way 49641455.");
  }

  // Oneway test.  Make sure auto backward is set for ways where oneway=no.
  way = GetWay(192573108, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.moped_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != true || way.bus_backward() != true || way.moped_backward() != true ||  way.bike_backward() != true) {
    throw std::runtime_error("Forward/Backward/Pedestrian access is not set correctly for way 192573108.");
  }

  sequence<OSMWayNode> way_nodes(way_nodes_file, false, true);
  way_nodes.sort(node_predicate);
  auto node = GetNode(49473254, way_nodes);

  if (!node.intersection() || node.type() != NodeType::kTollBooth)
    throw std::runtime_error("Toll Booth 49473254 test failed.");

  auto res = osmdata.restrictions.equal_range(98040438);

  if (res.first == osmdata.restrictions.end())
    throw std::runtime_error("Failed to find 98040438 restriction.");

  for (auto r = res.first; r != res.second; ++r) {
    if (r->second.to() == 6003340) {
      if (r->second.via() != 2123388822 || r->second.type() != RestrictionType::kNoLeftTurn)
        throw std::runtime_error("98040438 restriction test failed for to: 6003340");
    }
    else if (r->second.to() == 98040438) {
      if (r->second.via() != 2123388822 || r->second.type() != RestrictionType::kNoUTurn)
        throw std::runtime_error("98040438 restriction test failed for to: 98040438");
    }
    else throw std::runtime_error("98040438 restriction test failed.");
  }

  boost::filesystem::remove(ways_file);
  boost::filesystem::remove(way_nodes_file);
  boost::filesystem::remove(access_file);
}

void Bike(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string access_file = "test_access.bin";
  std::string restriction_file = "test_complex_restrictions.bin";

  auto osmdata = PBFGraphParser::Parse(conf.get_child("mjolnir"), {"test/data/bike.osm.pbf"},
                                       ways_file, way_nodes_file, access_file, restriction_file);
  sequence<OSMWay> ways(ways_file, false);
  ways.sort(way_predicate);

  //http://www.openstreetmap.org/way/6885577#map=14/51.9774/5.7718
  //direction of this way for oneway is flipped.  Confirmed on opencyclemap.org.
  auto way = GetWay(6885577, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.moped_forward() != true || way.bike_forward() != false || way.pedestrian() != true ||
      way.auto_backward() != true || way.bus_backward() != true || way.moped_backward() != true || way.bike_backward() != true) {
    throw std::runtime_error("Access is not correct for way 6885577.");
  }

  way = GetWay(156539494, ways);
  if (way.auto_forward() != false || way.bus_forward() != false || way.moped_forward() != false || way.bike_forward() != true || way.pedestrian() != false ||
      way.auto_backward() != false || way.bus_backward() != false || way.moped_backward() != false || way.bike_backward() != false) {
    throw std::runtime_error("Access is not correct for way 156539494.");
  }

  way = GetWay(6885404, ways);
  if (way.auto_forward() != false || way.bus_forward() != false || way.moped_forward() != false || way.bike_forward() != true || way.pedestrian() != false ||
      way.auto_backward() != false || way.bus_backward() != false || way.moped_backward() != false || way.bike_backward() != true) {
    throw std::runtime_error("Access is not correct for way 6885404.");
  }

  way = GetWay(156539492, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.moped_forward() != true || way.bike_forward() != false || way.pedestrian() != true ||
      way.auto_backward() != true || way.bus_backward() != true || way.moped_backward() != true || way.bike_backward() != false) {
    throw std::runtime_error("Access is not correct for way 156539492.");
  }

  way = GetWay(156539491, ways);
  if (way.auto_forward() != true || way.bus_forward() != true || way.moped_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != true || way.bus_backward() != true || way.moped_forward() != true || way.bike_backward() != true) {
    throw std::runtime_error("Access is not correct for way 156539491.");
  }

  boost::filesystem::remove(ways_file);
  boost::filesystem::remove(way_nodes_file);
  boost::filesystem::remove(access_file);
}

void Bus(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string access_file = "test_access.bin";
  std::string restriction_file = "test_complex_restrictions.bin";

  auto osmdata = PBFGraphParser::Parse(conf.get_child("mjolnir"), {"test/data/bus.osm.pbf"},
                                       ways_file, way_nodes_file, access_file, restriction_file);
  sequence<OSMWay> ways(ways_file, false);
  ways.sort(way_predicate);

  auto way = GetWay(14327599, ways);
  if (way.auto_forward() != false || way.moped_forward() != false || way.bus_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
      way.auto_backward() != false || way.moped_backward() != false || way.bus_backward() != false || way.bike_backward() != true) {
      throw std::runtime_error("Access is not correct for way 14327599.");
  }

  way = GetWay(87358588, ways);
  if (way.auto_forward() != false || way.moped_forward() != false || way.bus_forward() != false || way.bike_forward() != true || way.pedestrian() != true ||
       way.auto_backward() != false || way.moped_backward() != false || way.bus_backward() != false || way.bike_backward() != true) {
       throw std::runtime_error("Access is not correct for way 87358588.");
   }

  way = GetWay(49771553, ways);
  if (way.auto_forward() != true || way.moped_forward() != true || way.bus_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
       way.auto_backward() != true || way.moped_backward() != true || way.bus_backward() != true || way.bike_backward() != true) {
       throw std::runtime_error("Access is not correct for way 49771553.");
   }

  way = GetWay(225895737, ways);
  if (way.auto_forward() != true || way.moped_forward() != true || way.bus_forward() != true || way.bike_forward() != true || way.pedestrian() != true ||
       way.auto_backward() != false || way.moped_backward() != false || way.bus_backward() != false || way.bike_backward() != false) {
       throw std::runtime_error("Access is not correct for way 225895737.");
   }

  boost::filesystem::remove(ways_file);
  boost::filesystem::remove(way_nodes_file);
  boost::filesystem::remove(access_file);
}

void BicycleTrafficSignals(const std::string& config_file) {
  boost::property_tree::ptree conf;
  boost::property_tree::json_parser::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string access_file = "test_access.bin";
  std::string restriction_file = "test_complex_restrictions.bin";

  auto osmdata = PBFGraphParser::Parse(conf.get_child("mjolnir"), {"test/data/nyc.osm.pbf"},
                                       ways_file, way_nodes_file, access_file, restriction_file);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  way_nodes.sort(node_predicate);

  auto node = GetNode(42439096, way_nodes);
  if (!node.intersection() || !node.traffic_signal())
    throw std::runtime_error("Traffic Signal test failed.");
/*
  //When we support finding bike rentals, this test will need updated.
  node = GetNode(3146484929, way_nodes);
  if (node.intersection())
    throw std::runtime_error("Bike rental not marked as intersection.");

  //When we support finding shops that rent bikes, this test will need updated.
  node = GetNode(2592264881, way_nodes);
  if (node.intersection())
    throw std::runtime_error("Bike rental at a shop not marked as intersection.");
*/

  boost::filesystem::remove(ways_file);
  boost::filesystem::remove(way_nodes_file);
  boost::filesystem::remove(access_file);

}

void DoConfig() {
  std::ofstream file;
  try {
    file.open(config_file, std::ios_base::trunc);
    file << "{ \
      \"mjolnir\": { \
      \"tile_dir\": \"test/data/parser_tiles\" \
      } \
    }";
  }
  catch(...) {

  }
  file.close();
}

void TestBollardsGatesAndAccess() {
  //write the tiles with it
  BollardsGatesAndAccess(config_file);
}

void TestRemovableBollards() {
  //write the tiles with it
  RemovableBollards(config_file);
}

void TestBicycleTrafficSignals() {
  //write the tiles with it
  BicycleTrafficSignals(config_file);
}

void TestExits() {
  //write the tiles with it
  Exits(config_file);
}

void TestBaltimoreArea() {
  //write the tiles with it
  Baltimore(config_file);
}

void TestBike() {
  //write the tiles with it
  Bike(config_file);
}

void TestBus() {
  //write the tiles with it
  Bus(config_file);
}

}

int main() {

  // Test data BBs are as follows:
  // Rome:        <bounds minlat="41.8957000" minlon="12.4820400" maxlat="41.8973400" maxlon="12.4855600"/>
  // NYC:         <bounds minlat="40.7330200" minlon="-74.0136900" maxlat="40.7396900" maxlon="-73.9996000"/>
  // Baltimore:   <bounds minlat="39.2586000" minlon="-76.6081000" maxlat="39.3065000" maxlon="-76.5288000"/>
  // Harrisburg:  <bounds minlat="40.2075000" minlon="-76.8459000" maxlat="40.3136000" maxlon="-76.7474000"/>
  // Liechtenstein: None

  test::suite suite("parser");

  suite.test(TEST_CASE(DoConfig));
  suite.test(TEST_CASE(TestBollardsGatesAndAccess));
  suite.test(TEST_CASE(TestRemovableBollards));
  suite.test(TEST_CASE(TestBicycleTrafficSignals));
  suite.test(TEST_CASE(TestExits));
  suite.test(TEST_CASE(TestBaltimoreArea));
  suite.test(TEST_CASE(TestBike));
  suite.test(TEST_CASE(TestBus));

  return suite.tear_down();
}
