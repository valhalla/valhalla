#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/sequence.h"
#include "mjolnir/bssbuilder.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/osmnode.h"
#include "mjolnir/pbfgraphparser.h"
#include "test.h"

#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <fstream>

#include "baldr/directededge.h"
#include "baldr/graphconstants.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla::midgard;
using namespace valhalla::mjolnir;
using namespace valhalla::baldr;

namespace {

const std::string config_file = "test/test_config_gp";

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

void BollardsGatesAndAccess(const std::string& config_file) {
  boost::property_tree::ptree conf;
  rapidjson::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string access_file = "test_access.bin";
  std::string from_restriction_file = "test_from_complex_restrictions.bin";
  std::string to_restriction_file = "test_to_complex_restrictions.bin";
  std::string bss_nodes_file = "test_bss_nodes.bin";

  auto osmdata =
      PBFGraphParser::ParseWays(conf.get_child("mjolnir"),
                                {VALHALLA_SOURCE_DIR "test/data/liechtenstein-latest.osm.pbf"},
                                ways_file, way_nodes_file, access_file);

  PBFGraphParser::ParseRelations(conf.get_child("mjolnir"),
                                 {VALHALLA_SOURCE_DIR "test/data/liechtenstein-latest.osm.pbf"},
                                 from_restriction_file, to_restriction_file, osmdata);

  PBFGraphParser::ParseNodes(conf.get_child("mjolnir"),
                             {VALHALLA_SOURCE_DIR "test/data/liechtenstein-latest.osm.pbf"},
                             way_nodes_file, bss_nodes_file, osmdata);

  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  way_nodes.sort(node_predicate);

  sequence<OSMWay> ways(ways_file, false);
  ways.sort(way_predicate);

  // bus access tests.
  auto way_85744121 = GetWay(85744121, ways);
  EXPECT_FALSE(way_85744121.auto_forward());
  EXPECT_FALSE(way_85744121.moped_forward());
  EXPECT_TRUE(way_85744121.bike_forward());
  EXPECT_TRUE(way_85744121.bus_forward());
  EXPECT_TRUE(way_85744121.pedestrian_forward());
  EXPECT_TRUE(way_85744121.pedestrian_backward());
  EXPECT_FALSE(way_85744121.auto_backward());
  EXPECT_FALSE(way_85744121.moped_backward());
  EXPECT_FALSE(way_85744121.bike_backward());
  EXPECT_FALSE(way_85744121.bus_backward());

  auto way_86260080 = GetWay(86260080, ways);
  EXPECT_TRUE(way_86260080.auto_forward());
  EXPECT_TRUE(way_86260080.bike_forward());
  EXPECT_TRUE(way_86260080.bus_forward());
  EXPECT_TRUE(way_86260080.moped_forward());
  EXPECT_TRUE(way_86260080.pedestrian_forward());
  EXPECT_TRUE(way_86260080.pedestrian_backward());
  EXPECT_TRUE(way_86260080.auto_backward());
  EXPECT_TRUE(way_86260080.bike_backward());
  EXPECT_TRUE(way_86260080.bus_backward());
  EXPECT_TRUE(way_86260080.moped_backward());

  auto way_161683833 = GetWay(161683833, ways);
  EXPECT_TRUE(way_161683833.auto_forward());
  EXPECT_TRUE(way_161683833.bike_forward());
  EXPECT_TRUE(way_161683833.bus_forward());
  EXPECT_TRUE(way_161683833.moped_forward());
  EXPECT_TRUE(way_161683833.pedestrian_forward());
  EXPECT_TRUE(way_161683833.pedestrian_backward());
  EXPECT_TRUE(way_161683833.auto_backward());
  EXPECT_TRUE(way_161683833.bike_backward());
  EXPECT_TRUE(way_161683833.bus_backward());
  EXPECT_TRUE(way_161683833.moped_backward());

  // We split set the uses at bollards and gates.
  auto node = GetNode(392700757, way_nodes);
  EXPECT_TRUE(node.intersection()) << "Bollard not marked as intersection.";

  // We split set the uses at bollards and gates.
  node = GetNode(376947468, way_nodes);
  EXPECT_TRUE(node.intersection()) << "Gate not marked as intersection.";

  // Is a gate with foot and bike flags set; however, access is private.
  // Gate at the end of way
  node = GetNode(2949666866, way_nodes);
  EXPECT_TRUE(node.intersection());
  EXPECT_EQ(node.type(), NodeType::kGate);
  EXPECT_EQ(node.access(), (kAutoAccess | kHOVAccess | kTaxiAccess | kTruckAccess | kBusAccess |
                            kEmergencyAccess | kPedestrianAccess | kWheelchairAccess |
                            kBicycleAccess | kMopedAccess | kMotorcycleAccess));

  // block
  node = GetNode(1819036441, way_nodes);
  EXPECT_TRUE(node.intersection());
  EXPECT_EQ(node.type(), NodeType::kBollard);
  EXPECT_EQ(node.access(), kPedestrianAccess | kWheelchairAccess | kBicycleAccess);

  // border control
  node = GetNode(3256854624, way_nodes);
  EXPECT_TRUE(node.intersection());
  EXPECT_EQ(node.type(), NodeType::kBorderControl);
  EXPECT_EQ(node.access(), kAutoAccess | kHOVAccess | kTaxiAccess | kTruckAccess | kBusAccess |
                               kEmergencyAccess | kPedestrianAccess | kWheelchairAccess |
                               kBicycleAccess | kMopedAccess | kMotorcycleAccess);

  // has bike tag but all should have access
  // Bike access only test
  node = GetNode(696222071, way_nodes);
  EXPECT_TRUE(node.intersection());
  EXPECT_EQ(node.access(), kAutoAccess | kHOVAccess | kTaxiAccess | kTruckAccess | kBusAccess |
                               kEmergencyAccess | kPedestrianAccess | kWheelchairAccess |
                               kBicycleAccess | kMopedAccess | kMotorcycleAccess);

  // Is a bollard with no flags set.
  node = GetNode(569645326, way_nodes);
  EXPECT_TRUE(node.intersection());
  EXPECT_EQ(node.type(), NodeType::kBollard);
  EXPECT_EQ(node.access(), kPedestrianAccess | kWheelchairAccess | kBicycleAccess);

  // Is a bollard=block with foot flag set.
  node = GetNode(1819036441, way_nodes);
  EXPECT_TRUE(node.intersection());
  EXPECT_EQ(node.type(), NodeType::kBollard);
  EXPECT_EQ(node.access(), kPedestrianAccess | kWheelchairAccess | kBicycleAccess);

  auto bike = osmdata.bike_relations.equal_range(25452580);
  auto way_25452580 = GetWay(25452580, ways);
  uint32_t bike_network = 0;

  for (auto b = bike.first; b != bike.second; ++b)
    bike_network |= b->second.bike_network;

  EXPECT_EQ(bike_network, kRcn) << "rcn not marked on way 25452580.";
  EXPECT_EQ(way_25452580.bike_network(), 0) << "rcn not marked on way 25452580.";

  auto way_74584853 = GetWay(74584853, ways);
  bike_network = 0;
  bike = osmdata.bike_relations.equal_range(74584853);

  for (auto b = bike.first; b != bike.second; ++b)
    // mountain bike networks have local, regional, and national networks set too.
    if (b->second.bike_network & kMcn)
      bike_network |= kMcn;
    else
      bike_network |= b->second.bike_network;

  EXPECT_TRUE((bike_network & kMcn) && (bike_network & kLcn) && way_74584853.bike_network() == 0)
      << "lcn and mtb not marked on way 74584853.";

  auto way_75786176 = GetWay(75786176, ways);
  bike_network = 0;
  bike = osmdata.bike_relations.equal_range(75786176);

  for (auto b = bike.first; b != bike.second; ++b) {
    // mountain bike networks have local, regional, and national networks set too.
    if (b->second.bike_network & kMcn)
      bike_network |= kMcn;
    else
      bike_network |= b->second.bike_network;
  }

  EXPECT_TRUE((bike_network & kMcn) && (bike_network & kRcn) && way_75786176.bike_network() == 0)
      << "rcn and mtb not marked on way 75786176.";

  filesystem::remove(ways_file);
  filesystem::remove(way_nodes_file);
  filesystem::remove(access_file);
  filesystem::remove(from_restriction_file);
  filesystem::remove(to_restriction_file);
  filesystem::remove(bss_nodes_file);
}

void RemovableBollards(const std::string& config_file) {
  boost::property_tree::ptree conf;
  rapidjson::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string access_file = "test_access.bin";
  std::string from_restriction_file = "test_from_complex_restrictions.bin";
  std::string to_restriction_file = "test_to_complex_restrictions.bin";
  std::string bss_nodes_file = "test_bss_nodes.bin";

  auto osmdata = PBFGraphParser::ParseWays(conf.get_child("mjolnir"),
                                           {VALHALLA_SOURCE_DIR "test/data/rome.osm.pbf"}, ways_file,
                                           way_nodes_file, access_file);

  PBFGraphParser::ParseRelations(conf.get_child("mjolnir"),
                                 {VALHALLA_SOURCE_DIR "test/data/rome.osm.pbf"},
                                 from_restriction_file, to_restriction_file, osmdata);

  PBFGraphParser::ParseNodes(conf.get_child("mjolnir"),
                             {VALHALLA_SOURCE_DIR "test/data/rome.osm.pbf"}, way_nodes_file,
                             bss_nodes_file, osmdata);

  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  way_nodes.sort(node_predicate);

  // Is a bollard=rising is saved as a gate...with foot flag and bike set.
  auto node = GetNode(2425784125, way_nodes);
  EXPECT_TRUE(node.intersection()) << "Rising Bollard not marked as intersection.";
  EXPECT_EQ(node.type(), NodeType::kGate);
  EXPECT_EQ(node.access(), kAutoAccess | kHOVAccess | kTaxiAccess | kTruckAccess | kBusAccess |
                               kEmergencyAccess | kPedestrianAccess | kWheelchairAccess |
                               kBicycleAccess | kMopedAccess | kMotorcycleAccess);

  filesystem::remove(ways_file);
  filesystem::remove(way_nodes_file);
  filesystem::remove(access_file);
  filesystem::remove(from_restriction_file);
  filesystem::remove(to_restriction_file);
  filesystem::remove(bss_nodes_file);
}

void Exits(const std::string& config_file) {
  boost::property_tree::ptree conf;
  rapidjson::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string access_file = "test_access.bin";
  std::string from_restriction_file = "test_from_complex_restrictions.bin";
  std::string to_restriction_file = "test_to_complex_restrictions.bin";
  std::string bss_nodes_file = "test_bss_nodes.bin";

  auto osmdata = PBFGraphParser::ParseWays(conf.get_child("mjolnir"),
                                           {VALHALLA_SOURCE_DIR "test/data/harrisburg.osm.pbf"},
                                           ways_file, way_nodes_file, access_file);

  PBFGraphParser::ParseRelations(conf.get_child("mjolnir"),
                                 {VALHALLA_SOURCE_DIR "test/data/harrisburg.osm.pbf"},
                                 from_restriction_file, to_restriction_file, osmdata);

  PBFGraphParser::ParseNodes(conf.get_child("mjolnir"),
                             {VALHALLA_SOURCE_DIR "test/data/harrisburg.osm.pbf"}, way_nodes_file,
                             bss_nodes_file, osmdata);

  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  way_nodes.sort(node_predicate);

  auto node = GetNode(33698177, way_nodes);
  EXPECT_TRUE(node.intersection());
  EXPECT_TRUE(node.has_ref());
  EXPECT_EQ(osmdata.node_names.name(node.ref_index()), "51A-B") << "Ref not set correctly .";

  node = GetNode(1901353894, way_nodes);
  EXPECT_TRUE(node.intersection());
  EXPECT_TRUE(node.has_ref());
  EXPECT_EQ(osmdata.node_names.name(node.name_index()), "Harrisburg East")
      << "node name not set correctly .";

  node = GetNode(462240654, way_nodes);
  EXPECT_TRUE(node.intersection());
  EXPECT_EQ(osmdata.node_names.name(node.exit_to_index()), "PA441")
      << "node exit_to not set correctly .";

  filesystem::remove(ways_file);
  filesystem::remove(way_nodes_file);
  filesystem::remove(access_file);
  filesystem::remove(from_restriction_file);
  filesystem::remove(to_restriction_file);
  filesystem::remove(bss_nodes_file);
}

void Baltimore(const std::string& config_file) {
  boost::property_tree::ptree conf;
  rapidjson::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string access_file = "test_access.bin";
  std::string from_restriction_file = "test_from_complex_restrictions.bin";
  std::string to_restriction_file = "test_to_complex_restrictions.bin";
  std::string bss_nodes_file = "test_bss_nodes.bin";

  auto osmdata = PBFGraphParser::ParseWays(conf.get_child("mjolnir"),
                                           {VALHALLA_SOURCE_DIR "test/data/baltimore.osm.pbf"},
                                           ways_file, way_nodes_file, access_file);

  PBFGraphParser::ParseRelations(conf.get_child("mjolnir"),
                                 {VALHALLA_SOURCE_DIR "test/data/baltimore.osm.pbf"},
                                 from_restriction_file, to_restriction_file, osmdata);

  PBFGraphParser::ParseNodes(conf.get_child("mjolnir"),
                             {VALHALLA_SOURCE_DIR "test/data/baltimore.osm.pbf"}, way_nodes_file,
                             bss_nodes_file, osmdata);

  sequence<OSMWay> ways(ways_file, false);
  ways.sort(way_predicate);

  // bike_forward and reverse is set to false by default.  Meaning defaults for
  // highway = pedestrian.  Bike overrides bicycle=designated and/or cycleway=shared_lane
  // make it bike_forward and reverse = true
  auto way_216240466 = GetWay(216240466, ways);
  EXPECT_FALSE(way_216240466.auto_forward());
  EXPECT_FALSE(way_216240466.bus_forward());
  EXPECT_FALSE(way_216240466.moped_forward());
  EXPECT_TRUE(way_216240466.bike_forward());
  EXPECT_TRUE(way_216240466.pedestrian_forward());
  EXPECT_TRUE(way_216240466.pedestrian_backward());
  EXPECT_FALSE(way_216240466.auto_backward());
  EXPECT_FALSE(way_216240466.bus_backward());
  EXPECT_FALSE(way_216240466.moped_backward());
  EXPECT_TRUE(way_216240466.bike_backward());

  // access for all
  auto way_138388359 = GetWay(138388359, ways);
  EXPECT_TRUE(way_138388359.auto_forward());
  EXPECT_TRUE(way_138388359.bus_forward());
  EXPECT_TRUE(way_138388359.moped_forward());
  EXPECT_TRUE(way_138388359.bike_forward());
  EXPECT_TRUE(way_138388359.pedestrian_forward());
  EXPECT_TRUE(way_138388359.pedestrian_backward());
  EXPECT_TRUE(way_138388359.auto_backward());
  EXPECT_TRUE(way_138388359.bus_backward());
  EXPECT_TRUE(way_138388359.moped_backward());
  EXPECT_TRUE(way_138388359.bike_backward());

  // footway...pedestrian only
  auto way_133689121 = GetWay(133689121, ways);
  EXPECT_FALSE(way_133689121.auto_forward());
  EXPECT_FALSE(way_133689121.bus_forward());
  EXPECT_FALSE(way_133689121.moped_forward());
  EXPECT_FALSE(way_133689121.bike_forward());
  EXPECT_TRUE(way_133689121.pedestrian_forward());
  EXPECT_TRUE(way_133689121.pedestrian_backward());
  EXPECT_FALSE(way_133689121.auto_backward());
  EXPECT_FALSE(way_133689121.bus_backward());
  EXPECT_FALSE(way_133689121.moped_backward());
  EXPECT_FALSE(way_133689121.bike_backward());

  // oneway
  auto way_49641455 = GetWay(49641455, ways);
  EXPECT_TRUE(way_49641455.auto_forward());
  EXPECT_TRUE(way_49641455.bus_forward());
  EXPECT_TRUE(way_49641455.moped_forward());
  EXPECT_TRUE(way_49641455.bike_forward());
  EXPECT_TRUE(way_49641455.pedestrian_forward());
  EXPECT_TRUE(way_49641455.pedestrian_backward());
  EXPECT_FALSE(way_49641455.auto_backward());
  EXPECT_FALSE(way_49641455.bus_backward());
  EXPECT_FALSE(way_49641455.moped_backward());
  EXPECT_FALSE(way_49641455.bike_backward());

  // Oneway test.  Make sure auto backward is set for ways where oneway=no.
  // Check Forward/Backward/Pedestrian access is set correctly for way 192573108.
  auto way_192573108 = GetWay(192573108, ways);
  EXPECT_TRUE(way_192573108.auto_forward());
  EXPECT_TRUE(way_192573108.bus_forward());
  EXPECT_TRUE(way_192573108.moped_forward());
  EXPECT_TRUE(way_192573108.bike_forward());
  EXPECT_TRUE(way_192573108.pedestrian_forward());
  EXPECT_TRUE(way_192573108.pedestrian_backward());
  EXPECT_TRUE(way_192573108.auto_backward());
  EXPECT_TRUE(way_192573108.bus_backward());
  EXPECT_TRUE(way_192573108.moped_backward());
  EXPECT_TRUE(way_192573108.bike_backward());

  sequence<OSMWayNode> way_nodes(way_nodes_file, false, true);
  way_nodes.sort(node_predicate);
  auto node = GetNode(49473254, way_nodes);

  EXPECT_TRUE(node.intersection()) << "Toll Booth 49473254";
  EXPECT_EQ(node.type(), NodeType::kTollBooth) << "Toll Booth 49473254";

  auto res = osmdata.restrictions.equal_range(98040438);

  ASSERT_NE(res.first, osmdata.restrictions.end()) << "Failed to find 98040438 restriction.";

  for (auto r = res.first; r != res.second; ++r) {
    if (r->second.to() == 6003340) {
      EXPECT_EQ(r->second.via(), 2123388822) << "98040438 restriction test failed for to: 6003340";
      EXPECT_EQ(r->second.type(), RestrictionType::kNoLeftTurn)
          << "98040438 restriction test failed for to: 6003340";
    } else if (r->second.to() == 98040438) {
      EXPECT_EQ(r->second.via(), 2123388822) << "98040438 restriction test failed for to: 98040438";
      EXPECT_EQ(r->second.type(), RestrictionType::kNoUTurn)
          << "98040438 restriction test failed for to: 98040438";
    } else
      FAIL() << "98040438 restriction test failed.";
  }

  filesystem::remove(ways_file);
  filesystem::remove(way_nodes_file);
  filesystem::remove(access_file);
  filesystem::remove(from_restriction_file);
  filesystem::remove(to_restriction_file);
  filesystem::remove(bss_nodes_file);
}

void Bike(const std::string& config_file) {
  boost::property_tree::ptree conf;
  rapidjson::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string access_file = "test_access.bin";
  std::string from_restriction_file = "test_from_complex_restrictions.bin";
  std::string to_restriction_file = "test_to_complex_restrictions.bin";
  std::string bss_nodes_file = "test_bss_nodes.bin";

  auto osmdata = PBFGraphParser::ParseWays(conf.get_child("mjolnir"),
                                           {VALHALLA_SOURCE_DIR "test/data/bike.osm.pbf"}, ways_file,
                                           way_nodes_file, access_file);

  PBFGraphParser::ParseRelations(conf.get_child("mjolnir"),
                                 {VALHALLA_SOURCE_DIR "test/data/bike.osm.pbf"},
                                 from_restriction_file, to_restriction_file, osmdata);

  PBFGraphParser::ParseNodes(conf.get_child("mjolnir"),
                             {VALHALLA_SOURCE_DIR "test/data/bike.osm.pbf"}, way_nodes_file,
                             bss_nodes_file, osmdata);

  sequence<OSMWay> ways(ways_file, false);
  ways.sort(way_predicate);

  // http://www.openstreetmap.org/way/6885577#map=14/51.9774/5.7718
  // direction of this way for oneway is flipped.  Confirmed on opencyclemap.org.
  auto way_6885577 = GetWay(6885577, ways);
  EXPECT_TRUE(way_6885577.auto_forward());
  EXPECT_TRUE(way_6885577.bus_forward());
  EXPECT_TRUE(way_6885577.moped_forward());
  EXPECT_FALSE(way_6885577.bike_forward());
  EXPECT_TRUE(way_6885577.pedestrian_forward());
  EXPECT_TRUE(way_6885577.pedestrian_backward());
  EXPECT_TRUE(way_6885577.auto_backward());
  EXPECT_TRUE(way_6885577.bus_backward());
  EXPECT_TRUE(way_6885577.moped_backward());
  EXPECT_TRUE(way_6885577.bike_backward());

  auto way_156539494 = GetWay(156539494, ways);
  EXPECT_FALSE(way_156539494.auto_forward());
  EXPECT_FALSE(way_156539494.bus_forward());
  EXPECT_FALSE(way_156539494.moped_forward());
  EXPECT_TRUE(way_156539494.bike_forward());
  EXPECT_FALSE(way_156539494.pedestrian_forward());
  EXPECT_FALSE(way_156539494.pedestrian_backward());
  EXPECT_FALSE(way_156539494.auto_backward());
  EXPECT_FALSE(way_156539494.bus_backward());
  EXPECT_FALSE(way_156539494.moped_backward());
  EXPECT_FALSE(way_156539494.bike_backward());

  auto way_6885404 = GetWay(6885404, ways);
  EXPECT_FALSE(way_6885404.auto_forward());
  EXPECT_FALSE(way_6885404.bus_forward());
  EXPECT_FALSE(way_6885404.moped_forward());
  EXPECT_TRUE(way_6885404.bike_forward());
  EXPECT_FALSE(way_6885404.pedestrian_forward());
  EXPECT_FALSE(way_6885404.pedestrian_backward());
  EXPECT_FALSE(way_6885404.auto_backward());
  EXPECT_FALSE(way_6885404.bus_backward());
  EXPECT_FALSE(way_6885404.moped_backward());
  EXPECT_TRUE(way_6885404.bike_backward());

  auto way_156539492 = GetWay(156539492, ways);
  EXPECT_TRUE(way_156539492.auto_forward());
  EXPECT_TRUE(way_156539492.bus_forward());
  EXPECT_TRUE(way_156539492.moped_forward());
  EXPECT_FALSE(way_156539492.bike_forward());
  EXPECT_TRUE(way_156539492.pedestrian_forward());
  EXPECT_TRUE(way_156539492.pedestrian_backward());
  EXPECT_TRUE(way_156539492.auto_backward());
  EXPECT_TRUE(way_156539492.bus_backward());
  EXPECT_TRUE(way_156539492.moped_backward());
  EXPECT_FALSE(way_156539492.bike_backward());

  auto way_156539491 = GetWay(156539491, ways);
  EXPECT_TRUE(way_156539491.auto_forward());
  EXPECT_TRUE(way_156539491.bus_forward());
  EXPECT_TRUE(way_156539491.moped_forward());
  EXPECT_TRUE(way_156539491.bike_forward());
  EXPECT_TRUE(way_156539492.pedestrian_forward());
  EXPECT_TRUE(way_156539492.pedestrian_backward());
  EXPECT_TRUE(way_156539491.auto_backward());
  EXPECT_TRUE(way_156539491.bus_backward());
  EXPECT_TRUE(way_156539491.moped_forward());
  EXPECT_TRUE(way_156539491.bike_backward());

  filesystem::remove(ways_file);
  filesystem::remove(way_nodes_file);
  filesystem::remove(access_file);
  filesystem::remove(from_restriction_file);
  filesystem::remove(to_restriction_file);
}

void Bus(const std::string& config_file) {
  boost::property_tree::ptree conf;
  rapidjson::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string access_file = "test_access.bin";
  std::string from_restriction_file = "test_from_complex_restrictions.bin";
  std::string to_restriction_file = "test_to_complex_restrictions.bin";
  std::string bss_nodes_file = "test_bss_nodes.bin";

  auto osmdata = PBFGraphParser::ParseWays(conf.get_child("mjolnir"),
                                           {VALHALLA_SOURCE_DIR "test/data/bus.osm.pbf"}, ways_file,
                                           way_nodes_file, access_file);

  PBFGraphParser::ParseRelations(conf.get_child("mjolnir"),
                                 {VALHALLA_SOURCE_DIR "test/data/bus.osm.pbf"}, from_restriction_file,
                                 to_restriction_file, osmdata);

  PBFGraphParser::ParseNodes(conf.get_child("mjolnir"), {VALHALLA_SOURCE_DIR "test/data/bus.osm.pbf"},
                             way_nodes_file, bss_nodes_file, osmdata);

  sequence<OSMWay> ways(ways_file, false);
  ways.sort(way_predicate);

  auto way_14327599 = GetWay(14327599, ways);
  EXPECT_FALSE(way_14327599.auto_forward());
  EXPECT_FALSE(way_14327599.moped_forward());
  EXPECT_TRUE(way_14327599.bus_forward());
  EXPECT_TRUE(way_14327599.bike_forward());
  EXPECT_TRUE(way_14327599.pedestrian_forward());
  EXPECT_TRUE(way_14327599.pedestrian_backward());
  EXPECT_FALSE(way_14327599.auto_backward());
  EXPECT_FALSE(way_14327599.moped_backward());
  EXPECT_FALSE(way_14327599.bus_backward());
  EXPECT_TRUE(way_14327599.bike_backward());

  auto way_87358588 = GetWay(87358588, ways);
  EXPECT_FALSE(way_87358588.auto_forward());
  EXPECT_FALSE(way_87358588.moped_forward());
  EXPECT_FALSE(way_87358588.bus_forward());
  EXPECT_TRUE(way_87358588.bike_forward());
  EXPECT_TRUE(way_87358588.pedestrian_forward());
  EXPECT_TRUE(way_87358588.pedestrian_backward());
  EXPECT_FALSE(way_87358588.auto_backward());
  EXPECT_FALSE(way_87358588.moped_backward());
  EXPECT_FALSE(way_87358588.bus_backward());
  EXPECT_TRUE(way_87358588.bike_backward());

  auto way_49771553 = GetWay(49771553, ways);
  EXPECT_TRUE(way_49771553.auto_forward());
  EXPECT_TRUE(way_49771553.moped_forward());
  EXPECT_TRUE(way_49771553.bus_forward());
  EXPECT_TRUE(way_49771553.bike_forward());
  EXPECT_TRUE(way_49771553.pedestrian_forward());
  EXPECT_TRUE(way_49771553.pedestrian_backward());
  EXPECT_TRUE(way_49771553.auto_backward());
  EXPECT_TRUE(way_49771553.moped_backward());
  EXPECT_TRUE(way_49771553.bus_backward());
  EXPECT_TRUE(way_49771553.bike_backward());

  auto way_225895737 = GetWay(225895737, ways);
  EXPECT_TRUE(way_225895737.auto_forward());
  EXPECT_TRUE(way_225895737.moped_forward());
  EXPECT_TRUE(way_225895737.bus_forward());
  EXPECT_TRUE(way_225895737.bike_forward());
  EXPECT_TRUE(way_225895737.pedestrian_forward());
  EXPECT_TRUE(way_225895737.pedestrian_backward());
  EXPECT_FALSE(way_225895737.auto_backward());
  EXPECT_FALSE(way_225895737.moped_backward());
  EXPECT_FALSE(way_225895737.bus_backward());
  EXPECT_FALSE(way_225895737.bike_backward());

  filesystem::remove(ways_file);
  filesystem::remove(way_nodes_file);
  filesystem::remove(access_file);
  filesystem::remove(from_restriction_file);
  filesystem::remove(to_restriction_file);
  filesystem::remove(bss_nodes_file);
}

void BicycleTrafficSignals(const std::string& config_file) {
  boost::property_tree::ptree conf;
  rapidjson::read_json(config_file, conf);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string access_file = "test_access.bin";
  std::string from_restriction_file = "test_from_complex_restrictions.bin";
  std::string to_restriction_file = "test_to_complex_restrictions.bin";
  std::string bss_nodes_file = "test_bss_nodes.bin";

  auto osmdata = PBFGraphParser::ParseWays(conf.get_child("mjolnir"),
                                           {VALHALLA_SOURCE_DIR "test/data/nyc.osm.pbf"}, ways_file,
                                           way_nodes_file, access_file);

  PBFGraphParser::ParseRelations(conf.get_child("mjolnir"),
                                 {VALHALLA_SOURCE_DIR "test/data/nyc.osm.pbf"}, from_restriction_file,
                                 to_restriction_file, osmdata);

  PBFGraphParser::ParseNodes(conf.get_child("mjolnir"), {VALHALLA_SOURCE_DIR "test/data/nyc.osm.pbf"},
                             way_nodes_file, bss_nodes_file, osmdata);

  sequence<OSMWayNode> way_nodes(way_nodes_file, false);
  way_nodes.sort(node_predicate);

  auto node = GetNode(42439096, way_nodes);
  EXPECT_TRUE(node.intersection());
  EXPECT_TRUE(node.traffic_signal());

  /*
    //When we support finding bike rentals, this test will need updated.
    node = GetNode(3146484929, way_nodes);
    EXPECT_FALSE(node.intersection())
      << "Bike rental not marked as intersection.";

    //When we support finding shops that rent bikes, this test will need updated.
    node = GetNode(2592264881, way_nodes);
    EXPECT_FALSE(node.intersection())
      << "Bike rental at a shop not marked as intersection."
  */

  filesystem::remove(ways_file);
  filesystem::remove(way_nodes_file);
  filesystem::remove(access_file);
  filesystem::remove(from_restriction_file);
  filesystem::remove(to_restriction_file);
  filesystem::remove(bss_nodes_file);
}

void DoConfig() {
  std::ofstream file;
  try {
    file.open(config_file, std::ios_base::trunc);
    file << "{ \
      \"mjolnir\": { \
      \"id_table_size\": 1000, \
      \"tile_dir\": \"test/data/parser_tiles\" \
      } \
    }";
  } catch (...) {}
  file.close();
}

TEST(GraphParser, TestBollardsGatesAndAccess) {
  // write the tiles with it
  BollardsGatesAndAccess(config_file);
}

TEST(GraphParser, TestRemovableBollards) {
  // write the tiles with it
  RemovableBollards(config_file);
}

TEST(GraphParser, TestBicycleTrafficSignals) {
  // write the tiles with it
  BicycleTrafficSignals(config_file);
}

TEST(GraphParser, TestExits) {
  // write the tiles with it
  Exits(config_file);
}

TEST(GraphParser, TestBaltimoreArea) {
  // write the tiles with it
  Baltimore(config_file);
}

TEST(GraphParser, TestBike) {
  // write the tiles with it
  Bike(config_file);
}

TEST(GraphParser, TestBus) {
  // write the tiles with it
  Bus(config_file);
}

TEST(GraphParser, TestImportBssNode) {

  boost::property_tree::ptree conf;
  rapidjson::read_json(config_file, conf);

  conf.put("mjolnir.import_bike_share_stations", true);

  std::string ways_file = "test_ways.bin";
  std::string way_nodes_file = "test_way_nodes.bin";
  std::string nodes_file = "test_nodes.bin";
  std::string edges_file = "test_edges.bin";
  std::string access_file = "test_access.bin";
  std::string from_restriction_file = "test_from_complex_restrictions.bin";
  std::string to_restriction_file = "test_to_complex_restrictions.bin";
  std::string bss_nodes_file = "test_bss_nodes.bin";

  auto osmdata = PBFGraphParser::ParseWays(conf.get_child("mjolnir"),
                                           {VALHALLA_SOURCE_DIR "test/data/rome.osm.pbf"}, ways_file,
                                           way_nodes_file, access_file);

  PBFGraphParser::ParseRelations(conf.get_child("mjolnir"),
                                 {VALHALLA_SOURCE_DIR "test/data/rome.osm.pbf"},
                                 from_restriction_file, to_restriction_file, osmdata);

  PBFGraphParser::ParseNodes(conf.get_child("mjolnir"),
                             {VALHALLA_SOURCE_DIR "test/data/rome.osm.pbf"}, way_nodes_file,
                             bss_nodes_file, osmdata);

  GraphReader reader(conf.get_child("mjolnir"));

  std::map<valhalla::baldr::GraphId, size_t> tiles =
      GraphBuilder::BuildEdges(conf.get_child("mjolnir"), ways_file, way_nodes_file, nodes_file,
                               edges_file);

  GraphBuilder::Build(conf, osmdata, ways_file, way_nodes_file, nodes_file, edges_file,
                      from_restriction_file, to_restriction_file, tiles);

  BssBuilder::Build(conf, bss_nodes_file);

  auto local_level = TileHierarchy::levels().back().level;

  graph_tile_ptr local_tile = reader.GetGraphTile({759649, local_level, 0});
  auto count = local_tile->header()->nodecount();

  EXPECT_EQ(local_tile->node(count - 1)->type(), NodeType::kBikeShare)
      << "The added node is not bike share";

  EXPECT_EQ(local_tile->node(count - 1)->edge_count(), 4)
      << "The bike share node must have 4 outbound edges";

  auto check_edge_attribute = [](const DirectedEdge* directededge, uint16_t forwardaccess,
                                 uint16_t reverseaccess) {
    EXPECT_TRUE(directededge->bss_connection())
        << "The bike share node's edges is not a bss connection";
    EXPECT_TRUE(directededge->forwardaccess() & forwardaccess)
        << "The edge's forwardaccess is incorrect";

    EXPECT_TRUE(directededge->reverseaccess() & reverseaccess)
        << "The edge's reverseaccess is incorrect";

    EXPECT_EQ(directededge->surface(), Surface::kPavedRough) << "The edges' surface is incorrect";
    EXPECT_EQ(directededge->cyclelane(), CycleLane::kNone) << "The edges' cyclelane is incorrect";
    EXPECT_EQ(directededge->classification(), RoadClass::kResidential)
        << "The edges' road calss is incorrect";
    EXPECT_EQ(directededge->use(), Use::kRoad) << "The edges' use is incorrect";
  };

  auto bss_edge_idx = local_tile->node(count - 1)->edge_index();

  check_edge_attribute(local_tile->directededge(bss_edge_idx), kPedestrianAccess, kPedestrianAccess);
  check_edge_attribute(local_tile->directededge(bss_edge_idx + 1), kPedestrianAccess,
                       kPedestrianAccess);
  check_edge_attribute(local_tile->directededge(bss_edge_idx + 2), kPedestrianAccess, kBicycleAccess);
  check_edge_attribute(local_tile->directededge(bss_edge_idx + 3), kBicycleAccess, kPedestrianAccess);

  auto endnode_1 = local_tile->directededge(bss_edge_idx)->endnode();
  auto count_1 = local_tile->node(endnode_1)->edge_count();
  auto edge_idx_1 = local_tile->node(endnode_1)->edge_index();
  // in this case the bike share edges should be the last two edges of this node
  check_edge_attribute(local_tile->directededge(edge_idx_1 + count_1 - 1), kPedestrianAccess,
                       kPedestrianAccess);
  check_edge_attribute(local_tile->directededge(edge_idx_1 + count_1 - 2), kBicycleAccess,
                       kPedestrianAccess);

  auto endnode_2 = local_tile->directededge(bss_edge_idx + 1)->endnode();
  auto count_2 = local_tile->node(endnode_2)->edge_count();
  auto edge_idx_2 = local_tile->node(endnode_2)->edge_index();
  // in this case the bike share edges should be the last two edges of this node
  check_edge_attribute(local_tile->directededge(edge_idx_2 + count_2 - 1), kPedestrianAccess,
                       kPedestrianAccess);
  check_edge_attribute(local_tile->directededge(edge_idx_2 + count_2 - 2), kPedestrianAccess,
                       kBicycleAccess);

  filesystem::remove(ways_file);
  filesystem::remove(way_nodes_file);
  filesystem::remove(bss_nodes_file);
  filesystem::remove(access_file);
  filesystem::remove(from_restriction_file);
  filesystem::remove(to_restriction_file);
  filesystem::remove(bss_nodes_file);
}

} // namespace

class GraphParserEnv : public ::testing::Environment {
public:
  void SetUp() override {
    DoConfig();
  }

  void TearDown() override {
  }
};

int main(int argc, char* argv[]) {
  // Test data BBs are as follows:
  // Rome:        <bounds minlat="41.8957000" minlon="12.4820400" maxlat="41.8973400"
  // maxlon="12.4855600"/> NYC:         <bounds minlat="40.7330200" minlon="-74.0136900"
  // maxlat="40.7396900" maxlon="-73.9996000"/> Baltimore:   <bounds minlat="39.2586000"
  // minlon="-76.6081000" maxlat="39.3065000" maxlon="-76.5288000"/> Harrisburg:  <bounds
  // minlat="40.2075000" minlon="-76.8459000" maxlat="40.3136000" maxlon="-76.7474000"/>
  // Liechtenstein: None
  testing::AddGlobalTestEnvironment(new GraphParserEnv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
