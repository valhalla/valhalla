#include "test.h"
#include "tyr/route_handler.h"
#include "tyr/custom_route_handler.h"
#include "tyr/json.h"

#include <valhalla/mjolnir/pbfgraphparser.h>
#include <valhalla/mjolnir/graphbuilder.h>
#include <valhalla/mjolnir/graphenhancer.h>
#include <valhalla/mjolnir/hierarchybuilder.h>
#include <valhalla/mjolnir/graphvalidator.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/location.h>

#include <fstream>
#include <sstream>
#include <boost/python/dict.hpp>
#include <boost/python/str.hpp>
#include <boost/python/list.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <Python.h>

using namespace valhalla::baldr;
using namespace valhalla::tyr;

namespace {

void write_config(const std::string& filename) {

  std::ofstream file;
  try {
    file.open(filename, std::ios_base::trunc);
    file << *json::map
      ({
        {"mjolnir", json::map
          ({
            {"input", json::map({{"type", std::string("protocolbuffer")}})},
            {"hierarchy", json::map
              ({
                {"tile_dir", std::string("test/tiles")},
                {"levels", json::array
                  ({
                    json::map({
                      {"name",std::string("local")},
                      {"level", static_cast<uint64_t>(2)},
                      {"size", static_cast<long double>(0.25)}
                    }),
                    json::map({
                      {"name",std::string("arterial")},
                      {"level", static_cast<uint64_t>(1)},
                      {"size", static_cast<long double>(1)},
                      {"importance_cutoff",std::string("Tertiary")}
                    }),
                    json::map({
                      {"name",std::string("highway")},
                      {"level", static_cast<uint64_t>(0)},
                      {"size", static_cast<long double>(4)},
                      {"importance_cutoff",std::string("Trunk")}
                    })
                  })
                }
              })
            },
            {"tagtransform", json::map
              ({
                {"node_script", std::string("test/lua/vertices.lua")},
                {"node_function", std::string("nodes_proc")},
                {"way_script", std::string("test/lua/edges.lua")},
                {"way_function", std::string("ways_proc")},
                {"relation_script", std::string("test/lua/relations.lua")},
                {"relation_function", std::string("rels_proc")}
              })
            },
            {"admin", json::map
              ({
                {"admin_dir", std::string("/data/valhalla")},
                {"db_name", std::string("admin.sqlite")}
              })
            }
          })
        },
        {"thor", json::map({})},
        {"costing_options", json::map
          ({
            {"auto_shorter", json::map({})},
            {"bicycle", json::map({})},
            {"auto", json::map
              ({
                {"maneuver_penalty", static_cast<long double>(5.0)},
                {"gate_cost", static_cast<long double>(30.0)},
                {"toll_booth_cost", static_cast<long double>(15.0)},
                {"toll_booth_penalty", static_cast<long double>(0.0)}
              })
            },
            {"pedestrian", json::map
              ({
                {"walking_speed", static_cast<long double>(5.1)},
                {"walkway_factor", static_cast<long double>(0.9)},
                {"alley_factor", static_cast<long double>(2.0)},
                {"driveway_factor", static_cast<long double>(2.0)},
                {"step_penalty", static_cast<long double>(30.0)}
              })
            }
          })
        }
      });
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

  auto json = json::map
  ({
    {"loc", json::array({ loc1, loc2 })},
    {"costing", request_type},
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

json::ArrayPtr locations(const std::vector<Location>& loc_list){

  auto locations = json::array({});
  for(const auto& loc : loc_list) {

    auto location = json::map({});

    location->emplace("latitude", static_cast<long double>(loc.latlng_.lat()));
    location->emplace("longitude",static_cast<long double>(loc.latlng_.lng()));
    location->emplace(
        "type",
        (loc.stoptype_ == Location::StopType::THROUGH ?
            std::string("through") : std::string("break")));

    locations->emplace_back(location);
  }

  return locations;
}

boost::property_tree::ptree make_json_request(float lat1, float lng1,
                                              float lat2, float lng2,
                                              const std::string& request_type) {
  std::vector<Location> loc_list;
  loc_list.emplace_back(Location({lng1, lat1}));
  loc_list.emplace_back(Location({lng2, lat2}));

  auto json = json::map
  ({
    {"locations", locations(loc_list)},
    {"costing", request_type},
    {"output", std::string("json")},
    {"z", static_cast<uint64_t>(17)},
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
    make_json_request(47.139815, 9.525708, 47.167321, 9.509609, "auto");

  //run the route
  valhalla::tyr::CustomRouteHandler handler(config, request);
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
