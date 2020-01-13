#include "test.h"
#include <cstdint>
#include <fstream>

#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/location.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "loki/search.h"
#include "loki/worker.h"
#include "midgard/pointll.h"
#include "midgard/vector2.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/graphenhancer.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/graphvalidator.h"
#include "mjolnir/pbfgraphparser.h"
#include "odin/directionsbuilder.h"
#include "odin/worker.h"
#include "sif/costconstants.h"
#include "sif/dynamiccost.h"
#include "sif/pedestriancost.h"
#include "thor/astar.h"
#include "thor/attributes_controller.h"
#include "thor/bidirectional_astar.h"
#include "thor/pathalgorithm.h"
#include "thor/timedep.h"
#include "thor/triplegbuilder.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

#include <valhalla/proto/directions.pb.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto/trip.pb.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm> // std::copy

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

namespace bpt = boost::property_tree;

using namespace valhalla;
namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vs = valhalla::sif;
namespace vt = valhalla::thor;
namespace vk = valhalla::loki;
namespace vj = valhalla::mjolnir;
namespace vo = valhalla::odin;

#include "mjolnir/directededgebuilder.h"
#include "mjolnir/graphtilebuilder.h"

namespace {

std::string test_dir = "test/data/paris_bss_tiles";
vb::GraphId tile_id = vb::TileHierarchy::GetGraphId({.125, .125}, 2);

const std::string config_file = "test/test_astar_bss";

void write_config(const std::string& filename) {
  std::ofstream file;
  try {
    file.open(filename, std::ios_base::trunc);
    file << "{ \
      \"mjolnir\": { \
      \"concurrency\": 1, \
       \"tile_dir\": \"test/data/paris_bss_tiles\", \
      } \
    }";
  } catch (...) {}
  file.close();
}

void DoConfig() {
  // make a config file
  write_config(config_file);
}

boost::property_tree::ptree get_conf(const char* tiles) {
  std::stringstream ss;
  ss << R"({
      "mjolnir":{"tile_dir":"test/data/)"
     << tiles << R"(", "concurrency": 1},
      "loki":{
        "actions":["route"],
        "logging":{"long_request": 100},
        "service_defaults":{"minimum_reachability": 2,"radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "heading_tolerance": 60}
      },
      "thor":{"logging":{"long_request": 100}},
      "odin":{"logging":{"long_request": 100}},
      "skadi":{"actons":["height"],"logging":{"long_request": 5}},
      "meili":{"customizable": ["turn_penalty_factor","max_route_distance_factor","max_route_time_factor","search_radius"],
              "mode":"auto","grid":{"cache_size":100240,"size":500},
              "default":{"beta":3,"breakage_distance":2000,"geometry":false,"gps_accuracy":5.0,"interpolation_distance":10,
              "max_route_distance_factor":5,"max_route_time_factor":5,"max_search_radius":200,"route":true,
              "search_radius":15.0,"sigma_z":4.07,"turn_penalty_factor":200}},
      "service_limits": {
        "auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
        "bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time": 120},
        "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
        "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
        "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
        "skadi": {"max_shape": 750000,"min_resample": 10.0},
        "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
        "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
        "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "bikeshare": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
      }
    })";
  boost::property_tree::ptree conf;
  rapidjson::read_json(ss, conf);
  return conf;
}

struct route_tester {
  route_tester(const boost::property_tree::ptree& _conf)
      : conf(_conf), reader(std::make_shared<GraphReader>(conf.get_child("mjolnir"))),
        loki_worker(conf, reader), thor_worker(conf, reader), odin_worker(conf) {
  }
  Api test(const std::string& request_json) {
    Api request;
    ParseApi(request_json, valhalla::Options::route, request);
    loki_worker.route(request);
    thor_worker.route(request);
    odin_worker.narrate(request);
    return request;
  }
  boost::property_tree::ptree conf;
  std::shared_ptr<GraphReader> reader;
  vk::loki_worker_t loki_worker;
  vt::thor_worker_t thor_worker;
  vo::odin_worker_t odin_worker;
};

using TravelMode = valhalla::DirectionsLeg_TravelMode;
using BssManeuverType = valhalla::DirectionsLeg_Maneuver_BssManeuverType;

void test(const std::string& request,
          const std::vector<TravelMode>& expected_travel_modes,
          const std::vector<std::string>& expected_route,
          // We mark only the maneuvers that are RentBikbe and ReturnBike
          const std::map<size_t, BssManeuverType>& expected_bss_maneuver) {

  auto conf = get_conf("paris_bss_tiles");
  route_tester tester(conf);
  auto response = tester.test(request);
  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  if (legs.size() != 1) {
    throw std::logic_error("Should have 1 leg");
  }
  // We going to count how many times the travel_mode has been changed
  std::vector<valhalla::DirectionsLeg_TravelMode> travel_modes;

  std::vector<std::string> route;

  for (const auto& d : directions) {

    size_t idx = -1;
    for (const auto& m : d.maneuver()) {
      auto it = expected_bss_maneuver.find(++idx);
      if (it == expected_bss_maneuver.end()) {
        if (m.bss_maneuver_type() !=
            BssManeuverType::DirectionsLeg_Maneuver_BssManeuverType_kNoneAction) {
          throw std::logic_error(std::string("BSS maneuver type at") + std::to_string(idx) +
                                 " is incorrect");
        }
      } else {
        if (m.bss_maneuver_type() != it->second) {
          throw std::logic_error(std::string("BSS maneuver type at") + std::to_string(idx) +
                                 " is incorrect");
        }
      }
      travel_modes.push_back(m.travel_mode());
      std::string name;
      for (const auto& n : m.street_name()) {
        name += n.value() + " ";
      }
      if (!name.empty()) {
        name.pop_back();
      }
      if (!name.empty()) {
        route.push_back(name);
      }
    }
  }

  travel_modes.erase(std::unique(travel_modes.begin(), travel_modes.end()), travel_modes.end());
  if (!std::equal(travel_modes.begin(), travel_modes.end(), expected_travel_modes.begin())) {
    std::copy(travel_modes.begin(), travel_modes.end(), std::ostream_iterator<int>(std::cout, ", "));
    throw std::logic_error(std::string("Should have ") +
                           std::to_string(expected_travel_modes.size()) + " travel_modes");
  }

  if (!std::equal(route.begin(), route.end(), expected_route.begin())) {
    std::copy(route.begin(), route.end(), std::ostream_iterator<std::string>(std::cout, ", "));
    throw std::logic_error("The route is incorrect");
  }

}

void TestBSS_With_Mode_Changes() {

  std::string request =
      R"({"locations":[{"lat":48.865020,"lon":2.369113},{"lat":48.859782,"lon":2.36101}],"costing":"bikeshare"})";
  std::vector<valhalla::DirectionsLeg_TravelMode>
      expected_travel_modes{valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kPedestrian,
                            valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kBicycle,
                            valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kPedestrian};

  std::vector<std::string>
      expected_route{"Rue du Grand Prieuré", "Rue du Grand Prieuré",       "Rue de Crussol",
                     "Boulevard du Temple",  "Rue des Filles du Calvaire", "Rue de Turenne",
                     "Rue du Parc Royal",    "Place de Thorigny",          "Rue de la Perle",
                     "Rue de la Perle"};

  const std::map<size_t, BssManeuverType>&
      expected_bss_maneuver{{1, DirectionsLeg_Maneuver_BssManeuverType_kRentBikeAtBikeShare},
                            {9, DirectionsLeg_Maneuver_BssManeuverType_kReturnBikeAtBikeShare}};

  test(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}

void TestBSS_With_Mode_Changes_2() {
  std::string request =
      R"({"locations":[{"lat":48.8597457895,"lon":2.3610520},{"lat":48.8582528,"lon":2.3678675}],"costing":"bikeshare"})";
  std::vector<valhalla::DirectionsLeg_TravelMode>
      expected_travel_modes{valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kPedestrian,
                            valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kBicycle,
                            valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kPedestrian};
  std::vector<std::string> expected_route{"Rue de la Perle",       "Rue de la Perle",
                                          "Rue Vieille du Temple", "Rue des Francs Bourgeois",
                                          "Rue de Turenne",        "Rue des Minimes",
                                          "Rue des Tournelles",    "Boulevard Beaumarchais",
                                          "Boulevard Beaumarchais"};
  const std::map<size_t, BssManeuverType>&
      expected_bss_maneuver{{1, DirectionsLeg_Maneuver_BssManeuverType_kRentBikeAtBikeShare},
                            {8, DirectionsLeg_Maneuver_BssManeuverType_kReturnBikeAtBikeShare}};

  test(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}

void TestBSS_Pedestrian() {
  std::string request =
      R"({"locations":[{"lat":48.859895,"lon":2.3610976338},{"lat":48.86271911,"lon":2.367111146}],"costing":"pedestrian"})";
  std::vector<valhalla::DirectionsLeg_TravelMode> expected_travel_modes{
      valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kPedestrian};
  std::vector<std::string> expected_route{"Rue de la Perle", "Rue Vieille du Temple", "Rue Froissart",
                                          "Rue Commines",    "Rue Oberkampf",         "Rue Amelot"};
  // There shouldn't be any bss maneuvers
  const std::map<size_t, BssManeuverType>& expected_bss_maneuver{};

  test(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}

void TestBSS_Bicycle() {
  std::string request =
      R"({"locations":[{"lat":48.859895,"lon":2.3610976338},{"lat":48.86271911,"lon":2.367111146}],"costing":"bicycle"})";
  std::vector<valhalla::DirectionsLeg_TravelMode> expected_travel_modes{
      valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kBicycle};
  std::vector<std::string> expected_route{"Rue de la Perle",  "Rue des Quatre Fils",
                                          "Rue des Archives", "Rue de Bretagne",
                                          "Rue Froissart",    "Rue Commines",
                                          "Rue Oberkampf",    "Rue Amelot"};
  // There shouldn't be any bss maneuvers
  const std::map<size_t, BssManeuverType>& expected_bss_maneuver{};

  test(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}

void TestBSS_Auto() {
  std::string request =
      R"({"locations":[{"lat":48.859895,"lon":2.3610976338},{"lat":48.86271911,"lon":2.367111146}],"costing":"auto"})";
  std::vector<valhalla::DirectionsLeg_TravelMode> expected_travel_modes{
      valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kDrive};
  std::vector<std::string> expected_route{"Rue de la Perle",   "Rue Vieille du Temple",
                                          "Rue Barbette",      "Rue Elzévir",
                                          "Place de Thorigny", "Rue de Thorigny",
                                          "Rue Debelleyme",    "Rue de Turenne",
                                          "Rue Commines",      "Rue Oberkampf",
                                          "Rue Amelot"};
  // There shouldn't be any bss maneuvers
  const std::map<size_t, BssManeuverType>& expected_bss_maneuver{};

  test(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}

} // anonymous namespace

int main() {
  test::suite suite("bss_astar");

  suite.test(TEST_CASE(DoConfig));

  // A general case
  suite.test(TEST_CASE(TestBSS_With_Mode_Changes));
  // We test if the bss connection edges respect the forward/reverse access
  suite.test(TEST_CASE(TestBSS_With_Mode_Changes_2));
  // When pedestrian is chosen as travel_mode, the departure edge must NOT be a bss connections edge
  suite.test(TEST_CASE(TestBSS_Pedestrian));
  // When bicycle is chosen as travel_mode, the departure edge must NOT be a bss connections edge
  suite.test(TEST_CASE(TestBSS_Bicycle));
  // When auto is chosen as travel_mode, the departure edge must NOT be a bss connections edge
  suite.test(TEST_CASE(TestBSS_Auto));

  return suite.tear_down();
}
