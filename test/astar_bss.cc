#include "test.h"

#include "loki/worker.h"
#include "odin/worker.h"
#include "thor/worker.h"

#include <boost/property_tree/ptree.hpp>

#include <algorithm> // std::copy

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

namespace bpt = boost::property_tree;

using namespace valhalla;
namespace vt = valhalla::thor;
namespace vk = valhalla::loki;
namespace vo = valhalla::odin;

namespace {

// NOTE: the default radius is set to 10 below. this is because of the stupid way we connect transit
// and bike share stations to the graph by essentially duplicating the edge instead of creating a
// false node. if you duplicate the edge but you use a radius of 0, you get a random edge candidate in
// loki which leads to random results when you make changes to the way the tiles are built. so to
// avoid that we set a radius here to get both sets of edges and let the algorithm take the cheaper
// one. this only worked before by luck
const auto conf =
    test::make_config("test/data/paris_bss_tiles", {{"loki.service_defaults.radius", "10"}});

struct route_tester {
  route_tester()
      : reader(std::make_shared<GraphReader>(conf.get_child("mjolnir"))), loki_worker(conf, reader),
        thor_worker(conf, reader), odin_worker(conf) {
  }
  Api test(const std::string& request_json) {
    Api request;
    ParseApi(request_json, valhalla::Options::route, request);
    loki_worker.route(request);
    thor_worker.route(request);
    odin_worker.narrate(request);
    return request;
  }

  std::shared_ptr<GraphReader> reader;
  vk::loki_worker_t loki_worker;
  vt::thor_worker_t thor_worker;
  vo::odin_worker_t odin_worker;
};

using TravelMode = valhalla::DirectionsLeg_TravelMode;
using BssManeuverType = valhalla::DirectionsLeg_Maneuver_BssManeuverType;

void test_request(const std::string& request,
                  const std::vector<TravelMode>& expected_travel_modes,
                  const std::vector<std::string>& expected_route,
                  // We mark only the maneuvers that are RentBike and ReturnBike
                  const std::map<size_t, BssManeuverType>& expected_bss_maneuver) {

  route_tester tester;
  auto response = tester.test(request);
  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  /*std::cout << legs.begin()->shape() << std::endl;
  auto shape = midgard::decode<std::vector<midgard::PointLL>>(legs.begin()->shape());
  for (const auto& node : legs.begin()->node()) {
    if (node.has_edge()) {
      std::vector<midgard::PointLL> subshape(shape.begin() + node.edge().begin_shape_index(),
                                             shape.begin() + node.edge().end_shape_index() + 1);
      std::cout << midgard::encode(subshape) << std::endl;
    }
  }*/

  EXPECT_EQ(legs.size(), 1) << "Should have 1 leg";

  // We going to count how many times the travel_mode has been changed
  std::vector<valhalla::DirectionsLeg_TravelMode> travel_modes;

  std::vector<std::string> route;

  for (const auto& d : directions) {
    size_t idx = -1;
    for (const auto& m : d.maneuver()) {
      auto it = expected_bss_maneuver.find(++idx);
      if (it == expected_bss_maneuver.end()) {
        EXPECT_EQ(m.bss_maneuver_type(),
                  BssManeuverType::DirectionsLeg_Maneuver_BssManeuverType_kNoneAction)
            << "BSS maneuver type at " + std::to_string(idx) + " is incorrect";

      } else {
        EXPECT_EQ(m.bss_maneuver_type(), it->second)
            << "BSS maneuver type at " + std::to_string(idx) + " is incorrect";
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
  std::copy(travel_modes.begin(), travel_modes.end(), std::ostream_iterator<int>(std::cout, ", "));
  std::cout << std::endl;
  EXPECT_TRUE(std::equal(travel_modes.begin(), travel_modes.end(), expected_travel_modes.begin(),
                         expected_travel_modes.end()))
      << "Should have " + std::to_string(expected_travel_modes.size()) + " travel_modes";

  std::copy(route.begin(), route.end(), std::ostream_iterator<std::string>(std::cout, ", "));
  std::cout << std::endl;
  EXPECT_TRUE(std::equal(route.begin(), route.end(), expected_route.begin(), expected_route.end()))
      << "The route is incorrect";
}

} // namespace

/*
 * In this test, we make it free to rent/return, so the algorithm will prefer bike share routes
 */
TEST(AstarBss, test_With_Mode_Changes) {
  std::string request =
      R"({"locations":[{"lat":48.86481,"lon":2.361015},{"lat":48.859782,"lon":2.36101}],"costing":"bikeshare",
	       "costing_options":{"pedestrian":{"bss_rent_cost":0,"bss_rent_penalty":0},
	                          "bicycle"   :{"bss_return_cost":0,"bss_return_penalty":0}}})";
  std::vector<valhalla::DirectionsLeg_TravelMode>
      expected_travel_modes{valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kPedestrian,
                            valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kBicycle,
                            valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kPedestrian};

  std::vector<std::string> expected_route{"Rue Perrée",        "Rue Perrée",        "Rue Perrée",
                                          "Rue Caffarelli",    "Rue de Bretagne",   "Rue de Turenne",
                                          "Rue du Parc Royal", "Place de Thorigny", "Rue de la Perle",
                                          "Rue de la Perle"};

  const std::map<size_t, BssManeuverType>&
      expected_bss_maneuver{{2, DirectionsLeg_Maneuver_BssManeuverType_kRentBikeAtBikeShare},
                            {9, DirectionsLeg_Maneuver_BssManeuverType_kReturnBikeAtBikeShare}};

  test_request(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}

/*
 * In this test, we increase the bss rent/return penalty considerably, so the algorithm will avoid
 * bike share stations
 */
TEST(AstarBss, test_BSS_mode_Without_Mode_Changes) {

  std::string request =
      R"({"locations":[{"lat":48.865020,"lon":2.369113},{"lat":48.859782,"lon":2.36101}],
	       "costing":"bikeshare",
	       "costing_options":{"pedestrian":{"bss_rent_cost":0,"bss_rent_penalty":1800},
	                          "bicycle"   :{"bss_return_cost":0,"bss_return_penalty":1800}}})";

  std::vector<valhalla::DirectionsLeg_TravelMode> expected_travel_modes{
      valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kPedestrian};

  // yes... the departure is still projected on the bss connection..
  std::vector<std::string> expected_route{"Rue du Grand Prieuré", "Rue de Crussol",
                                          "Rue Amelot",           "Place Pasdeloup",
                                          "Boulevard du Temple",  "Rue des Filles du Calvaire",
                                          "Rue de Turenne",       "Rue Vieille du Temple",
                                          "Rue de la Perle"};

  const std::map<size_t, BssManeuverType>& expected_bss_maneuver{};

  test_request(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}

/*
 * In this test, we increase the bss rent/return cost considerably, so the algorithm will avoid bike
 * share stations
 */
TEST(AstarBss, test_BSS_mode_Without_Mode_Changes_2) {
  std::string request =
      R"({"locations":[{"lat":48.865020,"lon":2.369113},{"lat":48.859782,"lon":2.36101}],
	       "costing":"bikeshare",
	       "costing_options":{"pedestrian":{"bss_rent_cost":1800,"bss_rent_penalty":0},
	                          "bicycle"   :{"bss_return_cost":1800,"bss_return_penalty":0}}})";

  std::vector<valhalla::DirectionsLeg_TravelMode> expected_travel_modes{
      valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kPedestrian};

  // yes... the departure is still projected on the bss connection..
  std::vector<std::string> expected_route{"Rue du Grand Prieuré", "Rue de Crussol",
                                          "Rue Amelot",           "Place Pasdeloup",
                                          "Boulevard du Temple",  "Rue des Filles du Calvaire",
                                          "Rue de Turenne",       "Rue Vieille du Temple",
                                          "Rue de la Perle"};

  const std::map<size_t, BssManeuverType>& expected_bss_maneuver{};

  test_request(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}

// We test if the bss connection edges respect the forward/reverse access
TEST(AstarBss, test_With_Mode_Changes_2) {
  std::string request =
      R"({"locations":[{"lat":48.8601411,"lon":2.3716413},{"lat":48.8594916,"lon":2.3602581}],"costing":"bikeshare",
	       "costing_options":{"pedestrian":{"bss_rent_cost":0,"bss_rent_penalty":0},
	                          "bicycle"   :{"bss_return_cost":0,"bss_return_penalty":0}}})";
  std::vector<valhalla::DirectionsLeg_TravelMode>
      expected_travel_modes{valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kPedestrian,
                            valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kBicycle,
                            valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kPedestrian};
  std::vector<std::string> expected_route{"Rue Pelée",
                                          "Rue Pelée",
                                          "Rue Alphonse Baudin",
                                          "Rue Saint-Sébastien",
                                          "Boulevard Beaumarchais",
                                          "Rue du Pont aux Choux",
                                          "Rue de Turenne",
                                          "Rue du Parc Royal",
                                          "Place de Thorigny",
                                          "Rue de la Perle",
                                          "Rue de la Perle",
                                          "Rue Vieille du Temple"};
  const std::map<size_t, BssManeuverType>&
      expected_bss_maneuver{{1, DirectionsLeg_Maneuver_BssManeuverType_kRentBikeAtBikeShare},
                            {10, DirectionsLeg_Maneuver_BssManeuverType_kReturnBikeAtBikeShare}};

  test_request(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}

// When pedestrian is chosen as travel_mode, the departure edge must NOT be a bss connections edge
TEST(AstarBss, test_Pedestrian) {
  std::string request =
      R"({"locations":[{"lat":48.859895,"lon":2.3610976338},{"lat":48.86271911,"lon":2.367111146}],"costing":"pedestrian"})";
  std::vector<valhalla::DirectionsLeg_TravelMode> expected_travel_modes{
      valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kPedestrian};
  std::vector<std::string> expected_route{"Rue de la Perle", "Rue Vieille du Temple", "Rue Froissart",
                                          "Rue Commines", "Rue Amelot"};
  // There shouldn't be any bss maneuvers
  const std::map<size_t, BssManeuverType>& expected_bss_maneuver{};

  test_request(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}

// When bicycle is chosen as travel_mode, the departure edge must NOT be a bss connections edge
TEST(AstarBss, test_Bicycle) {
  std::string request =
      R"({"locations":[{"lat":48.859895,"lon":2.3610976338},{"lat":48.86271911,"lon":2.367111146}],"costing":"bicycle"})";
  std::vector<valhalla::DirectionsLeg_TravelMode> expected_travel_modes{
      valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kBicycle};
  std::vector<std::string> expected_route{"Rue de la Perle", "Rue des Archives", "Rue de Bretagne",
                                          "Rue Commines", "Rue Amelot"};
  // There shouldn't be any bss maneuvers
  const std::map<size_t, BssManeuverType>& expected_bss_maneuver{};

  test_request(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}

// When auto is chosen as travel_mode, the departure edge must NOT be a bss connections edge
TEST(AstarBss, test_Auto) {
  std::string request =
      R"({"locations":[{"lat":48.859895,"lon":2.3610976338},{"lat":48.86271911,"lon":2.367111146}],"costing":"auto"})";
  std::vector<valhalla::DirectionsLeg_TravelMode> expected_travel_modes{
      valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kDrive};
  std::vector<std::string> expected_route{"Rue de la Perle",   "Rue Vieille du Temple",
                                          "Rue Barbette",      "Rue Elzévir",
                                          "Place de Thorigny", "Rue de Thorigny",
                                          "Rue Debelleyme",    "Rue de Turenne",
                                          "Rue Commines",      "Rue Amelot"};
  // There shouldn't be any bss maneuvers
  const std::map<size_t, BssManeuverType>& expected_bss_maneuver{};

  test_request(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}

// When auto is chosen as travel_mode, the departure edge must NOT be a bss connections edge
TEST(AstarBss, test_Truck) {
  std::string request =
      R"({"locations":[{"lat":48.859895,"lon":2.3610976338},{"lat":48.86271911,"lon":2.367111146}],"costing":"truck"})";
  std::vector<valhalla::DirectionsLeg_TravelMode> expected_travel_modes{
      valhalla::DirectionsLeg_TravelMode::DirectionsLeg_TravelMode_kDrive};
  std::vector<std::string> expected_route{"Rue de la Perle",        "Rue des Archives",
                                          "Rue Pastourelle",        "Rue du Temple",
                                          "Place de la République", "Boulevard du Temple",
                                          "Rue Oberkampf",          "Rue Amelot"};
  // There shouldn't be any bss maneuvers
  const std::map<size_t, BssManeuverType>& expected_bss_maneuver{};

  test_request(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}
