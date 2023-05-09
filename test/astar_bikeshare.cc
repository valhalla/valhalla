#include <algorithm> // std::copy
#include <optional>

#include <boost/property_tree/ptree.hpp>

#include "loki/worker.h"
#include "odin/worker.h"
#include "test.h"
#include "thor/worker.h"

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

using BssManeuverType = valhalla::DirectionsLeg_Maneuver_BssManeuverType;

void test_request(const std::string& request,
                  const std::vector<TravelMode>& expected_travel_modes,
                  const std::vector<std::string>& expected_route,
                  // We mark only the maneuvers that are RentBike and ReturnBike
                  const std::map<size_t, BssManeuverType>& expected_bss_maneuver,
                  const std::map<size_t, std::string>& expected_bss_ref = {},
                  const std::optional<std::string>& expected_shape = {}) {

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
  std::vector<TravelMode> travel_modes;

  std::vector<std::string> route;

  for (const auto& d : directions) {
    if (expected_shape) {
      EXPECT_EQ(d.shape(), *expected_shape) << "The shape is incorrect";
    }
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
      auto search = expected_bss_ref.find(idx);
      if (search != expected_bss_ref.end()) {
        EXPECT_EQ(m.bss_info().ref(), search->second)
            << "bss_info.osm_node_id at " + std::to_string(idx) + " is incorrect";
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
 * A general case
 */
TEST(AstarBss, test_With_Mode_Changes) {
  std::string request =
      R"({"locations":[{"lat":48.864655,"lon":2.361374},{"lat":48.859608,"lon":2.36117}],"costing":"bikeshare",
	       "costing_options":{"pedestrian":{"bss_rent_cost":0,"bss_rent_penalty":0},
	                          "bicycle"   :{"bss_return_cost":0,"bss_return_penalty":0}}})";
  std::vector<TravelMode> expected_travel_modes{TravelMode::kPedestrian, TravelMode::kBicycle,
                                                TravelMode::kPedestrian};

  std::vector<std::string> expected_route{"Rue Gabriel Vicaire", "Rue Perrée",
                                          "Rue Perrée",          "Rue Caffarelli",
                                          "Rue de Bretagne",     "Rue de Turenne",
                                          "Rue du Parc Royal",   "Place de Thorigny",
                                          "Rue de la Perle",     "Rue de la Perle"};

  const std::map<size_t, BssManeuverType>&
      expected_bss_maneuver{{2, DirectionsLeg_Maneuver_BssManeuverType_kRentBikeAtBikeShare},
                            {9, DirectionsLeg_Maneuver_BssManeuverType_kReturnBikeAtBikeShare}};
  const std::map<size_t, std::string>& expected_bss_ref{{2, "3006"}, {9, "3008"}};

  test_request(request, expected_travel_modes, expected_route, expected_bss_maneuver,
               expected_bss_ref);
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

  std::vector<TravelMode> expected_travel_modes{TravelMode::kPedestrian};

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

  std::vector<TravelMode> expected_travel_modes{TravelMode::kPedestrian};

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
  std::vector<TravelMode> expected_travel_modes{TravelMode::kPedestrian, TravelMode::kBicycle,
                                                TravelMode::kPedestrian};
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
  const std::map<size_t, std::string>& expected_bss_ref{{1, "11103"}, {10, "3008"}};

  test_request(request, expected_travel_modes, expected_route, expected_bss_maneuver,
               expected_bss_ref);
}

// When pedestrian is chosen as travel_mode, the departure edge must NOT be a bss connections edge
TEST(AstarBss, test_Pedestrian) {
  std::string request =
      R"({"locations":[{"lat":48.859895,"lon":2.3610976338},{"lat":48.86271911,"lon":2.367111146}],"costing":"pedestrian"})";
  std::vector<TravelMode> expected_travel_modes{TravelMode::kPedestrian};
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
  std::vector<TravelMode> expected_travel_modes{TravelMode::kBicycle};
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
  std::vector<TravelMode> expected_travel_modes{TravelMode::kDrive};
  std::vector<std::string> expected_route{"Rue de la Perle", "Rue des Archives", "Rue Pastourelle",
                                          "Rue de Turenne",  "Rue Commines",     "Rue Amelot"};
  // There shouldn't be any bss maneuvers
  const std::map<size_t, BssManeuverType>& expected_bss_maneuver{};

  test_request(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}

// When auto is chosen as travel_mode, the departure edge must NOT be a bss connections edge
TEST(AstarBss, test_Truck) {
  std::string request =
      R"({"locations":[{"lat":48.859895,"lon":2.3610976338},{"lat":48.86271911,"lon":2.367111146}],"costing":"truck"})";
  std::vector<TravelMode> expected_travel_modes{TravelMode::kDrive};
  std::vector<std::string>
      expected_route{"Rue de la Perle",     "Rue des Archives",       "Rue Pastourelle",
                     "Rue du Temple",       "Place de la République", "Place de la République",
                     "Boulevard du Temple", "Rue Oberkampf",          "Rue Amelot"};
  // There shouldn't be any bss maneuvers
  const std::map<size_t, BssManeuverType>& expected_bss_maneuver{};

  test_request(request, expected_travel_modes, expected_route, expected_bss_maneuver);
}

// In this test case, the bike share station(48.8690345, 2.3622890) is located a dedicated cyclelane
// and a pedestrian way.
//
//        (cyclelane) ------------------------------>
//
//           user ->> ____________
//                               |
//                               * Bike Share Station
//                               |__________ ->>
//
//    (pedestrian way) ------------------------------>
//
// Since BSS connections are created over both Pedestrian mode and Bicycle mode, user should be able
// to turn back the bike right on the cyclelane, change the travel mode and coninue his journey way on
// the pedestrian way.
TEST(AstarBss, test_BSSConnections_on_Pedestrian_and_Bicycle) {
  std::string request =
      R"({"locations":[{"lat":48.864218,"lon":2.362034},{"lat":48.869068,"lon":2.362151}],"costing":"bikeshare"})";
  std::vector<TravelMode> expected_travel_modes{TravelMode::kPedestrian, TravelMode::kBicycle,
                                                TravelMode::kPedestrian};
  std::vector<std::string> expected_route{"Rue Perrée",
                                          "Rue Perrée",
                                          "Rue Perrée",
                                          "Rue Eugène Spuller",
                                          "Rue Béranger",
                                          "Rue du Temple",
                                          "Place de la République",
                                          "Place de la République",
                                          "Boulevard de Magenta",
                                          "Rue du Château d'Eau",
                                          "Rue du Château d'Eau"};

  const std::map<size_t, BssManeuverType>&
      expected_bss_maneuver{{2, DirectionsLeg_Maneuver_BssManeuverType_kRentBikeAtBikeShare},
                            {11, DirectionsLeg_Maneuver_BssManeuverType_kReturnBikeAtBikeShare}};

  const std::map<size_t, std::string>& expected_bss_ref{{2, "3006"}, {11, "10011"}};

  std::string expected_shape =
      "e~le|A_ldoCyD~IoAtCkArC]z@kBpEeAsAdArAjBqE\\{@jAsCad@ai@yAgBo@iCuF_Ua@_B[uAyQgz@i@cCwAt@mg@bXyt@b`@yCvAyBqH{EgLiCvEoD|G{\\`r@wFqHoPqTy@gAyAkBe@o@i@q@{D_CeB{@wCfC{XfVt@jCjA~Dn@xB?lBcA|BV\\f@r@wBlE";

  test_request(request, expected_travel_modes, expected_route, expected_bss_maneuver,
               expected_bss_ref, expected_shape);
}

class AstarBSSTest : public thor::AStarBSSAlgorithm {
public:
  explicit AstarBSSTest(const boost::property_tree::ptree& config = {}) : AStarBSSAlgorithm(config) {
  }

  void Clear() {
    AStarBSSAlgorithm::Clear();
    if (clear_reserved_memory_) {
      EXPECT_EQ(edgelabels_.capacity(), 0);
    } else {
      EXPECT_LE(edgelabels_.capacity(), max_reserved_labels_count_);
    }
  }
};

TEST(AstarBss, test_clear_reserved_memory) {
  boost::property_tree::ptree config;
  config.put("clear_reserved_memory", true);

  AstarBSSTest astar(config);
  astar.Clear();
}

TEST(AstarBss, test_max_reserved_labels_count) {
  boost::property_tree::ptree config;
  config.put("max_reserved_labels_count_astar", 10);

  AstarBSSTest astar(config);
  astar.Clear();
}
