#include "test.h"
#include "tyr/actor.h"

#include <boost/property_tree/ptree.hpp>
using namespace valhalla;
using namespace baldr;

TEST(Standalone, Utrecht) {
  const auto utrecht_conf = test::make_config(VALHALLA_BUILD_DIR "test/data/utrecht_tiles");
  auto reader = test::make_clean_graphreader(utrecht_conf.get_child("mjolnir"));
  tyr::actor_t actor(utrecht_conf);

  std::string request =
      R"({"locations": [{"lat": 52.105031, "lon": 5.077844}, {"lat": 52.0942903,
      "lon": 5.1300778}], "costing": "auto_pedestrian", "format": "pbf"})";
  auto r = actor.route(request);
  valhalla::Api response;

  response.ParseFromString(r);
  ASSERT_EQ(response.directions().routes_size(), 1);
  EXPECT_EQ(response.directions().routes(0).legs(0).maneuver(0).travel_mode(), TravelMode::kDrive);
  EXPECT_EQ(response.directions().routes(0).legs(0).maneuver(20).travel_mode(),
            TravelMode::kPedestrian);
  actor.cleanup();
}
