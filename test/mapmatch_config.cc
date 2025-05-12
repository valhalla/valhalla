#include "meili/config.h"

#include "test.h"

namespace {

const auto fake_config = test::json_to_pt(R"({
    "customizable": [
      "search_radius",
      "gps_accuracy"
    ],
    "mode": "auto",
    "grid": {
      "cache_size": 100500,
      "size": 100
    },
    "default": {
      "beta": 5,
      "breakage_distance": 5000,
      "gps_accuracy": 6,
      "interpolation_distance": 5,
      "max_route_distance_factor": 11,
      "max_route_time_factor": 10,
      "max_search_radius": 500,
      "search_radius": 10,
      "sigma_z": 5.1,
      "turn_penalty_factor": 100
    }
  })");

TEST(MapmatchConfig, check_read_all_params) {
  valhalla::meili::Config config;
  config.Read(fake_config);

  // check candidate search params
  const auto& candidate_search = config.candidate_search;
  EXPECT_EQ(candidate_search.search_radius_meters, 10.f);
  EXPECT_TRUE(candidate_search.is_search_radius_customizable);
  EXPECT_EQ(candidate_search.max_search_radius_meters, 500.f);
  EXPECT_EQ(candidate_search.grid_size, 100);
  EXPECT_EQ(candidate_search.cache_size, 100500);

  // check transition params
  const auto& transition = config.transition_cost;
  EXPECT_EQ(transition.beta, 5.f);
  EXPECT_EQ(transition.breakage_distance_meters, 5000.f);
  EXPECT_FALSE(transition.is_breakage_distance_customizable);
  EXPECT_EQ(transition.turn_penalty_factor, 100.f);
  EXPECT_FALSE(transition.is_turn_penalty_factor_customizable);
  EXPECT_EQ(transition.max_route_time_factor, 10.f);
  EXPECT_EQ(transition.max_route_distance_factor, 11.f);

  // check emission params
  const auto& emission = config.emission_cost;
  EXPECT_EQ(emission.sigma_z, 5.1f);
  EXPECT_EQ(emission.gps_accuracy_meters, 6.f);
  EXPECT_TRUE(emission.is_gps_accuracy_customizable);

  // check routing params
  const auto& routing = config.routing;
  EXPECT_EQ(routing.interpolation_distance_meters, 5.f);
  EXPECT_FALSE(routing.is_interpolation_distance_customizable);
}

TEST(MapmatchConfig, validate_candidate_search_params) {
  valhalla::meili::Config config;

  auto pt = fake_config;
  pt.put<float>("default.search_radius", -1.f);
  EXPECT_THROW(config.Read(pt), std::exception);

  pt = fake_config;
  pt.put<float>("default.max_search_radius", -1.f);
  EXPECT_THROW(config.Read(pt), std::exception);
}

TEST(MapmatchConfig, validate_emission_params) {
  valhalla::meili::Config config;

  auto pt = fake_config;
  pt.put<float>("default.gps_accuracy", 0.f);
  EXPECT_NO_THROW(config.Read(pt));

  pt.put<float>("default.gps_accuracy", -1.f);
  EXPECT_THROW(config.Read(pt), std::exception);

  pt = fake_config;
  pt.put<float>("default.sigma_z", -1.f);
  EXPECT_THROW(config.Read(pt), std::exception);
}

TEST(MapmatchConfig, validate_transition_params) {
  valhalla::meili::Config config;

  auto pt = fake_config;
  pt.put<float>("default.beta", -1.f);
  EXPECT_THROW(config.Read(pt), std::exception);

  pt = fake_config;
  pt.put<float>("default.breakage_distance", -1.f);
  EXPECT_THROW(config.Read(pt), std::exception);

  pt = fake_config;
  pt.put<float>("default.max_route_distance_factor", -1.f);
  EXPECT_THROW(config.Read(pt), std::exception);

  pt = fake_config;
  pt.put<float>("default.max_route_time_factor", -1.f);
  EXPECT_THROW(config.Read(pt), std::exception);

  pt = fake_config;
  pt.put<float>("default.turn_penalty_factor", 0.f);
  EXPECT_NO_THROW(config.Read(pt));

  pt.put<float>("default.turn_penalty_factor", -1.f);
  EXPECT_THROW(config.Read(pt), std::exception);
}

TEST(MapmatchConfig, validate_routing_params) {
  valhalla::meili::Config config;

  auto pt = fake_config;
  pt.put<float>("default.interpolation_distance", -1.f);
  EXPECT_THROW(config.Read(pt), std::exception);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
