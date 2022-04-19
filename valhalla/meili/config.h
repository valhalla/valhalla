#pragma once

#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace meili {

struct Config {
  // create config with default parameters
  Config() = default;
  // read parameters from boost::ptree (use default values for missing params)
  explicit Config(const boost::property_tree::ptree& params);
  // override default config parameters
  void Read(const boost::property_tree::ptree& params);

  struct CandidateSearch {
    // default search radius (meters) for measurements; it's used to determine maximum search radius
    float search_radius_meters = 50.f;
    // define if 'search_radius' option can be reassigned with user request
    bool is_search_radius_customizable = true;
    // maximum allowed difference (meters) between GPS point and corresponding route point
    float max_search_radius_meters = 200.f;

    size_t cache_size = 100240;
    size_t grid_size = 500;

    void Read(const boost::property_tree::ptree& params);
  };

  struct TransitionCost {
    // beta parameter of exponential distribution
    float beta = 3.f;
    // maximum allowed difference (meters) between route distance and great circle distance
    float breakage_distance_meters = 2000.f;
    // define if 'breakage_distance' option can be reassigned with user request
    bool is_breakage_distance_customizable = false;
    // maximum allowed ratio of route distance to great circle distance
    float max_route_distance_factor = 5.f;
    // maximum allowed ratio of route time distance to the corresponding measurements time distance
    float max_route_time_factor = 5.f;
    // it's used to determine 'turn cost' depending on turn degree
    float turn_penalty_factor = 200.f;
    // define if 'turn_penalty_factor' option can be reassigned with user request
    bool is_turn_penalty_factor_customizable = true;

    void Read(const boost::property_tree::ptree& params);
  };

  struct EmissionCost {
    // standard deviation of GPS measurements
    float sigma_z = 4.07f;
    // default accuracy (meters) of GPS measurements; it's used to determine maximum search radius
    float gps_accuracy_meters = 5.f;
    // define if 'gps_accuracy' option can be reassigned with user request
    bool is_gps_accuracy_customizable = false;

    void Read(const boost::property_tree::ptree& params);
  };

  struct Routing {
    // interpolation distance (meters); threshold for merging close points together
    float interpolation_distance_meters = 10.f;
    // define if 'interpolation_distance' option can be reassigned with user request
    bool is_interpolation_distance_customizable = false;

    void Read(const boost::property_tree::ptree& params);
  };

  CandidateSearch candidate_search{};
  TransitionCost transition_cost{};
  EmissionCost emission_cost{};
  Routing routing{};
};

} // namespace meili
} // namespace valhalla
