#include "meili/config.h"
#include "macro.h"

#define NONNEGATIVE_VALUE_MSG(value, name)                                                           \
  std::string("Expect '") + name + "' to be nonnegative (got: " + std::to_string(value) + ")"

#define POSITIVE_VALUE_MSG(value, name)                                                              \
  std::string("Expect '") + name + "' to be positive (got: " + std::to_string(value) + ")"

namespace {
template <typename T>
inline void
ReadParamOptional(T& value, const boost::property_tree::ptree& ptree, const std::string& name) {
  if (auto item = ptree.get_optional<T>(name))
    value = *item;
}

inline bool FindValue(const boost::property_tree::ptree& node, const std::string& value) {
  return std::find_if(node.begin(), node.end(),
                      [&value](const boost::property_tree::ptree::value_type& item) {
                        return item.second.get_value<std::string>() == value;
                      }) != node.end();
}
} // namespace

namespace valhalla {
namespace meili {

Config::Config(const boost::property_tree::ptree& params) {
  Read(params);
}

void Config::Read(const boost::property_tree::ptree& params) {
  candidate_search.Read(params);
  transition_cost.Read(params);
  emission_cost.Read(params);
  routing.Read(params);
}

void Config::CandidateSearch::Read(const boost::property_tree::ptree& params) {
  ReadParamOptional(search_radius_meters, params, "default.search_radius");
  CHECK_THROWS(search_radius_meters >= 0.f,
               NONNEGATIVE_VALUE_MSG(search_radius_meters, "search_radius"));

  if (const auto node = params.get_child_optional("customizable")) {
    is_search_radius_customizable = FindValue(*node, "search_radius");
  }

  ReadParamOptional(max_search_radius_meters, params, "default.max_search_radius");
  CHECK_THROWS(max_search_radius_meters > 0.f,
               POSITIVE_VALUE_MSG(max_search_radius_meters, "max_search_radius"));

  ReadParamOptional(cache_size, params, "grid.cache_size");
  ReadParamOptional(grid_size, params, "grid.size");
}

void Config::TransitionCost::Read(const boost::property_tree::ptree& params) {
  ReadParamOptional(beta, params, "default.beta");
  CHECK_THROWS(beta > 0.f, POSITIVE_VALUE_MSG(beta, "beta"));

  ReadParamOptional(breakage_distance_meters, params, "default.breakage_distance");
  CHECK_THROWS(breakage_distance_meters > 0.f,
               POSITIVE_VALUE_MSG(breakage_distance_meters, "breakage_distance"));

  if (const auto node = params.get_child_optional("customizable")) {
    is_breakage_distance_customizable = FindValue(*node, "breakage_distance");
  }

  ReadParamOptional(max_route_distance_factor, params, "default.max_route_distance_factor");
  CHECK_THROWS(max_route_distance_factor > 0.f,
               POSITIVE_VALUE_MSG(max_route_distance_factor, "max_route_distance_factor"));

  ReadParamOptional(max_route_time_factor, params, "default.max_route_time_factor");
  CHECK_THROWS(max_route_time_factor > 0.f,
               POSITIVE_VALUE_MSG(max_route_time_factor, "max_route_time_factor"));

  ReadParamOptional(turn_penalty_factor, params, "default.turn_penalty_factor");
  CHECK_THROWS(turn_penalty_factor >= 0.f,
               NONNEGATIVE_VALUE_MSG(turn_penalty_factor, "turn_penalty_factor"));

  if (const auto node = params.get_child_optional("customizable")) {
    is_turn_penalty_factor_customizable = FindValue(*node, "turn_penalty_factor");
  }
}

void Config::EmissionCost::Read(const boost::property_tree::ptree& params) {
  ReadParamOptional(sigma_z, params, "default.sigma_z");
  CHECK_THROWS(sigma_z > 0.f, POSITIVE_VALUE_MSG(sigma_z, "sigma_z"));

  ReadParamOptional(gps_accuracy_meters, params, "default.gps_accuracy");
  CHECK_THROWS(gps_accuracy_meters >= 0.f,
               NONNEGATIVE_VALUE_MSG(gps_accuracy_meters, "gps_accuracy"));

  if (const auto node = params.get_child_optional("customizable")) {
    is_gps_accuracy_customizable = FindValue(*node, "gps_accuracy");
  }
}

void Config::Routing::Read(const boost::property_tree::ptree& params) {
  ReadParamOptional(interpolation_distance_meters, params, "default.interpolation_distance");
  CHECK_THROWS(interpolation_distance_meters > 0.f,
               POSITIVE_VALUE_MSG(interpolation_distance_meters, "interpolation_distance"));

  if (const auto node = params.get_child_optional("customizable")) {
    is_interpolation_distance_customizable = FindValue(*node, "interpolation_distance");
  }
}

} // namespace meili
} // namespace valhalla
