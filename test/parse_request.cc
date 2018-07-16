#include <iostream>
#include <iterator>
#include <list>
#include <string>
#include <vector>

#include "worker.h"

#include <valhalla/proto/directions_options.pb.h>

#include "test.h"

namespace {
// Auto defaults
constexpr float kDefaultAutoManeuverPenalty = 5.0f;          // Seconds
constexpr float kDefaultAutoDestinationOnlyPenalty = 600.0f; // Seconds
constexpr float kDefaultAutoAlleyPenalty = 5.0f;             // Seconds
constexpr float kDefaultAutoGateCost = 30.0f;                // Seconds
constexpr float kDefaultAutoGatePenalty = 300.0f;            // Seconds
constexpr float kDefaultAutoTollBoothCost = 15.0f;           // Seconds
constexpr float kDefaultAutoTollBoothPenalty = 0.0f;         // Seconds
constexpr float kDefaultAutoFerryCost = 300.0f;              // Seconds
constexpr float kDefaultAutoCountryCrossingCost = 600.0f;    // Seconds
constexpr float kDefaultAutoCountryCrossingPenalty = 0.0f;   // Seconds
constexpr float kDefaultAutoUseFerry = 0.5f;                 // Factor between 0 and 1
constexpr float kDefaultAutoUseHighways = 1.0f;              // Factor between 0 and 1
constexpr float kDefaultAutoUseTolls = 0.5f;                 // Factor between 0 and 1

// Motor Scooter defaults
constexpr float kDefaultMotorScooterManeuverPenalty = 5.0f;          // Seconds
constexpr float kDefaultMotorScooterAlleyPenalty = 5.0f;             // Seconds
constexpr float kDefaultMotorScooterGateCost = 30.0f;                // Seconds
constexpr float kDefaultMotorScooterGatePenalty = 300.0f;            // Seconds
constexpr float kDefaultMotorScooterFerryCost = 300.0f;              // Seconds
constexpr float kDefaultMotorScooterCountryCrossingCost = 600.0f;    // Seconds
constexpr float kDefaultMotorScooterCountryCrossingPenalty = 0.0f;   // Seconds
constexpr float kDefaultMotorScooterUseFerry = 0.5f;                 // Factor between 0 and 1
constexpr float kDefaultMotorScooterDestinationOnlyPenalty = 120.0f; // Seconds
constexpr float kDefaultMotorScooterUseHills = 0.5f;                 // Factor between 0 and 1
constexpr float kDefaultMotorScooterUsePrimary = 0.5f;               // Factor between 0 and 1
constexpr uint32_t kDefaultMotorScooterTopSpeed = 45;                // Kilometers per hour

// Motorcycle defaults
constexpr float kDefaultMotorcycleManeuverPenalty = 5.0f;          // Seconds
constexpr float kDefaultMotorcycleAlleyPenalty = 5.0f;             // Seconds
constexpr float kDefaultMotorcycleGateCost = 30.0f;                // Seconds
constexpr float kDefaultMotorcycleGatePenalty = 300.0f;            // Seconds
constexpr float kDefaultMotorcycleTollBoothCost = 15.0f;           // Seconds
constexpr float kDefaultMotorcycleTollBoothPenalty = 0.0f;         // Seconds
constexpr float kDefaultMotorcycleFerryCost = 300.0f;              // Seconds
constexpr float kDefaultMotorcycleCountryCrossingCost = 600.0f;    // Seconds
constexpr float kDefaultMotorcycleCountryCrossingPenalty = 0.0f;   // Seconds
constexpr float kDefaultMotorcycleUseFerry = 0.5f;                 // Factor between 0 and 1
constexpr float kDefaultMotorcycleUseHighways = 1.0f;              // Factor between 0 and 1
constexpr float kDefaultMotorcycleUseTolls = 0.5f;                 // Factor between 0 and 1
constexpr float kDefaultMotorcycleUsePrimary = 0.5f;               // Factor between 0 and 1
constexpr float kDefaultMotorcycleUseTrails = 0.0f;                // Factor between 0 and 1
constexpr float kDefaultMotorcycleDestinationOnlyPenalty = 600.0f; // Seconds

// Pedestrian defaults
constexpr uint32_t kDefaultPedestrianMaxDistanceFoot = 100000;      // 100 km
constexpr uint32_t kDefaultPedestrianMaxDistanceWheelchair = 10000; // 10 km
constexpr float kDefaultPedestrianSpeedFoot = 5.1f;                 // 3.16 MPH
constexpr float kDefaultPedestrianSpeedWheelchair = 4.0f;           // 2.5  MPH  TODO
constexpr float kDefaultPedestrianStepPenaltyFoot = 30.0f;          // 30 seconds
constexpr float kDefaultPedestrianStepPenaltyWheelchair = 600.0f;   // 10 minutes
constexpr uint32_t kDefaultPedestrianMaxGradeFoot = 90;
constexpr uint32_t kDefaultPedestrianMaxGradeWheelchair = 12;    // Conservative for now...
constexpr uint8_t kDefaultPedestrianMaxHikingDifficulty = 1;     // T1 (kHiking)
constexpr float kDefaultPedestrianModeFactor = 1.5f;             // Favor this mode?
constexpr float kDefaultPedestrianManeuverPenalty = 5.0f;        // Seconds
constexpr float kDefaultPedestrianGatePenalty = 10.0f;           // Seconds
constexpr float kDefaultPedestrianWalkwayFactor = 0.9f;          // Slightly favor walkways
constexpr float kDefaultPedestrianSideWalkFactor = 0.95f;        // Slightly favor sidewalks
constexpr float kDefaultPedestrianAlleyFactor = 2.0f;            // Avoid alleys
constexpr float kDefaultPedestrianDrivewayFactor = 5.0f;         // Avoid driveways
constexpr float kDefaultPedestrianFerryCost = 300.0f;            // Seconds
constexpr float kDefaultPedestrianCountryCrossingCost = 600.0f;  // Seconds
constexpr float kDefaultPedestrianCountryCrossingPenalty = 0.0f; // Seconds
constexpr float kDefaultPedestrianUseFerry = 1.0f;
constexpr uint32_t kDefaultPedestrianTransitStartEndMaxDistance = 2415; // 1.5 miles
constexpr uint32_t kDefaultPedestrianTransitTransferMaxDistance = 805;  // 0.5 miles

///////////////////////////////////////////////////////////////////////////////
// validate by type methods
void validate(const std::string& key,
              const bool expected_value,
              const bool has_pbf_value,
              const bool pbf_value) {
  if (!has_pbf_value) {
    throw std::runtime_error("bool value not found in pbf for key=" + key);
  }
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key + " bool | expected_value=" +
                             std::string(expected_value ? "true" : "false") +
                             " | found=" + std::string(pbf_value ? "true" : "false"));
  }
}

void validate(const std::string& key, const float expected_value, const float pbf_value) {
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key +
                             " float | expected_value=" + std::to_string(expected_value) +
                             " | found=" + std::to_string(pbf_value));
  }
}

void validate(const std::string& key,
              const float expected_value,
              const bool has_pbf_value,
              const float pbf_value) {
  if (!has_pbf_value) {
    throw std::runtime_error("float value not found in pbf for key=" + key);
  }
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key +
                             " float | expected_value=" + std::to_string(expected_value) +
                             " | found=" + std::to_string(pbf_value));
  }
}

void validate(const std::string& key, const uint32_t expected_value, const uint32_t pbf_value) {
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key +
                             " uint32_t | expected_value=" + std::to_string(expected_value) +
                             " | found=" + std::to_string(pbf_value));
  }
}

void validate(const std::string& key,
              const uint32_t expected_value,
              const bool has_pbf_value,
              const uint32_t pbf_value) {
  if (!has_pbf_value) {
    throw std::runtime_error("uint32_t value not found in pbf for key=" + key);
  }
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key +
                             " uint32_t | expected_value=" + std::to_string(expected_value) +
                             " | found=" + std::to_string(pbf_value));
  }
}

void validate(const std::string& key,
              const std::string& expected_value,
              const std::string& pbf_value) {
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key + " string | expected_value=" + expected_value +
                             " | found=" + pbf_value);
  }
}

void validate(const std::string& key,
              const std::string& expected_value,
              const bool has_pbf_value,
              const std::string& pbf_value) {
  if (!has_pbf_value) {
    throw std::runtime_error("string value not found in pbf for key=" + key);
  }
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key + " string | expected_value=" + expected_value +
                             " | found=" + pbf_value);
  }
}

void validate(const std::string& key,
              const valhalla::odin::ShapeMatch expected_value,
              const bool has_pbf_value,
              const valhalla::odin::ShapeMatch pbf_value) {
  if (!has_pbf_value) {
    throw std::runtime_error("ShapeMatch value not found in pbf for key=" + key);
  }
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key +
                             " ShapeMatch | expected_value=" + ShapeMatch_Name(expected_value) +
                             " | found=" + ShapeMatch_Name(pbf_value));
  }
}

void validate(const std::string& key,
              const valhalla::odin::FilterAction expected_value,
              const bool has_pbf_value,
              const valhalla::odin::FilterAction pbf_value) {
  if (!has_pbf_value) {
    throw std::runtime_error("FilterAction value not found in pbf for key=" + key);
  }
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key +
                             " FilterAction | expected_value=" + FilterAction_Name(expected_value) +
                             " | found=" + FilterAction_Name(pbf_value));
  }
}

void validate(const std::string& key,
              const std::vector<std::string>& expected_values,
              const bool has_pbf_values,
              const google::protobuf::RepeatedPtrField<std::string>& pbf_values) {
  if (!has_pbf_values) {
    throw std::runtime_error("string values not found in pbf for key=" + key);
  }
  if (expected_values.size() != pbf_values.size()) {
    throw std::runtime_error("invalid count in pbf for key=" + key);
  }
  for (size_t i = 0; i < expected_values.size(); ++i) {
    if (pbf_values.Get(i) != expected_values.at(i)) {
      throw std::runtime_error("Incorrect " + key + " string | expected_value=" +
                               expected_values.at(i) + " | found=" + pbf_values.Get(i));
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// get request methods
std::string get_request_str(const std::string& key, const bool expected_value) {
  return R"({")" + key + R"(":)" + std::string(expected_value ? "true" : "false") + R"(})";
}

std::string get_request_str(const std::string& key, const float expected_value) {
  return R"({")" + key + R"(":)" + std::to_string(expected_value) + R"(})";
}

std::string
get_request_str(const std::string& parent_key, const std::string& key, const float expected_value) {
  return R"({")" + parent_key + R"(":{")" + key + R"(":)" + std::to_string(expected_value) + R"(}})";
}

std::string get_request_str(const std::string& grandparent_key,
                            const std::string& parent_key,
                            const std::string& key,
                            const float specified_value) {
  return R"({")" + grandparent_key + R"(":{")" + parent_key + R"(":{")" + key + R"(":)" +
         std::to_string(specified_value) + R"(}}})";
}

std::string get_request_str(const std::string& grandparent_key,
                            const std::string& parent_key,
                            const std::string& sibling_key,
                            const std::string& sibling_value,
                            const std::string& key,
                            const float specified_value) {
  return R"({")" + grandparent_key + R"(":{")" + parent_key + R"(":{")" + sibling_key +
         R"(":")" + sibling_value + R"(",")" + key + R"(":)" + std::to_string(specified_value) +
         R"(}}})";
}

std::string get_request_str(const std::string& grandparent_key,
                            const std::string& parent_key,
                            const std::string& key,
                            const uint32_t specified_value) {
  return R"({")" + grandparent_key + R"(":{")" + parent_key + R"(":{")" + key + R"(":)" +
         std::to_string(specified_value) + R"(}}})";
}

std::string get_request_str(const std::string& grandparent_key,
                            const std::string& parent_key,
                            const std::string& sibling_key,
                            const std::string& sibling_value,
                            const std::string& key,
                            const uint32_t specified_value) {
  return R"({")" + grandparent_key + R"(":{")" + parent_key + R"(":{")" + sibling_key +
         R"(":")" + sibling_value + R"(",")" + key + R"(":)" + std::to_string(specified_value) +
         R"(}}})";
}

std::string get_request_str(const std::string& grandparent_key,
                            const std::string& parent_key,
                            const std::string& key,
                            const std::string& specified_value) {
  return R"({")" + grandparent_key + R"(":{")" + parent_key + R"(":{")" + key + R"(":")" +
         specified_value + R"("}}})";
}

std::string get_request_str(const std::string& key, const uint32_t expected_value) {
  return R"({")" + key + R"(":)" + std::to_string(expected_value) + R"(})";
}

std::string get_request_str(const std::string& key, const std::string& expected_value) {
  return R"({")" + key + R"(":")" + expected_value + R"("})";
}

std::string get_request_str(const std::string& key, const valhalla::odin::ShapeMatch expected_value) {
  return R"({")" + key + R"(":")" + ShapeMatch_Name(expected_value) + R"("})";
}

std::string get_request_str(const std::string& parent_key,
                            const std::string& key,
                            const valhalla::odin::FilterAction expected_value) {
  return R"({")" + parent_key + R"(":{")" + key + R"(":")" + FilterAction_Name(expected_value) +
         R"("}})";
}

std::string get_request_str(const std::string& parent_key,
                            const std::string& key,
                            const std::vector<std::string>& expected_values) {
  std::string request_str = R"({")" + parent_key + R"(":{")" + key + R"(":[)";
  bool first = true;
  for (const auto& expected_value : expected_values) {
    if (first) {
      request_str += R"(")";
      first = false;
    } else {
      request_str += R"(,")";
    }
    request_str += expected_value + R"(")";
  }
  request_str += R"(]}})";
  return request_str;
}

valhalla::valhalla_request_t get_request(const std::string& request_str,
                                         const valhalla::odin::DirectionsOptions::Action action) {
  std::cout << ">>>>> request_str=" << request_str << "<<<<<" << std::endl;
  valhalla::valhalla_request_t request;
  request.parse(request_str, action);
  return request;
}

///////////////////////////////////////////////////////////////////////////////
// test parsing methods
void test_polygons_parsing(const bool expected_value,
                           const valhalla::odin::DirectionsOptions::Action action =
                               valhalla::odin::DirectionsOptions::isochrone) {
  const std::string key = "polygons";
  valhalla::valhalla_request_t request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options.has_polygons(), request.options.polygons());
}

void test_denoise_parsing(const float expected_value,
                          const valhalla::odin::DirectionsOptions::Action action =
                              valhalla::odin::DirectionsOptions::isochrone) {
  const std::string key = "denoise";
  valhalla::valhalla_request_t request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options.has_denoise(), request.options.denoise());
}

void test_generalize_parsing(const float expected_value,
                             const valhalla::odin::DirectionsOptions::Action action =
                                 valhalla::odin::DirectionsOptions::isochrone) {
  const std::string key = "generalize";
  valhalla::valhalla_request_t request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options.has_generalize(), request.options.generalize());
}

void test_show_locations_parsing(const bool expected_value,
                                 const valhalla::odin::DirectionsOptions::Action action =
                                     valhalla::odin::DirectionsOptions::isochrone) {
  const std::string key = "show_locations";
  valhalla::valhalla_request_t request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options.has_show_locations(),
           request.options.show_locations());
}

void test_shape_match_parsing(const valhalla::odin::ShapeMatch expected_value,
                              const valhalla::odin::DirectionsOptions::Action action) {
  const std::string key = "shape_match";
  valhalla::valhalla_request_t request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options.has_shape_match(), request.options.shape_match());
}

void test_best_paths_parsing(const uint32_t expected_value,
                             const valhalla::odin::DirectionsOptions::Action action =
                                 valhalla::odin::DirectionsOptions::isochrone) {
  const std::string key = "best_paths";
  valhalla::valhalla_request_t request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options.has_best_paths(), request.options.best_paths());
}

void test_gps_accuracy_parsing(const float expected_value,
                               const valhalla::odin::DirectionsOptions::Action action =
                                   valhalla::odin::DirectionsOptions::isochrone) {
  const std::string parent_key = "trace_options";
  const std::string key = "gps_accuracy";
  valhalla::valhalla_request_t request =
      get_request(get_request_str(parent_key, key, expected_value), action);
  validate(key, expected_value, request.options.has_gps_accuracy(), request.options.gps_accuracy());
}

void test_search_radius_parsing(const float expected_value,
                                const valhalla::odin::DirectionsOptions::Action action =
                                    valhalla::odin::DirectionsOptions::isochrone) {
  const std::string parent_key = "trace_options";
  const std::string key = "search_radius";
  valhalla::valhalla_request_t request =
      get_request(get_request_str(parent_key, key, expected_value), action);
  validate(key, expected_value, request.options.has_search_radius(), request.options.search_radius());
}

void test_turn_penalty_factor_parsing(const float expected_value,
                                      const valhalla::odin::DirectionsOptions::Action action =
                                          valhalla::odin::DirectionsOptions::isochrone) {
  const std::string parent_key = "trace_options";
  const std::string key = "turn_penalty_factor";
  valhalla::valhalla_request_t request =
      get_request(get_request_str(parent_key, key, expected_value), action);
  validate(key, expected_value, request.options.has_turn_penalty_factor(),
           request.options.turn_penalty_factor());
}

void test_filter_action_parsing(const valhalla::odin::FilterAction expected_value,
                                const valhalla::odin::DirectionsOptions::Action action =
                                    valhalla::odin::DirectionsOptions::trace_attributes) {
  const std::string parent_key = "filters";
  const std::string key = "action";
  valhalla::valhalla_request_t request =
      get_request(get_request_str(parent_key, key, expected_value), action);
  validate(key, expected_value, request.options.has_filter_action(), request.options.filter_action());
}

void test_filter_attributes_parsing(const std::vector<std::string>& expected_values,
                                    const valhalla::odin::DirectionsOptions::Action action =
                                        valhalla::odin::DirectionsOptions::trace_attributes) {
  const std::string parent_key = "filters";
  const std::string key = "attributes";
  valhalla::valhalla_request_t request =
      get_request(get_request_str(parent_key, key, expected_values), action);
  validate(key, expected_values, (request.options.filter_attributes_size() > 0),
           request.options.filter_attributes());
}

void test_default_base_auto_cost_options(const valhalla::odin::Costing costing,
                                         const valhalla::odin::DirectionsOptions::Action action) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string key = "costing";

  // Get cost request with no cost options
  valhalla::valhalla_request_t request = get_request(get_request_str(key, costing_str), action);

  validate("maneuver_penalty", kDefaultAutoManeuverPenalty,
           request.options.costing_options(static_cast<int>(costing)).maneuver_penalty());
  validate("destination_only_penalty", kDefaultAutoDestinationOnlyPenalty,
           request.options.costing_options(static_cast<int>(costing)).destination_only_penalty());
  validate("gate_cost", kDefaultAutoGateCost,
           request.options.costing_options(static_cast<int>(costing)).gate_cost());
  validate("gate_penalty", kDefaultAutoGatePenalty,
           request.options.costing_options(static_cast<int>(costing)).gate_penalty());
  validate("toll_booth_cost", kDefaultAutoTollBoothCost,
           request.options.costing_options(static_cast<int>(costing)).toll_booth_cost());
  validate("toll_booth_penalty", kDefaultAutoTollBoothPenalty,
           request.options.costing_options(static_cast<int>(costing)).toll_booth_penalty());
  validate("alley_penalty", kDefaultAutoAlleyPenalty,
           request.options.costing_options(static_cast<int>(costing)).alley_penalty());
  validate("country_crossing_cost", kDefaultAutoCountryCrossingCost,
           request.options.costing_options(static_cast<int>(costing)).country_crossing_cost());
  validate("country_crossing_penalty", kDefaultAutoCountryCrossingPenalty,
           request.options.costing_options(static_cast<int>(costing)).country_crossing_penalty());
  validate("ferry_cost", kDefaultAutoFerryCost,
           request.options.costing_options(static_cast<int>(costing)).ferry_cost());
  validate("use_ferry", kDefaultAutoUseFerry,
           request.options.costing_options(static_cast<int>(costing)).use_ferry());
  validate("use_highways", kDefaultAutoUseHighways,
           request.options.costing_options(static_cast<int>(costing)).use_highways());
  validate("use_tolls", kDefaultAutoUseTolls,
           request.options.costing_options(static_cast<int>(costing)).use_tolls());
}

void test_default_motor_scooter_cost_options(const valhalla::odin::Costing costing,
                                             const valhalla::odin::DirectionsOptions::Action action) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string key = "costing";

  // Get cost request with no cost options
  valhalla::valhalla_request_t request = get_request(get_request_str(key, costing_str), action);

  validate("maneuver_penalty", kDefaultMotorScooterManeuverPenalty,
           request.options.costing_options(static_cast<int>(costing)).maneuver_penalty());
  validate("destination_only_penalty", kDefaultMotorScooterDestinationOnlyPenalty,
           request.options.costing_options(static_cast<int>(costing)).destination_only_penalty());
  validate("gate_cost", kDefaultMotorScooterGateCost,
           request.options.costing_options(static_cast<int>(costing)).gate_cost());
  validate("gate_penalty", kDefaultMotorScooterGatePenalty,
           request.options.costing_options(static_cast<int>(costing)).gate_penalty());
  validate("alley_penalty", kDefaultMotorScooterAlleyPenalty,
           request.options.costing_options(static_cast<int>(costing)).alley_penalty());
  validate("country_crossing_cost", kDefaultMotorScooterCountryCrossingCost,
           request.options.costing_options(static_cast<int>(costing)).country_crossing_cost());
  validate("country_crossing_penalty", kDefaultMotorScooterCountryCrossingPenalty,
           request.options.costing_options(static_cast<int>(costing)).country_crossing_penalty());
  validate("ferry_cost", kDefaultMotorScooterFerryCost,
           request.options.costing_options(static_cast<int>(costing)).ferry_cost());
  validate("use_ferry", kDefaultMotorScooterUseFerry,
           request.options.costing_options(static_cast<int>(costing)).use_ferry());
  validate("top_speed", static_cast<float>(kDefaultMotorScooterTopSpeed),
           request.options.costing_options(static_cast<int>(costing)).top_speed());
  validate("use_hills", kDefaultMotorScooterUseHills,
           request.options.costing_options(static_cast<int>(costing)).use_hills());
  validate("use_primary", kDefaultMotorScooterUsePrimary,
           request.options.costing_options(static_cast<int>(costing)).use_primary());
}

void test_default_motorcycle_cost_options(const valhalla::odin::Costing costing,
                                          const valhalla::odin::DirectionsOptions::Action action) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string key = "costing";

  // Get cost request with no cost options
  valhalla::valhalla_request_t request = get_request(get_request_str(key, costing_str), action);

  validate("maneuver_penalty", kDefaultMotorcycleManeuverPenalty,
           request.options.costing_options(static_cast<int>(costing)).maneuver_penalty());
  validate("destination_only_penalty", kDefaultMotorcycleDestinationOnlyPenalty,
           request.options.costing_options(static_cast<int>(costing)).destination_only_penalty());
  validate("gate_cost", kDefaultMotorcycleGateCost,
           request.options.costing_options(static_cast<int>(costing)).gate_cost());
  validate("gate_penalty", kDefaultMotorcycleGatePenalty,
           request.options.costing_options(static_cast<int>(costing)).gate_penalty());
  validate("alley_penalty", kDefaultMotorcycleAlleyPenalty,
           request.options.costing_options(static_cast<int>(costing)).alley_penalty());
  validate("country_crossing_cost", kDefaultMotorcycleCountryCrossingCost,
           request.options.costing_options(static_cast<int>(costing)).country_crossing_cost());
  validate("country_crossing_penalty", kDefaultMotorcycleCountryCrossingPenalty,
           request.options.costing_options(static_cast<int>(costing)).country_crossing_penalty());
  validate("ferry_cost", kDefaultMotorcycleFerryCost,
           request.options.costing_options(static_cast<int>(costing)).ferry_cost());
  validate("use_ferry", kDefaultMotorcycleUseFerry,
           request.options.costing_options(static_cast<int>(costing)).use_ferry());
  validate("use_primary", kDefaultMotorcycleUsePrimary,
           request.options.costing_options(static_cast<int>(costing)).use_primary());
  validate("use_trails", kDefaultMotorcycleUseTrails,
           request.options.costing_options(static_cast<int>(costing)).use_trails());
}

void test_default_pedestrian_cost_options(const valhalla::odin::Costing costing,
                                          const valhalla::odin::DirectionsOptions::Action action) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string key = "costing";

  // Get cost request with no cost options
  valhalla::valhalla_request_t request = get_request(get_request_str(key, costing_str), action);

  validate("type", "foot",
           request.options.costing_options(static_cast<int>(costing)).transport_type());
  validate("maneuver_penalty", kDefaultPedestrianManeuverPenalty,
           request.options.costing_options(static_cast<int>(costing)).maneuver_penalty());
  validate("gate_penalty", kDefaultPedestrianGatePenalty,
           request.options.costing_options(static_cast<int>(costing)).gate_penalty());
  validate("country_crossing_cost", kDefaultPedestrianCountryCrossingCost,
           request.options.costing_options(static_cast<int>(costing)).country_crossing_cost());
  validate("country_crossing_penalty", kDefaultPedestrianCountryCrossingPenalty,
           request.options.costing_options(static_cast<int>(costing)).country_crossing_penalty());
  validate("ferry_cost", kDefaultPedestrianFerryCost,
           request.options.costing_options(static_cast<int>(costing)).ferry_cost());
  validate("use_ferry", kDefaultPedestrianUseFerry,
           request.options.costing_options(static_cast<int>(costing)).use_ferry());
  validate("max_distance", kDefaultPedestrianMaxDistanceFoot,
           request.options.costing_options(static_cast<int>(costing)).max_distance());
  validate("walking_speed", kDefaultPedestrianSpeedFoot,
           request.options.costing_options(static_cast<int>(costing)).walking_speed());
  validate("walking_speed", kDefaultPedestrianSpeedFoot,
           request.options.costing_options(static_cast<int>(costing)).walking_speed());
  validate("step_penalty", kDefaultPedestrianStepPenaltyFoot,
           request.options.costing_options(static_cast<int>(costing)).step_penalty());
  validate("max_grade", kDefaultPedestrianMaxGradeFoot,
           request.options.costing_options(static_cast<int>(costing)).max_grade());
  validate("max_hiking_difficulty", kDefaultPedestrianMaxHikingDifficulty,
           request.options.costing_options(static_cast<int>(costing)).max_hiking_difficulty());
  validate("mode_factor", kDefaultPedestrianModeFactor,
           request.options.costing_options(static_cast<int>(costing)).mode_factor());
  validate("walkway_factor", kDefaultPedestrianWalkwayFactor,
           request.options.costing_options(static_cast<int>(costing)).walkway_factor());
  validate("sidewalk_factor", kDefaultPedestrianSideWalkFactor,
           request.options.costing_options(static_cast<int>(costing)).sidewalk_factor());
  validate("alley_factor", kDefaultPedestrianAlleyFactor,
           request.options.costing_options(static_cast<int>(costing)).alley_factor());
  validate("driveway_factor", kDefaultPedestrianDrivewayFactor,
           request.options.costing_options(static_cast<int>(costing)).driveway_factor());
  validate("transit_start_end_max_distance", kDefaultPedestrianTransitStartEndMaxDistance,
           request.options.costing_options(static_cast<int>(costing))
               .transit_start_end_max_distance());
  validate("transit_transfer_max_distance", kDefaultPedestrianTransitTransferMaxDistance,
           request.options.costing_options(static_cast<int>(costing))
               .transit_transfer_max_distance());
}

void test_transport_type_parsing(const valhalla::odin::Costing costing,
                                 const std::string& key,
                                 const std::string& specified_value,
                                 const std::string& expected_value,
                                 const valhalla::odin::DirectionsOptions::Action action =
                                     valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).transport_type());
}

void test_maneuver_penalty_parsing(const valhalla::odin::Costing costing,
                                   const float specified_value,
                                   const float expected_value,
                                   const valhalla::odin::DirectionsOptions::Action action =
                                       valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "maneuver_penalty";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).maneuver_penalty());
}

void test_destination_only_penalty_parsing(const valhalla::odin::Costing costing,
                                           const float specified_value,
                                           const float expected_value,
                                           const valhalla::odin::DirectionsOptions::Action action =
                                               valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "destination_only_penalty";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).destination_only_penalty());
}

void test_gate_cost_parsing(const valhalla::odin::Costing costing,
                            const float specified_value,
                            const float expected_value,
                            const valhalla::odin::DirectionsOptions::Action action =
                                valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "gate_cost";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).gate_cost());
}

void test_gate_penalty_parsing(const valhalla::odin::Costing costing,
                               const float specified_value,
                               const float expected_value,
                               const valhalla::odin::DirectionsOptions::Action action =
                                   valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "gate_penalty";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).gate_penalty());
}

void test_toll_booth_cost_parsing(const valhalla::odin::Costing costing,
                                  const float specified_value,
                                  const float expected_value,
                                  const valhalla::odin::DirectionsOptions::Action action =
                                      valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "toll_booth_cost";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).toll_booth_cost());
}

void test_toll_booth_penalty_parsing(const valhalla::odin::Costing costing,
                                     const float specified_value,
                                     const float expected_value,
                                     const valhalla::odin::DirectionsOptions::Action action =
                                         valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "toll_booth_penalty";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).toll_booth_penalty());
}

void test_alley_penalty_parsing(const valhalla::odin::Costing costing,
                                const float specified_value,
                                const float expected_value,
                                const valhalla::odin::DirectionsOptions::Action action =
                                    valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "alley_penalty";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).alley_penalty());
}

void test_country_crossing_cost_parsing(const valhalla::odin::Costing costing,
                                        const float specified_value,
                                        const float expected_value,
                                        const valhalla::odin::DirectionsOptions::Action action =
                                            valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "country_crossing_cost";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).country_crossing_cost());
}

void test_country_crossing_penalty_parsing(const valhalla::odin::Costing costing,
                                           const float specified_value,
                                           const float expected_value,
                                           const valhalla::odin::DirectionsOptions::Action action =
                                               valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "country_crossing_penalty";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).country_crossing_penalty());
}

void test_ferry_cost_parsing(const valhalla::odin::Costing costing,
                             const float specified_value,
                             const float expected_value,
                             const valhalla::odin::DirectionsOptions::Action action =
                                 valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "ferry_cost";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).ferry_cost());
}

void test_use_ferry_parsing(const valhalla::odin::Costing costing,
                            const float specified_value,
                            const float expected_value,
                            const valhalla::odin::DirectionsOptions::Action action =
                                valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "use_ferry";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).use_ferry());
}

void test_use_highways_parsing(const valhalla::odin::Costing costing,
                               const float specified_value,
                               const float expected_value,
                               const valhalla::odin::DirectionsOptions::Action action =
                                   valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "use_highways";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).use_highways());
}

void test_use_tolls_parsing(const valhalla::odin::Costing costing,
                            const float specified_value,
                            const float expected_value,
                            const valhalla::odin::DirectionsOptions::Action action =
                                valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "use_tolls";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).use_tolls());
}

void test_use_hills_parsing(const valhalla::odin::Costing costing,
                            const float specified_value,
                            const float expected_value,
                            const valhalla::odin::DirectionsOptions::Action action =
                                valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "use_hills";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).use_hills());
}

void test_use_primary_parsing(const valhalla::odin::Costing costing,
                              const float specified_value,
                              const float expected_value,
                              const valhalla::odin::DirectionsOptions::Action action =
                                  valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "use_primary";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).use_primary());
}

void test_top_speed_parsing(const valhalla::odin::Costing costing,
                            const float specified_value,
                            const float expected_value,
                            const valhalla::odin::DirectionsOptions::Action action =
                                valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "top_speed";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).top_speed());
}

void test_use_trails_parsing(const valhalla::odin::Costing costing,
                             const float specified_value,
                             const float expected_value,
                             const valhalla::odin::DirectionsOptions::Action action =
                                 valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "use_trails";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).use_trails());
}

void test_max_distance_parsing(const valhalla::odin::Costing costing,
                               const std::string& transport_type,
                               const uint32_t specified_value,
                               const uint32_t expected_value,
                               const valhalla::odin::DirectionsOptions::Action action =
                                   valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string sibling_key = "type";
  const std::string key = "max_distance";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, sibling_key, transport_type, key,
                                  specified_value),
                  action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).max_distance());
}

void test_walking_speed_parsing(const valhalla::odin::Costing costing,
                                const std::string& transport_type,
                                const float specified_value,
                                const float expected_value,
                                const valhalla::odin::DirectionsOptions::Action action =
                                    valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string sibling_key = "type";
  const std::string key = "walking_speed";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, sibling_key, transport_type, key,
                                  specified_value),
                  action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).walking_speed());
}

void test_step_penalty_parsing(const valhalla::odin::Costing costing,
                               const std::string& transport_type,
                               const float specified_value,
                               const float expected_value,
                               const valhalla::odin::DirectionsOptions::Action action =
                                   valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string sibling_key = "type";
  const std::string key = "step_penalty";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, sibling_key, transport_type, key,
                                  specified_value),
                  action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).step_penalty());
}

void test_max_grade_parsing(const valhalla::odin::Costing costing,
                            const std::string& transport_type,
                            const uint32_t specified_value,
                            const uint32_t expected_value,
                            const valhalla::odin::DirectionsOptions::Action action =
                                valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string sibling_key = "type";
  const std::string key = "max_grade";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, sibling_key, transport_type, key,
                                  specified_value),
                  action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).max_grade());
}

void test_max_hiking_difficulty_parsing(const valhalla::odin::Costing costing,
                                        const uint32_t specified_value,
                                        const uint32_t expected_value,
                                        const valhalla::odin::DirectionsOptions::Action action =
                                            valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "max_hiking_difficulty";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).max_hiking_difficulty());
}

void test_mode_factor_parsing(const valhalla::odin::Costing costing,
                              const float specified_value,
                              const float expected_value,
                              const valhalla::odin::DirectionsOptions::Action action =
                                  valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "mode_factor";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).mode_factor());
}

void test_walkway_factor_parsing(const valhalla::odin::Costing costing,
                                 const float specified_value,
                                 const float expected_value,
                                 const valhalla::odin::DirectionsOptions::Action action =
                                     valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "walkway_factor";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).walkway_factor());
}

void test_sidewalk_factor_parsing(const valhalla::odin::Costing costing,
                                  const float specified_value,
                                  const float expected_value,
                                  const valhalla::odin::DirectionsOptions::Action action =
                                      valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "sidewalk_factor";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).sidewalk_factor());
}

void test_alley_factor_parsing(const valhalla::odin::Costing costing,
                               const float specified_value,
                               const float expected_value,
                               const valhalla::odin::DirectionsOptions::Action action =
                                   valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "alley_factor";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).alley_factor());
}

void test_driveway_factor_parsing(const valhalla::odin::Costing costing,
                                  const float specified_value,
                                  const float expected_value,
                                  const valhalla::odin::DirectionsOptions::Action action =
                                      valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "driveway_factor";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing)).driveway_factor());
}

void test_transit_start_end_max_distance_parsing(
    const valhalla::odin::Costing costing,
    const uint32_t specified_value,
    const uint32_t expected_value,
    const valhalla::odin::DirectionsOptions::Action action =
        valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "transit_start_end_max_distance";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing))
               .transit_start_end_max_distance());
}

void test_transit_transfer_max_distance_parsing(
    const valhalla::odin::Costing costing,
    const uint32_t specified_value,
    const uint32_t expected_value,
    const valhalla::odin::DirectionsOptions::Action action =
        valhalla::odin::DirectionsOptions::route) {
  // Create the costing string
  auto costing_str = valhalla::odin::Costing_Name(costing);
  // Remove the trailing '_' from 'auto_' - this is a work around since 'auto' is a keyword
  if (costing_str.back() == '_') {
    costing_str.pop_back();
  }
  const std::string grandparent_key = "costing_options";
  const std::string parent_key = costing_str;
  const std::string key = "transit_transfer_max_distance";

  valhalla::valhalla_request_t request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options.costing_options(static_cast<int>(costing))
               .transit_transfer_max_distance());
}

///////////////////////////////////////////////////////////////////////////////
// test by key methods
void test_polygons() {
  test_polygons_parsing(true);
  test_polygons_parsing(false);
}

void test_denoise() {
  test_denoise_parsing(0.0f);
  test_denoise_parsing(0.5f);
  test_denoise_parsing(1.0f);
}

void test_generalize() {
  test_generalize_parsing(20.f);
  test_generalize_parsing(50.f);
}

void test_show_locations() {
  test_show_locations_parsing(true);
  test_show_locations_parsing(false);
}

void test_shape_match() {
  test_shape_match_parsing(valhalla::odin::ShapeMatch::map_snap,
                           valhalla::odin::DirectionsOptions::trace_route);
  test_shape_match_parsing(valhalla::odin::ShapeMatch::map_snap,
                           valhalla::odin::DirectionsOptions::trace_attributes);
  test_shape_match_parsing(valhalla::odin::ShapeMatch::edge_walk,
                           valhalla::odin::DirectionsOptions::trace_route);
  test_shape_match_parsing(valhalla::odin::ShapeMatch::edge_walk,
                           valhalla::odin::DirectionsOptions::trace_attributes);
}

void test_best_paths() {
  test_best_paths_parsing(1);
  test_best_paths_parsing(2);
  test_best_paths_parsing(4);
}

void test_gps_accuracy() {
  test_gps_accuracy_parsing(5.f);
  test_gps_accuracy_parsing(30.f);
}

void test_search_radius() {
  test_search_radius_parsing(10.f);
  test_search_radius_parsing(40.f);
}

void test_turn_penalty_factor() {
  test_turn_penalty_factor_parsing(50.f);
  test_turn_penalty_factor_parsing(100.f);
}

void test_filter_action() {
  test_filter_action_parsing(valhalla::odin::FilterAction::exclude);
  test_filter_action_parsing(valhalla::odin::FilterAction::include);
}

void test_filter_attributes() {
  test_filter_attributes_parsing({"edge.names", "edge.id", "edge.weighted_grade", "edge.speed"});
}

std::list<valhalla::odin::Costing> get_base_auto_costing_list() {
  return {valhalla::odin::Costing::auto_, valhalla::odin::Costing::auto_shorter,
          valhalla::odin::Costing::auto_data_fix, valhalla::odin::Costing::bus,
          valhalla::odin::Costing::hov};
}
void test_default_base_auto_cost_options() {
  for (auto costing : get_base_auto_costing_list()) {
    test_default_base_auto_cost_options(costing, valhalla::odin::DirectionsOptions::route);
  }
}

void test_default_motor_scooter_cost_options() {
  test_default_motor_scooter_cost_options(valhalla::odin::motor_scooter,
                                          valhalla::odin::DirectionsOptions::route);
}

void test_default_motorcycle_cost_options() {
  test_default_motorcycle_cost_options(valhalla::odin::motorcycle,
                                       valhalla::odin::DirectionsOptions::route);
}

void test_default_pedestrian_cost_options() {
  test_default_pedestrian_cost_options(valhalla::odin::pedestrian,
                                       valhalla::odin::DirectionsOptions::route);
}

void test_transport_type() {
  std::string transport_type_key = "type";
  std::string transport_type_value = "car";
  for (auto costing : get_base_auto_costing_list()) {
    test_transport_type_parsing(costing, transport_type_key, transport_type_value,
                                transport_type_value);
  }

  valhalla::odin::Costing costing = valhalla::odin::Costing::pedestrian;
  transport_type_value = "foot";
  test_transport_type_parsing(costing, transport_type_key, transport_type_value,
                              transport_type_value);
  transport_type_value = "wheelchair";
  test_transport_type_parsing(costing, transport_type_key, transport_type_value,
                              transport_type_value);
}

void test_maneuver_penalty() {
  float default_value = kDefaultAutoManeuverPenalty;
  for (auto costing : get_base_auto_costing_list()) {
    test_maneuver_penalty_parsing(costing, default_value, default_value);
    test_maneuver_penalty_parsing(costing, 2.f, 2.f);
    test_maneuver_penalty_parsing(costing, 30.f, 30.f);
    test_maneuver_penalty_parsing(costing, -2.f, default_value);
    test_maneuver_penalty_parsing(costing, 500000.f, default_value);
  }

  valhalla::odin::Costing costing = valhalla::odin::Costing::motor_scooter;
  default_value = kDefaultMotorScooterManeuverPenalty;
  test_maneuver_penalty_parsing(costing, default_value, default_value);
  test_maneuver_penalty_parsing(costing, 2.f, 2.f);
  test_maneuver_penalty_parsing(costing, 30.f, 30.f);
  test_maneuver_penalty_parsing(costing, -2.f, default_value);
  test_maneuver_penalty_parsing(costing, 500000.f, default_value);

  costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultMotorcycleManeuverPenalty;
  test_maneuver_penalty_parsing(costing, default_value, default_value);
  test_maneuver_penalty_parsing(costing, 2.f, 2.f);
  test_maneuver_penalty_parsing(costing, 30.f, 30.f);
  test_maneuver_penalty_parsing(costing, -2.f, default_value);
  test_maneuver_penalty_parsing(costing, 500000.f, default_value);

  costing = valhalla::odin::Costing::pedestrian;
  default_value = kDefaultPedestrianManeuverPenalty;
  test_maneuver_penalty_parsing(costing, default_value, default_value);
  test_maneuver_penalty_parsing(costing, 2.f, 2.f);
  test_maneuver_penalty_parsing(costing, 30.f, 30.f);
  test_maneuver_penalty_parsing(costing, -2.f, default_value);
  test_maneuver_penalty_parsing(costing, 500000.f, default_value);
}

void test_destination_only_penalty() {
  float default_value = kDefaultAutoDestinationOnlyPenalty;
  for (auto costing : get_base_auto_costing_list()) {
    test_destination_only_penalty_parsing(costing, default_value, default_value);
    test_destination_only_penalty_parsing(costing, 2.f, 2.f);
    test_destination_only_penalty_parsing(costing, 700.f, 700.f);
    test_destination_only_penalty_parsing(costing, -2.f, default_value);
    test_destination_only_penalty_parsing(costing, 500000.f, default_value);
  }

  valhalla::odin::Costing costing = valhalla::odin::Costing::motor_scooter;
  default_value = kDefaultMotorScooterDestinationOnlyPenalty;
  test_destination_only_penalty_parsing(costing, default_value, default_value);
  test_destination_only_penalty_parsing(costing, 2.f, 2.f);
  test_destination_only_penalty_parsing(costing, 700.f, 700.f);
  test_destination_only_penalty_parsing(costing, -2.f, default_value);
  test_destination_only_penalty_parsing(costing, 500000.f, default_value);

  costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultMotorcycleDestinationOnlyPenalty;
  test_destination_only_penalty_parsing(costing, default_value, default_value);
  test_destination_only_penalty_parsing(costing, 2.f, 2.f);
  test_destination_only_penalty_parsing(costing, 700.f, 700.f);
  test_destination_only_penalty_parsing(costing, -2.f, default_value);
  test_destination_only_penalty_parsing(costing, 500000.f, default_value);
}

void test_gate_cost() {
  float default_value = kDefaultAutoGateCost;
  for (auto costing : get_base_auto_costing_list()) {
    test_gate_cost_parsing(costing, default_value, default_value);
    test_gate_cost_parsing(costing, 2.f, 2.f);
    test_gate_cost_parsing(costing, 60.f, 60.f);
    test_gate_cost_parsing(costing, -2.f, default_value);
    test_gate_cost_parsing(costing, 500000.f, default_value);
  }

  valhalla::odin::Costing costing = valhalla::odin::Costing::motor_scooter;
  default_value = kDefaultMotorScooterGateCost;
  test_gate_cost_parsing(costing, default_value, default_value);
  test_gate_cost_parsing(costing, 2.f, 2.f);
  test_gate_cost_parsing(costing, 60.f, 60.f);
  test_gate_cost_parsing(costing, -2.f, default_value);
  test_gate_cost_parsing(costing, 500000.f, default_value);

  costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultMotorcycleGateCost;
  test_gate_cost_parsing(costing, default_value, default_value);
  test_gate_cost_parsing(costing, 2.f, 2.f);
  test_gate_cost_parsing(costing, 60.f, 60.f);
  test_gate_cost_parsing(costing, -2.f, default_value);
  test_gate_cost_parsing(costing, 500000.f, default_value);
}

void test_gate_penalty() {
  float default_value = kDefaultAutoGatePenalty;
  for (auto costing : get_base_auto_costing_list()) {
    test_gate_penalty_parsing(costing, default_value, default_value);
    test_gate_penalty_parsing(costing, 2.f, 2.f);
    test_gate_penalty_parsing(costing, 60.f, 60.f);
    test_gate_penalty_parsing(costing, -2.f, default_value);
    test_gate_penalty_parsing(costing, 500000.f, default_value);
  }

  valhalla::odin::Costing costing = valhalla::odin::Costing::motor_scooter;
  default_value = kDefaultMotorScooterGatePenalty;
  test_gate_penalty_parsing(costing, default_value, default_value);
  test_gate_penalty_parsing(costing, 2.f, 2.f);
  test_gate_penalty_parsing(costing, 60.f, 60.f);
  test_gate_penalty_parsing(costing, -2.f, default_value);
  test_gate_penalty_parsing(costing, 500000.f, default_value);

  costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultMotorcycleGatePenalty;
  test_gate_penalty_parsing(costing, default_value, default_value);
  test_gate_penalty_parsing(costing, 2.f, 2.f);
  test_gate_penalty_parsing(costing, 600.f, 600.f);
  test_gate_penalty_parsing(costing, -2.f, default_value);
  test_gate_penalty_parsing(costing, 500000.f, default_value);

  costing = valhalla::odin::Costing::pedestrian;
  default_value = kDefaultPedestrianGatePenalty;
  test_gate_penalty_parsing(costing, default_value, default_value);
  test_gate_penalty_parsing(costing, 5.f, 5.f);
  test_gate_penalty_parsing(costing, 20.f, 20.f);
  test_gate_penalty_parsing(costing, -2.f, default_value);
  test_gate_penalty_parsing(costing, 500000.f, default_value);
}

void test_toll_booth_cost() {
  float default_value = kDefaultAutoTollBoothCost;
  for (auto costing : get_base_auto_costing_list()) {
    test_toll_booth_cost_parsing(costing, default_value, default_value);
    test_toll_booth_cost_parsing(costing, 2.f, 2.f);
    test_toll_booth_cost_parsing(costing, 60.f, 60.f);
    test_toll_booth_cost_parsing(costing, -2.f, default_value);
    test_toll_booth_cost_parsing(costing, 500000.f, default_value);
  }

  valhalla::odin::Costing costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultMotorcycleTollBoothCost;
  test_toll_booth_cost_parsing(costing, default_value, default_value);
  test_toll_booth_cost_parsing(costing, 2.f, 2.f);
  test_toll_booth_cost_parsing(costing, 20.f, 20.f);
  test_toll_booth_cost_parsing(costing, -2.f, default_value);
  test_toll_booth_cost_parsing(costing, 500000.f, default_value);
}

void test_toll_booth_penalty() {
  float default_value = kDefaultAutoTollBoothPenalty;
  for (auto costing : get_base_auto_costing_list()) {
    test_toll_booth_penalty_parsing(costing, default_value, default_value);
    test_toll_booth_penalty_parsing(costing, 2.f, 2.f);
    test_toll_booth_penalty_parsing(costing, 60.f, 60.f);
    test_toll_booth_penalty_parsing(costing, -2.f, default_value);
    test_toll_booth_penalty_parsing(costing, 500000.f, default_value);
  }

  valhalla::odin::Costing costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultMotorcycleTollBoothPenalty;
  test_toll_booth_penalty_parsing(costing, default_value, default_value);
  test_toll_booth_penalty_parsing(costing, 2.f, 2.f);
  test_toll_booth_penalty_parsing(costing, 60.f, 60.f);
  test_toll_booth_penalty_parsing(costing, -2.f, default_value);
  test_toll_booth_penalty_parsing(costing, 500000.f, default_value);
}

void test_alley_penalty() {
  float default_value = kDefaultAutoAlleyPenalty;
  for (auto costing : get_base_auto_costing_list()) {
    test_alley_penalty_parsing(costing, default_value, default_value);
    test_alley_penalty_parsing(costing, 2.f, 2.f);
    test_alley_penalty_parsing(costing, 60.f, 60.f);
    test_alley_penalty_parsing(costing, -2.f, default_value);
    test_alley_penalty_parsing(costing, 500000.f, default_value);
  }

  valhalla::odin::Costing costing = valhalla::odin::Costing::motor_scooter;
  default_value = kDefaultMotorScooterAlleyPenalty;
  test_alley_penalty_parsing(costing, default_value, default_value);
  test_alley_penalty_parsing(costing, 2.f, 2.f);
  test_alley_penalty_parsing(costing, 60.f, 60.f);
  test_alley_penalty_parsing(costing, -2.f, default_value);
  test_alley_penalty_parsing(costing, 500000.f, default_value);

  costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultMotorcycleAlleyPenalty;
  test_alley_penalty_parsing(costing, default_value, default_value);
  test_alley_penalty_parsing(costing, 2.f, 2.f);
  test_alley_penalty_parsing(costing, 10.f, 10.f);
  test_alley_penalty_parsing(costing, -2.f, default_value);
  test_alley_penalty_parsing(costing, 500000.f, default_value);
}

void test_country_crossing_cost() {
  float default_value = kDefaultAutoCountryCrossingCost;
  for (auto costing : get_base_auto_costing_list()) {
    test_country_crossing_cost_parsing(costing, default_value, default_value);
    test_country_crossing_cost_parsing(costing, 2.f, 2.f);
    test_country_crossing_cost_parsing(costing, 60.f, 60.f);
    test_country_crossing_cost_parsing(costing, -2.f, default_value);
    test_country_crossing_cost_parsing(costing, 500000.f, default_value);
  }

  valhalla::odin::Costing costing = valhalla::odin::Costing::motor_scooter;
  default_value = kDefaultMotorScooterCountryCrossingCost;
  test_country_crossing_cost_parsing(costing, default_value, default_value);
  test_country_crossing_cost_parsing(costing, 2.f, 2.f);
  test_country_crossing_cost_parsing(costing, 700.f, 700.f);
  test_country_crossing_cost_parsing(costing, -2.f, default_value);
  test_country_crossing_cost_parsing(costing, 500000.f, default_value);

  costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultMotorcycleCountryCrossingCost;
  test_country_crossing_cost_parsing(costing, default_value, default_value);
  test_country_crossing_cost_parsing(costing, 2.f, 2.f);
  test_country_crossing_cost_parsing(costing, 700.f, 700.f);
  test_country_crossing_cost_parsing(costing, -2.f, default_value);
  test_country_crossing_cost_parsing(costing, 500000.f, default_value);

  costing = valhalla::odin::Costing::pedestrian;
  default_value = kDefaultPedestrianCountryCrossingCost;
  test_country_crossing_cost_parsing(costing, default_value, default_value);
  test_country_crossing_cost_parsing(costing, 2.f, 2.f);
  test_country_crossing_cost_parsing(costing, 700.f, 700.f);
  test_country_crossing_cost_parsing(costing, -2.f, default_value);
  test_country_crossing_cost_parsing(costing, 500000.f, default_value);
}

void test_country_crossing_penalty() {
  float default_value = kDefaultAutoCountryCrossingPenalty;
  for (auto costing : get_base_auto_costing_list()) {
    test_country_crossing_penalty_parsing(costing, default_value, default_value);
    test_country_crossing_penalty_parsing(costing, 2.f, 2.f);
    test_country_crossing_penalty_parsing(costing, 60.f, 60.f);
    test_country_crossing_penalty_parsing(costing, -2.f, default_value);
    test_country_crossing_penalty_parsing(costing, 500000.f, default_value);
  }

  valhalla::odin::Costing costing = valhalla::odin::Costing::motor_scooter;
  default_value = kDefaultMotorScooterCountryCrossingPenalty;
  test_country_crossing_penalty_parsing(costing, default_value, default_value);
  test_country_crossing_penalty_parsing(costing, 2.f, 2.f);
  test_country_crossing_penalty_parsing(costing, 60.f, 60.f);
  test_country_crossing_penalty_parsing(costing, -2.f, default_value);
  test_country_crossing_penalty_parsing(costing, 500000.f, default_value);

  costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultMotorcycleCountryCrossingPenalty;
  test_country_crossing_penalty_parsing(costing, default_value, default_value);
  test_country_crossing_penalty_parsing(costing, 2.f, 2.f);
  test_country_crossing_penalty_parsing(costing, 60.f, 60.f);
  test_country_crossing_penalty_parsing(costing, -2.f, default_value);
  test_country_crossing_penalty_parsing(costing, 500000.f, default_value);

  costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultPedestrianCountryCrossingPenalty;
  test_country_crossing_penalty_parsing(costing, default_value, default_value);
  test_country_crossing_penalty_parsing(costing, 2.f, 2.f);
  test_country_crossing_penalty_parsing(costing, 60.f, 60.f);
  test_country_crossing_penalty_parsing(costing, -2.f, default_value);
  test_country_crossing_penalty_parsing(costing, 500000.f, default_value);
}

void test_ferry_cost() {
  float default_value = kDefaultAutoFerryCost;
  for (auto costing : get_base_auto_costing_list()) {
    test_ferry_cost_parsing(costing, default_value, default_value);
    test_ferry_cost_parsing(costing, 2.f, 2.f);
    test_ferry_cost_parsing(costing, 600.f, 600.f);
    test_ferry_cost_parsing(costing, -2.f, default_value);
    test_ferry_cost_parsing(costing, 500000.f, default_value);
  }

  valhalla::odin::Costing costing = valhalla::odin::Costing::motor_scooter;
  default_value = kDefaultMotorScooterFerryCost;
  test_ferry_cost_parsing(costing, default_value, default_value);
  test_ferry_cost_parsing(costing, 2.f, 2.f);
  test_ferry_cost_parsing(costing, 600.f, 600.f);
  test_ferry_cost_parsing(costing, -2.f, default_value);
  test_ferry_cost_parsing(costing, 500000.f, default_value);

  costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultMotorcycleFerryCost;
  test_ferry_cost_parsing(costing, default_value, default_value);
  test_ferry_cost_parsing(costing, 2.f, 2.f);
  test_ferry_cost_parsing(costing, 600.f, 600.f);
  test_ferry_cost_parsing(costing, -2.f, default_value);
  test_ferry_cost_parsing(costing, 500000.f, default_value);

  costing = valhalla::odin::Costing::pedestrian;
  default_value = kDefaultPedestrianFerryCost;
  test_ferry_cost_parsing(costing, default_value, default_value);
  test_ferry_cost_parsing(costing, 2.f, 2.f);
  test_ferry_cost_parsing(costing, 600.f, 600.f);
  test_ferry_cost_parsing(costing, -2.f, default_value);
  test_ferry_cost_parsing(costing, 500000.f, default_value);
}

void test_use_ferry() {
  float default_value = kDefaultAutoUseFerry;
  for (auto costing : get_base_auto_costing_list()) {
    test_use_ferry_parsing(costing, default_value, default_value);
    test_use_ferry_parsing(costing, 0.2f, 0.2f);
    test_use_ferry_parsing(costing, 0.6f, 0.6f);
    test_use_ferry_parsing(costing, -2.f, default_value);
    test_use_ferry_parsing(costing, 2.f, default_value);
  }

  valhalla::odin::Costing costing = valhalla::odin::Costing::motor_scooter;
  default_value = kDefaultMotorScooterUseFerry;
  test_use_ferry_parsing(costing, default_value, default_value);
  test_use_ferry_parsing(costing, 0.2f, 0.2f);
  test_use_ferry_parsing(costing, 0.6f, 0.6f);
  test_use_ferry_parsing(costing, -2.f, default_value);
  test_use_ferry_parsing(costing, 2.f, default_value);

  costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultMotorcycleUseFerry;
  test_use_ferry_parsing(costing, default_value, default_value);
  test_use_ferry_parsing(costing, 0.2f, 0.2f);
  test_use_ferry_parsing(costing, 0.6f, 0.6f);
  test_use_ferry_parsing(costing, -2.f, default_value);
  test_use_ferry_parsing(costing, 2.f, default_value);

  costing = valhalla::odin::Costing::pedestrian;
  default_value = kDefaultPedestrianUseFerry;
  test_use_ferry_parsing(costing, default_value, default_value);
  test_use_ferry_parsing(costing, 0.2f, 0.2f);
  test_use_ferry_parsing(costing, 0.6f, 0.6f);
  test_use_ferry_parsing(costing, -2.f, default_value);
  test_use_ferry_parsing(costing, 2.f, default_value);
}

void test_use_highways() {
  float default_value = kDefaultAutoUseHighways;
  for (auto costing : get_base_auto_costing_list()) {
    test_use_highways_parsing(costing, default_value, default_value);
    test_use_highways_parsing(costing, 0.2f, 0.2f);
    test_use_highways_parsing(costing, 0.6f, 0.6f);
    test_use_highways_parsing(costing, -2.f, default_value);
    test_use_highways_parsing(costing, 2.f, default_value);
  }

  valhalla::odin::Costing costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultMotorcycleUseHighways;
  test_use_highways_parsing(costing, default_value, default_value);
  test_use_highways_parsing(costing, 0.2f, 0.2f);
  test_use_highways_parsing(costing, 0.6f, 0.6f);
  test_use_highways_parsing(costing, -2.f, default_value);
  test_use_highways_parsing(costing, 2.f, default_value);
}

void test_use_tolls() {
  float default_value = kDefaultAutoUseTolls;
  for (auto costing : get_base_auto_costing_list()) {
    test_use_tolls_parsing(costing, default_value, default_value);
    test_use_tolls_parsing(costing, 0.2f, 0.2f);
    test_use_tolls_parsing(costing, 0.6f, 0.6f);
    test_use_tolls_parsing(costing, -2.f, default_value);
    test_use_tolls_parsing(costing, 2.f, default_value);
  }

  valhalla::odin::Costing costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultMotorcycleUseTolls;
  test_use_tolls_parsing(costing, default_value, default_value);
  test_use_tolls_parsing(costing, 0.2f, 0.2f);
  test_use_tolls_parsing(costing, 0.6f, 0.6f);
  test_use_tolls_parsing(costing, -2.f, default_value);
  test_use_tolls_parsing(costing, 2.f, default_value);
}

void test_use_hills() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::motor_scooter;
  float default_value = kDefaultMotorScooterUseHills;
  test_use_hills_parsing(costing, default_value, default_value);
  test_use_hills_parsing(costing, 0.2f, 0.2f);
  test_use_hills_parsing(costing, 0.6f, 0.6f);
  test_use_hills_parsing(costing, -2.f, default_value);
  test_use_hills_parsing(costing, 2.f, default_value);
}

void test_use_primary() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::motor_scooter;
  float default_value = kDefaultMotorScooterUsePrimary;
  test_use_primary_parsing(costing, default_value, default_value);
  test_use_primary_parsing(costing, 0.2f, 0.2f);
  test_use_primary_parsing(costing, 0.6f, 0.6f);
  test_use_primary_parsing(costing, -2.f, default_value);
  test_use_primary_parsing(costing, 2.f, default_value);

  costing = valhalla::odin::Costing::motorcycle;
  default_value = kDefaultMotorcycleUsePrimary;
  test_use_primary_parsing(costing, default_value, default_value);
  test_use_primary_parsing(costing, 0.2f, 0.2f);
  test_use_primary_parsing(costing, 0.6f, 0.6f);
  test_use_primary_parsing(costing, -2.f, default_value);
  test_use_primary_parsing(costing, 2.f, default_value);
}

void test_top_speed() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::motor_scooter;
  float default_value = kDefaultMotorScooterTopSpeed;
  test_top_speed_parsing(costing, default_value, default_value);
  test_top_speed_parsing(costing, 25, 25);
  test_top_speed_parsing(costing, 50, 50);
  test_top_speed_parsing(costing, -2, default_value);
  test_top_speed_parsing(costing, 200, default_value);
}

void test_use_trails() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::motorcycle;
  float default_value = kDefaultMotorcycleUseTrails;
  test_use_trails_parsing(costing, default_value, default_value);
  test_use_trails_parsing(costing, 0.2f, 0.2f);
  test_use_trails_parsing(costing, 0.6f, 0.6f);
  test_use_trails_parsing(costing, -2.f, default_value);
  test_use_trails_parsing(costing, 2.f, default_value);
}

void test_max_distance() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::pedestrian;

  std::string transport_type = "foot";
  uint32_t default_value = kDefaultPedestrianMaxDistanceFoot;
  test_max_distance_parsing(costing, transport_type, default_value, default_value);
  test_max_distance_parsing(costing, transport_type, 50, 50);
  test_max_distance_parsing(costing, transport_type, 200000, default_value);

  transport_type = "wheelchair";
  default_value = kDefaultPedestrianMaxDistanceWheelchair;
  test_max_distance_parsing(costing, transport_type, default_value, default_value);
  test_max_distance_parsing(costing, transport_type, 50, 50);
  test_max_distance_parsing(costing, transport_type, 200000, default_value);
}

void test_walking_speed() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::pedestrian;

  std::string transport_type = "foot";
  float default_value = kDefaultPedestrianSpeedFoot;
  test_walking_speed_parsing(costing, transport_type, default_value, default_value);
  test_walking_speed_parsing(costing, transport_type, 2.5f, 2.5f);
  test_walking_speed_parsing(costing, transport_type, 6.f, 6.f);
  test_walking_speed_parsing(costing, transport_type, -2.f, default_value);
  test_walking_speed_parsing(costing, transport_type, 50.f, default_value);

  transport_type = "wheelchair";
  default_value = kDefaultPedestrianSpeedWheelchair;
  test_walking_speed_parsing(costing, transport_type, default_value, default_value);
  test_walking_speed_parsing(costing, transport_type, 2.f, 2.f);
  test_walking_speed_parsing(costing, transport_type, 5.f, 5.f);
  test_walking_speed_parsing(costing, transport_type, -2.f, default_value);
  test_walking_speed_parsing(costing, transport_type, 50.f, default_value);
}

void test_step_penalty() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::pedestrian;

  std::string transport_type = "foot";
  float default_value = kDefaultPedestrianStepPenaltyFoot;
  test_step_penalty_parsing(costing, transport_type, default_value, default_value);
  test_step_penalty_parsing(costing, transport_type, 10.f, 10.f);
  test_step_penalty_parsing(costing, transport_type, 40.f, 40.f);
  test_step_penalty_parsing(costing, transport_type, -2.f, default_value);
  test_step_penalty_parsing(costing, transport_type, 500000.f, default_value);

  transport_type = "wheelchair";
  default_value = kDefaultPedestrianStepPenaltyWheelchair;
  test_step_penalty_parsing(costing, transport_type, default_value, default_value);
  test_step_penalty_parsing(costing, transport_type, 400.f, 400.f);
  test_step_penalty_parsing(costing, transport_type, 800.f, 800.f);
  test_step_penalty_parsing(costing, transport_type, -2.f, default_value);
  test_step_penalty_parsing(costing, transport_type, 500000.f, default_value);
}

void test_max_grade() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::pedestrian;

  std::string transport_type = "foot";
  uint32_t default_value = kDefaultPedestrianMaxGradeFoot;
  test_max_grade_parsing(costing, transport_type, default_value, default_value);
  test_max_grade_parsing(costing, transport_type, 30, 30);
  test_max_grade_parsing(costing, transport_type, 100, default_value);

  transport_type = "wheelchair";
  default_value = kDefaultPedestrianMaxGradeWheelchair;
  test_max_grade_parsing(costing, transport_type, default_value, default_value);
  test_max_grade_parsing(costing, transport_type, 10, 10);
  test_max_grade_parsing(costing, transport_type, 14, 14);
  test_max_grade_parsing(costing, transport_type, 100, default_value);
}

void test_max_hiking_difficulty() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::pedestrian;
  float default_value = kDefaultPedestrianMaxHikingDifficulty;
  test_max_hiking_difficulty_parsing(costing, default_value, default_value);
  test_max_hiking_difficulty_parsing(costing, 3, 3);
  test_max_hiking_difficulty_parsing(costing, 10, default_value);
}

void test_mode_factor() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::pedestrian;
  float default_value = kDefaultPedestrianModeFactor;
  test_mode_factor_parsing(costing, default_value, default_value);
  test_mode_factor_parsing(costing, 1.f, 1.f);
  test_mode_factor_parsing(costing, 10.f, 10.f);
  test_mode_factor_parsing(costing, -2.f, default_value);
  test_mode_factor_parsing(costing, 200000.f, default_value);
}

void test_walkway_factor() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::pedestrian;
  float default_value = kDefaultPedestrianWalkwayFactor;
  test_walkway_factor_parsing(costing, default_value, default_value);
  test_walkway_factor_parsing(costing, 0.5f, 0.5f);
  test_walkway_factor_parsing(costing, 10.f, 10.f);
  test_walkway_factor_parsing(costing, -2.f, default_value);
  test_walkway_factor_parsing(costing, 200000.f, default_value);
}

void test_sidewalk_factor() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::pedestrian;
  float default_value = kDefaultPedestrianSideWalkFactor;
  test_sidewalk_factor_parsing(costing, default_value, default_value);
  test_sidewalk_factor_parsing(costing, 0.5f, 0.5f);
  test_sidewalk_factor_parsing(costing, 10.f, 10.f);
  test_sidewalk_factor_parsing(costing, -2.f, default_value);
  test_sidewalk_factor_parsing(costing, 200000.f, default_value);
}

void test_alley_factor() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::pedestrian;
  float default_value = kDefaultPedestrianAlleyFactor;
  test_alley_factor_parsing(costing, default_value, default_value);
  test_alley_factor_parsing(costing, 1.f, 1.f);
  test_alley_factor_parsing(costing, 10.f, 10.f);
  test_alley_factor_parsing(costing, -2.f, default_value);
  test_alley_factor_parsing(costing, 200000.f, default_value);
}

void test_driveway_factor() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::pedestrian;
  float default_value = kDefaultPedestrianDrivewayFactor;
  test_driveway_factor_parsing(costing, default_value, default_value);
  test_driveway_factor_parsing(costing, 1.f, 1.f);
  test_driveway_factor_parsing(costing, 10.f, 10.f);
  test_driveway_factor_parsing(costing, -2.f, default_value);
  test_driveway_factor_parsing(costing, 200000.f, default_value);
}

void test_transit_start_end_max_distance() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::pedestrian;
  float default_value = kDefaultPedestrianTransitStartEndMaxDistance;
  test_transit_start_end_max_distance_parsing(costing, default_value, default_value);
  test_transit_start_end_max_distance_parsing(costing, 3000, 3000);
  test_transit_start_end_max_distance_parsing(costing, 200000, default_value);
}

void test_transit_transfer_max_distance() {
  valhalla::odin::Costing costing = valhalla::odin::Costing::pedestrian;
  float default_value = kDefaultPedestrianTransitTransferMaxDistance;
  test_transit_transfer_max_distance_parsing(costing, default_value, default_value);
  test_transit_transfer_max_distance_parsing(costing, 1500, 1500);
  test_transit_transfer_max_distance_parsing(costing, 100000, default_value);
}

} // namespace

int main() {
  test::suite suite("parse_request");

  // TODO repeated Contour contours

  // polygons
  suite.test(TEST_CASE(test_polygons));

  // denoise
  suite.test(TEST_CASE(test_denoise));

  // generalize
  suite.test(TEST_CASE(test_generalize));

  // show_locations
  suite.test(TEST_CASE(test_show_locations));

  // shape_match
  suite.test(TEST_CASE(test_shape_match));

  // best_paths
  suite.test(TEST_CASE(test_best_paths));

  // gps_accuracy
  suite.test(TEST_CASE(test_gps_accuracy));

  // search_radius
  suite.test(TEST_CASE(test_search_radius));

  // turn_penalty_factor
  suite.test(TEST_CASE(test_turn_penalty_factor));

  // filter_action
  suite.test(TEST_CASE(test_filter_action));

  // filter_attributes
  suite.test(TEST_CASE(test_filter_attributes));

  /////////////////////////////////////////////////////////////////////////////
  // CostingOptions

  // default auto cost options
  suite.test(TEST_CASE(test_default_base_auto_cost_options));

  // default motor_scooter cost options
  suite.test(TEST_CASE(test_default_motor_scooter_cost_options));

  // default motorcycle cost options
  suite.test(TEST_CASE(test_default_motorcycle_cost_options));

  // default pedestrian cost options
  suite.test(TEST_CASE(test_default_pedestrian_cost_options));

  // transport_type
  suite.test(TEST_CASE(test_transport_type));

  // maneuver_penalty
  suite.test(TEST_CASE(test_maneuver_penalty));

  // destination_only_penalty
  suite.test(TEST_CASE(test_destination_only_penalty));

  // gate_cost
  suite.test(TEST_CASE(test_gate_cost));

  // gate_penalty
  suite.test(TEST_CASE(test_gate_penalty));

  // toll_booth_cost
  suite.test(TEST_CASE(test_toll_booth_cost));

  // toll_booth_penalty
  suite.test(TEST_CASE(test_toll_booth_penalty));

  // alley_penalty
  suite.test(TEST_CASE(test_alley_penalty));

  // country_crossing_cost
  suite.test(TEST_CASE(test_country_crossing_cost));

  // country_crossing_penalty
  suite.test(TEST_CASE(test_country_crossing_penalty));

  // ferry_cost
  suite.test(TEST_CASE(test_ferry_cost));

  // use_ferry
  suite.test(TEST_CASE(test_use_ferry));

  // use_highways
  suite.test(TEST_CASE(test_use_highways));

  // use_tolls
  suite.test(TEST_CASE(test_use_tolls));

  // use_hills
  suite.test(TEST_CASE(test_use_hills));

  // use_primary
  suite.test(TEST_CASE(test_use_primary));

  // top_speed
  suite.test(TEST_CASE(test_top_speed));

  // use_trails
  suite.test(TEST_CASE(test_use_trails));

  // max_distance
  suite.test(TEST_CASE(test_max_distance));

  // step_penalty
  suite.test(TEST_CASE(test_step_penalty));

  // walking_speed
  suite.test(TEST_CASE(test_walking_speed));

  // max_grade
  suite.test(TEST_CASE(test_max_grade));

  // max_hiking_difficulty
  suite.test(TEST_CASE(test_max_hiking_difficulty));

  // mode_factor
  suite.test(TEST_CASE(test_mode_factor));

  // walkway_factor
  suite.test(TEST_CASE(test_walkway_factor));

  // sidewalk_factor
  suite.test(TEST_CASE(test_sidewalk_factor));

  // alley_factor
  suite.test(TEST_CASE(test_alley_factor));

  // driveway_factor
  suite.test(TEST_CASE(test_driveway_factor));

  // transit_start_end_max_distance
  suite.test(TEST_CASE(test_transit_start_end_max_distance));

  // transit_transfer_max_distance
  suite.test(TEST_CASE(test_transit_transfer_max_distance));

  return suite.tear_down();
}
