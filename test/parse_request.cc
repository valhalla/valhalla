#include <iostream>
#include <string>
#include <vector>

#include "proto/options.pb.h"
#include "proto_conversions.h"
#include "sif/costconstants.h"
#include "worker.h"

#include "test.h"

using namespace valhalla;

namespace {

// Base defaults
constexpr float kDefaultClosureFactor = 9.0f;

// Auto defaults
constexpr float kDefaultAuto_ManeuverPenalty = 5.0f;          // Seconds
constexpr float kDefaultAuto_DestinationOnlyPenalty = 600.0f; // Seconds
constexpr float kDefaultAuto_AlleyPenalty = 5.0f;             // Seconds
constexpr float kDefaultAuto_GateCost = 30.0f;                // Seconds
constexpr float kDefaultAuto_GatePenalty = 300.0f;            // Seconds
constexpr float kDefaultAuto_PrivateAccessPenalty = 450.0f;   // Seconds
constexpr float kDefaultAuto_TollBoothCost = 15.0f;           // Seconds
constexpr float kDefaultAuto_TollBoothPenalty = 0.0f;         // Seconds
constexpr float kDefaultAuto_FerryCost = 300.0f;              // Seconds
constexpr float kDefaultAuto_CountryCrossingCost = 600.0f;    // Seconds
constexpr float kDefaultAuto_CountryCrossingPenalty = 0.0f;   // Seconds
constexpr float kDefaultAuto_UseFerry = 0.5f;                 // Factor between 0 and 1
constexpr float kDefaultAuto_UseHighways = 0.5f;              // Factor between 0 and 1
constexpr float kDefaultAuto_UseTolls = 0.5f;                 // Factor between 0 and 1
constexpr float kDefaultAuto_UseTracks = 0.f;                 // Factor between 0 and 1
constexpr float kDefaultAuto_UseLivingStreets = 0.1f;         // Factor between 0 and 1
constexpr float kDefaultAuto_ServicePenalty = 75.0f;          // Seconds
constexpr float kDefaultAuto_ServiceFactor = 1.f;             // Positive factor

// Motor Scooter defaults
constexpr float kDefaultMotorScooter_ManeuverPenalty = 5.0f;          // Seconds
constexpr float kDefaultMotorScooter_AlleyPenalty = 5.0f;             // Seconds
constexpr float kDefaultMotorScooter_GateCost = 30.0f;                // Seconds
constexpr float kDefaultMotorScooter_GatePenalty = 300.0f;            // Seconds
constexpr float kDefaultMotorScooter_PrivateAccessPenalty = 450.0f;   // Seconds
constexpr float kDefaultMotorScooter_FerryCost = 300.0f;              // Seconds
constexpr float kDefaultMotorScooter_CountryCrossingCost = 600.0f;    // Seconds
constexpr float kDefaultMotorScooter_CountryCrossingPenalty = 0.0f;   // Seconds
constexpr float kDefaultMotorScooter_UseFerry = 0.5f;                 // Factor between 0 and 1
constexpr float kDefaultMotorScooter_DestinationOnlyPenalty = 120.0f; // Seconds
constexpr float kDefaultMotorScooter_UseHills = 0.5f;                 // Factor between 0 and 1
constexpr float kDefaultMotorScooter_UsePrimary = 0.5f;               // Factor between 0 and 1
constexpr float kDefaultMotorScooter_UseLivingStreets = 0.1f;         // Factor between 0 and 1
constexpr uint32_t kDefaultMotorScooter_TopSpeed = 45;                // Kilometers per hour
constexpr float kDefaultMotorScooter_ServicePenalty = 15.0f;          // Seconds
constexpr float kDefaultMotorScooter_ServiceFactor = 1.f;             // Positive factor

// Motorcycle defaults
constexpr float kDefaultMotorcycle_ManeuverPenalty = 5.0f;          // Seconds
constexpr float kDefaultMotorcycle_AlleyPenalty = 5.0f;             // Seconds
constexpr float kDefaultMotorcycle_GateCost = 30.0f;                // Seconds
constexpr float kDefaultMotorcycle_GatePenalty = 300.0f;            // Seconds
constexpr float kDefaultMotorcycle_PrivateAccessPenalty = 450.0f;   // Seconds
constexpr float kDefaultMotorcycle_TollBoothCost = 15.0f;           // Seconds
constexpr float kDefaultMotorcycle_TollBoothPenalty = 0.0f;         // Seconds
constexpr float kDefaultMotorcycle_FerryCost = 300.0f;              // Seconds
constexpr float kDefaultMotorcycle_CountryCrossingCost = 600.0f;    // Seconds
constexpr float kDefaultMotorcycle_CountryCrossingPenalty = 0.0f;   // Seconds
constexpr float kDefaultMotorcycle_UseFerry = 0.5f;                 // Factor between 0 and 1
constexpr float kDefaultMotorcycle_UseHighways = 0.5f;              // Factor between 0 and 1
constexpr float kDefaultMotorcycle_UseTolls = 0.5f;                 // Factor between 0 and 1
constexpr float kDefaultMotorcycle_UseTrails = 0.0f;                // Factor between 0 and 1
constexpr float kDefaultMotorcycle_UseLivingStreets = 0.1f;         // Factor between 0 and 1
constexpr float kDefaultMotorcycle_DestinationOnlyPenalty = 600.0f; // Seconds
constexpr float kDefaultMotorcycle_ServicePenalty = 15.0f;          // Seconds
constexpr float kDefaultMotorcycle_ServiceFactor = 1.f;             // Positive factor

// Pedestrian defaults
constexpr uint32_t kDefaultPedestrian_MaxDistanceFoot = 100000;      // 100 km
constexpr uint32_t kDefaultPedestrian_MaxDistanceWheelchair = 10000; // 10 km
constexpr float kDefaultPedestrian_SpeedFoot = 5.1f;                 // 3.16 MPH
constexpr float kDefaultPedestrian_SpeedWheelchair = 4.0f;           // 2.5  MPH  TODO
constexpr float kDefaultPedestrian_StepPenaltyFoot = 30.0f;          // 30 seconds
constexpr float kDefaultPedestrian_StepPenaltyWheelchair = 600.0f;   // 10 minutes
constexpr uint32_t kDefaultPedestrian_MaxGradeFoot = 90;
constexpr uint32_t kDefaultPedestrian_MaxGradeWheelchair = 12;    // Conservative for now...
constexpr uint8_t kDefaultPedestrian_MaxHikingDifficulty = 1;     // T1 (kHiking)
constexpr float kDefaultPedestrian_ModeFactor = 1.5f;             // Favor this mode?
constexpr float kDefaultPedestrian_ManeuverPenalty = 5.0f;        // Seconds
constexpr float kDefaultPedestrian_GatePenalty = 10.0f;           // Seconds
constexpr float kDefaultPedestrian_WalkwayFactor = 1.0f;          // Neutral value for walkways
constexpr float kDefaultPedestrian_SideWalkFactor = 1.0f;         // Neutral value for sidewalks
constexpr float kDefaultPedestrian_AlleyFactor = 2.0f;            // Avoid alleys
constexpr float kDefaultPedestrian_DrivewayFactor = 5.0f;         // Avoid driveways
constexpr float kDefaultPedestrian_FerryCost = 300.0f;            // Seconds
constexpr float kDefaultPedestrian_CountryCrossingCost = 600.0f;  // Seconds
constexpr float kDefaultPedestrian_CountryCrossingPenalty = 0.0f; // Seconds
constexpr float kDefaultPedestrian_UseFerry = 1.0f;
constexpr float kDefaultPedestrian_UseLivingStreets = 0.6f;              // Factor between 0 and 1
constexpr uint32_t kDefaultPedestrian_TransitStartEndMaxDistance = 2415; // 1.5 miles
constexpr uint32_t kDefaultPedestrian_TransitTransferMaxDistance = 805;  // 0.5 miles
constexpr float kDefaultPedestrian_ServicePenalty = 0.0f;                // Seconds
constexpr float kDefaultPedestrian_ServiceFactor = 1.f;                  // Positive factor

// Bicycle defaults
constexpr float kDefaultBicycle_ManeuverPenalty = 5.0f;        // Seconds
constexpr float kDefaultBicycle_AlleyPenalty = 60.0f;          // Seconds
constexpr float kDefaultBicycle_GateCost = 30.0f;              // Seconds
constexpr float kDefaultBicycle_GatePenalty = 300.0f;          // Seconds
constexpr float kDefaultBicycle_PrivateAccessPenalty = 450.0f; // Seconds
constexpr float kDefaultBicycle_FerryCost = 300.0f;            // Seconds
constexpr float kDefaultBicycle_CountryCrossingCost = 600.0f;  // Seconds
constexpr float kDefaultBicycle_CountryCrossingPenalty = 0.0f; // Seconds
constexpr float kDefaultBicycle_UseRoad = 0.25f;               // Factor between 0 and 1
constexpr float kDefaultBicycle_UseFerry = 0.5f;               // Factor between 0 and 1
constexpr float kDefaultBicycle_UseHills = 0.25f;
constexpr float kDefaultBicycle_AvoidBadSurfaces = 0.25f; // Factor between 0 and 1
constexpr float kDefaultBicycle_UseLivingStreets = 0.5f;  // Factor between 0 and 1
constexpr float kDefaultBicycle_ServicePenalty = 15.0f;   // Seconds
const std::string kDefaultBicycle_BicycleType = "Hybrid"; // Bicycle type
constexpr float kDefaultBicycle_CyclingSpeed[] = {
    25.0f, // Road bicycle: ~15.5 MPH
    20.0f, // Cross bicycle: ~13 MPH
    18.0f, // Hybrid or "city" bicycle: ~11.5 MPH
    16.0f  // Mountain bicycle: ~10 MPH
};

// Truck defaults
constexpr float kDefaultTruck_ManeuverPenalty = 5.0f;          // Seconds
constexpr float kDefaultTruck_DestinationOnlyPenalty = 600.0f; // Seconds
constexpr float kDefaultTruck_AlleyPenalty = 5.0f;             // Seconds
constexpr float kDefaultTruck_GateCost = 30.0f;                // Seconds
constexpr float kDefaultTruck_GatePenalty = 300.0f;            // Seconds
constexpr float kDefaultTruck_PrivateAccessPenalty = 450.0f;   // Seconds
constexpr float kDefaultTruck_TollBoothCost = 15.0f;           // Seconds
constexpr float kDefaultTruck_TollBoothPenalty = 0.0f;         // Seconds
constexpr float kDefaultTruck_CountryCrossingCost = 600.0f;    // Seconds
constexpr float kDefaultTruck_CountryCrossingPenalty = 0.0f;   // Seconds
constexpr float kDefaultTruck_LowClassPenalty = 30.0f;         // Seconds
constexpr float kDefaultTruck_TruckWeight = 21.77f;            // Metric Tons (48,000 lbs)
constexpr float kDefaultTruck_TruckAxleLoad = 9.07f;           // Metric Tons (20,000 lbs)
constexpr float kDefaultTruck_TruckHeight = 4.11f;             // Meters (13 feet 6 inches)
constexpr float kDefaultTruck_TruckWidth = 2.6f;               // Meters (102.36 inches)
constexpr float kDefaultTruck_TruckLength = 21.64f;            // Meters (71 feet)
constexpr float kDefaultTruck_UseTracks = 0.f;                 // Factor between 0 and 1
constexpr float kDefaultTruck_UseLivingStreets = 0.f;          // Factor between 0 and 1
constexpr float kDefaultTruck_ServicePenalty = 0.0f;           // Seconds
constexpr float kDefaultTruck_ServiceFactor = 1.f;             // Positive factor

// Transit defaults
constexpr float kDefaultTransit_ModeFactor = 1.0f; // Favor this mode?
constexpr float kDefaultTransit_TransferCost = 15.0f;
constexpr float kDefaultTransit_TransferPenalty = 300.0f;
constexpr float kDefaultTransit_UseBus = 0.3f;
constexpr float kDefaultTransit_UseRail = 0.6f;
constexpr float kDefaultTransit_UseTransfers = 0.3f;

///////////////////////////////////////////////////////////////////////////////
// validate by type methods
void validate(const std::string& key, const bool expected_value, const bool pbf_value) {
  EXPECT_EQ(pbf_value, expected_value) << "incorrect " << key;
}

void validate(const std::string& key,
              const bool expected_value,
              const bool has_pbf_value,
              const bool pbf_value) {
  ASSERT_TRUE(has_pbf_value) << "bool value not found in pbf for key=" + key;
  EXPECT_EQ(pbf_value, expected_value) << "incorrect " << key;
}

void validate(const std::string& key, const float expected_value, const float pbf_value) {
  EXPECT_EQ(pbf_value, expected_value) << "incorrect " << key;
}

void validate(const std::string& key,
              const float expected_value,
              const bool has_pbf_value,
              const float pbf_value) {
  ASSERT_TRUE(has_pbf_value) << "float value not found in pbf for key=" + key;
  EXPECT_EQ(pbf_value, expected_value) << "incorrect " << key;
}

void validate(const std::string& key, const uint32_t expected_value, const uint32_t pbf_value) {
  EXPECT_EQ(pbf_value, expected_value) << "incorrect " << key;
}

void validate(const std::string& key,
              const uint32_t expected_value,
              const bool has_pbf_value,
              const uint32_t pbf_value) {
  ASSERT_TRUE(has_pbf_value) << "uint32_t value not found in pbf for key=" + key;
  EXPECT_EQ(pbf_value, expected_value) << "incorrect " << key;
}

void validate(const std::string& key,
              const std::string& expected_value,
              const std::string& pbf_value) {
  EXPECT_EQ(pbf_value, expected_value) << "incorrect " << key;
}

void validate(const std::string& key,
              const std::string& expected_value,
              const bool has_pbf_value,
              const std::string& pbf_value) {

  ASSERT_TRUE(has_pbf_value) << "string value not found in pbf for key=" + key;
  EXPECT_EQ(pbf_value, expected_value) << "incorrect " << key;
}

void validate(const std::string& key,
              const ShapeMatch expected_value,
              const bool has_pbf_value,
              const ShapeMatch pbf_value) {
  ASSERT_TRUE(has_pbf_value) << "ShapeMatch value not found in pbf for key=" + key;
  EXPECT_EQ(pbf_value, expected_value)
      << "Incorrect " + key + " ShapeMatch | expected_value=" + ShapeMatch_Enum_Name(expected_value) +
             " | found=" + ShapeMatch_Enum_Name(pbf_value);
}

void validate(const std::string& key,
              const valhalla::FilterAction expected_value,
              const bool has_pbf_value,
              const valhalla::FilterAction pbf_value) {
  ASSERT_TRUE(has_pbf_value) << "FilterAction value not found in pbf for key=" + key;

  EXPECT_EQ(pbf_value, expected_value)
      << "Incorrect " + key +
             " FilterAction | expected_value=" + FilterAction_Enum_Name(expected_value) +
             " | found=" + FilterAction_Enum_Name(pbf_value);
}

void validate(const std::string& key,
              const std::vector<std::string>& expected_values,
              const bool has_pbf_values,
              const google::protobuf::RepeatedPtrField<std::string>& pbf_values) {
  ASSERT_TRUE(has_pbf_values) << "string values not found in pbf for key=" + key;

  ASSERT_EQ(expected_values.size(), pbf_values.size()) << "invalid count in pbf for key=" + key;

  for (size_t i = 0; i < expected_values.size(); ++i) {
    ASSERT_EQ(pbf_values.Get(i), expected_values.at(i)) << "incorrect " << key;
  }
}

///////////////////////////////////////////////////////////////////////////////
// get request methods
std::string get_request_str(const std::string& key, const bool expected_value) {
  return R"({")" + key + R"(":)" + std::string(expected_value ? "true" : "false") + R"(})";
}

std::string get_request_str(const std::string& grandparent_key,
                            const std::string& parent_key,
                            const std::string& key,
                            const bool specified_value) {
  return R"({")" + grandparent_key + R"(":{")" + parent_key + R"(":{")" + key + R"(":)" +
         std::string(specified_value ? "true" : "false") + R"(}}})";
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
  return R"({")" + grandparent_key + R"(":{")" + parent_key + R"(":{")" + sibling_key + R"(":")" +
         sibling_value + R"(",")" + key + R"(":)" + std::to_string(specified_value) + R"(}}})";
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
  return R"({")" + grandparent_key + R"(":{")" + parent_key + R"(":{")" + sibling_key + R"(":")" +
         sibling_value + R"(",")" + key + R"(":)" + std::to_string(specified_value) + R"(}}})";
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

std::string get_request_str(const std::string& key, const ShapeMatch expected_value) {
  return R"({")" + key + R"(":")" + ShapeMatch_Enum_Name(expected_value) + R"("})";
}

std::string get_kv_str(const std::string& key, const valhalla::FilterAction value) {
  return R"(")" + key + R"(":")" + FilterAction_Enum_Name(value) + R"(")";
}

std::string get_request_str(const std::string& parent_key,
                            const std::string& key,
                            const valhalla::FilterAction expected_value) {
  return R"({")" + parent_key + R"(":{)" + get_kv_str(key, expected_value) + R"(}})";
}

std::string get_kv_str(const std::string& key, const std::vector<std::string>& values) {
  std::string kv_str = R"(")" + key + R"(":[)";
  bool first = true;
  for (const auto& value : values) {
    if (first) {
      kv_str += R"(")";
      first = false;
    } else {
      kv_str += R"(,")";
    }
    kv_str += value + R"(")";
  }
  kv_str += R"(])";
  return kv_str;
}

std::string get_request_str(const std::string& parent_key,
                            const std::string& key,
                            const std::vector<std::string>& expected_values) {
  return R"({")" + parent_key + R"(":{)" + get_kv_str(key, expected_values) + R"(}})";
}

std::string get_filter_request_str(const std::string& costing,
                                   const std::string& filter_type,
                                   const valhalla::FilterAction filter_action,
                                   const std::vector<std::string>& filter_ids) {
  return R"({"costing_options":{")" + costing + R"(":{"filters":{")" + filter_type + R"(":{)" +
         get_kv_str("action", filter_action) + R"(,)" + get_kv_str("ids", filter_ids) + R"(}}}}})";
}

Api get_request(const std::string& request_str, const Options::Action action) {
  // std::cout << ">>>>> request_str=" << request_str << "<<<<<" << std::endl;
  Api request;
  ParseApi(request_str, action, request);
  return request;
}

///////////////////////////////////////////////////////////////////////////////
// test parsing methods
std::string get_costing_str(Costing costing) {
  // Create the costing string
  auto costing_str = Costing_Enum_Name(costing);
  return costing_str;
}

void test_polygons_parsing(const bool expected_value,
                           const Options::Action action = Options::isochrone) {
  const std::string key = "polygons";
  Api request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options().has_polygons(), request.options().polygons());
}

void test_denoise_parsing(const float expected_value,
                          const Options::Action action = Options::isochrone) {
  const std::string key = "denoise";
  Api request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options().has_denoise(), request.options().denoise());
}

void test_generalize_parsing(const float expected_value,
                             const Options::Action action = Options::isochrone) {
  const std::string key = "generalize";
  Api request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options().has_generalize(), request.options().generalize());
}

void test_show_locations_parsing(const bool expected_value,
                                 const Options::Action action = Options::isochrone) {
  const std::string key = "show_locations";
  Api request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options().has_show_locations(),
           request.options().show_locations());
}

void test_shape_match_parsing(const ShapeMatch expected_value, const Options::Action action) {
  const std::string key = "shape_match";
  Api request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options().has_shape_match(), request.options().shape_match());
}

void test_best_paths_parsing(const uint32_t expected_value,
                             const Options::Action action = Options::isochrone) {
  const std::string key = "best_paths";
  Api request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options().has_best_paths(), request.options().best_paths());
}

void test_gps_accuracy_parsing(const float expected_value,
                               const Options::Action action = Options::isochrone) {
  const std::string parent_key = "trace_options";
  const std::string key = "gps_accuracy";
  Api request = get_request(get_request_str(parent_key, key, expected_value), action);
  validate(key, expected_value, request.options().has_gps_accuracy(),
           request.options().gps_accuracy());
}

void test_search_radius_parsing(const float expected_value,
                                const Options::Action action = Options::isochrone) {
  const std::string parent_key = "trace_options";
  const std::string key = "search_radius";
  Api request = get_request(get_request_str(parent_key, key, expected_value), action);
  validate(key, expected_value, request.options().has_search_radius(),
           request.options().search_radius());
}

void test_turn_penalty_factor_parsing(const float expected_value,
                                      const Options::Action action = Options::isochrone) {
  const std::string parent_key = "trace_options";
  const std::string key = "turn_penalty_factor";
  Api request = get_request(get_request_str(parent_key, key, expected_value), action);
  validate(key, expected_value, request.options().has_turn_penalty_factor(),
           request.options().turn_penalty_factor());
}

void test_filter_action_parsing(const valhalla::FilterAction expected_value,
                                const Options::Action action = Options::trace_attributes) {
  const std::string parent_key = "filters";
  const std::string key = "action";
  Api request = get_request(get_request_str(parent_key, key, expected_value), action);
  validate(key, expected_value, request.options().has_filter_action(),
           request.options().filter_action());
}

void test_filter_attributes_parsing(const std::vector<std::string>& expected_values,
                                    const Options::Action action = Options::trace_attributes) {
  const std::string parent_key = "filters";
  const std::string key = "attributes";
  Api request = get_request(get_request_str(parent_key, key, expected_values), action);
  validate(key, expected_values, (request.options().filter_attributes_size() > 0),
           request.options().filter_attributes());
}

void test_default_base_auto_cost_options(const Costing costing, const Options::Action action) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string key = "costing";

  // Get cost request with no cost options
  Api request = get_request(get_request_str(key, costing_str), action);

  validate("maneuver_penalty", kDefaultAuto_ManeuverPenalty,
           request.options().costing_options(static_cast<int>(costing)).maneuver_penalty());
  validate("destination_only_penalty", kDefaultAuto_DestinationOnlyPenalty,
           request.options().costing_options(static_cast<int>(costing)).destination_only_penalty());
  validate("gate_cost", kDefaultAuto_GateCost,
           request.options().costing_options(static_cast<int>(costing)).gate_cost());
  validate("gate_penalty", kDefaultAuto_GatePenalty,
           request.options().costing_options(static_cast<int>(costing)).gate_penalty());
  validate("private_access_penalty", kDefaultAuto_PrivateAccessPenalty,
           request.options().costing_options(static_cast<int>(costing)).private_access_penalty());
  validate("toll_booth_cost", kDefaultAuto_TollBoothCost,
           request.options().costing_options(static_cast<int>(costing)).toll_booth_cost());
  validate("toll_booth_penalty", kDefaultAuto_TollBoothPenalty,
           request.options().costing_options(static_cast<int>(costing)).toll_booth_penalty());
  validate("alley_penalty", kDefaultAuto_AlleyPenalty,
           request.options().costing_options(static_cast<int>(costing)).alley_penalty());
  validate("country_crossing_cost", kDefaultAuto_CountryCrossingCost,
           request.options().costing_options(static_cast<int>(costing)).country_crossing_cost());
  validate("country_crossing_penalty", kDefaultAuto_CountryCrossingPenalty,
           request.options().costing_options(static_cast<int>(costing)).country_crossing_penalty());
  validate("ferry_cost", kDefaultAuto_FerryCost,
           request.options().costing_options(static_cast<int>(costing)).ferry_cost());
  validate("use_ferry", kDefaultAuto_UseFerry,
           request.options().costing_options(static_cast<int>(costing)).use_ferry());
  validate("use_highways", kDefaultAuto_UseHighways,
           request.options().costing_options(static_cast<int>(costing)).use_highways());
  validate("use_tolls", kDefaultAuto_UseTolls,
           request.options().costing_options(static_cast<int>(costing)).use_tolls());
  validate("use_tracks", kDefaultAuto_UseTracks,
           request.options().costing_options(static_cast<int>(costing)).use_tracks());
  validate("use_living_streets", kDefaultAuto_UseLivingStreets,
           request.options().costing_options(static_cast<int>(costing)).use_living_streets());
  validate("service_penalty", kDefaultAuto_ServicePenalty,
           request.options().costing_options(static_cast<int>(costing)).service_penalty());
  validate("service_factor", kDefaultAuto_ServiceFactor,
           request.options().costing_options(static_cast<int>(costing)).service_factor());
}

void test_default_motor_scooter_cost_options(const Costing costing, const Options::Action action) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string key = "costing";

  // Get cost request with no cost options
  Api request = get_request(get_request_str(key, costing_str), action);

  validate("maneuver_penalty", kDefaultMotorScooter_ManeuverPenalty,
           request.options().costing_options(static_cast<int>(costing)).maneuver_penalty());
  validate("destination_only_penalty", kDefaultMotorScooter_DestinationOnlyPenalty,
           request.options().costing_options(static_cast<int>(costing)).destination_only_penalty());
  validate("gate_cost", kDefaultMotorScooter_GateCost,
           request.options().costing_options(static_cast<int>(costing)).gate_cost());
  validate("gate_penalty", kDefaultMotorScooter_GatePenalty,
           request.options().costing_options(static_cast<int>(costing)).gate_penalty());
  validate("private_access_penalty", kDefaultMotorScooter_PrivateAccessPenalty,
           request.options().costing_options(static_cast<int>(costing)).private_access_penalty());
  validate("alley_penalty", kDefaultMotorScooter_AlleyPenalty,
           request.options().costing_options(static_cast<int>(costing)).alley_penalty());
  validate("country_crossing_cost", kDefaultMotorScooter_CountryCrossingCost,
           request.options().costing_options(static_cast<int>(costing)).country_crossing_cost());
  validate("country_crossing_penalty", kDefaultMotorScooter_CountryCrossingPenalty,
           request.options().costing_options(static_cast<int>(costing)).country_crossing_penalty());
  validate("ferry_cost", kDefaultMotorScooter_FerryCost,
           request.options().costing_options(static_cast<int>(costing)).ferry_cost());
  validate("use_ferry", kDefaultMotorScooter_UseFerry,
           request.options().costing_options(static_cast<int>(costing)).use_ferry());
  validate("top_speed", static_cast<float>(kDefaultMotorScooter_TopSpeed),
           request.options().costing_options(static_cast<int>(costing)).top_speed());
  validate("use_hills", kDefaultMotorScooter_UseHills,
           request.options().costing_options(static_cast<int>(costing)).use_hills());
  validate("use_primary", kDefaultMotorScooter_UsePrimary,
           request.options().costing_options(static_cast<int>(costing)).use_primary());
  validate("use_living_streets", kDefaultMotorScooter_UseLivingStreets,
           request.options().costing_options(static_cast<int>(costing)).use_living_streets());
  validate("service_penalty", kDefaultMotorScooter_ServicePenalty,
           request.options().costing_options(static_cast<int>(costing)).service_penalty());
  validate("service_factor", kDefaultMotorcycle_ServiceFactor,
           request.options().costing_options(static_cast<int>(costing)).service_factor());
}

void test_default_motorcycle_cost_options(const Costing costing, const Options::Action action) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string key = "costing";

  // Get cost request with no cost options
  Api request = get_request(get_request_str(key, costing_str), action);

  validate("maneuver_penalty", kDefaultMotorcycle_ManeuverPenalty,
           request.options().costing_options(static_cast<int>(costing)).maneuver_penalty());
  validate("destination_only_penalty", kDefaultMotorcycle_DestinationOnlyPenalty,
           request.options().costing_options(static_cast<int>(costing)).destination_only_penalty());
  validate("gate_cost", kDefaultMotorcycle_GateCost,
           request.options().costing_options(static_cast<int>(costing)).gate_cost());
  validate("gate_penalty", kDefaultMotorcycle_GatePenalty,
           request.options().costing_options(static_cast<int>(costing)).gate_penalty());
  validate("private_access_penalty", kDefaultMotorcycle_PrivateAccessPenalty,
           request.options().costing_options(static_cast<int>(costing)).private_access_penalty());
  validate("alley_penalty", kDefaultMotorcycle_AlleyPenalty,
           request.options().costing_options(static_cast<int>(costing)).alley_penalty());
  validate("country_crossing_cost", kDefaultMotorcycle_CountryCrossingCost,
           request.options().costing_options(static_cast<int>(costing)).country_crossing_cost());
  validate("country_crossing_penalty", kDefaultMotorcycle_CountryCrossingPenalty,
           request.options().costing_options(static_cast<int>(costing)).country_crossing_penalty());
  validate("ferry_cost", kDefaultMotorcycle_FerryCost,
           request.options().costing_options(static_cast<int>(costing)).ferry_cost());
  validate("use_ferry", kDefaultMotorcycle_UseFerry,
           request.options().costing_options(static_cast<int>(costing)).use_ferry());
  validate("use_trails", kDefaultMotorcycle_UseTrails,
           request.options().costing_options(static_cast<int>(costing)).use_trails());
  validate("use_living_streets", kDefaultMotorcycle_UseLivingStreets,
           request.options().costing_options(static_cast<int>(costing)).use_living_streets());
  validate("service_penalty", kDefaultMotorcycle_ServicePenalty,
           request.options().costing_options(static_cast<int>(costing)).service_penalty());
  validate("service_factor", kDefaultMotorcycle_ServiceFactor,
           request.options().costing_options(static_cast<int>(costing)).service_factor());
}

void test_default_pedestrian_cost_options(const Costing costing, const Options::Action action) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string key = "costing";

  // Get cost request with no cost options
  Api request = get_request(get_request_str(key, costing_str), action);

  validate("type", "foot",
           request.options().costing_options(static_cast<int>(costing)).transport_type());
  validate("maneuver_penalty", kDefaultPedestrian_ManeuverPenalty,
           request.options().costing_options(static_cast<int>(costing)).maneuver_penalty());
  validate("gate_penalty", kDefaultPedestrian_GatePenalty,
           request.options().costing_options(static_cast<int>(costing)).gate_penalty());
  validate("country_crossing_cost", kDefaultPedestrian_CountryCrossingCost,
           request.options().costing_options(static_cast<int>(costing)).country_crossing_cost());
  validate("country_crossing_penalty", kDefaultPedestrian_CountryCrossingPenalty,
           request.options().costing_options(static_cast<int>(costing)).country_crossing_penalty());
  validate("ferry_cost", kDefaultPedestrian_FerryCost,
           request.options().costing_options(static_cast<int>(costing)).ferry_cost());
  validate("use_ferry", kDefaultPedestrian_UseFerry,
           request.options().costing_options(static_cast<int>(costing)).use_ferry());
  validate("max_distance", kDefaultPedestrian_MaxDistanceFoot,
           request.options().costing_options(static_cast<int>(costing)).max_distance());
  validate("walking_speed", kDefaultPedestrian_SpeedFoot,
           request.options().costing_options(static_cast<int>(costing)).walking_speed());
  validate("walking_speed", kDefaultPedestrian_SpeedFoot,
           request.options().costing_options(static_cast<int>(costing)).walking_speed());
  validate("step_penalty", kDefaultPedestrian_StepPenaltyFoot,
           request.options().costing_options(static_cast<int>(costing)).step_penalty());
  validate("max_grade", kDefaultPedestrian_MaxGradeFoot,
           request.options().costing_options(static_cast<int>(costing)).max_grade());
  validate("max_hiking_difficulty", kDefaultPedestrian_MaxHikingDifficulty,
           request.options().costing_options(static_cast<int>(costing)).max_hiking_difficulty());
  validate("mode_factor", kDefaultPedestrian_ModeFactor,
           request.options().costing_options(static_cast<int>(costing)).mode_factor());
  validate("walkway_factor", kDefaultPedestrian_WalkwayFactor,
           request.options().costing_options(static_cast<int>(costing)).walkway_factor());
  validate("sidewalk_factor", kDefaultPedestrian_SideWalkFactor,
           request.options().costing_options(static_cast<int>(costing)).sidewalk_factor());
  validate("alley_factor", kDefaultPedestrian_AlleyFactor,
           request.options().costing_options(static_cast<int>(costing)).alley_factor());
  validate("driveway_factor", kDefaultPedestrian_DrivewayFactor,
           request.options().costing_options(static_cast<int>(costing)).driveway_factor());
  validate("use_living_streets", kDefaultPedestrian_UseLivingStreets,
           request.options().costing_options(static_cast<int>(costing)).use_living_streets());
  validate("service_penalty", kDefaultPedestrian_ServicePenalty,
           request.options().costing_options(static_cast<int>(costing)).service_penalty());
  validate("service_factor", kDefaultPedestrian_ServiceFactor,
           request.options().costing_options(static_cast<int>(costing)).service_factor());
  validate("transit_start_end_max_distance", kDefaultPedestrian_TransitStartEndMaxDistance,
           request.options()
               .costing_options(static_cast<int>(costing))
               .transit_start_end_max_distance());
  validate("transit_transfer_max_distance", kDefaultPedestrian_TransitTransferMaxDistance,
           request.options()
               .costing_options(static_cast<int>(costing))
               .transit_transfer_max_distance());
}

void test_default_bicycle_cost_options(const Costing costing, const Options::Action action) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string key = "costing";

  // Get cost request with no cost options
  Api request = get_request(get_request_str(key, costing_str), action);

  validate("bicycle_type", kDefaultBicycle_BicycleType,
           request.options().costing_options(static_cast<int>(costing)).transport_type());
  validate("maneuver_penalty", kDefaultBicycle_ManeuverPenalty,
           request.options().costing_options(static_cast<int>(costing)).maneuver_penalty());
  validate("alley_penalty", kDefaultBicycle_AlleyPenalty,
           request.options().costing_options(static_cast<int>(costing)).alley_penalty());
  validate("gate_cost", kDefaultBicycle_GateCost,
           request.options().costing_options(static_cast<int>(costing)).gate_cost());
  validate("gate_penalty", kDefaultBicycle_GatePenalty,
           request.options().costing_options(static_cast<int>(costing)).gate_penalty());
  validate("private_access_penalty", kDefaultBicycle_PrivateAccessPenalty,
           request.options().costing_options(static_cast<int>(costing)).private_access_penalty());
  validate("country_crossing_cost", kDefaultBicycle_CountryCrossingCost,
           request.options().costing_options(static_cast<int>(costing)).country_crossing_cost());
  validate("country_crossing_penalty", kDefaultBicycle_CountryCrossingPenalty,
           request.options().costing_options(static_cast<int>(costing)).country_crossing_penalty());
  validate("ferry_cost", kDefaultBicycle_FerryCost,
           request.options().costing_options(static_cast<int>(costing)).ferry_cost());
  validate("use_ferry", kDefaultBicycle_UseFerry,
           request.options().costing_options(static_cast<int>(costing)).use_ferry());
  validate("use_roads", kDefaultBicycle_UseRoad,
           request.options().costing_options(static_cast<int>(costing)).use_roads());
  validate("use_hills", kDefaultBicycle_UseHills,
           request.options().costing_options(static_cast<int>(costing)).use_hills());
  validate("avoid_bad_surfaces", kDefaultBicycle_AvoidBadSurfaces,
           request.options().costing_options(static_cast<int>(costing)).avoid_bad_surfaces());
  validate("use_living_streets", kDefaultBicycle_UseLivingStreets,
           request.options().costing_options(static_cast<int>(costing)).use_living_streets());
  validate("service_penalty", kDefaultBicycle_ServicePenalty,
           request.options().costing_options(static_cast<int>(costing)).service_penalty());
  validate("cycling_speed",
           kDefaultBicycle_CyclingSpeed[static_cast<uint32_t>(valhalla::sif::BicycleType::kHybrid)],
           request.options().costing_options(static_cast<int>(costing)).cycling_speed());
}

void test_default_truck_cost_options(const Costing costing, const Options::Action action) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string key = "costing";

  // Get cost request with no cost options
  Api request = get_request(get_request_str(key, costing_str), action);

  validate("maneuver_penalty", kDefaultTruck_ManeuverPenalty,
           request.options().costing_options(static_cast<int>(costing)).maneuver_penalty());
  validate("destination_only_penalty", kDefaultTruck_DestinationOnlyPenalty,
           request.options().costing_options(static_cast<int>(costing)).destination_only_penalty());
  validate("alley_penalty", kDefaultTruck_AlleyPenalty,
           request.options().costing_options(static_cast<int>(costing)).alley_penalty());
  validate("gate_cost", kDefaultTruck_GateCost,
           request.options().costing_options(static_cast<int>(costing)).gate_cost());
  validate("gate_penalty", kDefaultTruck_GatePenalty,
           request.options().costing_options(static_cast<int>(costing)).gate_penalty());
  validate("private_access_penalty", kDefaultTruck_PrivateAccessPenalty,
           request.options().costing_options(static_cast<int>(costing)).private_access_penalty());
  validate("toll_booth_cost", kDefaultTruck_TollBoothCost,
           request.options().costing_options(static_cast<int>(costing)).toll_booth_cost());
  validate("toll_booth_penalty", kDefaultTruck_TollBoothPenalty,
           request.options().costing_options(static_cast<int>(costing)).toll_booth_penalty());
  validate("country_crossing_cost", kDefaultTruck_CountryCrossingCost,
           request.options().costing_options(static_cast<int>(costing)).country_crossing_cost());
  validate("country_crossing_penalty", kDefaultTruck_CountryCrossingPenalty,
           request.options().costing_options(static_cast<int>(costing)).country_crossing_penalty());
  validate("low_class_penalty", kDefaultTruck_LowClassPenalty,
           request.options().costing_options(static_cast<int>(costing)).low_class_penalty());
  validate("hazmat", false, request.options().costing_options(static_cast<int>(costing)).hazmat());
  validate("weight", kDefaultTruck_TruckWeight,
           request.options().costing_options(static_cast<int>(costing)).weight());
  validate("axle_load", kDefaultTruck_TruckAxleLoad,
           request.options().costing_options(static_cast<int>(costing)).axle_load());
  validate("height", kDefaultTruck_TruckHeight,
           request.options().costing_options(static_cast<int>(costing)).height());
  validate("width", kDefaultTruck_TruckWidth,
           request.options().costing_options(static_cast<int>(costing)).width());
  validate("length", kDefaultTruck_TruckLength,
           request.options().costing_options(static_cast<int>(costing)).length());
  validate("use_tracks", kDefaultTruck_UseTracks,
           request.options().costing_options(static_cast<int>(costing)).use_tracks());
  validate("use_living_streets", kDefaultTruck_UseLivingStreets,
           request.options().costing_options(static_cast<int>(costing)).use_living_streets());
  validate("service_penalty", kDefaultTruck_ServicePenalty,
           request.options().costing_options(static_cast<int>(costing)).service_penalty());
  validate("service_factor", kDefaultTruck_ServiceFactor,
           request.options().costing_options(static_cast<int>(costing)).service_factor());
}

void test_default_transit_cost_options(const Costing costing, const Options::Action action) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string key = "costing";

  // Get cost request with no cost options
  Api request = get_request(get_request_str(key, costing_str), action);

  validate("mode_factor", kDefaultTransit_ModeFactor,
           request.options().costing_options(static_cast<int>(costing)).mode_factor());
  validate("wheelchair", false,
           request.options().costing_options(static_cast<int>(costing)).wheelchair());
  validate("bicycle", false, request.options().costing_options(static_cast<int>(costing)).bicycle());
  validate("use_bus", kDefaultTransit_UseBus,
           request.options().costing_options(static_cast<int>(costing)).use_bus());
  validate("use_rail", kDefaultTransit_UseRail,
           request.options().costing_options(static_cast<int>(costing)).use_rail());
  validate("use_transfers", kDefaultTransit_UseTransfers,
           request.options().costing_options(static_cast<int>(costing)).use_transfers());
  validate("transfer_cost", kDefaultTransit_TransferCost,
           request.options().costing_options(static_cast<int>(costing)).transfer_cost());
  validate("transfer_penalty", kDefaultTransit_TransferPenalty,
           request.options().costing_options(static_cast<int>(costing)).transfer_penalty());
}

void test_default_base_cost_options(const Costing costing, const Options::Action action) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string key = "costing";

  // Get cost request with no cost options
  Api request = get_request(get_request_str(key, costing_str), action);

  validate(costing_str + " closure_factor", kDefaultClosureFactor,
           request.options().costing_options(static_cast<float>(costing)).closure_factor());
  // TODO: Validate more common cost attributes
}

void test_transport_type_parsing(const Costing costing,
                                 const std::string& key,
                                 const std::string& specified_value,
                                 const std::string& expected_value,
                                 const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).transport_type());
}

void test_maneuver_penalty_parsing(const Costing costing,
                                   const float specified_value,
                                   const float expected_value,
                                   const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "maneuver_penalty";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).maneuver_penalty());
}

void test_destination_only_penalty_parsing(const Costing costing,
                                           const float specified_value,
                                           const float expected_value,
                                           const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "destination_only_penalty";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).destination_only_penalty());
}

void test_gate_cost_parsing(const Costing costing,
                            const float specified_value,
                            const float expected_value,
                            const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "gate_cost";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).gate_cost());
}

void test_gate_penalty_parsing(const Costing costing,
                               const float specified_value,
                               const float expected_value,
                               const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "gate_penalty";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).gate_penalty());
}

void test_private_access_penalty_parsing(const Costing costing,
                                         const float specified_value,
                                         const float expected_value,
                                         const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "private_access_penalty";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).private_access_penalty());
}

void test_toll_booth_cost_parsing(const Costing costing,
                                  const float specified_value,
                                  const float expected_value,
                                  const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "toll_booth_cost";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).toll_booth_cost());
}

void test_toll_booth_penalty_parsing(const Costing costing,
                                     const float specified_value,
                                     const float expected_value,
                                     const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "toll_booth_penalty";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).toll_booth_penalty());
}

void test_alley_penalty_parsing(const Costing costing,
                                const float specified_value,
                                const float expected_value,
                                const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "alley_penalty";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).alley_penalty());
}

void test_country_crossing_cost_parsing(const Costing costing,
                                        const float specified_value,
                                        const float expected_value,
                                        const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "country_crossing_cost";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).country_crossing_cost());
}

void test_country_crossing_penalty_parsing(const Costing costing,
                                           const float specified_value,
                                           const float expected_value,
                                           const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "country_crossing_penalty";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).country_crossing_penalty());
}

void test_ferry_cost_parsing(const Costing costing,
                             const float specified_value,
                             const float expected_value,
                             const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "ferry_cost";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).ferry_cost());
}

void test_use_ferry_parsing(const Costing costing,
                            const float specified_value,
                            const float expected_value,
                            const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "use_ferry";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).use_ferry());
}

void test_use_highways_parsing(const Costing costing,
                               const float specified_value,
                               const float expected_value,
                               const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "use_highways";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).use_highways());
}

void test_use_tolls_parsing(const Costing costing,
                            const float specified_value,
                            const float expected_value,
                            const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "use_tolls";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).use_tolls());
}

void test_use_tracks_parsing(const Costing costing,
                             const float specified_value,
                             const float expected_value,
                             const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "use_tracks";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).use_tracks());
}

void test_use_living_streets_parsing(const Costing costing,
                                     const float specified_value,
                                     const float expected_value,
                                     const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "use_living_streets";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).use_living_streets());
}

void test_use_hills_parsing(const Costing costing,
                            const float specified_value,
                            const float expected_value,
                            const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "use_hills";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).use_hills());
}

void test_use_primary_parsing(const Costing costing,
                              const float specified_value,
                              const float expected_value,
                              const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "use_primary";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).use_primary());
}

void test_top_speed_parsing(const Costing costing,
                            const float specified_value,
                            const float expected_value,
                            const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "top_speed";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).top_speed());
}

void test_use_trails_parsing(const Costing costing,
                             const float specified_value,
                             const float expected_value,
                             const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "use_trails";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).use_trails());
}

void test_max_distance_parsing(const Costing costing,
                               const std::string& transport_type,
                               const uint32_t specified_value,
                               const uint32_t expected_value,
                               const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string sibling_key = "type";
  const std::string key = "max_distance";

  Api request = get_request(get_request_str(grandparent_key, parent_key, sibling_key, transport_type,
                                            key, specified_value),
                            action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).max_distance());
}

void test_walking_speed_parsing(const Costing costing,
                                const std::string& transport_type,
                                const float specified_value,
                                const float expected_value,
                                const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string sibling_key = "type";
  const std::string key = "walking_speed";

  Api request = get_request(get_request_str(grandparent_key, parent_key, sibling_key, transport_type,
                                            key, specified_value),
                            action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).walking_speed());
}

void test_cycling_speed_parsing(const Costing costing,
                                const std::string& transport_type,
                                const float specified_value,
                                const float expected_value,
                                const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string sibling_key = "bicycle_type";
  const std::string key = "cycling_speed";

  Api request = get_request(get_request_str(grandparent_key, parent_key, sibling_key, transport_type,
                                            key, specified_value),
                            action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).cycling_speed());
}

void test_step_penalty_parsing(const Costing costing,
                               const std::string& transport_type,
                               const float specified_value,
                               const float expected_value,
                               const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string sibling_key = "type";
  const std::string key = "step_penalty";

  Api request = get_request(get_request_str(grandparent_key, parent_key, sibling_key, transport_type,
                                            key, specified_value),
                            action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).step_penalty());
}

void test_max_grade_parsing(const Costing costing,
                            const std::string& transport_type,
                            const uint32_t specified_value,
                            const uint32_t expected_value,
                            const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string sibling_key = "type";
  const std::string key = "max_grade";

  Api request = get_request(get_request_str(grandparent_key, parent_key, sibling_key, transport_type,
                                            key, specified_value),
                            action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).max_grade());
}

void test_max_hiking_difficulty_parsing(const Costing costing,
                                        const uint32_t specified_value,
                                        const uint32_t expected_value,
                                        const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "max_hiking_difficulty";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).max_hiking_difficulty());
}

void test_mode_factor_parsing(const Costing costing,
                              const float specified_value,
                              const float expected_value,
                              const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "mode_factor";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).mode_factor());
}

void test_walkway_factor_parsing(const Costing costing,
                                 const float specified_value,
                                 const float expected_value,
                                 const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "walkway_factor";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).walkway_factor());
}

void test_sidewalk_factor_parsing(const Costing costing,
                                  const float specified_value,
                                  const float expected_value,
                                  const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "sidewalk_factor";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).sidewalk_factor());
}

void test_alley_factor_parsing(const Costing costing,
                               const float specified_value,
                               const float expected_value,
                               const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "alley_factor";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).alley_factor());
}

void test_driveway_factor_parsing(const Costing costing,
                                  const float specified_value,
                                  const float expected_value,
                                  const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "driveway_factor";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).driveway_factor());
}

void test_use_roads_parsing(const Costing costing,
                            const float specified_value,
                            const float expected_value,
                            const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "use_roads";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).use_roads());
}

void test_avoid_bad_surfaces_parsing(const Costing costing,
                                     const float specified_value,
                                     const float expected_value,
                                     const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "avoid_bad_surfaces";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).avoid_bad_surfaces());
}

void test_transit_start_end_max_distance_parsing(const Costing costing,
                                                 const uint32_t specified_value,
                                                 const uint32_t expected_value,
                                                 const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "transit_start_end_max_distance";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options()
               .costing_options(static_cast<int>(costing))
               .transit_start_end_max_distance());
}

void test_transit_transfer_max_distance_parsing(const Costing costing,
                                                const uint32_t specified_value,
                                                const uint32_t expected_value,
                                                const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "transit_transfer_max_distance";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options()
               .costing_options(static_cast<int>(costing))
               .transit_transfer_max_distance());
}

void test_low_class_penalty_parsing(const Costing costing,
                                    const float specified_value,
                                    const float expected_value,
                                    const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "low_class_penalty";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).low_class_penalty());
}

void test_weight_parsing(const Costing costing,
                         const float specified_value,
                         const float expected_value,
                         const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "weight";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).weight());
}

void test_axle_load_parsing(const Costing costing,
                            const float specified_value,
                            const float expected_value,
                            const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "axle_load";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).axle_load());
}

void test_height_parsing(const Costing costing,
                         const float specified_value,
                         const float expected_value,
                         const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "height";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).height());
}

void test_width_parsing(const Costing costing,
                        const float specified_value,
                        const float expected_value,
                        const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "width";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value, request.options().costing_options(static_cast<int>(costing)).width());
}

void test_length_parsing(const Costing costing,
                         const float specified_value,
                         const float expected_value,
                         const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "length";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).length());
}

void test_use_bus_parsing(const Costing costing,
                          const float specified_value,
                          const float expected_value,
                          const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "use_bus";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).use_bus());
}

void test_use_rail_parsing(const Costing costing,
                           const float specified_value,
                           const float expected_value,
                           const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "use_rail";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).use_rail());
}

void test_use_transfers_parsing(const Costing costing,
                                const float specified_value,
                                const float expected_value,
                                const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "use_transfers";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).use_transfers());
}

void test_transfer_cost_parsing(const Costing costing,
                                const float specified_value,
                                const float expected_value,
                                const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "transfer_cost";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).transfer_cost());
}

void test_transfer_penalty_parsing(const Costing costing,
                                   const float specified_value,
                                   const float expected_value,
                                   const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "transfer_penalty";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).transfer_penalty());
}

void test_hazmat_parsing(const Costing costing,
                         const bool specified_value,
                         const bool expected_value,
                         const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "hazmat";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).hazmat());
}

void test_wheelchair_parsing(const Costing costing,
                             const bool specified_value,
                             const bool expected_value,
                             const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "wheelchair";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).wheelchair());
}

void test_bicycle_parsing(const Costing costing,
                          const bool specified_value,
                          const bool expected_value,
                          const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "bicycle";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).bicycle());
}

void test_filter_stop_parsing(const Costing costing,
                              const valhalla::FilterAction filter_action,
                              const std::vector<std::string>& filter_ids,
                              const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string filter_type = "stops";
  const std::string action_key = "filter_stop_action";
  const std::string ids_key = "filter_stop_ids";

  Api request =
      get_request(get_filter_request_str(costing_str, filter_type, filter_action, filter_ids),
                  action);
  validate(action_key, filter_action,
           request.options().costing_options(static_cast<int>(costing)).has_filter_stop_action(),
           request.options().costing_options(static_cast<int>(costing)).filter_stop_action());
  validate(ids_key, filter_ids,
           (request.options().costing_options(static_cast<int>(costing)).filter_stop_ids_size() > 0),
           request.options().costing_options(static_cast<int>(costing)).filter_stop_ids());
}

void test_filter_route_parsing(const Costing costing,
                               const valhalla::FilterAction filter_action,
                               const std::vector<std::string>& filter_ids,
                               const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string filter_type = "routes";
  const std::string action_key = "filter_route_action";
  const std::string ids_key = "filter_route_ids";

  Api request =
      get_request(get_filter_request_str(costing_str, filter_type, filter_action, filter_ids),
                  action);
  validate(action_key, filter_action,
           request.options().costing_options(static_cast<int>(costing)).has_filter_route_action(),
           request.options().costing_options(static_cast<int>(costing)).filter_route_action());
  validate(ids_key, filter_ids,
           (request.options().costing_options(static_cast<int>(costing)).filter_route_ids_size() > 0),
           request.options().costing_options(static_cast<int>(costing)).filter_route_ids());
}

void test_filter_operator_parsing(const Costing costing,
                                  const valhalla::FilterAction filter_action,
                                  const std::vector<std::string>& filter_ids,
                                  const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string filter_type = "operators";
  const std::string action_key = "filter_operator_action";
  const std::string ids_key = "filter_operator_ids";

  Api request =
      get_request(get_filter_request_str(costing_str, filter_type, filter_action, filter_ids),
                  action);
  validate(action_key, filter_action,
           request.options().costing_options(static_cast<int>(costing)).has_filter_operator_action(),
           request.options().costing_options(static_cast<int>(costing)).filter_operator_action());
  validate(ids_key, filter_ids,
           (request.options().costing_options(static_cast<int>(costing)).filter_operator_ids_size() >
            0),
           request.options().costing_options(static_cast<int>(costing)).filter_operator_ids());
}

void test_service_penalty_parsing(const Costing costing,
                                  const float specified_value,
                                  const float expected_value,
                                  const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "service_penalty";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).service_penalty());
}

void test_service_factor_parsing(const Costing costing,
                                 const float specified_value,
                                 const float expected_value,
                                 const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "service_factor";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).service_factor());
}

void test_closure_factor_parsing(const Costing costing,
                                 const float specified_value,
                                 const float expected_value,
                                 const Options::Action action = Options::route) {
  // Create the costing string
  auto costing_str = get_costing_str(costing);
  const std::string grandparent_key = "costing_options";
  const std::string& parent_key = costing_str;
  const std::string key = "closure_factor";

  Api request =
      get_request(get_request_str(grandparent_key, parent_key, key, specified_value), action);
  validate(key, expected_value,
           request.options().costing_options(static_cast<int>(costing)).closure_factor());
}

///////////////////////////////////////////////////////////////////////////////
// test by key methods
TEST(ParseRequest, test_polygons) {
  test_polygons_parsing(true);
  test_polygons_parsing(false);
}

TEST(ParseRequest, test_denoise) {
  test_denoise_parsing(0.0f);
  test_denoise_parsing(0.5f);
  test_denoise_parsing(1.0f);
}

TEST(ParseRequest, test_generalize) {
  test_generalize_parsing(20.f);
  test_generalize_parsing(50.f);
}

TEST(ParseRequest, test_show_locations) {
  test_show_locations_parsing(true);
  test_show_locations_parsing(false);
}

TEST(ParseRequest, test_shape_match) {
  test_shape_match_parsing(ShapeMatch::map_snap, Options::trace_route);
  test_shape_match_parsing(ShapeMatch::map_snap, Options::trace_attributes);
  test_shape_match_parsing(ShapeMatch::edge_walk, Options::trace_route);
  test_shape_match_parsing(ShapeMatch::edge_walk, Options::trace_attributes);
}

TEST(ParseRequest, test_best_paths) {
  test_best_paths_parsing(1);
  test_best_paths_parsing(2);
  test_best_paths_parsing(4);
}

TEST(ParseRequest, test_gps_accuracy) {
  test_gps_accuracy_parsing(5.f);
  test_gps_accuracy_parsing(30.f);
}

TEST(ParseRequest, test_search_radius) {
  test_search_radius_parsing(10.f);
  test_search_radius_parsing(40.f);
}

TEST(ParseRequest, test_turn_penalty_factor) {
  test_turn_penalty_factor_parsing(50.f);
  test_turn_penalty_factor_parsing(100.f);
}

TEST(ParseRequest, test_filter_action) {
  test_filter_action_parsing(valhalla::FilterAction::exclude);
  test_filter_action_parsing(valhalla::FilterAction::include);
}

TEST(ParseRequest, test_filter_attributes) {
  test_filter_attributes_parsing({"edge.names", "edge.id", "edge.weighted_grade", "edge.speed"});
}

std::vector<Costing> get_all_motor_costings() {
  return {Costing::auto_,         Costing::bicycle,    Costing::bus,   Costing::taxi,
          Costing::motor_scooter, Costing::pedestrian, Costing::truck, Costing::motorcycle};
}

std::vector<Costing> get_base_auto_costing_list() {
  return {Costing::auto_, Costing::bus, Costing::taxi};
}
TEST(ParseRequest, test_default_base_auto_cost_options) {
  for (auto costing : get_base_auto_costing_list()) {
    test_default_base_auto_cost_options(costing, Options::route);
  }
}

TEST(ParseRequest, test_default_motor_scooter_cost_options) {
  test_default_motor_scooter_cost_options(motor_scooter, Options::route);
}

TEST(ParseRequest, test_default_motorcycle_cost_options) {
  test_default_motorcycle_cost_options(motorcycle, Options::route);
}

TEST(ParseRequest, test_default_pedestrian_cost_options) {
  test_default_pedestrian_cost_options(pedestrian, Options::route);
}

TEST(ParseRequest, test_default_bicycle_cost_options) {
  test_default_bicycle_cost_options(bicycle, Options::route);
}

TEST(ParseRequest, test_default_truck_cost_options) {
  test_default_truck_cost_options(truck, Options::route);
}

TEST(ParseRequest, test_default_transit_cost_options) {
  test_default_transit_cost_options(transit, Options::route);
}

TEST(ParseRequest, test_default_base_cost_options) {
  for (auto costing : get_all_motor_costings()) {
    test_default_base_cost_options(costing, Options::route);
  }
}

TEST(ParseRequest, test_transport_type) {
  std::string transport_type_key = "type";
  std::string transport_type_value = "car";
  for (auto costing : get_base_auto_costing_list()) {
    test_transport_type_parsing(costing, transport_type_key, transport_type_value,
                                transport_type_value);
  }

  Costing costing = Costing::pedestrian;
  for (const auto& transport_type_value : {"foot", "wheelchair"}) {
    test_transport_type_parsing(costing, transport_type_key, transport_type_value,
                                transport_type_value);
  }

  costing = Costing::bicycle;
  transport_type_key = "bicycle_type";
  for (const auto& transport_type_value : {"Road", "Cross", "Hybrid", "Mountain"}) {
    test_transport_type_parsing(costing, transport_type_key, transport_type_value,
                                transport_type_value);
  }
}

TEST(ParseRequest, test_maneuver_penalty) {
  float default_value = kDefaultAuto_ManeuverPenalty;
  for (auto costing : get_base_auto_costing_list()) {
    test_maneuver_penalty_parsing(costing, default_value, default_value);
    test_maneuver_penalty_parsing(costing, 2.f, 2.f);
    test_maneuver_penalty_parsing(costing, 30.f, 30.f);
    test_maneuver_penalty_parsing(costing, -2.f, default_value);
    test_maneuver_penalty_parsing(costing, 50000.f, default_value);
  }

  Costing costing = Costing::motor_scooter;
  default_value = kDefaultMotorScooter_ManeuverPenalty;
  test_maneuver_penalty_parsing(costing, default_value, default_value);
  test_maneuver_penalty_parsing(costing, 2.f, 2.f);
  test_maneuver_penalty_parsing(costing, 30.f, 30.f);
  test_maneuver_penalty_parsing(costing, -2.f, default_value);
  test_maneuver_penalty_parsing(costing, 50000.f, default_value);

  costing = Costing::motorcycle;
  default_value = kDefaultMotorcycle_ManeuverPenalty;
  test_maneuver_penalty_parsing(costing, default_value, default_value);
  test_maneuver_penalty_parsing(costing, 2.f, 2.f);
  test_maneuver_penalty_parsing(costing, 30.f, 30.f);
  test_maneuver_penalty_parsing(costing, -2.f, default_value);
  test_maneuver_penalty_parsing(costing, 50000.f, default_value);

  costing = Costing::pedestrian;
  default_value = kDefaultPedestrian_ManeuverPenalty;
  test_maneuver_penalty_parsing(costing, default_value, default_value);
  test_maneuver_penalty_parsing(costing, 2.f, 2.f);
  test_maneuver_penalty_parsing(costing, 30.f, 30.f);
  test_maneuver_penalty_parsing(costing, -2.f, default_value);
  test_maneuver_penalty_parsing(costing, 50000.f, default_value);

  costing = Costing::bicycle;
  default_value = kDefaultBicycle_ManeuverPenalty;
  test_maneuver_penalty_parsing(costing, default_value, default_value);
  test_maneuver_penalty_parsing(costing, 2.f, 2.f);
  test_maneuver_penalty_parsing(costing, 30.f, 30.f);
  test_maneuver_penalty_parsing(costing, -2.f, default_value);
  test_maneuver_penalty_parsing(costing, 50000.f, default_value);

  costing = Costing::truck;
  default_value = kDefaultTruck_ManeuverPenalty;
  test_maneuver_penalty_parsing(costing, default_value, default_value);
  test_maneuver_penalty_parsing(costing, 2.f, 2.f);
  test_maneuver_penalty_parsing(costing, 10.f, 10.f);
  test_maneuver_penalty_parsing(costing, -2.f, default_value);
  test_maneuver_penalty_parsing(costing, 50000.f, default_value);
}

TEST(ParseRequest, test_destination_only_penalty) {
  float default_value = kDefaultAuto_DestinationOnlyPenalty;
  for (auto costing : get_base_auto_costing_list()) {
    test_destination_only_penalty_parsing(costing, default_value, default_value);
    test_destination_only_penalty_parsing(costing, 2.f, 2.f);
    test_destination_only_penalty_parsing(costing, 700.f, 700.f);
    test_destination_only_penalty_parsing(costing, -2.f, default_value);
    test_destination_only_penalty_parsing(costing, 500000.f, default_value);
  }

  Costing costing = Costing::motor_scooter;
  default_value = kDefaultMotorScooter_DestinationOnlyPenalty;
  test_destination_only_penalty_parsing(costing, default_value, default_value);
  test_destination_only_penalty_parsing(costing, 2.f, 2.f);
  test_destination_only_penalty_parsing(costing, 700.f, 700.f);
  test_destination_only_penalty_parsing(costing, -2.f, default_value);
  test_destination_only_penalty_parsing(costing, 500000.f, default_value);

  costing = Costing::motorcycle;
  default_value = kDefaultMotorcycle_DestinationOnlyPenalty;
  test_destination_only_penalty_parsing(costing, default_value, default_value);
  test_destination_only_penalty_parsing(costing, 2.f, 2.f);
  test_destination_only_penalty_parsing(costing, 700.f, 700.f);
  test_destination_only_penalty_parsing(costing, -2.f, default_value);
  test_destination_only_penalty_parsing(costing, 500000.f, default_value);

  costing = Costing::truck;
  default_value = kDefaultTruck_DestinationOnlyPenalty;
  test_destination_only_penalty_parsing(costing, default_value, default_value);
  test_destination_only_penalty_parsing(costing, 300.f, 300.f);
  test_destination_only_penalty_parsing(costing, 700.f, 700.f);
  test_destination_only_penalty_parsing(costing, -2.f, default_value);
  test_destination_only_penalty_parsing(costing, 50000.f, default_value);
}

TEST(ParseRequest, test_gate_cost) {
  float default_value = kDefaultAuto_GateCost;
  for (auto costing : get_base_auto_costing_list()) {
    test_gate_cost_parsing(costing, default_value, default_value);
    test_gate_cost_parsing(costing, 2.f, 2.f);
    test_gate_cost_parsing(costing, 60.f, 60.f);
    test_gate_cost_parsing(costing, -2.f, default_value);
    test_gate_cost_parsing(costing, 500000.f, default_value);
  }

  Costing costing = Costing::motor_scooter;
  default_value = kDefaultMotorScooter_GateCost;
  test_gate_cost_parsing(costing, default_value, default_value);
  test_gate_cost_parsing(costing, 2.f, 2.f);
  test_gate_cost_parsing(costing, 60.f, 60.f);
  test_gate_cost_parsing(costing, -2.f, default_value);
  test_gate_cost_parsing(costing, 500000.f, default_value);

  costing = Costing::motorcycle;
  default_value = kDefaultMotorcycle_GateCost;
  test_gate_cost_parsing(costing, default_value, default_value);
  test_gate_cost_parsing(costing, 2.f, 2.f);
  test_gate_cost_parsing(costing, 60.f, 60.f);
  test_gate_cost_parsing(costing, -2.f, default_value);
  test_gate_cost_parsing(costing, 500000.f, default_value);

  costing = Costing::bicycle;
  default_value = kDefaultBicycle_GateCost;
  test_gate_cost_parsing(costing, default_value, default_value);
  test_gate_cost_parsing(costing, 15.f, 15.f);
  test_gate_cost_parsing(costing, 60.f, 60.f);
  test_gate_cost_parsing(costing, -2.f, default_value);
  test_gate_cost_parsing(costing, 50000.f, default_value);

  costing = Costing::truck;
  default_value = kDefaultTruck_GateCost;
  test_gate_cost_parsing(costing, default_value, default_value);
  test_gate_cost_parsing(costing, 15.f, 15.f);
  test_gate_cost_parsing(costing, 60.f, 60.f);
  test_gate_cost_parsing(costing, -2.f, default_value);
  test_gate_cost_parsing(costing, 50000.f, default_value);
}

TEST(ParseRequest, test_gate_penalty) {
  float default_value = kDefaultAuto_GatePenalty;
  for (auto costing : get_base_auto_costing_list()) {
    test_gate_penalty_parsing(costing, default_value, default_value);
    test_gate_penalty_parsing(costing, 2.f, 2.f);
    test_gate_penalty_parsing(costing, 60.f, 60.f);
    test_gate_penalty_parsing(costing, -2.f, default_value);
    test_gate_penalty_parsing(costing, 500000.f, default_value);
  }

  Costing costing = Costing::motor_scooter;
  default_value = kDefaultMotorScooter_GatePenalty;
  test_gate_penalty_parsing(costing, default_value, default_value);
  test_gate_penalty_parsing(costing, 2.f, 2.f);
  test_gate_penalty_parsing(costing, 60.f, 60.f);
  test_gate_penalty_parsing(costing, -2.f, default_value);
  test_gate_penalty_parsing(costing, 500000.f, default_value);

  costing = Costing::motorcycle;
  default_value = kDefaultMotorcycle_GatePenalty;
  test_gate_penalty_parsing(costing, default_value, default_value);
  test_gate_penalty_parsing(costing, 2.f, 2.f);
  test_gate_penalty_parsing(costing, 600.f, 600.f);
  test_gate_penalty_parsing(costing, -2.f, default_value);
  test_gate_penalty_parsing(costing, 500000.f, default_value);

  costing = Costing::pedestrian;
  default_value = kDefaultPedestrian_GatePenalty;
  test_gate_penalty_parsing(costing, default_value, default_value);
  test_gate_penalty_parsing(costing, 5.f, 5.f);
  test_gate_penalty_parsing(costing, 20.f, 20.f);
  test_gate_penalty_parsing(costing, -2.f, default_value);
  test_gate_penalty_parsing(costing, 500000.f, default_value);

  costing = Costing::bicycle;
  default_value = kDefaultBicycle_GatePenalty;
  test_gate_penalty_parsing(costing, default_value, default_value);
  test_gate_penalty_parsing(costing, 150.f, 150.f);
  test_gate_penalty_parsing(costing, 600.f, 600.f);
  test_gate_penalty_parsing(costing, -2.f, default_value);
  test_gate_penalty_parsing(costing, 50000.f, default_value);

  costing = Costing::truck;
  default_value = kDefaultTruck_GatePenalty;
  test_gate_penalty_parsing(costing, default_value, default_value);
  test_gate_penalty_parsing(costing, 150.f, 150.f);
  test_gate_penalty_parsing(costing, 600.f, 600.f);
  test_gate_penalty_parsing(costing, -2.f, default_value);
  test_gate_penalty_parsing(costing, 50000.f, default_value);
}

TEST(ParseRequest, test_private_access_penalty) {
  float default_value = kDefaultAuto_PrivateAccessPenalty;
  for (auto costing : get_base_auto_costing_list()) {
    test_private_access_penalty_parsing(costing, default_value, default_value);
    test_private_access_penalty_parsing(costing, 2.f, 2.f);
    test_private_access_penalty_parsing(costing, 60.f, 60.f);
    test_private_access_penalty_parsing(costing, -2.f, default_value);
    test_private_access_penalty_parsing(costing, 500000.f, default_value);
  }

  Costing costing = Costing::motor_scooter;
  default_value = kDefaultMotorScooter_PrivateAccessPenalty;
  test_private_access_penalty_parsing(costing, default_value, default_value);
  test_private_access_penalty_parsing(costing, 2.f, 2.f);
  test_private_access_penalty_parsing(costing, 60.f, 60.f);
  test_private_access_penalty_parsing(costing, -2.f, default_value);
  test_private_access_penalty_parsing(costing, 500000.f, default_value);

  costing = Costing::motorcycle;
  default_value = kDefaultMotorcycle_PrivateAccessPenalty;
  test_private_access_penalty_parsing(costing, default_value, default_value);
  test_private_access_penalty_parsing(costing, 2.f, 2.f);
  test_private_access_penalty_parsing(costing, 60.f, 60.f);
  test_private_access_penalty_parsing(costing, -2.f, default_value);
  test_private_access_penalty_parsing(costing, 500000.f, default_value);

  costing = Costing::bicycle;
  default_value = kDefaultBicycle_PrivateAccessPenalty;
  test_private_access_penalty_parsing(costing, default_value, default_value);
  test_private_access_penalty_parsing(costing, 15.f, 15.f);
  test_private_access_penalty_parsing(costing, 60.f, 60.f);
  test_private_access_penalty_parsing(costing, -2.f, default_value);
  test_private_access_penalty_parsing(costing, 50000.f, default_value);

  costing = Costing::truck;
  default_value = kDefaultTruck_PrivateAccessPenalty;
  test_private_access_penalty_parsing(costing, default_value, default_value);
  test_private_access_penalty_parsing(costing, 15.f, 15.f);
  test_private_access_penalty_parsing(costing, 60.f, 60.f);
  test_private_access_penalty_parsing(costing, -2.f, default_value);
  test_private_access_penalty_parsing(costing, 50000.f, default_value);
}

TEST(ParseRequest, test_toll_booth_cost) {
  float default_value = kDefaultAuto_TollBoothCost;
  for (auto costing : get_base_auto_costing_list()) {
    test_toll_booth_cost_parsing(costing, default_value, default_value);
    test_toll_booth_cost_parsing(costing, 2.f, 2.f);
    test_toll_booth_cost_parsing(costing, 60.f, 60.f);
    test_toll_booth_cost_parsing(costing, -2.f, default_value);
    test_toll_booth_cost_parsing(costing, 500000.f, default_value);
  }

  Costing costing = Costing::motorcycle;
  default_value = kDefaultMotorcycle_TollBoothCost;
  test_toll_booth_cost_parsing(costing, default_value, default_value);
  test_toll_booth_cost_parsing(costing, 2.f, 2.f);
  test_toll_booth_cost_parsing(costing, 20.f, 20.f);
  test_toll_booth_cost_parsing(costing, -2.f, default_value);
  test_toll_booth_cost_parsing(costing, 500000.f, default_value);

  costing = Costing::truck;
  default_value = kDefaultTruck_TollBoothCost;
  test_toll_booth_cost_parsing(costing, default_value, default_value);
  test_toll_booth_cost_parsing(costing, 5.f, 5.f);
  test_toll_booth_cost_parsing(costing, 20.f, 20.f);
  test_toll_booth_cost_parsing(costing, -2.f, default_value);
  test_toll_booth_cost_parsing(costing, 50000.f, default_value);
}

TEST(ParseRequest, test_toll_booth_penalty) {
  float default_value = kDefaultAuto_TollBoothPenalty;
  for (auto costing : get_base_auto_costing_list()) {
    test_toll_booth_penalty_parsing(costing, default_value, default_value);
    test_toll_booth_penalty_parsing(costing, 2.f, 2.f);
    test_toll_booth_penalty_parsing(costing, 60.f, 60.f);
    test_toll_booth_penalty_parsing(costing, -2.f, default_value);
    test_toll_booth_penalty_parsing(costing, 500000.f, default_value);
  }

  Costing costing = Costing::motorcycle;
  default_value = kDefaultMotorcycle_TollBoothPenalty;
  test_toll_booth_penalty_parsing(costing, default_value, default_value);
  test_toll_booth_penalty_parsing(costing, 2.f, 2.f);
  test_toll_booth_penalty_parsing(costing, 60.f, 60.f);
  test_toll_booth_penalty_parsing(costing, -2.f, default_value);
  test_toll_booth_penalty_parsing(costing, 500000.f, default_value);

  costing = Costing::truck;
  default_value = kDefaultTruck_TollBoothPenalty;
  test_toll_booth_penalty_parsing(costing, default_value, default_value);
  test_toll_booth_penalty_parsing(costing, 0.f, 0.f);
  test_toll_booth_penalty_parsing(costing, 60.f, 60.f);
  test_toll_booth_penalty_parsing(costing, -2.f, default_value);
  test_toll_booth_penalty_parsing(costing, 50000.f, default_value);
}

TEST(ParseRequest, test_alley_penalty) {
  float default_value = kDefaultAuto_AlleyPenalty;
  for (auto costing : get_base_auto_costing_list()) {
    test_alley_penalty_parsing(costing, default_value, default_value);
    test_alley_penalty_parsing(costing, 2.f, 2.f);
    test_alley_penalty_parsing(costing, 60.f, 60.f);
    test_alley_penalty_parsing(costing, -2.f, default_value);
    test_alley_penalty_parsing(costing, 500000.f, default_value);
  }

  Costing costing = Costing::motor_scooter;
  default_value = kDefaultMotorScooter_AlleyPenalty;
  test_alley_penalty_parsing(costing, default_value, default_value);
  test_alley_penalty_parsing(costing, 2.f, 2.f);
  test_alley_penalty_parsing(costing, 60.f, 60.f);
  test_alley_penalty_parsing(costing, -2.f, default_value);
  test_alley_penalty_parsing(costing, 500000.f, default_value);

  costing = Costing::motorcycle;
  default_value = kDefaultMotorcycle_AlleyPenalty;
  test_alley_penalty_parsing(costing, default_value, default_value);
  test_alley_penalty_parsing(costing, 2.f, 2.f);
  test_alley_penalty_parsing(costing, 10.f, 10.f);
  test_alley_penalty_parsing(costing, -2.f, default_value);
  test_alley_penalty_parsing(costing, 500000.f, default_value);

  costing = Costing::bicycle;
  default_value = kDefaultBicycle_AlleyPenalty;
  test_alley_penalty_parsing(costing, default_value, default_value);
  test_alley_penalty_parsing(costing, 30.f, 30.f);
  test_alley_penalty_parsing(costing, 90.f, 90.f);
  test_alley_penalty_parsing(costing, -2.f, default_value);
  test_alley_penalty_parsing(costing, 50000.f, default_value);

  costing = Costing::truck;
  default_value = kDefaultTruck_AlleyPenalty;
  test_alley_penalty_parsing(costing, default_value, default_value);
  test_alley_penalty_parsing(costing, 2.f, 2.f);
  test_alley_penalty_parsing(costing, 10.f, 10.f);
  test_alley_penalty_parsing(costing, -2.f, default_value);
  test_alley_penalty_parsing(costing, 50000.f, default_value);
}

TEST(ParseRequest, test_country_crossing_cost) {
  float default_value = kDefaultAuto_CountryCrossingCost;
  for (auto costing : get_base_auto_costing_list()) {
    test_country_crossing_cost_parsing(costing, default_value, default_value);
    test_country_crossing_cost_parsing(costing, 2.f, 2.f);
    test_country_crossing_cost_parsing(costing, 60.f, 60.f);
    test_country_crossing_cost_parsing(costing, -2.f, default_value);
    test_country_crossing_cost_parsing(costing, 500000.f, default_value);
  }

  Costing costing = Costing::motor_scooter;
  default_value = kDefaultMotorScooter_CountryCrossingCost;
  test_country_crossing_cost_parsing(costing, default_value, default_value);
  test_country_crossing_cost_parsing(costing, 2.f, 2.f);
  test_country_crossing_cost_parsing(costing, 700.f, 700.f);
  test_country_crossing_cost_parsing(costing, -2.f, default_value);
  test_country_crossing_cost_parsing(costing, 500000.f, default_value);

  costing = Costing::motorcycle;
  default_value = kDefaultMotorcycle_CountryCrossingCost;
  test_country_crossing_cost_parsing(costing, default_value, default_value);
  test_country_crossing_cost_parsing(costing, 2.f, 2.f);
  test_country_crossing_cost_parsing(costing, 700.f, 700.f);
  test_country_crossing_cost_parsing(costing, -2.f, default_value);
  test_country_crossing_cost_parsing(costing, 500000.f, default_value);

  costing = Costing::pedestrian;
  default_value = kDefaultPedestrian_CountryCrossingCost;
  test_country_crossing_cost_parsing(costing, default_value, default_value);
  test_country_crossing_cost_parsing(costing, 2.f, 2.f);
  test_country_crossing_cost_parsing(costing, 700.f, 700.f);
  test_country_crossing_cost_parsing(costing, -2.f, default_value);
  test_country_crossing_cost_parsing(costing, 500000.f, default_value);

  costing = Costing::bicycle;
  default_value = kDefaultBicycle_CountryCrossingCost;
  test_country_crossing_cost_parsing(costing, default_value, default_value);
  test_country_crossing_cost_parsing(costing, 300.f, 300.f);
  test_country_crossing_cost_parsing(costing, 1200.f, 1200.f);
  test_country_crossing_cost_parsing(costing, -2.f, default_value);
  test_country_crossing_cost_parsing(costing, 50000.f, default_value);

  costing = Costing::truck;
  default_value = kDefaultTruck_CountryCrossingCost;
  test_country_crossing_cost_parsing(costing, default_value, default_value);
  test_country_crossing_cost_parsing(costing, 300.f, 300.f);
  test_country_crossing_cost_parsing(costing, 1200.f, 1200.f);
  test_country_crossing_cost_parsing(costing, -2.f, default_value);
  test_country_crossing_cost_parsing(costing, 50000.f, default_value);
}

TEST(ParseRequest, test_country_crossing_penalty) {
  float default_value = kDefaultAuto_CountryCrossingPenalty;
  for (auto costing : get_base_auto_costing_list()) {
    test_country_crossing_penalty_parsing(costing, default_value, default_value);
    test_country_crossing_penalty_parsing(costing, 2.f, 2.f);
    test_country_crossing_penalty_parsing(costing, 60.f, 60.f);
    test_country_crossing_penalty_parsing(costing, -2.f, default_value);
    test_country_crossing_penalty_parsing(costing, 500000.f, default_value);
  }

  Costing costing = Costing::motor_scooter;
  default_value = kDefaultMotorScooter_CountryCrossingPenalty;
  test_country_crossing_penalty_parsing(costing, default_value, default_value);
  test_country_crossing_penalty_parsing(costing, 2.f, 2.f);
  test_country_crossing_penalty_parsing(costing, 60.f, 60.f);
  test_country_crossing_penalty_parsing(costing, -2.f, default_value);
  test_country_crossing_penalty_parsing(costing, 500000.f, default_value);

  costing = Costing::motorcycle;
  default_value = kDefaultMotorcycle_CountryCrossingPenalty;
  test_country_crossing_penalty_parsing(costing, default_value, default_value);
  test_country_crossing_penalty_parsing(costing, 2.f, 2.f);
  test_country_crossing_penalty_parsing(costing, 60.f, 60.f);
  test_country_crossing_penalty_parsing(costing, -2.f, default_value);
  test_country_crossing_penalty_parsing(costing, 500000.f, default_value);

  costing = Costing::pedestrian;
  default_value = kDefaultPedestrian_CountryCrossingPenalty;
  test_country_crossing_penalty_parsing(costing, default_value, default_value);
  test_country_crossing_penalty_parsing(costing, 2.f, 2.f);
  test_country_crossing_penalty_parsing(costing, 60.f, 60.f);
  test_country_crossing_penalty_parsing(costing, -2.f, default_value);
  test_country_crossing_penalty_parsing(costing, 500000.f, default_value);

  costing = Costing::bicycle;
  default_value = kDefaultBicycle_CountryCrossingPenalty;
  test_country_crossing_penalty_parsing(costing, default_value, default_value);
  test_country_crossing_penalty_parsing(costing, 60.f, 60.f);
  test_country_crossing_penalty_parsing(costing, -2.f, default_value);
  test_country_crossing_penalty_parsing(costing, 50000.f, default_value);

  costing = Costing::truck;
  default_value = kDefaultTruck_CountryCrossingPenalty;
  test_country_crossing_penalty_parsing(costing, default_value, default_value);
  test_country_crossing_penalty_parsing(costing, 60.f, 60.f);
  test_country_crossing_penalty_parsing(costing, -2.f, default_value);
  test_country_crossing_penalty_parsing(costing, 50000.f, default_value);
}

TEST(ParseRequest, test_ferry_cost) {
  float default_value = kDefaultAuto_FerryCost;
  for (auto costing : get_base_auto_costing_list()) {
    test_ferry_cost_parsing(costing, default_value, default_value);
    test_ferry_cost_parsing(costing, 2.f, 2.f);
    test_ferry_cost_parsing(costing, 600.f, 600.f);
    test_ferry_cost_parsing(costing, -2.f, default_value);
    test_ferry_cost_parsing(costing, 500000.f, default_value);
  }

  Costing costing = Costing::motor_scooter;
  default_value = kDefaultMotorScooter_FerryCost;
  test_ferry_cost_parsing(costing, default_value, default_value);
  test_ferry_cost_parsing(costing, 2.f, 2.f);
  test_ferry_cost_parsing(costing, 600.f, 600.f);
  test_ferry_cost_parsing(costing, -2.f, default_value);
  test_ferry_cost_parsing(costing, 500000.f, default_value);

  costing = Costing::motorcycle;
  default_value = kDefaultMotorcycle_FerryCost;
  test_ferry_cost_parsing(costing, default_value, default_value);
  test_ferry_cost_parsing(costing, 2.f, 2.f);
  test_ferry_cost_parsing(costing, 600.f, 600.f);
  test_ferry_cost_parsing(costing, -2.f, default_value);
  test_ferry_cost_parsing(costing, 500000.f, default_value);

  costing = Costing::pedestrian;
  default_value = kDefaultPedestrian_FerryCost;
  test_ferry_cost_parsing(costing, default_value, default_value);
  test_ferry_cost_parsing(costing, 2.f, 2.f);
  test_ferry_cost_parsing(costing, 600.f, 600.f);
  test_ferry_cost_parsing(costing, -2.f, default_value);
  test_ferry_cost_parsing(costing, 500000.f, default_value);

  costing = Costing::bicycle;
  default_value = kDefaultBicycle_FerryCost;
  test_ferry_cost_parsing(costing, default_value, default_value);
  test_ferry_cost_parsing(costing, 150.f, 150.f);
  test_ferry_cost_parsing(costing, 600.f, 600.f);
  test_ferry_cost_parsing(costing, -2.f, default_value);
  test_ferry_cost_parsing(costing, 50000.f, default_value);
}

TEST(ParseRequest, test_use_ferry) {
  float default_value = kDefaultAuto_UseFerry;
  for (auto costing : get_base_auto_costing_list()) {
    test_use_ferry_parsing(costing, default_value, default_value);
    test_use_ferry_parsing(costing, 0.2f, 0.2f);
    test_use_ferry_parsing(costing, 0.6f, 0.6f);
    test_use_ferry_parsing(costing, -2.f, default_value);
    test_use_ferry_parsing(costing, 2.f, default_value);
  }

  Costing costing = Costing::motor_scooter;
  default_value = kDefaultMotorScooter_UseFerry;
  test_use_ferry_parsing(costing, default_value, default_value);
  test_use_ferry_parsing(costing, 0.2f, 0.2f);
  test_use_ferry_parsing(costing, 0.6f, 0.6f);
  test_use_ferry_parsing(costing, -2.f, default_value);
  test_use_ferry_parsing(costing, 2.f, default_value);

  costing = Costing::motorcycle;
  default_value = kDefaultMotorcycle_UseFerry;
  test_use_ferry_parsing(costing, default_value, default_value);
  test_use_ferry_parsing(costing, 0.2f, 0.2f);
  test_use_ferry_parsing(costing, 0.6f, 0.6f);
  test_use_ferry_parsing(costing, -2.f, default_value);
  test_use_ferry_parsing(costing, 2.f, default_value);

  costing = Costing::pedestrian;
  default_value = kDefaultPedestrian_UseFerry;
  test_use_ferry_parsing(costing, default_value, default_value);
  test_use_ferry_parsing(costing, 0.2f, 0.2f);
  test_use_ferry_parsing(costing, 0.6f, 0.6f);
  test_use_ferry_parsing(costing, -2.f, default_value);
  test_use_ferry_parsing(costing, 2.f, default_value);

  costing = Costing::bicycle;
  default_value = kDefaultBicycle_UseFerry;
  test_use_ferry_parsing(costing, default_value, default_value);
  test_use_ferry_parsing(costing, 0.2f, 0.2f);
  test_use_ferry_parsing(costing, 0.6f, 0.6f);
  test_use_ferry_parsing(costing, -2.f, default_value);
  test_use_ferry_parsing(costing, 2.f, default_value);
}

TEST(ParseRequest, test_use_living_streets) {
  std::vector<std::pair<Costing, float>> costing_with_defaults;
  for (auto costing : get_base_auto_costing_list())
    costing_with_defaults.emplace_back(costing, kDefaultAuto_UseLivingStreets);
  costing_with_defaults.emplace_back(Costing::truck, kDefaultTruck_UseLivingStreets);
  costing_with_defaults.emplace_back(Costing::motor_scooter, kDefaultMotorScooter_UseLivingStreets);
  costing_with_defaults.emplace_back(Costing::motorcycle, kDefaultMotorcycle_UseLivingStreets);
  costing_with_defaults.emplace_back(Costing::pedestrian, kDefaultPedestrian_UseLivingStreets);
  costing_with_defaults.emplace_back(Costing::bicycle, kDefaultBicycle_UseLivingStreets);

  for (const auto& p : costing_with_defaults) {
    const auto& costing = p.first;
    const auto& default_value = p.second;
    test_use_living_streets_parsing(costing, default_value, default_value);
    test_use_living_streets_parsing(costing, 0.2f, 0.2f);
    test_use_living_streets_parsing(costing, 0.6f, 0.6f);
    test_use_living_streets_parsing(costing, -2.f, default_value);
    test_use_living_streets_parsing(costing, 2.f, default_value);
  }
}

TEST(ParseRequest, test_service_penalty) {
  std::vector<std::pair<Costing, float>> costing_with_defaults;
  for (auto costing : get_base_auto_costing_list())
    costing_with_defaults.emplace_back(costing, kDefaultAuto_ServicePenalty);
  costing_with_defaults.emplace_back(Costing::truck, kDefaultTruck_ServicePenalty);
  costing_with_defaults.emplace_back(Costing::motor_scooter, kDefaultMotorScooter_ServicePenalty);
  costing_with_defaults.emplace_back(Costing::motorcycle, kDefaultMotorcycle_ServicePenalty);
  costing_with_defaults.emplace_back(Costing::pedestrian, kDefaultPedestrian_ServicePenalty);
  costing_with_defaults.emplace_back(Costing::bicycle, kDefaultBicycle_ServicePenalty);

  for (const auto& p : costing_with_defaults) {
    const auto& costing = p.first;
    const auto& default_value = p.second;
    test_service_penalty_parsing(costing, default_value, default_value);
    test_service_penalty_parsing(costing, 0.2f, 0.2f);
    test_service_penalty_parsing(costing, 600.f, 600.f);
    test_service_penalty_parsing(costing, -2.f, default_value);
    test_service_penalty_parsing(costing, 50000.f, default_value);
  }
}

TEST(ParseRequest, test_service_factor) {
  std::vector<std::pair<Costing, float>> costing_with_defaults;
  for (auto costing : get_base_auto_costing_list())
    costing_with_defaults.emplace_back(costing, kDefaultAuto_ServiceFactor);
  costing_with_defaults.emplace_back(Costing::truck, kDefaultTruck_ServiceFactor);
  costing_with_defaults.emplace_back(Costing::motor_scooter, kDefaultMotorScooter_ServiceFactor);
  costing_with_defaults.emplace_back(Costing::motorcycle, kDefaultMotorcycle_ServiceFactor);
  costing_with_defaults.emplace_back(Costing::pedestrian, kDefaultPedestrian_ServiceFactor);

  for (const auto& p : costing_with_defaults) {
    const auto& costing = p.first;
    const auto& default_value = p.second;
    test_service_factor_parsing(costing, default_value, default_value);
    test_service_factor_parsing(costing, 0.5f, 0.5f);
    test_service_factor_parsing(costing, 10.f, 10.f);
    test_service_factor_parsing(costing, -2.f, default_value);
    test_service_factor_parsing(costing, 200000.f, default_value);
  }
}

TEST(ParseRequest, test_closure_factor) {
  for (const auto& costing : get_all_motor_costings()) {
    test_closure_factor_parsing(costing, 0.f, kDefaultClosureFactor);
    test_closure_factor_parsing(costing, 1.0f, 1.0f);
    test_closure_factor_parsing(costing, 5.0f, 5.0f);
    test_closure_factor_parsing(costing, 9.0f, 9.0f);
    test_closure_factor_parsing(costing, 10.f, 10.f);
    test_closure_factor_parsing(costing, 100.f, kDefaultClosureFactor);
  }
}

TEST(ParseRequest, test_use_highways) {
  float default_value = kDefaultAuto_UseHighways;
  for (auto costing : get_base_auto_costing_list()) {
    test_use_highways_parsing(costing, default_value, default_value);
    test_use_highways_parsing(costing, 0.2f, 0.2f);
    test_use_highways_parsing(costing, 0.6f, 0.6f);
    test_use_highways_parsing(costing, -2.f, default_value);
    test_use_highways_parsing(costing, 2.f, default_value);
  }

  Costing costing = Costing::motorcycle;
  default_value = kDefaultMotorcycle_UseHighways;
  test_use_highways_parsing(costing, default_value, default_value);
  test_use_highways_parsing(costing, 0.2f, 0.2f);
  test_use_highways_parsing(costing, 0.6f, 0.6f);
  test_use_highways_parsing(costing, -2.f, default_value);
  test_use_highways_parsing(costing, 2.f, default_value);
}

TEST(ParseRequest, test_use_tolls) {
  float default_value = kDefaultAuto_UseTolls;
  for (auto costing : get_base_auto_costing_list()) {
    test_use_tolls_parsing(costing, default_value, default_value);
    test_use_tolls_parsing(costing, 0.2f, 0.2f);
    test_use_tolls_parsing(costing, 0.6f, 0.6f);
    test_use_tolls_parsing(costing, -2.f, default_value);
    test_use_tolls_parsing(costing, 2.f, default_value);
  }

  Costing costing = Costing::motorcycle;
  default_value = kDefaultMotorcycle_UseTolls;
  test_use_tolls_parsing(costing, default_value, default_value);
  test_use_tolls_parsing(costing, 0.2f, 0.2f);
  test_use_tolls_parsing(costing, 0.6f, 0.6f);
  test_use_tolls_parsing(costing, -2.f, default_value);
  test_use_tolls_parsing(costing, 2.f, default_value);
}

TEST(ParseRequest, test_use_tracks) {
  std::vector<std::pair<Costing, float>> costing_with_defaults;
  for (auto costing : get_base_auto_costing_list())
    costing_with_defaults.emplace_back(costing, kDefaultTruck_UseTracks);
  costing_with_defaults.emplace_back(Costing::truck, kDefaultTruck_UseTracks);

  for (const auto& p : costing_with_defaults) {
    const auto& costing = p.first;
    const auto& default_value = p.second;
    test_use_tracks_parsing(costing, default_value, default_value);
    test_use_tracks_parsing(costing, 0.2f, 0.2f);
    test_use_tracks_parsing(costing, 0.6f, 0.6f);
    test_use_tracks_parsing(costing, -2.f, default_value);
    test_use_tracks_parsing(costing, 2.f, default_value);
  }
}

TEST(ParseRequest, test_use_hills) {
  Costing costing = Costing::motor_scooter;
  float default_value = kDefaultMotorScooter_UseHills;
  test_use_hills_parsing(costing, default_value, default_value);
  test_use_hills_parsing(costing, 0.2f, 0.2f);
  test_use_hills_parsing(costing, 0.6f, 0.6f);
  test_use_hills_parsing(costing, -2.f, default_value);
  test_use_hills_parsing(costing, 2.f, default_value);

  costing = Costing::bicycle;
  default_value = kDefaultBicycle_UseHills;
  test_use_hills_parsing(costing, default_value, default_value);
  test_use_hills_parsing(costing, 0.15f, 0.15f);
  test_use_hills_parsing(costing, 0.5f, 0.5f);
  test_use_hills_parsing(costing, -2.f, default_value);
  test_use_hills_parsing(costing, 2.f, default_value);
}

TEST(ParseRequest, test_use_primary) {
  Costing costing = Costing::motor_scooter;
  float default_value = kDefaultMotorScooter_UsePrimary;
  test_use_primary_parsing(costing, default_value, default_value);
  test_use_primary_parsing(costing, 0.2f, 0.2f);
  test_use_primary_parsing(costing, 0.6f, 0.6f);
  test_use_primary_parsing(costing, -2.f, default_value);
  test_use_primary_parsing(costing, 2.f, default_value);
}

TEST(ParseRequest, test_top_speed) {
  Costing costing = Costing::motor_scooter;
  float default_value = kDefaultMotorScooter_TopSpeed;
  test_top_speed_parsing(costing, default_value, default_value);
  test_top_speed_parsing(costing, 25, 25);
  test_top_speed_parsing(costing, 50, 50);
  test_top_speed_parsing(costing, -2, default_value);
  test_top_speed_parsing(costing, 200, default_value);
}

TEST(ParseRequest, test_use_trails) {
  Costing costing = Costing::motorcycle;
  float default_value = kDefaultMotorcycle_UseTrails;
  test_use_trails_parsing(costing, default_value, default_value);
  test_use_trails_parsing(costing, 0.2f, 0.2f);
  test_use_trails_parsing(costing, 0.6f, 0.6f);
  test_use_trails_parsing(costing, -2.f, default_value);
  test_use_trails_parsing(costing, 2.f, default_value);
}

TEST(ParseRequest, test_max_distance) {
  Costing costing = Costing::pedestrian;

  std::string transport_type = "foot";
  uint32_t default_value = kDefaultPedestrian_MaxDistanceFoot;
  test_max_distance_parsing(costing, transport_type, default_value, default_value);
  test_max_distance_parsing(costing, transport_type, 50, 50);
  test_max_distance_parsing(costing, transport_type, 200000, default_value);

  transport_type = "wheelchair";
  default_value = kDefaultPedestrian_MaxDistanceWheelchair;
  test_max_distance_parsing(costing, transport_type, default_value, default_value);
  test_max_distance_parsing(costing, transport_type, 50, 50);
  test_max_distance_parsing(costing, transport_type, 200000, default_value);
}

TEST(ParseRequest, test_walking_speed) {
  Costing costing = Costing::pedestrian;

  std::string transport_type = "foot";
  float default_value = kDefaultPedestrian_SpeedFoot;
  test_walking_speed_parsing(costing, transport_type, default_value, default_value);
  test_walking_speed_parsing(costing, transport_type, 2.5f, 2.5f);
  test_walking_speed_parsing(costing, transport_type, 6.f, 6.f);
  test_walking_speed_parsing(costing, transport_type, -2.f, default_value);
  test_walking_speed_parsing(costing, transport_type, 50.f, default_value);

  transport_type = "wheelchair";
  default_value = kDefaultPedestrian_SpeedWheelchair;
  test_walking_speed_parsing(costing, transport_type, default_value, default_value);
  test_walking_speed_parsing(costing, transport_type, 2.f, 2.f);
  test_walking_speed_parsing(costing, transport_type, 5.f, 5.f);
  test_walking_speed_parsing(costing, transport_type, -2.f, default_value);
  test_walking_speed_parsing(costing, transport_type, 50.f, default_value);
}

TEST(ParseRequest, test_step_penalty) {
  Costing costing = Costing::pedestrian;

  std::string transport_type = "foot";
  float default_value = kDefaultPedestrian_StepPenaltyFoot;
  test_step_penalty_parsing(costing, transport_type, default_value, default_value);
  test_step_penalty_parsing(costing, transport_type, 10.f, 10.f);
  test_step_penalty_parsing(costing, transport_type, 40.f, 40.f);
  test_step_penalty_parsing(costing, transport_type, -2.f, default_value);
  test_step_penalty_parsing(costing, transport_type, 500000.f, default_value);

  transport_type = "wheelchair";
  default_value = kDefaultPedestrian_StepPenaltyWheelchair;
  test_step_penalty_parsing(costing, transport_type, default_value, default_value);
  test_step_penalty_parsing(costing, transport_type, 400.f, 400.f);
  test_step_penalty_parsing(costing, transport_type, 800.f, 800.f);
  test_step_penalty_parsing(costing, transport_type, -2.f, default_value);
  test_step_penalty_parsing(costing, transport_type, 500000.f, default_value);
}

TEST(ParseRequest, test_max_grade) {
  Costing costing = Costing::pedestrian;

  std::string transport_type = "foot";
  uint32_t default_value = kDefaultPedestrian_MaxGradeFoot;
  test_max_grade_parsing(costing, transport_type, default_value, default_value);
  test_max_grade_parsing(costing, transport_type, 30, 30);
  test_max_grade_parsing(costing, transport_type, 100, default_value);

  transport_type = "wheelchair";
  default_value = kDefaultPedestrian_MaxGradeWheelchair;
  test_max_grade_parsing(costing, transport_type, default_value, default_value);
  test_max_grade_parsing(costing, transport_type, 10, 10);
  test_max_grade_parsing(costing, transport_type, 14, 14);
  test_max_grade_parsing(costing, transport_type, 100, default_value);
}

TEST(ParseRequest, test_max_hiking_difficulty) {
  Costing costing = Costing::pedestrian;
  float default_value = kDefaultPedestrian_MaxHikingDifficulty;
  test_max_hiking_difficulty_parsing(costing, default_value, default_value);
  test_max_hiking_difficulty_parsing(costing, 3, 3);
  test_max_hiking_difficulty_parsing(costing, 10, default_value);
}

TEST(ParseRequest, test_mode_factor) {
  Costing costing = Costing::pedestrian;
  float default_value = kDefaultPedestrian_ModeFactor;
  test_mode_factor_parsing(costing, default_value, default_value);
  test_mode_factor_parsing(costing, 1.f, 1.f);
  test_mode_factor_parsing(costing, 10.f, 10.f);
  test_mode_factor_parsing(costing, -2.f, default_value);
  test_mode_factor_parsing(costing, 200000.f, default_value);

  costing = Costing::transit;
  default_value = kDefaultTransit_ModeFactor;
  test_mode_factor_parsing(costing, default_value, default_value);
  test_mode_factor_parsing(costing, 10.f, 10.f);
  test_mode_factor_parsing(costing, -2.f, default_value);
  test_mode_factor_parsing(costing, 200000.f, default_value);
}

TEST(ParseRequest, test_walkway_factor) {
  Costing costing = Costing::pedestrian;
  float default_value = kDefaultPedestrian_WalkwayFactor;
  test_walkway_factor_parsing(costing, default_value, default_value);
  test_walkway_factor_parsing(costing, 0.5f, 0.5f);
  test_walkway_factor_parsing(costing, 10.f, 10.f);
  test_walkway_factor_parsing(costing, -2.f, default_value);
  test_walkway_factor_parsing(costing, 200000.f, default_value);
}

TEST(ParseRequest, test_sidewalk_factor) {
  Costing costing = Costing::pedestrian;
  float default_value = kDefaultPedestrian_SideWalkFactor;
  test_sidewalk_factor_parsing(costing, default_value, default_value);
  test_sidewalk_factor_parsing(costing, 0.5f, 0.5f);
  test_sidewalk_factor_parsing(costing, 10.f, 10.f);
  test_sidewalk_factor_parsing(costing, -2.f, default_value);
  test_sidewalk_factor_parsing(costing, 200000.f, default_value);
}

TEST(ParseRequest, test_alley_factor) {
  Costing costing = Costing::pedestrian;
  float default_value = kDefaultPedestrian_AlleyFactor;
  test_alley_factor_parsing(costing, default_value, default_value);
  test_alley_factor_parsing(costing, 1.f, 1.f);
  test_alley_factor_parsing(costing, 10.f, 10.f);
  test_alley_factor_parsing(costing, -2.f, default_value);
  test_alley_factor_parsing(costing, 200000.f, default_value);
}

TEST(ParseRequest, test_driveway_factor) {
  Costing costing = Costing::pedestrian;
  float default_value = kDefaultPedestrian_DrivewayFactor;
  test_driveway_factor_parsing(costing, default_value, default_value);
  test_driveway_factor_parsing(costing, 1.f, 1.f);
  test_driveway_factor_parsing(costing, 10.f, 10.f);
  test_driveway_factor_parsing(costing, -2.f, default_value);
  test_driveway_factor_parsing(costing, 200000.f, default_value);
}

TEST(ParseRequest, test_transit_start_end_max_distance) {
  Costing costing = Costing::pedestrian;
  float default_value = kDefaultPedestrian_TransitStartEndMaxDistance;
  test_transit_start_end_max_distance_parsing(costing, default_value, default_value);
  test_transit_start_end_max_distance_parsing(costing, 3000, 3000);
  test_transit_start_end_max_distance_parsing(costing, 200000, default_value);
}

TEST(ParseRequest, test_transit_transfer_max_distance) {
  Costing costing = Costing::pedestrian;
  float default_value = kDefaultPedestrian_TransitTransferMaxDistance;
  test_transit_transfer_max_distance_parsing(costing, default_value, default_value);
  test_transit_transfer_max_distance_parsing(costing, 1500, 1500);
  test_transit_transfer_max_distance_parsing(costing, 100000, default_value);
}

TEST(ParseRequest, test_use_roads) {
  Costing costing = Costing::bicycle;
  float default_value = kDefaultBicycle_UseRoad;
  test_use_roads_parsing(costing, default_value, default_value);
  test_use_roads_parsing(costing, 0.1f, 0.1f);
  test_use_roads_parsing(costing, 0.5f, 0.5f);
  test_use_roads_parsing(costing, -2.f, default_value);
  test_use_roads_parsing(costing, 2.f, default_value);
}

TEST(ParseRequest, test_avoid_bad_surfaces) {
  Costing costing = Costing::bicycle;
  float default_value = kDefaultBicycle_AvoidBadSurfaces;
  test_avoid_bad_surfaces_parsing(costing, default_value, default_value);
  test_avoid_bad_surfaces_parsing(costing, 0.1f, 0.1f);
  test_avoid_bad_surfaces_parsing(costing, 0.5f, 0.5f);
  test_avoid_bad_surfaces_parsing(costing, -2.f, default_value);
  test_avoid_bad_surfaces_parsing(costing, 2.f, default_value);
}

TEST(ParseRequest, test_cycling_speed) {
  Costing costing = Costing::bicycle;

  std::string transport_type = "Road";
  float default_value =
      kDefaultBicycle_CyclingSpeed[static_cast<uint32_t>(valhalla::sif::BicycleType::kRoad)];
  test_cycling_speed_parsing(costing, transport_type, default_value, default_value);
  test_cycling_speed_parsing(costing, transport_type, 23.5f, 23.5f);
  test_cycling_speed_parsing(costing, transport_type, 26.5f, 26.5f);
  test_cycling_speed_parsing(costing, transport_type, 2.f, default_value);
  test_cycling_speed_parsing(costing, transport_type, 70.f, default_value);

  transport_type = "Cross";
  default_value =
      kDefaultBicycle_CyclingSpeed[static_cast<uint32_t>(valhalla::sif::BicycleType::kCross)];
  test_cycling_speed_parsing(costing, transport_type, default_value, default_value);
  test_cycling_speed_parsing(costing, transport_type, 18.5f, 18.5f);
  test_cycling_speed_parsing(costing, transport_type, 21.5f, 21.5f);
  test_cycling_speed_parsing(costing, transport_type, 2.f, default_value);
  test_cycling_speed_parsing(costing, transport_type, 70.f, default_value);

  transport_type = "Hybrid";
  default_value =
      kDefaultBicycle_CyclingSpeed[static_cast<uint32_t>(valhalla::sif::BicycleType::kHybrid)];
  test_cycling_speed_parsing(costing, transport_type, default_value, default_value);
  test_cycling_speed_parsing(costing, transport_type, 16.5f, 16.5f);
  test_cycling_speed_parsing(costing, transport_type, 19.5f, 19.5f);
  test_cycling_speed_parsing(costing, transport_type, 2.f, default_value);
  test_cycling_speed_parsing(costing, transport_type, 70.f, default_value);

  transport_type = "Mountain";
  default_value =
      kDefaultBicycle_CyclingSpeed[static_cast<uint32_t>(valhalla::sif::BicycleType::kMountain)];
  test_cycling_speed_parsing(costing, transport_type, default_value, default_value);
  test_cycling_speed_parsing(costing, transport_type, 14.5f, 14.5f);
  test_cycling_speed_parsing(costing, transport_type, 17.5f, 17.5f);
  test_cycling_speed_parsing(costing, transport_type, 2.f, default_value);
  test_cycling_speed_parsing(costing, transport_type, 70.f, default_value);
}

TEST(ParseRequest, test_low_class_penalty) {
  Costing costing = Costing::truck;
  float default_value = kDefaultTruck_LowClassPenalty;
  test_low_class_penalty_parsing(costing, default_value, default_value);
  test_low_class_penalty_parsing(costing, 15.f, 15.f);
  test_low_class_penalty_parsing(costing, 60.f, 60.f);
  test_low_class_penalty_parsing(costing, -2.f, default_value);
  test_low_class_penalty_parsing(costing, 50000.f, default_value);
}

TEST(ParseRequest, test_weight) {
  Costing costing = Costing::truck;
  float default_value = kDefaultTruck_TruckWeight;
  test_weight_parsing(costing, default_value, default_value);
  test_weight_parsing(costing, 15.f, 15.f);
  test_weight_parsing(costing, 30.f, 30.f);
  test_weight_parsing(costing, -2.f, default_value);
  test_weight_parsing(costing, 200.f, default_value);
}

TEST(ParseRequest, test_axle_load) {
  Costing costing = Costing::truck;
  float default_value = kDefaultTruck_TruckAxleLoad;
  test_axle_load_parsing(costing, default_value, default_value);
  test_axle_load_parsing(costing, 5.f, 5.f);
  test_axle_load_parsing(costing, 15.f, 15.f);
  test_axle_load_parsing(costing, -2.f, default_value);
  test_axle_load_parsing(costing, 100.f, default_value);
}

TEST(ParseRequest, test_height) {
  Costing costing = Costing::truck;
  float default_value = kDefaultTruck_TruckHeight;
  test_height_parsing(costing, default_value, default_value);
  test_height_parsing(costing, 2.f, 2.f);
  test_height_parsing(costing, 8.f, 8.f);
  test_height_parsing(costing, -2.f, default_value);
  test_height_parsing(costing, 20.f, default_value);
}

TEST(ParseRequest, test_width) {
  Costing costing = Costing::truck;
  float default_value = kDefaultTruck_TruckWidth;
  test_width_parsing(costing, default_value, default_value);
  test_width_parsing(costing, 2.f, 2.f);
  test_width_parsing(costing, 4.f, 4.f);
  test_width_parsing(costing, -2.f, default_value);
  test_width_parsing(costing, 20.f, default_value);
}

TEST(ParseRequest, test_length) {
  Costing costing = Costing::truck;
  float default_value = kDefaultTruck_TruckLength;
  test_length_parsing(costing, default_value, default_value);
  test_length_parsing(costing, 15.f, 15.f);
  test_length_parsing(costing, 40.f, 40.f);
  test_length_parsing(costing, -2.f, default_value);
  test_length_parsing(costing, 100.f, default_value);
}

TEST(ParseRequest, test_hazmat) {
  Costing costing = Costing::truck;
  bool default_value = false;
  test_hazmat_parsing(costing, default_value, default_value);
  test_hazmat_parsing(costing, true, true);
  test_hazmat_parsing(costing, false, false);
}

TEST(ParseRequest, test_wheelchair) {
  Costing costing = Costing::transit;
  bool default_value = false;
  test_wheelchair_parsing(costing, default_value, default_value);
  test_wheelchair_parsing(costing, true, true);
  test_wheelchair_parsing(costing, false, false);
}

TEST(ParseRequest, test_bicycle) {
  Costing costing = Costing::transit;
  bool default_value = false;
  test_bicycle_parsing(costing, default_value, default_value);
  test_bicycle_parsing(costing, true, true);
  test_bicycle_parsing(costing, false, false);
}

TEST(ParseRequest, test_use_bus) {
  Costing costing = Costing::transit;
  float default_value = kDefaultTransit_UseBus;
  test_use_bus_parsing(costing, default_value, default_value);
  test_use_bus_parsing(costing, 0.2f, 0.2f);
  test_use_bus_parsing(costing, 0.4f, 0.4f);
  test_use_bus_parsing(costing, -2.f, default_value);
  test_use_bus_parsing(costing, 2.f, default_value);
}

TEST(ParseRequest, test_use_rail) {
  Costing costing = Costing::transit;
  float default_value = kDefaultTransit_UseRail;
  test_use_rail_parsing(costing, default_value, default_value);
  test_use_rail_parsing(costing, 0.3f, 0.3f);
  test_use_rail_parsing(costing, 0.9f, 0.9f);
  test_use_rail_parsing(costing, -2.f, default_value);
  test_use_rail_parsing(costing, 2.f, default_value);
}

TEST(ParseRequest, test_use_transfers) {
  Costing costing = Costing::transit;
  float default_value = kDefaultTransit_UseTransfers;
  test_use_transfers_parsing(costing, default_value, default_value);
  test_use_transfers_parsing(costing, 0.2f, 0.2f);
  test_use_transfers_parsing(costing, 0.4f, 0.4f);
  test_use_transfers_parsing(costing, -2.f, default_value);
  test_use_transfers_parsing(costing, 2.f, default_value);
}

TEST(ParseRequest, test_transfer_cost) {
  Costing costing = Costing::transit;
  float default_value = kDefaultTransit_TransferCost;
  test_transfer_cost_parsing(costing, default_value, default_value);
  test_transfer_cost_parsing(costing, 10.f, 10.f);
  test_transfer_cost_parsing(costing, 15.f, 15.f);
  test_transfer_cost_parsing(costing, -2.f, default_value);
  test_transfer_cost_parsing(costing, 50000.f, default_value);
}

TEST(ParseRequest, test_transfer_penalty) {
  Costing costing = Costing::transit;
  float default_value = kDefaultTransit_TransferPenalty;
  test_transfer_penalty_parsing(costing, default_value, default_value);
  test_transfer_penalty_parsing(costing, 150.f, 150.f);
  test_transfer_penalty_parsing(costing, 600.f, 600.f);
  test_transfer_penalty_parsing(costing, -2.f, default_value);
  test_transfer_penalty_parsing(costing, 50000.f, default_value);
}

TEST(ParseRequest, test_stops_transit_filter) {
  Costing costing = Costing::transit;

  valhalla::FilterAction filter_action = valhalla::FilterAction::exclude;
  std::vector<std::string> filter_ids = {"stop1", "stop2", "stop3"};
  test_filter_stop_parsing(costing, filter_action, filter_ids);

  filter_action = valhalla::FilterAction::include;
  filter_ids = {"stop10", "stop20", "stop30"};
  test_filter_stop_parsing(costing, filter_action, filter_ids);
}

TEST(ParseRequest, test_routes_transit_filter) {
  Costing costing = Costing::transit;

  valhalla::FilterAction filter_action = valhalla::FilterAction::exclude;
  std::vector<std::string> filter_ids = {"route1", "route2", "route3"};
  test_filter_route_parsing(costing, filter_action, filter_ids);

  filter_action = valhalla::FilterAction::include;
  filter_ids = {"route10", "route20", "route30"};
  test_filter_route_parsing(costing, filter_action, filter_ids);
}

TEST(ParseRequest, test_operators_transit_filter) {
  Costing costing = Costing::transit;

  valhalla::FilterAction filter_action = valhalla::FilterAction::exclude;
  std::vector<std::string> filter_ids = {"operator1", "operator2", "operator3"};
  test_filter_operator_parsing(costing, filter_action, filter_ids);

  filter_action = valhalla::FilterAction::include;
  filter_ids = {"operator10", "operator20", "operator30"};
  test_filter_operator_parsing(costing, filter_action, filter_ids);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
