#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "midgard/encoded.h"
#include "test.h"
#include "tyr/actor.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <string>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

const auto cfg = test::make_config(VALHALLA_BUILD_DIR "test/data/utrecht_tiles");

constexpr double kStartLat = 52.10386;
constexpr double kStartLon = 5.113364;

std::string make_request(const std::string& costing, const std::string& round_trip_options) {
  return R"({"locations":[{"lat":)" + std::to_string(kStartLat) + R"(,"lon":)" +
         std::to_string(kStartLon) + R"(}],"costing":")" + costing + R"(","round_trip_options":)" +
         round_trip_options + R"(,"verbose":true,"format":"osrm"})";
}

// runs only loki, which validates and fills in the round trip options
Api loki_round_trip(const std::string& request_json) {
  auto reader = std::make_shared<GraphReader>(cfg.get_child("mjolnir"));
  loki::loki_worker_t loki_worker(cfg, reader);
  Api request;
  ParseApi(request_json, Options::round_trip, request);
  loki_worker.round_trip(request);
  return request;
}

void expect_exception(const std::string& request_json, unsigned code) {
  tyr::actor_t actor(cfg, true);
  try {
    actor.round_trip(request_json);
    FAIL() << "Expected valhalla_exception_t " << code;
  } catch (const valhalla_exception_t& e) { EXPECT_EQ(e.code, code); } catch (...) {
    FAIL() << "Expected valhalla_exception_t " << code;
  }
}

bool has_warning(const Api& request, unsigned code) {
  const auto& warnings = request.info().warnings();
  return std::any_of(warnings.begin(), warnings.end(),
                     [code](const auto& warning) { return warning.code() == code; });
}

struct RoundTripCase {
  std::string name;
  std::string request;
  // a warning the request has to produce, 0 for none
  unsigned warning = 0;
};

class RoundTripDistance : public ::testing::TestWithParam<RoundTripCase> {};

TEST_P(RoundTripDistance, WithinTolerance) {
  const auto& param = GetParam();
  tyr::actor_t actor(cfg, true);
  Api api;
  const auto json = actor.round_trip(param.request, nullptr, &api);

  if (param.warning) {
    EXPECT_TRUE(has_warning(api, param.warning));
  }

  // a single route with a single leg from the start back to the start
  ASSERT_EQ(api.trip().routes_size(), 1);
  ASSERT_EQ(api.trip().routes(0).legs_size(), 1);
  const auto& leg = api.trip().routes(0).legs(0);
  ASSERT_EQ(leg.location_size(), 2);
  const auto shape = decode<std::vector<PointLL>>(leg.shape());
  ASSERT_GE(shape.size(), 2);
  EXPECT_LT(shape.front().Distance(shape.back()), 1.);

  rapidjson::Document doc;
  doc.Parse(json);
  ASSERT_FALSE(doc.HasParseError());
  EXPECT_STREQ(doc["code"].GetString(), "Ok");
  ASSERT_EQ(doc["routes"].Size(), 1);
  const double distance = doc["routes"][0]["distance"].GetDouble();

  // compare against the options after loki clamped them
  const auto& options = api.options().round_trip_options();

  if (param.warning != 307) {
    EXPECT_NEAR(distance, options.distance(), options.distance_tolerance() * options.distance());
  }
}

INSTANTIATE_TEST_SUITE_P(
    Utrecht,
    RoundTripDistance,
    ::testing::Values(
        RoundTripCase{"BicycleShort",
                      R"({"locations":[{"lat":52.10386,"lon":5.113364}],"costing":"bicycle",
                          "round_trip_options":{"distance":2000,"heading":90},
                          "verbose":true,"format":"osrm"})"},
        RoundTripCase{"BicycleLongLooseTolerance",
                      R"({"locations":[{"lat":52.107412,"lon":5.099205}],"costing":"bicycle",
                          "round_trip_options":{"distance":20000,"heading":180,
                          "distance_tolerance":0.5},"verbose":true,"format":"osrm"})"},
        RoundTripCase{"BicycleVeryShort",
                      R"({"locations":[{"lat":52.087543,"lon":5.085762}],"costing":"bicycle",
                          "round_trip_options":{"distance":300,"heading":90,
                          "distance_tolerance":0.1},"verbose":true,"format":"osrm"})"},
        RoundTripCase{"BicycleToleranceClampedUp",
                      R"({"locations":[{"lat":52.091062,"lon":5.097793}],"costing":"bicycle",
                          "round_trip_options":{"distance":3000,"heading":90,
                          "distance_tolerance":0.01},"verbose":true,"format":"osrm"})",
                      307},
        RoundTripCase{"BicycleToleranceClampedDown",
                      R"({"locations":[{"lat":52.09647,"lon":5.123119}],"costing":"bicycle",
                          "round_trip_options":{"distance":3000,"heading":90,
                          "distance_tolerance":0.6,"heading_tolerance":90},
                          "verbose":true,"format":"osrm"})",
                      307},
        RoundTripCase{"Auto",
                      R"({"locations":[{"lat":52.09647,"lon":5.123119}],"costing":"auto",
                          "round_trip_options":{"distance":3000,"heading":90,
                          "distance_tolerance":0.1,"heading_tolerance":90},
                          "verbose":true,"format":"osrm"})"},
        RoundTripCase{"PedestrianTightTolerance",
                      R"({"locations":[{"lat":52.084521,"lon":5.119936}],"costing":"pedestrian",
                          "round_trip_options":{"distance":5000,"heading":90,
                          "distance_tolerance":0.02,"heading_tolerance":90},
                          "verbose":true,"format":"osrm",
                          "costing_options":{"auto":{"use_highways":0}}})"},
        RoundTripCase{"MotorcycleTightTolerance",
                      R"({"locations":[{"lat":52.084521,"lon":5.119936}],"costing":"motorcycle",
                          "round_trip_options":{"distance":5100,"heading":90,
                          "distance_tolerance":0.02,"heading_tolerance":90},
                          "verbose":true,"format":"osrm",
                          "costing_options":{"auto":{"use_highways":0}}})"}),
    [](const ::testing::TestParamInfo<RoundTripCase>& info) { return info.param.name; });

/**
 * Some loki only tests that check options parsing
 */

TEST(RoundTripOptions, Defaults) {
  const auto request = loki_round_trip(make_request("bicycle", R"({"distance":2000})"));
  const auto& options = request.options().round_trip_options();
  EXPECT_EQ(options.distance(), 2000.);
  EXPECT_EQ(options.heading(), 0.);
  EXPECT_EQ(options.heading_tolerance(), 45.);
  EXPECT_DOUBLE_EQ(options.distance_tolerance(), 0.1);
  EXPECT_EQ(request.info().warnings_size(), 0);
}

TEST(RoundTripOptions, HeadingWrapped) {
  auto request = loki_round_trip(make_request("bicycle", R"({"distance":2000,"heading":-90})"));
  EXPECT_DOUBLE_EQ(request.options().round_trip_options().heading(), 270.);

  request = loki_round_trip(make_request("bicycle", R"({"distance":2000,"heading":450})"));
  EXPECT_DOUBLE_EQ(request.options().round_trip_options().heading(), 90.);
}

TEST(RoundTripOptions, DistanceClamped) {
  const auto max_distance = cfg.get<double>("service_limits.bicycle.max_round_trip_distance");
  const auto request = loki_round_trip(
      make_request("bicycle", R"({"distance":)" + std::to_string(max_distance * 2) + "}"));
  EXPECT_EQ(request.options().round_trip_options().distance(), max_distance);
  EXPECT_TRUE(has_warning(request, 304));
}

TEST(RoundTripOptions, HeadingToleranceClamped) {
  auto request =
      loki_round_trip(make_request("bicycle", R"({"distance":2000,"heading_tolerance":5})"));
  EXPECT_EQ(request.options().round_trip_options().heading_tolerance(), 10.);
  EXPECT_TRUE(has_warning(request, 305));

  request = loki_round_trip(make_request("bicycle", R"({"distance":2000,"heading_tolerance":270})"));
  EXPECT_EQ(request.options().round_trip_options().heading_tolerance(), 180.);
  EXPECT_TRUE(has_warning(request, 305));
}

TEST(RoundTripOptions, DistanceToleranceClamped) {
  auto request =
      loki_round_trip(make_request("bicycle", R"({"distance":2000,"distance_tolerance":0.01})"));
  EXPECT_DOUBLE_EQ(request.options().round_trip_options().distance_tolerance(), 0.02);
  EXPECT_TRUE(has_warning(request, 307));

  request = loki_round_trip(make_request("bicycle", R"({"distance":2000,"distance_tolerance":0.9})"));
  EXPECT_DOUBLE_EQ(request.options().round_trip_options().distance_tolerance(), 0.5);
  EXPECT_TRUE(has_warning(request, 307));
}

/**
 * Check invalid parameters that throw
 */

TEST(RoundTripErrors, NoLocation) {
  expect_exception(R"({"locations":[],"costing":"bicycle","round_trip_options":{"distance":2000}})",
                   120);
}

TEST(RoundTripErrors, TooManyLocations) {
  expect_exception(R"({"locations":[{"lat":52.091062,"lon":5.097793},
                       {"lat":52.091062,"lon":5.097793}],"costing":"bicycle",
                       "round_trip_options":{"distance":3000,"heading":90,
                       "distance_tolerance":0.01},"verbose":true,"format":"osrm"})",
                   150);
}

TEST(RoundTripErrors, UnsupportedCosting) {
  expect_exception(make_request("bikeshare", R"({"distance":2000})"), 125);
}

TEST(RoundTripErrors, MissingDistance) {
  expect_exception(make_request("bicycle", R"({"heading":90})"), 178);
  expect_exception(make_request("bicycle", R"({"distance":-100})"), 178);
}

} // namespace
