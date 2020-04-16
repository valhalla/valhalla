#include <iostream>
#include <vector>

#include <benchmark/benchmark.h>
#include <boost/property_tree/ptree.hpp>
#include <rapidjson/rapidjson.h>

#include "meili/map_matcher_factory.h"
#include "meili/measurement.h"
#include "sif/costconstants.h"
#include "sif/costfactory.h"
#include "tyr/actor.h"

using namespace valhalla::midgard;
using namespace valhalla::meili;
using namespace valhalla::sif;

namespace {

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

constexpr float kGpsAccuracy = 4.07;
constexpr float kSearchRadius = 50;

class OfflineMapmatchFixture : public benchmark::Fixture {
public:
  void SetUp(const ::benchmark::State& state) {
    InitEngineConfig();
    InitMapMatcher();
  }

  void TearDown(const ::benchmark::State& state) {
    matcher_factory_->ClearCache();
    mapmatcher_->Clear();
    matcher_factory_.reset();
    mapmatcher_.reset();
  }

private:
  void InitEngineConfig() {
    rapidjson::read_json(VALHALLA_SOURCE_DIR "bench/meili/config.json", config_);
    const rapidjson::Document doc;
    ParseAutoCostOptions(doc, "/costing_options/auto", options_.add_costing_options());
  }

  void InitMapMatcher() {
    matcher_factory_ = std::make_shared<MapMatcherFactory>(config_);
    mapmatcher_.reset(matcher_factory_->Create(valhalla::Costing::auto_, options_));
  }

protected:
  // Mapmatcher implementation
  std::shared_ptr<MapMatcher> mapmatcher_;
  std::shared_ptr<MapMatcherFactory> matcher_factory_;
  // Mapmatching configuration
  boost::property_tree::ptree config_;
  valhalla::Options options_;
};

std::vector<Measurement> BuildMeasurements(float gps_accuracy, float search_radius) {
  std::vector<Measurement> meas;
  meas.emplace_back(PointLL(5.09806, 52.09110), gps_accuracy, search_radius);
  meas.emplace_back(PointLL(5.09769, 52.09050), gps_accuracy, search_radius);
  meas.emplace_back(PointLL(5.09679, 52.09098), gps_accuracy, search_radius);
  return meas;
}

BENCHMARK_DEFINE_F(OfflineMapmatchFixture, BasicOfflineMatch)(benchmark::State& state) {
  const auto& meas = BuildMeasurements(kGpsAccuracy, kSearchRadius);
  for (auto _ : state) {
    benchmark::DoNotOptimize(mapmatcher_->OfflineMatch(meas));
  }
}

BENCHMARK_REGISTER_F(OfflineMapmatchFixture, BasicOfflineMatch);

const std::vector<std::string> kBenchmarkCases = {
    // Intersection matching test cases
    R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.0981267, "lon": 5.1296180, "type": "break"},
          {"lat": 52.0981280, "lon": 5.1297250, "type": "break"}]})",
    R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.0981346, "lon": 5.1300437, "type": "break"},
          {"lat": 52.0981145, "lon": 5.1309431, "type": "break"},
          {"lat": 52.0980642, "lon": 5.1314993, "type": "break"},
          {"lat": 52.0971149, "lon": 5.1311002, "type": "break"}]})",
    R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.0951641, "lon": 5.1285609, "type": "break"},
          {"lat": 52.0952055, "lon": 5.1292756, "type": "break"},
          {"lat": 52.0952580, "lon": 5.1301359, "type": "break"},
          {"lat": 52.0952939, "lon": 5.1309020, "type": "break"},
          {"lat": 52.0944788, "lon": 5.1304066, "type": "break"} ]})",
    // 2.8km loop in Utrecht
    R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lon": 5.08531221, "lat": 52.0938563, "type": "break"},
          {"lon": 5.0865867, "lat": 52.0930211, "type": "break"},
          {"lon": 5.08769141, "lat": 52.0923946, "type": "break"},
          {"lon": 5.0896245, "lat": 52.0912591, "type": "break"},
          {"lon": 5.0909416, "lat": 52.090737, "type": "break"},
          {"lon": 5.0926623, "lat": 52.0905021, "type": "break"},
          {"lon": 5.0946379, "lat": 52.090737, "type": "break"},
          {"lon": 5.0961035, "lat": 52.0907892, "type": "break"},
          {"lon": 5.097442, "lat": 52.0909328, "type": "break"},
          {"lon": 5.09884401, "lat": 52.09115474, "type": "break"},
          {"lon": 5.100416, "lat": 52.0913244, "type": "break"},
          {"lon": 5.101733, "lat": 52.09137664, "type": "break"},
          {"lon": 5.1034112, "lat": 52.0915854, "type": "break"},
          {"lon": 5.10351751, "lat": 52.09202915, "type": "break"},
          {"lon": 5.102345, "lat": 52.0929627, "type": "break"},
          {"lon": 5.0959337, "lat": 52.093477899999996, "type": "break"},
          {"lon": 5.0932129, "lat": 52.0939153, "type": "break"},
          {"lon": 5.08858141, "lat": 52.094623799999994, "type": "break"},
          {"lon": 5.0858904, "lat": 52.0958159, "type": "break"}]})"};

static void BM_ManyCases(benchmark::State& state) {
  boost::property_tree::ptree config;
  rapidjson::read_json(VALHALLA_SOURCE_DIR "bench/meili/config.json", config);
  valhalla::tyr::actor_t actor(config, true);
  const auto& test_case = kBenchmarkCases[state.range(0)];
  for (auto _ : state) {
    benchmark::DoNotOptimize(actor.trace_route(test_case));
  }
}

BENCHMARK(BM_ManyCases)->DenseRange(0, kBenchmarkCases.size() - 1);

} // namespace

BENCHMARK_MAIN();
