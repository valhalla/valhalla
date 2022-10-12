#include <iostream>
#include <sstream>

#include <benchmark/benchmark.h>
#include <boost/property_tree/ptree.hpp>

#include "baldr/rapidjson_utils.h"
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

constexpr float kGpsAccuracyMeters = 4.07;
constexpr float kSearchRadiusMeters = 50;

// Inline benchmarks for OfflineMatch

class OfflineMapmatchFixture : public benchmark::Fixture {
public:
  void SetUp(const ::benchmark::State& state) {
    (void)state;
    InitEngineConfig();
    InitMapMatcher();
  }

  void TearDown(const ::benchmark::State& state) {
    (void)state;
    matcher_factory_->ClearCache();
    mapmatcher_->Clear();
    matcher_factory_.reset();
    mapmatcher_.reset();
  }

private:
  void InitEngineConfig() {
    rapidjson::read_json(VALHALLA_SOURCE_DIR "bench/meili/config.json", config_);
    const rapidjson::Document doc;
    valhalla::sif::ParseCosting(doc, "/costing_options", options_);
    options_.set_costing_type(valhalla::Costing::auto_);
  }

  void InitMapMatcher() {
    matcher_factory_ = std::make_shared<MapMatcherFactory>(config_);
    mapmatcher_.reset(matcher_factory_->Create(options_));
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
  logging::Configure({{"type", ""}});
  const auto& meas = BuildMeasurements(kGpsAccuracyMeters, kSearchRadiusMeters);
  for (auto _ : state) {
    benchmark::DoNotOptimize(mapmatcher_->OfflineMatch(meas));
  }
}

BENCHMARK_REGISTER_F(OfflineMapmatchFixture, BasicOfflineMatch);

// Load fixture files, intended to mirror test cases defined in test/mapmatch.cc.

std::string LoadFile(const std::string& filename) {
  std::stringstream ss;
  std::string line;
  std::ifstream input_file;
  input_file.open(filename.c_str());
  while (std::getline(input_file, line)) {
    ss << line;
  }
  return ss.str();
}

const std::vector<std::string> kBenchmarkCases = {
    // Intersection matching test cases
    VALHALLA_SOURCE_DIR "bench/meili/fixtures/intersection_matching1.json",
    VALHALLA_SOURCE_DIR "bench/meili/fixtures/intersection_matching2.json",
    VALHALLA_SOURCE_DIR "bench/meili/fixtures/intersection_matching3.json",
    // 2.8km loop in Utrecht
    VALHALLA_SOURCE_DIR "bench/meili/fixtures/3km_loop_utrecht.json",
};

static void BM_ManyCases(benchmark::State& state) {
  logging::Configure({{"type", ""}});
  boost::property_tree::ptree config;
  rapidjson::read_json(VALHALLA_SOURCE_DIR "bench/meili/config.json", config);
  valhalla::tyr::actor_t actor(config, true);
  const std::string test_case(LoadFile(kBenchmarkCases[state.range(0)]));
  for (auto _ : state) {
    benchmark::DoNotOptimize(actor.trace_route(test_case));
  }
}

BENCHMARK(BM_ManyCases)->DenseRange(0, kBenchmarkCases.size() - 1);

} // namespace

BENCHMARK_MAIN();
