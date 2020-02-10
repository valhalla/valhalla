#include <iostream>
#include <vector>

#include <benchmark/benchmark.h>
#include <boost/property_tree/ptree.hpp>
#include <rapidjson/rapidjson.h>

#include "meili/map_matcher_factory.h"
#include "meili/measurement.h"
#include "sif/costconstants.h"
#include "sif/costfactory.h"

using namespace valhalla::midgard;
using namespace valhalla::meili;
using namespace valhalla::sif;

namespace {

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

class MapmatchFixture : public benchmark::Fixture {
public:
  void SetUp(const ::benchmark::State& state) {
    InitEngineConfig();
    InitMapMatcher();
    InitMeasurements();
  }

  void TearDown(const ::benchmark::State& state) {
    matcher_factory_->ClearCache();
    mapmatcher_->Clear();
    matcher_factory_.reset();
    mapmatcher_.reset();
    measurements_.clear();
  }

private:
  void InitEngineConfig() {
    rapidjson::read_json(VALHALLA_SOURCE_DIR "test/valhalla.json", config_);
    const rapidjson::Document doc;
    ParseAutoCostOptions(doc, "/costing_options/auto", options_.add_costing_options());
  }

  void InitMapMatcher() {
    matcher_factory_ = std::make_shared<MapMatcherFactory>(config_);
    mapmatcher_.reset(matcher_factory_->Create(valhalla::Costing::auto_, options_));
  }

  void InitMeasurements() {
    float gps_accuracy = mapmatcher_->config().get<float>("gps_accuracy");
    float search_radius = mapmatcher_->config().get<float>("search_radius");
    measurements_.emplace_back(PointLL(5.09806, 52.09110), gps_accuracy, search_radius);
    measurements_.emplace_back(PointLL(5.09769, 52.09050), gps_accuracy, search_radius);
    measurements_.emplace_back(PointLL(5.09679, 52.09098), gps_accuracy, search_radius);
  }

protected:
  // Mapmatcher implementation
  std::shared_ptr<MapMatcher> mapmatcher_;
  std::shared_ptr<MapMatcherFactory> matcher_factory_;
  // Inputs
  std::vector<Measurement> measurements_;
  // Mapmatching configuration
  boost::property_tree::ptree config_;
  valhalla::Options options_;
};

BENCHMARK_DEFINE_F(MapmatchFixture, BasicOfflineMatch)(benchmark::State& state) {
  for (auto _ : state) {
    mapmatcher_->OfflineMatch(measurements_);
  }
}

BENCHMARK_REGISTER_F(MapmatchFixture, BasicOfflineMatch);

} // namespace

BENCHMARK_MAIN();
