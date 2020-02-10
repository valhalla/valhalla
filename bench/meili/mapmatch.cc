#include <iostream>
#include <vector>

#include <benchmark/benchmark.h>
#include <boost/property_tree/ptree.hpp>
#include <rapidjson/rapidjson.h>

#include "meili/map_matcher_factory.h"
#include "meili/measurement.h"

using namespace valhalla::midgard;
using namespace valhalla::meili;

namespace {

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

class MapmatchFixture : public benchmark::Fixture {
public:
  void SetUp(const ::benchmark::State& state) {
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
  void InitMapMatcher() {
    boost::property_tree::ptree config;
    rapidjson::read_json(VALHALLA_SOURCE_DIR "test/valhalla.json", config);
    matcher_factory_ = std::make_shared<MapMatcherFactory>(config);
    mapmatcher_.reset(matcher_factory_->Create(valhalla::Costing::auto_));
  }

  void InitMeasurements() {
    float gps_accuracy = mapmatcher_->config().get<float>("gps_accuracy");
    float search_radius = mapmatcher_->config().get<float>("search_radius");
    measurements_.emplace_back(PointLL(5.09806, 52.09110), gps_accuracy, search_radius);
    measurements_.emplace_back(PointLL(5.09769, 52.09050), gps_accuracy, search_radius);
    measurements_.emplace_back(PointLL(5.09679, 52.09098), gps_accuracy, search_radius);
  }

protected:
  std::shared_ptr<MapMatcher> mapmatcher_;
  std::shared_ptr<MapMatcherFactory> matcher_factory_;
  std::vector<Measurement> measurements_;
};

BENCHMARK_DEFINE_F(MapmatchFixture, MapMatching)(benchmark::State& state) {
  for (auto _ : state) {
    mapmatcher_->OfflineMatch(measurements_);
  }
}

BENCHMARK_REGISTER_F(MapmatchFixture, MapMatching);

} // namespace

BENCHMARK_MAIN();
