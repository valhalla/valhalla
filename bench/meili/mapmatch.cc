#include <iostream>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "benchmark/benchmark.h"

#include "meili/map_matcher_factory.h"
#include "meili/measurement.h"

using namespace valhalla::midgard;
using namespace valhalla::meili;

namespace {

boost::property_tree::ptree json_to_pt(const std::string& json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  rapidjson::read_json(ss, pt);
  return pt;
}

const boost::property_tree::ptree kConfig = json_to_pt(R"({
    "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1},
    "loki":{
      "actions":["locate","route","sources_to_targets","optimized_route","isochrone","trace_route","trace_attributes"],
      "logging":{"long_request": 100},
      "service_defaults":{"minimum_reachability": 50,"radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "heading_tolerance": 60}
    },
    "thor":{"logging":{"long_request": 110}},
    "skadi":{"actons":["height"],"logging":{"long_request": 5}},
    "meili":{"customizable": ["turn_penalty_factor","max_route_distance_factor","max_route_time_factor","search_radius"],
             "mode":"auto","grid":{"cache_size":100240,"size":500},
             "default":{"beta":3,"breakage_distance":2000,"geometry":false,"gps_accuracy":5.0,"interpolation_distance":10,
             "max_route_distance_factor":5,"max_route_time_factor":5,"max_search_radius":200,"route":true,
             "search_radius":15.0,"sigma_z":4.07,"turn_penalty_factor":200}},
    "service_limits": {
      "auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "taxi": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time": 120},
      "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
      "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
      "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
      "skadi": {"max_shape": 750000,"min_resample": 10.0},
      "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
      "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
    }
  })");

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
    const std::string modename = kConfig.get<std::string>("meili.mode");
    valhalla::Costing costing;
    if (!valhalla::Costing_Enum_Parse(modename, &costing)) {
      throw std::runtime_error("No costing method found");
    }
    matcher_factory_ = std::make_shared<MapMatcherFactory>(kConfig);
    mapmatcher_.reset(matcher_factory_->Create(costing));
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
