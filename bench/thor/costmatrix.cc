#include <array>
#include <benchmark/benchmark.h>
#include <iostream>
#include <random>
#include <string>

#include "baldr/graphreader.h"
#include "loki/search.h"
#include "midgard/pointll.h"
#include "sif/autocost.h"
#include "sif/costfactory.h"
#include "thor/costmatrix.h"
#include <valhalla/proto/options.pb.h>

using namespace valhalla;

namespace {

void create_costing_options(Options& options) {
  const rapidjson::Document doc;
  sif::ParseAutoCostOptions(doc, "/costing_options/auto", options.add_costing_options());
  options.add_costing_options();
}

boost::property_tree::ptree json_to_pt(const std::string& json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  rapidjson::read_json(ss, pt);
  return pt;
}

const auto config = json_to_pt(R"({
    "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1},
    "loki":{
      "actions":["sources_to_targets"],
      "logging":{"long_request": 100},
      "service_defaults":{"minimum_reachability": 50,"radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "street_side_max_distance": 1000, "heading_tolerance": 60}
    },
    "thor":{
      "logging":{"long_request": 100}
    },
    "meili":{
      "grid": {
        "cache_size": 100240,
        "size": 500
      }
    },
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

constexpr float kMaxRange = 256;

static void BM_UtrechtCostMatrix(benchmark::State& state) {
  const int size = state.range(0);
  baldr::GraphReader reader(config.get_child("mjolnir"));

  // Generate N random locations within the Utrect bounding box;
  std::vector<valhalla::baldr::Location> locations;
  const double min_lon = 5.0163;
  const double max_lon = 5.1622;
  const double min_lat = 52.0469999;
  const double max_lat = 52.1411;

  std::mt19937 gen(0); // Seed with the same value for consistent benchmarking
  std::uniform_real_distribution<> lng_distribution(min_lon, max_lon);
  std::uniform_real_distribution<> lat_distribution(min_lat, max_lat);

  locations.reserve(size);
  for (int i = 0; i < size; i++) {
    locations.emplace_back(midgard::PointLL{lng_distribution(gen), lat_distribution(gen)});
  }

  Options options;
  create_costing_options(options);
  auto costs = sif::CreateAutoCost(Costing::auto_, options);

  const auto projections = loki::Search(locations, reader, costs);
  if (projections.size() == 0) {
    throw std::runtime_error("Found no matching locations");
  }

  google::protobuf::RepeatedPtrField<valhalla::Location> sources;

  for (const auto& projection : projections) {
    auto* p = sources.Add();
    baldr::PathLocation::toPBF(projection.second, p, reader);
  }

  std::size_t result_size = 0;

  for (auto _ : state) {
    thor::CostMatrix matrix;
    auto result =
        matrix.SourceToTarget(sources, sources, reader, &costs, sif::TravelMode::kDrive, 100000.);
    result_size += result.size();
  }
  state.counters["Routes"] = benchmark::Counter(size, benchmark::Counter::kIsIterationInvariantRate);
}

BENCHMARK(BM_UtrechtCostMatrix)
    ->Unit(benchmark::kMillisecond)
    ->RangeMultiplier(2)
    ->Range(1, kMaxRange);

} // namespace

BENCHMARK_MAIN();
