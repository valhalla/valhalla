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
#include "test.h"
#include "thor/costmatrix.h"
#include <valhalla/config.h>
#include <valhalla/proto/options.pb.h>

using namespace valhalla;

namespace {
constexpr float kMaxRange = 256;

static void BM_UtrechtCostMatrix(benchmark::State& state) {
  const int size = state.range(0);
  const auto config =
      test::make_config("test/data/utrecht_tiles", {},
                        {"additional_data", "mjolnir.traffic_extract", "mjolnir.tile_extract"});
  logging::Configure({{"type", ""}});
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

  Api request;
  auto& options = *request.mutable_options();
  options.set_costing_type(Costing::auto_);
  rapidjson::Document doc;
  sif::ParseCosting(doc, "/costing_options", options);
  sif::TravelMode mode;
  auto costs = sif::CostFactory().CreateModeCosting(options, mode);
  auto cost = costs[static_cast<size_t>(mode)];

  auto& sources = *options.mutable_sources();
  const auto projections = loki::Search(locations, reader, cost);
  if (projections.size() == 0) {
    throw std::runtime_error("Found no matching locations");
  }
  for (const auto& projection : projections) {
    auto* p = sources.Add();
    baldr::PathLocation::toPBF(projection.second, p, reader);
  }

  thor::CostMatrix matrix;
  for (auto _ : state) {
    matrix.SourceToTarget(request, reader, costs, mode, 100000.);
    matrix.clear();
    request.clear_matrix();
  }
  state.counters["Routes"] = benchmark::Counter(size, benchmark::Counter::kIsIterationInvariantRate);
}

BENCHMARK(BM_UtrechtCostMatrix)
    ->Unit(benchmark::kMillisecond)
    ->RangeMultiplier(2)
    ->Range(1, kMaxRange);

} // namespace

BENCHMARK_MAIN();
