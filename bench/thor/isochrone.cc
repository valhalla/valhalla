#include <array>
#include <benchmark/benchmark.h>
#include <iostream>
#include <random>
#include <string>

#include "baldr/graphreader.h"
//#include <valhalla/proto/options.pb.h>

#include "loki/worker.h"
#include "thor/worker.h"

#include "test.h"

using namespace valhalla;

namespace {

// Maximum isochrone range to test during the benchmark
// This benchmark is on the Utrecht tiles we test against, so
// 30 minutes pretty much gets you to the edge from the middle
// in any direction
constexpr float kMaxDurationMinutes = 120;

// Test the core isochrone calculation algorithm
void BM_IsochroneUtrecht(benchmark::State& state) {
  const int size = state.range(0);

  const auto config =
      test::make_config("test/data/utrecht_tiles", {},
                        {{"additional_data", "mjolnir.traffic_extract", "mjolnir.tile_extract"}});
  valhalla::loki::loki_worker_t loki_worker(config);
  valhalla::thor::thor_worker_t thor_worker(config);

  const auto request_json =
      R"({"locations":[{"lat":52.078937,"lon":5.115321}],"costing":"auto","contours":[{"time":)" +
      std::to_string(size) + R"(}],"polygons":false,"denoise":1,"generalize":20})";

  // compute the isochrone
  valhalla::Api request;
  valhalla::ParseApi(request_json, Options::isochrone, request);
  loki_worker.isochrones(request);

  for (auto _ : state) {
    auto response_json = thor_worker.isochrones(request);
    // std::cout << response_json << std::endl;
  }
}

BENCHMARK(BM_IsochroneUtrecht)
    ->Unit(benchmark::kMillisecond)
    ->RangeMultiplier(2)
    ->Range(1, kMaxDurationMinutes)
    ->Repetitions(10);

} // namespace

BENCHMARK_MAIN();
