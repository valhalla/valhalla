#include <iostream>
#include <sstream>

#include <benchmark/benchmark.h>
#include <boost/property_tree/ptree.hpp>

#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "tyr/actor.h"

#include "bench/common/utils.h"

namespace {

using valhalla::bench::LoadFile;

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

const std::vector<std::string> kBenchmarkCases = {
    VALHALLA_SOURCE_DIR "bench/thor/fixtures/utrecht100m.json",
    VALHALLA_SOURCE_DIR "bench/thor/fixtures/utrecht200m.json",
    VALHALLA_SOURCE_DIR "bench/thor/fixtures/utrecht2km.json",
    VALHALLA_SOURCE_DIR "bench/thor/fixtures/utrecht4km.json",
};

static void BM_RoutingFixtures(benchmark::State& state) {
  valhalla::midgard::logging::Configure({{"type", ""}});
  boost::property_tree::ptree config;
  rapidjson::read_json(VALHALLA_SOURCE_DIR "bench/thor/config.json", config);
  valhalla::tyr::actor_t actor(config, true);
  const std::string test_case(LoadFile(kBenchmarkCases[state.range(0)]));
  for (auto _ : state) {
    benchmark::DoNotOptimize(actor.route(test_case));
  }
}

BENCHMARK(BM_RoutingFixtures)->DenseRange(0, kBenchmarkCases.size() - 1);

} // namespace

BENCHMARK_MAIN();
