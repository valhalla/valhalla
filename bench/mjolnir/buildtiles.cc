#include <benchmark/benchmark.h>

#include "baldr/rapidjson_utils.h"
#include "mjolnir/util.h"

using namespace valhalla::mjolnir;

int main(int argc, char** argv) {
  valhalla::midgard::logging::Configure({{"type", ""}});

  const auto build_tiles = [](benchmark::State& state, const std::string& pbf_path,
                              BuildStage stage) {
    const std::string kConfigFilePath = VALHALLA_SOURCE_DIR "bench/mjolnir/config.json";
    boost::property_tree::ptree config;
    rapidjson::read_json(kConfigFilePath, config);
    config.get_child("mjolnir").erase("tile_extract");
    config.get_child("mjolnir").erase("tile_url");

    for (auto _ : state) {
      state.PauseTiming();
      build_tile_set(config, {pbf_path}, BuildStage::kInitialize,
                     static_cast<BuildStage>(static_cast<uint8_t>(stage) - 1));
      state.ResumeTiming();
      benchmark::DoNotOptimize(build_tile_set(config, {pbf_path}, stage, stage));
    }
  };

  const std::string kProtobufs[] = {
      "andorra.osm.pbf", "london.osm.pbf", "monaco.osm.pbf",    "moscow.osm.pbf",
      "munchen.osm.pbf", "porto.osm.pbf",  "stockholm.osm.pbf", "sydney.osm.pbf",
  };
  const BuildStage kStages[] = {BuildStage::kParseWays, BuildStage::kParseRelations,
                                BuildStage::kParseNodes, BuildStage::kBuild, BuildStage::kEnhance};
  for (const BuildStage stage : kStages) {
    for (const std::string& filename : kProtobufs) {
      std::string bench_name = filename.substr(0, filename.size() - 8) + "/" + to_string(stage);
      benchmark::RegisterBenchmark(bench_name.c_str(), build_tiles,
                                   VALHALLA_SOURCE_DIR "bench/mjolnir/pbf/" + filename, stage)
          ->Unit(benchmark::kMillisecond);
    }
  }
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
