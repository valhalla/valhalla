#include <benchmark/benchmark.h>

#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include "mjolnir/util.h"

using namespace valhalla::mjolnir;

namespace {

const std::string kConfigFilePath = "bench/mjolnir/config.json";

boost::property_tree::ptree GetConfig(const std::string& tile_dir = "bench/mjolnir/tiles") {
  boost::property_tree::ptree config;
  rapidjson::read_json(kConfigFilePath, config);
  config.get_child("mjolnir").erase("tile_extract");
  config.get_child("mjolnir").erase("tile_url");
  config.put("mjolnir.tile_dir", tile_dir);
  return config;
}

void CopyFile(std::string const& src, const std::string& dst) {
  std::ifstream in(src, std::ios::binary);
  std::ofstream out(dst, std::ios::binary);
  out << in.rdbuf();
}

void CopyDir(const std::string& src, const std::string& dst) {
  if (!filesystem::exists(src) || !filesystem::is_directory(src))
    throw std::runtime_error("Invalid source directory: " + src);

  if (filesystem::exists(dst))
    filesystem::remove_all(dst);
  filesystem::create_directories(dst);

  for (filesystem::recursive_directory_iterator i(src), end; i != end; ++i) {
    if (i->is_regular_file() || i->is_symlink()) {
      std::string dst_file_path = i->path().string();
      auto it = dst_file_path.find(src);
      dst_file_path.erase(it, src.size());
      dst_file_path.insert(it, dst);
      CopyFile(i->path().string(), dst_file_path);
    }
  }
}

void RegisterByStageBenchmark() {
  const auto build_tiles = [](benchmark::State& state, const std::string& pbf_path,
                              BuildStage stage) {
    const std::string kStageDir = "bench/mjolnir/stage_files";
    const std::string kTileDir = "bench/mjolnir/tiles";
    const boost::property_tree::ptree stage_config = GetConfig(kStageDir);
    // prepare files for the stage once and copy them every time before running the stage
    build_tile_set(stage_config, {pbf_path}, BuildStage::kInitialize,
                   static_cast<BuildStage>(static_cast<uint8_t>(stage) - 1), false);

    for (auto _ : state) {
      state.PauseTiming();
      // copy the files needed for the stage
      CopyDir(kStageDir, kTileDir);
      const boost::property_tree::ptree config = GetConfig(kTileDir);
      state.ResumeTiming();

      benchmark::DoNotOptimize(build_tile_set(config, {pbf_path}, stage, stage, false));
    }
  };

  const std::string kProtobufs[] = {"stockholm.osm.pbf"};
  const BuildStage kStages[] = {BuildStage::kParseWays, BuildStage::kParseRelations,
                                BuildStage::kParseNodes, BuildStage::kBuild, BuildStage::kEnhance};
  for (const BuildStage stage : kStages) {
    for (const std::string& filename : kProtobufs) {
      std::string bench_name = filename.substr(0, filename.size() - 8) + "/" + to_string(stage);
      benchmark::RegisterBenchmark(bench_name.c_str(), build_tiles,
                                   VALHALLA_SOURCE_DIR "bench/mjolnir/pbf/" + filename, stage)
          ->Unit(benchmark::kMillisecond)
          ->Iterations(50);
    }
  }
}

void EndToEndTileBuild(benchmark::State& state) {
  const boost::property_tree::ptree config = GetConfig();
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        build_tile_set(config, {VALHALLA_SOURCE_DIR "bench/mjolnir/pbf/stockholm.osm.pbf"},
                       BuildStage::kInitialize, BuildStage::kCleanup, false));
  }
}

} // namespace

BENCHMARK(EndToEndTileBuild)->Unit(benchmark::kMillisecond)->Iterations(5);

int main(int argc, char** argv) {
  valhalla::midgard::logging::Configure({{"type", ""}});
  RegisterByStageBenchmark();
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
