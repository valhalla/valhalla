#include <benchmark/benchmark.h>

#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include "mjolnir/util.h"

using namespace valhalla::mjolnir;

namespace {

void BM_SomeFunction(benchmark::State& state, const std::string& pbf_path) {
  valhalla::midgard::logging::Configure({{"type", ""}});

  std::string const kConfigFilePath = VALHALLA_SOURCE_DIR "bench/mjolnir/config.json";
  boost::property_tree::ptree config;
  rapidjson::read_json(kConfigFilePath, config);
  config.get_child("mjolnir").erase("tile_extract");
  config.get_child("mjolnir").erase("tile_url");

  BuildStage const start_stage_str = string_to_buildstage("initialize");
  BuildStage const end_stage_str = string_to_buildstage("parseways");

  for (auto _ : state) {
    build_tile_set(config, {pbf_path}, start_stage_str, end_stage_str);
  }
};

struct FileWithSize {
  filesystem::path path;
  uintmax_t size;

  bool operator<(const FileWithSize& file) const {
    return size < file.size;
  }
};

std::vector<FileWithSize> CollectInputProtobufs(const std::string& path) {
  std::vector<FileWithSize> files;
  for (filesystem::recursive_directory_iterator i(path), end; i != end; ++i)
    files.push_back({i->path(), i->file_size()});
  std::sort(files.begin(), files.end());
  return files;
}

} // namespace

int main(int argc, char** argv) {
  auto protobufs = CollectInputProtobufs(VALHALLA_SOURCE_DIR "bench/mjolnir/pbf");
  for (const FileWithSize& file : protobufs) {
    benchmark::RegisterBenchmark(file.path.filename().c_str(), BM_SomeFunction, file.path.string())
        ->Unit(benchmark::kMillisecond);
  }
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
