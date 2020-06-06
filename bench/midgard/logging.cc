#include <benchmark/benchmark.h>
#include <glog/logging.h>

#include "midgard/logging.h"

namespace {

static void BM_BasicInfoLoggingMidgard(benchmark::State& state) {
  // valhalla::midgard::logging::Configure({{"type", ""}});
  valhalla::midgard::logging::Configure({{"type", "file"}, {"file_name", "/tmp/benchmarking.log"}});
  for (auto _ : state) {
    LOG_INFO("HELLO!");
  }
}

BENCHMARK(BM_BasicInfoLoggingMidgard);
BENCHMARK(BM_BasicInfoLoggingMidgard)->Threads(1);
BENCHMARK(BM_BasicInfoLoggingMidgard)->Threads(2);
BENCHMARK(BM_BasicInfoLoggingMidgard)->Threads(4);
BENCHMARK(BM_BasicInfoLoggingMidgard)->Threads(8);
BENCHMARK(BM_BasicInfoLoggingMidgard)->Threads(16);

static void BM_BasicInfoLoggingMidgard10k(benchmark::State& state) {
  // valhalla::midgard::logging::Configure({{"type", ""}});
  valhalla::midgard::logging::Configure({{"type", "file"}, {"file_name", "/tmp/benchmarking.log"}});
  for (auto _ : state) {
    size_t n = 10000;
    while (n--) {
      LOG_INFO("HELLO!");
    }
  }
}

BENCHMARK(BM_BasicInfoLoggingMidgard10k);
BENCHMARK(BM_BasicInfoLoggingMidgard10k)->Threads(1);
BENCHMARK(BM_BasicInfoLoggingMidgard10k)->Threads(2);
BENCHMARK(BM_BasicInfoLoggingMidgard10k)->Threads(4);
BENCHMARK(BM_BasicInfoLoggingMidgard10k)->Threads(8);
BENCHMARK(BM_BasicInfoLoggingMidgard10k)->Threads(16);

static void BM_BasicInfoLogging(benchmark::State& state) {
  for (auto _ : state) {
    LOG(INFO) << "Hello!";
  }
}

BENCHMARK(BM_BasicInfoLogging);
BENCHMARK(BM_BasicInfoLogging)->Threads(1);
BENCHMARK(BM_BasicInfoLogging)->Threads(2);
BENCHMARK(BM_BasicInfoLogging)->Threads(4);
BENCHMARK(BM_BasicInfoLogging)->Threads(8);
BENCHMARK(BM_BasicInfoLogging)->Threads(16);

static void BM_BasicInfoLogging10k(benchmark::State& state) {
  for (auto _ : state) {
    size_t n = 10000;
    while (n--) {
      LOG(INFO) << "Hello!";
    }
  }
}

BENCHMARK(BM_BasicInfoLogging10k);
BENCHMARK(BM_BasicInfoLogging10k)->Threads(1);
BENCHMARK(BM_BasicInfoLogging10k)->Threads(2);
BENCHMARK(BM_BasicInfoLogging10k)->Threads(4);
BENCHMARK(BM_BasicInfoLogging10k)->Threads(8);
BENCHMARK(BM_BasicInfoLogging10k)->Threads(16);

} // namespace

int main(int argc, char** argv) {
  benchmark::Initialize(&argc, argv);
  FLAGS_logtostderr = false;
  google::InitGoogleLogging("logging-test");
  if (benchmark::ReportUnrecognizedArguments(argc, argv))
    return 1;
  benchmark::RunSpecifiedBenchmarks();
}
