#include "test.h"

#include "baldr/graphreader.h"
#include "loki/worker.h"
#include "thor/worker.h"
#include "worker.h"

using namespace testing;
using namespace valhalla;
using namespace std::string_literals;

using Seed_t = std::mt19937_64::result_type;

namespace {

constexpr size_t AttemptsCount = 1000;
const auto Config = test::make_config("test/data/whitelion_tiles");
constexpr auto TestRequest =
    R"({"locations":[{"lat":51.455768530466514,"lon":-2.5954368710517883},{"lat":51.456082740244824,"lon":-2.595050632953644}],"costing":"auto"})";

class RandomGraphReader : public baldr::GraphReader {
public:
  explicit RandomGraphReader(std::shared_ptr<baldr::GraphReader> wrapped, std::mt19937_64& randEngine)
      : baldr::GraphReader({}, {}), wrapped_(std::move(wrapped)), randEngine_(randEngine) {
  }

  void setProba(double proba) {
    if (proba < 0.0 || proba > 1.0) {
      throw std::invalid_argument("proba should be from 0 till 1");
    }
    proba_ = proba;
  }

  bool DoesTileExist(const valhalla::baldr::GraphId&) const override {
    return true;
  }

  graph_tile_ptr GetGraphTile(const valhalla::baldr::GraphId& graphId) override {
    auto rnd = distribution_(randEngine_);
    if (rnd < proba_) {
      return {};
    }
    return wrapped_->GetGraphTile(graphId);
  }

private:
  std::shared_ptr<valhalla::baldr::GraphReader> wrapped_;
  std::uniform_real_distribution<double> distribution_{0, 1};
  std::mt19937_64& randEngine_;
  double proba_ = 0.5;
};

} // namespace

using GraphReaderPtr_t = std::shared_ptr<baldr::GraphReader>;
using RandomGraphReaderPtr_t = std::shared_ptr<RandomGraphReader>;

class WorkerNullptrTiles : public Test {
protected:
  static std::mt19937_64 randEngine_;

  static void SetUpTestSuite() {
    randEngine_.seed(std::time(nullptr));
    std::cout << "Random engine=" << randEngine_ << std::endl;
  }

  static GraphReaderPtr_t createGraphReader() {
    return std::make_shared<GraphReader>(Config.get_child("mjolnir"));
  }

  static RandomGraphReaderPtr_t createRandomGraphReader(GraphReaderPtr_t original_reader) {
    auto random_reader = std::make_shared<RandomGraphReader>(std::move(original_reader), randEngine_);

    return random_reader;
  }
};

/*static*/
std::mt19937_64 WorkerNullptrTiles::randEngine_;

TEST_F(WorkerNullptrTiles, loki_worker_null_test) {
  auto random_reader = createRandomGraphReader(createGraphReader());

  loki::loki_worker_t loki_worker(Config, random_reader);

  for (size_t attempt = 0; attempt < AttemptsCount; ++attempt) {
    Api request;
    ParseApi(TestRequest, Options::route, request);
    try {
      ASSERT_NO_FATAL_FAILURE(loki_worker.route(request));
    } catch (const std::exception& e) {
      // Ignore exceptions
    }
    loki_worker.cleanup();
  }
}

TEST_F(WorkerNullptrTiles, thor_worker_null_test) {
  auto original_reader = createGraphReader();
  auto random_reader = createRandomGraphReader(original_reader);

  loki::loki_worker_t loki_worker(Config, original_reader);
  thor::thor_worker_t thor_worker(Config, random_reader);

  for (size_t attempt = 0; attempt < AttemptsCount; ++attempt) {
    Api request;
    ParseApi(TestRequest, Options::route, request);
    ASSERT_NO_THROW(loki_worker.route(request));
    try {
      ASSERT_NO_FATAL_FAILURE(thor_worker.route(request));
    } catch (const std::exception& e) {
      // Ignore exceptions
    }
    loki_worker.cleanup();
    thor_worker.cleanup();
  }
}

int main(int argc, char* argv[]) {
  logging::Configure({{"type", ""}});

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
