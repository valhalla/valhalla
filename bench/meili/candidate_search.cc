#include <memory>

#include <benchmark/benchmark.h>
#include <boost/property_tree/ptree.hpp>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "meili/candidate_search.h"
#include "midgard/logging.h"
#include "sif/costfactory.h"
#include "sif/dynamiccost.h"
#include "worker.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::meili;
using namespace valhalla::sif;

namespace {

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

float GetLocalTileSize() {
  return (TileHierarchy::levels().rbegin()->second.tiles).TileSize();
}

cost_ptr_t MakeCosting(const std::string& cost_mode) {
  Options options;
  for (int i = 0; i < Costing_MAX; ++i) {
    options.add_costing_options();
  }
  Costing costing;
  Costing_Enum_Parse(cost_mode, &costing);
  options.set_costing(costing);
  CostFactory<DynamicCost> factory;
  factory.RegisterStandardCostingModels();
  return factory.Create(options);
}

class CandidateSearchFixture : public benchmark::Fixture {
public:
  void SetUp(const benchmark::State& state) {
    (void)state;
    InitConfig();
    InitCandidateSearch();
  }

  cost_ptr_t costing() const {
    return (&mode_costing_)[static_cast<size_t>(travelmode_)];
  }

private:
  void InitConfig() {
    rapidjson::read_json(VALHALLA_SOURCE_DIR "bench/meili/config.json", config_);
    const boost::property_tree::ptree& conf = config_.get_child("meili");
    mode_costing_ = MakeCosting(conf.get<std::string>("mode"));
    travelmode_ = mode_costing_->travel_mode();
  }

  void InitCandidateSearch() {
    const boost::property_tree::ptree& conf = config_.get_child("meili");
    const float local_tile_size = GetLocalTileSize();
    const float cell_width = local_tile_size / conf.get<size_t>("grid.size");
    const float cell_height = local_tile_size / conf.get<size_t>("grid.size");
    graph_reader_ = std::make_shared<GraphReader>(config_.get_child("mjolnir"));
    index_ = std::make_shared<CandidateGridQuery>(*graph_reader_, cell_width, cell_height);
  }

protected:
  boost::property_tree::ptree config_;
  cost_ptr_t mode_costing_;
  TravelMode travelmode_;

  std::shared_ptr<GraphReader> graph_reader_;
  std::shared_ptr<CandidateGridQuery> index_;
};

BENCHMARK_DEFINE_F(CandidateSearchFixture, BM_CandidateSearch)(benchmark::State& state) {
  logging::Configure({{"type", ""}});
  // Location @ Kanaalstraat 179-193, 3531 CG Utrecht, Netherlands
  PointLL location(5.09806, 52.09110);
  EdgeFilter edgefilter = costing()->GetEdgeFilter();
  const size_t radius_sq_m2 = std::pow(state.range(0), 2);
  for (auto _ : state) {
    benchmark::DoNotOptimize(index_->Query(location, radius_sq_m2, edgefilter));
  }
}

// Benchmark candidate search from 5 to 500 meters
BENCHMARK_REGISTER_F(CandidateSearchFixture, BM_CandidateSearch)->RangeMultiplier(2)->Range(5, 500);

} // namespace

BENCHMARK_MAIN();
