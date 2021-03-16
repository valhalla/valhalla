#include <iostream>
#include <memory>
#include <sstream>
#include <unordered_map>
#include <utility>

#include <benchmark/benchmark.h>
#include <boost/property_tree/ptree.hpp>
#include <valhalla/proto/options.pb.h>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "loki/search.h"
#include "midgard/logging.h"
#include "sif/costconstants.h"
#include "sif/costfactory.h"
#include "thor/astar_bss.h"
#include "thor/bidirectional_astar.h"
#include "thor/pathalgorithm.h"

#include "bench/common/utils.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla::midgard;

namespace {

using valhalla::baldr::GraphReader;
using valhalla::baldr::Location;
using valhalla::baldr::PathLocation;
using valhalla::midgard::PointLL;
using valhalla::thor::AStarBSSAlgorithm;
using valhalla::thor::BidirectionalAStar;

using valhalla::bench::MakeCosting;

void create_costing_options(valhalla::Options& options) {
  options.set_costing(valhalla::Costing::auto_);
  rapidjson::Document doc;
  valhalla::sif::ParseCostingOptions(doc, "/costing_options", options);
}

class RoutingFixture : public benchmark::Fixture {
public:
  void SetUp(const benchmark::State& state) {
    (void)state;
    InitConfig();
    InitGraph();
  }

  // Produce a GraphReader-correlated pair (source, destination) from points.
  std::pair<valhalla::Location, valhalla::Location> PointToLocation(PointLL src, PointLL dst) const {
    const auto& stop_type = Location::StopType::BREAK;
    std::vector<Location> locations = {Location(src, stop_type), Location(dst, stop_type)};
    const auto& cost = mode_costing_[static_cast<size_t>(travel_mode_)];
    const std::unordered_map<Location, PathLocation>& projections =
        valhalla::loki::Search(locations, *graph_reader_, cost);
    valhalla::Location origin;
    PathLocation::toPBF(projections.at(locations[0]), &origin, *graph_reader_);
    valhalla::Location dest;
    PathLocation::toPBF(projections.at(locations[1]), &dest, *graph_reader_);
    return std::make_pair(origin, dest);
  }

private:
  void InitConfig() {
    rapidjson::read_json(VALHALLA_SOURCE_DIR "bench/thor/config.json", config_);
    valhalla::Options options;
    create_costing_options(options);
    mode_costing_ = valhalla::sif::CostFactory().CreateModeCosting(options, travel_mode_);
  }

  void InitGraph() {
    graph_reader_ = std::make_shared<GraphReader>(config_.get_child("mjolnir"));
  }

protected:
  boost::property_tree::ptree config_;
  valhalla::sif::mode_costing_t mode_costing_;
  std::shared_ptr<GraphReader> graph_reader_;
  valhalla::sif::TravelMode travel_mode_;
};

// clang-format off
const std::vector<std::pair<PointLL, PointLL>> kBenchmarkCases = {
    // ./fixtures/utrecht100m.json
    {PointLL(5.08531221, 52.0938563), PointLL(5.0865867, 52.0930211)},
    // ./fixtures/utrecht200m.json
    {PointLL(5.08531221, 52.0938563), PointLL(5.08769141, 52.0923946)},
    // ./fixtures/utrecht2km.json
    {PointLL(5.08531221, 52.0938563), PointLL(5.10351751, 52.09202915)},
    // ./fixtures/utrecht4km.json
    {PointLL(5.08531221, 52.0938563), PointLL(5.1117335, 52.0851706)}};
// clang-format on

BENCHMARK_DEFINE_F(RoutingFixture, BidirectionalAStar)(benchmark::State& state) {
  logging::Configure({{"type", ""}});
  std::pair<PointLL, PointLL> bench_case = kBenchmarkCases[state.range(0)];
  std::pair<valhalla::Location, valhalla::Location> locations =
      PointToLocation(bench_case.first, bench_case.second);
  BidirectionalAStar astar;
  for (auto _ : state) {
    benchmark::DoNotOptimize(astar.GetBestPath(locations.first, locations.second, *graph_reader_,
                                               mode_costing_, travel_mode_));
  }
}

BENCHMARK_REGISTER_F(RoutingFixture, BidirectionalAStar)->DenseRange(0, kBenchmarkCases.size() - 1);

} // namespace

BENCHMARK_MAIN();
