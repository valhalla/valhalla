#include <array>
#include <benchmark/benchmark.h>
#include <iostream>
#include <random>
#include <string>

#include "baldr/graphreader.h"
#include "loki/reach.h"
#include "sif/costfactory.h"
#include "test.h"

using namespace valhalla;

namespace {

// Test the core isochrone calculation algorithm
void BM_ReachUtrecht(benchmark::State& state) {

  const auto config =
      test::make_config("test/data/utrecht_tiles", {},
                        {{"additional_data", "mjolnir.traffic_extract", "mjolnir.tile_extract"}});

  // get tile access
  GraphReader reader(config.get_child("mjolnir"));

  auto costing = sif::CostFactory{}.Create(Costing::auto_);
  loki::Reach reach_finder;

  using Edge = std::pair<GraphId, const DirectedEdge*>;
  std::vector<Edge> edges;

  for (auto tile_id : reader.GetTileSet()) {
    auto tile = reader.GetGraphTile(tile_id);
    for (GraphId edge_id = tile->header()->graphid();
         edge_id.id() < tile->header()->directededgecount(); ++edge_id) {
      const auto* edge = tile->directededge(edge_id);
      edges.emplace_back(edge_id, edge);
    }
  }

  for (auto _ : state) {
    for (const auto& edge : edges) {
      auto reach = reach_finder(edge.second, edge.first, 50, reader, costing, kInbound | kOutbound);
    }
  }
}

BENCHMARK(BM_ReachUtrecht)->Unit(benchmark::kMillisecond)->Repetitions(10);

} // namespace

BENCHMARK_MAIN();
