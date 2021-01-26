#include <array>
#include <benchmark/benchmark.h>
#include <iostream>
#include <random>
#include <string>

#include "baldr/graphreader.h"
#include "loki/search.h"
#include "midgard/pointll.h"
#include "sif/autocost.h"
#include "sif/costfactory.h"
#include "test.h"
#include "thor/bidirectional_astar.h"
#include <valhalla/proto/options.pb.h>

using namespace valhalla;

namespace {

void create_costing_options(Options& options) {
  options.set_costing(Costing::auto_);
  rapidjson::Document doc;
  sif::ParseCostingOptions(doc, "/costing_options", options);
}

boost::property_tree::ptree json_to_pt(const std::string& json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  rapidjson::read_json(ss, pt);
  return pt;
}

boost::property_tree::ptree build_config(const char* live_traffic_tar) {
  return json_to_pt(R"({
    "mjolnir":{
      "traffic_extract": "test/data/utrecht_tiles/)" +
                    std::string(live_traffic_tar) + R"(",
      "tile_dir": "test/data/utrecht_tiles",
      "concurrency": 1
    },
    "loki":{
        "actions":["route"],
        "logging":{"long_request": 100},
        "service_defaults":{
          "minimum_reachability": 10,
          "radius": 50,
          "search_cutoff": 35000,
          "node_snap_tolerance": 5,
          "street_side_tolerance": 5,
          "heading_tolerance": 360
        }
    },
    "thor":{
      "logging":{"long_request": 100}
    },
    "meili":{
      "grid": {
        "cache_size": 100240,
        "size": 500
      }
    },
    "service_limits": {
      "auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "taxi": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time_contour": 120,"max_distance_contour":200},
      "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
      "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
      "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
      "skadi": {"max_shape": 750000,"min_resample": 10.0},
      "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
      "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
    }
  })");
}

constexpr float kMaxRange = 256;

static void BM_UtrechtBidirectionalAstar(benchmark::State& state) {
  const auto config = build_config("generated-live-data.tar");
  test::build_live_traffic_data(config);

  std::mt19937 gen(0); // Seed with the same value for consistent benchmarking
  {
    // Something to generate traffic with
    std::uniform_real_distribution<> traffic_dist(0., 1.);
    // This fraction of edges have live traffic
    float has_live_traffic = 0.2;

    // Make some updates to the traffic .tar file.
    // Generate traffic data
    std::function<void(baldr::GraphReader&, baldr::TrafficTile&, int, baldr::TrafficSpeed*)>
        generate_traffic = [&gen, &traffic_dist,
                            &has_live_traffic](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                               int index, baldr::TrafficSpeed* current) -> void {
      baldr::GraphId tile_id(tile.header->tile_id);
      if (traffic_dist(gen) < has_live_traffic) {
        current->breakpoint1 = 255;
        current->overall_speed = traffic_dist(gen) * 100;
      } else {
      }
    };
    test::customize_live_traffic_data(config, generate_traffic);
  }

  auto clean_reader = test::make_clean_graphreader(config.get_child("mjolnir"));

  // Generate N random route queries within the Utrect bounding box;
  const int N = 2;

  std::vector<valhalla::baldr::Location> locations;
  const double min_lon = 5.0163;
  const double max_lon = 5.1622;
  const double min_lat = 52.0469999;
  const double max_lat = 52.1411;

  std::uniform_real_distribution<> lng_distribution(min_lon, max_lon);
  std::uniform_real_distribution<> lat_distribution(min_lat, max_lat);

  Options options;
  create_costing_options(options);
  sif::TravelMode mode;
  auto costs = sif::CostFactory().CreateModeCosting(options, mode);
  auto cost = costs[static_cast<size_t>(mode)];

  // A few locations around Utrecht. Origins and destinations are constructed
  // from these for the queries
  // locations.emplace_back(midgard::PointLL{5.115873, 52.099247});
  locations.emplace_back(midgard::PointLL{5.117328, 52.099464});
  locations.emplace_back(midgard::PointLL{5.114576, 52.101841});
  locations.emplace_back(midgard::PointLL{5.114598, 52.103607});
  locations.emplace_back(midgard::PointLL{5.112481, 52.074073});
  locations.emplace_back(midgard::PointLL{5.135983, 52.110116});
  locations.emplace_back(midgard::PointLL{5.095273, 52.108956});
  locations.emplace_back(midgard::PointLL{5.110077, 52.062043});
  locations.emplace_back(midgard::PointLL{5.025595, 52.067372});

  const auto projections = loki::Search(locations, *clean_reader, cost);
  if (projections.size() == 0) {
    throw std::runtime_error("Found no matching locations");
  }

  std::vector<valhalla::Location> origins;
  std::vector<valhalla::Location> destinations;

  {
    auto it = projections.cbegin();
    if (it == projections.cend()) {
      throw std::runtime_error("Found no matching locations");
    }
    while (true) {
      auto origin = valhalla::Location{};
      baldr::PathLocation::toPBF(it->second, &origin, *clean_reader);
      ++it;
      if (it == projections.cend()) {
        break;
      }
      origins.push_back(origin);
      destinations.push_back(valhalla::Location{});
      baldr::PathLocation::toPBF(it->second, &destinations.back(), *clean_reader);
    }
  }

  if (origins.size() == 0) {
    throw std::runtime_error("No origins available for test");
  }

  std::size_t route_size = 0;

  for (auto _ : state) {
    thor::BidirectionalAStar astar;
    for (int i = 0; i < origins.size(); ++i) {
      // LOG_WARN("Running index "+std::to_string(i));
      auto result = astar.GetBestPath(origins[i], destinations[i], *clean_reader, costs,
                                      sif::TravelMode::kDrive);
      astar.Clear();
      route_size += 1;
    }
  }
  if (route_size == 0) {
    throw std::runtime_error("Failed all routes");
  }
  state.counters["Routes"] =
      benchmark::Counter(route_size, benchmark::Counter::kIsIterationInvariantRate);
}

void customize_traffic(const boost::property_tree::ptree& config,
                       baldr::GraphId& target_edge_id,
                       const int target_speed) {
  test::build_live_traffic_data(config);
  // Make some updates to the traffic .tar file.
  // Generate traffic data
  std::function<void(baldr::GraphReader&, baldr::TrafficTile&, int, baldr::TrafficSpeed*)>
      generate_traffic = [&target_edge_id, &target_speed](baldr::GraphReader& reader,
                                                          baldr::TrafficTile& tile, int index,
                                                          baldr::TrafficSpeed* current) -> void {
    baldr::GraphId tile_id(tile.header->tile_id);
    auto edge_id = baldr::GraphId(tile_id.tileid(), tile_id.level(), index);
    if (edge_id == target_edge_id) {
      current->breakpoint1 = 255;
      current->overall_speed = target_speed >> 1;
      current->speed1 = target_speed >> 1;
    }
  };
  test::customize_live_traffic_data(config, generate_traffic);
}

BENCHMARK(BM_UtrechtBidirectionalAstar)->Unit(benchmark::kMillisecond);

/** Benchmarks the GetSpeed function */
static void BM_GetSpeed(benchmark::State& state) {

  const auto config = build_config("get-speed.tar");
  auto tgt_edge_id = baldr::GraphId(3196, 0, 3221);
  const auto tgt_speed = 50;
  customize_traffic(config, tgt_edge_id, tgt_speed);

  auto clean_reader = test::make_clean_graphreader(config.get_child("mjolnir"));

  auto tile = clean_reader->GetGraphTile(baldr::GraphId(tgt_edge_id));
  if (tile == nullptr) {
    throw std::runtime_error("Target tile not found");
  }
  auto edge = tile->directededge(tgt_edge_id);
  if (edge == nullptr) {
    throw std::runtime_error("Target edge not found");
  }

  if (tile->GetSpeed(edge, 255, 1) != tgt_speed) {
    fprintf(stderr, "ERROR: tgt_speed: %i, GetSpeed(...): %i\n", tgt_speed,
            tile->GetSpeed(edge, 255, 1));
    throw std::runtime_error("Target edge was not at target speed");
  }

  for (auto _ : state) {
    tile->GetSpeed(edge, 255, 1);
  }
}

BENCHMARK(BM_GetSpeed)->Unit(benchmark::kNanosecond);

/** Benchmarks the Allowed function */
static void BM_Sif_Allowed(benchmark::State& state) {

  const auto config = build_config("sif-allowed.tar");
  auto tgt_edge_id = baldr::GraphId(3196, 0, 3221);
  auto tgt_speed = 100;
  customize_traffic(config, tgt_edge_id, tgt_speed);

  auto clean_reader = test::make_clean_graphreader(config.get_child("mjolnir"));

  Options options;
  create_costing_options(options);
  sif::TravelMode mode;
  auto costs = sif::CostFactory().CreateModeCosting(options, mode);
  auto cost = costs[static_cast<size_t>(mode)];

  auto tile = clean_reader->GetGraphTile(baldr::GraphId(tgt_edge_id));
  if (tile == nullptr) {
    throw std::runtime_error("Target tile not found");
  }
  auto edge = tile->directededge(tgt_edge_id);
  if (edge == nullptr) {
    throw std::runtime_error("Target edge not found");
  }

  if (tile->GetSpeed(edge, 255, 1) != tgt_speed) {
    throw std::runtime_error("Target edge was not at target speed");
  }

  // Mock a phony predecessor
  // auto pred = sif::EdgeLabel(0, tgt_edge_id, edge, costs, 1.0, 1.0,
  // sif::TravelMode::kDrive,10,sif::Cost());
  auto pred = sif::EdgeLabel();
  uint8_t restriction_idx;

  for (auto _ : state) {
    cost->Allowed(edge, pred, tile, tgt_edge_id, 0, 0, restriction_idx);
  }
}

BENCHMARK(BM_Sif_Allowed)->Unit(benchmark::kNanosecond);

} // namespace

BENCHMARK_MAIN();
