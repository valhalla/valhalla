#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "mjolnir/graphtilebuilder.h"
#include "speed_assigner.h"

#include <cxxopts.hpp>

#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <future>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

namespace bpt = boost::property_tree;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

void assign(const boost::property_tree::ptree& config,
            std::deque<GraphId>& tilequeue,
            std::mutex& lock,
            std::promise<std::pair<size_t, size_t>>& result) {
  size_t assigned = 0, total = 0;
  SpeedAssigner assigner(config.get_optional<std::string>("mjolnir.default_speeds_config"));
  bool infer_turn_channels = config.get<bool>("mjolnir.data_processing.infer_turn_channels");
  GraphReader graph_reader(config.get_child("mjolnir"));

  while (true) {
    // get the next job or quit
    lock.lock();
    if (tilequeue.empty()) {
      lock.unlock();
      break;
    }
    GraphId tile_id = tilequeue.front();
    tilequeue.pop_front();
    lock.unlock();

    // update all the edges
    graph_tile_ptr tile = graph_reader.GetGraphTile(tile_id);
    if (tile->header()->directededgecount() == 0)
      continue;
    std::vector<DirectedEdge> edges(tile->directededge(0),
                                    tile->directededge(0) + tile->header()->directededgecount());
    for (auto& edge : edges) {
      // get the end node
      lock.lock();
      graph_tile_ptr end_tile = graph_reader.GetGraphTile(edge.endnode());
      lock.unlock();
      const auto* node = end_tile->node(edge.endnode());
      const auto* admin = end_tile->admin(node->admin_index());
      // TODO: if this was a shortcut we need to bother about turn durations...
      // update the speed
      assigned += assigner.UpdateSpeed(edge, node->density(), infer_turn_channels,
                                       admin->country_iso(), admin->state_iso());
      ++total;
    }

    // copy the nodes
    std::vector<NodeInfo> nodes(tile->node(0), tile->node(0) + tile->header()->nodecount());

    // write the tile back out
    GraphTileBuilder tilebuilder(config.get<std::string>("mjolnir.tile_dir"), tile_id, false);
    lock.lock();
    tilebuilder.Update(nodes, edges);
    if (graph_reader.OverCommitted()) {
      graph_reader.Trim();
    }
    lock.unlock();
  }

  result.set_value({assigned, total});
}

int main(int argc, char** argv) {
  // args
  filesystem::path config_file_path;
  bpt::ptree config;

  try {
    // clang-format off
    cxxopts::Options options(
      "valhalla_assign_speeds",
      "valhalla_assign_speeds " VALHALLA_VERSION "\n\n"
      "Modifies default speeds based on provided configuration.\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("version")) {
      std::cout << "pbfadminbuilder " << VALHALLA_VERSION << "\n";
      return EXIT_SUCCESS;
    }

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      return EXIT_SUCCESS;
    }

    // Read the config file
    if (result.count("inline-config")) {
      std::stringstream ss;
      ss << result["inline-config"].as<std::string>();
      rapidjson::read_json(ss, config);
    } else if (result.count("config") &&
               filesystem::is_regular_file(
                   config_file_path = filesystem::path(result["config"].as<std::string>()))) {
      rapidjson::read_json(config_file_path.string(), config);
    } else {
      std::cerr << "Configuration is required\n" << options.help() << std::endl;
      return EXIT_FAILURE;
    }

    if (!result.count("config")) {
      std::cout << "You must provide a config for loading and modifying tiles.\n";
      return EXIT_FAILURE;
    }
  } catch (cxxopts::OptionException& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
  }

  // configure logging
  rapidjson::read_json(config_file_path.string(), config);
  config.get_child("mjolnir").erase("tile_extract");
  config.get_child("mjolnir").erase("tile_url");
  config.get_child("mjolnir").erase("traffic_extract");
  auto logging_subtree = config.get_child_optional("mjolnir.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  // queue some tiles up to modify
  std::deque<GraphId> tilequeue;
  GraphReader reader(config.get_child("mjolnir"));
  auto tileset = reader.GetTileSet();
  for (const auto& id : tileset) {
    tilequeue.emplace_back(id);
  }
  std::shuffle(tilequeue.begin(), tilequeue.end(), std::mt19937(3));

  // spawn threads to modify the tiles
  auto concurrency =
      std::max(static_cast<unsigned int>(1),
               config.get<unsigned int>("mjolnir.concurrency", std::thread::hardware_concurrency()));
  std::vector<std::shared_ptr<std::thread>> threads(concurrency);
  std::list<std::promise<std::pair<size_t, size_t>>> results;
  std::mutex lock;
  for (auto& thread : threads) {
    results.emplace_back();
    thread.reset(new std::thread(assign, std::cref(config), std::ref(tilequeue), std::ref(lock),
                                 std::ref(results.back())));
  }

  // collect the results
  for (auto& thread : threads) {
    thread->join();
  }
  size_t assigned = 0, total = 0;
  for (auto& result : results) {
    auto stat = result.get_future().get();
    assigned += stat.first;
    total += stat.second;
  }

  LOG_INFO("Assigned speeds to " + std::to_string(assigned) + " edges in total out of " +
           std::to_string(total));
}
