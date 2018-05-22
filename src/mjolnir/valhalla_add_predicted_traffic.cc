#include "baldr/graphreader.h"
#include "midgard/logging.h"
#include "mjolnir/graphtilebuilder.h"
#include <cmath>
#include <cstdint>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <deque>
#include <future>
#include <mutex>
#include <queue>
#include <thread>

#include "config.h"
#include "segment.pb.h"
#include "tile.pb.h"

namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vj = valhalla::mjolnir;

namespace bpo = boost::program_options;
namespace bpt = boost::property_tree;
namespace bfs = boost::filesystem;

namespace {

// Struct to hold stats information during each threads work
struct stats {
  uint32_t count;

  // Accumulate counts from all threads
  void operator()(const stats& other) {
    count += other.count;
  }
};

void add_predicted_traffic(const bpt::ptree& pt,
                           std::deque<std::string>& traffic_tiles,
                           std::queue<vb::GraphId>& tilequeue,
                           std::mutex& lock,
                           std::promise<stats>& result) {

  stats stat;

  // Get the tile dir
  std::string tile_dir = pt.get<std::string>("mjolnir.tile_dir");
  // Iterate through the tiles in the queue and perform enhancements
  while (true) {
    // Get the next tile Id from the queue and get writeable and readable
    // tile. Lock while we access the tile queue and get the tile.
    lock.lock();
    if (tilequeue.empty()) {
      lock.unlock();
      break;
    }
    GraphId tile_id = tilequeue.front();
    tilequeue.pop();

    vj::GraphTileBuilder tile_builder(tile_dir, tile_id.Tile_Base(), true);

    for (uint32_t i = 0; i < tile_builder.header()->nodecount(); i++) {
      NodeInfo& nodeinfo = tile_builder.node_builder(i);

      // Go through directed edges and add predicted traffic
      const DirectedEdge* edges = tile_builder.directededges(nodeinfo.edge_index());
      for (uint32_t j = 0; j < nodeinfo.edge_count(); j++) {
        DirectedEdge& directededge = tile_builder.directededge_builder(nodeinfo.edge_index() + j);
        tile_builder.predicted_traffic().emplace_back(0, 0);
        stat.count++;
      }
    }
    tile_builder.UpdatePedictedTraffic();
    lock.unlock();
  }
  result.set_value(stat);
}

} // anonymous namespace

int main(int argc, char** argv) {
  std::string config, tile_dir;
  unsigned int num_threads = 1;

  bpo::options_description options("valhalla_add_predicted_traffic " VERSION "\n"
                                   "\n"
                                   " Usage: valhalla_add_predicted_traffice [options]\n"
                                   "\n"
                                   "adds predicted traffic to valhalla tiles. "
                                   "\n"
                                   "\n");

  options.add_options()("help,h", "Print this help message.")("version,v",
                                                              "Print the version of this software.")(
      "traffic-tile-dir,t", bpo::value<std::string>(&tile_dir),
      "Location of traffic csv tiles.")("concurrency,j", bpo::value<unsigned int>(&num_threads),
                                        "Number of threads to use.")
      // positional arguments
      ("config", bpo::value<std::string>(&config), "Valhalla configuration file [required]");

  bpo::positional_options_description pos_options;
  pos_options.add("config", 1);
  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(),
               vm);
    bpo::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  if (vm.count("help") || !vm.count("config")) {
    std::cout << options << "\n";
    return EXIT_SUCCESS;
  }

  if (vm.count("version")) {
    std::cout << "valhalla_add_predicted_traffic " << VERSION << "\n";
    return EXIT_SUCCESS;
  }

  if (!vm.count("traffic-tile-dir")) {
    std::cout << "You must provide a tile directory to read the csv tiles from.\n";
    return EXIT_FAILURE;
  }

  // queue up all the work we'll be doing
  std::deque<std::string> traffic_tiles;
  auto itr = bfs::recursive_directory_iterator(tile_dir);
  auto end = bfs::recursive_directory_iterator();
  for (; itr != end; ++itr) {
    auto dir_entry = *itr;
    if (bfs::is_regular_file(dir_entry)) {
      auto ext = dir_entry.path().extension();
      if (ext == ".csv") {
        traffic_tiles.emplace_back(dir_entry.path().string());
      }
    }
  }

  // Shuffle the list to minimize the chance of adjacent tiles being access
  // by different threads at the same time
  // std::random_shuffle(traffic_tiles.begin(), traffic_tiles.end());

  // configure logging
  vm::logging::Configure({{"type", "std_err"}, {"color", "true"}});

  // parse the config
  bpt::ptree pt;
  bpt::read_json(config.c_str(), pt);

  // fire off some threads to do the work
  LOG_INFO("Adding predicted traffic with " + std::to_string(num_threads) + " threads");
  std::vector<std::shared_ptr<std::thread>> threads(num_threads);

  boost::property_tree::ptree hierarchy_properties = pt.get_child("mjolnir");
  GraphReader reader(hierarchy_properties);
  auto level = TileHierarchy::levels().rbegin();
  uint32_t total_count = 0;

  for (; level != TileHierarchy::levels().rend(); ++level) {
    // Create a randomized queue of tiles to work from
    auto tile_level = level->second;
    std::deque<vb::GraphId> tempqueue;
    auto level_tiles = reader.GetTileSet(tile_level.level);
    for (const auto& tile_id : level_tiles) {
      tempqueue.emplace_back(tile_id);
    }
    std::random_shuffle(tempqueue.begin(), tempqueue.end());
    std::queue<vb::GraphId> tilequeue(tempqueue);

    // A place to hold the results of those threads (exceptions, stats)
    std::list<std::promise<stats>> results;

    std::mutex lock;
    for (auto& thread : threads) {
      results.emplace_back();

      thread.reset(new std::thread(add_predicted_traffic, std::cref(pt), std::ref(traffic_tiles),
                                   std::ref(tilequeue), std::ref(lock), std::ref(results.back())));
    }

    // wait for it to finish
    for (auto& thread : threads)
      thread->join();

    for (auto& result : results) {
      try {
        auto thread_stats = result.get_future().get();
        total_count += thread_stats.count;
      } catch (std::exception& e) {
        // TODO: throw further up the chain?
      }
    }
  }
  LOG_INFO("Finished");
  LOG_INFO("Added " + std::to_string(total_count) + " predicted traffic records");
  return EXIT_SUCCESS;
}
