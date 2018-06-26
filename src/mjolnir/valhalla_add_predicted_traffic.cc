#include "baldr/graphreader.h"
#include "midgard/logging.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/util.h"
#include <cmath>
#include <cstdint>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/tokenizer.hpp>

#include <deque>
#include <future>
#include <mutex>
#include <queue>
#include <thread>

#include "config.h"

namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vj = valhalla::mjolnir;

namespace bpo = boost::program_options;
namespace bpt = boost::property_tree;
namespace bfs = boost::filesystem;

namespace {

// Struct to hold stats information during each threads work
struct stats {
  uint32_t constrained_count;
  uint32_t free_flow_count;
  uint32_t updated_count;

  // Accumulate counts from all threads
  void operator()(const stats& other) {
    constrained_count += other.constrained_count;
    free_flow_count += other.free_flow_count;
    updated_count += other.updated_count;
  }
};

struct TrafficSpeeds {
  uint32_t id;
  uint8_t constrained_flow_speed;
  uint8_t free_flow_speed;
};

struct unique_data_t {
  std::unordered_map<vb::GraphId, std::vector<TrafficSpeeds>> tile_speeds;
};

/**
 * Read speed CSV file and update the tile_speeds in unique_data
 */
void ParseTrafficFile(const std::string& directory,
                      const std::string& filename,
                      const std::string& ext,
                      unique_data_t& unique_data,
                      std::mutex& lock,
                      stats& stat) {
  typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
  boost::char_separator<char> sep{","};

  // Open file
  std::string line;
  std::ifstream file(directory + filename + "." + ext + "." + "csv");
  if (file.is_open()) {
    GraphId last_tile_id;
    std::vector<TrafficSpeeds> ts;
    while (getline(file, line)) {
      TrafficSpeeds traffic{};
      tokenizer tok{line, sep};
      uint32_t field_num = 0;
      GraphId tile_id;
      for (const auto& t : tok) {
        switch (field_num) {
          case 0: {
            GraphId tmp(std::stoull(t));
            tile_id = tmp.Tile_Base();
            // only need to save the unique id in the tile as
            // our key in tile_speeds is the tileID.
            traffic.id = tmp.id();
          } break;
          case 1:
            if (ext == "constrained") {
              traffic.constrained_flow_speed = std::stoi(t);
              stat.constrained_count++;
            } else if (ext == "freeflow") {
              traffic.free_flow_speed = std::stoi(t);
              stat.free_flow_count++;
            }
            break;
        }
        field_num++;
      }

      // assume that we are in the same tile.
      // trying to save on looking up/finding a vector for a tile.
      if (last_tile_id == tile_id) {
        ts.emplace_back(traffic);
      } else {
        if (last_tile_id != kInvalidGraphId) {
          lock.lock();
          auto it = unique_data.tile_speeds.find(last_tile_id);
          if (it != unique_data.tile_speeds.end())
            it->second.insert(std::end(it->second), std::begin(ts), std::end(ts));
          else
            unique_data.tile_speeds.insert({last_tile_id, ts});
          lock.unlock();
          ts.clear();
        }
        last_tile_id = tile_id;
        ts.emplace_back(traffic);
      }
    }
    if (last_tile_id != kInvalidGraphId) {
      lock.lock();
      auto it = unique_data.tile_speeds.find(last_tile_id);
      if (it != unique_data.tile_speeds.end())
        it->second.insert(std::end(it->second), std::begin(ts), std::end(ts));
      else
        unique_data.tile_speeds.insert({last_tile_id, ts});
      lock.unlock();
    }
  }
  file.close();
}

/**
 * Read both the constrained and freeflow speed CSV files
 * We expect the files to be named as <quadtreeID>.constrained.csv and
 * <quadtreeID>.freeflow.csv. (e.g., 1202021.constrained.csv and 1202021.freeflow.csv)
 */
void parse_traffic_tiles(const std::string& traffic_dir,
                         std::mutex& lock,
                         std::unordered_set<std::string>::const_iterator tile_start,
                         std::unordered_set<std::string>::const_iterator tile_end,
                         unique_data_t& unique_data,
                         std::promise<stats>& result) {

  // Iterate through the tiles and parse them
  stats stat{};
  for (; tile_start != tile_end; ++tile_start) {
    ParseTrafficFile(traffic_dir, *tile_start, "constrained", unique_data, lock, stat);
    ParseTrafficFile(traffic_dir, *tile_start, "freeflow", unique_data, lock, stat);
  }
  result.set_value(stat);
}

void update_valhalla_tiles(
    const bpt::ptree& pt,
    std::unordered_map<vb::GraphId, std::vector<TrafficSpeeds>>::const_iterator tile_start,
    std::unordered_map<vb::GraphId, std::vector<TrafficSpeeds>>::const_iterator tile_end,
    const unique_data_t& unique_data,
    std::promise<stats>& result) {

  // Iterate through the tiles and parse them
  stats stat{};
  // Get the tile dir
  std::string tile_dir = pt.get<std::string>("mjolnir.tile_dir");
  // Iterate through the tiles in the queue and perform enhancements
  for (; tile_start != tile_end; ++tile_start) {
    // Get the tile
    vj::GraphTileBuilder tile_builder(tile_dir, tile_start->first, false);
    // Update nodes and directed edges as needed
    std::vector<NodeInfo> nodes;
    std::vector<DirectedEdge> directededges;
    std::unordered_set<GraphId> edges;
    uint32_t nodecount = tile_builder.header()->nodecount();
    GraphId node = tile_start->first;
    for (uint32_t i = 0; i < nodecount; i++, ++node) {
      // The node we will modify
      NodeInfo nodeinfo = tile_builder.node(i);
      // Go through directed edges and validate/update data
      uint32_t idx = nodeinfo.edge_index();
      GraphId edgeid(node.tileid(), node.level(), idx);
      for (uint32_t j = 0, n = nodeinfo.edge_count(); j < n; j++, idx++, ++edgeid) {
        DirectedEdge& directededge = tile_builder.directededge(idx);
        // could be more than one in the vector
        for (const auto& speeds : tile_start->second) {
          if (speeds.id == idx) {
            if (speeds.constrained_flow_speed)
              directededge.set_constrained_flow_speed(speeds.constrained_flow_speed);
            if (speeds.free_flow_speed)
              directededge.set_free_flow_speed(speeds.free_flow_speed);
            edges.emplace(tile_start->first.tileid(), tile_start->first.level(), speeds.id);
          }
        }
        // Add the directed edge to the local list
        directededges.emplace_back(std::move(directededge));
      }
      // Add the node to the list
      nodes.emplace_back(std::move(nodeinfo));
    }
    stat.updated_count += edges.size();

    // Write the new tile
    tile_builder.Update(nodes, directededges);
  }
  result.set_value(stat);
}

} // anonymous namespace

int main(int argc, char** argv) {
  std::string config, tile_dir;
  std::string inline_config;
  boost::filesystem::path config_file_path;

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
      "concurrency,j", bpo::value<unsigned int>(&num_threads),
      "Number of threads to use.")("config,c",
                                   boost::program_options::value<boost::filesystem::path>(
                                       &config_file_path),
                                   "Path to the json configuration file.")("inline-config,i",
                                                                           boost::program_options::
                                                                               value<std::string>(
                                                                                   &inline_config),
                                                                           "Inline json config.")
      // positional arguments
      ("traffic-tile-dir,t", bpo::value<std::string>(&tile_dir), "Location of traffic csv tiles.");

  bpo::positional_options_description pos_options;
  pos_options.add("traffic-tile-dir", 1);
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

  if (vm.count("help")) {
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
  std::unordered_set<std::string> traffic_tiles;
  auto itr = bfs::recursive_directory_iterator(tile_dir);
  auto end = bfs::recursive_directory_iterator();
  for (; itr != end; ++itr) {
    auto dir_entry = *itr;
    if (bfs::is_regular_file(dir_entry)) {
      auto ext = dir_entry.path().extension();
      if (ext == ".csv") {
        std::string file_name = dir_entry.path().filename().stem().string();
        size_t speed_type = file_name.find_last_of(".");
        if (speed_type != std::string::npos) {
          file_name.substr(0, speed_type);
          traffic_tiles.emplace(file_name.substr(0, speed_type));
        }
      }
    }
  }

  // Read the config file
  boost::property_tree::ptree pt;
  if (vm.count("inline-config")) {
    std::stringstream ss;
    ss << inline_config;
    boost::property_tree::read_json(ss, pt);
  } else if (vm.count("config") && boost::filesystem::is_regular_file(config_file_path)) {
    boost::property_tree::read_json(config_file_path.string(), pt);
  } else {
    std::cerr << "Configuration is required\n\n" << options << "\n\n";
    return EXIT_FAILURE;
  }

  // configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree =
      pt.get_child_optional("mjolnir.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  LOG_INFO("Adding predicted traffic with " + std::to_string(num_threads) + " threads");
  std::vector<std::shared_ptr<std::thread>> threads(num_threads);

  LOG_INFO("Parsing speeds from " + std::to_string(traffic_tiles.size()) + " tiles.");
  size_t floor = traffic_tiles.size() / threads.size();
  size_t at_ceiling = traffic_tiles.size() - (threads.size() * floor);
  std::unordered_set<std::string>::const_iterator tile_start, tile_end = traffic_tiles.begin();
  uint32_t constrained_count = 0;
  uint32_t free_flow_count = 0;
  unique_data_t unique_data;
  std::mutex lock;
  // A place to hold the results of those threads (exceptions, stats)
  std::list<std::promise<stats>> results;
  // Atomically pass around stats info
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    tile_start = tile_end;
    // Where the range ends
    std::advance(tile_end, tile_count);
    // Make the thread
    results.emplace_back();
    threads[i].reset(new std::thread(parse_traffic_tiles, tile_dir, std::ref(lock),
                                     tile_start, tile_end,
                                     std::ref(unique_data), std::ref(results.back())));
  }

  // wait for it to finish
  for (auto& thread : threads)
    thread->join();

  for (auto& result : results) {
    try {
      auto thread_stats = result.get_future().get();
      constrained_count += thread_stats.constrained_count;
      free_flow_count += thread_stats.free_flow_count;

    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }

  LOG_INFO("Parsed " + std::to_string(constrained_count) + " constrained traffic speeds.");
  LOG_INFO("Parsed " + std::to_string(free_flow_count) + " free flow traffic speeds.");

  LOG_INFO("Updating speeds for " + std::to_string(unique_data.tile_speeds.size()) + " tiles.");
  floor = unique_data.tile_speeds.size() / threads.size();
  at_ceiling = unique_data.tile_speeds.size() - (threads.size() * floor);
  std::unordered_map<vb::GraphId, std::vector<TrafficSpeeds>>::const_iterator t_start,
      t_end = unique_data.tile_speeds.cbegin();
  uint32_t updated_count = 0;
  // A place to hold the results of those threads (exceptions, stats)
  results.clear();
  // Atomically pass around stats info
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    t_start = t_end;
    // Where the range ends
    std::advance(t_end, tile_count);
    // Make the thread
    results.emplace_back();
    threads[i].reset(new std::thread(update_valhalla_tiles, std::cref(pt), t_start,
                                     t_end, std::ref(unique_data), std::ref(results.back())));
  }

  // wait for it to finish
  for (auto& thread : threads)
    thread->join();

  for (auto& result : results) {
    try {
      auto thread_stats = result.get_future().get();
      updated_count += thread_stats.updated_count;

    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }

  LOG_INFO("Updated " + std::to_string(updated_count) + " directed edges.");
  LOG_INFO("Finished");

  return EXIT_SUCCESS;
}
