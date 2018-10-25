#include "baldr/graphreader.h"
#include "baldr/predictedspeeds.h"
#include "midgard/logging.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/util.h"
#include <cmath>
#include <cstdint>

#include "baldr/rapidjson_utils.h"
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/tokenizer.hpp>

#include <deque>
#include <future>
#include <mutex>
#include <queue>
#include <string>
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
  uint32_t compressed_count;
  uint32_t updated_count;
  uint32_t dup_count;

  // Accumulate counts from all threads
  void operator()(const stats& other) {
    constrained_count += other.constrained_count;
    free_flow_count += other.free_flow_count;
    compressed_count += other.compressed_count;
    updated_count += other.updated_count;
    dup_count += other.dup_count;
  }
};

struct TrafficSpeeds {
  uint32_t id;
  uint8_t constrained_flow_speed;
  uint8_t free_flow_speed;
  std::vector<int16_t> coefficients;

  bool operator<(const TrafficSpeeds& other) const {
    return id < other.id;
  }
};

struct unique_data_t {
  std::unordered_map<vb::GraphId, std::vector<TrafficSpeeds>> tile_speeds;
};

// Convert big endian bytes to little endian
int16_t to_little_endian(const int16_t val) {
  return (val << 8) | ((val >> 8) & 0x00ff);
}

// base64 decoding
std::string decode64(const std::string& val) {
  using namespace boost::archive::iterators;
  using It = transform_width<binary_from_base64<std::string::const_iterator>, 8, 6>;
  return std::string(It(std::begin(val)), It(std::end(val)));
}

/**
 * Read speed CSV file and update the tile_speeds in unique_data
 */
void ParseTrafficFile(const std::string& directory,
                      const std::string& filename,
                      unique_data_t& unique_data,
                      std::mutex& lock,
                      stats& stat) {
  typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
  boost::char_separator<char> sep{","};

  // Open file
  std::string line;
  const std::string& full_filename = directory + "/" + filename + "." + "csv";
  std::ifstream file(full_filename);
  uint32_t line_num = 1;
  if (file.is_open()) {
    GraphId last_tile_id;
    std::vector<TrafficSpeeds> ts;
    while (getline(file, line)) {
      TrafficSpeeds traffic{};
      tokenizer tok{line, sep};
      uint32_t field_num = 0;
      GraphId tile_id;
      bool has_error = false;
      for (const auto& t : tok) {
        switch (field_num) {
          case 0: {
            try {
              GraphId tmp(std::stoull(t));
              tile_id = tmp.Tile_Base();
              // only need to save the unique id in the tile as
              // our key in tile_speeds is the tileID.
              traffic.id = tmp.id();
            } catch (std::exception& e) {
              LOG_WARN("Invalid GraphId in file: " + full_filename + " line number " +
                       std::to_string(line_num));
              has_error = true;
            }
          } break;
          case 1: {
            if (has_error)
              break;
            try {
              traffic.free_flow_speed = std::stoi(t);
              stat.free_flow_count++;
            } catch (std::exception& e) {
              LOG_WARN("Invalid free flow speed in file: " + full_filename + " line number " +
                       std::to_string(line_num));
              has_error = true;
            }
          } break;
          case 2: {
            if (has_error)
              break;
            try {
              traffic.constrained_flow_speed = std::stoi(t);
              stat.constrained_count++;
            } catch (std::exception& e) {
              LOG_WARN("Invalid constrained flow speed in file: " + full_filename + " line number " +
                       std::to_string(line_num));
              has_error = true;
            }
          } break;
          case 3: {
            if (has_error)
              break;
            if (t.size()) {
              // Decode the base64 predicted speeds
              // Decode the base64 string and cast the data to a raw string of signed bytes
              auto decoded_str = decode64(t);
              if (decoded_str.size() != 402) {
                throw std::runtime_error("Decoded speed string size should be 402 but is " +
                                         std::to_string(decoded_str.size()));
              }
              auto raw = reinterpret_cast<const int8_t*>(decoded_str.data());

              // Check that the first value pair == 1
              if (static_cast<std::int8_t>(raw[0]) != 1) {
                throw std::runtime_error("First value should be 1");
              }

              // Create the coefficients. Each group of 2 bytes represents a signed, int16 number (big
              // endian). Convert to little endian.
              int idx = 1;
              traffic.coefficients.reserve(kCoefficientCount);
              for (uint32_t i = 0; i < kCoefficientCount; ++i, idx += 2) {
                traffic.coefficients.push_back(
                    to_little_endian(*(reinterpret_cast<const int16_t*>(&raw[idx]))));
              }
              stat.compressed_count++;
            }
          } break;
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
      line_num++;
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
    file.close();
  } else {
    LOG_ERROR("Could not open file: " + directory + filename + "." + "csv");
  }
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
  for (; tile_start != tile_end; ++tile_start)
    ParseTrafficFile(traffic_dir, *tile_start, unique_data, lock, stat);

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

    // Update directed edges as needed
    uint32_t idx = 0;
    uint32_t count = 0;
    uint32_t duplicates = 0;
    std::vector<DirectedEdge> directededges;
    for (uint32_t j = 0; j < tile_builder.header()->directededgecount(); ++j, ++idx) {
      DirectedEdge& directededge = tile_builder.directededge(idx);
      for (const auto& speeds : tile_start->second) {
        if (speeds.id == idx) {

          if (directededge.constrained_flow_speed() || directededge.free_flow_speed() ||
              directededge.predicted_speed()) {
            duplicates++;
          }

          if (speeds.constrained_flow_speed) {
            directededge.set_constrained_flow_speed(speeds.constrained_flow_speed);
          }
          if (speeds.free_flow_speed) {
            directededge.set_free_flow_speed(speeds.free_flow_speed);
          }
          if (speeds.coefficients.size() > 0) {
            tile_builder.AddPredictedSpeed(idx, speeds.coefficients);
            directededge.set_predicted_speed(true);
          }
          count++;
        }
      }

      // Add the directed edge to the local list
      directededges.emplace_back(std::move(directededge));
    }
    stat.updated_count += count;
    stat.dup_count += duplicates;

    // Write the new tile with updated directed edges and the predicted speeds
    tile_builder.UpdatePredictedSpeeds(directededges);
  }
  result.set_value(stat);
}

} // anonymous namespace

int main(int argc, char** argv) {
  std::string config, tile_dir;
  std::string inline_config;
  boost::filesystem::path config_file_path;

  unsigned int num_threads = 1;

  bpo::options_description options("valhalla_add_predicted_traffic " VALHALLA_VERSION "\n"
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
    std::cout << "valhalla_add_predicted_traffic " << VALHALLA_VERSION << "\n";
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
        traffic_tiles.emplace(file_name);
      }
    }
  }

  // Read the config file
  boost::property_tree::ptree pt;
  if (vm.count("inline-config")) {
    std::stringstream ss;
    ss << inline_config;
    rapidjson::read_json(ss, pt);
  } else if (vm.count("config") && boost::filesystem::is_regular_file(config_file_path)) {
    rapidjson::read_json(config_file_path.string(), pt);
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

  LOG_INFO("Parsing speeds from " + std::to_string(traffic_tiles.size()) + " quadkey tiles.");
  size_t floor = traffic_tiles.size() / threads.size();
  size_t at_ceiling = traffic_tiles.size() - (threads.size() * floor);
  std::unordered_set<std::string>::const_iterator tile_start, tile_end = traffic_tiles.begin();
  uint32_t constrained_count = 0, free_flow_count = 0, compressed_count = 0;
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
    threads[i].reset(new std::thread(parse_traffic_tiles, tile_dir, std::ref(lock), tile_start,
                                     tile_end, std::ref(unique_data), std::ref(results.back())));
  }

  // wait for it to finish
  for (auto& thread : threads)
    thread->join();

  for (auto& result : results) {
    try {
      auto thread_stats = result.get_future().get();
      constrained_count += thread_stats.constrained_count;
      free_flow_count += thread_stats.free_flow_count;
      compressed_count += thread_stats.compressed_count;

    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }

  LOG_INFO("Parsed " + std::to_string(constrained_count) + " constrained traffic speeds.");
  LOG_INFO("Parsed " + std::to_string(free_flow_count) + " free flow traffic speeds.");
  LOG_INFO("Parsed " + std::to_string(compressed_count) + " compressed records.");

  // Sort the Traffic speeds within each tile. Wanted to do this so that the linear search
  // for speeds attached to an edge would be faster...but the data seemed to be corrupted?
  //  for (auto& tile : unique_data.tile_speeds) {
  //    std::sort(tile.second.begin(), tile.second.end());
  //  }

  LOG_INFO("Updating speeds for " + std::to_string(unique_data.tile_speeds.size()) +
           " Valhalla tiles.");
  floor = unique_data.tile_speeds.size() / threads.size();
  at_ceiling = unique_data.tile_speeds.size() - (threads.size() * floor);
  std::unordered_map<vb::GraphId, std::vector<TrafficSpeeds>>::const_iterator t_start,
      t_end = unique_data.tile_speeds.cbegin();
  uint32_t updated_count = 0;
  uint32_t duplicate_count = 0;

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
    threads[i].reset(new std::thread(update_valhalla_tiles, std::cref(pt), t_start, t_end,
                                     std::ref(unique_data), std::ref(results.back())));
  }

  // wait for it to finish
  for (auto& thread : threads)
    thread->join();

  for (auto& result : results) {
    try {
      auto thread_stats = result.get_future().get();
      updated_count += thread_stats.updated_count;
      duplicate_count += thread_stats.dup_count;

    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }

  LOG_INFO("Updated " + std::to_string(updated_count) + " directed edges.");
  LOG_INFO("Duplicate count " + std::to_string(duplicate_count) + ".");

  LOG_INFO("Finished");

  GraphReader reader(pt.get_child("mjolnir"));
  // Iterate through the tiles
  int shortcuts_with_speed = 0;
  int non_dr_with_speed = 0;
  std::vector<uint32_t> dr_class_edges_links(8);
  std::vector<uint32_t> dr_road_class_edges(8);
  std::vector<uint32_t> pred_road_class_edges(8);
  std::vector<uint32_t> ff_road_class_edges(8);
  for (uint32_t level = 0; level < 3; level++) {
    auto tiles = reader.GetTileSet(level);
    for (const auto& tile_id : tiles) {
      const GraphTile* tile = reader.GetGraphTile(tile_id);
      uint32_t n = tile->header()->directededgecount();
      if (n == 0)
        continue;
      const DirectedEdge* de = tile->directededge(0);
      for (uint32_t i = 0; i < n; i++, de++) {
        uint32_t rc = (int)de->classification();

        if (de->is_shortcut() && de->free_flow_speed() > 0) {
          shortcuts_with_speed++;
        }
        if (de->is_shortcut()) {
          continue;
        }
        if ((de->forwardaccess() & kAutoAccess)) {
          dr_road_class_edges[rc]++;
          if (de->link())
            dr_class_edges_links[rc]++;
        } else {
          if (de->free_flow_speed()) {
            non_dr_with_speed++;
          }
          continue;
        }

        // Presence of predicted speeds
        if (de->predicted_speed()) {
          pred_road_class_edges[rc]++;
        }
        auto shape = tile->edgeinfo(de->edgeinfo_offset()).shape();

        // Presence of free flow and/or constrained flow speeds
        if (de->free_flow_speed() > 0 || de->constrained_flow_speed() > 0) {
          ff_road_class_edges[rc]++;
        }

        if (de->predicted_speed() && de->free_flow_speed() == 0 &&
            de->constrained_flow_speed() == 0) {
          LOG_WARN("Edge has predicted speed but no ff or constrained speed");
        }
      }
    }
  }
  LOG_INFO("Stats - excluding shortcut edges");
  LOG_INFO("non driveable with speed = " + std::to_string(non_dr_with_speed));
  LOG_INFO("Shortcuts with speed = " + std::to_string(shortcuts_with_speed));
  LOG_INFO("Transitions with speed = " + std::to_string(shortcuts_with_speed));
  uint32_t totaldriveable = 0, totalpt = 0, totalff = 0, totaldriveablelink = 0;
  for (uint32_t i = 0; i < 8; i++) {
    float pct1 = 100.0f * (float)pred_road_class_edges[i] / dr_road_class_edges[i];
    float pct2 = 100.0f * (float)ff_road_class_edges[i] / dr_road_class_edges[i];

    std::stringstream ss_pct1, ss_pct2;
    ss_pct1 << std::setprecision(1) << std::fixed << pct1;
    ss_pct2 << std::setprecision(1) << std::fixed << pct2;
    LOG_INFO("RC " + std::to_string(i) + ": driveable edges " +
             std::to_string(dr_road_class_edges[i]) + " predtraffic " +
             std::to_string(pred_road_class_edges[i]) + " pct " + ss_pct1.str() + " ff " +
             std::to_string(ff_road_class_edges[i]) + " pct " + ss_pct2.str());
    totaldriveable += dr_road_class_edges[i];
    totaldriveablelink += dr_class_edges_links[i];
    totalpt += pred_road_class_edges[i];
    totalff += ff_road_class_edges[i];
  }
  LOG_INFO("total driveable = " + std::to_string(totaldriveable) + " total driveable ramps/links = " +
           std::to_string(totaldriveablelink) + " total driveable non ramps/links = " +
           std::to_string(totaldriveable - totaldriveablelink) + " total pred " +
           std::to_string(totalpt) + " total ff " + std::to_string(totalff));

  return EXIT_SUCCESS;
}
