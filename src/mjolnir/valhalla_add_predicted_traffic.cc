#include "baldr/graphreader.h"
#include "baldr/predictedspeeds.h"
#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/util.h"

#include <cmath>
#include <cstdint>

#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/tokenizer.hpp>

#include <algorithm>
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
  uint8_t constrained_flow_speed;
  uint8_t free_flow_speed;
  std::vector<int16_t> coefficients;
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
std::unordered_map<uint32_t, TrafficSpeeds>
ParseTrafficFile(const std::vector<std::string>& filenames, stats& stat) {
  typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
  boost::char_separator<char> sep{","};
  std::unordered_map<uint32_t, TrafficSpeeds> ts;

  // for each traffic tile
  for (const auto& full_filename : filenames) {
    // Open file
    std::string line;
    std::ifstream file(full_filename);
    uint32_t line_num = 0;
    if (file.is_open()) {
      // for each row in the file
      while (getline(file, line) && ++line_num) {
        decltype(ts)::iterator traffic = ts.end();
        tokenizer tok{line, sep};
        uint32_t field_num = 0;
        bool has_error = false;
        // for each column in the row
        for (const auto& t : tok) {
          if (has_error)
            break;
          // parse each column
          switch (field_num) {
            case 0: {
              try {
                auto inserted = ts.insert(decltype(ts)::value_type(GraphId(t).id(), {}));
                traffic = inserted.first;
                // skip duplicates
                if (!inserted.second) {
                  ++stat.dup_count;
                  has_error = true;
                  traffic = ts.end();
                }
              } catch (std::exception& e) {
                LOG_WARN("Invalid GraphId in file: " + full_filename + " line number " +
                         std::to_string(line_num));
                has_error = true;
              }
            } break;
            case 1: {
              try {
                traffic->second.free_flow_speed = std::stoi(t);
                stat.free_flow_count++;
              } catch (std::exception& e) {
                LOG_WARN("Invalid free flow speed in file: " + full_filename + " line number " +
                         std::to_string(line_num));
                has_error = true;
              }
            } break;
            case 2: {
              try {
                traffic->second.constrained_flow_speed = std::stoi(t);
                stat.constrained_count++;
              } catch (std::exception& e) {
                LOG_WARN("Invalid constrained flow speed in file: " + full_filename +
                         " line number " + std::to_string(line_num));
                has_error = true;
              }
            } break;
            case 3: {
              if (t.size()) {
                try {
                  // Decode the base64 predicted speeds
                  // Decode the base64 string and cast the data to a raw string of signed bytes
                  auto decoded_str = decode64(t);
                  if (decoded_str.size() != 402) {
                    throw std::runtime_error("Decoded speed string size should be 402 but is " +
                                             std::to_string(decoded_str.size()));
                  }
                  auto raw = reinterpret_cast<const int8_t*>(decoded_str.data());

                  // Create the coefficients. Each group of 2 bytes represents a signed, int16 number
                  // (big endian). Convert to little endian.
                  traffic->second.coefficients.reserve(kCoefficientCount);
                  for (uint32_t i = 0, idx = 0; i < kCoefficientCount; ++i, idx += 2) {
                    traffic->second.coefficients.push_back(
                        to_little_endian(*(reinterpret_cast<const int16_t*>(&raw[idx]))));
                  }
                  stat.compressed_count++;
                } catch (std::exception& e) {
                  LOG_WARN("Invalid compressed speeds in file: " + full_filename + " line number " +
                           std::to_string(line_num));
                  has_error = true;
                }
              }
            } break;
            default:
              break;
          }
          field_num++;
        }
        // if this one was erroneous lets not keep it
        if (has_error && traffic != ts.end())
          ts.erase(traffic);
      }
      file.close();
    } else {
      LOG_ERROR("Could not open file: " + full_filename);
    }
  }

  return ts;
}

void update_tile(const std::string& tile_dir,
                 const GraphId& tile_id,
                 const std::unordered_map<uint32_t, TrafficSpeeds>& speeds,
                 stats& stat) {
  auto tile_path = tile_dir + filesystem::path::preferred_separator + GraphTile::FileSuffix(tile_id);
  if (!filesystem::exists(tile_path)) {
    LOG_ERROR("No tile at " + tile_path);
    return;
  }

  // Get the tile
  vj::GraphTileBuilder tile_builder(tile_dir, tile_id, false);

  // Get a count of how many predicted speed edges there will be this avoids reallocs
  size_t pred_count = 0;
  for (uint32_t j = 0; j < tile_builder.header()->directededgecount(); ++j) {
    auto found = speeds.find(j);
    pred_count += found != speeds.cend() && found->second.coefficients.size() == kCoefficientCount;
  }

  // Update directed edges as needed
  std::vector<DirectedEdge> directededges;
  directededges.reserve(tile_builder.header()->directededgecount());
  for (uint32_t j = 0; j < tile_builder.header()->directededgecount(); ++j) {
    // skip edges for which we dont have speed data
    DirectedEdge& directededge = tile_builder.directededge(j);
    auto found = speeds.find(j);
    if (found != speeds.end()) {
      const auto& speed = found->second;
      if (speed.constrained_flow_speed) {
        directededge.set_constrained_flow_speed(speed.constrained_flow_speed);
      }
      if (speed.free_flow_speed) {
        directededge.set_free_flow_speed(speed.free_flow_speed);
      }
      if (speed.coefficients.size() == kCoefficientCount) {
        tile_builder.AddPredictedSpeed(j, speed.coefficients, pred_count);
        directededge.set_has_predicted_speed(true);
      }
      ++stat.updated_count;
    }

    // Add the directed edge to the local list
    directededges.emplace_back(std::move(directededge));
  }

  // Write the new tile with updated directed edges and the predicted speeds
  tile_builder.UpdatePredictedSpeeds(directededges);
}

/**
 * Read both the constrained and freeflow speed CSV files
 * We expect the files to be named as <quadtreeID>.constrained.csv and
 * <quadtreeID>.freeflow.csv. (e.g., 1202021.constrained.csv and 1202021.freeflow.csv)
 */
void update_tiles(
    const std::string& tile_dir,
    std::vector<std::pair<GraphId, std::vector<std::string>>>::const_iterator tile_start,
    std::vector<std::pair<GraphId, std::vector<std::string>>>::const_iterator tile_end,
    std::promise<stats>& result) {

  std::stringstream thread_name;
  thread_name << std::this_thread::get_id();

  // Iterate through the tiles and parse them
  size_t total = tile_end - tile_start;
  double count = 0;
  stats stat{};
  for (; tile_start != tile_end; ++tile_start) {
    LOG_INFO(thread_name.str() + " parsing traffic data for " + std::to_string(tile_start->first));
    auto traffic = ParseTrafficFile(tile_start->second, stat);
    LOG_INFO(thread_name.str() + " add traffic data to " + std::to_string(tile_start->first));
    update_tile(tile_dir, tile_start->first, traffic, stat);
    LOG_INFO(thread_name.str() + " finished " + std::to_string(tile_start->first) + "(" +
             std::to_string(++count / total * 100.0) + ")");
  }

  result.set_value(stat);
}

} // anonymous namespace

int main(int argc, char** argv) {
  std::string config, traffic_tile_dir;
  std::string inline_config;
  std::string config_file_path;
  unsigned int num_threads = std::thread::hardware_concurrency();
  bool summary = false;

  bpo::options_description options("valhalla_add_predicted_traffic " VALHALLA_VERSION "\n"
                                   "\n"
                                   " Usage: valhalla_add_predicted_traffic [options]\n"
                                   "\n"
                                   "adds predicted traffic to valhalla tiles. "
                                   "\n"
                                   "\n");

  options.add_options()("help,h", "Print this help message.")("version,v",
                                                              "Print the version of this software.")(
      "concurrency,j", bpo::value<unsigned int>(&num_threads),
      "Number of threads to use.")("config,c",
                                   boost::program_options::value<std::string>(&config_file_path),
                                   "Path to the json configuration file.")(
      "inline-config,i", boost::program_options::value<std::string>(&inline_config),
      "Inline json config.")("summary,s", bpo::value<bool>(&summary),
                             "Output summary information about traffic coverage for the tile set")
      // positional arguments
      ("traffic-tile-dir,t", bpo::value<std::string>(&traffic_tile_dir),
       "Location of traffic csv tiles.");

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
  std::unordered_map<GraphId, std::vector<std::string>> files_per_tile;
  for (filesystem::recursive_directory_iterator i(traffic_tile_dir), end; i != end; ++i) {
    if (i->is_regular_file()) {
      // remove any extension
      auto file_name = i->path().string();
      auto pos = file_name.rfind(filesystem::path::preferred_separator);
      file_name = file_name.substr(0, file_name.find('.', pos == std::string::npos ? 0 : pos));
      try {
        // parse it into a tile id and store the file path with it
        auto id = GraphTile::GetTileId(file_name);
        files_per_tile[id].push_back(i->path().string());
      } catch (...) {}
    }
  }
  std::vector<std::pair<GraphId, std::vector<std::string>>> traffic_tiles(files_per_tile.begin(),
                                                                          files_per_tile.end());
  std::random_shuffle(traffic_tiles.begin(), traffic_tiles.end());

  // Read the config file
  boost::property_tree::ptree pt;
  if (vm.count("inline-config")) {
    std::stringstream ss;
    ss << inline_config;
    rapidjson::read_json(ss, pt);
  } else if (vm.count("config") && filesystem::is_regular_file(config_file_path)) {
    rapidjson::read_json(config_file_path, pt);
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
  decltype(traffic_tiles)::const_iterator tile_start, tile_end = traffic_tiles.begin();
  auto tile_dir = pt.get<std::string>("mjolnir.tile_dir");
  // A place to hold the results of those threads (exceptions, stats)
  std::list<std::promise<stats>> results;
  // Atomically pass around stats info
  for (size_t i = 0; i < threads.size(); ++i) {
    // Where the range begins
    tile_start = tile_end;
    // Where the range ends
    tile_end += (i < at_ceiling ? floor + 1 : floor);
    // Make the thread
    results.emplace_back();
    threads[i].reset(
        new std::thread(update_tiles, tile_dir, tile_start, tile_end, std::ref(results.back())));
  }

  // wait for it to finish
  for (auto& thread : threads)
    thread->join();

  // collect some stats
  uint32_t constrained_count = 0, free_flow_count = 0, compressed_count = 0, updated_count = 0,
           duplicate_count = 0;
  for (auto& result : results) {
    try {
      auto thread_stats = result.get_future().get();
      constrained_count += thread_stats.constrained_count;
      free_flow_count += thread_stats.free_flow_count;
      compressed_count += thread_stats.compressed_count;
      updated_count += thread_stats.updated_count;
      duplicate_count += thread_stats.dup_count;
    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }

  LOG_INFO("Parsed " + std::to_string(constrained_count) + " constrained traffic speeds.");
  LOG_INFO("Parsed " + std::to_string(free_flow_count) + " free flow traffic speeds.");
  LOG_INFO("Parsed " + std::to_string(compressed_count) + " compressed records.");
  LOG_INFO("Updated " + std::to_string(updated_count) + " directed edges.");
  LOG_INFO("Duplicate count " + std::to_string(duplicate_count) + ".");
  LOG_INFO("Finished");

  if (!summary)
    return EXIT_SUCCESS;

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

      if (reader.OverCommitted()) {
        reader.Trim();
      }

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
        if (de->has_predicted_speed()) {
          pred_road_class_edges[rc]++;
        }

        // Presence of free flow and/or constrained flow speeds
        if (de->free_flow_speed() > 0 || de->constrained_flow_speed() > 0) {
          ff_road_class_edges[rc]++;
        }

        if (de->has_predicted_speed() && de->free_flow_speed() == 0 &&
            de->constrained_flow_speed() == 0) {
          LOG_WARN("Edge has predicted speed but no ff or constrained speed");
        }
      }
    }
  }
  LOG_INFO("Stats - excluding shortcut edges");
  LOG_INFO("non driveable with speed = " + std::to_string(non_dr_with_speed));
  LOG_INFO("Shortcuts with speed = " + std::to_string(shortcuts_with_speed));
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
  LOG_INFO("total driveable = " + std::to_string(totaldriveable) +
           " total driveable ramps/links = " + std::to_string(totaldriveablelink));
  LOG_INFO("total driveable non ramps/links = " +
           std::to_string(totaldriveable - totaldriveablelink));
  LOG_INFO("total pred " + std::to_string(totalpt));
  LOG_INFO("total ff " + std::to_string(totalff));

  return EXIT_SUCCESS;
}
