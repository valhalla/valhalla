#include "argparse_utils.h"
#include "baldr/graphreader.h"
#include "midgard/logging.h"
#include "mjolnir/way_edges_processor.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <cstdlib>
#include <filesystem>
#include <string>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
namespace vm = valhalla::mjolnir;

// Main application to create a list wayids and directed edges belonging
// to ways that are drivable.
int main(int argc, char** argv) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  // args
  boost::property_tree::ptree config;
  bool include_length = false;
  std::string osm_filename;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_PRINT_VERSION + "\n\n"
      "a program that creates a list of edges for each auto-drivable OSM way. CSV row format is <osm_way_id, [...edge.forward, edge.edgeid(, edge.length_in_meters)]>\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("v,version", "Print the version of this software.")
      ("o,osm-file", "The OSM PBF file used to produce the tile set; required only if sort-edges is true", cxxopts::value<std::string>(osm_filename))
      ("l,include-length", "If true, will include edge lengths in meter as an additional column in the CSV; requires sort-edges to be set", cxxopts::value<bool>(include_length)->default_value("false"))
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config))
      return EXIT_SUCCESS;

  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << "\n";
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  // Create an unordered map of OSM ways Ids and their associated graph edges
  GraphReader reader(config.get_child("mjolnir"));

  std::filesystem::path file_path = {config.get<std::string>("mjolnir.tile_dir")};
  file_path.append("way_edges.txt");

  try {
    std::optional<std::unordered_map<uint64_t, std::vector<valhalla::mjolnir::way_edge>>> ways_edges;

    // Collect all way edges
    // optionally sort
    if (!osm_filename.empty()) {
      ways_edges =
          vm::collect_way_edges_sorted(osm_filename, reader, include_length, file_path.string());
    } else {
      ways_edges = vm::collect_way_edges(reader, file_path.string());
    }

    LOG_INFO("Finished with " + std::to_string(ways_edges->size()) + " ways.");

    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    LOG_ERROR(e.what());
    return EXIT_FAILURE;
  }
}
