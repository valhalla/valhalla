#include "filesystem.h"

#include <cxxopts.hpp>

#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "mjolnir/adminbuilder.h"

filesystem::path config_file_path;
std::vector<std::string> input_files;

bool ParseArguments(int argc, char* argv[]) {
  try {
    // clang-format off
    cxxopts::Options options(
      "valhalla_build_admins",
      "valhalla_build_admins " VALHALLA_VERSION "\n\n"
      "valhalla_build_admins is a program that creates a administrative SQLite database from \n"
      "one or multiple osm.pbf files. The admin db is used during graph building to enrich \n"
      "nodes and edges."
      "\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("input_files", "positional arguments", cxxopts::value<std::vector<std::string>>(input_files));
    // clang-format on

    options.parse_positional({"input_files"});
    options.positional_help("OSM PBF file(s)");
    auto result = options.parse(argc, argv);

    if (result.count("version")) {
      std::cout << "pbfadminbuilder " << VALHALLA_VERSION << "\n";
      exit(0);
    }

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      exit(0);
    }

    // input files are positional
    if (!result.count("input_files")) {
      std::cerr << "Input file is required\n" << std::endl;
      return false;
    }

    if (result.count("config") &&
        filesystem::is_regular_file(config_file_path =
                                        filesystem::path(result["config"].as<std::string>()))) {
      return true;
    } else {
      std::cerr << "Configuration file is required\n" << options.help() << "\n\n";
    }
  } catch (cxxopts::OptionException& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
  }

  return EXIT_FAILURE;
}

int main(int argc, char** argv) {

  if (!ParseArguments(argc, argv)) {
    return EXIT_FAILURE;
  }

  // check what type of input we are getting
  boost::property_tree::ptree pt;
  rapidjson::read_json(config_file_path.string(), pt);

  // configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree =
      pt.get_child_optional("mjolnir.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  valhalla::mjolnir::BuildAdminFromPBF(pt.get_child("mjolnir"), input_files);

  return EXIT_SUCCESS;
}
