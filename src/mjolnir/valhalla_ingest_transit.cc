#include "baldr/rapidjson_utils.h"
#include "mjolnir/ingest_transit.h"

#include "config.h"
#include <cxxopts.hpp>

filesystem::path config_file_path;

bool ParseArguments(int argc, char* argv[]) {
  try {
    // clang-format off
    cxxopts::Options options(
      "valhalla_ingest_transit",
      "valhalla_ingest_transit " VALHALLA_VERSION "\n\n"
      "valhalla_ingest_transit is a program that reads GTFS data. It converts a directory of transit feeds into protobuf tiles."
      "\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("version")) {
      std::cout << "ingest_transit " << VALHALLA_VERSION << "\n";
      exit(0);
    }

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      exit(0);
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

  // args and config file loading
  boost::property_tree::ptree pt;
  rapidjson::read_json(config_file_path.string(), pt);

  // spawn threads to download all the tiles returning a list of
  // tiles that ended up having dangling stop pairs
  auto dangling_tiles = valhalla::mjolnir::ingest_transit(pt);

  // spawn threads to connect dangling stop pairs to adjacent tiles' stops
  valhalla::mjolnir::stitch_transit(pt, dangling_tiles);

  return 0;
}
