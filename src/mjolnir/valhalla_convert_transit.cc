#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include "mjolnir/convert_transit.h"
#include "mjolnir/validatetransit.h"

#include "config.h"
#include <cxxopts.hpp>

filesystem::path config_file_path;
boost::property_tree::ptree pt;
std::vector<valhalla::mjolnir::OneStopTest> onestoptests;

bool ParseArguments(int argc, char* argv[]) {
  try {
    // clang-format off
    cxxopts::Options options(
      "valhalla_convert_transit",
      "valhalla_convert_transit " VALHALLA_VERSION "\n\n"
      "valhalla_convert_transit is a program that reads protobuf files and creates Level 3 Transit Tiles."
      "\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("target_directory", "Path to write transit tiles", cxxopts::value<std::string>())
      ("test_file", "file where tests are written", cxxopts::value<std::string>());

    options.parse_positional({"target_directory", "test_file"});
    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("version")) {
      std::cout << "convert_transit " << VALHALLA_VERSION << "\n";
      exit(0);
    }

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      exit(0);
    }

    if (result.count("config") &&
        filesystem::is_regular_file(config_file_path =
                                        filesystem::path(result["config"].as<std::string>()))) {
      rapidjson::read_json(config_file_path.string(), pt);
      return true;
    } else {
      std::cerr << "Configuration file is required\n" << options.help() << "\n\n";
    }

    if (result.count("target_directory")) {
      pt.get_child("mjolnir").erase("transit_dir");
      pt.add("mjolnir.transit_dir", result["target_directory"].as<std::string>());
    }

    if (result.count("test_file")) {
      std::string testfile;

      testfile = result["test_file"].as<std::string>();
      onestoptests = valhalla::mjolnir::ParseTestFile(testfile);
      std::sort(onestoptests.begin(), onestoptests.end());
    }

    if (result.count("target_directory")) {
      pt.get_child("mjolnir").erase("tile_dir");
      pt.add("mjolnir.tile_dir", result["target_directory"].as<std::string>());
    }

  } catch (cxxopts::OptionException& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";

    std::cerr << "Usage: " << std::string(argv[0])
              << " valhalla_config [target_directory] [test_file]" << std::endl;
    std::cerr << "Sample: " << std::string(argv[0]) << " conf/valhalla.json ./transit_tiles"
              << std::endl;
  }

  return EXIT_FAILURE;
}

int main(int argc, char** argv) {
  if (!ParseArguments(argc, argv)) {
    return EXIT_FAILURE;
  }

  LOG_INFO("Building transit network.");
  auto all_tiles = valhalla::mjolnir::convert_transit(pt);
  valhalla::mjolnir::ValidateTransit::Validate(pt, all_tiles, onestoptests);
  return 0;
}
