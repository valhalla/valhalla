#include <cxxopts.hpp>

#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include "mjolnir/convert_transit.h"
#include "mjolnir/validatetransit.h"

#include "argparse_utils.h"

int main(int argc, char** argv) {
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  boost::property_tree::ptree pt;
  std::vector<valhalla::mjolnir::OneStopTest> onestoptests;
  boost::property_tree::ptree config;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_VERSION + "\n\n"
      "a program that reads protobuf files and creates Level 3 Transit Tiles."
      "\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("j,concurrency", "Number of threads to use. Defaults to all threads.", cxxopts::value<uint32_t>())
      ("target_directory", "Path to write transit tiles", cxxopts::value<std::string>())
      ("test_file", "file where tests are written", cxxopts::value<std::string>());

    options.parse_positional({"target_directory", "test_file"});
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, config, "mjolnir.logging", true))
      return EXIT_SUCCESS;

    if (result.count("target_directory")) {
      config.get_child("mjolnir").erase("transit_dir");
      config.add("mjolnir.transit_dir", result["target_directory"].as<std::string>());
    }

    if (result.count("test_file")) {
      std::string testfile;

      testfile = result["test_file"].as<std::string>();
      onestoptests = valhalla::mjolnir::ParseTestFile(testfile);
      std::sort(onestoptests.begin(), onestoptests.end());
    }
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  auto all_tiles = valhalla::mjolnir::convert_transit(config);
  valhalla::mjolnir::ValidateTransit::Validate(config, all_tiles, onestoptests);
  return 0;
}
