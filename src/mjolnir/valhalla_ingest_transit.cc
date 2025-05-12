#include <cxxopts.hpp>

#include "baldr/rapidjson_utils.h"
#include "mjolnir/ingest_transit.h"

#include "argparse_utils.h"

int main(int argc, char** argv) {
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  boost::property_tree::ptree config;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_VERSION + "\n\n"
      "a program that reads GTFS data. It converts a directory of transit feeds into protobuf tiles."
      "\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("j,concurrency", "Number of threads to use. Defaults to all threads.", cxxopts::value<uint32_t>());
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, config, "mjolnir.logging", true))
      return EXIT_SUCCESS;
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  try {
    // spawn threads to download all the tiles returning a list of
    // tiles that ended up having dangling stop pairs
    auto dangling_tiles = valhalla::mjolnir::ingest_transit(config);

    // spawn threads to connect dangling stop pairs to adjacent tiles' stops
    valhalla::mjolnir::stitch_transit(config, dangling_tiles);
  } catch (const std::runtime_error& e) {
    LOG_ERROR(e.what());
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
