#include "argparse_utils.h"
#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "mjolnir/landmarks.h"
#include <cxxopts.hpp>

int main(int argc, char** argv) {
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  boost::property_tree::ptree config;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_VERSION + "\n\n"
      "valhalla_add_landmarks is a program that adds landmarks to existing graph tiles via a SQLite"
      " database containing landmark POIs.\n"
      "\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("j,concurrency", "Number of threads to use when processing the data.", cxxopts::value<unsigned int>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>());
    // clang-format on

    options.parse_positional({"input_files"});
    options.positional_help("OSM PBF file(s)");
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

  if (!valhalla::mjolnir::AddLandmarks(config.get_child("mjolnir"))) {
    return EXIT_FAILURE;
  };

  return EXIT_SUCCESS;
}
