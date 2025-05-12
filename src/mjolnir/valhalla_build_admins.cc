#include "filesystem.h"

#include <cxxopts.hpp>

#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "mjolnir/adminbuilder.h"

#include "argparse_utils.h"

int main(int argc, char** argv) {
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  std::vector<std::string> input_files;
  boost::property_tree::ptree config;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_VERSION + "\n\n"
      "valhalla_build_admins is a program that creates a administrative SQLite database from \n"
      "one or multiple osm.pbf files. The admin db is used during graph building to enrich \n"
      "nodes and edges."
      "\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("input_files", "positional arguments", cxxopts::value<std::vector<std::string>>(input_files));
    // clang-format on

    options.parse_positional({"input_files"});
    options.positional_help("OSM PBF file(s)");
    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, config, "mjolnir.logging"))
      return EXIT_SUCCESS;

    // input files are positional
    if (!result.count("input_files")) {
      throw cxxopts::exceptions::exception("Input file is required\n\n" + options.help());
    }
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  if (!valhalla::mjolnir::BuildAdminFromPBF(config.get_child("mjolnir"), input_files)) {
    return EXIT_FAILURE;
  };

  return EXIT_SUCCESS;
}
