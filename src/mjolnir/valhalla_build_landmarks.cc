#include "argparse_utils.h"
#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "mjolnir/landmark_builder.h"
#include <cxxopts.hpp>

int main(int argc, char** argv) {
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  std::vector<std::string> input_files;
  boost::property_tree::ptree pt;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_VERSION + "\n\n"
      "valhalla_build_landmarks is a program that builds a SQLite database to store POI as landmarks from \n"
      "one or multiple osm.pbf files. The landmark db is used during graph building to facilitate navigation."
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
    if (!parse_common_args(program, options, result, pt, "mjolnir.logging"))
      return EXIT_SUCCESS;

    // input files are positional
    if (!result.count("input_files")) {
      throw cxxopts::OptionException("Input file is required\n\n" + options.help());
    }
  } catch (cxxopts::OptionException& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  // configure logging
  auto logging_subtree = pt.get_child_optional("mjolnir.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  if (!valhalla::mjolnir::BuildLandmarkFromPBF(pt.get_child("mjolnir"), input_files)) {
    return EXIT_FAILURE;
  };

  return EXIT_SUCCESS;
}
