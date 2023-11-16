#include "argparse_utils.h"
#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "mjolnir/landmarks.h"
#include <cxxopts.hpp>

int main(int argc, char** argv) {
  const auto program = filesystem::path(__FILE__).stem().string();

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
    if (!parse_common_args(program, options, result, "mjolnir.logging", true))
      return EXIT_SUCCESS;
  } catch (cxxopts::OptionException& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  auto pt = valhalla::config();

  // configure logging
  auto logging_subtree = pt.get_child_optional("mjolnir.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  if (!valhalla::mjolnir::AddLandmarks(pt.get_child("mjolnir"))) {
    return EXIT_FAILURE;
  };

  return EXIT_SUCCESS;
}
