#include "argparse_utils.h"
#include "midgard/logging.h"
#include "mjolnir/custom_attributes_builder.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  boost::property_tree::ptree config;
  std::vector<std::string> input_files;
  std::vector<std::string> tag_keys;
  std::string lua_path;
  std::string shortcut_strategy_name;
  auto shortcut_strategy = valhalla::mjolnir::ShortcutStrategy::kMean;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_PRINT_VERSION + "\n\n"
      "a program that transforms OSM way tags to custom attributes using a provided Lua script.\n"
      "Requires an existing custom attributes extract (mjolnir.custom_attributes_extract). The Lua script must define\n"
      "a function custom_attributes(tags) returning a table of attribute name -> number for both directions of the\n"
      "way, or a forward and a backward table, relative to the direction of the OSM way.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("k,tag-keys", "Comma-separated OSM tag keys, ways with none of them are skipped",
        cxxopts::value<std::vector<std::string>>(tag_keys))
      ("l,lua", "Path to the Lua script", cxxopts::value<std::string>(lua_path))
      ("s,shortcut-strategy", "How shortcut values are derived from the edges they replace: none, min, max or "
        "mean (length weighted)", cxxopts::value<std::string>(shortcut_strategy_name)->default_value("mean"))
      ("input_files", "positional arguments", cxxopts::value<std::vector<std::string>>(input_files));
    // clang-format on

    options.parse_positional({"input_files"});
    options.positional_help("OSM PBF file(s) used to build the initial tileset.");

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config))
      return EXIT_SUCCESS;

    if (input_files.empty() || tag_keys.empty() || lua_path.empty()) {
      std::cerr << "OSM PBF file(s), --tag-keys and --lua are required\n\n"
                << options.help() << std::endl;
      return EXIT_FAILURE;
    }

    using valhalla::mjolnir::ShortcutStrategy;
    if (shortcut_strategy_name == "none") {
      shortcut_strategy = ShortcutStrategy::kNone;
    } else if (shortcut_strategy_name == "min") {
      shortcut_strategy = ShortcutStrategy::kMin;
    } else if (shortcut_strategy_name == "max") {
      shortcut_strategy = ShortcutStrategy::kMax;
    } else if (shortcut_strategy_name == "mean") {
      shortcut_strategy = ShortcutStrategy::kMean;
    } else {
      std::cerr << "--shortcut-strategy must be one of none, min, max or mean\n\n"
                << options.help() << std::endl;
      return EXIT_FAILURE;
    }
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  std::ifstream lua_file(lua_path);
  if (!lua_file) {
    LOG_ERROR("Failed to open Lua script: {}", lua_path);
    return EXIT_FAILURE;
  }
  std::stringstream lua_script;
  lua_script << lua_file.rdbuf();

  try {
    valhalla::mjolnir::add_custom_attributes(config, input_files, tag_keys, lua_script.str(),
                                             shortcut_strategy);
  } catch (const std::exception& e) {
    LOG_ERROR(e.what());
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
