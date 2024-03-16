#include <iostream>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "mjolnir/util.h"

#include "argparse_utils.h"

using namespace valhalla::mjolnir;

// List the build stages
void list_stages() {
  std::cout << "Build stage strings (in order)" << std::endl;
  for (int i = static_cast<int>(BuildStage::kInitialize); i <= static_cast<int>(BuildStage::kCleanup);
       ++i) {
    std::cout << "    " << to_string(static_cast<BuildStage>(i)) << std::endl;
  }
}

int main(int argc, char** argv) {
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  std::vector<std::string> input_files;
  BuildStage start_stage = BuildStage::kInitialize;
  BuildStage end_stage = BuildStage::kCleanup;
  boost::property_tree::ptree config;

  try {

    // ref:
    // https://github.com/jarro2783/cxxopts/blob/302302b30839505703d37fb82f536c53cf9172fa/src/example.cpp
    cxxopts::Options options(
        program,
        program + " " + VALHALLA_VERSION +
            "\n\n"
            "a program that creates the route graph\nfrom one or multiple osm.pbf extract(s)\n");

    // clang-format off
    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version","Print the version of this software.")
      ("c,config", "Path to the configuration file", cxxopts::value<std::string>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("s,start", "Starting stage of the build pipeline", cxxopts::value<std::string>()->default_value("initialize"))
      ("e,end", "End stage of the build pipeline", cxxopts::value<std::string>()->default_value("cleanup"))
      ("input_files", "positional arguments", cxxopts::value<std::vector<std::string>>(input_files))
      ("j,concurrency", "Number of threads to use. Defaults to all threads.", cxxopts::value<uint32_t>());
    // clang-format on

    options.parse_positional({"input_files"});
    options.positional_help("OSM PBF file(s)");
    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, config, "mjolnir.logging", true, &list_stages))
      return EXIT_SUCCESS;

    // Convert stage strings to BuildStage
    if (result.count("start")) {
      start_stage = string_to_buildstage(result["start"].as<std::string>());
      if (start_stage == BuildStage::kInvalid) {
        list_stages();
        throw cxxopts::exceptions::exception("Invalid start stage, see above");
      }
    }
    if (result.count("end")) {
      end_stage = string_to_buildstage(result["end"].as<std::string>());
      if (end_stage == BuildStage::kInvalid) {
        list_stages();
        throw cxxopts::exceptions::exception("Invalid end stage, see above");
      }
    }
    LOG_INFO("Start stage = " + to_string(start_stage) + " End stage = " + to_string(end_stage));

    // Make sure start stage < end stage
    if (static_cast<int>(start_stage) > static_cast<int>(end_stage)) {
      list_stages();
      throw cxxopts::exceptions::exception(
          "Starting build stage is after ending build stage in pipeline, see above");
    }

    if (!result.count("input_files") && start_stage <= BuildStage::kParseNodes &&
        end_stage >= BuildStage::kParseWays) {
      throw cxxopts::exceptions::exception("Input file is required\n\n" + options.help() + "\n\n");
    }
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  // Build some tiles!
  if (build_tile_set(config, input_files, start_stage, end_stage)) {
    return EXIT_SUCCESS;
  } else {
    return EXIT_FAILURE;
  }
}
