#include <ostream>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include "baldr/graphid.h"
#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "filesystem.h"
#include "midgard/aabb2.h"
#include "midgard/logging.h"
#include "midgard/point2.h"
#include "midgard/polyline2.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/validatetransit.h"

using namespace valhalla::mjolnir;

filesystem::path config_file_path;

bool ParseArguments(int argc, char* argv[]) {
  try {
    // clang-format off
    cxxopts::Options options(
        "valhalla_validate_transit",
        "valhalla_validate_transit " VALHALLA_VERSION "\n\n"
        "valhalla_validate_transit is a program that validates the transit graph and \n"
        "schedule at a particular time.  It will not use the route tiles at all. It \n"
        "will only use the transit tiles.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      exit(0);
    }

    if (result.count("version")) {
      std::cout << "valhalla_validate_transit " << VALHALLA_VERSION << "\n";
      exit(0);
    }

    if (result.count("config") &&
        filesystem::is_regular_file(config_file_path =
                                        filesystem::path(result["config"].as<std::string>()))) {
      return true;
    } else {
      std::cerr << "Configuration file is required\n\n" << options.help() << "\n\n";
    }

    return false;
  } catch (const cxxopts::OptionException& e) {
    std::cout << "Unable to parse command line options because: " << e.what() << std::endl;
  }

  return EXIT_FAILURE;
}

int main(int argc, char** argv) {

  if (!ParseArguments(argc, argv)) {
    return EXIT_FAILURE;
  }

  // check what type of input we are getting
  boost::property_tree::ptree pt;
  rapidjson::read_json(config_file_path.string(), pt);

  // configure logging
  auto logging_subtree = pt.get_child_optional("mjolnir.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  std::string testfile, build_validate;
  std::vector<OneStopTest> onestoptests;

  if (argc > 3) {
    build_validate = std::string(argv[3]);
  }

  if (argc > 4) {
    // do we validate the transit or build the test.
    if (build_validate == "validate") {
      testfile = std::string(std::string(argv[4]));
      onestoptests = ParseTestFile(testfile);
      std::sort(onestoptests.begin(), onestoptests.end());
      // Validate transit
      std::unordered_set<valhalla::baldr::GraphId> all_tiles;
      if (!ValidateTransit::Validate(pt, all_tiles, onestoptests)) {
        return EXIT_FAILURE;
      }
    } else if (build_validate == "build") {
      // test file is usually the results of running transit_prod_routes.tmpl tests
      testfile = std::string(std::string(argv[4]));
      ParseLogFile(testfile);
    }
  }

  return EXIT_SUCCESS;
}
