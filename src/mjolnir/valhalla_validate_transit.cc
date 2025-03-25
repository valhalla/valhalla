#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include "baldr/graphid.h"
#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include "midgard/aabb2.h"
#include "midgard/logging.h"
#include "midgard/point2.h"
#include "midgard/polyline2.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/validatetransit.h"

#include "argparse_utils.h"

using namespace valhalla::mjolnir;

int main(int argc, char** argv) {
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  boost::property_tree::ptree config;

  try {
    // clang-format off
    cxxopts::Options options(
        program,
        program + " " + VALHALLA_VERSION + "\n\n"
        "a program that validates the transit graph and \n"
        "schedule at a particular time.  It will not use the route tiles at all. It \n"
        "will only use the transit tiles.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
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
      if (!ValidateTransit::Validate(config, all_tiles, onestoptests)) {
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
