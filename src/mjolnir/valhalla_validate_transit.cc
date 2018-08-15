#include <string>
#include <vector>

#include "baldr/graphid.h"
#include "config.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/validatetransit.h"

using namespace valhalla::mjolnir;

#include "baldr/rapidjson_utils.h"
#include <boost/filesystem/operations.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ostream>

#include "midgard/aabb2.h"
#include "midgard/logging.h"
#include "midgard/point2.h"
#include "midgard/polyline2.h"

namespace bpo = boost::program_options;

boost::filesystem::path config_file_path;
std::vector<std::string> input_files;

bool ParseArguments(int argc, char* argv[]) {

  bpo::options_description options(
      "valhalla_validate_transit " VALHALLA_VERSION "\n"
      "\n"
      " Usage: valhalla_validate_transit [options] <protocolbuffer_input_file>\n"
      "\n"
      "valhalla_validate_transit is a program that validates the transit graph and "
      "schedule at a particular time.  It will not use the route tiles at all.  It "
      "will only the transit tiles."
      "\n"
      "\n");

  options.add_options()("help,h", "Print this help message.")("version,v",
                                                              "Print the version of this software.")(
      "config,c",
      boost::program_options::value<boost::filesystem::path>(&config_file_path)->required(),
      "Path to the json configuration file.")
      // positional arguments
      ("input_files",
       boost::program_options::value<std::vector<std::string>>(&input_files)->multitoken());

  bpo::positional_options_description pos_options;
  pos_options.add("input_files", 16);

  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(),
               vm);
    bpo::notify(vm);

  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return false;
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return true;
  }

  if (vm.count("version")) {
    std::cout << "valhalla_validate_transit " << VALHALLA_VERSION << "\n";
    return true;
  }

  if (vm.count("config")) {
    if (boost::filesystem::is_regular_file(config_file_path)) {
      return true;
    } else {
      std::cerr << "Configuration file is required\n\n" << options << "\n\n";
    }
  }

  return false;
}

int main(int argc, char** argv) {

  if (!ParseArguments(argc, argv)) {
    return EXIT_FAILURE;
  }

  // check what type of input we are getting
  boost::property_tree::ptree pt;
  rapidjson::read_json(config_file_path.c_str(), pt);

  // configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree =
      pt.get_child_optional("mjolnir.logging");
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
