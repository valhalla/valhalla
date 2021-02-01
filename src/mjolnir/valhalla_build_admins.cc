#include "filesystem.h"

#include <boost/program_options.hpp>

#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "mjolnir/adminbuilder.h"

namespace bpo = boost::program_options;

filesystem::path config_file_path;
std::vector<std::string> input_files;

bool ParseArguments(int argc, char* argv[]) {

  bpo::options_description options(
      "pbfadminbuilder " VALHALLA_VERSION "\n"
      "\n"
      " Usage: pbfadminbuilder [options] <protocolbuffer_input_file>\n"
      "\n"
      "pbfadminbuilder is a program that creates the route graph from a osm.pbf "
      "extract or osm2pgsql import.  You should use the lua scripts provided for "
      "either method.  The scripts are located in the ./import/osm2pgsql directory.  "
      "Moreover, sample json configs are located in ./import/configs directory."
      "\n"
      "\n");

  options.add_options()("help,h", "Print this help message.")("version,v",
                                                              "Print the version of this software.")(
      "config,c", boost::program_options::value<filesystem::path>(&config_file_path)->required(),
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
    std::cout << "pbfadminbuilder " << VALHALLA_VERSION << "\n";
    return true;
  }

  if (vm.count("config")) {
    if (filesystem::is_regular_file(config_file_path)) {
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
  rapidjson::read_json(config_file_path.string(), pt);

  // configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree =
      pt.get_child_optional("mjolnir.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  valhalla::mjolnir::BuildAdminFromPBF(pt.get_child("mjolnir"), input_files);

  return EXIT_SUCCESS;
}
