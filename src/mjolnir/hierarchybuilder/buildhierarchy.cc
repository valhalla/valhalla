#include <string>

#include "hierarchybuilder.h"
#include "config.h"

#include <ostream>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace bpo = boost::program_options;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;

boost::filesystem::path config_file_path;
std::string input_file;

bool ParseArguments(int argc, char *argv[]) {

  bpo::options_description options(
    "hierarchybuilder " VERSION "\n"
    "\n"
    " Usage: hierarchybuilder [options]\n"
    "\n"
    "hierarchybuilder is a program that creates hierarchy levels using the "
    "initial/base level. Sample JSON config files for specifying tile "
    "hierarchy are located in ./import/configs directory."
    "\n"
    "\n");

  options.add_options()
      ("help,h", "Print this help message.")
      ("version,v", "Print the version of this software.")
      ("config,c",
        boost::program_options::value<boost::filesystem::path>(&config_file_path)->required(),
        "Path to the json configuration file.")
      // positional arguments
      ("input_file", boost::program_options::value<std::string>(&input_file));

  bpo::positional_options_description pos_options;
  pos_options.add("input_file", 1);

  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(), vm);
    bpo::notify(vm);

  } catch (std::exception &e) {
    std::cerr << "Unable to parse command line options because: " << e.what()
      << "\n" << "This is a bug, please report it at " PACKAGE_BUGREPORT
      << "\n";
    return false;
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return true;
  }

  if (vm.count("version")) {
    std::cout << "hierarchybuilder " << VERSION << "\n";
    return true;
  }

  if (vm.count("config")) {
    if (boost::filesystem::is_regular_file(config_file_path))
      return true;
    else
      std::cerr << "Configuration file is required\n\n" << options << "\n\n";
  }

  return false;
}

// Entry point for hierarchy builder
int main(int argc, char** argv) {
  // Parse arguments
  if (!ParseArguments(argc, argv))
    return EXIT_FAILURE;

  // Read config
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config_file_path.c_str(), pt);

  // Builds additional hierarchies based on the config file. Connections
  // (directed edges) are formed between nodes at adjacent levels.
  HierarchyBuilder hierarchybuilder(pt);
  hierarchybuilder.Build();

  return EXIT_SUCCESS;
}


