#include <string>
#include <vector>

#include "pbfgraphbuilder.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/hierarchybuilder.h"
#include "mjolnir/graphoptimizer.h"
#include "config.h"

// For OSM pbf reader
using namespace valhalla::mjolnir;

#include <ostream>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>

#include <valhalla/midgard/point2.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/polyline2.h>
#include <valhalla/midgard/logging.h>

namespace bpo = boost::program_options;
using namespace valhalla::midgard;

boost::filesystem::path config_file_path;
std::vector<std::string> input_files;

bool ParseArguments(int argc, char *argv[]) {

  bpo::options_description options(
    "pbfgraphbuilder " VERSION "\n"
    "\n"
    " Usage: pbfgraphbuilder [options] <protocolbuffer_input_file>\n"
    "\n"
    "pbfgraphbuilder is a program that creates the route graph from a osm.pbf "
    "extract or osm2pgsql import.  You should use the lua scripts provided for "
    "either method.  The scripts are located in the ./import/osm2pgsql directory.  "
    "Moreover, sample json cofigs are located in ./import/configs directory."
    "\n"
    "\n");

  options.add_options()
      ("help,h", "Print this help message.")
      ("version,v", "Print the version of this software.")
      ("config,c",
        boost::program_options::value<boost::filesystem::path>(&config_file_path)->required(),
        "Path to the json configuration file.")
      // positional arguments
      ("input_files", boost::program_options::value<std::vector<std::string> >(&input_files)->multitoken());

  bpo::positional_options_description pos_options;
  pos_options.add("input_files", 16);

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
    std::cout << "pbfgraphbuilder " << VERSION << "\n";
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

/**
 * Build local graph from protocol buffer input.
 */
void BuildLocalGraphFromPBF(const boost::property_tree::ptree& pt,
               const std::vector<std::string>& input_files) {
  // Read the OSM protocol buffer file. Callbacks for nodes, ways, and
  // relations are defined within the GraphConstructor class
  GraphBuilder graphbuilder(pt);
  graphbuilder.Load(input_files);
  graphbuilder.Build();
}

int main(int argc, char** argv) {

  if (!ParseArguments(argc, argv))
    return EXIT_FAILURE;

  //check what type of input we are getting
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config_file_path.c_str(), pt);

  //configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree = pt.get_child_optional("logging.mjolnir");
  if(logging_subtree) {
    auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&, std::unordered_map<std::string, std::string> >(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  //we only support protobuf at present
  std::string input_type = pt.get<std::string>("input.type");
  if(input_type == "protocolbuffer"){
    BuildLocalGraphFromPBF(pt, input_files);
  }/*else if("postgres"){
    //TODO
    if (v.first == "host")
      host = v.second.get_value<std::string>();
    else if (v.first == "port")
      port = v.second.get_value<unsigned int>();
    else if (v.first == "username")
      username = v.second.get_value<std::string>();
    else if (v.first == "password")
      password = v.second.get_value<std::string>();
    else
      return false;  //unknown value;
  }*/

  // Builds additional hierarchies based on the config file. Connections
  // (directed edges) are formed between nodes at adjacent levels.
  HierarchyBuilder hierarchybuilder(pt);
  hierarchybuilder.Build();

  // Optimize the graph to add information that cannot be added until
  // full graph is formed.
  GraphOptimizer graphoptimizer(pt);
  graphoptimizer.Optimize();

  return EXIT_SUCCESS;
}

