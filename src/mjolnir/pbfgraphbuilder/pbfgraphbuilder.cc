#include <string>

#include "pbfgraphbuilder.h"
#include "graphbuilder.h"
#include "config.h"

// Use open source PBF reader from:
//     https://github.com/CanalTP/libosmpbfreader
#include "osmpbfreader.h"
using namespace CanalTP;
// For OSM pbf reader
using namespace valhalla::mjolnir;

#include <ostream>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

#include "geo/point2.h"
#include "geo/aabb2.h"
#include "geo/polyline2.h"

namespace bpo = boost::program_options;
using namespace valhalla::geo;

boost::filesystem::path config_file_path;
std::string pbffiletoread, nodetagtransformscript, nodetagtransformfunction;
std::string waytagtransformscript, waytagtransformfunction;
std::string host, username, password, graphoutputdir;
unsigned int port;
bool ispbf = true;

bool ParseArguments(int argc, char *argv[]) {

  bpo::options_description options(
      "pbfgraphbuilder " VERSION "\n"
      "\n"
      " Usage: pbfgraphbuilder [options]\n"
      "\n"
      "pbfgraphbuilder is a program that creates the route graph from a osm.pbf "
      "extract or osm2pgsql import.  You should use the lua scripts provided for "
      "either method.  The scripts are located in the ./import/osm2pgsql directory.  "
      "Moreover, sample json cofigs are located in ./import/configs directory."
      "\n"
      "\n");

  std::string echo;

  options.add_options()("help,h", "Print this help message.")(
      "version,v", "Print the version of this software.")
      // positional arguments
      ("config,c",
          boost::program_options::value<boost::filesystem::path>(&config_file_path)
          ->default_value("./import/configs/pbf2graph.json"),
          "Path to the json configuration file.");

  bpo::positional_options_description pos_options;
  pos_options.add("config", 1);

  bpo::variables_map vm;

  try {
    bpo::store(
        bpo::command_line_parser(argc, argv).options(options).positional(
            pos_options).run(),
            vm);
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
    if (boost::filesystem::is_regular_file(config_file_path)) {
      std::cout << "config file: " << config_file_path << "\n";

      return true;
    }
  }

  return false;
}

bool ParseConfigFile() {

  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config_file_path.c_str(), pt);

  BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("graphbuilder")) {

    //std::cout << "key: " << v.first << " value: " << v.second.get_value<std::string>() << std::endl;

    if (v.first == "importtype") {
      if (v.second.get_value<std::string>() == "db")
        ispbf = false;

      continue;
    }

    if (v.first == "graphoutputdir") {
      graphoutputdir = v.second.get_value<std::string>();
      continue;
    }

    if (ispbf) {

      if (v.first == "pbffiletoread")
        pbffiletoread = v.second.get_value<std::string>();
      else if (v.first == "nodetagtransformscript")
        nodetagtransformscript = v.second.get_value<std::string>();
      else if (v.first == "nodetagtransformfunction")
        nodetagtransformfunction = v.second.get_value<std::string>();
      else if (v.first == "waytagtransformscript")
        waytagtransformscript = v.second.get_value<std::string>();
      else if (v.first == "waytagtransformfunction")
        waytagtransformfunction = v.second.get_value<std::string>();
      else
        return false;  //unknown value;

    } else {

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
    }
  }

  return true;
}

int main(int argc, char** argv) {

  if (!ParseArguments(argc, argv))
    return EXIT_FAILURE;

  if (!ParseConfigFile()) {
    std::cout << "Parsing of config file failed!: " << config_file_path << "\n";

    return EXIT_FAILURE;
  }

  if (ispbf) {  //TODO db logic.

    // Read the OSM protocol buffer file. Callbacks for nodes, ways, and
    // relations are defined within the GraphConstructor class

    GraphBuilder graphbuilder;

    graphbuilder.LuaInit(nodetagtransformscript, nodetagtransformfunction,
                         waytagtransformscript, waytagtransformfunction);  //TODO db logic.

    read_osm_pbf(pbffiletoread, graphbuilder);
    graphbuilder.PrintCounts();

    // Compute node use counts
    graphbuilder.SetNodeUses();

    // Construct edges
    graphbuilder.ConstructEdges();

    // Remove unused node (maybe this can recover memory?)
    graphbuilder.RemoveUnusedNodes();

    // Tile the nodes
    unsigned int level = 2;    // Local hierarchy - TODO - configurable
    float tilesize = 0.25f;    // TODO - configurable
    graphbuilder.TileNodes(tilesize, level);

    // TODO - make bounding box and tilesize configurable (via properties file?)
    // Iterate through edges - tile the end nodes to create connected graph
    graphbuilder.BuildLocalTiles(graphoutputdir, tilesize);
  }

  return EXIT_SUCCESS;
}

