#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "filesystem.h"

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <string>

#include "config.h"

namespace bpo = boost::program_options;
namespace bpt = boost::property_tree;

int main(int argc, char** argv) {
  std::string config, bbox;
  std::string inline_config;
  std::string config_file_path;

  bpo::options_description options("valhalla_expand_bounding_box " VALHALLA_VERSION "\n"
                                   "\n"
                                   " Usage: valhalla_expand_bounding_box [options]\n"
                                   "\n"
                                   "Finds all the nodes in the bounding box and then expands "
                                   "the bounding box by the shape of the edges that leave the nodes."
                                   "\n"
                                   "\n");

  auto adder = options.add_options();
  adder("help,h", "Print this help message.");
  adder("version,v", "Print the version of this software.");
  adder("config,c", bpo::value<std::string>(&config_file_path),
        "Path to the json configuration file.");
  adder("inline-config,i", bpo::value<std::string>(&inline_config), "Inline json config.");
  adder(
      "bounding-box,b", bpo::value<std::string>(&bbox),
      "Bounding box to expand. The format is lower left lng/lat and upper right lng/lat or min_x,min_y,max_x,max_y");

  bpo::positional_options_description pos_options;
  pos_options.add("bounding-box", 1);
  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(),
               vm);
    bpo::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return EXIT_SUCCESS;
  }

  if (vm.count("version")) {
    std::cout << "valhalla_expand_bounding_box " << VALHALLA_VERSION << "\n";
    return EXIT_SUCCESS;
  }

  if (!vm.count("bounding-box")) {
    std::cout << "You must provide a bounding box to expand.\n";
    return EXIT_FAILURE;
  }

  // Read the config file
  boost::property_tree::ptree pt;
  if (vm.count("inline-config")) {
    std::stringstream ss;
    ss << inline_config;
    rapidjson::read_json(ss, pt);
  } else if (vm.count("config") && filesystem::exists(config_file_path)) {
    rapidjson::read_json(config_file_path, pt);
  } else {
    std::cerr << "Configuration is required\n\n" << options << "\n\n";
    return EXIT_FAILURE;
  }

  // configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree =
      pt.get_child_optional("mjolnir.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  std::stringstream ss(bbox);
  std::vector<float> result;

  while (ss.good()) {
    std::string substr;
    getline(ss, substr, ',');
    result.push_back(std::stof(substr));
  }

  if (result.size() != 4) {
    std::cout << "You must provide a valid bounding box to expand.\n";
    return EXIT_FAILURE;
  }

  valhalla::midgard::AABB2<valhalla::midgard::PointLL> bb{{result[0], result[1]},
                                                          {result[2], result[3]}};
  valhalla::baldr::GraphReader reader(pt.get_child("mjolnir"));
  bb = reader.GetMinimumBoundingBox(bb);

  std::cout << std::fixed << std::setprecision(6) << bb.minx() << "," << bb.miny() << "," << bb.maxx()
            << "," << bb.maxy() << std::endl;

  return EXIT_SUCCESS;
}
