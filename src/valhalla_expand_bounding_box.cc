#include <string>

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "filesystem.h"

namespace bpt = boost::property_tree;

int main(int argc, char** argv) {
  std::string bbox, config_file_path;
  boost::property_tree::ptree pt;

  try {
    // clang-format off
    cxxopts::Options options(
      "valhalla_expand_bounding_box",
      "valhalla_expand_bounding_box " VALHALLA_VERSION "\n\n"
      "Finds all the nodes in the bounding box and then expands \n"
      "the bounding box by the shape of the edges that leave the nodes.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>(config_file_path))
      ("i,inline-config", "Inline json config.", cxxopts::value<std::string>())
      ("b,bounding-box", "Bounding box to expand. The format is min_x,min_y,max_x,max_y. Required", cxxopts::value<std::string>(bbox));
    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      return EXIT_SUCCESS;
    }

    if (result.count("version")) {
      std::cout << "valhalla_expand_bounding_box " << VALHALLA_VERSION << "\n";
      return EXIT_SUCCESS;
    }

    // Read the config file
    if (result.count("inline-config")) {
      std::stringstream ss;
      ss << result["inline-config"].as<std::string>();
      rapidjson::read_json(ss, pt);
    } else if (result.count("config") && filesystem::is_regular_file(config_file_path)) {
      rapidjson::read_json(config_file_path, pt);
    } else {
      std::cerr << "Configuration is required\n\n" << options.help() << "\n\n";
      return EXIT_FAILURE;
    }

    // configure logging
    auto logging_subtree = pt.get_child_optional("mjolnir.logging");
    if (logging_subtree) {
      auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                                     std::unordered_map<std::string, std::string>>(
          logging_subtree.get());
      valhalla::midgard::logging::Configure(logging_config);
    }

    if (!result.count("bounding-box")) {
      std::cerr << "You must provide a bounding box to expand.\n\n";
      std::cerr << options.help() << std::endl;
      return EXIT_FAILURE;
    }
  } catch (const cxxopts::OptionException& e) {
    std::cout << "Unable to parse command line options because: " << e.what() << std::endl;
    return EXIT_FAILURE;
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
