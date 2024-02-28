#include <string>

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "filesystem.h"

#include "argparse_utils.h"

namespace bpt = boost::property_tree;

int main(int argc, char** argv) {
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  std::string bbox;
  boost::property_tree::ptree config;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_VERSION + "\n\n"
      "Finds all the nodes in the bounding box and then expands \n"
      "the bounding box by the shape of the edges that leave the nodes.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline json config.", cxxopts::value<std::string>())
      ("b,bounding-box", "Bounding box to expand. The format is min_x,min_y,max_x,max_y. Required", cxxopts::value<std::string>(bbox));
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, config, "mjolnir.logging"))
      return EXIT_SUCCESS;

    if (!result.count("bounding-box")) {
      std::cerr << "You must provide a bounding box to expand.\n\n";
      std::cerr << options.help() << std::endl;
      return EXIT_FAILURE;
    }
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
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
  valhalla::baldr::GraphReader reader(config.get_child("mjolnir"));
  bb = reader.GetMinimumBoundingBox(bb);

  std::cout << std::fixed << std::setprecision(6) << bb.minx() << "," << bb.miny() << "," << bb.maxx()
            << "," << bb.maxy() << std::endl;

  return EXIT_SUCCESS;
}
