#include <cassert>
#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "baldr/connectivity_map.h"
#include "baldr/tilehierarchy.h"
#include "config.h"
#include "filesystem.h"

using namespace valhalla::baldr;

#include "baldr/rapidjson_utils.h"
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>
#include <ostream>

using namespace valhalla::midgard;

filesystem::path config_file_path;

struct PPMObject {
  std::string magic_num;
  int32_t width, height, maxColVal;
  char* m_image;
};

bool ParseArguments(int argc, char* argv[]) {
  try {
    // clang-format off
    cxxopts::Options options(
      "valhalla_build_connectivity",
      "valhalla_build_connectivity " VALHALLA_VERSION "\n\n"
      "valhalla_build_connectivity is a program that creates a PPM image file representing\n"
      "the connectivity between tiles.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("version")) {
      std::cout << "valhalla_build_connectivity " << VALHALLA_VERSION << "\n";
      return EXIT_SUCCESS;
    }

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      return EXIT_SUCCESS;
    }

    if (result.count("config") &&
        filesystem::is_regular_file(config_file_path =
                                        filesystem::path(result["config"].as<std::string>()))) {
      return true;
    } else {
      std::cerr << "Configuration file is required\n\n" << options.help() << "\n\n";
    }
  } catch (const cxxopts::OptionException& e) {
    std::cout << "Unable to parse command line options because: " << e.what() << std::endl;
  }

  return false;
}

struct RGB {
  uint8_t red;
  uint8_t green;
  uint8_t blue;

  RGB() : red(0), green(0), blue(0) {
  }

  RGB(uint8_t r, uint8_t g, uint8_t b) : red(r), green(g), blue(b) {
  }
};

// NOTE: a PPM image can be converted to png and flipped
// using ImageMagick: e.g.,
//     convert -flip connectivity2.ppm connectivity2.png

// Main application to create a ppm image file of connectivity.
int main(int argc, char** argv) {
  // Parse command line arguments
  if (!ParseArguments(argc, argv)) {
    return EXIT_FAILURE;
  }

  // Get the config to see which coverage we are using
  boost::property_tree::ptree pt;
  rapidjson::read_json(config_file_path.string(), pt);

  // Get something we can use to fetch tiles
  valhalla::baldr::connectivity_map_t connectivity_map(pt.get_child("mjolnir"));

  uint32_t transit_level = TileHierarchy::levels().back().level + 1;
  for (uint32_t level = 0; level <= transit_level; level++) {
    if (!connectivity_map.has_data(level)) {
      continue;
    }

    // Make the vector representation of it
    std::string fname = "connectivity" + std::to_string(level) + ".geojson";
    std::ofstream geojson_file(fname, std::ios::out);
    if (!geojson_file) {
      std::cout << "Unable to open output file: " << fname << std::endl;
      return EXIT_FAILURE;
    }
    geojson_file << connectivity_map.to_geojson(level);
    geojson_file.close();

    // Make the ppm file for raster images
    std::vector<size_t> connectivity_image = connectivity_map.to_image(level);

    // Create output file
    fname = "connectivity" + std::to_string(level) + ".ppm";
    std::ofstream outfile(fname, std::ios::binary | std::ios::out);
    if (!outfile) {
      std::cout << "Unable to open output file: " << fname << std::endl;
      return EXIT_FAILURE;
    }

    uint32_t width, height;
    if (level == transit_level) {
      auto tiles = TileHierarchy::levels().back().tiles;
      width = tiles.ncolumns();
      height = tiles.nrows();
    } else {
      const auto& tiles = TileHierarchy::levels()[level].tiles;
      width = tiles.ncolumns();
      height = tiles.nrows();
    }

    // Add background color: white
    std::unordered_map<size_t, RGB> colormap;
    RGB white(255, 255, 255);
    colormap[0] = white;

    // Create an array of RGB values
    std::vector<RGB> ppm;
    for (auto c : connectivity_image) {
      auto color = colormap.find(c);
      if (color == colormap.end()) {
        // Add a random color to the colormap
        RGB newcolor(rand() % 200, rand() % 200, rand() % 200);
        ppm.push_back(newcolor);
        colormap[c] = newcolor;
      } else {
        ppm.push_back(color->second);
      }
    }
    assert(ppm.size() == height * width);

    std::string tmp;
    outfile << "P6" << std::endl; //  << “# foreground “ << std::endl;
    outfile << std::to_string(width) << " " << std::to_string(height) << std::endl;
    outfile << std::to_string(255) << std::endl;
    outfile.write(reinterpret_cast<char*>(ppm.data()), height * width * 3);
    outfile.close();
  }

  return EXIT_SUCCESS;
}
