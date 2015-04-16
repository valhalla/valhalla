#include <iostream>
#include <string>

#include <valhalla/baldr/srtmtile.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

/**
 * Main method of the swap SRTM conversion program
 * @return  Returns 0 if successful, -1 on error
 */
int main(int argc, char **argv) {
  // Check for correct usage
  if (argc != 4) {
    std::cout << "Usage: swapsrtm <srtm_directory> <lat> <lng>" << std::endl;
    return -1;
  }

  // Set the input and output directories and the filename
  std::string srtm_directory = argv[1];
  PointLL ll(std::atof(argv[3]), std::atof(argv[2]));

  // Get the base lat,lng
  int32_t baselat = (ll.lat() >= 0.0f) ?
      static_cast<int32_t>(ll.lat()) :
      static_cast<int32_t>(ll.lat()) - 1;
  int32_t baselng = (ll.lng() >= 0.0f) ?
      static_cast<int32_t>(ll.lng()) :
      static_cast<int32_t>(ll.lng()) - 1;
  SRTMTile tile(srtm_directory, baselat, baselng);

  if (!tile.loaded()) {
    std::cout << "Could not load SRTM file" << std::endl;
    return -1;
  }
  float h = tile.height(ll, true);
  std::cout << "Height: " << h << " meters, " << (h * 3.28084) << " feet" << std::endl;
  return 0;
}
