#include "baldr/rapidjson_utils.h"
#include "mjolnir/ingest_transit.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << std::string(argv[0]) << " Error" << std::endl;
    return 1;
  }

  // args and config file loading
  boost::property_tree::ptree pt;
  rapidjson::read_json(std::string(argv[1]), pt);

  // spawn threads to download all the tiles returning a list of
  // tiles that ended up having dangling stop pairs
  auto dangling_tiles = valhalla::mjolnir::ingest_transit(pt);

  // spawn threads to connect dangling stop pairs to adjacent tiles' stops
  valhalla::mjolnir::stitch_transit(pt, dangling_tiles);

  return 0;
}
