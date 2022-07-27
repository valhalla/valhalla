#include "mjolnir/convert_transit.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << std::string(argv[0])
              << " valhalla_config [target_directory] [test_file]" << std::endl;
    std::cerr << "Sample: " << std::string(argv[0]) << " conf/valhalla.json ./transit_tiles"
              << std::endl;
    return 1;
  }

  // args and config file loading
  ptree pt;
  rapidjson::read_json(std::string(argv[1]), pt);
  if (argc > 2) {
    pt.get_child("mjolnir").erase("transit_dir");
    pt.add("mjolnir.transit_dir", std::string(argv[2]));
  }
  std::string testfile;
  std::vector<OneStopTest> onestoptests;
  if (argc > 3) {
    testfile = std::string(argv[3]);
    onestoptests = ParseTestFile(testfile);
    std::sort(onestoptests.begin(), onestoptests.end());
  }

  // update tile dir loc.  Don't want to overwrite the real transit tiles
  if (argc > 2) {
    pt.get_child("mjolnir").erase("tile_dir");
    pt.add("mjolnir.tile_dir", std::string(argv[2]));
  }

  LOG_INFO("Building transit network.");
  build(pt);
  ValidateTransit::Validate(pt, all_tiles, onestoptests);
  return 0;
}
