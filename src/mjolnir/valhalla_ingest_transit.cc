#include "mjolnir/ingest_transit.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << std::string(argv[0]) << "Error" << std::endl;
    return 1;
  }

  // args and config file loading
  ptree pt;
  rapidjson::read_json(std::string(argv[1]), pt);
  pt.erase("base_url");
  pt.add("base_url", std::string(argv[2]));
  pt.erase("per_page");
  pt.add("per_page", argc > 3 ? std::string(argv[3]) : std::to_string(1000));
  if (argc > 4) {
    pt.get_child("mjolnir").erase("transit_dir");
    pt.add("mjolnir.transit_dir", std::string(argv[4]));
  }
  std::string feed;
  // go get information about what transit tiles we should be fetching
  auto transit_tiles = select_transit_tiles(pt, feed);
  // spawn threads to download all the tiles returning a list of
  // tiles that ended up having dangling stop pairs
  auto dangling_tiles = ingest_transit(pt, transit_tiles);

  // figure out which transit tiles even exist
  filesystem::recursive_directory_iterator transit_file_itr(
      pt.get<std::string>("mjolnir.transit_dir") + filesystem::path::preferred_separator +
      std::to_string(TileHierarchy::levels().back().level));
  filesystem::recursive_directory_iterator end_file_itr;
  std::unordered_set<GraphId> all_tiles;
  for (; transit_file_itr != end_file_itr; ++transit_file_itr) {
    if (filesystem::is_regular_file(transit_file_itr->path()) &&
        transit_file_itr->path().extension() == ".pbf") {
      all_tiles.emplace(GraphTile::GetTileId(transit_file_itr->path().string()));
    }
  }

  // spawn threads to connect dangling stop pairs to adjacent tiles' stops
  stitch(pt, all_tiles, dangling_tiles);

  return 0;
}
