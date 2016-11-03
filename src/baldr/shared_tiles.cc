#include "baldr/shared_tiles.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <boost/filesystem.hpp>

#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace {

}

namespace valhalla {
namespace baldr {

/**
 * Constructor given the property tree and the combined tile filename.
 */
SharedTiles::SharedTiles(const boost::property_tree::ptree& pt) {
  std::string tile_dir = pt.get<std::string>("tile_dir");
  TileHierarchy hierarchy(tile_dir);

  std::string shared_tile_file = pt.get<std::string>("combined_tile_file", "");
  if (shared_tile_file.empty()) {
    tile_ptr_ = nullptr;
    return;
  }

  // Open to the end of the file so we can immediately get size;
  size_t filesize = 0;
  std::string file_location = tile_dir + "/" + shared_tile_file;
  std::ifstream file(file_location, std::ios::in | std::ios::binary | std::ios::ate);
  if (file.is_open()) {
    filesize = file.tellg();
    file.close();
  }

  if (filesize == 0) {
    LOG_INFO("Could not find shared file!");
    max_level_ = -1;
    tile_ptr_  = nullptr;
  } else {
    LOG_INFO("Memory mapping the tiles!");

    // memory map the file
    tiles_.map(file_location, filesize);
    tile_ptr_ = tiles_.get();

    // Iterate through the hierarchy and set the indexes and tile counts
    char* ptr = tile_ptr_;
    uint32_t tile_count;
    for (const auto& level : hierarchy.levels()) {
      // Set the index and size arrays
      tile_count = level.second.tiles.TileCount();
      indexes_[level.first] = reinterpret_cast<uint64_t*>(ptr);
      ptr += tile_count * sizeof(uint64_t);
      sizes_[level.first] = reinterpret_cast<uint32_t*>(ptr);
      ptr += tile_count * sizeof(uint32_t);
      tile_count_[level.first] = tile_count;
    }

    // Add the transit indexes and sizes
    uint32_t level_id = hierarchy.levels().rbegin()->second.level + 1;
    tile_count = hierarchy.levels().rbegin()->second.tiles.TileCount();
    indexes_[level_id] = reinterpret_cast<uint64_t*>(ptr);
    ptr += tile_count * sizeof(uint64_t);
    sizes_[level_id] = reinterpret_cast<uint32_t*>(ptr);
    ptr += tile_count * sizeof(uint32_t);
    tile_count_[level_id] = tile_count;

    // Set the max level to transit level
    max_level_ = level_id;
  }
}

/**
 * Gets a pointer to the tile data (nullptr if not available)
 */
char* SharedTiles::get_tile_ptr() const {
  return tile_ptr_;
}

/**
 * Get a pointer to the beginning of the tile within the mmap'd file. Also
 * returns the tile size.
 * @param   graphid  Tile Id.
 * @return  Returns a pair with the pointer to the tile as the first element
 *          and the size of the tile as the second.
 */
tile_pair SharedTiles::GetTile(const GraphId& graphid) const {
  // Make sure level and tile Id are valid
  if (graphid.level() > max_level_ ||
      graphid.tileid() > tile_count_[graphid.level()]) {
    return { nullptr, 0 };
  } else {
    size_t idx = indexes_[graphid.level()][graphid.tileid()];
    if (idx == 0) {
      return { nullptr, 0 };
    } else {
      return { tile_ptr_ + idx, sizes_[graphid.level()][graphid.tileid()] };
    }
  }
}


}
}
