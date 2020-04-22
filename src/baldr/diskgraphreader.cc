#include "baldr/diskgraphreader.h"

#include "baldr/tilecache.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>

#include "midgard/encoded.h"
#include "midgard/logging.h"

#include "baldr/connectivity_map.h"
#include "baldr/curl_tilegetter.h"
#include "filesystem.h"

using namespace valhalla::midgard;

namespace {
constexpr size_t AVERAGE_TILE_SIZE = 2097152;         // 2 megs
constexpr size_t AVERAGE_MM_TILE_SIZE = 1024;         // 1k
} // namespace

namespace valhalla {
namespace baldr {

DiskGraphReader::tile_extract_t::tile_extract_t(const boost::property_tree::ptree& pt) {
  // if you really meant to load it
  if (pt.get_optional<std::string>("tile_extract")) {
    try {
      // load the tar
      archive.reset(new midgard::tar(pt.get<std::string>("tile_extract")));
      // map files to graph ids
      for (auto& c : archive->contents) {
        try {
          auto id = GraphTile::GetTileId(c.first);
          tiles[id] = std::make_pair(const_cast<char*>(c.second.first), c.second.second);
        } catch (...) {
          // It's possible to put non-tile files inside the tarfile.  As we're only
          // parsing the file *name* as a GraphId here, we will just silently skip
          // any file paths that can't be parsed by GraphId::GetTileId()
          // If we end up with *no* recognizable tile files in the tarball at all,
          // checks lower down will warn on that.
        }
      }
      // couldn't load it
      if (tiles.empty()) {
        LOG_WARN("Tile extract contained no usuable tiles");
      } // loaded ok but with possibly bad blocks
      else {
        LOG_INFO("Tile extract successfully loaded with tile count: " + std::to_string(tiles.size()));
        if (archive->corrupt_blocks) {
          LOG_WARN("Tile extract had " + std::to_string(archive->corrupt_blocks) + " corrupt blocks");
        }
      }
    } catch (const std::exception& e) {
      LOG_ERROR(e.what());
      LOG_WARN("Tile extract could not be loaded");
    }
  }

  if (pt.get_optional<std::string>("traffic_extract")) {
    try {
      // load the tar
      traffic_archive.reset(new midgard::tar(pt.get<std::string>("traffic_extract")));
      // map files to graph ids
      for (auto& c : traffic_archive->contents) {
        try {
          auto id = GraphTile::GetTileId(c.first);
          traffic_tiles[id] = std::make_pair(const_cast<char*>(c.second.first), c.second.second);
        } catch (...) {
          // It's possible to put non-tile files inside the tarfile.  As we're only
          // parsing the file *name* as a GraphId here, we will just silently skip
          // any file paths that can't be parsed by GraphId::GetTileId()
          // If we end up with *no* recognizable tile files in the tarball at all,
          // checks lower down will warn on that.
        }
      }
      // couldn't load it
      if (traffic_tiles.empty()) {
        LOG_WARN("Traffic tile extract contained no usuable tiles");
      } // loaded ok but with possibly bad blocks
      else {
        LOG_INFO("Traffic tile extract successfully loaded with tile count: " +
                 std::to_string(traffic_tiles.size()));
        if (traffic_archive->corrupt_blocks) {
          LOG_WARN("Traffic tile extract had " + std::to_string(traffic_archive->corrupt_blocks) +
                   " corrupt blocks");
        }
      }
    } catch (const std::exception& e) {
      LOG_WARN(e.what());
      LOG_WARN("Traffic tile extract could not be loaded");
    }
  }
}

std::shared_ptr<const DiskGraphReader::tile_extract_t>
DiskGraphReader::get_extract_instance(const boost::property_tree::ptree& pt) {
  static std::shared_ptr<const DiskGraphReader::tile_extract_t> tile_extract(
      new DiskGraphReader::tile_extract_t(pt));
  return tile_extract;
}

// Constructor using separate tile files
DiskGraphReader::DiskGraphReader(const boost::property_tree::ptree& pt,
                                           std::unique_ptr<tile_getter_t>&& tile_getter)
    : tile_extract_(get_extract_instance(pt)), tile_dir_(pt.get<std::string>("tile_dir", "")),
      tile_getter_(std::move(tile_getter)),
      max_concurrent_users_(pt.get<size_t>("max_concurrent_reader_users", 1)),
      tile_url_(pt.get<std::string>("tile_url", "")), cache_(TileCacheFactory::createTileCache(pt)) {
  if (!tile_getter_ && !tile_url_.empty()) {
    tile_getter_ = std::make_unique<curl_tile_getter_t>(max_concurrent_users_,
                                                        pt.get<std::string>("user_agent", ""),
                                                        pt.get<bool>("tile_url_gz", false));
  }

  // validate tile url
  if (!tile_url_.empty() && tile_url_.find(GraphTile::kTilePathPattern) == std::string::npos)
    throw std::runtime_error("Not found tilePath pattern in tile url");
  // Reserve cache (based on whether using individual tile files or shared,
  // mmap'd file
  cache_->Reserve(tile_extract_->tiles.empty() ? AVERAGE_TILE_SIZE : AVERAGE_MM_TILE_SIZE);
}

DiskGraphReader::~DiskGraphReader() = default;

// Method to test if tile exists
bool DiskGraphReader::DoesTileExist(const GraphId& graphid) const {
  if (!graphid.Is_Valid() || graphid.level() > TileHierarchy::get_max_level()) {
    return false;
  }
  // if you are using an extract only check that
  if (!tile_extract_->tiles.empty()) {
    return tile_extract_->tiles.find(graphid) != tile_extract_->tiles.cend();
  }
  // otherwise check memory or disk
  if (cache_->Contains(graphid)) {
    return true;
  }
  if (tile_dir_.empty())
    return false;
  std::string file_location =
      tile_dir_ + filesystem::path::preferred_separator + GraphTile::FileSuffix(graphid.Tile_Base());
  struct stat buffer;
  return stat(file_location.c_str(), &buffer) == 0 ||
         stat((file_location + ".gz").c_str(), &buffer) == 0;
}

bool DiskGraphReader::DoesTileExist(const boost::property_tree::ptree& pt, const GraphId& graphid) {
  if (!graphid.Is_Valid() || graphid.level() > TileHierarchy::get_max_level()) {
    return false;
  }
  // if you are using an extract only check that
  auto extract = get_extract_instance(pt);
  if (!extract->tiles.empty()) {
    return extract->tiles.find(graphid) != extract->tiles.cend();
  }
  // otherwise check the disk
  std::string file_location = pt.get<std::string>("tile_dir") +
                              filesystem::path::preferred_separator +
                              GraphTile::FileSuffix(graphid.Tile_Base());
  struct stat buffer;
  return stat(file_location.c_str(), &buffer) == 0 ||
         stat((file_location + ".gz").c_str(), &buffer) == 0;
}

// Get a pointer to a graph tile object given a GraphId. Return nullptr
// if the tile is not found/empty
const GraphTile* DiskGraphReader::GetGraphTile(const GraphId& graphid) {
  // TODO: clear the cache automatically once we become overcommitted by a certain amount

  // Return nullptr if not a valid tile
  if (!graphid.Is_Valid()) {
    return nullptr;
  }

  // Check if the level/tileid combination is in the cache
  auto base = graphid.Tile_Base();
  if (auto cached = cache_->Get(base)) {
    // LOG_DEBUG("Memory cache hit " + GraphTile::FileSuffix(base));
    return cached;
  }

  // Try getting it from the memmapped tar extract
  if (!tile_extract_->tiles.empty()) {
    // Do we have this tile
    auto t = tile_extract_->tiles.find(base);
    if (t == tile_extract_->tiles.cend()) {
      // LOG_DEBUG("Memory map cache miss " + GraphTile::FileSuffix(base));
      return nullptr;
    }

    auto traffic_ptr = tile_extract_->traffic_tiles.find(base);

    // This initializes the tile from mmap
    GraphTile tile(base, t->second.first, t->second.second,
                   traffic_ptr != tile_extract_->traffic_tiles.end() ? traffic_ptr->second.first
                                                                     : nullptr);
    if (!tile.header()) {
      // LOG_DEBUG("Memory map cache miss " + GraphTile::FileSuffix(base));
      return nullptr;
    }
    // LOG_DEBUG("Memory map cache hit " + GraphTile::FileSuffix(base));

    // Keep a copy in the cache and return it
    size_t size = AVERAGE_MM_TILE_SIZE; // tile.end_offset();  // TODO what size??
    auto inserted = cache_->Put(base, tile, size);
    return inserted;
  } // Try getting it from flat file
  else {
    auto traffic_ptr = tile_extract_->traffic_tiles.find(base);
    // Try to get it from disk and if we cant..
    GraphTile tile(tile_dir_, base,
                   traffic_ptr != tile_extract_->traffic_tiles.end() ? traffic_ptr->second.first
                                                                     : nullptr);
    if (!tile.header()) {
      if (!tile_getter_) {
        return nullptr;
      }

      {
        std::lock_guard<std::mutex> lock(_404s_lock);
        if (_404s.find(base) != _404s.end()) {
          // LOG_DEBUG("Url cache miss " + GraphTile::FileSuffix(base));
          return nullptr;
        }
      }

      // Get it from the url and cache it to disk if you can
      tile = GraphTile::CacheTileURL(tile_url_, base, tile_getter_.get(), tile_dir_);
      if (!tile.header()) {
        std::lock_guard<std::mutex> lock(_404s_lock);
        _404s.insert(base);
        // LOG_DEBUG("Url cache miss " + GraphTile::FileSuffix(base));
        return nullptr;
      }
      // LOG_DEBUG("Url cache hit " + GraphTile::FileSuffix(base));
    } else {
      // LOG_DEBUG("Disk cache hit " + GraphTile::FileSuffix(base));
    }

    // Keep a copy in the cache and return it
    size_t size = tile.header()->end_offset();
    auto inserted = cache_->Put(base, tile, size);
    return inserted;
  }
}

void DiskGraphReader::Clear() {
  cache_->Clear();
}

void DiskGraphReader::Trim() {
  cache_->Trim();
}

bool DiskGraphReader::OverCommitted() const {
  return cache_->OverCommitted();
}

// Note: this will grab all road tiles and transit tiles.
std::unordered_set<GraphId> DiskGraphReader::GetTileSet() const {
  // either mmap'd tiles
  std::unordered_set<GraphId> tiles;
  if (tile_extract_->tiles.size()) {
    for (const auto& t : tile_extract_->tiles) {
      tiles.emplace(t.first);
    }
  } // or individually on disk
  else if (!tile_dir_.empty()) {
    // for each level
    for (uint8_t level = 0; level <= TileHierarchy::levels().rbegin()->first + 1; ++level) {
      // crack open this level of tiles directory
      filesystem::path root_dir(tile_dir_ + filesystem::path::preferred_separator +
                                std::to_string(level) + filesystem::path::preferred_separator);
      if (filesystem::exists(root_dir) && filesystem::is_directory(root_dir)) {
        // iterate over all the files in there
        for (filesystem::recursive_directory_iterator i(root_dir), end; i != end; ++i) {
          if (i->is_regular_file() || i->is_symlink()) {
            // add it if it can be parsed as a valid tile file name
            try {
              tiles.emplace(GraphTile::GetTileId(i->path().string()));
            } catch (...) {}
          }
        }
      }
    }
  }

  // give them back
  return tiles;
}

// Get the set of tiles for a specified level
std::unordered_set<GraphId> DiskGraphReader::GetTileSet(const uint8_t level) const {
  // either mmap'd tiles
  std::unordered_set<GraphId> tiles;
  if (tile_extract_->tiles.size()) {
    for (const auto& t : tile_extract_->tiles) {
      if (static_cast<GraphId>(t.first).level() == level) {
        tiles.emplace(t.first);
      }
    } // or individually on disk
  } else if (!tile_dir_.empty()) {
    // crack open this level of tiles directory
    filesystem::path root_dir(tile_dir_ + filesystem::path::preferred_separator +
                              std::to_string(level) + filesystem::path::preferred_separator);
    if (filesystem::exists(root_dir) && filesystem::is_directory(root_dir)) {
      // iterate over all the files in the directory and turn into GraphIds
      for (filesystem::recursive_directory_iterator i(root_dir), end; i != end; ++i) {
        if (i->is_regular_file() || i->is_symlink()) {
          // add it if it can be parsed as a valid tile file name
          try {
            tiles.emplace(GraphTile::GetTileId(i->path().string()));
          } catch (...) {}
        }
      }
    }
  }
  return tiles;
}

} // namespace baldr
} // namespace valhalla
