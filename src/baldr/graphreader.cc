#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <utility>

#include "baldr/connectivity_map.h"
#include "baldr/curl_tilegetter.h"
#include "baldr/graphreader.h"
#include "filesystem.h"
#include "incident_singleton.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "shortcut_recovery.h"

using namespace valhalla::midgard;

namespace {

constexpr size_t DEFAULT_MAX_CACHE_SIZE = 1073741824; // 1 gig
constexpr size_t AVERAGE_TILE_SIZE = 2097152;         // 2 megs
constexpr size_t AVERAGE_MM_TILE_SIZE = 1024;         // 1k

struct tile_index_entry {
  uint64_t offset;  // byte offset from the beginning of the tar
  uint32_t tile_id; // just level and tileindex hence fitting in 32bits
  uint32_t size;    // size of the tile in bytes
};

} // namespace

namespace valhalla {
namespace baldr {

tile_gone_error_t::tile_gone_error_t(const std::string& errormessage)
    : std::runtime_error(errormessage) {
}

tile_gone_error_t::tile_gone_error_t(std::string prefix, baldr::GraphId edgeid)
    : std::runtime_error(std::move(prefix) + ", tile no longer available " +
                         std::to_string(edgeid.Tile_Base())) {
}

GraphReader::tile_extract_t::tile_extract_t(const boost::property_tree::ptree& pt) {
  // A lambda for loading the contents of a graph tile tar from an index file
  bool traffic_from_index = false;
  auto index_loader = [this, &traffic_from_index](const std::string& filename,
                                                  const char* index_begin, const char* file_begin,
                                                  size_t size) -> decltype(midgard::tar::contents) {
    // has to be our specially named index.bin file
    if (filename != "index.bin")
      return {};

    // get the info
    decltype(midgard::tar::contents) contents;
    auto entry_size = sizeof(tile_index_entry);
    auto entries = midgard::iterable_t<tile_index_entry>(reinterpret_cast<tile_index_entry*>(
                                                             const_cast<char*>(index_begin)),
                                                         size / sizeof(tile_index_entry));
    for (const auto& entry : entries) {
      auto inserted = contents.insert(
          std::make_pair(std::to_string(entry.tile_id),
                         std::make_pair(const_cast<char*>(file_begin + entry.offset), entry.size)));
      if (!traffic_from_index) {
        tiles.emplace(std::piecewise_construct, std::forward_as_tuple(entry.tile_id),
                      std::forward_as_tuple(const_cast<char*>(file_begin + entry.offset),
                                            entry.size));
      } else {
        traffic_tiles.emplace(std::piecewise_construct, std::forward_as_tuple(entry.tile_id),
                              std::forward_as_tuple(const_cast<char*>(file_begin + entry.offset),
                                                    entry.size));
      }
    }
    // hand it back to the tar parser
    return contents;
  };

  bool scan_tar = pt.get<bool>("data_processing.scan_tar", false);

  // if you really meant to load it
  if (pt.get_optional<std::string>("tile_extract")) {
    try {
      // load the tar
      // TODO: use the "scan" to iterate over tar
      archive.reset(new midgard::tar(pt.get<std::string>("tile_extract"), true, index_loader));
      // map files to graph ids
      if (tiles.empty()) {
        for (const auto& c : archive->contents) {
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
      } else if (scan_tar) {
        checksum = 0;
        for (const auto& kv : tiles) {
          checksum += *const_cast<char*>(kv.second.first);
        }
      }
      // couldn't load it
      if (tiles.empty()) {
        LOG_WARN("Tile extract contained no usuable tiles");
        archive.reset();
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
      traffic_from_index = true;
      traffic_archive.reset(
          new midgard::tar(pt.get<std::string>("traffic_extract"), true, index_loader));
      if (traffic_tiles.empty()) {
        LOG_WARN(
            "Traffic extract contained no index file, expect degraded performance for tile (re-)loading.");
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
      }
      // couldn't load it
      if (traffic_tiles.empty()) {
        LOG_WARN("Traffic tile extract contained no usuable tiles");
        archive.reset();
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

// ----------------------------------------------------------------------------
// FlatTileCache implementation
// ----------------------------------------------------------------------------

// Constructor.
FlatTileCache::FlatTileCache(size_t max_size) : cache_size_(0), max_cache_size_(max_size) {
  index_offsets_[0] = 0;
  index_offsets_[1] = index_offsets_[0] + TileHierarchy::levels()[0].tiles.TileCount();
  index_offsets_[2] = index_offsets_[1] + TileHierarchy::levels()[1].tiles.TileCount();
  index_offsets_[3] = index_offsets_[2] + TileHierarchy::levels()[2].tiles.TileCount();
  cache_indices_.resize(index_offsets_[3] + TileHierarchy::GetTransitLevel().tiles.TileCount(), -1);
}

// Reserves enough cache to hold (max_cache_size / tile_size) items.
void FlatTileCache::Reserve(size_t tile_size) {
  cache_.reserve(max_cache_size_ / tile_size);
}

// Checks if tile exists in the cache.
bool FlatTileCache::Contains(const GraphId& graphid) const {
  return get_index(graphid) != -1;
}

// Lets you know if the cache is too large.
bool FlatTileCache::OverCommitted() const {
  return cache_size_ > max_cache_size_;
}

// Clears the cache.
void FlatTileCache::Clear() {
  cache_size_ = 0;
  cache_.clear();
  // TODO: this could be optimized by using the remaining bits in tileid. we need to track a 7bit
  // value which is the current color of the cache. every time we clear we increment the color,
  // thereby invalidating the last rounds color. we'd also need to check the color instead of just
  // checking for the index being set to -1. when the color wraps around to 0 again we would have to
  // do the hard reassign here, though that should be quite infrequent
  cache_indices_.assign(index_offsets_[3] + TileHierarchy::GetTransitLevel().tiles.TileCount(), -1);
}

// Get a pointer to a graph tile object given a GraphId.
graph_tile_ptr FlatTileCache::Get(const GraphId& graphid) const {
  auto index = get_index(graphid);
  if (index == -1)
    return nullptr;
  return cache_[index];
}

// Puts a copy of a tile of into the cache.
graph_tile_ptr FlatTileCache::Put(const GraphId& graphid, graph_tile_ptr tile, size_t size) {
  // TODO: protect against crazy tileid?
  cache_size_ += size;
  cache_indices_[get_offset(graphid)] = cache_.size();
  cache_.emplace_back(std::move(tile));
  return cache_.back();
}

void FlatTileCache::Trim() {
  Clear();
}

// ----------------------------------------------------------------------------
// SimpleTileCache implementation
// ----------------------------------------------------------------------------

// Constructor.
SimpleTileCache::SimpleTileCache(size_t max_size) : cache_size_(0), max_cache_size_(max_size) {
}

// Reserves enough cache to hold (max_cache_size / tile_size) items.
void SimpleTileCache::Reserve(size_t tile_size) {
  cache_.reserve(max_cache_size_ / tile_size);
}

// Checks if tile exists in the cache.
bool SimpleTileCache::Contains(const GraphId& graphid) const {
  return cache_.find(graphid) != cache_.end();
}

// Lets you know if the cache is too large.
bool SimpleTileCache::OverCommitted() const {
  return cache_size_ > max_cache_size_;
}

// Clears the cache.
void SimpleTileCache::Clear() {
  cache_size_ = 0;
  cache_.clear();
}

// Get a pointer to a graph tile object given a GraphId.
graph_tile_ptr SimpleTileCache::Get(const GraphId& graphid) const {
  auto cached = cache_.find(graphid);
  if (cached != cache_.end()) {
    return cached->second;
  }
  return nullptr;
}

// Puts a copy of a tile of into the cache.
graph_tile_ptr SimpleTileCache::Put(const GraphId& graphid, graph_tile_ptr tile, size_t size) {
  cache_size_ += size;
  return cache_.emplace(graphid, std::move(tile)).first->second;
}

void SimpleTileCache::Trim() {
  Clear();
}

// ----------------------------------------------------------------------------
// TileCacheLRU implementation
// ----------------------------------------------------------------------------

// Constructor.
TileCacheLRU::TileCacheLRU(size_t max_size, MemoryLimitControl mem_control)
    : mem_control_(mem_control), cache_size_(0), max_cache_size_(max_size) {
}

void TileCacheLRU::Reserve(size_t tile_size) {
  assert(tile_size != 0);
  cache_.reserve(max_cache_size_ / tile_size);
}

bool TileCacheLRU::Contains(const GraphId& graphid) const {
  // todo: real experiments are needed to check if we need to
  // promote the entry in LRU list to the head here
  return cache_.find(graphid) != cache_.cend();
}

bool TileCacheLRU::OverCommitted() const {
  return cache_size_ > max_cache_size_;
}

void TileCacheLRU::Clear() {
  cache_size_ = 0;
  cache_.clear();
  key_val_lru_list_.clear();
}

void TileCacheLRU::Trim() {
  TrimToFit(0);
}

graph_tile_ptr TileCacheLRU::Get(const GraphId& graphid) const {
  auto cached = cache_.find(graphid);
  if (cached == cache_.cend()) {
    return nullptr;
  }

  const KeyValueIter& entry_iter = cached->second;
  MoveToLruHead(entry_iter);

  return entry_iter->tile;
}

size_t TileCacheLRU::TrimToFit(const size_t required_size) {
  size_t freed_space = 0;
  while ((OverCommitted() || (max_cache_size_ - cache_size_) < required_size) &&
         !key_val_lru_list_.empty()) {
    const KeyValue& entry_to_evict = key_val_lru_list_.back();
    const auto tile_size = entry_to_evict.tile->header()->end_offset();
    cache_size_ -= tile_size;
    freed_space += tile_size;
    cache_.erase(entry_to_evict.id);
    key_val_lru_list_.pop_back();
  }
  return freed_space;
}

void TileCacheLRU::MoveToLruHead(const KeyValueIter& entry_iter) const {
  key_val_lru_list_.splice(key_val_lru_list_.begin(), key_val_lru_list_, entry_iter);
}

graph_tile_ptr TileCacheLRU::Put(const GraphId& graphid, graph_tile_ptr tile, size_t new_tile_size) {
  if (new_tile_size > max_cache_size_) {
    throw std::runtime_error("TileCacheLRU: tile size is bigger than max cache size");
  }

  auto cached = cache_.find(graphid);
  if (cached == cache_.end()) {
    if (mem_control_ == MemoryLimitControl::HARD) {
      TrimToFit(new_tile_size);
    }
    key_val_lru_list_.emplace_front(KeyValue{graphid, std::move(tile)});
    cache_.emplace(graphid, key_val_lru_list_.begin());
  } else {
    // Value update; the new size may be different form the previous
    // TODO: in practice tile size for a specific id never changes
    //  if tile set is changed (e.g. new version), the cache should be cleared
    //  do we need to take it into account here? (can dramatically simplify the code)
    // note: SimpleTileCache does not handle the overwrite at the moment
    auto& entry_iter = cached->second;
    const auto old_tile_size = entry_iter->tile->header()->end_offset();

    // do it before TrimToFit avoid its eviction to free space
    MoveToLruHead(entry_iter);

    if (mem_control_ == MemoryLimitControl::HARD) {
      if (new_tile_size > old_tile_size) {
        const auto extra_size_required = new_tile_size - old_tile_size;
        // We do not allow insertion of items greater than the cache size
        // Thus it's not possible the current value will be evicted.
        TrimToFit(extra_size_required);
      }
    }

    entry_iter->tile = std::move(tile);
    cache_size_ -= old_tile_size;
  }
  cache_size_ += new_tile_size;

  return key_val_lru_list_.front().tile;
}

// ----------------------------------------------------------------------------
// SynchronizedTileCache implementation
// ----------------------------------------------------------------------------

// Constructor.
SynchronizedTileCache::SynchronizedTileCache(TileCache& cache, std::mutex& mutex)
    : cache_(cache), mutex_ref_(mutex) {
}

// Reserves enough cache to hold (max_cache_size / tile_size) items.
void SynchronizedTileCache::Reserve(size_t tile_size) {
  std::lock_guard<std::mutex> lock(mutex_ref_);
  cache_.Reserve(tile_size);
}

// Checks if tile exists in the cache.
bool SynchronizedTileCache::Contains(const GraphId& graphid) const {
  std::lock_guard<std::mutex> lock(mutex_ref_);
  return cache_.Contains(graphid);
}

// Lets you know if the cache is too large.
bool SynchronizedTileCache::OverCommitted() const {
  std::lock_guard<std::mutex> lock(mutex_ref_);
  return cache_.OverCommitted();
}

// Clears the cache.
void SynchronizedTileCache::Clear() {
  std::lock_guard<std::mutex> lock(mutex_ref_);
  cache_.Clear();
}

void SynchronizedTileCache::Trim() {
  std::lock_guard<std::mutex> lock(mutex_ref_);
  cache_.Trim();
}

// Get a pointer to a graph tile object given a GraphId.
graph_tile_ptr SynchronizedTileCache::Get(const GraphId& graphid) const {
  std::lock_guard<std::mutex> lock(mutex_ref_);
  return cache_.Get(graphid);
}

// Puts a copy of a tile of into the cache.
graph_tile_ptr SynchronizedTileCache::Put(const GraphId& graphid, graph_tile_ptr tile, size_t size) {
  std::lock_guard<std::mutex> lock(mutex_ref_);
  return cache_.Put(graphid, std::move(tile), size);
}

// Constructs tile cache.
TileCache* TileCacheFactory::createTileCache(const boost::property_tree::ptree& pt) {
  size_t max_cache_size = pt.get<size_t>("max_cache_size", DEFAULT_MAX_CACHE_SIZE);

  bool use_lru_cache = pt.get<bool>("use_lru_mem_cache", false);
  auto lru_mem_control = pt.get<bool>("lru_mem_cache_hard_control", false)
                             ? TileCacheLRU::MemoryLimitControl::HARD
                             : TileCacheLRU::MemoryLimitControl::SOFT;

  bool use_simple_cache = pt.get<bool>("use_simple_mem_cache", false);

  // wrap tile cache with thread-safe version
  if (pt.get<bool>("global_synchronized_cache", false)) {
    // Handle synchronization of cache
    static std::mutex globalCacheMutex_;
    static std::shared_ptr<TileCache> globalTileCache_;
    // We need to lock the factory method itself to prevent races
    static std::mutex factoryMutex;
    std::lock_guard<std::mutex> lock(factoryMutex);
    if (!globalTileCache_) {
      if (use_lru_cache) {
        globalTileCache_.reset(new TileCacheLRU(max_cache_size, lru_mem_control));
      } else {
        // globalTileCache_.reset(new SimpleTileCache(max_cache_size));
        globalTileCache_.reset(new FlatTileCache(max_cache_size));
      }
    }
    return new SynchronizedTileCache(*globalTileCache_, globalCacheMutex_);
  }

  // or do you want to use an LRU cache
  if (use_lru_cache) {
    return new TileCacheLRU(max_cache_size, lru_mem_control);
  }

  // maybe you want a basic hashmap of tiles
  if (use_simple_cache) {
    return new SimpleTileCache(max_cache_size);
  }

  // by default you get a fixed size vector of tiles, requires about 8.5 megs of ram
  return new FlatTileCache(max_cache_size);
}

// Constructor using separate tile files
GraphReader::GraphReader(const boost::property_tree::ptree& pt,
                         std::unique_ptr<tile_getter_t>&& tile_getter)
    : tile_extract_(new tile_extract_t(pt)),
      tile_dir_(tile_extract_->tiles.empty() ? pt.get<std::string>("tile_dir", "") : ""),
      tile_getter_(std::move(tile_getter)),
      max_concurrent_users_(pt.get<size_t>("max_concurrent_reader_users", 1)),
      tile_url_(pt.get<std::string>("tile_url", "")), cache_(TileCacheFactory::createTileCache(pt)) {

  // Make a tile fetcher if we havent passed one in from somewhere else
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

  // Initialize the incident cache singleton if we have any kind of configuration to do so. if the
  // configuration is wrong or any kind of problem occurs this throws. the call below will spawn a
  // single background thread which is responsible for loading incidents continually
  enable_incidents_ = !pt.get<std::string>("incident_log", "").empty() ||
                      !pt.get<std::string>("incident_dir", "").empty();
  if (enable_incidents_) {
    incident_singleton_t::get({}, pt,
                              tile_extract_->tiles.empty() ? std::unordered_set<GraphId>{}
                                                           : GetTileSet());
  }

  // Fill shortcut recovery cache if requested or by default in memmap mode
  if (pt.get<bool>("shortcut_caching", false)) {
    shortcut_recovery_t::get_instance(this);
  }
}

// Method to test if tile exists
bool GraphReader::DoesTileExist(const GraphId& graphid) const {
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

class TarballGraphMemory final : public GraphMemory {
public:
  TarballGraphMemory(std::shared_ptr<midgard::tar> archive, std::pair<char*, size_t> position)
      : archive_(std::move(archive)) {
    data = position.first;
    size = position.second;
  }

private:
  const std::shared_ptr<midgard::tar> archive_;
};

// Get a pointer to a graph tile object given a GraphId. Return nullptr
// if the tile is not found/empty
graph_tile_ptr GraphReader::GetGraphTile(const GraphId& graphid) {
  // Return nullptr if not a valid tile
  if (!graphid.Is_Valid()) {
    return nullptr;
  }

  // Check if the level/tileid combination is in the cache
  auto base = graphid.Tile_Base();
  if (const auto& cached = cache_->Get(base)) {
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
    auto memory = std::make_unique<TarballGraphMemory>(tile_extract_->archive, t->second);

    auto traffic_ptr = tile_extract_->traffic_tiles.find(base);
    auto traffic_memory = traffic_ptr != tile_extract_->traffic_tiles.end()
                              ? std::make_unique<TarballGraphMemory>(tile_extract_->traffic_archive,
                                                                     traffic_ptr->second)
                              : nullptr;

    // This initializes the tile from mmap
    auto tile = GraphTile::Create(base, std::move(memory), std::move(traffic_memory));
    if (!tile) {
      // LOG_DEBUG("Memory map cache miss " + GraphTile::FileSuffix(base));
      return nullptr;
    }
    // LOG_DEBUG("Memory map cache hit " + GraphTile::FileSuffix(base));

    // Keep a copy in the cache and return it
    const size_t size = AVERAGE_MM_TILE_SIZE; // tile.end_offset();  // TODO what size??
    return cache_->Put(base, std::move(tile), size);
  } // Try getting it from flat file
  else {
    auto traffic_ptr = tile_extract_->traffic_tiles.find(base);
    auto traffic_memory = traffic_ptr != tile_extract_->traffic_tiles.end()
                              ? std::make_unique<TarballGraphMemory>(tile_extract_->traffic_archive,
                                                                     traffic_ptr->second)
                              : nullptr;

    // Try to get it from disk and if we cant..
    graph_tile_ptr tile = GraphTile::Create(tile_dir_, base, std::move(traffic_memory));
    if (!tile || !tile->header()) {
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
      if (!tile) {
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
    const size_t size = tile->header()->end_offset();
    return cache_->Put(base, std::move(tile), size);
  }
}

// Convenience method to get an opposing directed edge graph Id.
GraphId GraphReader::GetOpposingEdgeId(const GraphId& edgeid, graph_tile_ptr& opp_tile) {
  // If you cant get the tile you get an invalid id
  auto tile = opp_tile;
  if (!GetGraphTile(edgeid, tile)) {
    return {};
  };

  // For now return an invalid Id if this is a transit edge
  const auto* directededge = tile->directededge(edgeid);
  if (directededge->IsTransitLine()) {
    return {};
  };

  // If edge leaves the tile get the end node's tile
  GraphId id = directededge->endnode();
  if (!GetGraphTile(id, opp_tile)) {
    return {};
  };

  // Get the opposing edge
  id.set_id(opp_tile->node(id)->edge_index() + directededge->opp_index());
  return id;
}

// Convenience method to determine if 2 directed edges are connected.
bool GraphReader::AreEdgesConnected(const GraphId& edge1, const GraphId& edge2) {
  // Check if there is a transition edge between n1 and n2
  auto is_transition = [this](const GraphId& n1, const GraphId& n2) {
    if (n1.level() == n2.level()) {
      return false;
    } else {
      graph_tile_ptr tile = GetGraphTile(n1);
      const NodeInfo* ni = nullptr;
      if (!tile || (ni = tile->node(n1))->transition_count() == 0) {
        return false;
      }
      const NodeTransition* trans = tile->transition(ni->transition_index());
      for (uint32_t i = 0; i < ni->transition_count(); ++i, ++trans) {
        if (trans->endnode() == n2) {
          return true;
        }
      }
    }
    return false;
  };

  // Get both directed edges
  graph_tile_ptr t1 = GetGraphTile(edge1);
  graph_tile_ptr t2 = t1;

  if (!t1 || !(t2 = GetGraphTile(edge2, t2))) {
    return false;
  }
  const DirectedEdge* de1 = t1->directededge(edge1);
  const DirectedEdge* de2 = t2->directededge(edge2);

  if (de1->endnode() == de2->endnode() || is_transition(de1->endnode(), de2->endnode())) {
    return true;
  }

  // Get opposing edge to de1
  const DirectedEdge* de1_opp = GetOpposingEdge(edge1, t1);
  if (de1_opp &&
      (de1_opp->endnode() == de2->endnode() || is_transition(de1_opp->endnode(), de2->endnode()))) {
    return true;
  }

  // Get opposing edge to de2 and compare to both edge1 endnodes
  const DirectedEdge* de2_opp = GetOpposingEdge(edge2, t2);
  return de1_opp && de2_opp &&
         (de2_opp->endnode() == de1->endnode() || de2_opp->endnode() == de1_opp->endnode() ||
          is_transition(de2_opp->endnode(), de1->endnode()) ||
          is_transition(de2_opp->endnode(), de1_opp->endnode()));
}

// Convenience method to determine if 2 directed edges are connected from
// end node of edge1 to the start node of edge2.
bool GraphReader::AreEdgesConnectedForward(const GraphId& edge1,
                                           const GraphId& edge2,
                                           graph_tile_ptr& tile) {
  // Get end node of edge1
  GraphId endnode = edge_endnode(edge1, tile);
  if (endnode.Tile_Base() != edge1.Tile_Base()) {
    tile = GetGraphTile(endnode);
    if (tile == nullptr) {
      return false;
    }
  }

  // If edge2 is on a different tile level transition to the node on that level
  if (edge2.level() != endnode.level()) {
    for (const auto& trans : tile->GetNodeTransitions(endnode)) {
      if (trans.endnode().level() == edge2.level()) {
        endnode = trans.endnode();
        tile = GetGraphTile(endnode);
        if (tile == nullptr) {
          return false;
        }
        break;
      }
    }
  }

  // Check if edge2's Id is an outgoing directed edge of the node
  const NodeInfo* node = tile->node(endnode);
  return (node->edge_index() <= edge2.id() && edge2.id() < (node->edge_index() + node->edge_count()));
}

// Get the shortcut edge that includes this edge.
GraphId GraphReader::GetShortcut(const GraphId& id) {
  // Lambda to get continuing edge at a node. Skips the specified edge Id
  // transition edges, shortcut edges, and transit connections. Returns
  // nullptr if more than one edge remains or no continuing edge is found.
  auto continuing_edge = [](const graph_tile_ptr& tile, const GraphId& edgeid,
                            const NodeInfo* nodeinfo) {
    uint32_t idx = nodeinfo->edge_index();
    const DirectedEdge* continuing_edge = static_cast<const DirectedEdge*>(nullptr);
    const DirectedEdge* directededge = tile->directededge(idx);
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, idx++) {
      if (idx == edgeid.id() || directededge->is_shortcut() ||
          directededge->use() == Use::kTransitConnection ||
          directededge->use() == Use::kEgressConnection ||
          directededge->use() == Use::kPlatformConnection) {
        continue;
      }
      if (continuing_edge != nullptr) {
        return static_cast<const DirectedEdge*>(nullptr);
      }
      continuing_edge = directededge;
    }
    return continuing_edge;
  };

  // No shortcuts on the local level or transit level.
  if (id.level() >= TileHierarchy::levels().back().level) {
    return {};
  }

  // If this edge is a shortcut return this edge Id
  graph_tile_ptr tile = GetGraphTile(id);
  const DirectedEdge* directededge = tile->directededge(id);
  if (directededge->is_shortcut()) {
    return id;
  }

  // Walk backwards along the opposing directed edge until a shortcut
  // beginning is found or to get the continuing edge until a node that starts
  // the shortcut is found or there are 2 or more other regular edges at the
  // node.
  GraphId edgeid = id;
  const NodeInfo* node = nullptr;
  const DirectedEdge* cont_de = nullptr;
  while (true) {
    // Get the continuing directed edge. Initial case is to use the opposing
    // directed edge.
    cont_de = (node == nullptr) ? GetOpposingEdge(id) : continuing_edge(tile, edgeid, node);
    if (cont_de == nullptr) {
      return {};
    }

    // Get the end node and end node tile
    GraphId endnode = cont_de->endnode();
    if (cont_de->leaves_tile()) {
      tile = GetGraphTile(endnode.Tile_Base());
    }
    node = tile->node(endnode);

    // Get the opposing edge Id and its directed edge
    uint32_t idx = node->edge_index() + cont_de->opp_index();
    edgeid = {endnode.tileid(), endnode.level(), idx};
    directededge = tile->directededge(edgeid);
    if (directededge->superseded()) {
      // Get the shortcut edge Id that supersedes this edge
      uint32_t idx = node->edge_index() + (directededge->superseded() - 1);
      return GraphId(endnode.tileid(), endnode.level(), idx);
    }
  }
  return {};
}

// Unpack edges for a given shortcut edge
std::vector<GraphId> GraphReader::RecoverShortcut(const GraphId& shortcut_id) {
  return shortcut_recovery_t::get_instance().get(shortcut_id, *this);
}

// Convenience method to get the relative edge density (from the
// begin node of an edge).
uint32_t GraphReader::GetEdgeDensity(const GraphId& edgeid) {
  // Get the end node of the opposing directed edge
  const DirectedEdge* opp_edge = GetOpposingEdge(edgeid);
  if (opp_edge) {
    GraphId id = opp_edge->endnode();
    graph_tile_ptr tile = GetGraphTile(id);
    return (tile != nullptr) ? tile->node(id)->density() : 0;
  } else {
    return 0;
  }
}

// Get the end nodes of a directed edge.
std::pair<GraphId, GraphId> GraphReader::GetDirectedEdgeNodes(graph_tile_ptr tile,
                                                              const DirectedEdge* edge) {
  GraphId end_node = edge->endnode();
  GraphId start_node;
  graph_tile_ptr t2 = (edge->leaves_tile()) ? GetGraphTile(end_node) : std::move(tile);
  if (t2 != nullptr) {
    auto edge_idx = t2->node(end_node)->edge_index() + edge->opp_index();
    start_node = t2->directededge(edge_idx)->endnode();
  }
  return std::make_pair(start_node, end_node);
}

std::string GraphReader::encoded_edge_shape(const valhalla::baldr::GraphId& edgeid) {
  auto t_debug = GetGraphTile(edgeid);
  if (t_debug == nullptr) {
    return {};
  }

  const baldr::DirectedEdge* directedEdge = t_debug->directededge(edgeid);
  auto shape = t_debug->edgeinfo(directedEdge).shape();
  if (!directedEdge->forward()) {
    std::reverse(shape.begin(), shape.end());
  }
  return midgard::encode(shape);
}

// Note: this will grab all road tiles and transit tiles.
std::unordered_set<GraphId> GraphReader::GetTileSet() const {
  // either mmap'd tiles
  std::unordered_set<GraphId> tiles;
  if (tile_extract_->tiles.size()) {
    for (const auto& t : tile_extract_->tiles) {
      tiles.emplace(t.first);
    }
  } // or individually on disk
  else if (!tile_dir_.empty()) {
    // for each level
    for (uint8_t level = 0; level <= TileHierarchy::GetTransitLevel().level; ++level) {
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
std::unordered_set<GraphId> GraphReader::GetTileSet(const uint8_t level) const {
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

AABB2<PointLL> GraphReader::GetMinimumBoundingBox(const AABB2<PointLL>& bb) {
  // Iterate through all the tiles that intersect this bounding box
  const auto& ids = TileHierarchy::GetGraphIds(bb);
  AABB2<PointLL> min_bb{PointLL{}, PointLL{}};
  for (const auto& tile_id : ids) {
    // Don't take too much ram
    if (OverCommitted())
      Trim();

    // Look at every node in the tile
    auto tile = GetGraphTile(tile_id);
    for (uint32_t i = 0; tile && i < tile->header()->nodecount(); i++) {

      // If the node is within the input bounding box
      const auto* node = tile->node(i);
      auto node_ll = node->latlng(tile->header()->base_ll());
      if (bb.Contains(node_ll)) {

        // If we havent done anything with our bbox yet initialize it
        if (!min_bb.minpt().IsValid())
          min_bb = AABB2<PointLL>(node_ll, node_ll);

        // Look at the shape of each edge leaving the node
        const auto* diredge = tile->directededge(node->edge_index());
        for (uint32_t i = 0; i < node->edge_count(); i++, diredge++) {
          auto shape = tile->edgeinfo(diredge).lazy_shape();
          while (!shape.empty()) {
            min_bb.Expand(shape.pop());
          }
        }
      }
    }
  }

  // give back the expanded box
  return min_bb;
}

int GraphReader::GetTimezone(const baldr::GraphId& node, graph_tile_ptr& tile) {
  GetGraphTile(node, tile);
  return (tile == nullptr) ? 0 : tile->node(node)->timezone();
}

std::shared_ptr<const valhalla::IncidentsTile>
GraphReader::GetIncidentTile(const GraphId& tile_id) const {
  return enable_incidents_ ? incident_singleton_t::get(tile_id.Tile_Base())
                           : std::shared_ptr<valhalla::IncidentsTile>{};
}

IncidentResult GraphReader::GetIncidents(const GraphId& edge_id, graph_tile_ptr& tile) {
  // if we are not doing this for any reason then bail
  std::shared_ptr<const valhalla::IncidentsTile> itile;
  if (!enable_incidents_ || !GetGraphTile(edge_id, tile) ||
      !tile->trafficspeed(tile->directededge(edge_id)).has_incidents ||
      !(itile = GetIncidentTile(edge_id))) {
    return {};
  }

  // get the range of incidents we care about and hand it back, equal_range has no lambda option
  auto begin = std::partition_point(itile->locations().begin(), itile->locations().end(),
                                    [&edge_id](const valhalla::IncidentsTile::Location& candidate) {
                                      // first one that is >= the id we want
                                      bool is_found = candidate.edge_index() < edge_id.id();
                                      return is_found;
                                    });
  auto end = std::partition_point(begin, itile->locations().end(),
                                  [&edge_id](const valhalla::IncidentsTile::Location& candidate) {
                                    // first one that is > the id we want
                                    bool is_found = candidate.edge_index() <= edge_id.id();
                                    return is_found;
                                  });

  int begin_index = begin - itile->locations().begin();
  int end_index = end - itile->locations().begin();

  return {itile, begin_index, end_index};
}

const valhalla::IncidentsTile::Metadata&
getIncidentMetadata(const std::shared_ptr<const valhalla::IncidentsTile>& tile,
                    const valhalla::IncidentsTile::Location& incident_location) {
  const auto metadata_index = incident_location.metadata_index();
  if (metadata_index >= tile->metadata_size()) {
    throw std::runtime_error(std::string("Invalid incident tile with an incident_index of ") +
                             std::to_string(metadata_index) + " but total incident metadata of " +
                             std::to_string(tile->metadata_size()));
  }
  return tile->metadata(metadata_index);
}

} // namespace baldr
} // namespace valhalla
