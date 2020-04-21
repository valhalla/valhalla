#include "baldr/tilecache.h"

namespace {
constexpr size_t DEFAULT_MAX_CACHE_SIZE = 1073741824; // 1 gig
} // namespace

namespace valhalla {
namespace baldr {

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
const GraphTile* SimpleTileCache::Get(const GraphId& graphid) const {
  auto cached = cache_.find(graphid);
  if (cached != cache_.end()) {
    return &cached->second;
  }
  return nullptr;
}

// Puts a copy of a tile of into the cache.
const GraphTile* SimpleTileCache::Put(const GraphId& graphid, const GraphTile& tile, size_t size) {
  cache_size_ += size;
  return &cache_.emplace(graphid, tile).first->second;
}

void SimpleTileCache::Trim() {
  Clear();
}

// ----------------------------------------------------------------------------
// TileCacheLRU implementation
// ----------------------------------------------------------------------------

// Constructor.
TileCacheLRU::TileCacheLRU(size_t max_size, MemoryLimitControl mem_control)
    : cache_size_(0), max_cache_size_(max_size), mem_control_(mem_control) {
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

const GraphTile* TileCacheLRU::Get(const GraphId& graphid) const {
  auto cached = cache_.find(graphid);
  if (cached == cache_.cend()) {
    return nullptr;
  }

  const KeyValueIter& entry_iter = cached->second;
  MoveToLruHead(entry_iter);

  return &entry_iter->tile;
}

size_t TileCacheLRU::TrimToFit(const size_t required_size) {
  size_t freed_space = 0;
  while ((OverCommitted() || (max_cache_size_ - cache_size_) < required_size) &&
         !key_val_lru_list_.empty()) {
    const KeyValue& entry_to_evict = key_val_lru_list_.back();
    const auto tile_size = entry_to_evict.tile.header()->end_offset();
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

const GraphTile*
TileCacheLRU::Put(const GraphId& graphid, const GraphTile& tile, size_t new_tile_size) {
  if (new_tile_size > max_cache_size_) {
    throw std::runtime_error("TileCacheLRU: tile size is bigger than max cache size");
  }

  auto cached = cache_.find(graphid);
  if (cached == cache_.end()) {
    if (mem_control_ == MemoryLimitControl::HARD) {
      TrimToFit(new_tile_size);
    }
    key_val_lru_list_.emplace_front(KeyValue{graphid, tile});
    cache_.emplace(graphid, key_val_lru_list_.begin());
  } else {
    // Value update; the new size may be different form the previous
    // TODO: in practice tile size for a specific id never changes
    //  if tile set is changed (e.g. new version), the cache should be cleared
    //  do we need to take it into account here? (can dramatically simplify the code)
    // note: SimpleTileCache does not handle the overwrite at the moment
    auto& entry_iter = cached->second;
    const auto old_tile_size = entry_iter->tile.header()->end_offset();

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

    entry_iter->tile = tile;
    cache_size_ -= old_tile_size;
  }
  cache_size_ += new_tile_size;

  return &key_val_lru_list_.front().tile;
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
const GraphTile* SynchronizedTileCache::Get(const GraphId& graphid) const {
  std::lock_guard<std::mutex> lock(mutex_ref_);
  return cache_.Get(graphid);
}

// Puts a copy of a tile of into the cache.
const GraphTile*
SynchronizedTileCache::Put(const GraphId& graphid, const GraphTile& tile, size_t size) {
  std::lock_guard<std::mutex> lock(mutex_ref_);
  return cache_.Put(graphid, tile, size);
}

// Constructs tile cache.
TileCache* TileCacheFactory::createTileCache(const boost::property_tree::ptree& pt) {
  size_t max_cache_size = pt.get<size_t>("max_cache_size", DEFAULT_MAX_CACHE_SIZE);

  bool use_lru_cache = pt.get<bool>("use_lru_mem_cache", false);
  auto lru_mem_control = pt.get<bool>("lru_mem_cache_hard_control", false)
                             ? TileCacheLRU::MemoryLimitControl::HARD
                             : TileCacheLRU::MemoryLimitControl::SOFT;

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
        globalTileCache_.reset(new SimpleTileCache(max_cache_size));
      }
    }
    return new SynchronizedTileCache(*globalTileCache_, globalCacheMutex_);
  }

  // Otherwise: No synchronization
  if (use_lru_cache) {
    return new TileCacheLRU(max_cache_size, lru_mem_control);
  }
  return new SimpleTileCache(max_cache_size);
}

} // namespace baldr
} // namespace valhalla
