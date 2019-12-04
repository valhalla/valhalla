#include "baldr/graphreader.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>

#include "midgard/logging.h"
#include "midgard/sequence.h"

#include "baldr/connectivity_map.h"
#include "filesystem.h"

using namespace valhalla::midgard;

namespace {
constexpr size_t DEFAULT_MAX_CACHE_SIZE = 1073741824; // 1 gig
constexpr size_t AVERAGE_TILE_SIZE = 2097152;         // 2 megs
constexpr size_t AVERAGE_MM_TILE_SIZE = 1024;         // 1k
} // namespace

namespace valhalla {
namespace baldr {

struct GraphReader::tile_extract_t {
  tile_extract_t(const boost::property_tree::ptree& pt) {
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
            // skip files we dont understand
          }
        }
        // couldn't load it
        if (tiles.empty()) {
          LOG_WARN("Tile extract contained no usuable tiles");
        } // loaded ok but with possibly bad blocks
        else {
          LOG_INFO("Tile extract successfully loaded with tile count: " +
                   std::to_string(tiles.size()));
          if (archive->corrupt_blocks) {
            LOG_WARN("Tile extract had " + std::to_string(archive->corrupt_blocks) +
                     " corrupt blocks");
          }
        }
      } catch (const std::exception& e) {
        LOG_ERROR(e.what());
        LOG_WARN("Tile extract could not be loaded");
      }
    }
  }
  // TODO: dont remove constness, and actually make graphtile read only?
  std::unordered_map<uint64_t, std::pair<char*, size_t>> tiles;
  std::shared_ptr<midgard::tar> archive;
};

std::shared_ptr<const GraphReader::tile_extract_t>
GraphReader::get_extract_instance(const boost::property_tree::ptree& pt) {
  static std::shared_ptr<const GraphReader::tile_extract_t> tile_extract(
      new GraphReader::tile_extract_t(pt));
  return tile_extract;
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
  static std::mutex globalCacheMutex_;
  static std::shared_ptr<TileCache> globalTileCache_;

  size_t max_cache_size = pt.get<size_t>("max_cache_size", DEFAULT_MAX_CACHE_SIZE);

  bool use_lru_cache = pt.get<bool>("use_lru_mem_cache", false);
  auto lru_mem_control = pt.get<bool>("lru_mem_cache_hard_control", false)
                             ? TileCacheLRU::MemoryLimitControl::HARD
                             : TileCacheLRU::MemoryLimitControl::SOFT;

  // wrap tile cache with thread-safe version
  if (pt.get<bool>("global_synchronized_cache", false)) {
    if (!globalTileCache_) {
      if (use_lru_cache) {
        globalTileCache_.reset(new TileCacheLRU(max_cache_size, lru_mem_control));
      } else {
        globalTileCache_.reset(new SimpleTileCache(max_cache_size));
      }
    }
    return new SynchronizedTileCache(*globalTileCache_, globalCacheMutex_);
  }

  if (use_lru_cache) {
    return new TileCacheLRU(max_cache_size, lru_mem_control);
  }
  return new SimpleTileCache(max_cache_size);
}

// Constructor using separate tile files
GraphReader::GraphReader(const boost::property_tree::ptree& pt)
    : tile_extract_(get_extract_instance(pt)), tile_dir_(pt.get<std::string>("tile_dir", "")),
      curlers_(std::make_unique<curler_pool_t>(pt.get<size_t>("max_concurrent_reader_users", 1),
                                               pt.get<std::string>("user_agent", ""))),
      tile_url_(pt.get<std::string>("tile_url", "")),
      tile_url_gz_(pt.get<bool>("tile_url_gz", false)),
      cache_(TileCacheFactory::createTileCache(pt)) {
  // validate tile url
  if (!tile_url_.empty() && tile_url_.find(GraphTile::kTilePathPattern) == std::string::npos)
    throw std::runtime_error("Not found tilePath pattern in tile url");
  // Reserve cache (based on whether using individual tile files or shared,
  // mmap'd file
  cache_->Reserve(tile_extract_->tiles.empty() ? AVERAGE_TILE_SIZE : AVERAGE_MM_TILE_SIZE);
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

bool GraphReader::DoesTileExist(const boost::property_tree::ptree& pt, const GraphId& graphid) {
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
const GraphTile* GraphReader::GetGraphTile(const GraphId& graphid) {
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

    // This initializes the tile from mmap
    GraphTile tile(base, t->second.first, t->second.second);
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
    // Try to get it from disk and if we cant..
    GraphTile tile(tile_dir_, base);
    if (!tile.header()) {
      {
        std::lock_guard<std::mutex> lock(_404s_lock);
        // See if we are configured for a url and if we are
        if (tile_url_.empty() || _404s.find(base) != _404s.end()) {
          // LOG_DEBUG("Url cache miss " + GraphTile::FileSuffix(base));
          return nullptr;
        }
      }

      {
        scoped_curler_t curler(*curlers_);
        // Get it from the url and cache it to disk if you can
        tile = GraphTile::CacheTileURL(tile_url_, base, curler.get(), tile_url_gz_, tile_dir_);
      }

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

// Convenience method to get an opposing directed edge graph Id.
GraphId GraphReader::GetOpposingEdgeId(const GraphId& edgeid, const GraphTile*& tile) {
  // If you cant get the tile you get an invalid id
  tile = GetGraphTile(edgeid);
  if (!tile) {
    return {};
  };
  // For now return an invalid Id if this is a transit edge
  const auto* directededge = tile->directededge(edgeid);
  if (directededge->IsTransitLine()) {
    return {};
  };

  // If edge leaves the tile get the end node's tile
  GraphId id = directededge->endnode();
  if (!GetGraphTile(id, tile)) {
    return {};
  };

  // Get the opposing edge
  id.set_id(tile->node(id)->edge_index() + directededge->opp_index());
  return id;
}

// Convenience method to determine if 2 directed edges are connected.
bool GraphReader::AreEdgesConnected(const GraphId& edge1, const GraphId& edge2) {
  // Check if there is a transition edge between n1 and n2
  auto is_transition = [this](const GraphId& n1, const GraphId& n2) {
    if (n1.level() == n2.level()) {
      return false;
    } else {
      const GraphTile* tile = GetGraphTile(n1);
      const NodeInfo* ni = tile->node(n1);
      if (ni->transition_count() == 0)
        return false;
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
  const GraphTile* t1 = GetGraphTile(edge1);
  const DirectedEdge* de1 = t1->directededge(edge1);
  const GraphTile* t2 = (edge2.Tile_Base() == edge1.Tile_Base()) ? t1 : GetGraphTile(edge2);
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
  if (de2_opp && (de2_opp->endnode() == de1->endnode() || de2_opp->endnode() == de1_opp->endnode() ||
                  is_transition(de2_opp->endnode(), de1->endnode()) ||
                  is_transition(de2_opp->endnode(), de1_opp->endnode()))) {
    return true;
  }
  return false;
}

// Convenience method to determine if 2 directed edges are connected from
// end node of edge1 to the start node of edge2.
bool GraphReader::AreEdgesConnectedForward(const GraphId& edge1,
                                           const GraphId& edge2,
                                           const GraphTile*& tile) {
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
  auto continuing_edge = [](const GraphTile* tile, const GraphId& edgeid, const NodeInfo* nodeinfo) {
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
  if (id.level() >= TileHierarchy::levels().rbegin()->second.level) {
    return {};
  }

  // If this edge is a shortcut return this edge Id
  const GraphTile* tile = GetGraphTile(id);
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
  // grab the shortcut edge
  const GraphTile* tile = GetGraphTile(shortcut_id);
  const DirectedEdge* shortcut = tile->directededge(shortcut_id);

  // bail if this isnt a shortcut
  if (!shortcut->is_shortcut()) {
    return {shortcut_id};
  }

  // loop over the edges leaving its begin node and find the superseded edge
  GraphId begin_node = edge_startnode(shortcut_id);
  if (!begin_node)
    return {shortcut_id};

  // loop over the edges leaving its begin node and find the superseded edge
  std::vector<GraphId> edges;
  for (const DirectedEdge& de : tile->GetDirectedEdges(begin_node.id())) {
    if (shortcut->shortcut() & de.superseded()) {
      edges.push_back(tile->header()->graphid());
      edges.back().set_id(&de - tile->directededge(0));
      break;
    }
  }

  // bail if we couldnt find it
  if (edges.empty()) {
    LOG_ERROR("Unable to recover shortcut for edgeid " + std::to_string(shortcut_id) +
              " | no superseded edge");
    return {shortcut_id};
  }

  // seed the edge walking with the first edge
  const DirectedEdge* current_edge = tile->directededge(edges.back());
  uint32_t accumulated_length = current_edge->length();

  // walk edges until we find the same ending node as the shortcut
  while (current_edge->endnode() != shortcut->endnode()) {
    // get the node at the end of the last edge we added
    const NodeInfo* node = GetEndNode(current_edge, tile);
    if (!node)
      return {shortcut_id};
    auto node_index = node - tile->node(0);

    // check the edges leaving this node to see if we can find the one that is part of the shortcut
    current_edge = nullptr;
    for (const DirectedEdge& edge : tile->GetDirectedEdges(node_index)) {
      // are they the same enough that its part of the shortcut
      // NOTE: this fails in about .05% of cases where there are two candidates and its not clear
      // which edge is the right one. looking at shortcut builder its not obvious how this is possible
      // as it seems to terminate a shortcut if more than one edge pair can be contracted...
      // NOTE: because we change the speed of the edge in graph enhancer we cant use speed as a
      // reliable determining factor
      if (begin_node != edge.endnode() && !edge.is_shortcut() &&
          (edge.forwardaccess() & kAutoAccess) && edge.exitsign() == shortcut->exitsign() &&
          edge.use() == shortcut->use() && edge.classification() == shortcut->classification() &&
          edge.roundabout() == shortcut->roundabout() && edge.link() == shortcut->link() &&
          edge.toll() == shortcut->toll() && edge.destonly() == shortcut->destonly() &&
          edge.unpaved() == shortcut->unpaved() && edge.surface() == shortcut->surface()/* &&
          edge.speed() == shortcut->speed()*/) {
        // we are going to keep this edge
        edges.emplace_back(tile->header()->graphid());
        edges.back().set_id(&edge - tile->directededge(0));
        // and keep expanding from the end of it
        current_edge = &edge;
        begin_node = tile->header()->graphid();
        begin_node.set_id(node_index);
        accumulated_length += edge.length();
        break;
      }
    }

    // if we didnt add an edge or we went over the length we failed
    if (current_edge == nullptr || accumulated_length > shortcut->length()) {
      LOG_ERROR("Unable to recover shortcut for edgeid " + std::to_string(shortcut_id) +
                " | accumulated_length: " + std::to_string(accumulated_length) +
                " | shortcut_length: " + std::to_string(shortcut->length()));
      return {shortcut_id};
    }
  }

  // we somehow got to the end via a shorter path
  if (accumulated_length < shortcut->length()) {
    LOG_ERROR("Unable to recover shortcut for edgeid (accumulated_length < shortcut->length()) " +
              std::to_string(shortcut_id) +
              " | accumulated_length: " + std::to_string(accumulated_length) +
              " | shortcut_length: " + std::to_string(shortcut->length()));
    return {shortcut_id};
  }

  // these edges make up this shortcut
  return edges;
}

// Convenience method to get the relative edge density (from the
// begin node of an edge).
uint32_t GraphReader::GetEdgeDensity(const GraphId& edgeid) {
  // Get the end node of the opposing directed edge
  const DirectedEdge* opp_edge = GetOpposingEdge(edgeid);
  if (opp_edge) {
    GraphId id = opp_edge->endnode();
    const GraphTile* tile = GetGraphTile(id);
    return (tile != nullptr) ? tile->node(id)->density() : 0;
  } else {
    return 0;
  }
}

// Get the end nodes of a directed edge.
std::pair<GraphId, GraphId> GraphReader::GetDirectedEdgeNodes(const GraphTile* tile,
                                                              const DirectedEdge* edge) {
  GraphId end_node = edge->endnode();
  GraphId start_node;
  const GraphTile* t2 = (edge->leaves_tile()) ? GetGraphTile(end_node) : tile;
  if (t2 != nullptr) {
    auto edge_idx = t2->node(end_node)->edge_index() + edge->opp_index();
    start_node = t2->directededge(edge_idx)->endnode();
  }
  return std::make_pair(start_node, end_node);
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
    const auto* tile = GetGraphTile(tile_id);
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
          auto shape = tile->edgeinfo(diredge->edgeinfo_offset()).lazy_shape();
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

} // namespace baldr
} // namespace valhalla
