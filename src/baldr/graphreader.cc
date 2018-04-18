#include "baldr/graphreader.h"

#include <string>
#include <thread>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <boost/filesystem.hpp>

#include "midgard/logging.h"
#include "midgard/sequence.h"

#include "baldr/connectivity_map.h"
#include "baldr/filesystem_utils.h"
#include "baldr/curler.h"

using namespace valhalla::baldr;

namespace {
  constexpr size_t DEFAULT_MAX_CACHE_SIZE = 1073741824; //1 gig
  constexpr size_t AVERAGE_TILE_SIZE = 2097152; //2 megs
  constexpr size_t AVERAGE_MM_TILE_SIZE = 1024; //1k
}

namespace valhalla {
namespace baldr {

class GraphReader::tile_source_t
{
public:
  tile_source_t(const std::shared_ptr<tile_source_t> &next) : next_(next){
  }

  virtual ~tile_source_t(){
  }

  virtual size_t getAverageTileSize(){
    return AVERAGE_TILE_SIZE;
  }

  virtual bool DoesTileExist(const GraphId& base){
    return next_ ? next_->DoesTileExist(base) : false;
  }

  virtual std::pair<GraphTile, uint32_t> GetGraphTile(const GraphId& graphid){
    return next_ ? next_->GetGraphTile(graphid) : std::make_pair(GraphTile(), (uint32_t)0);
  }

  virtual void FillTileSet(std::unordered_set<GraphId> &result){
    if(next_)
      next_->FillTileSet(result);
  }

  virtual void FillTileSet(std::unordered_set<GraphId> &result, const uint8_t level) {
    if(next_)
      next_->FillTileSet(result, level);
  }

private:
  std::shared_ptr<tile_source_t> next_;
};

class GraphReader::tile_source_extract_t : public GraphReader::tile_source_t {
public:
  tile_source_extract_t(const std::shared_ptr<tile_source_t> &next, const std::string &tile_extract_path) :
      tile_source_t(std::move(next)),
      tar_(tile_extract_path){
    for(auto& c : tar_.contents) {
      try {
        auto id = GraphTile::GetTileId(c.first);
        tiles_[id] = std::make_pair(const_cast<char*>(c.second.first), c.second.second);
      }
      catch(...){}
    }
    if(tiles_.empty()) {
      LOG_WARN("Tile extract could not be loaded");
    }//loaded ok but with possibly bad blocks
    else {
      LOG_INFO("Tile extract successfully loaded");
      if(tar_.corrupt_blocks)
        LOG_WARN("Tile extract had " + std::to_string(tar_.corrupt_blocks) + " corrupt blocks");
    }
  }

  virtual ~tile_source_extract_t(){
  }

  virtual size_t getAverageTileSize(){
    return AVERAGE_MM_TILE_SIZE;
  }

  virtual bool DoesTileExist(const GraphId& graphid){
    return tiles_.find(graphid) != tiles_.cend() ? true : tile_source_t::DoesTileExist(graphid);
  }

  virtual void FillTileSet(std::unordered_set<GraphId> &result){
    for(const auto& t : tiles_)
      result.emplace(t.first);
    tile_source_t::FillTileSet(result);
  }

  virtual void FillTileSet(std::unordered_set<GraphId> &result, const uint8_t level){
    for(const auto& t : tiles_)
      if(GraphId(t.first).level() == level)
        result.emplace(t.first);
    tile_source_t::FillTileSet(result, level);
  }

  virtual std::pair<GraphTile, uint32_t> GetGraphTile(const GraphId& base){
    auto t = tiles_.find(base);
    if(t == tiles_.cend())
      return tile_source_t::GetGraphTile(base);
    GraphTile tile(base, t->second.first, t->second.second);
    if (!tile.header())
      return tile_source_t::GetGraphTile(base);
    return std::make_pair(tile, t->second.second);
  }

  const std::unordered_map<uint64_t, std::pair<char*, size_t> > &getTiles(){
    return tiles_;
  }
private:
  midgard::tar tar_;
  std::unordered_map<uint64_t, std::pair<char*, size_t> > tiles_;
};

class GraphReader::tile_source_files_t : public GraphReader::tile_source_t
{
public:
  tile_source_files_t(const std::shared_ptr<tile_source_t> &next, const std::string &tile_dir):
      tile_source_t(std::move(next)),tile_dir_(tile_dir) {
  }

  virtual bool DoesTileExist(const GraphId& graphid){
    std::string file_location = tile_dir_ + filesystem::path_separator + GraphTile::FileSuffix(graphid.Tile_Base());
    struct stat buffer;
    return stat(file_location.c_str(), &buffer) == 0 || stat((file_location + ".gz").c_str(), &buffer) == 0;
  }

  virtual void FillTileSet(std::unordered_set<GraphId> &result){
    //Check files only when other sources have no tiles
    if(result.size() == 0)
    {
      for(uint8_t level = 0; level <= TileHierarchy::levels().rbegin()->first + 1; ++level) {
        //crack open this level of tiles directory
        boost::filesystem::path root_dir(tile_dir_ + filesystem::path_separator + std::to_string(level) + filesystem::path_separator);
        if(boost::filesystem::exists(root_dir) && boost::filesystem::is_directory(root_dir)) {
          //iterate over all the files in there
          for (boost::filesystem::recursive_directory_iterator i(root_dir), end; i != end; ++i) {
            if (!boost::filesystem::is_directory(i->path())) {
              //add it if it can be parsed as a valid tile file name
              try { result.emplace(GraphTile::GetTileId(i->path().string())); }
              catch (...) { }
            }
          }
        }
      }
    }
    tile_source_t::FillTileSet(result);
  }

  virtual void FillTileSet(std::unordered_set<GraphId> &result, const uint8_t level) {
    //Check files only when other sources have no tiles
    if(result.size() == 0)
    {
      boost::filesystem::path root_dir(tile_dir_ + filesystem::path_separator + std::to_string(level) + filesystem::path_separator);
      if (boost::filesystem::exists(root_dir) && boost::filesystem::is_directory(root_dir)) {
        // iterate over all the files in the directory and turn into GraphIds
        for (boost::filesystem::recursive_directory_iterator i(root_dir), end; i != end; ++i) {
          if (!boost::filesystem::is_directory(i->path())) {
            //add it if it can be parsed as a valid tile file name
            try { result.emplace(GraphTile::GetTileId(i->path().string())); }
            catch (...) { }
          }
        }
      }
    }
    tile_source_t::FillTileSet(result, level);
  }

  virtual std::pair<GraphTile, uint32_t> GetGraphTile(const GraphId& base){
    GraphTile tile(tile_dir_, base);
    if (!tile.header())
      return tile_source_t::GetGraphTile(base);
    return std::make_pair(tile, tile.header()->end_offset());
  }

private:
  std::string tile_dir_;
};

class GraphReader::tile_source_curl_t : public GraphReader::tile_source_t
{
public:
  tile_source_curl_t(const std::shared_ptr<tile_source_t> &next, const std::string &tile_url):
      tile_source_t(std::move(next)),tile_url_(tile_url) {

  }

  virtual ~tile_source_curl_t(){
  }

  virtual std::pair<GraphTile, uint32_t> GetGraphTile(const GraphId& base){
    if(_404s.find(base) != _404s.end())
      return tile_source_t::GetGraphTile(base);
    
    GraphTile tile(tile_url_, base, curler_);
    if(!tile.header()) {
      _404s.insert(base);
      return tile_source_t::GetGraphTile(base);
    }
    return std::make_pair(tile, tile.header()->end_offset());
  }

private:
  std::string tile_url_;
  curler_t curler_;
  std::unordered_set<GraphId> _404s;
};

std::shared_ptr<GraphReader::tile_source_t> GraphReader::get_source_instance(const boost::property_tree::ptree& pt)
{
  static std::once_flag once_flag;
  static std::shared_ptr<tile_source_t> instance;
  std::call_once(once_flag, [&]{

    auto tile_url = pt.get_optional<std::string>("tile_url");
    if(tile_url)
    {
      instance = std::make_shared<tile_source_curl_t>(instance ,tile_url.get());
    }

    auto tile_dir =  pt.get_optional<std::string>("tile_dir");
    if(tile_dir)
    {
      instance = std::make_shared<tile_source_files_t>(instance ,tile_dir.get());
    }

    auto tile_extracts = pt.get_child_optional("tile_extracts");
    if(tile_extracts)
    {
      for(auto it = tile_extracts.get().rbegin(); it != tile_extracts.get().rend(); ++it)
      {
        auto tile_source_extract = std::make_shared<tile_source_extract_t>(instance, it->second.get_value<std::string>());
        //Does something readed from tar ?
        if(tile_source_extract->getTiles().size())
          instance = tile_source_extract;
      }
    }

    auto tile_extract = pt.get_optional<std::string>("tile_extract");
    if(tile_extract)
    {
      auto tile_source_extract = std::make_shared<tile_source_extract_t>(instance, tile_extract.get());
      //Does something readed from tar ?
      if(tile_source_extract->getTiles().size())
        instance = tile_source_extract;
    }
  });
  return instance;
}

// Constructor.
SimpleTileCache::SimpleTileCache(size_t max_size)
      : cache_size_(0), max_cache_size_(max_size)
{
}

// Reserves enough cache to hold (max_cache_size / tile_size) items.
void SimpleTileCache::Reserve(size_t tile_size)
{
  cache_.reserve(max_cache_size_ / tile_size);
}

// Checks if tile exists in the cache.
bool SimpleTileCache::Contains(const GraphId& graphid) const
{
  return cache_.find(graphid) != cache_.end();
}

// Lets you know if the cache is too large.
bool SimpleTileCache::OverCommitted() const
{
  return max_cache_size_ < cache_size_;
}

// Clears the cache.
void SimpleTileCache::Clear()
{
  cache_size_ = 0;
  cache_.clear();
}

// Get a pointer to a graph tile object given a GraphId.
const GraphTile* SimpleTileCache::Get(const GraphId& graphid) const
{
  auto cached = cache_.find(graphid);
  if(cached != cache_.end()) {
    return &cached->second;
  }
  return nullptr;
}

// Puts a copy of a tile of into the cache.
const GraphTile* SimpleTileCache::Put(const GraphId& graphid, const GraphTile& tile, size_t size)
{
  cache_size_ += size;
  return &cache_.emplace(graphid, tile).first->second;
}

// Constructor.
SynchronizedTileCache::SynchronizedTileCache(TileCache& cache, std::mutex& mutex)
      : cache_(cache), mutex_ref_(mutex)
{
}

// Reserves enough cache to hold (max_cache_size / tile_size) items.
void SynchronizedTileCache::Reserve(size_t tile_size)
{
  std::lock_guard<std::mutex> lock(mutex_ref_);
  cache_.Reserve(tile_size);
}

// Checks if tile exists in the cache.
bool SynchronizedTileCache::Contains(const GraphId& graphid) const
{
  std::lock_guard<std::mutex> lock(mutex_ref_);
  return cache_.Contains(graphid);
}

// Lets you know if the cache is too large.
bool SynchronizedTileCache::OverCommitted() const
{
  std::lock_guard<std::mutex> lock(mutex_ref_);
  return cache_.OverCommitted();
}

// Clears the cache.
void SynchronizedTileCache::Clear()
{
  std::lock_guard<std::mutex> lock(mutex_ref_);
  cache_.Clear();
}

// Get a pointer to a graph tile object given a GraphId.
const GraphTile* SynchronizedTileCache::Get(const GraphId& graphid) const
{
  std::lock_guard<std::mutex> lock(mutex_ref_);
  return cache_.Get(graphid);
}

// Puts a copy of a tile of into the cache.
const GraphTile* SynchronizedTileCache::Put(const GraphId& graphid, const GraphTile& tile, size_t size)
{
  std::lock_guard<std::mutex> lock(mutex_ref_);
  return cache_.Put(graphid, tile, size);
}

// Constructs tile cache.
TileCache* TileCacheFactory::createTileCache(const boost::property_tree::ptree& pt)
{
  static std::mutex globalCacheMutex_;
  static std::shared_ptr<TileCache> globalTileCache_;

  size_t max_cache_size = pt.get<size_t>("max_cache_size", DEFAULT_MAX_CACHE_SIZE);

  // wrap tile cache with thread-safe version
  if (pt.get<bool>("global_synchronized_cache", false)) {
    if (!globalTileCache_)
      globalTileCache_.reset(new SimpleTileCache(max_cache_size));
    return new SynchronizedTileCache(*globalTileCache_, globalCacheMutex_);
  }

  // default
  return new SimpleTileCache(max_cache_size);
}

// Constructor using separate tile files
GraphReader::GraphReader(const boost::property_tree::ptree& pt)
    : cache_(TileCacheFactory::createTileCache(pt)),
      tile_dir_(pt.get<std::string>("tile_dir")),
      tile_source_(get_source_instance(pt)) {
  // Reserve cache (based on whether using individual tile files or shared,
  // mmap'd file
  cache_->Reserve(tile_source_->getAverageTileSize());
}

// Method to test if tile exists
bool GraphReader::DoesTileExist(const GraphId& graphid) const {
  if (!graphid.Is_Valid() || graphid.level() > TileHierarchy::get_max_level()) {
    return false;
  }
  //check tiles cahced in memory or disk
  if(cache_->Contains(graphid))
    return true;

  return tile_source_->DoesTileExist(graphid);
}

bool GraphReader::DoesTileExist(const boost::property_tree::ptree& pt, const GraphId& graphid) {
  if (!graphid.Is_Valid() || graphid.level() > TileHierarchy::get_max_level()) {
    return false;
  }
  return get_source_instance(pt)->DoesTileExist(graphid);
}

// Get a pointer to a graph tile object given a GraphId. Return nullptr
// if the tile is not found/empty
const GraphTile* GraphReader::GetGraphTile(const GraphId& graphid) {
  //TODO: clear the cache automatically once we become overcommitted by a certain amount

  // Return nullptr if not a valid tile
  if (!graphid.Is_Valid()) {
    return nullptr;
  }

  // Check if the level/tileid combination is in the cache
  auto base = graphid.Tile_Base();
  if(auto cached = cache_->Get(base)) {
    return cached;
  }

  //Load tile from source.
  auto p = tile_source_->GetGraphTile(base);
  if (!p.first.header())
    return nullptr;
  return cache_->Put(base, p.first, p.second);
}

// Convenience method to get an opposing directed edge graph Id.
GraphId GraphReader::GetOpposingEdgeId(const GraphId& edgeid, const GraphTile*& tile) {
  // If you cant get the tile you get an invalid id
  tile = GetGraphTile(edgeid);
  if(!tile)
    return {};
  // For now return an invalid Id if this is a transit edge
  const auto* directededge = tile->directededge(edgeid);
  if (directededge->IsTransitLine())
    return {};

  // If edge leaves the tile get the end node's tile
  GraphId id = directededge->endnode();
  if (!GetGraphTile(id, tile))
    return {};

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
      uint32_t n = 0, id = 0;
      const DirectedEdge* de = GetGraphTile(n1)->GetDirectedEdges(n1.id(), n, id);
      for (uint32_t i = 0; i < n; i++, de++) {
        if (de->IsTransition() && de->endnode() == n2) {
          return true;
        }
      }
    }
    return false;
  };

  // Get both directed edges
  const GraphTile* t1 = GetGraphTile(edge1);
  const DirectedEdge* de1 = t1->directededge(edge1);
  const GraphTile* t2 = (edge2.Tile_Base() == edge1.Tile_Base()) ?
                          t1 : GetGraphTile(edge2);
  const DirectedEdge* de2 = t2->directededge(edge2);
  if (de1->endnode() == de2->endnode() ||
      is_transition(de1->endnode(), de2->endnode())) {
    return true;
  }

  // Get opposing edge to de1
  const DirectedEdge* de1_opp = GetOpposingEdge(edge1, t1);
  if (de1_opp->endnode() == de2->endnode() ||
      is_transition(de1_opp->endnode(), de2->endnode())) {
    return true;
  }

  // Get opposing edge to de2 and compare to both edge1 endnodes
  const DirectedEdge* de2_opp = GetOpposingEdge(edge2, t2);
  if (de2_opp->endnode() == de1->endnode() ||
      de2_opp->endnode() == de1_opp->endnode() ||
      is_transition(de2_opp->endnode(), de1->endnode()) ||
      is_transition(de2_opp->endnode(), de1_opp->endnode())) {
    return true;
  }
  return false;
}

// Convenience method to determine if 2 directed edges are connected from
// end node of edge1 to the start node of edge2.
bool GraphReader::AreEdgesConnectedForward(const GraphId& edge1, const GraphId& edge2,
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
    for (const auto& edge : tile->GetDirectedEdges(endnode)) {
      if (edge.IsTransition() && edge.endnode().level() == edge2.level()) {
        endnode = edge.endnode();
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
  return (node->edge_index() <= edge2.id() &&
          edge2.id() < (node->edge_index() + node->edge_count()));
}

// Get the shortcut edge that includes this edge.
GraphId GraphReader::GetShortcut(const GraphId& id) {
  // Lambda to get continuing edge at a node. Skips the specified edge Id
  // transition edges, shortcut edges, and transit connections. Returns
  // nullptr if more than one edge remains or no continuing edge is found.
  auto continuing_edge = [](const GraphTile* tile, const GraphId& edgeid,
                            const NodeInfo* nodeinfo) {
    uint32_t idx = nodeinfo->edge_index();
    const DirectedEdge* continuing_edge = static_cast<const DirectedEdge*>(nullptr);
    const DirectedEdge* directededge = tile->directededge(idx);
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, idx++) {
      if (directededge->IsTransition() ||
          idx == edgeid.id() || directededge->is_shortcut() ||
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
    return { };
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
    cont_de = (node == nullptr) ? GetOpposingEdge(id) :
                continuing_edge(tile, edgeid, node);
    if (cont_de == nullptr) {
      return { };
    }

    // Get the end node and end node tile
    GraphId endnode = cont_de->endnode();
    if (cont_de->leaves_tile()) {
      tile = GetGraphTile(endnode.Tile_Base());
    }
    node = tile->node(endnode);

    // Get the opposing edge Id and its directed edge
    uint32_t idx = node->edge_index() + cont_de->opp_index();
    edgeid = { endnode.tileid(), endnode.level(), idx };
    directededge = tile->directededge(edgeid);
    if (directededge->superseded()) {
      // Get the shortcut edge Id that supersedes this edge
      uint32_t idx = node->edge_index() + (directededge->superseded() - 1);
      return GraphId(endnode.tileid(), endnode.level(), idx);
    }
  }
  return { };
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
  std::unordered_set<GraphId> rv;
  tile_source_->FillTileSet(rv);
  return rv;
}

// Get the set of tiles for a specified level
std::unordered_set<GraphId> GraphReader::GetTileSet(const uint8_t level) const {
  std::unordered_set<GraphId> rv;
  tile_source_->FillTileSet(rv, level);
  return rv;
}

}
}
