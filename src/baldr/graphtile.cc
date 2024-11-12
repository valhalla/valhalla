#include "baldr/graphtile.h"

#include "baldr/compression_utils.h"
#include "baldr/curl_tilegetter.h"
#include "baldr/datetime.h"
#include "baldr/sign.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"
#include "midgard/tiles.h"

#include <boost/algorithm/string.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace valhalla::midgard;

namespace {
const AABB2<PointLL> world_box(PointLL(-180, -90), PointLL(180, 90));
constexpr float COMPRESSION_HINT = 3.5f;

// the point of this function is to avoid race conditions for writing a tile between threads
// so the easiest thing to do is just use the thread id to differentiate
std::string GenerateTmpSuffix() {
  std::stringstream ss;
  ss << ".tmp_" << std::this_thread::get_id() << "_"
     << std::chrono::high_resolution_clock::now().time_since_epoch().count();
  return ss.str();
}

} // namespace

namespace valhalla {
namespace baldr {

class VectorGraphMemory final : public GraphMemory {
public:
  VectorGraphMemory(std::vector<char>&& memory) : memory_(std::move(memory)) {
    data = const_cast<char*>(memory_.data());
    size = memory_.size();
  }

private:
  const std::vector<char> memory_;
};

graph_tile_ptr GraphTile::DecompressTile(const GraphId& graphid,
                                         const std::vector<char>& compressed) {
  // for setting where to read compressed data from
  auto src_func = [&compressed](z_stream& s) -> void {
    s.next_in =
        const_cast<Byte*>(static_cast<const Byte*>(static_cast<const void*>(compressed.data())));
    s.avail_in = static_cast<unsigned int>(compressed.size());
  };

  // for setting where to write the uncompressed data to
  std::vector<char> data;
  auto dst_func = [&data, &compressed](z_stream& s) -> int {
    // if the whole buffer wasn't used we are done
    auto size = data.size();
    if (s.total_out < size)
      data.resize(s.total_out);
    // we need more space
    else {
      // assume we need 3.5x the space
      data.resize(size + (compressed.size() * COMPRESSION_HINT));
      // set the pointer to the next spot
      s.next_out = static_cast<Byte*>(static_cast<void*>(data.data() + size));
      s.avail_out = compressed.size() * COMPRESSION_HINT;
    }
    return Z_NO_FLUSH;
  };

  // Decompress tile into memory
  if (!baldr::inflate(src_func, dst_func)) {
    LOG_ERROR("Failed to gunzip " + GraphTile::FileSuffix(graphid, SUFFIX_COMPRESSED));
    return nullptr;
  }

  return graph_tile_ptr{
      new GraphTile(graphid, std::make_unique<const VectorGraphMemory>(std::move(data)))};
}

// Constructor given a filename. Reads the graph data into memory.
graph_tile_ptr GraphTile::Create(const std::string& tile_dir,
                                 const GraphId& graphid,
                                 std::unique_ptr<const GraphMemory>&& traffic_memory) {
  if (!graphid.Is_Valid()) {
    LOG_ERROR("Failed to build GraphTile. Error: GraphId is invalid");
    return nullptr;
  }

  if (graphid.level() > TileHierarchy::get_max_level()) {
    LOG_ERROR("Failed to build GraphTile. Error: GraphId level exceeds tile hierarchy max level");
    return nullptr;
  }

  if (tile_dir.empty()) {
    LOG_ERROR("Failed to build GraphTile. Error: Tile dir is empty");
    return nullptr;
  }

  // Open to the end of the file so we can immediately get size
  const std::string file_location =
      tile_dir + filesystem::path::preferred_separator + FileSuffix(graphid.Tile_Base());
  std::ifstream file(file_location, std::ios::in | std::ios::binary | std::ios::ate);
  if (file.is_open()) {
    // Read binary file into memory. TODO - protect against failure to allocate memory
    size_t filesize = file.tellg();

    std::vector<char> data(filesize);
    file.seekg(0, std::ios::beg);
    file.read(data.data(), filesize);
    file.close();
    return graph_tile_ptr{new GraphTile(graphid,
                                        std::make_unique<const VectorGraphMemory>(std::move(data)),
                                        std::move(traffic_memory))};
  }

  // Try to load a gzipped tile
  std::ifstream gz_file(file_location + ".gz", std::ios::in | std::ios::binary | std::ios::ate);
  if (gz_file.is_open()) {
    // Read the compressed file into memory
    size_t filesize = gz_file.tellg();
    gz_file.seekg(0, std::ios::beg);
    std::vector<char> compressed(filesize);
    gz_file.read(&compressed[0], filesize);
    gz_file.close();
    return DecompressTile(graphid, std::move(compressed));
  }

  // Nothing to load anywhere
  return nullptr;
}

graph_tile_ptr GraphTile::Create(const GraphId& graphid, std::vector<char>&& memory) {
  return graph_tile_ptr{
      new GraphTile(graphid, std::make_unique<const VectorGraphMemory>(std::move(memory)))};
}

graph_tile_ptr GraphTile::Create(const GraphId& graphid,
                                 std::unique_ptr<const GraphMemory>&& memory,
                                 std::unique_ptr<const GraphMemory>&& traffic_memory) {
  return graph_tile_ptr{new GraphTile(graphid, std::move(memory), std::move(traffic_memory))};
}

// the right c-tor for GraphTile
GraphTile::GraphTile(const GraphId& graphid,
                     std::unique_ptr<const GraphMemory> memory,
                     std::unique_ptr<const GraphMemory> traffic_memory)
    : header_(nullptr), traffic_tile(std::move(traffic_memory)) {
  // Initialize the internal tile data structures using a pointer to the
  // tile and the tile size
  memory_ = std::move(memory);
  Initialize(graphid);
}

GraphTile::GraphTile(const std::string& tile_dir,
                     const GraphId& graphid,
                     std::unique_ptr<const GraphMemory>&& traffic_memory) {
  // const_cast is only ok here because Create actually makes a new non-const GraphTile
  // which is then coerced to const via the template parameter of the managed pointer
  if (auto tile = Create(tile_dir, graphid, std::move(traffic_memory))) {
    *this = std::move(const_cast<GraphTile&>(*tile));
  }
}

GraphTile::GraphTile() = default;

void GraphTile::SaveTileToFile(const std::vector<char>& tile_data, const std::string& disk_location) {
  // At first we save tile to a temporary file and then move it
  // so we can avoid cases when another thread could read partially written file.
  auto dir = filesystem::path(disk_location);
  dir.replace_filename("");

  bool success = true;
  filesystem::path tmp_location;
  if (filesystem::create_directories(dir)) {
    // Technically this is a race condition but its super unlikely (famous last words)
    while (tmp_location.string().empty() || filesystem::exists(tmp_location))
      tmp_location = disk_location + GenerateTmpSuffix();
    std::ofstream file(tmp_location.string(), std::ios::out | std::ios::binary | std::ios::ate);
    file.write(tile_data.data(), tile_data.size());
    file.close();
    if (file.fail())
      success = false;
    int err = std::rename(tmp_location.c_str(), disk_location.c_str());
    if (err)
      success = false;
  } else {
    LOG_ERROR("Failed to create directory " + disk_location);
  }

  if (!success)
    filesystem::remove(tmp_location);
}

void store(const std::string& cache_location,
           const GraphId& graphid,
           const tile_getter_t* tile_getter,
           const std::vector<char>& raw_data) {
  if (!cache_location.empty()) {
    auto suffix =
        valhalla::baldr::GraphTile::FileSuffix(graphid.Tile_Base(),
                                               (tile_getter->gzipped()
                                                    ? valhalla::baldr::SUFFIX_COMPRESSED
                                                    : valhalla::baldr::SUFFIX_NON_COMPRESSED));
    auto disk_location = cache_location + filesystem::path::preferred_separator + suffix;
    filesystem::save(disk_location, raw_data);
  }
}

graph_tile_ptr GraphTile::CacheTileURL(const std::string& tile_url,
                                       const GraphId& graphid,
                                       tile_getter_t* tile_getter,
                                       const std::string& cache_location) {
  // Don't bother with invalid ids
  if (!graphid.Is_Valid() || graphid.level() > TileHierarchy::get_max_level() || !tile_getter) {
    return nullptr;
  }

  auto fname = valhalla::baldr::GraphTile::FileSuffix(graphid.Tile_Base(),
                                                      valhalla::baldr::SUFFIX_NON_COMPRESSED, false);
  auto result = tile_getter->get(baldr::make_single_point_url(tile_url, fname));
  if (result.status_ != tile_getter_t::status_code_t::SUCCESS) {
    return nullptr;
  }
  // try to cache it on disk so we dont have to keep fetching it from url
  store(cache_location, graphid, tile_getter, result.bytes_);

  // turn the memory into a tile
  if (tile_getter->gzipped()) {
    return DecompressTile(graphid, result.bytes_);
  }

  return graph_tile_ptr{
      new GraphTile(graphid, std::make_unique<const VectorGraphMemory>(std::move(result.bytes_)))};
}

GraphTile::~GraphTile() = default;

// Set pointers to internal tile data structures
void GraphTile::Initialize(const GraphId& graphid) {
  if (!memory_) {
    throw std::runtime_error("Missing tile data");
  }

  char* const tile_ptr = memory_->data;
  const size_t tile_size = memory_->size;

  if (tile_size < sizeof(GraphTileHeader)) {
    throw std::runtime_error("Invalid tile data size = " + std::to_string(tile_size) +
                             ". Tile file might me corrupted");
  }

  char* ptr = tile_ptr;
  header_ = reinterpret_cast<GraphTileHeader*>(ptr);
  ptr += sizeof(GraphTileHeader);

  if (header_->end_offset() != tile_size)
    throw std::runtime_error("Mismatch in end offset = " + std::to_string(header_->end_offset()) +
                             " vs raw tile data size = " + std::to_string(tile_size) +
                             ". Tile file might me corrupted");

  // TODO check version

  // Set a pointer to the node list
  nodes_ = reinterpret_cast<NodeInfo*>(ptr);
  ptr += header_->nodecount() * sizeof(NodeInfo);

  // Set a pointer to the node transition list
  transitions_ = reinterpret_cast<NodeTransition*>(ptr);
  ptr += header_->transitioncount() * sizeof(NodeTransition);

  // Set a pointer to the directed edge list
  directededges_ = reinterpret_cast<DirectedEdge*>(ptr);
  ptr += header_->directededgecount() * sizeof(DirectedEdge);

  // Extended directed edge attribution (if available).
  if (header_->has_ext_directededge()) {
    ext_directededges_ = reinterpret_cast<DirectedEdgeExt*>(ptr);
    ptr += header_->directededgecount() * sizeof(DirectedEdgeExt);
  }

  // Set a pointer access restriction list
  access_restrictions_ = reinterpret_cast<AccessRestriction*>(ptr);
  ptr += header_->access_restriction_count() * sizeof(AccessRestriction);

  // Set a pointer to the transit departure list
  departures_ = reinterpret_cast<TransitDeparture*>(ptr);
  ptr += header_->departurecount() * sizeof(TransitDeparture);

  // Set a pointer to the transit stop list
  transit_stops_ = reinterpret_cast<TransitStop*>(ptr);
  ptr += header_->stopcount() * sizeof(TransitStop);

  // Set a pointer to the transit route list
  transit_routes_ = reinterpret_cast<TransitRoute*>(ptr);
  ptr += header_->routecount() * sizeof(TransitRoute);

  // Set a pointer to the transit schedule list
  transit_schedules_ = reinterpret_cast<TransitSchedule*>(ptr);
  ptr += header_->schedulecount() * sizeof(TransitSchedule);

  // Set a pointer to the transit transfer list
  transit_transfers_ = reinterpret_cast<TransitTransfer*>(ptr);
  ptr += header_->transfercount() * sizeof(TransitTransfer);

  // Set a pointer to the sign list
  signs_ = reinterpret_cast<Sign*>(ptr);
  ptr += header_->signcount() * sizeof(Sign);

  // Start of turn lane data.
  turnlanes_ = reinterpret_cast<TurnLanes*>(ptr);
  ptr += header_->turnlane_count() * sizeof(TurnLanes);

  // Set a pointer to the admininstrative information list
  admins_ = reinterpret_cast<Admin*>(ptr);
  ptr += header_->admincount() * sizeof(Admin);

  // Set a pointer to the edge bin list
  edge_bins_ = reinterpret_cast<GraphId*>(ptr);

  // Start of forward restriction information and its size
  complex_restriction_forward_ = tile_ptr + header_->complex_restriction_forward_offset();
  complex_restriction_forward_size_ =
      header_->complex_restriction_reverse_offset() - header_->complex_restriction_forward_offset();

  // Start of reverse restriction information and its size
  complex_restriction_reverse_ = tile_ptr + header_->complex_restriction_reverse_offset();
  complex_restriction_reverse_size_ =
      header_->edgeinfo_offset() - header_->complex_restriction_reverse_offset();

  // Start of edge information and its size
  edgeinfo_ = tile_ptr + header_->edgeinfo_offset();
  edgeinfo_size_ = header_->textlist_offset() - header_->edgeinfo_offset();

  // Start of text list and its size
  textlist_ = tile_ptr + header_->textlist_offset();
  textlist_size_ = header_->lane_connectivity_offset() - header_->textlist_offset();

  // Start of lane connections and their size
  lane_connectivity_ =
      reinterpret_cast<LaneConnectivity*>(tile_ptr + header_->lane_connectivity_offset());
  lane_connectivity_size_ = header_->predictedspeeds_offset() - header_->lane_connectivity_offset();

  // Start of predicted speed data.
  if (header_->predictedspeeds_count() > 0) {
    char* ptr1 = tile_ptr + header_->predictedspeeds_offset();
    char* ptr2 = ptr1 + (header_->directededgecount() * sizeof(int32_t));
    predictedspeeds_.set_offset(reinterpret_cast<uint32_t*>(ptr1));
    predictedspeeds_.set_profiles(reinterpret_cast<int16_t*>(ptr2));

    lane_connectivity_size_ = header_->predictedspeeds_offset() - header_->lane_connectivity_offset();
  } else {
    lane_connectivity_size_ = header_->end_offset() - header_->lane_connectivity_offset();
  }

  // For reference - how to use the end offset to set size of an object (that
  // is not fixed size and count).
  // example_size_ = header_->end_offset() - header_->example_offset();

  // ANY NEW EXPANSION DATA GOES HERE

  // Associate one stop Ids for transit tiles
  if (graphid.level() == 3) {
    AssociateOneStopIds(graphid);
  }
}

// For transit tiles we need to save off the pair<tileid,lineid> lookup via
// onestop_ids.  This will be used for including or excluding transit lines
// for transit routes.  We save 2 maps because operators contain all of their
// route's tile_line pairs and it is used to include or exclude the operator
// as a whole. Also associates stops.
void GraphTile::AssociateOneStopIds(const GraphId& graphid) {
  // Associate stop Ids
  stop_one_stops.reserve(header_->stopcount());
  for (uint32_t i = 0; i < header_->stopcount(); i++) {
    const auto& stop = GetName(transit_stops_[i].one_stop_offset());
    stop_one_stops[stop] = {graphid.tileid(), graphid.level(), i};
  }

  // Associate route and operator Ids
  auto deps = GetTransitDepartures();
  for (auto const& dep : deps) {
    const auto* t = GetTransitRoute(dep.second->routeindex());
    const auto& route_one_stop = GetName(t->one_stop_offset());
    auto stops = route_one_stops.find(route_one_stop);
    if (stops == route_one_stops.end()) {
      std::list<GraphId> tile_line_ids;
      tile_line_ids.emplace_back(graphid.tileid(), graphid.level(), dep.second->lineid());
      route_one_stops[route_one_stop] = tile_line_ids;
    } else {
      route_one_stops[route_one_stop].emplace_back(graphid.tileid(), graphid.level(),
                                                   dep.second->lineid());
    }

    // operators contain all of their route's tile_line pairs.
    const auto& op_one_stop = GetName(t->op_by_onestop_id_offset());
    stops = oper_one_stops.find(op_one_stop);
    if (stops == oper_one_stops.end()) {
      std::list<GraphId> tile_line_ids;
      tile_line_ids.emplace_back(graphid.tileid(), graphid.level(), dep.second->lineid());
      oper_one_stops[op_one_stop] = tile_line_ids;
    } else {
      oper_one_stops[op_one_stop].emplace_back(graphid.tileid(), graphid.level(),
                                               dep.second->lineid());
    }
  }
}

std::string GraphTile::FileSuffix(const GraphId& graphid,
                                  const std::string& fname_suffix,
                                  bool is_file_path,
                                  const TileLevel* tiles) {
  /*
  if you have a graphid where level == 8 and tileid == 24134109851 you should get:
  8/024/134/109/851.gph since the number of levels is likely to be very small this limits the total
  number of objects in any one directory to 1000 which is an empirically derived good choice for
  mechanical hard drives this should be fine for s3 as well (even though it breaks the rule of most
  unique part of filename first) because there will be just so few objects in general in practice
  */

  // figure the largest id for this level
  if ((tiles && tiles->level != graphid.level()) ||
      (!tiles && graphid.level() >= TileHierarchy::levels().size() &&
       graphid.level() != TileHierarchy::GetTransitLevel().level)) {
    throw std::runtime_error("Could not compute FileSuffix for GraphId with invalid level: " +
                             std::to_string(graphid));
  }

  // get the level info
  const auto& level = tiles ? *tiles
                            : (graphid.level() == TileHierarchy::GetTransitLevel().level
                                   ? TileHierarchy::GetTransitLevel()
                                   : TileHierarchy::levels()[graphid.level()]);

  // figure out how many digits in tile-id
  const uint32_t max_id = static_cast<uint32_t>(level.tiles.ncolumns() * level.tiles.nrows() - 1);

  if (graphid.tileid() > max_id) {
    throw std::runtime_error("Could not compute FileSuffix for GraphId with invalid tile id:" +
                             std::to_string(graphid));
  }
  size_t max_length = static_cast<size_t>(std::log10(std::max(1u, max_id))) + 1;
  const size_t remainder = max_length % 3;
  if (remainder) {
    max_length += 3 - remainder;
  }
  assert(max_length % 3 == 0);

  // Calculate tile-id string length with separators
  const size_t tile_id_strlen = max_length + max_length / 3;
  assert(tile_id_strlen % 4 == 0);

  const char separator = is_file_path ? filesystem::path::preferred_separator : '/';

  std::string tile_id_str(tile_id_strlen, '0');
  size_t ind = tile_id_strlen - 1;
  for (uint32_t tile_id = graphid.tileid(); tile_id != 0; tile_id /= 10) {
    tile_id_str[ind--] = '0' + static_cast<char>(tile_id % 10);
    if ((tile_id_strlen - ind) % 4 == 0) {
      ind--; // skip an additional character to leave space for separators
    }
  }
  // add separators
  for (size_t sep_ind = 0; sep_ind < tile_id_strlen; sep_ind += 4) {
    tile_id_str[sep_ind] = separator;
  }

  return std::to_string(graphid.level()) + tile_id_str + fname_suffix;
}

// Get the tile Id given the full path to the file.
GraphId GraphTile::GetTileId(const std::string& fname) {
  std::unordered_set<std::string::value_type> allowed{filesystem::path::preferred_separator,
                                                      '0',
                                                      '1',
                                                      '2',
                                                      '3',
                                                      '4',
                                                      '5',
                                                      '6',
                                                      '7',
                                                      '8',
                                                      '9'};
  // we require slashes
  auto pos = fname.find_last_of(filesystem::path::preferred_separator);
  if (pos == fname.npos) {
    throw std::runtime_error("Invalid tile path: " + fname);
  }

  // swallow numbers until you reach the end or a dot
  for (; pos < fname.size(); ++pos) {
    if (allowed.find(fname[pos]) == allowed.cend()) {
      break;
    }
  }
  allowed.erase(static_cast<std::string::value_type>(filesystem::path::preferred_separator));

  // if you didnt reach the end and it wasnt a dot then this isnt valid
  if (pos != fname.size() && fname[pos] != '.') {
    throw std::runtime_error("Invalid tile path: " + fname);
  }

  // run backwards while you find an allowed char but stop if not 3 digits between slashes
  std::vector<uint32_t> digits;
  auto last = pos;
  while (--pos < last) {
    auto c = fname[pos];
    // invalid char showed up
    if (allowed.find(c) == allowed.cend()) {
      throw std::runtime_error("Invalid tile path: " + fname);
    }

    // if its the last thing or the next one is a separator thats another digit
    if (pos == 0 || fname[pos - 1] == filesystem::path::preferred_separator) {
      // this is not 3 or 1 digits so its wrong
      auto dist = last - pos;
      if (dist != 3 && dist != 1) {
        throw std::runtime_error("Invalid tile path: " + fname);
      }
      // we'll keep this
      auto i = atoi(fname.substr(pos, dist).c_str());
      digits.push_back(i);
      // and we'll stop if it was the level (always a single digit see GraphId)
      if (dist == 1) {
        break;
      }
      // next
      last = --pos;
    }
  }

  // if the first thing isnt a valid level bail
  if (digits.back() >= TileHierarchy::levels().size() &&
      digits.back() != TileHierarchy::GetTransitLevel().level) {
    throw std::runtime_error("Invalid tile path: " + fname);
  }

  // get the level info
  uint32_t level = digits.back();
  digits.pop_back();
  const auto& tile_level = level == TileHierarchy::GetTransitLevel().level
                               ? TileHierarchy::GetTransitLevel()
                               : TileHierarchy::levels()[level];

  // get the number of sub directories that we should have
  uint32_t max_id = static_cast<uint32_t>(tile_level.tiles.ncolumns() * tile_level.tiles.nrows() - 1);
  size_t parts = static_cast<size_t>(std::log10(std::max(1u, max_id))) + 1;
  if (parts % 3 != 0) {
    parts += 3 - (parts % 3);
  }
  parts /= 3;

  // bail if its the wrong number of sub dirs
  if (digits.size() != parts) {
    throw std::runtime_error("Invalid tile path: " + fname);
  }

  // parse the id of the tile
  int multiplier = 1;
  uint32_t id = 0;
  for (auto digit : digits) {
    id += digit * multiplier;
    multiplier *= 1000;
  }

  // if after parsing them the number is out of bounds bail
  if (id > max_id) {
    throw std::runtime_error("Invalid tile path: " + fname);
  }

  // you've passed the test enjoy your id
  return {id, level, 0};
}

// Get the bounding box of this graph tile.
AABB2<PointLL> GraphTile::BoundingBox() const {
  const auto& tiles = header_->graphid().level() == TileHierarchy::GetTransitLevel().level
                          ? TileHierarchy::GetTransitLevel().tiles
                          : TileHierarchy::levels()[header_->graphid().level()].tiles;
  return tiles.TileBounds(header_->graphid().tileid());
}

iterable_t<const DirectedEdge> GraphTile::GetDirectedEdges(const NodeInfo* node) const {
  if (node < nodes_ || node >= nodes_ + header_->nodecount()) {
    throw std::logic_error(
        std::string(__FILE__) + ":" + std::to_string(__LINE__) +
        " GraphTile NodeInfo out of bounds: " + std::to_string(header_->graphid()));
  }
  const auto* edge = directededges_ + node->edge_index();
  return iterable_t<const DirectedEdge>{edge, node->edge_count()};
}

iterable_t<const DirectedEdge> GraphTile::GetDirectedEdges(const GraphId& node) const {
  if (node.Tile_Base() != header_->graphid() || node.id() >= header_->nodecount()) {
    throw std::logic_error(
        std::string(__FILE__) + ":" + std::to_string(__LINE__) +
        " GraphTile NodeInfo index out of bounds: " + std::to_string(node.tileid()) + "," +
        std::to_string(node.level()) + "," + std::to_string(node.id()) +
        " nodecount= " + std::to_string(header_->nodecount()));
  }
  const auto* nodeinfo = nodes_ + node.id();
  return GetDirectedEdges(nodeinfo);
}

iterable_t<const DirectedEdge> GraphTile::GetDirectedEdges(const size_t idx) const {
  if (idx >= header_->nodecount()) {
    throw std::logic_error(
        std::string(__FILE__) + ":" + std::to_string(__LINE__) +
        " GraphTile NodeInfo index out of bounds 5: " + std::to_string(header_->graphid().tileid()) +
        "," + std::to_string(header_->graphid().level()) + "," + std::to_string(idx) +
        " nodecount= " + std::to_string(header_->nodecount()));
  }
  const auto& nodeinfo = nodes_[idx];
  const auto* edge = directededge(nodeinfo.edge_index());
  return iterable_t<const DirectedEdge>{edge, nodeinfo.edge_count()};
}

iterable_t<const DirectedEdgeExt> GraphTile::GetDirectedEdgeExts(const NodeInfo* node) const {
  if (node < nodes_ || node >= nodes_ + header_->nodecount()) {
    throw std::logic_error(
        std::string(__FILE__) + ":" + std::to_string(__LINE__) +
        " GraphTile NodeInfo out of bounds: " + std::to_string(header_->graphid()));
  }
  const auto* edge_ext = ext_directededges_ + node->edge_index();
  return iterable_t<const DirectedEdgeExt>{edge_ext, node->edge_count()};
}

iterable_t<const DirectedEdgeExt> GraphTile::GetDirectedEdgeExts(const GraphId& node) const {
  if (node.Tile_Base() != header_->graphid() || node.id() >= header_->nodecount()) {
    throw std::logic_error(
        std::string(__FILE__) + ":" + std::to_string(__LINE__) +
        " GraphTile NodeInfo index out of bounds: " + std::to_string(node.tileid()) + "," +
        std::to_string(node.level()) + "," + std::to_string(node.id()) +
        " nodecount= " + std::to_string(header_->nodecount()));
  }
  const auto* nodeinfo = nodes_ + node.id();
  return GetDirectedEdgeExts(nodeinfo);
}

iterable_t<const DirectedEdgeExt> GraphTile::GetDirectedEdgeExts(const size_t idx) const {
  if (idx >= header_->nodecount()) {
    throw std::logic_error(
        std::string(__FILE__) + ":" + std::to_string(__LINE__) +
        " GraphTile NodeInfo index out of bounds 5: " + std::to_string(header_->graphid().tileid()) +
        "," + std::to_string(header_->graphid().level()) + "," + std::to_string(idx) +
        " nodecount= " + std::to_string(header_->nodecount()));
  }
  const auto& nodeinfo = nodes_[idx];
  const auto* edge_ext = ext_directededge(nodeinfo.edge_index());
  return iterable_t<const DirectedEdgeExt>{edge_ext, nodeinfo.edge_count()};
}

EdgeInfo GraphTile::edgeinfo(const DirectedEdge* edge) const {
  return EdgeInfo(edgeinfo_ + edge->edgeinfo_offset(), textlist_, textlist_size_);
}

// Get the complex restrictions in the forward or reverse order based on
// the id and modes.
std::vector<ComplexRestriction*>
GraphTile::GetRestrictions(const bool forward, const GraphId id, const uint64_t modes) const {
  size_t offset = 0;
  std::vector<ComplexRestriction*> cr_vector;
  if (forward) {
    while (offset < complex_restriction_forward_size_) {
      ComplexRestriction* cr =
          reinterpret_cast<ComplexRestriction*>(complex_restriction_forward_ + offset);
      if (cr->to_graphid() == id && (cr->modes() & modes)) {
        cr_vector.push_back(cr);
      }
      offset += cr->SizeOf();
    }
  } else {
    while (offset < complex_restriction_reverse_size_) {
      ComplexRestriction* cr =
          reinterpret_cast<ComplexRestriction*>(complex_restriction_reverse_ + offset);
      if (cr->from_graphid() == id && (cr->modes() & modes)) {
        cr_vector.push_back(cr);
      }
      offset += cr->SizeOf();
    }
  }
  return cr_vector;
}

// Get the directed edges outbound from the specified node index.
const DirectedEdge*
GraphTile::GetDirectedEdges(const uint32_t node_index, uint32_t& count, uint32_t& edge_index) const {
  const NodeInfo* nodeinfo = node(node_index);
  count = nodeinfo->edge_count();
  edge_index = nodeinfo->edge_index();
  return directededge(nodeinfo->edge_index());
}

// Get the directed edge extensions outbound from the specified node index.
const DirectedEdgeExt* GraphTile::GetDirectedEdgeExts(const uint32_t node_index,
                                                      uint32_t& count,
                                                      uint32_t& edge_index) const {
  const NodeInfo* nodeinfo = node(node_index);
  count = nodeinfo->edge_count();
  edge_index = nodeinfo->edge_index();
  return ext_directededge(nodeinfo->edge_index());
}

// Convenience method to get the names for an edge
std::vector<std::string> GraphTile::GetNames(const DirectedEdge* edge) const {
  return edgeinfo(edge).GetNames();
}

uint16_t GraphTile::GetTypes(const DirectedEdge* edge) const {
  return edgeinfo(edge).GetTypes();
}

// Get the admininfo at the specified index.
AdminInfo GraphTile::admininfo(const size_t idx) const {
  if (idx < header_->admincount()) {
    const Admin& admin = admins_[idx];
    return AdminInfo(textlist_ + admin.country_offset(), textlist_ + admin.state_offset(),
                     admin.country_iso(), admin.state_iso());
  }
  throw std::runtime_error("GraphTile AdminInfo index out of bounds");
}

// Get the admin at the specified index.
const Admin* GraphTile::admin(const size_t idx) const {
  if (idx < header_->admincount()) {
    return &admins_[idx];
  }
  throw std::runtime_error("GraphTile Admin index out of bounds");
}

// Convenience method to get the text/name for a given offset to the textlist
std::string GraphTile::GetName(const uint32_t textlist_offset) const {
  if (textlist_offset < textlist_size_) {
    return textlist_ + textlist_offset;
  } else {
    throw std::runtime_error("GetName: offset exceeds size of text list");
  }
}

// Return the signs for a given directed edge or node index.
std::vector<SignInfo> GraphTile::GetSigns(const uint32_t idx, bool signs_on_node) const {
  const int32_t count = header_->signcount();
  std::vector<SignInfo> signs;
  if (count == 0) {
    return signs;
  }

  // Signs are sorted by edge index.
  // Binary search to find a sign with matching edge index.
  int32_t low = 0;
  int32_t high = count - 1;
  int32_t mid;
  int32_t found = count;
  while (low <= high) {
    mid = (low + high) / 2;
    const auto& sign = signs_[mid];
    // matching edge index
    if (idx == sign.index()) {
      found = mid;
      high = mid - 1;
    } // need a smaller index
    else if (idx < sign.index()) {
      high = mid - 1;
    } // need a bigger index
    else {
      low = mid + 1;
    }
  }

  // Add signs
  for (; found < count && signs_[found].index() == idx; ++found) {
    if (signs_[found].text_offset() < textlist_size_) {

      const char* text = (textlist_ + signs_[found].text_offset());

      bool isLinguistic = (signs_[found].type() == Sign::Type::kLinguistic);

      bool is_node_sign_type = signs_[found].type() == Sign::Type::kJunctionName ||
                               signs_[found].type() == Sign::Type::kTollName;

      // only add named signs when asking for signs at the node and
      // only add edge signs when asking for signs at the edges.
      // is_route_num_type indicates if this phonome is for a node or not; therefore,
      // we only return a node phoneme when is_route_num_type and signs_on_node are both true
      // and we only return an edge phoneme when is_route_num_type and signs_on_node are both
      // false
      if (((is_node_sign_type || (isLinguistic && signs_[found].is_route_num_type())) &&
           signs_on_node) ||
          (((!is_node_sign_type && !isLinguistic) ||
            (isLinguistic && !signs_[found].is_route_num_type())) &&
           !signs_on_node)) {
        std::string sign_text = text;
        if (isLinguistic) {
          sign_text.clear();
          while (*text != '\0') {
            if (signs_[found].type() == Sign::Type::kLinguistic) {
              const auto header = midgard::unaligned_read<linguistic_text_header_t>(text);
              sign_text.append(
                  std::string(reinterpret_cast<const char*>(&header), kLinguisticHeaderSize) +
                  std::string((text + kLinguisticHeaderSize), header.length_));

              text += header.length_ + kLinguisticHeaderSize;
            }
          }
        }

        signs.emplace_back(signs_[found].type(), signs_[found].is_route_num_type(),
                           signs_[found].tagged(), false, 0, 0, sign_text);
      }
    } else {
      throw std::runtime_error("GetSigns: offset exceeds size of text list");
    }
  }

  if (signs.size() == 0) {
    LOG_ERROR("No signs found for idx = " + std::to_string(idx));
  }

  return signs;
}

// Convenience method to get the signs for an edge given the
// directed edge index.
std::vector<SignInfo> GraphTile::GetSigns(
    const uint32_t idx,
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>& index_linguistic_map,
    bool signs_on_node) const {
  const int32_t count = header_->signcount();
  std::vector<SignInfo> signs;
  if (count == 0) {
    return signs;
  }

  // Signs are sorted by edge index.
  // Binary search to find a sign with matching edge index.
  int32_t low = 0;
  int32_t high = count - 1;
  int32_t mid;
  int32_t found = count;
  while (low <= high) {
    mid = (low + high) / 2;
    const auto& sign = signs_[mid];
    // matching edge index
    if (idx == sign.index()) {
      found = mid;
      high = mid - 1;
    } // need a smaller index
    else if (idx < sign.index()) {
      high = mid - 1;
    } // need a bigger index
    else {
      low = mid + 1;
    }
  }

  // Add signs
  for (; found < count && signs_[found].index() == idx; ++found) {
    if (signs_[found].text_offset() < textlist_size_) {

      const auto* text = (textlist_ + signs_[found].text_offset());
      if (signs_[found].tagged() && signs_[found].type() == Sign::Type::kLinguistic) {

        // is_route_num_type indicates if this phonome is for a node or not
        if ((signs_[found].is_route_num_type() && signs_on_node) ||
            (!signs_[found].is_route_num_type() && !signs_on_node)) {
          while (*text != '\0') {
            std::tuple<uint8_t, uint8_t, std::string> liguistic_attributes;
            uint8_t name_index = 0;
            if (signs_[found].type() == Sign::Type::kLinguistic) {
              const auto header = midgard::unaligned_read<linguistic_text_header_t>(text);

              std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(liguistic_attributes) =
                  header.phonetic_alphabet_;
              std::get<kLinguisticMapTupleLanguageIndex>(liguistic_attributes) = header.language_;

              std::get<kLinguisticMapTuplePronunciationIndex>(liguistic_attributes) =
                  std::string(text + kLinguisticHeaderSize, header.length_);
              text += header.length_ + kLinguisticHeaderSize;
              name_index = header.name_index_;

            } else
              continue;

            // Edge case.  Sometimes when phonemes exist but the language for that phoneme is not
            // supported in that area, we toss the phoneme but add the default language for that
            // name/destination key.  We only want to return the highest ranking phoneme type
            // over the language.
            auto iter = index_linguistic_map.insert(std::make_pair(name_index, liguistic_attributes));
            if (!iter.second) {
              if ((std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(liguistic_attributes) >
                   std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter.first->second)) &&
                  (std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(liguistic_attributes) !=
                   static_cast<uint8_t>(PronunciationAlphabet::kNone)) &&
                  (std::get<kLinguisticMapTupleLanguageIndex>(liguistic_attributes) ==
                   std::get<kLinguisticMapTupleLanguageIndex>(iter.first->second))) {
                iter.first->second = liguistic_attributes;
              }
            }
          }
        }
        continue;
      }

      bool is_node_sign_type = signs_[found].type() == Sign::Type::kJunctionName ||
                               signs_[found].type() == Sign::Type::kTollName;

      // only add named signs when asking for signs at the node and
      // only add edge signs when asking for signs at the edges.
      if ((is_node_sign_type && signs_on_node) || (!is_node_sign_type && !signs_on_node))
        signs.emplace_back(signs_[found].type(), signs_[found].is_route_num_type(),
                           signs_[found].tagged(), false, 0, 0, text);
    } else {
      throw std::runtime_error("GetSigns: offset exceeds size of text list");
    }
  }

  if (signs.size() == 0) {
    LOG_ERROR("No signs found for idx = " + std::to_string(idx));
  }
  return signs;
}

// Get lane connections ending on this edge.
std::vector<LaneConnectivity> GraphTile::GetLaneConnectivity(const uint32_t idx) const {
  uint32_t count = lane_connectivity_size_ / sizeof(LaneConnectivity);
  std::vector<LaneConnectivity> lcs;
  if (count == 0) {
    LOG_ERROR("No lane connections found for idx = " + std::to_string(idx));
    return lcs;
  }

  // Lane connections are sorted by edge index.
  // Binary search to find a sign with matching edge index.
  int32_t low = 0;
  int32_t high = count - 1;
  int32_t mid;
  auto found = count;
  while (low <= high) {
    mid = (low + high) / 2;
    const auto& lc = lane_connectivity_[mid];
    // matching edge index
    if (idx == lc.to()) {
      found = mid;
      high = mid - 1;
    } // need a smaller index
    else if (idx < lc.to()) {
      high = mid - 1;
    } // need a bigger index
    else {
      low = mid + 1;
    }
  }

  // Add Lane connections
  for (; found < count && lane_connectivity_[found].to() == idx; ++found) {
    lcs.emplace_back(lane_connectivity_[found]);
  }
  if (lcs.size() == 0) {
    LOG_ERROR("No lane connections found for idx = " + std::to_string(idx));
  }
  return lcs;
}

// Get the next departure given the directed line Id and the current
// time (seconds from midnight).
const TransitDeparture* GraphTile::GetNextDeparture(const uint32_t lineid,
                                                    const uint32_t current_time,
                                                    const uint32_t day,
                                                    const uint32_t dow,
                                                    bool date_before_tile,
                                                    bool wheelchair,
                                                    bool bicycle) const {
  uint32_t count = header_->departurecount();
  if (count == 0) {
    return nullptr;
  }

  // Departures are sorted by edge Id and then by departure time.
  // Binary search to find a departure with matching line Id.
  int32_t low = 0;
  int32_t high = count - 1;
  int32_t mid;
  auto found = count;
  while (low <= high) {
    mid = (low + high) / 2;
    const auto& dep = departures_[mid];
    // matching lineid and a workable time
    if (lineid == dep.lineid() &&
        ((current_time <= dep.departure_time() && dep.type() == kFixedSchedule) ||
         (current_time <= dep.end_time() && dep.type() == kFrequencySchedule))) {
      found = mid;
      high = mid - 1;
    } // need a smaller lineid
    else if (lineid < dep.lineid()) {
      high = mid - 1;
    } // either need a bigger lineid or a later time
    else {
      low = mid + 1;
    }
  }

  // Iterate through departures until one is found with valid date, dow or
  // calendar date, and does not have a calendar exception.
  for (; found < count && departures_[found].lineid() == lineid; ++found) {
    // Make sure it falls within the schedule and departure props are valid
    const auto& d = departures_[found];
    if ((wheelchair && !d.wheelchair_accessible()) || (bicycle && !d.bicycle_accessible()) ||
        !GetTransitSchedule(d.schedule_index())->IsValid(day, dow, date_before_tile)) {
      continue;
    }

    if (d.type() == kFixedSchedule) {
      return &d;
    } else {
      // TODO: this is for now only respecting frequencies.txt exact_times=true, e.g.
      // auto departure_time = kFrequencySchedule ? d.departure_time() : d.departure_time() +
      // (d.frequency() * 0.5f);
      auto departure_time = d.departure_time();
      const auto end_time = d.end_time();
      const auto frequency = d.frequency();
      // make sure the departure time is after the current_time for a frequency based trip
      while (departure_time < current_time && departure_time < end_time) {
        departure_time += frequency;
      }

      // make a new departure with a guess for departure time          ;
      return new TransitDeparture(d.lineid(), d.tripid(), d.routeindex(), d.blockid(),
                                  d.headsign_offset(), departure_time, d.end_time(), d.frequency(),
                                  d.elapsed_time(), d.schedule_index(), d.wheelchair_accessible(),
                                  d.bicycle_accessible());
    }
  }

  // TODO - maybe wrap around, try next day?
  LOG_DEBUG("No more departures found for lineid = " + std::to_string(lineid) +
            " current_time = " + std::to_string(current_time));
  return nullptr;
}

// Get the departure given the line Id and tripid
const TransitDeparture* GraphTile::GetTransitDeparture(const uint32_t lineid,
                                                       const uint32_t tripid,
                                                       const uint32_t current_time) const {
  uint32_t count = header_->departurecount();
  if (count == 0) {
    return nullptr;
  }

  // Departures are sorted by edge Id and then by departure time.
  // Binary search to find a departure with matching line Id.
  int32_t low = 0;
  int32_t high = count - 1;
  int32_t mid;
  auto found = count;
  while (low <= high) {
    mid = (low + high) / 2;
    const auto& dep = departures_[mid];
    // find the first matching lineid in the list
    if (lineid == dep.lineid() &&
        ((current_time <= dep.departure_time() && dep.type() == kFixedSchedule) ||
         (current_time <= dep.end_time() && dep.type() == kFrequencySchedule))) {
      found = mid;
      high = mid - 1;
    } // need a smaller lineid
    else if (lineid < dep.lineid()) {
      high = mid - 1;
    } // need a bigger lineid
    else {
      low = mid + 1;
    }
  }

  // Iterate through departures until one is found with matching trip id
  for (; found < count && departures_[found].lineid() == lineid; ++found) {
    if (departures_[found].tripid() == tripid) {

      if (departures_[found].type() == kFixedSchedule) {
        return &departures_[found];
      }

      uint32_t departure_time = departures_[found].departure_time();
      uint32_t end_time = departures_[found].end_time();
      uint32_t frequency = departures_[found].frequency();
      while (departure_time < current_time && departure_time < end_time) {
        departure_time += frequency;
      }

      if (departure_time >= current_time && departure_time < end_time) {
        const auto& d = departures_[found];
        const TransitDeparture* dep =
            new TransitDeparture(d.lineid(), d.tripid(), d.routeindex(), d.blockid(),
                                 d.headsign_offset(), departure_time, d.end_time(), d.frequency(),
                                 d.elapsed_time(), d.schedule_index(), d.wheelchair_accessible(),
                                 d.bicycle_accessible());
        return dep;
      }
    }
  }

  LOG_INFO("No departures found for lineid = " + std::to_string(lineid) +
           " and tripid = " + std::to_string(tripid));
  return nullptr;
}

// Get a map of departures based on lineid.  No dups exist in the map.
std::unordered_map<uint32_t, TransitDeparture*> GraphTile::GetTransitDepartures() const {

  std::unordered_map<uint32_t, TransitDeparture*> deps;
  deps.reserve(header_->departurecount());

  for (uint32_t i = 0; i < header_->departurecount(); i++) {
    deps.insert({departures_[i].lineid(), &departures_[i]});
  }

  return deps;
}

// Get the stop onestop Ids in this tile.
const std::unordered_map<std::string, GraphId>& GraphTile::GetStopOneStops() const {
  return stop_one_stops;
}

// Get the route onestop Ids in this tile.
const std::unordered_map<std::string, std::list<GraphId>>& GraphTile::GetRouteOneStops() const {
  return route_one_stops;
}

// Get the operator onestop Ids in this tile.
const std::unordered_map<std::string, std::list<GraphId>>& GraphTile::GetOperatorOneStops() const {
  return oper_one_stops;
}

// Get the transit stop given its index within the tile.
const TransitStop* GraphTile::GetTransitStop(const uint32_t idx) const {
  uint32_t count = header_->stopcount();
  if (count == 0) {
    return nullptr;
  }

  if (idx < count) {
    return &transit_stops_[idx];
  }
  throw std::runtime_error("GraphTile Transit Stop index out of bounds");
}

// Get the transit route given its index within the tile.
const TransitRoute* GraphTile::GetTransitRoute(const uint32_t idx) const {
  uint32_t count = header_->routecount();
  if (count == 0) {
    return nullptr;
  }

  if (idx < count) {
    return &transit_routes_[idx];
  }
  throw std::runtime_error("GraphTile GetTransitRoute index out of bounds");
}

// Get the transit schedule given its schedule index.
const TransitSchedule* GraphTile::GetTransitSchedule(const uint32_t idx) const {
  uint32_t count = header_->schedulecount();
  if (count == 0) {
    return nullptr;
  }

  if (idx < count) {
    return &transit_schedules_[idx];
  }
  throw std::runtime_error("GraphTile GetTransitSchedule index out of bounds");
}

// Get the access restriction given its directed edge index
std::vector<AccessRestriction> GraphTile::GetAccessRestrictions(const uint32_t idx,
                                                                const uint32_t access) const {

  std::vector<AccessRestriction> restrictions;
  uint32_t count = header_->access_restriction_count();
  if (count == 0) {
    return restrictions;
  }

  // Access restriction are sorted by edge Id.
  // Binary search to find an access restriction with matching edge Id.
  int32_t low = 0;
  int32_t high = count - 1;
  int32_t mid;
  auto found = count;
  while (low <= high) {
    mid = (low + high) / 2;
    const auto& res = access_restrictions_[mid];
    // find the first matching index in the list
    if (idx == res.edgeindex()) {
      found = mid;
      high = mid - 1;
    } // need a smaller index
    else if (idx < res.edgeindex()) {
      high = mid - 1;
    } // need a bigger index
    else {
      low = mid + 1;
    }
  }

  // Add restrictions for only the access that we are interested in
  for (; found < count && access_restrictions_[found].edgeindex() == idx; ++found) {
    if (access_restrictions_[found].modes() & access) {
      restrictions.emplace_back(access_restrictions_[found]);
    }
  }

  return restrictions;
}

// Get the array of graphids for this bin
midgard::iterable_t<GraphId> GraphTile::GetBin(size_t column, size_t row) const {
  auto offsets = header_->bin_offset(column, row);
  return iterable_t<GraphId>{edge_bins_ + offsets.first, edge_bins_ + offsets.second};
}

midgard::iterable_t<GraphId> GraphTile::GetBin(size_t index) const {
  auto offsets = header_->bin_offset(index);
  return iterable_t<GraphId>{edge_bins_ + offsets.first, edge_bins_ + offsets.second};
}

// Get turn lanes for this edge.
uint32_t GraphTile::turnlanes_offset(const uint32_t idx) const {
  uint32_t count = header_->turnlane_count();
  if (count == 0) {
    LOG_ERROR("No turn lanes found for idx = " + std::to_string(idx));
    return 0;
  }
  auto tl = std::lower_bound(&turnlanes_[0], &turnlanes_[count], TurnLanes(idx, 0));
  return tl != &turnlanes_[count] ? tl->text_offset() : 0;
}

} // namespace baldr
} // namespace valhalla
