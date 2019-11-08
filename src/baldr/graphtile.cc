#include "baldr/graphtile.h"
#include "baldr/compression_utils.h"
#include "baldr/datetime.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"
#include "midgard/tiles.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <string>
#include <vector>

using namespace valhalla::midgard;

namespace {
struct dir_facet : public std::numpunct<char> {
protected:
  virtual char do_thousands_sep() const {
    return '/';
  }

  virtual std::string do_grouping() const {
    return "\03";
  }
};
const std::locale dir_locale(std::locale("C"), new dir_facet());
const AABB2<PointLL> world_box(PointLL(-180, -90), PointLL(180, 90));
constexpr float COMPRESSION_HINT = 3.5f;
} // namespace

namespace valhalla {
namespace baldr {

// Default constructor
GraphTile::GraphTile()
    : header_(nullptr), nodes_(nullptr), directededges_(nullptr), ext_directededges_(nullptr),
      transitions_(nullptr), departures_(nullptr), transit_stops_(nullptr), transit_routes_(nullptr),
      transit_schedules_(nullptr), transit_transfers_(nullptr), access_restrictions_(nullptr),
      signs_(nullptr), admins_(nullptr), edge_bins_(nullptr), complex_restriction_forward_(nullptr),
      complex_restriction_reverse_(nullptr), edgeinfo_(nullptr), textlist_(nullptr),
      complex_restriction_forward_size_(0), complex_restriction_reverse_size_(0), edgeinfo_size_(0),
      textlist_size_(0), lane_connectivity_(nullptr), lane_connectivity_size_(0),
      turnlanes_(nullptr) {
}

// Constructor given a filename. Reads the graph data into memory.
GraphTile::GraphTile(const std::string& tile_dir, const GraphId& graphid) : header_(nullptr) {

  // Don't bother with invalid ids
  if (!graphid.Is_Valid() || graphid.level() > TileHierarchy::get_max_level() || tile_dir.empty()) {
    return;
  }

  // Open to the end of the file so we can immediately get size
  std::string file_location =
      tile_dir + filesystem::path::preferred_separator + FileSuffix(graphid.Tile_Base());
  std::ifstream file(file_location, std::ios::in | std::ios::binary | std::ios::ate);
  if (file.is_open()) {
    // Read binary file into memory. TODO - protect against failure to
    // allocate memory
    size_t filesize = file.tellg();
    graphtile_.reset(new std::vector<char>(filesize));
    file.seekg(0, std::ios::beg);
    file.read(graphtile_->data(), filesize);
    file.close();

    // Set pointers to internal data structures
    Initialize(graphid, graphtile_->data(), graphtile_->size());
  } else {
    // try to load a gzipped tile
    std::ifstream file(file_location + ".gz", std::ios::in | std::ios::binary | std::ios::ate);
    if (file.is_open()) {
      // read the compressed file into memory
      size_t filesize = file.tellg();
      file.seekg(0, std::ios::beg);
      std::vector<char> compressed(filesize);
      file.read(&compressed[0], filesize);
      file.close();

      // try to decompress it
      DecompressTile(graphid, compressed);
    }
  }
}

bool GraphTile::DecompressTile(const GraphId& graphid, std::vector<char>& compressed) {
  // for setting where to read compressed data from
  auto src_func = [&compressed](z_stream& s) -> void {
    s.next_in = static_cast<Byte*>(static_cast<void*>(compressed.data()));
    s.avail_in = static_cast<unsigned int>(compressed.size());
  };

  // for setting where to write the uncompressed data to
  graphtile_.reset(new std::vector<char>(0, 0));
  auto dst_func = [this, &compressed](z_stream& s) -> int {
    // if the whole buffer wasn't used we are done
    auto size = graphtile_->size();
    if (s.total_out < size)
      graphtile_->resize(s.total_out);
    // we need more space
    else {
      // assume we need 3.5x the space
      graphtile_->resize(size + (compressed.size() * COMPRESSION_HINT));
      // set the pointer to the next spot
      s.next_out = static_cast<Byte*>(static_cast<void*>(graphtile_->data() + size));
      s.avail_out = compressed.size() * COMPRESSION_HINT;
    }
    return Z_NO_FLUSH;
  };

  // Decompress tile into memory
  if (!baldr::inflate(src_func, dst_func)) {
    LOG_ERROR("Failed to gunzip " + FileSuffix(graphid, true));
    graphtile_.reset();
    return false;
  }

  // Set pointers to internal data structures
  Initialize(graphid, graphtile_->data(), graphtile_->size());
  return true;
}

GraphTile::GraphTile(const GraphId& graphid, char* ptr, size_t size) : header_(nullptr) {
  // Initialize the internal tile data structures using a pointer to the
  // tile and the tile size
  Initialize(graphid, ptr, size);
}

std::string MakeSingleTileUrl(const std::string& tile_url, const GraphId& graphid) {
  auto id_pos = tile_url.find(GraphTile::kTilePathPattern);
  return tile_url.substr(0, id_pos) + GraphTile::FileSuffix(graphid.Tile_Base()) +
         tile_url.substr(id_pos + std::strlen(GraphTile::kTilePathPattern));
}

void GraphTile::SaveTileToFile(const std::vector<char>& tile_data, const std::string& disk_location) {
  // At first we save tile to a temporary file and then move it
  // so we can avoid cases when another thread could read partially written file.
  auto tmp_location = disk_location + boost::filesystem::unique_path().string();
  auto dir = filesystem::path(disk_location);
  dir.replace_filename("");

  bool success = true;
  if (filesystem::create_directories(dir)) {
    std::ofstream file(tmp_location, std::ios::out | std::ios::binary | std::ios::ate);
    file.write(tile_data.data(), tile_data.size());
    file.close();
    if (file.fail())
      success = false;
    int err = std::rename(tmp_location.c_str(), disk_location.c_str());
    if (err)
      success = false;
  }

  if (!success)
    filesystem::remove(tmp_location);
}

GraphTile GraphTile::CacheTileURL(const std::string& tile_url,
                                  const GraphId& graphid,
                                  curler_t& curler,
                                  bool gzipped,
                                  const std::string& cache_location) {
  // Don't bother with invalid ids
  if (!graphid.Is_Valid() || graphid.level() > TileHierarchy::get_max_level()) {
    return {};
  }

  auto uri = MakeSingleTileUrl(tile_url, graphid);
  long http_code;
  auto tile_data = curler(uri, http_code, gzipped);

  if (http_code != 200)
    return {};

  // try to cache it on disk so we dont have to keep fetching it from url
  if (!cache_location.empty()) {
    auto suffix = FileSuffix(graphid.Tile_Base(), gzipped);
    auto disk_location = cache_location + filesystem::path::preferred_separator + suffix;
    SaveTileToFile(tile_data, disk_location);
  }

  // turn the memory into a tile
  auto tile = GraphTile();
  if (gzipped) {
    tile.DecompressTile(graphid, tile_data);
  } // we dont need to decompress so just take ownership of the data
  else {
    tile.graphtile_.reset(new std::vector<char>(0, 0));
    *tile.graphtile_ = std::move(tile_data);
    tile.Initialize(graphid, tile.graphtile_->data(), tile.graphtile_->size());
  }

  return tile;
}

GraphTile::~GraphTile() {
}

// Set pointers to internal tile data structures
void GraphTile::Initialize(const GraphId& graphid, char* tile_ptr, const size_t tile_size) {
  if (tile_size < sizeof(GraphTileHeader))
    throw std::runtime_error("Invalid tile data size = " + std::to_string(tile_size) +
                             ". Tile file might me corrupted");

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
    const auto* t = GetTransitRoute(dep.second->routeid());
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

std::string GraphTile::FileSuffix(const GraphId& graphid, bool gzipped) {
  /*
  if you have a graphid where level == 8 and tileid == 24134109851 you should get:
  8/024/134/109/851.gph since the number of levels is likely to be very small this limits the total
  number of objects in any one directory to 1000 which is an empirically derived good choice for
  mechanical hard drives this should be fine for s3 as well (even though it breaks the rule of most
  unique part of filename first) because there will be just so few objects in general in practice
  */

  // figure the largest id for this level
  auto found = TileHierarchy::levels().find(graphid.level());
  if (found == TileHierarchy::levels().cend() &&
      graphid.level() != TileHierarchy::GetTransitLevel().level) {
    throw std::runtime_error("Could not compute FileSuffix for non-existent level: " +
                             std::to_string(graphid.level()));
  }

  // get the level info
  const auto& level = graphid.level() == TileHierarchy::GetTransitLevel().level
                          ? TileHierarchy::GetTransitLevel()
                          : found->second;

  // figure out how many digits
  auto max_id = level.tiles.ncolumns() * level.tiles.nrows() - 1;
  size_t max_length = static_cast<size_t>(std::log10(std::max(1, max_id))) + 1;
  const size_t remainder = max_length % 3;
  if (remainder) {
    max_length += 3 - remainder;
  }

  // make a locale to use as a formatter for numbers
  std::ostringstream stream;
  stream.imbue(dir_locale);

  // if it starts with a zero the pow trick doesn't work
  if (graphid.level() == 0) {
    stream << static_cast<uint32_t>(std::pow(10, max_length)) + graphid.tileid() << ".gph"
           << (gzipped ? ".gz" : "");
    std::string suffix = stream.str();
    suffix[0] = '0';
    return suffix;
  }
  // it was something else
  stream << graphid.level() * static_cast<uint32_t>(std::pow(10, max_length)) + graphid.tileid()
         << ".gph" << (gzipped ? ".gz" : "");
  return stream.str();
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
  std::vector<int> digits;
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
  auto found = TileHierarchy::levels().find(digits.back());
  if (found == TileHierarchy::levels().cend() &&
      digits.back() != TileHierarchy::GetTransitLevel().level) {
    throw std::runtime_error("Invalid tile path: " + fname);
  }

  // get the level info
  uint32_t level = digits.back();
  digits.pop_back();
  const auto& tile_level = level == TileHierarchy::GetTransitLevel().level
                               ? TileHierarchy::GetTransitLevel()
                               : found->second;

  // get the number of sub directories that we should have
  auto max_id = tile_level.tiles.ncolumns() * tile_level.tiles.nrows() - 1;
  size_t parts = static_cast<size_t>(std::log10(std::max(1, max_id))) + 1;
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

  // figure the largest id for this level
  auto level = TileHierarchy::levels().find(header_->graphid().level());
  if (level == TileHierarchy::levels().end() &&
      header_->graphid().level() == ((TileHierarchy::levels().rbegin())->second.level + 1)) {
    level = TileHierarchy::levels().begin();
  }

  auto tiles = level->second.tiles;
  return tiles.TileBounds(header_->graphid().tileid());
}

iterable_t<const DirectedEdge> GraphTile::GetDirectedEdges(const NodeInfo* node) const {
  const auto* edge = directededges_ + node->edge_index();
  return iterable_t<const DirectedEdge>{edge, node->edge_count()};
}

iterable_t<const DirectedEdge> GraphTile::GetDirectedEdges(const GraphId& node) const {
  if (node.id() < header_->nodecount()) {
    const auto* nodeinfo = nodes_ + node.id();
    return GetDirectedEdges(nodeinfo);
  }
  throw std::runtime_error(
      std::string(__FILE__) + ":" + std::to_string(__LINE__) +
      " GraphTile NodeInfo index out of bounds: " + std::to_string(node.tileid()) + "," +
      std::to_string(node.level()) + "," + std::to_string(node.id()) +
      " nodecount= " + std::to_string(header_->nodecount()));
}

iterable_t<const DirectedEdge> GraphTile::GetDirectedEdges(const size_t idx) const {
  if (idx < header_->nodecount()) {
    const auto& nodeinfo = nodes_[idx];
    const auto* edge = directededge(nodeinfo.edge_index());
    return iterable_t<const DirectedEdge>{edge, nodeinfo.edge_count()};
  }
  throw std::runtime_error(
      std::string(__FILE__) + ":" + std::to_string(__LINE__) +
      " GraphTile NodeInfo index out of bounds 5: " + std::to_string(header_->graphid().tileid()) +
      "," + std::to_string(header_->graphid().level()) + "," + std::to_string(idx) +
      " nodecount= " + std::to_string(header_->nodecount()));
}

// Get a pointer to edge info.
EdgeInfo GraphTile::edgeinfo(const size_t offset) const {
  return EdgeInfo(edgeinfo_ + offset, textlist_, textlist_size_);
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

// Convenience method to get the names for an edge given the offset to the
// edge info
std::vector<std::string> GraphTile::GetNames(const uint32_t edgeinfo_offset) const {
  return edgeinfo(edgeinfo_offset).GetNames();
}

// Convenience method to get the types for the names given the offset to the
// edge info
uint16_t GraphTile::GetTypes(const uint32_t edgeinfo_offset) const {
  return edgeinfo(edgeinfo_offset).GetTypes();
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

// Convenience method to get the signs for an edge given the
// directed edge index.
std::vector<SignInfo> GraphTile::GetSigns(const uint32_t idx) const {
  uint32_t count = header_->signcount();
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
    if (idx == sign.edgeindex()) {
      found = mid;
      high = mid - 1;
    } // need a smaller index
    else if (idx < sign.edgeindex()) {
      high = mid - 1;
    } // need a bigger index
    else {
      low = mid + 1;
    }
  }

  // Add signs
  for (; found < count && signs_[found].edgeindex() == idx; ++found) {
    if (signs_[found].text_offset() < textlist_size_) {
      // Skip tagged text strings (Future code is needed to handle tagged strings)
      if (signs_[found].tagged()) {
        continue;
      }
      signs.emplace_back(signs_[found].type(), signs_[found].is_route_num(),
                         (textlist_ + signs_[found].text_offset()));
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
  int32_t found = count;
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
  int32_t found = count;
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
    // Make sure valid departure time
    if (departures_[found].type() == kFixedSchedule) {
      if (departures_[found].departure_time() >= current_time &&
          GetTransitSchedule(departures_[found].schedule_index())
              ->IsValid(day, dow, date_before_tile) &&
          (!wheelchair || departures_[found].wheelchair_accessible()) &&
          (!bicycle || departures_[found].bicycle_accessible())) {
        return &departures_[found];
      }
    } else {
      uint32_t departure_time = departures_[found].departure_time();
      uint32_t end_time = departures_[found].end_time();
      uint32_t frequency = departures_[found].frequency();
      while (departure_time < current_time && departure_time < end_time) {
        departure_time += frequency;
      }

      if (departure_time >= current_time && departure_time < end_time &&
          GetTransitSchedule(departures_[found].schedule_index())
              ->IsValid(day, dow, date_before_tile) &&
          (!wheelchair || departures_[found].wheelchair_accessible()) &&
          (!bicycle || departures_[found].bicycle_accessible())) {

        const auto& d = departures_[found];
        const TransitDeparture* dep =
            new TransitDeparture(d.lineid(), d.tripid(), d.routeid(), d.blockid(),
                                 d.headsign_offset(), departure_time, d.end_time(), d.frequency(),
                                 d.elapsed_time(), d.schedule_index(), d.wheelchair_accessible(),
                                 d.bicycle_accessible());
        return dep;
      }
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
  int32_t found = count;
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
            new TransitDeparture(d.lineid(), d.tripid(), d.routeid(), d.blockid(),
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
  // Binary search to find a access restriction with matching edge Id.
  int32_t low = 0;
  int32_t high = count - 1;
  int32_t mid;
  int32_t found = count;
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
