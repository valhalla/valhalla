#ifndef VALHALLA_MJOLNIR_TRANSITPBF_H
#define VALHALLA_MJOLNIR_TRANSITPBF_H

#include <mutex>
#include <string>
#include <vector>

#include <boost/filesystem/operations.hpp>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/filesystem.h>
#include <valhalla/midgard/encoded.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/sequence.h>
#include <valhalla/proto/transit.pb.h>

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace mjolnir {

// Shape
struct Shape {
  uint32_t begins;
  uint32_t ends;
  std::vector<PointLL> shape;
};

struct Departure {
  GraphId orig_pbf_graphid; // GraphId in pbf tiles
  GraphId dest_pbf_graphid; // GraphId in pbf tiles
  uint32_t trip;
  uint32_t route;
  uint32_t blockid;
  uint32_t shapeid;
  uint32_t headsign_offset;
  uint32_t dep_time;
  uint32_t schedule_index;
  uint32_t frequency_end_time;
  uint16_t elapsed_time;
  uint16_t frequency;
  float orig_dist_traveled;
  float dest_dist_traveled;
  bool wheelchair_accessible;
  bool bicycle_accessible;
};

// Unique route and stop
struct TransitLine {
  uint32_t lineid;
  uint32_t routeid;
  GraphId dest_pbf_graphid; // GraphId (from pbf) of the destination stop
  uint32_t shapeid;
  float orig_dist_traveled;
  float dest_dist_traveled;
};

struct StopEdges {
  GraphId origin_pbf_graphid;        // GraphId (from pbf) of the origin stop
  std::vector<GraphId> intrastation; // List of intra-station connections
  std::vector<TransitLine> lines;    // Set of unique route/stop pairs
};

Transit read_pbf(const std::string& file_name, std::mutex& lock) {
  lock.lock();
  std::fstream file(file_name, std::ios::in | std::ios::binary);
  if (!file) {
    throw std::runtime_error("Couldn't load " + file_name);
    lock.unlock();
  }
  std::string buffer((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  lock.unlock();
  google::protobuf::io::ArrayInputStream as(static_cast<const void*>(buffer.c_str()), buffer.size());
  google::protobuf::io::CodedInputStream cs(
      static_cast<google::protobuf::io::ZeroCopyInputStream*>(&as));
  auto limit = std::max(static_cast<size_t>(1), buffer.size() * 2);
  cs.SetTotalBytesLimit(limit, limit);
  Transit transit;
  if (!transit.ParseFromCodedStream(&cs)) {
    throw std::runtime_error("Couldn't load " + file_name);
  }
  return transit;
}

Transit read_pbf(const std::string& file_name) {
  std::fstream file(file_name, std::ios::in | std::ios::binary);
  if (!file) {
    throw std::runtime_error("Couldn't load " + file_name);
  }
  std::string buffer((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  google::protobuf::io::ArrayInputStream as(static_cast<const void*>(buffer.c_str()), buffer.size());
  google::protobuf::io::CodedInputStream cs(
      static_cast<google::protobuf::io::ZeroCopyInputStream*>(&as));
  auto limit = std::max(static_cast<size_t>(1), buffer.size() * 2);
  cs.SetTotalBytesLimit(limit, limit);
  Transit transit;
  if (!transit.ParseFromCodedStream(&cs)) {
    throw std::runtime_error("Couldn't load " + file_name);
  }
  return transit;
}

// Get PBF transit data given a GraphId / tile
Transit read_pbf(const GraphId& id, const std::string& transit_dir, std::string& file_name) {
  std::string fname = GraphTile::FileSuffix(id);
  fname = fname.substr(0, fname.size() - 3) + "pbf";
  file_name = transit_dir + '/' + fname;
  Transit transit;
  transit = read_pbf(file_name);
  return transit;
}

void write_pbf(const Transit& tile, const boost::filesystem::path& transit_tile) {
  // check for empty stop pairs and routes.
  if (tile.stop_pairs_size() == 0 && tile.routes_size() == 0 && tile.shapes_size() == 0) {
    LOG_WARN(transit_tile.string() + " had no data and will not be stored");
    return;
  }

  // write pbf to file
  if (!boost::filesystem::exists(transit_tile.parent_path())) {
    boost::filesystem::create_directories(transit_tile.parent_path());
  }
  auto size = tile.ByteSize();
  valhalla::midgard::mem_map<char> buffer;
  buffer.create(transit_tile.string(), size);
  if (!tile.SerializeToArray(buffer.get(), size)) {
    LOG_ERROR("Couldn't write: " + transit_tile.string() + " it would have been " +
              std::to_string(size));
  }

  if (tile.routes_size() && tile.nodes_size() && tile.stop_pairs_size() && tile.shapes_size()) {
    LOG_INFO(transit_tile.string() + " had " + std::to_string(tile.nodes_size()) + " nodes " +
             std::to_string(tile.routes_size()) + " routes " + std::to_string(tile.shapes_size()) +
             " shapes " + std::to_string(tile.stop_pairs_size()) + " stop pairs");
  } else {
    LOG_INFO(transit_tile.string() + " had " + std::to_string(tile.stop_pairs_size()) +
             " stop pairs");
  }
}

// Converts a stop's pbf graph Id to a Valhalla graph Id by adding the
// tile's node count. Returns an Invalid GraphId if the tile is not found
// in the list of Valhalla tiles
GraphId GetGraphId(const GraphId& nodeid, const std::unordered_set<GraphId>& all_tiles) {
  auto t = all_tiles.find(nodeid.Tile_Base());
  if (t == all_tiles.end()) {
    return GraphId(); // Invalid graph Id
  } else {
    return {nodeid.tileid(), nodeid.level() + 1, nodeid.id()};
  }
}
} // namespace mjolnir
} // namespace valhalla
#endif // VALHALLA_MJOLNIR_TRANSITPBF_H
