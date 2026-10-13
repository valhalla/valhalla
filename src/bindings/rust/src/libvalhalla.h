#pragma once

#include <valhalla/baldr/graphtile.h>

#include <boost/property_tree/ptree_fwd.hpp>
#include <rust/cxx.h>

namespace valhalla::midgard {
struct tar;
}

// Forward Declarations for shared types, defined in lib.rs
struct AdminInfo;
struct EdgeInfo;
struct TimeZoneInfo;
struct TrafficTile;
struct LatLon;
struct TileId;

// Protected `valhalla::baldr::GraphReader::tile_extract_t`, to access graph and traffic tiles
// directly.
struct TileSet {
  std::unordered_map<uint64_t, std::pair<char*, size_t>> tiles_;
  std::unordered_map<uint64_t, std::pair<char*, size_t>> traffic_tiles_;
  std::shared_ptr<valhalla::midgard::tar> tar_;
  std::shared_ptr<valhalla::midgard::tar> traffic_tar_;

  rust::Vec<TileId> tiles() const;
  rust::Vec<TileId>
  tiles_in_bbox(float min_lat, float min_lon, float max_lat, float max_lon, uint8_t level) const;
  // The caller owns a reference to the returned tile, see `add_ref()` and `release()`.
  const valhalla::baldr::GraphTile* get_graph_tile(TileId id) const;
  TrafficTile get_traffic_tile(TileId id) const;
  uint64_t dataset_id() const;
};

std::shared_ptr<TileSet> new_tileset(const boost::property_tree::ptree& config);

inline void add_ref(const valhalla::baldr::GraphTile* tile) {
  boost::sp_adl_block::intrusive_ptr_add_ref(tile);
}

inline void release(const valhalla::baldr::GraphTile* tile) {
  boost::sp_adl_block::intrusive_ptr_release(tile);
}

// cxx can't call constructors with arguments.
inline valhalla::baldr::GraphId from_parts(uint8_t level, uint32_t tile_index, uint32_t id) {
  return valhalla::baldr::GraphId(tile_index, level, id);
}

inline rust::Slice<const valhalla::baldr::DirectedEdge>
directededges(const valhalla::baldr::GraphTile& tile) {
  auto slice = tile.GetDirectedEdges();
  return rust::Slice(slice.data(), slice.size());
}

inline rust::Slice<const valhalla::baldr::NodeInfo> nodes(const valhalla::baldr::GraphTile& tile) {
  auto slice = tile.GetNodes();
  return rust::Slice(slice.data(), slice.size());
}

inline rust::Slice<const valhalla::baldr::DirectedEdge>
node_edges(const valhalla::baldr::GraphTile& tile, const valhalla::baldr::NodeInfo& node) {
  auto slice = tile.GetDirectedEdges(&node);
  return rust::Slice(slice.data(), slice.size());
}

inline rust::Slice<const valhalla::baldr::NodeTransition>
node_transitions(const valhalla::baldr::GraphTile& tile, const valhalla::baldr::NodeInfo& node) {
  auto slice = tile.GetNodeTransitions(&node);
  return rust::Slice(slice.data(), slice.size());
}

LatLon node_latlon(const valhalla::baldr::GraphTile& tile, const valhalla::baldr::NodeInfo& node);

// Borrowed view of the tile's edge info, Rust decodes the shape.
EdgeInfo edgeinfo(const valhalla::baldr::GraphTile& tile, const valhalla::baldr::DirectedEdge& de);

uint64_t live_traffic(const valhalla::baldr::GraphTile& tile,
                      const valhalla::baldr::DirectedEdge& de);

AdminInfo admininfo(const valhalla::baldr::GraphTile& tile, uint32_t index);

TimeZoneInfo from_id(uint32_t id, uint64_t unix_timestamp);

// Traffic tile helpers that read/write fields from/to the `TrafficTileHeader`
valhalla::baldr::GraphId id(const TrafficTile& tile);
uint64_t last_update(const TrafficTile& tile);
void write_last_update(const TrafficTile& tile, uint64_t t);
uint64_t spare(const TrafficTile& tile);
void write_spare(const TrafficTile& tile, uint64_t s);

inline rust::String encode_weekly_speeds(rust::Slice<const float> speeds) {
  if (speeds.size() != valhalla::baldr::kBucketsPerWeek) {
    throw std::runtime_error("Weekly speeds slice size must be equal to " +
                             std::to_string(valhalla::baldr::kBucketsPerWeek));
  }

  auto compressed = valhalla::baldr::compress_speed_buckets(speeds.data());
  return valhalla::baldr::encode_compressed_speeds(compressed.data());
}

inline rust::Vec<float> decode_weekly_speeds(rust::Str encoded) {
  std::string encoded_str(encoded.data(), encoded.size());
  const auto coefficients = valhalla::baldr::decode_compressed_speeds(encoded_str);

  rust::Vec<float> speeds;
  speeds.reserve(valhalla::baldr::kBucketsPerWeek);
  for (uint32_t i = 0; i < valhalla::baldr::kBucketsPerWeek; ++i) {
    float speed = valhalla::baldr::decompress_speed_bucket(coefficients.data(), i);
    speeds.push_back(speed);
  }

  return speeds;
}
