#pragma once

#include <valhalla/baldr/graphtile.h>
#include <boost/property_tree/ptree_fwd.hpp>

#include "rust/cxx.h"

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

/// Exposed internal [`valhalla::baldr::GraphReader::tile_extract_t`], used to
/// access exact graph and traffic tiles. Create it using [`new_tileset()`].
struct TileSet {
  /// Explicitly define destructor as otherwise compiler will fail with
  /// std::unique_ptr due to forward declarations for `midgard::tar`
  ~TileSet();

  std::unordered_map<uint64_t, std::pair<char*, size_t>> tiles_;
  std::unordered_map<uint64_t, std::pair<char*, size_t>> traffic_tiles_;
  std::shared_ptr<valhalla::midgard::tar> tar_;
  std::shared_ptr<valhalla::midgard::tar> traffic_tar_;

  rust::Vec<TileId> tiles() const;
  rust::Vec<TileId> tiles_in_bbox(float min_lat, float min_lon, float max_lat, float max_lon, uint8_t level) const;
  // It's Rust's side responsibility to manage GraphTile lifetime by calling
  // `add_ref()` each time the pointer is cloned and `release()` each time it is dropped.
  const valhalla::baldr::GraphTile* get_graph_tile(TileId id) const;
  TrafficTile get_traffic_tile(TileId id) const;
  uint64_t dataset_id() const;
};

/// Creates a new [`TileSet`] instance based on a Valhalla's config.
std::shared_ptr<TileSet> new_tileset(const boost::property_tree::ptree& config);

inline void add_ref(const valhalla::baldr::GraphTile* tile) {
  boost::sp_adl_block::intrusive_ptr_add_ref(tile);
}

inline void release(const valhalla::baldr::GraphTile* tile) {
  boost::sp_adl_block::intrusive_ptr_release(tile);
}

/// Helper function as cxx unable to call constructors with arguments.
inline valhalla::baldr::GraphId from_parts(uint8_t level, uint32_t tile_index, uint32_t id) {
  return valhalla::baldr::GraphId(tile_index, level, id);
}

/// Helper function that allows to iterate over a slice of directed edges of that tile in Rust
inline rust::Slice<const valhalla::baldr::DirectedEdge> directededges(const valhalla::baldr::GraphTile& tile) {
  auto slice = tile.GetDirectedEdges();
  return rust::Slice(slice.data(), slice.size());
}

/// Helper function that allows to iterate over a slice of nodes of that tile in Rust
inline rust::Slice<const valhalla::baldr::NodeInfo> nodes(const valhalla::baldr::GraphTile& tile) {
  auto slice = tile.GetNodes();
  return rust::Slice(slice.data(), slice.size());
}

/// Helper function that allows to iterate over a slice of node edges of that tile in Rust
inline rust::Slice<const valhalla::baldr::DirectedEdge> node_edges(const valhalla::baldr::GraphTile& tile,
                                                                   const valhalla::baldr::NodeInfo& node) {
  auto edges = tile.GetDirectedEdges();
  // Safety: Rust side of bindings has an assert that this node belongs to the given tile.
  return rust::Slice(edges.data() + node.edge_index(), node.edge_count());
}

/// Helper function that allows to iterate over a slice of node transitions of that tile in Rust
inline rust::Slice<const valhalla::baldr::NodeTransition> node_transitions(const valhalla::baldr::GraphTile& tile,
                                                                           const valhalla::baldr::NodeInfo& node) {
  auto slice = tile.GetNodeTransitions(&node);
  return rust::Slice(slice.data(), slice.size());
}

/// Helper function to get lat,lng for the given node
LatLon node_latlon(const valhalla::baldr::GraphTile& tile, const valhalla::baldr::NodeInfo& node);

/// Borrowed view of the tile's edge info: scalars, plus the shape as a slice for Rust to decode.
EdgeInfo edgeinfo(const valhalla::baldr::GraphTile& tile, const valhalla::baldr::DirectedEdge& de);

/// Helper method that returns the raw `TrafficSpeed` bits for the edge's live traffic record
uint64_t live_traffic(const valhalla::baldr::GraphTile& tile, const valhalla::baldr::DirectedEdge& de);

/// Helper function to get admin info for a given index
AdminInfo admininfo(const valhalla::baldr::GraphTile& tile, uint32_t index);

/// Helper function to resolve tz name and offset from a given id and unix timestamp.
TimeZoneInfo from_id(uint32_t id, uint64_t unix_timestamp);

// Traffic tile helpers that read/write fields from/to the `TrafficTileHeader`
valhalla::baldr::GraphId id(const TrafficTile& tile);
uint64_t last_update(const TrafficTile& tile);
void write_last_update(const TrafficTile& tile, uint64_t t);
uint64_t spare(const TrafficTile& tile);
void write_spare(const TrafficTile& tile, uint64_t s);

/// Helper function that encodes predicted speeds from a slice of floats and returns a base64 string
inline rust::String encode_weekly_speeds(rust::Slice<const float> speeds) {
  if (speeds.size() != valhalla::baldr::kBucketsPerWeek) {
    throw std::runtime_error("Weekly speeds slice size must be equal to " +
                             std::to_string(valhalla::baldr::kBucketsPerWeek));
  }

  auto compressed = valhalla::baldr::compress_speed_buckets(speeds.data());
  return valhalla::baldr::encode_compressed_speeds(compressed.data());
}

/// Helper function that decodes predicted speeds from a base64 string into an array or floats
inline rust::Vec<float> decode_weekly_speeds(rust::Str encoded) {
  // todo: replace by std::string_view once Valhalla supports it
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
