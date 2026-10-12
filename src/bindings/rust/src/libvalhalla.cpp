#include "libvalhalla.hpp"
#include "valhalla/src/lib.rs.h"

#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/midgard/encoded.h>

#include <boost/property_tree/ptree.hpp>

namespace baldr = valhalla::baldr;
namespace midgard = valhalla::midgard;

namespace {

struct GraphMemory : public baldr::GraphMemory {
  const std::shared_ptr<midgard::tar> tar_;

  GraphMemory(std::shared_ptr<midgard::tar> tar, std::pair<char*, size_t> position) : tar_(std::move(tar)) {
    data = position.first;
    size = position.second;
  }
};

}  // namespace

TileSet::~TileSet() {}

std::shared_ptr<TileSet> new_tileset(const boost::property_tree::ptree& pt) {
  // Hack to expose protected `baldr::GraphReader::tile_extract_t`
  struct TileSetReader : public baldr::GraphReader {
    static TileSet create(const boost::property_tree::ptree& pt) {
      auto extract = baldr::GraphReader::tile_extract_t(pt, false);
      return TileSet{
        .tiles_ = std::move(extract.tiles),
        .traffic_tiles_ = std::move(extract.traffic_tiles),
        .tar_ = std::move(extract.archive),
        .traffic_tar_ = std::move(extract.traffic_archive),
      };
    }
  };

  auto tile_set = TileSetReader::create(pt.get_child("mjolnir"));
  if (!tile_set.tar_) {
    throw std::runtime_error("Failed to load tile extract");
  }
  return std::make_shared<TileSet>(std::move(tile_set));
}

rust::Vec<TileId> TileSet::tiles() const {
  rust::Vec<TileId> result;
  result.reserve(tiles_.size());
  for (const auto& tile : tiles_) {
    result.push_back(TileId{.value = baldr::GraphId(tile.first).tile_value()});
  }
  return result;
}

rust::Vec<TileId> TileSet::tiles_in_bbox(float min_lat, float min_lon, float max_lat, float max_lon,
                                         uint8_t level) const {
  rust::Vec<TileId> result;
  if (level >= baldr::TileHierarchy::levels().size()) {
    return result;
  }

  const midgard::AABB2<midgard::PointLL> bbox(min_lon, min_lat, max_lon, max_lat);
  const auto tile_ids = baldr::TileHierarchy::levels()[static_cast<size_t>(level)].tiles.TileList(bbox);

  result.reserve(tile_ids.size());
  for (auto tile_id : tile_ids) {
    const baldr::GraphId graph_id(tile_id, level, 0);
    // List only tiles that we have
    if (tiles_.find(graph_id.tile_base()) != tiles_.end()) {
      result.push_back(TileId{.value = graph_id.tile_value()});
    }
  }
  return result;
}

/// Part of the [`baldr::GraphReader::GetGraphTile()`] that gets tile from mmap file
const baldr::GraphTile* TileSet::get_graph_tile(TileId id) const {
  const baldr::GraphId base(id.value);

  auto tile_it = tiles_.find(base);
  if (tile_it == tiles_.end()) {
    return nullptr;
  }

  // Optionally get the traffic tile if it exists
  auto traffic_it = traffic_tiles_.find(base);
  auto traffic =
      traffic_it != traffic_tiles_.end() ? std::make_unique<GraphMemory>(traffic_tar_, traffic_it->second) : nullptr;

  // cxx doesn't support `boost::intrusive_ptr<T>`, so instead all refcounting should be done manually
  auto ptr = baldr::GraphTile::Create(base, std::make_unique<GraphMemory>(tar_, tile_it->second), std::move(traffic));
  return ptr.detach();
}

TrafficTile TileSet::get_traffic_tile(TileId id) const {
  const baldr::GraphId base(id.value);
  auto traffic_it = traffic_tiles_.find(base);
  if (traffic_it == traffic_tiles_.end()) {
    throw std::runtime_error("No traffic tile for the given id");
  }

  auto header = reinterpret_cast<volatile baldr::TrafficTileHeader*>(traffic_it->second.first);
  if (header->traffic_tile_version != baldr::TRAFFIC_TILE_VERSION) {
    throw std::runtime_error("Unsupported TrafficTile version");
  }
  if (sizeof(baldr::TrafficTileHeader) + header->directed_edge_count * sizeof(baldr::TrafficSpeed) !=
      traffic_it->second.second) {
    throw std::runtime_error("TrafficTile data size does not match header count");
  }

  return TrafficTile{
    .header = reinterpret_cast<uint64_t*>(traffic_it->second.first),
    .speeds = reinterpret_cast<uint64_t*>(traffic_it->second.first + sizeof(baldr::TrafficTileHeader)),
    .edge_count = header->directed_edge_count,
    .traffic_tar = traffic_tar_,
  };
}

uint64_t TileSet::dataset_id() const {
  // Every tile starts with its header, so read it in place rather than building a tile around it.
  if (auto it = tiles_.begin(); it != tiles_.end() && it->second.second >= sizeof(baldr::GraphTileHeader)) {
    return reinterpret_cast<const baldr::GraphTileHeader*>(it->second.first)->dataset_id();
  }
  return 0;
}

LatLon node_latlon(const baldr::GraphTile& tile, const baldr::NodeInfo& node) {
  const auto base_ll = tile.header()->base_ll();
  const auto ll = node.latlng(base_ll);
  return LatLon{ .lat = ll.lat(), .lon = ll.lng() };
}

EdgeInfo edgeinfo(const baldr::GraphTile& tile, const baldr::DirectedEdge& de) {
  // `baldr::EdgeInfo` keeps these pointers protected, and a local class cannot hold static data
  // members - hence the `using`s and the pointers-to-member as locals.
  struct EdgeInfoPeek : baldr::EdgeInfo {
    using baldr::EdgeInfo::encoded_shape_;
  };
  constexpr auto shape_ptr = &EdgeInfoPeek::encoded_shape_;

  const auto edge_info = tile.edgeinfo(&de);

  const auto* shape = reinterpret_cast<const uint8_t*>(edge_info.*shape_ptr);
  const uint32_t shape_size = edge_info.encoded_shape_size();

  return EdgeInfo{
    .way_id = edge_info.wayid(),
    // todo: properly handle `0` and `baldr::kUnlimitedSpeedLimit`
    .speed_limit = static_cast<uint8_t>(edge_info.speed_limit()),
    .encoded_shape = rust::Slice<const uint8_t>(shape, shape_size),
  };
}

uint64_t live_traffic(const baldr::GraphTile& tile, const baldr::DirectedEdge& de) {
  static_assert(sizeof(baldr::TrafficSpeed) == sizeof(uint64_t), "TrafficSpeed must be a single u64");
  // trafficspeed() throws if `de` is out of tile bounds - guarded by the Rust-side `debug_assert!`
  // ("Wrong tile") and load-time tile validation.
  const volatile baldr::TrafficSpeed& ts = tile.trafficspeed(&de);
  return *reinterpret_cast<const volatile uint64_t*>(&ts);
}

AdminInfo admininfo(const baldr::GraphTile& tile, uint32_t index) {
  auto info = tile.admininfo(index);
  return AdminInfo{
    .country_text = info.country_text(),
    .state_text = info.state_text(),
    .country_iso = info.country_iso(),
    .state_iso = info.state_iso(),
  };
}

TimeZoneInfo from_id(uint32_t id, uint64_t unix_timestamp) {
  const date::time_zone* tz = baldr::DateTime::get_tz_db().from_index(id);
  if (!tz) {
    throw std::runtime_error("Invalid time zone id: " + std::to_string(id));
  }

  // Because of DST, offset can change during some time of the year
  std::chrono::seconds dur(unix_timestamp);
  std::chrono::time_point<std::chrono::system_clock> tp(dur);
  const auto zoned_tp = date::make_zoned(tz, tp);
  const auto tz_info = zoned_tp.get_info();

  return TimeZoneInfo{
    .name = tz->name(),
    .offset_seconds = static_cast<int32_t>(tz_info.offset.count()),
  };
}

inline volatile baldr::TrafficTileHeader* tile_header(const TrafficTile& tile) {
  return reinterpret_cast<volatile baldr::TrafficTileHeader*>(const_cast<uint64_t*>(tile.header));
}

baldr::GraphId id(const TrafficTile& tile) {
  auto header = tile_header(tile);
  return baldr::GraphId(header->tile_id);
}

uint64_t last_update(const TrafficTile& tile) {
  auto header = tile_header(tile);
  return header->last_update;
}

void write_last_update(const TrafficTile& tile, uint64_t t) {
  auto header = tile_header(tile);
  header->last_update = t;
}

uint64_t spare(const TrafficTile& tile) {
  auto header = tile_header(tile);
  return (static_cast<uint64_t>(header->spare2) << 32) | header->spare3;
}

void write_spare(const TrafficTile& tile, uint64_t s) {
  auto header = tile_header(tile);
  header->spare2 = static_cast<uint32_t>(s >> 32);
  header->spare3 = static_cast<uint32_t>(s & 0xFFFFFFFF);
}
