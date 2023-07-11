#include <valhalla/incidents/utils.h>
#include <valhalla/incidents/worker.h>

namespace vb = valhalla::baldr;
namespace vi = valhalla::incidents;

constexpr auto HEADER_SIZE = sizeof(valhalla::baldr::TrafficTileHeader);
constexpr auto TRAFFIC_SIZE = sizeof(valhalla::baldr::TrafficSpeed);

namespace valhalla {
namespace incidents {

// handles the traffic writing
void incident_worker_t::write_traffic(std::vector<OpenLrEdge>&& openlrs_edges,
                                      const IncidentsAction action) {

  if (action == IncidentsAction::RESET) {
    for (const auto& tile : reader->tile_extract_->traffic_tiles) {
      auto edge_count = (tile.second.second - HEADER_SIZE) / TRAFFIC_SIZE;
      auto base_pos = tile.second.first + HEADER_SIZE;
      for (int i = 0; i < edge_count; i++) {
        *reinterpret_cast<baldr::TrafficSpeed*>(base_pos + (i * TRAFFIC_SIZE)) =
            baldr::TrafficSpeed();
      }
    }
  }

  if (!openlrs_edges.size()) {
    return;
  }

  std::sort(openlrs_edges.begin(), openlrs_edges.end(), [](const OpenLrEdge& a, const OpenLrEdge& b) {
    // This ordering means when we iterate over this list, it'll be
    // cache friendly, in-memory-order
    if (a.edge_id.level() == b.edge_id.level())
      return a.edge_id.tileid() < b.edge_id.tileid();
    if (a.edge_id.tileid() == b.edge_id.tileid())
      return a.edge_id.id() < b.edge_id.id();
    return a.edge_id.level() < b.edge_id.level();
  });

  uint32_t count = 0;
  auto openlr_begin = openlrs_edges.begin();
  auto tile_id = baldr::GraphId{};
  char* tile_data = nullptr;

  for (; openlr_begin != openlrs_edges.end(); openlr_begin++) {
    uint8_t b1 = 0, b2 = 0;
    uint8_t s1 = 0, s2 = 0, s3 = 0;

    // only record actual data for /update & /reset
    if (action != IncidentsAction::DELETE) {
      auto poff = openlr_begin->poff_start_offset;
      auto noff = openlr_begin->noff_start_offset;
      auto length = openlr_begin->length;
      if (poff && noff) {
        // must be a single-edge openlr segment not even covering a full edge
        // x----b1=====b2----x
        //   s1     s2    s3
        s1 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
        b1 = static_cast<uint8_t>((poff / length) * 255.f);
        s2 = 0;
        b2 = static_cast<uint8_t>(((length - noff) / length) * 255.f);
        s3 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
      } else if (poff) {
        // either the starting edge or poff of a trivial openlr
        // x----b1===========x(---)
        //   s1       s2/s3
        s1 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
        b1 = static_cast<uint8_t>((poff / length) * 255.f);
        s2 = 0;
        b2 = 255;
        s3 = 0;
      } else if (noff) {
        // either the last edge or noff of a trivial openlr
        // x====b1-----------x(---)
        //   s1       s2/s3
        s1 = 0;
        b1 = static_cast<uint8_t>(((length - noff) / length) * 255.f);
        s2 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
        b2 = 255;
        s3 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
      } else {
        s1 = 0;
        b1 = 255;
        s2 = 0;
        b2 = 255;
        s3 = 0;
      }
    }

    auto current_tile_id = openlr_begin->edge_id.Tile_Base();
    if (current_tile_id != tile_id) {
      auto tile_offset =
          reader->tile_extract_->traffic_tiles.find(openlr_begin->edge_id.Tile_Base())->second;

      tile_data = tile_offset.first + HEADER_SIZE;
      tile_id = current_tile_id;
    }

    auto edge_position = tile_data + (openlr_begin->edge_id.id() * TRAFFIC_SIZE);
    *reinterpret_cast<baldr::TrafficSpeed*>(edge_position) =
        baldr::TrafficSpeed{0, s1, s2, s3, b1, b2, 0, 0, 0, false};
  }
}
} // namespace incidents
} // namespace valhalla
