#include <valhalla/incidents/utils.h>
#include <valhalla/incidents/worker.h>

namespace vb = valhalla::baldr;
namespace vi = valhalla::incidents;

namespace valhalla {
namespace incidents {

// handles the multithreaded traffic writing
void incident_worker_t::write_traffic(std::vector<OpenLrEdge>&& openlrs_edges,
                                      const IncidentsAction action) {
  const auto header_size = sizeof(baldr::TrafficTileHeader);

  std::sort(openlrs_edges.begin(), openlrs_edges.end(), [](const OpenLrEdge& a, const OpenLrEdge& b) {
    // This ordering means when we iterate over this list, it'll be
    // cache friendly, in-memory-order
    if (a.edge_id.level() == b.edge_id.level())
      return a.edge_id.tileid() < b.edge_id.tileid();
    if (a.edge_id.tileid() == b.edge_id.tileid())
      return a.edge_id.id() < b.edge_id.id();
    return a.edge_id.level() < b.edge_id.level();
  });

  // TODO: reset the entire tar file if action == RESET before updating the matched edges
  if (action == IncidentsAction::RESET) {
    for (const auto& tile : reader->tile_extract_->traffic_tiles) {
      *(tile.second.first + header_size) = (tile.second.second - header_size) * '\0';
    }
  }

  uint32_t count = 0;
  auto openlr_begin = openlrs_edges.begin();
  auto tile_id = baldr::GraphId{};
  char* tile_data = nullptr;

  for (; openlr_begin != openlrs_edges.end(); openlr_begin++) {
    baldr::TrafficSpeed traffic_speed{};
    if (action == IncidentsAction::DELETE) {
      traffic_speed = baldr::TrafficSpeed{baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                          baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                          baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                          baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                          255U,
                                          255U,
                                          0,
                                          0,
                                          0,
                                          false};
    } else if (action == IncidentsAction::UPDATE) {
      if (openlr_begin->breakpoint1 != 255 && openlr_begin->breakpoint2 != 255) {
        // must be a single-edge openlr segment not even covering a full edge
        traffic_speed.breakpoint1 = openlr_begin->breakpoint1;
        traffic_speed.encoded_speed1 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
        traffic_speed.encoded_speed2 = 0;
        traffic_speed.breakpoint2 = openlr_begin->breakpoint2;
        traffic_speed.encoded_speed3 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
      } else if (openlr_begin->breakpoint1 != 255) {
        // for the first edge of an openlr segment
        traffic_speed.breakpoint1 = openlr_begin->breakpoint1;
        traffic_speed.encoded_speed1 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
        traffic_speed.encoded_speed2 = 0;
        traffic_speed.breakpoint2 = 255;
        traffic_speed.encoded_speed3 = 0;
      } else if (openlr_begin->breakpoint2 != 255) {
        // for the last edge of an openlr segment
        traffic_speed.breakpoint1 = openlr_begin->breakpoint2;
        traffic_speed.encoded_speed1 = 0;
        traffic_speed.encoded_speed2 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
        traffic_speed.breakpoint2 = 255;
        traffic_speed.encoded_speed3 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
      } else {
        traffic_speed.breakpoint1 = 255;
        traffic_speed.encoded_speed1 = 0;
      }
    }

    auto current_tile_id = openlr_begin->edge_id.Tile_Base();
    if (current_tile_id != tile_id) {
      auto tile_offset =
          reader->tile_extract_->traffic_tiles.find(openlr_begin->edge_id.Tile_Base())->second;

      tile_data = tile_offset.first + sizeof(baldr::TrafficTileHeader);
      tile_id = current_tile_id;
    }

    auto edge_position = tile_data + (openlr_begin->edge_id.id() * sizeof(baldr::TrafficSpeed));
    *reinterpret_cast<baldr::TrafficSpeed*>(edge_position) = traffic_speed;
  }
}
} // namespace incidents
} // namespace valhalla
