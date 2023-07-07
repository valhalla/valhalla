#include <valhalla/incidents/utils.h>
#include <valhalla/incidents/worker.h>

namespace vb = valhalla::baldr;
namespace vi = valhalla::incidents;

namespace valhalla {
namespace incidents {

// handles the multithreaded traffic writing
void incident_worker_t::write_traffic(std::vector<OpenLrEdge>&& openlrs_edges) {

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
  for (const auto& openlr_edge : openlrs_edges) {
    baldr::TrafficSpeed traffic_speed{};
    if (openlr_edge.first_node_offset && openlr_edge.last_node_offset != 255) {
      // must be a single-edge openlr segment not even covering a full edge
      traffic_speed.breakpoint1 = openlr_edge.first_node_offset;
      traffic_speed.encoded_speed1 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
      traffic_speed.encoded_speed2 = 0;
      traffic_speed.breakpoint2 = openlr_edge.last_node_offset;
      traffic_speed.encoded_speed3 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
    } else if (openlr_edge.first_node_offset) {
      // for the first edge of an openlr segment
      traffic_speed.breakpoint1 = openlr_edge.first_node_offset;
      traffic_speed.encoded_speed1 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
      traffic_speed.encoded_speed2 = 0;
      traffic_speed.breakpoint2 = 255;
      traffic_speed.encoded_speed3 = 0;
    } else if (openlr_edge.last_node_offset != 255) {
      // for the last edge of an openlr segment
      traffic_speed.breakpoint1 = openlr_edge.last_node_offset;
      traffic_speed.encoded_speed1 = 0;
      traffic_speed.encoded_speed2 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
      traffic_speed.breakpoint2 = 255;
      traffic_speed.encoded_speed3 = vb::UNKNOWN_TRAFFIC_SPEED_RAW;
    } else {
      traffic_speed.breakpoint1 = 255;
      traffic_speed.encoded_speed1 = 0;
    }

    count++;
  }
}
} // namespace incidents
} // namespace valhalla
