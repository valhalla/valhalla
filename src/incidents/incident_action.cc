#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/incidents/edge_matcher.h>
#include <valhalla/incidents/worker.h>

namespace vb = valhalla::baldr;
namespace vi = valhalla::incidents;
namespace vm = valhalla::midgard;

namespace {
static const std::unordered_map<vi::IncidentsAction, std::string> endpoint_to_string = {
    {vi::IncidentsAction::UPDATE, "/update"},
    {vi::IncidentsAction::DELETE, "/delete"},
    {vi::IncidentsAction::RESET, "/reset"},
    {vi::IncidentsAction::GEOJSON, "/geojson"},
};

std::vector<vm::PointLL> get_lng_lat(const vi::OpenLrEdges& openlr_edges, vb::GraphReader& reader) {
  std::vector<vm::PointLL> coords(openlr_edges.edge_ids.size() + 1ULL);
  vb::graph_tile_ptr tile;
  const vb::DirectedEdge* current_de = nullptr;
  for (const auto edge_id : openlr_edges.edge_ids) {
    tile = reader.GetGraphTile(edge_id);
    current_de = reader.directededge(edge_id, tile);
    coords.emplace_back(tile->get_node_ll(reader.GetBeginNodeId(current_de, tile)));
  }
  // append the last coordinate
  tile = reader.GetGraphTile(current_de->endnode());
  coords.emplace_back(tile->get_node_ll(current_de->endnode()));

  return coords;
}

std::string serialize_geojson(const std::vector<vi::OpenLrEdges>& openlrs_edges,
                              vb::GraphReader& reader) {
  rapidjson::writer_wrapper_t writer(32768); // reserve 32 kb
  writer.start_object();
  writer("type", "FeatureCollection");
  writer.start_array("features");

  for (const auto& openlr_edges : openlrs_edges) {
    writer.start_object(); // single feature
    writer("type", "feature");
    writer.start_object("geometry");
    writer("type", "LineString");
    writer.start_array("coordinates");

    const auto& pts = get_lng_lat(openlr_edges, reader);
    auto first_p = pts.begin();
    for (; first_p != pts.end(); first_p++) {
      vm::PointLL coord{first_p->lng(), first_p->lat()};

      // first & last coordinate need to respect the offsets
      if (first_p == pts.begin() && openlr_edges.first_node_offset) {
        auto next_p = first_p;
        do {
          coord = first_p->PointAlongSegment(*next_p, openlr_edges.first_node_offset);
          next_p++;
        } while (next_p != pts.end() && openlr_edges.first_node_offset > first_p->Distance(coord));
      } else if (first_p == (pts.end() - 1) && openlr_edges.last_node_offset) {
        auto next_p = first_p - 1;
        do {
          coord = first_p->PointAlongSegment(*next_p, openlr_edges.last_node_offset);
          next_p--;
        } while (next_p != pts.begin() && openlr_edges.first_node_offset > first_p->Distance(coord));
      }
      writer.start_array(); // single coordinate
      writer(coord.lng());
      writer(coord.lat());
      writer.end_array(); // single coordinate
    }

    writer.end_array();  // coordinates
    writer.end_object(); // geometry
    writer.end_object(); // single feature

    writer.start_object("properties");
    writer("dummy", "test");
    writer.end_object(); // properties
  }

  writer.end_array();  // features
  writer.end_object(); // FeatureCollection

  return writer.get_buffer();
}
} // namespace

namespace valhalla {
namespace incidents {

std::string incident_worker_t::incidents(IncidentsAction action, rapidjson::Document& req) {
  LOG_WARN(endpoint_to_string.at(action) + " request");

  std::vector<OpenLrEdges> openlr_edges;
  openlr_edges.reserve(req.GetArray().Size());

  get_matched_edges(req, openlr_edges);

  switch (action) {
    case IncidentsAction::UPDATE:
    case IncidentsAction::DELETE:
    case IncidentsAction::RESET:
      // write to the tar
      break;
    case IncidentsAction::GEOJSON:
      return serialize_geojson(openlr_edges, *reader);
    default:
      LOG_ERROR("Can't be!");
      break;
  }

  return "";
}
} // namespace incidents
} // namespace valhalla
