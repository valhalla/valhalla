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

std::vector<vm::PointLL> get_lng_lat(const std::vector<vb::GraphId>& edge_ids,
                                     vb::GraphReader& reader) {
  std::vector<vm::PointLL> coords(edge_ids.size() + 1ULL);
  vb::graph_tile_ptr tile;
  const vb::DirectedEdge* current_de = nullptr;
  for (const auto edge_id : edge_ids) {
    tile = reader.GetGraphTile(edge_id);
    current_de = reader.directededge(edge_id, tile);
    coords.emplace_back(tile->get_node_ll(reader.GetBeginNodeId(current_de, tile)));
  }
  // append the last coordinate
  tile = reader.GetGraphTile(current_de->endnode());
  coords.emplace_back(tile->get_node_ll(current_de->endnode()));

  return coords;
}

std::string serialize_geojson(const std::vector<std::vector<vb::GraphId>>& all_edge_ids,
                              vb::GraphReader& reader) {
  rapidjson::writer_wrapper_t writer(32768); // reserve 32 kb
  writer.start_object();
  writer("type", "FeatureCollection");
  writer.start_array("features");

  for (const auto& openlr_edge_ids : all_edge_ids) {
    writer.start_object(); // single feature
    writer("type", "feature");
    writer.start_object("geometry");
    writer("type", "LineString");
    writer.start_array("coordinates");

    for (const auto& coord : get_lng_lat(openlr_edge_ids, reader)) {
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

  const auto edge_ids = get_matched_edges(req);
  LOG_WARN(endpoint_to_string.at(action) + " request");
  switch (action) {
    case IncidentsAction::UPDATE:
    case IncidentsAction::DELETE:
    case IncidentsAction::RESET:
      break;
    case IncidentsAction::GEOJSON:
      return serialize_geojson(edge_ids, *reader);
    default:
      LOG_ERROR("Can't be!");
      break;
  }

  return "";
}
} // namespace incidents
} // namespace valhalla
