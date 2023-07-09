#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/incidents/worker.h>

namespace vb = valhalla::baldr;
namespace vi = valhalla::incidents;
namespace vm = valhalla::midgard;

namespace {
static const std::unordered_map<vi::IncidentsAction, std::string> endpoint_to_string =
    {{vi::IncidentsAction::UPDATE, "/update"},
     {vi::IncidentsAction::DELETE, "/delete"},
     {vi::IncidentsAction::RESET, "/reset"},
     {vi::IncidentsAction::GEOJSON_MATCHES, "/geojson/matches"},
     {vi::IncidentsAction::GEOJSON_OPENLR, "/geojson/openlr"}};

std::vector<vm::PointLL> get_lng_lat(std::vector<vi::OpenLrEdge>::const_iterator& openlr_edge,
                                     vi::GraphReaderIncidents& reader,
                                     std::vector<vi::OpenLrEdge>::const_iterator end) {
  std::vector<vm::PointLL> coords;

  vb::graph_tile_ptr tile;
  const vb::DirectedEdge* current_de = nullptr;
  // could be there's only one edge for a whole openlr
  while (true) {
    if (openlr_edge == end) {
      break;
    }
    tile = reader.GetGraphTile(openlr_edge->edge_id, tile);
    current_de = reader.directededge(openlr_edge->edge_id, tile);
    coords.emplace_back(tile->get_node_ll(reader.GetBeginNodeId(current_de, tile)));
    if (!openlr_edge->is_last) {
      openlr_edge++;
    } else {
      break;
    }
  }
  // append the last coordinate
  tile = reader.GetGraphTile(current_de->endnode());
  coords.emplace_back(tile->get_node_ll(current_de->endnode()));

  return coords;
}

std::string serialize_geojson_matches(const std::vector<vi::OpenLrEdge>& openlrs_edges,
                                      vi::GraphReaderIncidents& reader) {
  rapidjson::writer_wrapper_t writer(32768); // reserve 32 kb
  writer.start_object();
  writer("type", "FeatureCollection");
  writer.start_array("features");

  writer.set_precision(6);

  auto write_coord = [](rapidjson::writer_wrapper_t& writer, const vm::PointLL coord) {
    writer.start_array(); // single coordinate
    writer(coord.lng());
    writer(coord.lat());
    writer.end_array(); // single coordinate
  };

  auto openlr_begin = openlrs_edges.begin();
  auto openlr_end = openlr_begin;
  while (openlr_end != openlrs_edges.end()) {
    writer.start_object(); // single feature
    writer("type", "Feature");
    writer.start_object("geometry");
    writer("type", "LineString");
    writer.start_array("coordinates");

    // will increment openlr_end to the last coord for an openlr segment
    auto pts = get_lng_lat(openlr_end, reader, openlrs_edges.end());
    assert(pts.size() >= 2U);

    // handle first and last points
    if (openlr_begin->breakpoint1) {
      auto& pt = pts.front();
      pt = pt.PointAlongSegment(pts[1], static_cast<double>(openlr_begin->breakpoint1) / 255.);
    }
    if (openlr_end->breakpoint2 != 255) {
      auto& pt = pts[pts.size() - 2];
      pts.back() =
          pt.PointAlongSegment(pts.back(), static_cast<double>(openlr_end->breakpoint2) / 255.);
    }

    // write all the points and sum up the total distance
    float distance = 0.f;
    auto first_p = pts.begin();
    for (; first_p != pts.end(); first_p++) {
      write_coord(writer, *first_p);

      // record the total distance
      if (first_p != (pts.end() - 1)) {
        distance += first_p->Distance(*(first_p + 1));
      }
    }

    writer.end_array();  // coordinates
    writer.end_object(); // geometry

    writer.start_object("properties");
    writer("distance", distance);
    writer.end_object(); // properties
    writer.end_object(); // single feature

    // increment to get past the last point of the previous openlr segment
    openlr_end++;
    openlr_begin = openlr_end;
  }

  writer.end_array();  // features
  writer.end_object(); // FeatureCollection

  return writer.get_buffer();
}

std::string serialize_geojson_openlr(rapidjson::Document& req_doc) {
  rapidjson::writer_wrapper_t writer(32768); // reserve 32 kb
  writer.start_object();
  writer("type", "FeatureCollection");
  writer.start_array("features");

  writer.set_precision(6);

  for (const auto& openlr_binary : req_doc.GetArray()) {
    const auto openlr = vb::OpenLR::OpenLr(openlr_binary.GetString(), true);

    writer.start_object(); // single feature
    writer("type", "Feature");
    writer.start_object("geometry");
    writer("type", "LineString");
    writer.start_array("coordinates");

    for (const auto& lrp : openlr.lrps) {
      writer.start_array(); // single coordinate
      writer(lrp.longitude);
      writer(lrp.latitude);
      writer.end_array(); // single coordinate
    }

    writer.end_array();  // coordinates
    writer.end_object(); // geometry

    writer.start_object("properties");

    float distance = 0.f;
    for (const auto& lrp : openlr.lrps) {
      // record the total distance
      if (!(lrp == *(openlr.lrps.end() - 1))) {
        distance += lrp.distance;
      }
    }

    auto poff = (static_cast<float>(openlr.poff) / 255.f) * static_cast<float>(openlr.getLength());
    auto noff = (static_cast<float>(openlr.noff) / 255.f) * static_cast<float>(openlr.getLength());
    writer("distance", static_cast<float>(distance) - noff - poff);
    writer("openlr", openlr_binary.GetString());
    writer("lrps", openlr.lrps.size());
    writer("poff", poff);
    writer("noff", noff);

    writer.end_object(); // properties
    writer.end_object(); // single feature
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

  switch (action) {
    case IncidentsAction::UPDATE:
    case IncidentsAction::DELETE:
    case IncidentsAction::RESET:
      // write to the tar
      write_traffic(get_matched_edges(req));
      break;
    case IncidentsAction::GEOJSON_MATCHES:
      return serialize_geojson_matches(get_matched_edges(req), *reader);
    case IncidentsAction::GEOJSON_OPENLR:
      return serialize_geojson_openlr(req);
    default:
      LOG_ERROR("Can't be!");
      break;
  }

  return "";
}
} // namespace incidents
} // namespace valhalla
