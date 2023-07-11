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

  vb::graph_tile_ptr tile;
  auto openlr_begin = openlrs_edges.begin();
  auto openlr_end = openlr_begin;
  do {
    writer.start_object(); // single feature
    writer("type", "Feature");
    writer.start_object("geometry");
    writer("type", "LineString");
    writer.start_array("coordinates");

    std::vector<vm::PointLL> pts;
    // find the next occurrence of is_last = true
    openlr_end = std::find_if(openlr_begin, openlrs_edges.end(),
                              [](const vi::OpenLrEdge& edge) { return edge.is_last; });

    // save first point's poff_start_offset before incrementing openlr_begin
    const auto poff_start_offset = openlr_begin->poff_start_offset;
    const auto first_edge_length = openlr_begin->length;
    for (; openlr_begin != (openlr_end + 1); openlr_begin++) {
      // auto* de = reader.directededge(openlr_begin->edge_id, tile);
      auto* de = reader.directededge(openlr_begin->edge_id, tile);
      auto ei = tile->edgeinfo(de);
      auto shp = ei.shape();
      if (!de->forward()) {
        std::reverse(shp.begin(), shp.end());
      }
      std::copy(shp.begin(), shp.end(), std::back_inserter(pts));
    }
    assert(pts.size() >= 2U);

    // remove all those duplicate points we introduced just now
    auto last = std::unique(pts.begin(), pts.end());
    pts.erase(last, pts.end());

    // handle first edge or start of trivial edge
    if (poff_start_offset) {
      auto& pt = pts.front();
      pt = pt.PointAlongSegment(pts[1], static_cast<double>(poff_start_offset / first_edge_length));
    }
    // handle last edge or end of trivial edge
    if (openlr_end->is_last && openlr_end->noff_start_offset) {
      auto& pt = pts[pts.size() - 2];
      pts.back() =
          pt.PointAlongSegment(pts.back(), static_cast<double>(
                                               (openlr_end->length - openlr_end->noff_start_offset) /
                                               openlr_end->length));
    }

    for (const auto& pt : pts) {
      writer.start_array(); // single coordinate
      writer(pt.lng());
      writer(pt.lat());
      writer.end_array(); // single coordinate
    }

    writer.end_array();  // coordinates
    writer.end_object(); // geometry

    writer.start_object("properties");
    writer("distance", static_cast<uint64_t>(vm::length(pts)));
    writer.end_object(); // properties
    writer.end_object(); // single feature

    // increment to get past the last point of the previous openlr segment
    openlr_end++;
    openlr_begin = openlr_end;
  } while (openlr_end != openlrs_edges.end());

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
  switch (action) {
    case IncidentsAction::RESET:
      if (req.GetArray().Size()) {
        write_traffic(get_matched_edges(req), action);
      } else {
        write_traffic({}, action);
      }
    case IncidentsAction::UPDATE:
    case IncidentsAction::DELETE:
      // write to the tar
      write_traffic(get_matched_edges(req), action);
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
