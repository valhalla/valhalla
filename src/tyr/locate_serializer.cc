#include "baldr/json.h"
#include "baldr/openlr.h"
#include "tyr/serializers.h"
#include <cstdint>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace {

OpenLR::LocationReferencePoint::FormOfWay get_fow(const baldr::DirectedEdge* de) {
  if (de->classification() == valhalla::baldr::RoadClass::kMotorway)
    return OpenLR::LocationReferencePoint::MOTORWAY;
  else if (de->roundabout())
    return OpenLR::LocationReferencePoint::ROUNDABOUT;
  else if (de->use() == valhalla::baldr::Use::kRamp ||
           de->use() == valhalla::baldr::Use::kTurnChannel)
    return OpenLR::LocationReferencePoint::SLIPROAD;
  else if ((de->forwardaccess() & kVehicularAccess) && (de->reverseaccess() & kVehicularAccess))
    return OpenLR::LocationReferencePoint::MULTIPLE_CARRIAGEWAY;
  else if ((de->forwardaccess() & kVehicularAccess) || (de->reverseaccess() & kVehicularAccess))
    return OpenLR::LocationReferencePoint::SINGLE_CARRIAGEWAY;

  return OpenLR::LocationReferencePoint::OTHER;
}

json::ArrayPtr get_access_restrictions(const graph_tile_ptr& tile, uint32_t edge_idx) {
  auto arr = json::array({});
  for (const auto& res : tile->GetAccessRestrictions(edge_idx, kAllAccess)) {
    arr->emplace_back(res.json());
  };

  return arr;
}

std::string
linear_reference(const baldr::DirectedEdge* de, float percent_along, const EdgeInfo& edgeinfo) {
  const auto fow = get_fow(de);
  const auto frc = static_cast<uint8_t>(de->classification());

  auto shape = edgeinfo.shape();
  if (!de->forward())
    std::reverse(shape.begin(), shape.end());
  float forward_heading = midgard::tangent_angle(0, shape.front(), shape, 20.f, true);
  float reverse_heading = midgard::tangent_angle(shape.size() - 1, shape.back(), shape, 20.f, false);

  std::vector<OpenLR::LocationReferencePoint> lrps;
  lrps.emplace_back(shape.front().lng(), shape.front().lat(), forward_heading, frc, fow, nullptr,
                    de->length(), frc);
  lrps.emplace_back(shape.back().lng(), shape.back().lat(), reverse_heading, frc, fow, &lrps.back());

  uint8_t poff = static_cast<uint8_t>(std::min(255.f, percent_along * 255 + .5f));

  return OpenLR::OpenLr{lrps,
                        poff,
                        0,
                        true,
                        OpenLR::Orientation::FirstLrpTowardsSecond,
                        OpenLR::SideOfTheRoad::DirectlyOnRoadOrNotApplicable}
      .toBase64();
}

json::ArrayPtr serialize_edges(const PathLocation& location, GraphReader& reader, bool verbose) {
  auto array = json::array({});
  for (const auto& edge : location.edges) {
    try {
      // get the osm way id
      auto tile = reader.GetGraphTile(edge.id);
      auto* directed_edge = tile->directededge(edge.id);
      auto edge_info = tile->edgeinfo(directed_edge);
      // they want MOAR!
      if (verbose) {
        // live traffic information
        const volatile auto& traffic = tile->trafficspeed(directed_edge);
        auto live_speed = traffic.json();

        // incident information
        if (traffic.has_incidents) {
          // TODO: incidents
        }

        // historical traffic information
        auto predicted_speeds = json::array({});
        if (directed_edge->has_predicted_speed()) {
          for (auto sec = 0; sec < midgard::kSecondsPerWeek; sec += 5 * midgard::kSecPerMinute) {
            predicted_speeds->emplace_back(
                static_cast<uint64_t>(tile->GetSpeed(directed_edge, kPredictedFlowMask, sec)));
          }
        }

        // basic rest of it plus edge metadata
        array->emplace_back(json::map({
            {"correlated_lat", json::fixed_t{edge.projected.lat(), 6}},
            {"correlated_lon", json::fixed_t{edge.projected.lng(), 6}},
            {"side_of_street",
             edge.sos == PathLocation::LEFT
                 ? std::string("left")
                 : (edge.sos == PathLocation::RIGHT ? std::string("right") : std::string("neither"))},
            {"percent_along", json::fixed_t{edge.percent_along, 5}},
            {"distance", json::fixed_t{edge.distance, 1}},
            {"heading", json::fixed_t{edge.projected_heading, 1}},
            {"outbound_reach", static_cast<int64_t>(edge.outbound_reach)},
            {"inbound_reach", static_cast<int64_t>(edge.inbound_reach)},
            {"edge_id", edge.id.json()},
            {"edge", directed_edge->json()},
            {"edge_info", edge_info.json()},
            {"linear_reference", linear_reference(directed_edge, edge.percent_along, edge_info)},
            {"predicted_speeds", predicted_speeds},
            {"live_speed", live_speed},
            {"access_restrictions", get_access_restrictions(tile, edge.id.id())},
        }));
      } // they want it lean and mean
      else {
        array->emplace_back(json::map({
            {"way_id", static_cast<uint64_t>(edge_info.wayid())},
            {"correlated_lat", json::fixed_t{edge.projected.lat(), 6}},
            {"correlated_lon", json::fixed_t{edge.projected.lng(), 6}},
            {"side_of_street",
             edge.sos == PathLocation::LEFT
                 ? std::string("left")
                 : (edge.sos == PathLocation::RIGHT ? std::string("right") : std::string("neither"))},
            {"percent_along", json::fixed_t{edge.percent_along, 5}},
        }));
      }
    } catch (...) {
      // this really shouldnt ever get hit
      LOG_WARN("Expected edge not found in graph but found by loki::search!");
    }
  }
  return array;
}

json::ArrayPtr serialize_nodes(const PathLocation& location, GraphReader& reader, bool verbose) {
  // get the nodes we need
  std::unordered_set<uint64_t> nodes;
  for (const auto& e : location.edges) {
    if (e.end_node()) {
      nodes.emplace(reader.GetGraphTile(e.id)->directededge(e.id)->endnode());
    }
  }
  // ad them into an array of json
  auto array = json::array({});
  for (auto node_id : nodes) {
    GraphId n(node_id);
    graph_tile_ptr tile = reader.GetGraphTile(n);
    auto* node_info = tile->node(n);
    json::MapPtr node;
    if (verbose) {
      node = node_info->json(tile);
      node->emplace("node_id", n.json());
    } else {
      midgard::PointLL node_ll = tile->get_node_ll(n);
      node = json::map({
          {"lon", json::fixed_t{node_ll.first, 6}}, {"lat", json::fixed_t{node_ll.second, 6}},
          // TODO: osm_id
      });
    }
    array->emplace_back(node);
  }
  // give them back
  return array;
}

json::MapPtr serialize(const PathLocation& location, GraphReader& reader, bool verbose) {
  // serialze all the edges
  auto m = json::map({
      {"edges", serialize_edges(location, reader, verbose)},
      {"nodes", serialize_nodes(location, reader, verbose)},
      {"input_lat", json::fixed_t{location.latlng_.lat(), 6}},
      {"input_lon", json::fixed_t{location.latlng_.lng(), 6}},
  });
  return m;
}

json::MapPtr serialize(const midgard::PointLL& ll, const std::string& reason, bool verbose) {
  auto m = json::map({
      {"edges", static_cast<std::nullptr_t>(nullptr)},
      {"nodes", static_cast<std::nullptr_t>(nullptr)},
      {"input_lat", json::fixed_t{ll.lat(), 6}},
      {"input_lon", json::fixed_t{ll.lng(), 6}},
  });
  if (verbose) {
    m->emplace("reason", reason);
  }

  return m;
}
} // namespace

namespace valhalla {
namespace tyr {

std::string serializeLocate(const Api& request,
                            const std::vector<baldr::Location>& locations,
                            const std::unordered_map<baldr::Location, PathLocation>& projections,
                            GraphReader& reader) {
  auto json = json::array({});
  for (const auto& location : locations) {
    try {
      json->emplace_back(serialize(projections.at(location), reader, request.options().verbose()));
    } catch (const std::exception& e) {
      json->emplace_back(
          serialize(location.latlng_, "No data found for location", request.options().verbose()));
    }
  }

  std::stringstream ss;
  ss << *json;
  return ss.str();
}

} // namespace tyr
} // namespace valhalla
