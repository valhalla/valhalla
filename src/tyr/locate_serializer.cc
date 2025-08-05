#include "baldr/graphreader.h"
#include "baldr/openlr.h"
#include "baldr/pathlocation.h"
#include "baldr/rapidjson_utils.h"
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

void get_access_restrictions(const graph_tile_ptr& tile,
                             rapidjson::writer_wrapper_t& writer,
                             uint32_t edge_idx) {
  for (const auto& res : tile->GetAccessRestrictions(edge_idx, kAllAccess)) {
    res.json(writer);
  }
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

void serialize_traffic_speed(const volatile baldr::TrafficSpeed& traffic_speed,
                             rapidjson::writer_wrapper_t& writer) {
  if (traffic_speed.speed_valid()) {
    writer.set_precision(2);
    writer("overall_speed", static_cast<uint64_t>(traffic_speed.get_overall_speed()));
    auto speed = static_cast<uint64_t>(traffic_speed.get_speed(0));
    if (speed == baldr::UNKNOWN_TRAFFIC_SPEED_KPH)
      writer("speed_0", nullptr);
    else
      writer("speed_0", speed);
    auto congestion = (traffic_speed.congestion1 - 1.0) / 62.0;
    if (congestion < 0)
      writer("congestion_0", nullptr);
    else {
      writer("congestion_0", congestion);
    }
    writer("breakpoint_0", traffic_speed.breakpoint1 / 255.0);

    speed = static_cast<uint64_t>(traffic_speed.get_speed(1));
    if (speed == baldr::UNKNOWN_TRAFFIC_SPEED_KPH)
      writer("speed_1", nullptr);
    else
      writer("speed_1", speed);
    congestion = (traffic_speed.congestion2 - 1.0) / 62.0;
    if (congestion < 0)
      writer("congestion_1", nullptr);
    else {
      writer("congestion_1", congestion);
    }
    writer("breakpoint_1", traffic_speed.breakpoint2 / 255.0);

    speed = static_cast<uint64_t>(traffic_speed.get_speed(2));
    if (speed == baldr::UNKNOWN_TRAFFIC_SPEED_KPH)
      writer("speed_2", nullptr);
    else
      writer("speed_2", speed);
    congestion = (traffic_speed.congestion3 - 1.0) / 62.0;
    if (congestion < 0)
      writer("congestion_2", nullptr);
    else {
      writer("congestion_2", congestion);
    }
    writer.set_precision(tyr::kDefaultPrecision);
  }
}

void serialize_edges(const PathLocation& location,
                     GraphReader& reader,
                     rapidjson::writer_wrapper_t& writer,
                     bool verbose) {
  writer.start_array("edges");
  for (const auto& edge : location.edges) {
    writer.start_object();
    try {
      // get the osm way id
      auto tile = reader.GetGraphTile(edge.id);
      auto* directed_edge = tile->directededge(edge.id);
      auto edge_info = tile->edgeinfo(directed_edge);
      // they want MOAR!
      if (verbose) {
        // live traffic information
        const volatile auto& traffic = tile->trafficspeed(directed_edge);

        // incident information
        if (traffic.has_incidents) {
          // TODO: incidents
        }
        writer.start_array("access_restrictions");
        get_access_restrictions(tile, writer, edge.id.id());
        writer.end_array();
        // write live_speed
        writer.start_object("live_speed");
        serialize_traffic_speed(traffic, writer);
        writer.end_object();

        // basic rest of it plus edge metadata
        writer.set_precision(tyr::kCoordinatePrecision);
        writer("correlated_lat", edge.projected.lat());
        writer("correlated_lon", edge.projected.lng());
        writer("side_of_street", edge.sos == PathLocation::LEFT
                                     ? std::string("left")
                                     : (edge.sos == PathLocation::RIGHT ? std::string("right")
                                                                        : std::string("neither")));

        writer("linear_reference", linear_reference(directed_edge, edge.percent_along, edge_info));
        writer.set_precision(5);
        writer("percent_along", edge.percent_along);
        writer.set_precision(1);
        writer("distance", edge.distance);
        writer("shoulder", directed_edge->shoulder());
        writer("heading", edge.projected_heading);
        writer.set_precision(tyr::kDefaultPrecision);
        writer("outbound_reach", static_cast<int64_t>(edge.outbound_reach));
        writer("inbound_reach", static_cast<int64_t>(edge.inbound_reach));

        writer.start_object("edge_info");
        edge_info.json(writer);
        writer.end_object();

        writer.start_object("edge");
        directed_edge->json(writer);
        writer.end_object();

        writer.start_object("edge_id");
        edge.id.json(writer);
        writer.end_object();

        // historical traffic information
        writer.start_array("predicted_speeds");
        if (directed_edge->has_predicted_speed()) {
          for (auto sec = 0; sec < midgard::kSecondsPerWeek; sec += 5 * midgard::kSecPerMinute) {
            writer(static_cast<uint64_t>(tile->GetSpeed(directed_edge, kPredictedFlowMask, sec)));
          }
        }
        writer.end_array();
      } // they want it lean and mean
      else {
        writer("way_id", static_cast<uint64_t>(edge_info.wayid()));
        writer.set_precision(tyr::kCoordinatePrecision);
        writer("correlated_lat", edge.projected.lat());
        writer("correlated_lon", edge.projected.lng());
        writer("side_of_street", edge.sos == PathLocation::LEFT
                                     ? std::string("left")
                                     : (edge.sos == PathLocation::RIGHT ? std::string("right")
                                                                        : std::string("neither")));
        writer.set_precision(5);
        writer("percent_along", edge.percent_along);
        writer.set_precision(tyr::kDefaultPrecision);
      }
    } catch (...) {
      // this really shouldnt ever get hit
      LOG_WARN("Expected edge not found in graph but found by loki::search!");
    }
    writer.end_object();
  }
  writer.end_array();
}

void serialize_nodes(const PathLocation& location,
                     GraphReader& reader,
                     rapidjson::writer_wrapper_t& writer,
                     bool verbose) {
  // get the nodes we need
  std::unordered_set<uint64_t> nodes;
  for (const auto& e : location.edges) {
    if (e.end_node()) {
      nodes.emplace(reader.GetGraphTile(e.id)->directededge(e.id)->endnode());
    }
  }
  writer.start_array("nodes");
  for (auto node_id : nodes) {
    writer.start_object();
    GraphId n(node_id);
    graph_tile_ptr tile = reader.GetGraphTile(n);
    auto* node_info = tile->node(n);

    if (verbose) {
      node_info->json(tile, writer);

      writer.start_object("node_id");
      n.json(writer);
      writer.end_object();
    } else {
      midgard::PointLL node_ll = tile->get_node_ll(n);
      writer.set_precision(tyr::kCoordinatePrecision);
      writer("lon", node_ll.first);
      writer("lat", node_ll.second);
      writer.set_precision(tyr::kDefaultPrecision);
      // TODO: osm_id
    }
    writer.end_object();
  }
  writer.end_array();
}

void serialize(rapidjson::writer_wrapper_t& writer,
               const PathLocation& location,
               GraphReader& reader,
               bool verbose) {
  // serialze all the edges
  writer.start_object();
  writer.set_precision(tyr::kCoordinatePrecision);
  writer("input_lat", location.latlng_.lat());
  writer("input_lon", location.latlng_.lng());
  writer.set_precision(tyr::kDefaultPrecision);
  serialize_edges(location, reader, writer, verbose);
  serialize_nodes(location, reader, writer, verbose);

  writer.end_object();
}

void serialize(rapidjson::writer_wrapper_t& writer,
               const midgard::PointLL& ll,
               const std::string& reason,
               bool verbose) {
  writer.start_object();
  writer.set_precision(tyr::kCoordinatePrecision);
  writer("input_lat", ll.lat());
  writer("input_lon", ll.lng());
  writer.set_precision(tyr::kDefaultPrecision);
  writer("edges", nullptr);
  writer("nodes", nullptr);

  if (verbose) {
    writer("reason", reason);
  }
  writer.end_object();
}
} // namespace

namespace valhalla {
namespace tyr {

std::string serializeLocate(const Api& request,
                            const std::vector<baldr::Location>& locations,
                            const std::unordered_map<baldr::Location, PathLocation>& projections,
                            GraphReader& reader) {
  rapidjson::writer_wrapper_t writer(4096);
  writer.start_array();

  for (const auto& location : locations) {
    try {
      serialize(writer, projections.at(location), reader, request.options().verbose());
    } catch (const std::exception& e) {
      serialize(writer, location.latlng_, "No data found for location", request.options().verbose());
    }
  }
  writer.end_array();
  return writer.get_buffer();
}

} // namespace tyr
} // namespace valhalla
