#include "baldr/json.h"
#include "baldr/openlr.h"
#include "sif/dynamiccost.h"
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

const DirectedEdge* get_opposing_edge(const DirectedEdge* de, GraphReader& reader) {
  auto tile = reader.GetGraphTile(de->endnode());
  auto opp = de->opp_index();
  auto end_node = reader.GetEndNode(de, tile);

  return tile->directededge(end_node->edge_index() + opp);
}

std::string get_edge_shape(const DirectedEdge* de, GraphReader& reader) {
  auto tile = reader.GetGraphTile(de->endnode());
  auto edge_info = tile->edgeinfo(de);

  return midgard::encode(edge_info.shape());
}

json::MapPtr get_full_road_segment(const DirectedEdge* de,
                                   const std::shared_ptr<sif::DynamicCost>& costing,
                                   const double percent_along,
                                   GraphReader& reader) {
  // first, find the beginning of the road segment
  // things we need: the opposing edge and its start node
  auto opp_edge = get_opposing_edge(de, reader);

  if (de->shortcut() || de->deadend() || opp_edge->deadend()) {
    return json::map({});
  }
  auto node_id = opp_edge->endnode();
  auto node = reader.GetGraphTile(node_id)->node(node_id);
  auto edge = de;

  std::unordered_set<const DirectedEdge*> added_edges;
  std::vector<const DirectedEdge*> edges;
  // crawl in reverse direction until we find a "true" intersection given the costing
  while (true) {
    int allowed_cnt = 0;
    const DirectedEdge* incoming_pred = nullptr;
    const DirectedEdge* outgoing_pred = nullptr;
    auto tile = reader.GetGraphTile(node_id);
    // check the number of outgoing edges accessible with the provided costing
    for (int i = 0; i < node->edge_count(); ++i) {
      auto outgoing_edge = tile->directededge(node->edge_index() + i);

      // except our current edge
      if (outgoing_edge == edge)
        continue;
      auto incoming_edge = get_opposing_edge(outgoing_edge, reader);
      if ((costing->Allowed(incoming_edge, reader.GetGraphTile(outgoing_edge->endnode()),
                            sif::kDisallowShortcut) ||
           costing->Allowed(outgoing_edge, reader.GetGraphTile(node_id), sif::kDisallowShortcut)) &&
          !(incoming_edge->deadend() || outgoing_edge->deadend())) {

        allowed_cnt++;
        outgoing_pred = outgoing_edge;
        incoming_pred = incoming_edge;
      }
    }
    std::function<void(const baldr::GraphId&, const bool, const bool)> expand_back;
    expand_back = [&](const baldr::GraphId& trans_node_id, const bool from_transition,
                      const bool up) {
      graph_tile_ptr tile = reader.GetGraphTile(trans_node_id);
      if (tile == nullptr) {
        return;
      }
      const baldr::NodeInfo* trans_node = tile->node(trans_node_id);
      // get the count of incoming edges
      for (int i = 0; i < trans_node->edge_count(); ++i) {
        auto outgoing_edge = tile->directededge(trans_node->edge_index() + i);
        auto incoming_edge = get_opposing_edge(outgoing_edge, reader);
        // auto opp_tile = reader.GetGraphTile(incoming_edge->endnode());
        // auto name = tile->edgeinfo(outgoing_edge).GetNames()[0];
        // auto opp_name = opp_tile->edgeinfo(incoming_edge).GetNames()[0];

        if ((costing->Allowed(incoming_edge, reader.GetGraphTile(outgoing_edge->endnode()),
                              sif::kDisallowShortcut) ||
             costing->Allowed(outgoing_edge, reader.GetGraphTile(node_id), sif::kDisallowShortcut)) &&
            !(incoming_edge->deadend() || outgoing_edge->deadend())) {

          allowed_cnt++;
          incoming_pred = incoming_edge;
          outgoing_pred = outgoing_pred;
        }
      }

      if (!from_transition && trans_node->transition_count() > 0) {
        const baldr::NodeTransition* trans = tile->transition(trans_node->transition_index());
        for (uint32_t i = 0; i < trans_node->transition_count(); ++i, ++trans) {
          if (!up == trans->up())
            continue;
          expand_back(trans->endnode(), true, up);
        }
      }
    };
    // follow node transitions
    for (int i = 0; i < node->transition_count(); ++i) {
      const baldr::NodeTransition* trans = tile->transition(node->transition_index() + i);
      expand_back(trans->endnode(), false, trans->up());
    }

    // if there's exactly one allowed incoming edge (except for the current opposing edge)
    if (allowed_cnt == 1) {
      // keep going back
      if (!(added_edges.insert(incoming_pred)).second) {
        LOG_WARN("Duplicate edge when finding beginning of road segment for edge.");
        break;
      } else {
        edges.push_back(incoming_pred);
      }
      edge = incoming_pred;
      node_id = outgoing_pred->endnode();
      node = reader.GetGraphTile(node_id)->node(node_id);
    } else {
      // we've found the start of the road segment
      // either because this is a valid intersection given the costing
      // or because there is no other edge to inspect
      break;
    }
  }
  // we've found our start node
  auto start_node_id = node_id;

  // resort and get our initial edge in there
  std::reverse(edges.begin(), edges.end());
  if (!(added_edges.insert(de)).second) {
    LOG_WARN("Initial edge already inserted into road segment edges");
  } else {
    edges.push_back(de);
  }

  // now move forward
  node_id = de->endnode();
  node = reader.GetGraphTile(node_id)->node(node_id);
  edge = de;

  while (true) {
    int allowed_cnt = 0;
    auto tile = reader.GetGraphTile(node_id);
    const DirectedEdge* possible_next = nullptr;

    // check the number of outgoing edges accessible with the provided costing
    for (int i = 0; i < node->edge_count(); ++i) {
      auto candidate_edge = tile->directededge(node->edge_index() + i);
      auto opp_candidate_edge = get_opposing_edge(candidate_edge, reader);
      if (edge == opp_candidate_edge)
        continue;
      if ((costing->Allowed(candidate_edge, tile, sif::kDisallowShortcut) ||
           costing->Allowed(opp_candidate_edge, reader.GetGraphTile(candidate_edge->endnode()),
                            sif::kDisallowShortcut)) &&
          !(candidate_edge->deadend() || opp_candidate_edge->deadend())) {
        allowed_cnt++;
        possible_next = candidate_edge;
      }
    }
    std::function<void(const baldr::GraphId&, const bool, const bool)> expand_forw;
    expand_forw = [&](const baldr::GraphId& trans_node_id, const bool from_transition,
                      const bool up) {
      graph_tile_ptr tile = reader.GetGraphTile(trans_node_id);
      if (tile == nullptr) {
        return;
      }
      const baldr::NodeInfo* trans_node = tile->node(trans_node_id);
      // get the count of outgoing edges
      for (int i = 0; i < trans_node->edge_count(); ++i) {
        auto candidate_edge = tile->directededge(trans_node->edge_index() + i);
        auto opp_candidate_edge = get_opposing_edge(candidate_edge, reader);
        if ((costing->Allowed(candidate_edge, tile, sif::kDisallowShortcut) ||
             costing->Allowed(opp_candidate_edge, reader.GetGraphTile(candidate_edge->endnode()),
                              sif::kDisallowShortcut)) &&
            !(candidate_edge->deadend() || opp_candidate_edge->deadend())) {
          allowed_cnt++;
          possible_next = candidate_edge;
        }
      }

      if (!from_transition && trans_node->transition_count() > 0) {
        const baldr::NodeTransition* trans = tile->transition(trans_node->transition_index());
        for (uint32_t i = 0; i < trans_node->transition_count(); ++i, ++trans) {
          if (!up == trans->up())
            continue;
          expand_forw(trans->endnode(), true, up);
        }
      }
    };
    // follow node transitions
    for (int i = 0; i < node->transition_count(); ++i) {
      const baldr::NodeTransition* trans = tile->transition(node->transition_index() + i);
      expand_forw(trans->endnode(), false, trans->up());
    }

    if (allowed_cnt == 1) {
      // keep moving forwards
      if (!(added_edges.insert(possible_next)).second) {
        LOG_WARN("Duplicate edge when finding beginning of road segment for edge.");
        break;
      } else {
        edges.push_back(possible_next);
      }

      edge = possible_next;
      node_id = possible_next->endnode();
      node = reader.GetGraphTile(node_id)->node(node_id);
    } else {
      break;
    }
  }

  double length = 0;
  double percent_along_total = 0;
  bool past_correlated_edge = false;
  std::function<double(const PointLL&, const PointLL)> segment_length;
  segment_length = [&](const PointLL& from, const PointLL to) { return from.Distance(to); };
  // assemble the shape
  std::list<midgard::PointLL> concatenated_shape;
  for (auto e : edges) {
    auto tile = reader.GetGraphTile(e->endnode());
    auto edge_info = tile->edgeinfo(e);
    auto shape = edge_info.shape();
    if (!e->forward())
      std::reverse(shape.begin(), shape.end());
    if (concatenated_shape.size())
      concatenated_shape.pop_back();
    // walk the edge segments
    auto shape_itr = shape.begin();
    auto next_shape_itr = ++shape.begin();

    // get the edge length
    double acc_ = 0;
    for (; next_shape_itr != shape.end(); ++shape_itr, ++next_shape_itr) {
      auto dist = segment_length(*shape_itr, *next_shape_itr);
      acc_ += dist;
    }

    // if this edge is the correlated edge, calculate
    // the percent_along along the complete road segment
    if (e == de) {
      percent_along_total = length + (acc_ * percent_along);
    }
    length += acc_;
    concatenated_shape.insert(concatenated_shape.end(), shape.begin(), shape.end());
  }

  // walk the edge segments
  auto shape_itr = concatenated_shape.begin();
  auto next_shape_itr = ++concatenated_shape.begin();

  double acc_;
  PointLL mid;
  for (; next_shape_itr != concatenated_shape.end(); ++shape_itr, ++next_shape_itr) {
    auto dist = (*shape_itr).Distance(*next_shape_itr);
    acc_ += dist;

    // if we've passed the 50% threshold, stop and get the exact mid point
    if (acc_ / length > 0.5f) {
      auto missing_length_m = (length * 0.5f) - (acc_ - dist);
      mid = (*shape_itr).PointAlongSegment(*next_shape_itr, missing_length_m / dist);
      break;
    }
  }

  if (next_shape_itr == concatenated_shape.end()) {
    LOG_ERROR("Unexpected end of shape.");
  }

  std::string shape = midgard::encode(concatenated_shape);
  auto start_tile = reader.GetGraphTile(start_node_id);
  auto start_node = start_tile->node(start_node_id);
  auto start_node_map =
      json::map({{"id", start_node_id.json()}, {"node", start_node->json(start_tile)}});
  auto end_tile = reader.GetGraphTile(node_id);
  auto end_node = end_tile->node(node_id);
  auto end_node_map = json::map({{"id", node_id.json()}, {"node", end_node->json(end_tile)}});
  auto intersection = json::map({{"start_node", start_node_map}, {"end_node", end_node_map}});
  auto mid_point =
      json::map({{"lat", json::fixed_t{mid.lat(), 6}}, {"lon", json::fixed_t{mid.lng(), 6}}});
  auto road_segments = json::map({{"shape", shape},
                                  {"intersections", intersection},
                                  {"mid_point", mid_point},
                                  {"percent_along", json::fixed_t{percent_along_total / length, 5}}});

  return road_segments;
}

json::ArrayPtr serialize_edges(const PathLocation& location,
                               GraphReader& reader,
                               bool verbose,
                               bool intersections,
                               sif::cost_ptr_t costing) {
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
        auto info = json::map({
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
        });

        if (intersections)
          info->insert({"full_road_segment",
                        get_full_road_segment(directed_edge, costing, edge.percent_along, reader)});

        array->emplace_back(info);
      } // they want it lean and mean
      else {
        auto info = json::map({
            {"way_id", static_cast<uint64_t>(edge_info.wayid())},
            {"correlated_lat", json::fixed_t{edge.projected.lat(), 6}},
            {"correlated_lon", json::fixed_t{edge.projected.lng(), 6}},
            {"side_of_street",
             edge.sos == PathLocation::LEFT
                 ? std::string("left")
                 : (edge.sos == PathLocation::RIGHT ? std::string("right") : std::string("neither"))},
            {"percent_along", json::fixed_t{edge.percent_along, 5}},
        });

        if (intersections)
          info->insert({"full_road_segment",
                        get_full_road_segment(directed_edge, costing, edge.percent_along, reader)});

        array->emplace_back(info);
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

json::MapPtr serialize(const PathLocation& location,
                       GraphReader& reader,
                       bool verbose,
                       bool road_segments,
                       sif::cost_ptr_t costing) {
  // serialze all the edges
  auto m = json::map({
      {"edges", serialize_edges(location, reader, verbose, road_segments, costing)},
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
                            GraphReader& reader,
                            sif::cost_ptr_t costing) {
  auto json = json::array({});
  for (const auto& location : locations) {
    try {
      json->emplace_back(serialize(projections.at(location), reader, request.options().verbose(),
                                   request.options().road_segments(), costing));
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
