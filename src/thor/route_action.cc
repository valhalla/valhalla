#include "thor/worker.h"
#include <cstdint>

#include "baldr/json.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "thor/attributes_controller.h"

#include <valhalla/proto/tripcommon.pb.h>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {
// Threshold for running a second pass pedestrian route with adjusted A*. The first
// pass for pedestrian routes is run with an aggressive A* threshold based on walking
// speed. If ferries are included in the path the A* heuristic rules can be violated
// which can lead to irregular paths. Running a second pass with less aggressive
// A* can take excessive time for longer paths - so exclude them to protect the service.
constexpr float kPedestrianMultipassThreshold = 50000.0f; // 50km

/**
 * Check if the paths meet at opposing edges (but not at a node). If so, add a route discontinuity
 * so that the shape / distance along the path is adjusted at the location.
 */
void via_discontinuity(
    GraphReader& reader,
    const valhalla::Location& loc,
    const GraphId& in,
    const GraphId& out,
    std::unordered_map<size_t, std::pair<RouteDiscontinuity, RouteDiscontinuity>>& vias,
    const size_t path_index,
    const bool flip_index) {
  // Find the path edges within the locations.
  auto in_pe =
      std::find_if(loc.path_edges().begin(), loc.path_edges().end(),
                   [&in](const valhalla::Location::PathEdge& e) { return e.graph_id() == in; });
  auto out_pe =
      std::find_if(loc.path_edges().begin(), loc.path_edges().end(),
                   [&out](const valhalla::Location::PathEdge& e) { return e.graph_id() == out; });

  // Could not find the edges. This seems like it should not happen. Log a warning
  // and do not add a discontinuity.
  if (in_pe == loc.path_edges().end() || out_pe == loc.path_edges().end()) {
    LOG_WARN("Could not find connecting edges within via_discontinuity");
    return;
  }

  // Do not add a discontinuity if the connections are at a graph node we dont need to signal a
  // discontinuity.
  if (in_pe->begin_node() || in_pe->end_node() || out_pe->begin_node() || out_pe->end_node()) {
    return;
  }

  // Add a discontinuity if the edges are opposing
  GraphId in_edge_id(in_pe->graph_id());
  GraphId out_edge_id(out_pe->graph_id());
  GraphId opp_edge_id = reader.GetOpposingEdgeId(in_edge_id);
  if (opp_edge_id == out_edge_id) {
    PointLL snap_ll(in_pe->ll().lng(), in_pe->ll().lat());
    float dist_along = in_pe->percent_along();

    // Insert a discontinuity so the last edge of the first segment is trimmed at the beginning
    // from 0 to dist_along. Set the first
    vias.insert(
        {path_index + (flip_index ? 1 : 0), {{false, PointLL(), 0.0f}, {true, snap_ll, dist_along}}});

    // Insert a second discontinuity so the next (opposing) edge is trimmed at the end from
    // 1-dist along to 1
    vias.insert({path_index + (flip_index ? 0 : 1),
                 {{false, snap_ll, 1.0f - dist_along}, {true, PointLL(), 1.0f}}});
  }
}

/**
// removes any edges from the location that aren't connected to it (because of radius)
void remove_edges(const GraphId& edge_id, valhalla::Location& loc, GraphReader& reader) {
  // find the path edge at this point
  auto pe =
      std::find_if(loc.path_edges().begin(), loc.path_edges().end(),
                   [&edge_id](const valhalla::Location::PathEdge& e) { return e.graph_id() == edge_id;
});
  // if its in the middle of the edge it can only be this edge or the opposing depending on type
  if (!pe->begin_node() && !pe->end_node()) {
    GraphId opposing;
    if (loc.type() == valhalla::Location::kBreak || loc.type() == valhalla::Location::kVia)
      opposing = reader.GetOpposingEdgeId(edge_id);
    // remove anything that isnt one of these two edges
    for (int i = 0; i < loc.path_edges_size(); ++i) {
      if (loc.path_edges(i).graph_id() != edge_id && loc.path_edges(i).graph_id() != opposing) {
        loc.mutable_path_edges()->SwapElements(i, loc.path_edges_size() - 1);
        loc.mutable_path_edges()->RemoveLast();
      }
    }
    return;
  }

  // if its at the begin node lets center our sights on that
  const GraphTile* tile = reader.GetGraphTile(edge_id);
  const auto* edge = tile->directededge(edge_id);
  const auto* node = reader.GetEndNode(edge, tile);
  if (pe->begin_node()) {
    const auto* opp_edge = tile->directededge(node->edge_index() + edge->opp_index());
    node = reader.GetEndNode(opp_edge, tile);
  }

  // TODO: nuke any edges that aren't connected to this node
  GraphId start_edge = tile->header()->graphid();
  start_edge.set_id(node->edge_index);
  GraphId end_edge = start_edge + node->edge_count;
  for (int i = 0; i < loc.path_edges_size(); ++i) {
    if (loc.path_edges(i).graph_id() < start_edge || loc.path_edges(i) >= end_edge) {
      loc.mutable_path_edges()->SwapElements(i, loc.path_edges_size() - 1);
      loc.mutable_path_edges()->RemoveLast();
    }
  }
}*/

/**
 * offset a time in one timezone by some number of seconds to a time in another timezone
 *
 * @param reader   graphreader for tile/edge/node access
 * @param in_dt    the input date time string
 * @param in_edge  the input edgeid (used for timezone lookup)
 * @param offset   the offset in seconds from the input date time string
 * @param out_edge the output edgeid (used for timezone lookup)
 * @return out_dt  the time at the out_edge in local time after the offset is applied to the in_dt
 */
std::string offset_date(GraphReader& reader,
                        const std::string& in_dt,
                        const GraphId& in_edge,
                        float offset,
                        const GraphId& out_edge) {
  // get the timezone of the input location
  const GraphTile* tile = nullptr;
  auto in_nodes = reader.GetDirectedEdgeNodes(in_edge, tile);
  uint32_t in_tz = 0;
  if (const auto* node = reader.nodeinfo(in_nodes.first, tile))
    in_tz = node->timezone();
  else if (const auto* node = reader.nodeinfo(in_nodes.second, tile))
    in_tz = node->timezone();

  // get the timezone of the output location
  auto out_nodes = reader.GetDirectedEdgeNodes(in_edge, tile);
  uint32_t out_tz = 0;
  if (const auto* node = reader.nodeinfo(out_nodes.first, tile))
    out_tz = node->timezone();
  else if (const auto* node = reader.nodeinfo(out_nodes.second, tile))
    out_tz = node->timezone();

  // offset the time
  uint64_t in_epoch = DateTime::seconds_since_epoch(in_dt, DateTime::get_tz_db().from_index(in_tz));
  double out_epoch = static_cast<double>(in_epoch) + offset;
  auto out_dt = DateTime::seconds_to_date(static_cast<uint64_t>(out_epoch + .5),
                                          DateTime::get_tz_db().from_index(out_tz));
  return out_dt;
}

} // namespace

namespace valhalla {
namespace thor {

std::string thor_worker_t::expansion(Api& request) {
  // default the expansion geojson so its easy to add to as we go
  rapidjson::Document dom;
  dom.SetObject();
  rapidjson::Pointer("/type").Set(dom, "FeatureCollection");
  rapidjson::Pointer("/properties/algorithm").Set(dom, "none");
  rapidjson::Pointer("/features/0/type").Set(dom, "Feature");
  rapidjson::Pointer("/features/0/geometry/type").Set(dom, "MultiLineString");
  rapidjson::Pointer("/features/0/geometry/coordinates").Create(dom).SetArray();
  rapidjson::Pointer("/features/0/properties/edge_ids").Create(dom).SetArray();
  rapidjson::Pointer("/features/0/properties/statuses").Create(dom).SetArray();

  // a lambda that the path algorithm can call to add stuff to the dom
  auto track_expansion = [&dom](baldr::GraphReader& reader, const char* algorithm,
                                baldr::GraphId edgeid, const char* status, bool full_shape = false) {
    // full shape might be overkill but meh, its trace
    const auto* tile = reader.GetGraphTile(edgeid);
    const auto* edge = tile->directededge(edgeid);
    auto shape = tile->edgeinfo(edge->edgeinfo_offset()).shape();
    if (!edge->forward())
      std::reverse(shape.begin(), shape.end());
    if (!full_shape && shape.size() > 2)
      shape.erase(shape.begin() + 1, shape.end() - 1);

    // make the geom
    auto& a = dom.GetAllocator();
    auto* coords = rapidjson::Pointer("/features/0/geometry/coordinates").Get(dom);
    coords->GetArray().PushBack(rapidjson::Value(rapidjson::kArrayType), a);
    auto& linestring = (*coords)[coords->Size() - 1];
    for (const auto& p : shape) {
      linestring.GetArray().PushBack(rapidjson::Value(rapidjson::kArrayType), a);
      auto point = linestring[linestring.Size() - 1].GetArray();
      point.PushBack(p.first, a);
      point.PushBack(p.second, a);
    }

    // make the properties
    rapidjson::Pointer("/properties/algorithm").Set(dom, algorithm);
    rapidjson::Pointer("/features/0/properties/edge_ids")
        .Get(dom)
        ->GetArray()
        .PushBack(static_cast<uint64_t>(edgeid), a);
    rapidjson::Pointer("/features/0/properties/statuses")
        .Get(dom)
        ->GetArray()
        .PushBack(rapidjson::Value{}.SetString(status, a), a);
  };

  // tell all the algorithms how to track expansion
  for (auto* alg : std::vector<PathAlgorithm*>{
           &multi_modal_astar,
           &timedep_forward,
           &timedep_reverse,
           &astar,
           &bidir_astar,
       }) {
    alg->set_track_expansion(track_expansion);
  }

  // track the expansion
  route(request);

  // tell all the algorithms to stop tracking the expansion
  for (auto* alg : std::vector<PathAlgorithm*>{
           &multi_modal_astar,
           &timedep_forward,
           &timedep_reverse,
           &astar,
           &bidir_astar,
       }) {
    alg->set_track_expansion(nullptr);
  }

  // serialize it
  return rapidjson::to_string(dom, 5);
}

void thor_worker_t::route(Api& request) {
  parse_locations(request);
  parse_filter_attributes(request);
  auto costing = parse_costing(request);
  auto& options = *request.mutable_options();

  // get all the legs
  if (options.has_date_time_type() && options.date_time_type() == Options::arrive_by)
    path_arrive_by(request, costing);
  else
    path_depart_at(request, costing);

  // log admin areas
  if (!options.do_not_track()) {
    for (const auto& route : request.trip().routes()) {
      for (const auto& leg : route.legs()) {
        log_admin(leg);
      }
    }
  }
}

thor::PathAlgorithm* thor_worker_t::get_path_algorithm(const std::string& routetype,
                                                       const valhalla::Location& origin,
                                                       const valhalla::Location& destination) {
  // Have to use multimodal for transit based routing
  if (routetype == "multimodal" || routetype == "transit") {
    multi_modal_astar.set_interrupt(interrupt);
    return &multi_modal_astar;
  }

  // If the origin has date_time set use timedep_forward method if the distance
  // between location is below some maximum distance (TBD).
  if (origin.has_date_time()) {
    PointLL ll1(origin.ll().lng(), origin.ll().lat());
    PointLL ll2(destination.ll().lng(), destination.ll().lat());
    if (ll1.Distance(ll2) < max_timedep_distance) {
      timedep_forward.set_interrupt(interrupt);
      return &timedep_forward;
    }
  }

  // If the destination has date_time set use timedep_reverse method if the distance
  // between location is below some maximum distance (TBD).
  if (destination.has_date_time()) {
    PointLL ll1(origin.ll().lng(), origin.ll().lat());
    PointLL ll2(destination.ll().lng(), destination.ll().lat());
    if (ll1.Distance(ll2) < max_timedep_distance) {
      timedep_reverse.set_interrupt(interrupt);
      return &timedep_reverse;
    }
  }

  // Use A* if any origin and destination edges are the same or are connected - otherwise
  // use bidirectional A*. Bidirectional A* does not handle trivial cases with oneways and
  // has issues when cost of origin or destination edge is high (needs a high threshold to
  // find the proper connection).
  for (auto& edge1 : origin.path_edges()) {
    for (auto& edge2 : destination.path_edges()) {
      if (edge1.graph_id() == edge2.graph_id() ||
          reader->AreEdgesConnected(GraphId(edge1.graph_id()), GraphId(edge2.graph_id()))) {
        astar.set_interrupt(interrupt);
        return &astar;
      }
    }
  }
  bidir_astar.set_interrupt(interrupt);
  return &bidir_astar;
}

std::vector<std::vector<thor::PathInfo>> thor_worker_t::get_path(PathAlgorithm* path_algorithm,
                                                                 valhalla::Location& origin,
                                                                 valhalla::Location& destination,
                                                                 const std::string& costing,
                                                                 const Options& options) {
  // Find the path. If bidirectional A* disable use of destination only edges on the
  // first pass. If there is a failure, we allow them on the second pass.
  valhalla::sif::cost_ptr_t cost = mode_costing[static_cast<uint32_t>(mode)];
  if (path_algorithm == &bidir_astar) {
    cost->set_allow_destination_only(false);
  }
  cost->set_pass(0);
  auto paths = path_algorithm->GetBestPath(origin, destination, *reader, mode_costing, mode, options);

  // Check if we should run a second pass pedestrian route with different A*
  // (to look for better routes where a ferry is taken)
  bool ped_second_pass = false;
  if (!paths.empty() && (costing == "pedestrian" && path_algorithm->has_ferry())) {
    // DO NOT run a second pass on long routes due to performance issues
    float d = PointLL(origin.ll().lng(), origin.ll().lat())
                  .Distance(PointLL(destination.ll().lng(), destination.ll().lat()));
    if (d < kPedestrianMultipassThreshold) {
      ped_second_pass = true;
    }
  }

  // If path is not found try again with relaxed limits (if allowed). Use less aggressive
  // hierarchy transition limits, and retry with more candidate edges (add those filtered
  // by heading on first pass).
  if ((paths.empty() || ped_second_pass) && cost->AllowMultiPass()) {
    // add filtered edges to candidate edges for origin and destination
    origin.mutable_path_edges()->MergeFrom(origin.filtered_edges());
    destination.mutable_path_edges()->MergeFrom(destination.filtered_edges());

    path_algorithm->Clear();
    cost->set_pass(1);
    bool using_astar = (path_algorithm == &astar);
    float relax_factor = using_astar ? 16.0f : 8.0f;
    float expansion_within_factor = using_astar ? 4.0f : 2.0f;
    cost->RelaxHierarchyLimits(relax_factor, expansion_within_factor);
    cost->set_allow_destination_only(true);

    // Get the best path. Return if not empty (else return the original path)
    auto relaxed_paths =
        path_algorithm->GetBestPath(origin, destination, *reader, mode_costing, mode, options);
    if (!relaxed_paths.empty()) {
      return relaxed_paths;
    }
  }

  // All or nothing
  if (paths.empty()) {
    throw valhalla_exception_t{442};
  }
  return paths;
}

void thor_worker_t::path_arrive_by(Api& api, const std::string& costing) {
  // Things we'll need
  TripRoute* route = nullptr;
  GraphId first_edge;
  std::unordered_map<size_t, std::pair<RouteDiscontinuity, RouteDiscontinuity>> vias;
  std::vector<thor::PathInfo> path;
  auto& correlated = *api.mutable_options()->mutable_locations();

  // For each pair of locations
  for (auto origin = ++correlated.rbegin(); origin != correlated.rend(); ++origin) {
    // Get the algorithm type for this location pair
    auto destination = std::prev(origin);
    thor::PathAlgorithm* path_algorithm = get_path_algorithm(costing, *origin, *destination);
    path_algorithm->Clear();

    // TODO: delete this and send all cases to the function above
    // If we are continuing through a location we need to make sure we
    // only allow the edge that was used previously (avoid u-turns)
    bool through = destination->type() == valhalla::Location::kThrough ||
                   destination->type() == valhalla::Location::kBreakThrough;
    while (through && first_edge.Is_Valid() && destination->path_edges_size() > 1) {
      if (destination->path_edges().rbegin()->graph_id() == first_edge) {
        destination->mutable_path_edges()->SwapElements(0, destination->path_edges_size() - 1);
      }
      destination->mutable_path_edges()->RemoveLast();
    }

    // Get best path and keep it
    auto temp_paths = get_path(path_algorithm, *origin, *destination, costing, api.options());
    for (auto& temp_path : temp_paths) {
      // back propagate time information
      if (destination->has_date_time()) {
        auto origin_dt = offset_date(*reader, destination->date_time(), temp_path.back().edgeid,
                                     -temp_path.back().elapsed_time, temp_path.front().edgeid);
        origin->set_date_time(origin_dt);
      }

      first_edge = temp_path.front().edgeid;
      temp_path.swap(path); // so we can append to path instead of prepend

      // Merge through legs by updating the time and splicing the lists
      if (!temp_path.empty()) {
        auto offset = path.back().elapsed_time;
        std::for_each(temp_path.begin(), temp_path.end(),
                      [offset](PathInfo& i) { i.elapsed_time += offset; });
        // Connects via the same edge so we only need it once
        if (path.back().edgeid == temp_path.front().edgeid) {
          path.pop_back();
        } else if (destination->type() == valhalla::Location::kVia) {
          // Insert a route discontinuity if the paths meet at opposing edges and not
          // at a graph node. Use path size - 1 as the index where the discontinuity lies.
          via_discontinuity(*reader, *destination, path.back().edgeid, temp_path.front().edgeid, vias,
                            temp_path.size(), true);
        }
        path.insert(path.end(), temp_path.begin(), temp_path.end());
      }

      // Build trip path for this leg and add to the result if this
      // location is a BREAK or if this is the last location
      if (origin->type() == valhalla::Location::kBreak ||
          origin->type() == valhalla::Location::kBreakThrough) {
        // Move destination back to the last break and collect the throughs
        std::list<valhalla::Location> throughs;
        while (destination->type() != valhalla::Location::kBreak &&
               destination->type() != valhalla::Location::kBreakThrough) {
          throughs.push_back(*destination);
          --destination;
        }

        // We have to flip the via indices because we built them in backwards order
        decltype(vias) flipped;
        flipped.reserve(vias.size());
        for (const auto& kv : vias)
          flipped.emplace(path.size() - kv.first, kv.second);
        vias.swap(flipped);

        // Form output information based on path edges
        if (api.trip().routes_size() == 0 || api.options().alternates() > 0)
          route = api.mutable_trip()->mutable_routes()->Add();
        auto& leg = *route->mutable_legs()->Add();
        TripLegBuilder::Build(controller, *reader, mode_costing, path.begin(), path.end(), *origin,
                              *destination, throughs, leg, interrupt, &vias);
        path.clear();
        vias.clear();
      }
    }
  }

  // Reverse the legs because protobuf only has adding to the end
  std::reverse(route->mutable_legs()->begin(), route->mutable_legs()->end());
}

void thor_worker_t::path_depart_at(Api& api, const std::string& costing) {
  // Things we'll need
  GraphId last_edge;
  TripRoute* route = nullptr;
  std::unordered_map<size_t, std::pair<RouteDiscontinuity, RouteDiscontinuity>> vias;
  std::vector<thor::PathInfo> path;
  std::list<valhalla::TripLeg> trip_paths;
  auto& correlated = *api.mutable_options()->mutable_locations();

  // For each pair of locations
  for (auto destination = ++correlated.begin(); destination != correlated.end(); ++destination) {
    // Get the algorithm type for this location pair
    auto origin = std::prev(destination);
    thor::PathAlgorithm* path_algorithm = get_path_algorithm(costing, *origin, *destination);
    path_algorithm->Clear();

    // TODO: delete this and send all cases to the function above
    // If we are continuing through a location we need to make sure we
    // only allow the edge that was used previously (avoid u-turns)
    bool through = origin->type() == valhalla::Location::kThrough ||
                   origin->type() == valhalla::Location::kBreakThrough;
    while (through && last_edge.Is_Valid() && origin->path_edges_size() > 1) {
      if (origin->path_edges().rbegin()->graph_id() == last_edge) {
        origin->mutable_path_edges()->SwapElements(0, origin->path_edges_size() - 1);
      }
      origin->mutable_path_edges()->RemoveLast();
    }

    // Get best path and keep it
    auto temp_paths = get_path(path_algorithm, *origin, *destination, costing, api.options());
    for (auto& temp_path : temp_paths) {
      // forward propagate time information
      if (origin->has_date_time()) {
        auto destination_dt = offset_date(*reader, origin->date_time(), temp_path.front().edgeid,
                                          temp_path.back().elapsed_time, temp_path.back().edgeid);
        destination->set_date_time(destination_dt);
      }

      last_edge = temp_path.back().edgeid;

      // Merge through legs by updating the time and splicing the lists
      if (!path.empty()) {
        auto offset = path.back().elapsed_time;
        std::for_each(temp_path.begin(), temp_path.end(),
                      [offset](PathInfo& i) { i.elapsed_time += offset; });
        // Connects via the same edge so we only need it once
        if (path.back().edgeid == temp_path.front().edgeid) {
          path.pop_back();
        } else if (origin->type() == valhalla::Location::kVia) {
          // Insert a route discontinuity if the paths meet at opposing edges and not
          // at a graph node. Use path size - 1 as the index where the discontinuity lies.
          via_discontinuity(*reader, *origin, path.back().edgeid, temp_path.front().edgeid, vias,
                            path.size() - 1, false);
        }
        path.insert(path.end(), temp_path.begin(), temp_path.end());
      } // Didnt need to merge
      else {
        path.swap(temp_path);
      }

      // Build trip path for this leg and add to the result if this
      // location is a BREAK or if this is the last location
      if (destination->type() == valhalla::Location::kBreak ||
          destination->type() == valhalla::Location::kBreakThrough) {
        // Move origin back to the last break and collect the throughs
        std::list<valhalla::Location> throughs;
        while (origin->type() != valhalla::Location::kBreak &&
               origin->type() != valhalla::Location::kBreakThrough) {
          throughs.push_front(*origin);
          --origin;
        }

        // Form output information based on path edges. vias are a route discontinuity map
        if (api.trip().routes_size() == 0 || api.options().alternates() > 0)
          route = api.mutable_trip()->mutable_routes()->Add();
        auto& leg = *route->mutable_legs()->Add();
        thor::TripLegBuilder::Build(controller, *reader, mode_costing, path.begin(), path.end(),
                                    *origin, *destination, throughs, leg, interrupt, &vias);
        path.clear();
        vias.clear();
      }
    }
  }
}

} // namespace thor
} // namespace valhalla
