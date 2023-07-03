#include "thor/route_matcher.h"
#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/tilehierarchy.h"
#include "baldr/time_info.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "proto_conversions.h"

#include <algorithm>
#include <exception>
#include <vector>

using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

/*
 * An “edge-walking” method for use when the input shape is exact
 * shape from a prior Valhalla route. This will walk the input shape
 * and compare to Valhalla edge’s end node positions to form the list of edges.
 *
 */

namespace {

using end_edge_t = std::pair<valhalla::PathEdge, float>;
using end_node_t = std::unordered_map<GraphId, end_edge_t>;

// Data type to record edges and transitions that have been followed for each shape
// index and hierarchy level. First entry in the pair is the index (from the node's
// starting directed edge index) of the directed edge. The second entry in the pair
// is the transition index.
using followed_edges_t = std::vector<std::vector<std::pair<uint32_t, uint32_t>>>;

// Total distance match delta
constexpr float kTotalDistanceEpsilon = 5.0f;

// Minimum tolerance for edge length
constexpr float kMinLengthTolerance = 10.0f;

// Get the length to compare to the edge length
float length_comparison(const float length, const bool exact_match) {
  // Alter tolerance based on exact_match flag
  float t, max_t;
  if (exact_match) {
    t = length * 0.05f;
    max_t = 25.0f;
  } else {
    t = length * 0.1f;
    max_t = 100.0f;
  }
  float tolerance = (t < kMinLengthTolerance) ? kMinLengthTolerance : (t > max_t) ? max_t : t;
  return length + tolerance;
}

// TODO: we need to stop relying on loki::Search to pre populate edge candidates for the first and
// last locations. Instead we need to do that here where we already know the edges in question and can
// make a single path edge for the edge we are interested in. Its the only way to get multi-leg
//
// Get a map of end edges and the start node
// of each edge. This is used to terminate the edge walking method.
end_node_t GetEndEdges(GraphReader& reader, const valhalla::Location& destination) {
  end_node_t end_nodes;
  for (const auto& edge : destination.correlation().edges()) {
    // If destination is at a node - skip any outbound edge
    GraphId graphid(edge.graph_id());
    if (edge.begin_node() || !graphid.Is_Valid()) {
      continue;
    }

    // Add the node which we look for to terminate edge walking. This is
    // generally the start of the end edge, unless the end edge ends at
    // a node (not partially along the edge)
    if (edge.end_node()) {
      // If this edge ends at a node add its end node
      auto tile = reader.GetGraphTile(graphid);
      auto* directededge = tile->directededge(graphid);
      end_nodes.insert({directededge->endnode(), std::make_pair(edge, 0.0f)});
    } else {
      // Get the start node of this edge
      GraphId opp_edge_id = reader.GetOpposingEdgeId(graphid);
      auto tile = reader.GetGraphTile(opp_edge_id);
      if (tile == nullptr) {
        throw std::runtime_error("Couldn't get the opposing edge tile");
      }
      auto* opp_edge = tile->directededge(opp_edge_id);

      // Compute partial distance along end edge
      float dist = opp_edge->length() * edge.percent_along();
      end_nodes.insert({opp_edge->endnode(), std::make_pair(edge, dist)});
    }
  }
  if (end_nodes.size() == 0) {
    throw std::runtime_error("No valid end edges are found");
  }
  return end_nodes;
}

// Expand from a correlated node. Walks the shape ahead to find the next correlated
// node and expands from there. Returns true once the end node has been found (and
// distance is approximately what it should be). Returns false if expansion from this
// node fails (cannot find edges that match the trace - either in position or distance).
bool expand_from_node(const mode_costing_t& mode_costing,
                      const TravelMode& mode,
                      GraphReader& reader,
                      const google::protobuf::RepeatedPtrField<valhalla::Location>& shape,
                      std::vector<std::pair<float, float>>& distances,
                      const valhalla::baldr::TimeInfo& time_info,
                      const bool use_timestamps,
                      size_t& correlated_index,
                      const graph_tile_ptr& tile,
                      const GraphId& node,
                      end_node_t& end_nodes,
                      EdgeLabel& prev_edge_label,
                      Cost& elapsed,
                      std::vector<PathInfo>& path_infos,
                      const bool from_transition,
                      GraphId& end_node,
                      followed_edges_t& followed_edges) {
  // Done expanding when node equals stop node and the accumulated distance to that node
  // plus the partial last edge distance is approximately equal to the total distance
  auto n = end_nodes.find(node);
  if (n != end_nodes.end() &&
      valhalla::midgard::equal<float>((distances[correlated_index].second + n->second.second),
                                      distances.back().second, kTotalDistanceEpsilon)) {
    end_node = node;
    return true;
  }

  // Get the last edge followed from this index
  uint32_t level = node.level();
  uint32_t start_de = followed_edges[correlated_index][level].first;

  // Iterate through directed edges from this node
  const NodeInfo* nodeinfo = tile->node(node);
  GraphId edge_id(node.tileid(), level, nodeinfo->edge_index());
  const DirectedEdge* de = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = start_de; i < nodeinfo->edge_count(); i++, de++, ++edge_id) {
    // Mark the directed edge as already followed
    followed_edges[correlated_index][level].first = i;

    // Skip shortcuts and transit connection edges
    // TODO - later might allow transit connections for multi-modal
    if (de->is_shortcut() || de->use() == Use::kTransitConnection) {
      continue;
    }

    // Look back in path_infos by 1-2 edges to make sure we aren't in a loop.
    // A loop can occur if we have edges shorter than the lat,lng tolerance.
    uint32_t n = path_infos.size();
    if (n > 1 && (edge_id == path_infos[n - 2].edgeid || edge_id == path_infos[n - 1].edgeid)) {
      continue;
    }

    // Get the end node LL and set up the length comparison
    graph_tile_ptr end_node_tile = reader.GetGraphTile(de->endnode());
    if (end_node_tile == nullptr) {
      continue;
    }
    valhalla::midgard::PointLL de_end_ll = end_node_tile->get_node_ll(de->endnode());
    float de_length = length_comparison(de->length(), true);

    // Process current edge until shape matches end node or shape length is longer than
    // the current edge. Increment to the next shape point after the correlated index.
    size_t index = correlated_index + 1;
    float length = 0.0f;
    while (index < shape.size()) {
      // Exclude edge if length along shape is longer than the edge length
      length += distances.at(index).first;
      if (length > de_length) {
        break;
      }

      // Found a match if shape equals directed edge LL within tolerance
      if (to_ll(shape.Get(index).ll()).ApproximatelyEqual(de_end_ll) &&
          de->length() < length_comparison(length, true)) {

        // Figure out what time it is right now, the first iteration is a no-op
        auto offset_time_info = nodeinfo
                                    ? time_info.forward(/*accumulated_elapsed.secs + */ elapsed.secs,
                                                        nodeinfo->timezone())
                                    : time_info;

        // get the cost of traversing the node and the edge
        auto& costing = mode_costing[static_cast<int>(mode)];
        auto transition_cost = costing->TransitionCost(de, nodeinfo, prev_edge_label);
        uint8_t flow_sources;
        auto cost =
            transition_cost + costing->EdgeCost(de, end_node_tile, offset_time_info, flow_sources);
        elapsed += cost;
        // overwrite time with timestamps
        if (use_timestamps)
          elapsed.secs = shape.Get(index).time() - shape.Get(0).time();

        // Add edge and update correlated index
        path_infos.emplace_back(mode, elapsed, edge_id, 0, 0.f, -1, transition_cost);

        InternalTurn turn = nodeinfo
                                ? costing->TurnType(prev_edge_label.opp_local_idx(), nodeinfo, de)
                                : InternalTurn::kNoTurn;
        // Set previous edge label
        prev_edge_label = {kInvalidLabel,
                           edge_id,
                           de,
                           {},
                           0,
                           0,
                           mode,
                           0,
                           {},
                           kInvalidRestriction,
                           true,
                           static_cast<bool>(flow_sources & kDefaultFlowMask),
                           turn};

        // Continue walking shape to find the end edge...
        if (expand_from_node(mode_costing, mode, reader, shape, distances, time_info, use_timestamps,
                             index, end_node_tile, de->endnode(), end_nodes, prev_edge_label, elapsed,
                             path_infos, false, end_node, followed_edges)) {
          return true;
        } else {
          // Match failed along this edge, pop the last entry off path_infos as well as what it
          // contributed to the elapsed cost/time and try to keep going on the next edge
          elapsed -= cost;
          path_infos.pop_back();
          break;
        }
      }
      index++;
    }
  }

  // Get the last transition followed from this index
  uint32_t start_trans = followed_edges[correlated_index][level].second;

  // Handle transitions - expand from the transition end nodes
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = start_trans; i < nodeinfo->transition_count(); ++i, ++trans) {
      followed_edges[correlated_index][level].second = i;
      graph_tile_ptr end_node_tile = reader.GetGraphTile(trans->endnode());
      if (end_node_tile == nullptr) {
        continue;
      }
      if (expand_from_node(mode_costing, mode, reader, shape, distances, time_info, use_timestamps,
                           correlated_index, end_node_tile, trans->endnode(), end_nodes,
                           prev_edge_label, elapsed, path_infos, true, end_node, followed_edges)) {
        return true;
      }
    }
  }
  return false;
}

valhalla::baldr::TimeInfo init_time_info(valhalla::baldr::GraphReader& reader,
                                         valhalla::Options& options,
                                         valhalla::baldr::DateTime::tz_sys_info_cache_t* tz_cache) {
  graph_tile_ptr tile = nullptr;
  const DirectedEdge* directededge = nullptr;
  const NodeInfo* nodeinfo = nullptr;

  // We support either the epoch timestamp that came with the trace point or
  // a local date time which we convert to epoch by finding the first timezone
  auto time_info = TimeInfo::invalid();
  for (const auto& e : options.locations(0).correlation().edges()) {
    GraphId graphid(e.graph_id());
    if (!graphid.Is_Valid() || !reader.GetGraphTile(graphid, tile))
      continue;
    directededge = tile->directededge(graphid);
    if (reader.GetGraphTile(directededge->endnode(), tile)) {
      // get the timezone
      nodeinfo = tile->node(directededge->endnode());
      const auto* tz = DateTime::get_tz_db().from_index(nodeinfo->timezone());
      if (!tz)
        continue;
      // if its timestamp based we need to convert that to a date time string on the location
      if (options.shape(0).date_time().empty() && options.shape(0).time() != -1.0) {
        options.mutable_shape(0)->set_date_time(
            DateTime::seconds_to_date(options.shape(0).time(), tz, false));
      }
      return TimeInfo::make(*options.mutable_shape(0), reader, tz_cache, nodeinfo->timezone());
    }
  }
  return TimeInfo::invalid();
}

} // namespace

namespace valhalla {
namespace thor {

// For the route path using an edge walking method.
bool RouteMatcher::FormPath(const sif::mode_costing_t& mode_costing,
                            const sif::TravelMode& mode,
                            baldr::GraphReader& reader,
                            valhalla::Options& options,
                            std::vector<std::vector<PathInfo>>& legs) {
  // TODO: build more than one leg based on location types
  legs.clear();
  legs.emplace_back();
  auto& path_infos = legs.back();

  if (options.shape_size() < 2) {
    throw std::runtime_error("Invalid shape - less than 2 points");
  }

  // Form distances between shape points and accumulated distance from start to each shape point
  float total_distance = 0.0f;
  std::vector<std::pair<float, float>> distances;
  distances.push_back(std::make_pair(0.0f, 0.0f));
  for (size_t i = 1; i < options.shape_size(); i++) {
    float d = to_ll(options.shape(i).ll()).Distance(to_ll(options.shape(i - 1).ll()));
    total_distance += d;
    distances.push_back(std::make_pair(d, total_distance));
  }

  // Keep a record of followed edges and transition from each shape index (for each hierarchy level) -
  // this prevents doubling back and causing an infinite loop (could be due to transitions)
  followed_edges_t followed_edges;
  followed_edges.resize(options.shape_size());
  for (auto& f : followed_edges) {
    f.resize(TileHierarchy::get_max_level());
  }

  // Process and validate end edges (can be more than 1). Create a map of
  // the end edges' start nodes and the edge information.
  // TODO: when we want to do more than one leg we need to create correlated path_edges on the fly
  const auto& destination = *options.locations().rbegin();
  auto end_nodes = GetEndEdges(reader, destination);

  // We support either the epoch timestamp that came with the trace point or
  // a local date time which we convert to epoch by finding the first timezone
  valhalla::baldr::DateTime::tz_sys_info_cache_t tz_cache;
  auto time_info = init_time_info(reader, options, &tz_cache);

  // Perform the edge walk by starting with one of the candidate edges and walking from it
  // if that walk fails we fall back to another candidate edge until we exhaust the candidates
  for (const auto& edge : options.locations().begin()->correlation().edges()) {
    // If origin is at a node - skip any inbound edge
    if (edge.end_node()) {
      continue;
    }

    // Process and validate begin edge
    GraphId graphid(edge.graph_id());
    if (!graphid.Is_Valid()) {
      throw std::runtime_error("Invalid begin edge id");
    }
    auto begin_edge_tile = reader.GetGraphTile(graphid);
    if (begin_edge_tile == nullptr) {
      throw std::runtime_error("Begin tile is null");
    }

    // Process directed edge and info
    const DirectedEdge* de = begin_edge_tile->directededge(graphid);
    auto end_node_tile = reader.GetGraphTile(de->endnode());
    if (end_node_tile == nullptr) {
      throw std::runtime_error("End node tile is null");
    }
    midgard::PointLL de_end_ll = end_node_tile->get_node_ll(de->endnode());

    // Initialize indexes and shape
    size_t index = 0;
    float length = 0.0f;
    float de_remaining_length = de->length() * (1 - edge.percent_along());
    float de_length = length_comparison(de_remaining_length, true);
    EdgeLabel prev_edge_label;
    Cost elapsed;
    // Cost accumulated_elapsed{}; // TODO: use this once we have more than one leg
    const NodeInfo* nodeinfo = nullptr;

    // Loop over shape to form path from matching edges
    while (index < options.shape_size()) {

      // bail on this edge if the length of input we checked is already longer than the edge
      length += distances.at(index).first;
      if (length > de_length) {
        break;
      }

      // Check if shape is within tolerance at the end node
      if (to_ll(options.shape(index).ll()).ApproximatelyEqual(de_end_ll) &&
          de_remaining_length < length_comparison(length, true)) {

        // Figure out what time it is right now, the first iteration is a no-op
        auto offset_time_info = nodeinfo
                                    ? time_info.forward(/*accumulated_elapsed.secs + */ elapsed.secs,
                                                        nodeinfo->timezone())
                                    : time_info;

        // Get the cost of traversing the edge
        uint8_t flow_sources;
        elapsed += mode_costing[static_cast<int>(mode)]->EdgeCost(de, end_node_tile, offset_time_info,
                                                                  flow_sources) *
                   (1 - edge.percent_along());
        // overwrite time with timestamps
        if (options.use_timestamps())
          elapsed.secs = options.shape(index).time() - options.shape(0).time();

        // Add begin edge
        path_infos.emplace_back(mode, elapsed, graphid, 0, 0.f, -1);

        InternalTurn turn =
            nodeinfo ? mode_costing[static_cast<int>(mode)]->TurnType(prev_edge_label.opp_local_idx(),
                                                                      nodeinfo, de)
                     : InternalTurn::kNoTurn;
        // Set previous edge label
        prev_edge_label = {kInvalidLabel,
                           graphid,
                           de,
                           {},
                           0,
                           0,
                           mode,
                           0,
                           {},
                           baldr::kInvalidRestriction,
                           true,
                           static_cast<bool>(flow_sources & kDefaultFlowMask),
                           turn};

        // Continue walking shape to find the end node
        GraphId end_node;
        if (expand_from_node(mode_costing, mode, reader, options.shape(), distances, time_info,
                             options.use_timestamps(), index, end_node_tile, de->endnode(), end_nodes,
                             prev_edge_label, elapsed, path_infos, false, end_node, followed_edges)) {
          // Find the edge we stopped on at the destination, if we didnt find it the greedy algorithm
          // hit a local maximum (made the wrong choice), TODO: we could rollback and try more
          auto n = end_nodes.find(end_node);
          if (n == end_nodes.end()) {
            return false;
          }

          // When the route ends at a node in the graph we have an ambiguous case. Multiple
          // destination edge candidates could have ended at this node but we use an unordered_map
          // instead of a multimap, which means when we go to insert the other candidates that end
          // there, they dont get inserted, only the first one does. This is all we really need for
          // finding the path but it means that once we do find the path to that node, the edge that
          // was in the value portion of the map entry might be the wrong edge. So here we need to
          // go find the edge candidate that was actually used in the path
          auto found_edge = destination.correlation().edges().end();
          if (n->second.first.end_node()) {
            found_edge = std::find_if(destination.correlation().edges().begin(),
                                      destination.correlation().edges().end(),
                                      [&path_infos](const auto& e) -> bool {
                                        return e.graph_id() == path_infos.back().edgeid;
                                      });
            if (found_edge == destination.correlation().edges().end()) {
              throw std::logic_error("Could not find destination candidate in shape-walked path");
            }
          }

          // TODO: when we actually have more than one leg, we have to do when we break legs
          // Store the matching edge candidates in the shapes locations
          const auto& end_edge =
              found_edge == destination.correlation().edges().end() ? n->second.first : *found_edge;
          options.mutable_shape(0)->mutable_correlation()->mutable_edges()->Add()->CopyFrom(edge);
          options.mutable_shape()->rbegin()->mutable_correlation()->mutable_edges()->Add()->CopyFrom(
              end_edge);

          // If the end edge is at a node then we are done (no partial time along a destination edge)
          if (end_edge.end_node()) {
            return true;
          }

          // Get the end edge and add transition time and partial time along
          // the destination edge.
          GraphId end_edge_graphid(end_edge.graph_id());
          graph_tile_ptr end_edge_tile = reader.GetGraphTile(end_edge_graphid);
          if (end_edge_tile == nullptr) {
            throw std::runtime_error("End edge tile is null");
          }
          const DirectedEdge* end_de = end_edge_tile->directededge(end_edge_graphid);

          // get the cost of traversing the node and the remaining part of the edge
          auto& costing = mode_costing[static_cast<int>(mode)];
          nodeinfo = end_edge_tile->node(n->first);
          auto transition_cost = costing->TransitionCost(end_de, nodeinfo, prev_edge_label);
          uint8_t flow_sources;
          elapsed += transition_cost +
                     costing->EdgeCost(end_de, end_edge_tile, offset_time_info, flow_sources) *
                         end_edge.percent_along();
          // overwrite time with timestamps
          if (options.use_timestamps())
            elapsed.secs = options.shape().rbegin()->time() - options.shape(0).time();

          // Add end edge
          path_infos.emplace_back(mode, elapsed, end_edge_graphid, 0, 0.f, -1, transition_cost);
          return true;
        } else {
          // Did not find an edge that correlates with the trace, return false.
          return false;
        }
      }
      index++;
    }

    // Did not find the end of the origin edge. Check for trivial route on a single edge
    for (const auto& end : end_nodes) {
      if (end.second.first.graph_id() == edge.graph_id()) {
        // Update the elapsed time based on edge cost
        uint8_t flow_sources;
        elapsed += mode_costing[static_cast<int>(mode)]->EdgeCost(de, end_node_tile, time_info,
                                                                  flow_sources) *
                   (end.second.first.percent_along() - edge.percent_along());
        if (options.use_timestamps())
          elapsed.secs = options.shape().rbegin()->time() - options.shape(0).time();

        // Add end edge
        path_infos.emplace_back(mode, elapsed, GraphId(edge.graph_id()), 0, 0.f, -1);
        return true;
      }
    }
  }
  // TODO - would be nice to know this, but if map-matching fallback is specified
  // this would not fall back.
  //  throw std::runtime_error("RouteMatcher::FormPath could not match to begin edge");
  return false;
}

} // namespace thor
} // namespace valhalla
