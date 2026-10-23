#include "mjolnir/elevationbuilder.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "midgard/elevation_encoding.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/util.h"
#include "scoped_timer.h"
#include "skadi/sample.h"
#include "skadi/util.h"

#include <ankerl/unordered_dense.h>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <filesystem>
#include <limits>
#include <optional>
#include <queue>
#include <random>
#include <thread>
#include <utility>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

// How many meters to resample shape to when checking elevations.
constexpr double POSTING_INTERVAL = 60;

// Do not compute grade for intervals less than 10 meters.
constexpr double kMinimumInterval = 10.0f;

// Structures with more nodes than this keep their ground samples, e.g. the elevated roads of
// Shanghai (1.9k nodes) or Tokyo (1.3k).
constexpr size_t kMaxStructureNodes = 250;

using cache_t =
    std::unordered_map<uint32_t, std::tuple<uint32_t, uint32_t, float, float, float, float>>;

/**
 * Encode elevation along an edge to store in tiles.
 */
std::vector<int8_t> encode_edge_elevation(const std::unique_ptr<valhalla::skadi::sample>& sample,
                                          const std::vector<PointLL>& shape,
                                          const uint32_t length,
                                          uint32_t wayid) {
  // Uniformly resample the polyline to create the desired number of vertices
  uint32_t n = encoded_elevation_count(length) + 2;
  std::vector<PointLL> resampled =
      valhalla::midgard::uniform_resample_spherical_polyline(shape, length, n);

  // Get elevation (height) at each sampled point along the edge.
  std::vector<double> heights(resampled.size());
  heights = sample->get_all(resampled);

  // Encode the elevation.
  bool error = false;
  std::vector<int8_t> encoded = encode_elevation(heights, error);
  if (error) {
    double diff = 0;
    for (size_t i = 1; i < heights.size(); i++) {
      auto d = std::abs(heights[i] - heights[i - 1]);
      diff = d < diff ? diff : d;
      LOG_DEBUG("  " + std::to_string(heights[i]));
    }
    LOG_DEBUG("edge elevation wayid = " + std::to_string(wayid) + " exceeds difference with " +
              std::to_string(diff) + " meters.");
    build_stats::get().increment(build_stats::kExceededElevationDiff);
  }
  return encoded;
}

// Bridges, tunnels and ferries get no elevation of their own; it is interpolated between nodes.
bool is_structure(const DirectedEdge& edge) {
  return edge.bridge() || edge.tunnel() || edge.use() == Use::kFerry || edge.use() == Use::kRailFerry;
}

// The smallest id among the node and its copies on other hierarchy levels keys it on every level.
GraphId node_key(const GraphTile& tile, const NodeInfo& node, GraphId id) {
  for (uint32_t i = 0; i < node.transition_count(); ++i)
    id = std::min(id, tile.transition(node.transition_index() + i)->endnode());
  return id;
}

// A structure in memory: its enclosed nodes, the portals around them and the edges between, every
// node keyed by node_key.
struct Structure {
  struct Node {
    GraphId id;
    bool portal = false;
    double height = 0;
    std::vector<std::pair<uint32_t, uint32_t>> edges;
  };
  std::vector<Node> nodes;
  bool oversized = false;
};

// Reads the structure around an enclosed node breadth first, stopping at portals.
Structure read_structure(GraphReader& graphreader,
                         const GraphId& seed,
                         const std::unique_ptr<valhalla::skadi::sample>& sample) {
  Structure s;
  ankerl::unordered_dense::map<GraphId, uint32_t> index;
  auto add = [&](const GraphId& id) {
    auto [it, inserted] = index.emplace(id, static_cast<uint32_t>(s.nodes.size()));
    if (inserted) {
      s.nodes.push_back({id});
    }
    return it->second;
  };
  add(seed);

  for (uint32_t u = 0; u < s.nodes.size(); ++u) {
    if (u == kMaxStructureNodes) {
      s.oversized = true;
      return s;
    }

    bool portal = false;
    std::vector<std::pair<GraphId, uint32_t>> next;
    auto scan = [&](const graph_tile_ptr& tile, const NodeInfo* node) {
      for (uint32_t i = 0; i < node->edge_count() && !portal; ++i) {
        const DirectedEdge* edge = tile->directededge(node->edge_index() + i);
        if (edge->is_shortcut())
          continue;
        if (!is_structure(*edge)) {
          portal = true;
          return;
        }
        next.emplace_back(edge->endnode(), std::max(edge->length(), 1u));
      }
    };

    const GraphId id = s.nodes[u].id;
    auto tile = graphreader.GetGraphTile(id);
    const NodeInfo* node = tile->node(id);
    scan(tile, node);
    for (uint32_t i = 0; i < node->transition_count(); ++i) {
      const GraphId copy = tile->transition(node->transition_index() + i)->endnode();
      auto copy_tile = graphreader.GetGraphTile(copy);
      scan(copy_tile, copy_tile->node(copy));
    }

    if (portal) {
      s.nodes[u].portal = true;
      s.nodes[u].height = sample->get(tile->get_node_ll(id));
      continue;
    }

    for (const auto& [end, length] : next) {
      auto end_tile = graphreader.GetGraphTile(end);
      const uint32_t v = add(node_key(*end_tile, *end_tile->node(end), end));
      s.nodes[u].edges.emplace_back(v, length);
      s.nodes[v].edges.emplace_back(u, length);
    }
  }
  return s;
}

// Blends portal heights into enclosed nodes by inverse graph distance, which is plain linear
// interpolation for a structure with two portals.
void interpolate(const Structure& s,
                 ankerl::unordered_dense::map<GraphId, std::optional<double>>& heights) {
  std::vector<double> weights(s.nodes.size(), 0);
  std::vector<double> weighted(s.nodes.size(), 0);
  std::vector<uint64_t> distance(s.nodes.size());
  using label_t = std::pair<uint64_t, uint32_t>;

  // Expand from each portal and accumulate its impact on other nodes.
  for (uint32_t p = 0; p < s.nodes.size(); ++p) {
    if (!s.nodes[p].portal || s.nodes[p].height == valhalla::skadi::get_no_data_value()) {
      continue;
    }

    std::fill(distance.begin(), distance.end(), std::numeric_limits<uint64_t>::max());
    std::priority_queue<label_t, std::vector<label_t>, std::greater<>> queue;
    distance[p] = 0;
    queue.emplace(0, p);
    while (!queue.empty()) {
      const auto [d, u] = queue.top();
      queue.pop();
      if (d > distance[u] || (u != p && s.nodes[u].portal)) {
        continue;
      }

      if (u != p) {
        weighted[u] += s.nodes[p].height / d;
        weights[u] += 1. / d;
      }
      for (const auto& [v, length] : s.nodes[u].edges) {
        if (d + length < distance[v]) {
          distance[v] = d + length;
          queue.emplace(distance[v], v);
        }
      }
    }
  }

  for (uint32_t u = 0; u < s.nodes.size(); ++u) {
    if (!s.nodes[u].portal) {
      heights[s.nodes[u].id] =
          weights[u] > 0 ? std::optional<double>(weighted[u] / weights[u]) : std::nullopt;
    }
  }
}

void add_elevations_to_single_tile(GraphReader& graphreader,
                                   std::mutex& graphreader_lck,
                                   cache_t& cache,
                                   const std::unique_ptr<valhalla::skadi::sample>& sample,
                                   GraphId& tile_id) {
  // Get the tile. Serialize the entire tile?
  GraphTileBuilder tilebuilder(graphreader.tile_dir(), tile_id, true);

  // Set the has_elevation flag. TODO - do we need to know if any elevation is actually
  // retrieved/used?
  tilebuilder.header_builder().set_has_elevation(true);

  // Iterate through the nodes edges, get the node lat,lng, sample and store elevation.
  std::vector<uint32_t> enclosed_nodes;
  for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); ++i) {
    // Get a writeable reference to the node edge
    NodeInfo& nodeinfo = tilebuilder.node_builder(i);
    PointLL ll = nodeinfo.latlng(tilebuilder.header()->base_ll());
    nodeinfo.set_elevation(sample->get(ll));

    bool enclosed = nodeinfo.edge_count() > 0;
    for (uint32_t j = 0; j < nodeinfo.edge_count() && enclosed; ++j) {
      const DirectedEdge& edge = tilebuilder.directededge_builder(nodeinfo.edge_index() + j);
      enclosed = edge.is_shortcut() || is_structure(edge);
    }
    if (enclosed) {
      enclosed_nodes.push_back(i);
    }
  }

  // An enclosed node samples the terrain above or below it, so blend portal heights instead.
  // Nodes enclosed by the same structure share one walk.
  ankerl::unordered_dense::map<GraphId, std::optional<double>> structure_heights;
  for (uint32_t i : enclosed_nodes) {
    const GraphId id = node_key(tilebuilder, tilebuilder.node_builder(i),
                                GraphId(tile_id.tileid(), tile_id.level(), i));
    auto it = structure_heights.find(id);
    if (it == structure_heights.end()) {
      const Structure structure = read_structure(graphreader, id, sample);
      if (structure.oversized) {
        for (const auto& node : structure.nodes) {
          structure_heights.try_emplace(node.id, std::nullopt);
        }
      } else {
        interpolate(structure, structure_heights);
      }
      // the seed is a portal when a copy on another level has a regular edge
      it = structure_heights.try_emplace(id, std::nullopt).first;
    }
    if (it->second) {
      tilebuilder.node_builder(i).set_elevation(*it->second);
    }
  }

  // Reserve twice the number of directed edges in the tile. We do not directly know
  // how many EdgeInfo records exist but it cannot be more than 2x the directed edge count.
  uint32_t count = tilebuilder.header()->directededgecount();
  cache.clear();
  cache.reserve(2 * count);

  // Order the directed edges by edge_info_offset
  std::multimap<uint32_t, uint32_t> edge_info_offsets;
  for (uint32_t i = 0; i < count; ++i) {
    DirectedEdge& directededge = tilebuilder.directededge_builder(i);
    uint32_t edge_info_offset = directededge.edgeinfo_offset();
    edge_info_offsets.insert(std::pair<uint32_t, uint32_t>(edge_info_offset, i));
  }

  // Map existing edge info offsets to new (after adding encoded elevation)
  std::unordered_map<uint32_t, uint32_t> new_offsets;

  // Iterate through the directed edges
  uint32_t ei_offset = 0;
  for (auto& elem : edge_info_offsets) {
    // Get a writeable reference to the directed edge
    DirectedEdge& directededge = tilebuilder.directededge_builder(elem.second);

    // Get the edge info offset
    uint32_t edge_info_offset = directededge.edgeinfo_offset();

    // Check if this edge has been cached (based on edge info offset)
    auto found = cache.find(edge_info_offset);
    if (found == cache.cend()) {
      // Get the shape and length
      auto shape = tilebuilder.edgeinfo(&directededge).shape();
      auto length = directededge.length();

      // Grade estimation and max slopes
      std::tuple<double, double, double, double> forward_grades(0.0, 0.0, 0.0, 0.0);
      std::tuple<double, double, double, double> reverse_grades(0.0, 0.0, 0.0, 0.0);

      // Evenly sample the shape and add the last shape point. TODO - if close to the end do not!
      std::vector<PointLL> resampled =
          valhalla::midgard::resample_spherical_polyline(shape, POSTING_INTERVAL);
      resampled.push_back(shape.back());

      // Get the heights at each sampled point.
      std::vector<double> heights(resampled.size());
      if (is_structure(directededge)) {
        // Get height at beginning and end of the structure
        std::vector<PointLL> tmp;
        tmp.emplace_back(resampled.front());
        tmp.emplace_back(resampled.back());
        auto h = sample->get_all(tmp);
        heights[0] = h[0];
        float dh = (h[1] - heights[0]) / heights.size();
        for (size_t i = 1; i < heights.size(); ++i) {
          heights[i] = heights[i - 1] + dh;
        }
      } else {
        heights = sample->get_all(resampled);
      }

      // Compute "weighted" grades as well as max grades in both directions. Valid range
      // for weighted grades is between -10 and +15 which is then mapped to a value
      // between 0 to 15 for use in costing.
      auto grades = valhalla::skadi::weighted_grade(heights, POSTING_INTERVAL);
      if (length < kMinimumInterval) {
        // Keep the default grades - but set the mean elevation
        forward_grades = std::make_tuple(0.0, 0.0, 0.0, std::get<3>(grades));
        reverse_grades = std::make_tuple(0.0, 0.0, 0.0, std::get<3>(grades));
      } else {
        // Set the forward grades. Reverse the path and compute the
        // weighted grade in reverse direction.
        forward_grades = grades;
        std::reverse(heights.begin(), heights.end());
        reverse_grades = valhalla::skadi::weighted_grade(heights, POSTING_INTERVAL);
      }

      // Add elevation info to the geo attribute cache.
      float mean_elevation = std::get<3>(forward_grades);
      uint32_t forward_grade = static_cast<uint32_t>(std::get<0>(forward_grades) * .6 + 6.5);
      uint32_t reverse_grade = static_cast<uint32_t>(std::get<0>(reverse_grades) * .6 + 6.5);
      auto inserted =
          cache.insert({edge_info_offset,
                        std::make_tuple(forward_grade, reverse_grade, std::get<1>(forward_grades),
                                        std::get<2>(forward_grades), std::get<1>(reverse_grades),
                                        std::get<2>(reverse_grades))});
      found = inserted.first;

      // Store the new edge info offset
      new_offsets[edge_info_offset] = ei_offset;

      // Encode elevation along the edge and add to EdgeInfo along with the mean elevation.
      // Skip for shortcuts as they are resolved into underlying edges before reading elevation.
      // Skip for bridges, tunnels and ferries/rail-shuttles as elevation for them should be
      // linearly interpolated between nodes and runtime does this.
      // Increment the new edge info offset.
      std::vector<int8_t> encoded;
      if (!directededge.is_shortcut() && !is_structure(directededge)) {
        auto wayid = tilebuilder.edgeinfo(&directededge).wayid();
        encoded = encode_edge_elevation(sample, shape, length, wayid);
      }
      ei_offset += tilebuilder.set_elevation(edge_info_offset, mean_elevation, encoded);
    }

    // Edge elevation information. If the edge is forward (with respect to the shape)
    // use the first value, otherwise use the second.
    bool forward = directededge.forward();
    float max_up_slope = forward ? std::get<2>(found->second) : std::get<4>(found->second);
    float max_down_slope = forward ? std::get<3>(found->second) : std::get<5>(found->second);
    auto weighted_grade = forward ? std::get<0>(found->second) : std::get<1>(found->second);

    // Structure grades come from the ground height at both ends, which can be the terrain above
    // a tunnel or a steep bank at a ferry landing. Note - elevation along a route is "fixed"
    // for these cases but weighted grade can cause route issues.
    if (is_structure(directededge)) {
      // Clamp grades to +/- 3% (weighted grade values between 4 and 8)
      weighted_grade = std::clamp(weighted_grade, 4u, 8u);
      max_up_slope = std::min(3.0f, max_up_slope);
      max_down_slope = std::max(-3.0f, max_down_slope);
    }
    directededge.set_weighted_grade(weighted_grade);
    directededge.set_max_up_slope(max_up_slope);
    directededge.set_max_down_slope(max_down_slope);
  }

  // Iterate through all directed edges and update their edge info offsets
  for (uint32_t i = 0; i < tilebuilder.header()->directededgecount(); ++i) {
    DirectedEdge& directededge = tilebuilder.directededge_builder(i);
    uint32_t edge_info_offset = directededge.edgeinfo_offset();
    auto ei_offset = new_offsets.find(edge_info_offset);
    if (ei_offset == new_offsets.end()) {
      LOG_ERROR("Could not find edge info offset in the map");
    } else {
      directededge.set_edgeinfo_offset(ei_offset->second);
    }
  }

  // Update the tile
  tilebuilder.StoreTileData();

  // Check if we need to clear the tile cache
  if (graphreader.OverCommitted()) {
    graphreader_lck.lock();
    graphreader.Trim();
    graphreader_lck.unlock();
  }
}

/**
 * Adds elevation to a set of tiles. Each thread pulls a tile of the queue
 */
void add_elevations_to_multiple_tiles(const boost::property_tree::ptree& pt,
                                      std::deque<GraphId>& tilequeue,
                                      std::mutex& lock,
                                      const std::unique_ptr<valhalla::skadi::sample>& sample) {
  // Local Graphreader
  GraphReader graphreader(pt.get_child("mjolnir"));

  // We usually end up accessing the same shape twice (once for each direction along an edge).
  // Use a cache to record elevation attributes based on the EdgeInfo offset. This includes
  // weighted grade (forward and reverse) as well as max slopes (up/down for forward and reverse).
  cache_t geo_attribute_cache;

  // Check for more tiles
  while (true) {
    lock.lock();
    if (tilequeue.empty()) {
      lock.unlock();
      break;
    }
    // Get the next tile Id
    GraphId tile_id = tilequeue.front();
    tilequeue.pop_front();
    lock.unlock();

    add_elevations_to_single_tile(graphreader, lock, geo_attribute_cache, sample, tile_id);
  }
}

std::deque<GraphId> get_tile_ids(const boost::property_tree::ptree& pt) {
  std::deque<GraphId> tilequeue;
  GraphReader reader(pt.get_child("mjolnir"));
  // Create a randomized queue of tiles (at all levels) to work from
  auto tileset = reader.GetTileSet();
  for (const auto& id : tileset)
    tilequeue.emplace_back(id);

  std::random_device rd;
  std::shuffle(tilequeue.begin(), tilequeue.end(), std::mt19937(rd()));

  return tilequeue;
}

} // namespace

namespace valhalla {
namespace mjolnir {

void ElevationBuilder::Build(const boost::property_tree::ptree& pt,
                             std::deque<baldr::GraphId> tile_ids) {

  auto elevation = pt.get_optional<std::string>("additional_data.elevation");
  if (!elevation || !std::filesystem::exists(*elevation)) {
    LOG_WARN("Elevation storage directory does not exist");
    return;
  }

  SCOPED_TIMER();
  std::unique_ptr<skadi::sample> sample = std::make_unique<skadi::sample>(pt);
  std::uint32_t nthreads =
      std::max(static_cast<std::uint32_t>(1),
               pt.get<std::uint32_t>("mjolnir.concurrency", std::thread::hardware_concurrency()));

  if (tile_ids.empty())
    tile_ids = get_tile_ids(pt);

  std::vector<std::shared_ptr<std::thread>> threads(nthreads);

  LOG_INFO("Adding elevation to " + std::to_string(tile_ids.size()) + " tiles with " +
           std::to_string(nthreads) + " threads...");
  std::mutex lock;
  for (auto& thread : threads) {
    thread = std::make_shared<std::thread>(add_elevations_to_multiple_tiles, std::cref(pt),
                                           std::ref(tile_ids), std::ref(lock), std::ref(sample));
  }

  for (auto& thread : threads) {
    thread->join();
  }

  LOG_INFO("Finished");
}

} // namespace mjolnir
} // namespace valhalla
