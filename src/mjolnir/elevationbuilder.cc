#include "mjolnir/elevationbuilder.h"

#include <future>
#include <random>
#include <thread>
#include <utility>

#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "filesystem.h"
#include "midgard/elevation_encoding.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/polyline2.h"
#include "midgard/util.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/util.h"
#include "skadi/sample.h"
#include "skadi/util.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

// How many meters to resample shape to when checking elevations.
constexpr double POSTING_INTERVAL = 60;

// Do not compute grade for intervals less than 10 meters.
constexpr double kMinimumInterval = 10.0f;

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
    LOG_WARN("edge elevation wayid = " + std::to_string(wayid) + " exceeds difference with " +
             std::to_string(diff) + " meters.");
  }
  return encoded;
}

/**
 * Encode elevation for a bridge, tunnel, ferry.
 */
std::vector<int8_t> encode_btf_elevation(const std::unique_ptr<valhalla::skadi::sample>& sample,
                                         const std::vector<PointLL>& shape,
                                         const uint32_t length,
                                         uint32_t wayid) {
  // Compute a uniform sampling interval along the edge based on its length.
  double interval = sampling_interval(length);

  // Sample at the first and last shape point
  double h1 = sample->get(shape.front());
  double h2 = sample->get(shape.back());

  // Use linear interpolation from h1 to h2 along the length of the edge
  uint32_t n = static_cast<uint32_t>(length / interval) + 1;
  std::vector<double> heights(n);
  heights.front() = h1;
  float delta = (h2 - h1) / n;
  for (uint32_t i = 1; i < n - 1; ++i) {
    heights[i] = heights[i - 1] + delta;
  }
  heights.back() = h2;

  // Encode the elevation.
  bool error = false;
  auto e = encode_elevation(heights, error);
  if (error) {
    double diff = 0;
    for (size_t i = 1; i < heights.size(); i++) {
      auto d = std::abs(heights[i] - heights[i - 1]);
      diff = d < diff ? diff : d;
      LOG_DEBUG("  " + std::to_string(heights[i]));
    }
    LOG_WARN("BTF edge elevation wayid = " + std::to_string(wayid) + " exceeds difference with " +
             std::to_string(diff) + " meters.");
  }
  return e;
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
  for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); ++i) {
    // Get a writeable reference to the node edge
    NodeInfo& nodeinfo = tilebuilder.node_builder(i);
    PointLL ll = nodeinfo.latlng(tilebuilder.header()->base_ll());
    nodeinfo.set_elevation(sample->get(ll));
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
      if (directededge.bridge() || directededge.tunnel() || directededge.use() == Use::kFerry) {
        // Get height at beginning and end of bridge/tunnel
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
      // Bridges, tunnels, ferries are special cases. Increment the new edge info offset.
      std::vector<int8_t> encoded;
      auto wayid = tilebuilder.edgeinfo(&directededge).wayid();
      if (directededge.bridge() || directededge.tunnel() || directededge.use() == Use::kFerry) {
        encoded = encode_btf_elevation(sample, shape, length, wayid);
      } else {
        encoded = encode_edge_elevation(sample, shape, length, wayid);
      }
      ei_offset += tilebuilder.set_elevation(edge_info_offset, mean_elevation, encoded);
    }

    // Edge elevation information. If the edge is forward (with respect to the shape)
    // use the first value, otherwise use the second.
    bool forward = directededge.forward();
    directededge.set_weighted_grade(forward ? std::get<0>(found->second)
                                            : std::get<1>(found->second));
    float max_up_slope = forward ? std::get<2>(found->second) : std::get<4>(found->second);
    float max_down_slope = forward ? std::get<3>(found->second) : std::get<5>(found->second);
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
                                      const std::unique_ptr<valhalla::skadi::sample>& sample,
                                      std::promise<uint32_t>& /*result*/) {
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
  boost::optional<std::string> elevation = pt.get_optional<std::string>("additional_data.elevation");
  boost::optional<bool> addElevation = pt.get_optional<bool>("additional_data.add_elevation_in_tiles");
  if (!elevation || !filesystem::exists(*elevation)) {
    LOG_WARN("Elevation storage directory does not exist");
    return;
  }
  if (addElevation == false) {
    LOG_WARN("Elevation won't be stored in tiles");
    return;
  }

  std::unique_ptr<skadi::sample> sample = std::make_unique<skadi::sample>(pt);
  std::uint32_t nthreads =
      std::max(static_cast<std::uint32_t>(1),
               pt.get<std::uint32_t>("mjolnir.concurrency", std::thread::hardware_concurrency()));

  if (tile_ids.empty())
    tile_ids = get_tile_ids(pt);

  std::vector<std::shared_ptr<std::thread>> threads(nthreads);
  std::vector<std::promise<uint32_t>> results(nthreads);

  LOG_INFO("Adding elevation to " + std::to_string(tile_ids.size()) + " tiles with " +
           std::to_string(nthreads) + " threads...");
  std::mutex lock;
  for (auto& thread : threads) {
    results.emplace_back();
    thread.reset(new std::thread(add_elevations_to_multiple_tiles, std::cref(pt), std::ref(tile_ids),
                                 std::ref(lock), std::ref(sample), std::ref(results.back())));
  }

  for (auto& thread : threads) {
    thread->join();
  }

  LOG_INFO("Finished");
}

} // namespace mjolnir
} // namespace valhalla
