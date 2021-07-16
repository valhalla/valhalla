#include "mjolnir/util.h"

#include <boost/format.hpp>
#include <future>
#include <set>
#include <thread>
#include <utility>

#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/polyline2.h"
#include "midgard/util.h"
#include "skadi/sample.h"
#include "skadi/util.h"

#include "mjolnir/elevationbuilder.h"
#include "mjolnir/graphtilebuilder.h"

#include <deque>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

// How many meters to resample shape to when checking elevations.
constexpr double POSTING_INTERVAL = 60;

// Do not compute grade for intervals less than 10 meters.
constexpr double kMinimumInterval = 10.0f;

/**
 * Adds elevation to a set of tiles. Each thread pulls a tile of the queue
 */
void add_elevation(const boost::property_tree::ptree& pt,
                   std::deque<GraphId>& tilequeue,
                   std::mutex& lock,
                   const std::unique_ptr<valhalla::skadi::sample>& sample,
                   std::promise<uint32_t>& /*result*/) {
  // Local Graphreader
  GraphReader graphreader(pt.get_child("mjolnir"));

  // We usually end up accessing the same shape twice (once for each direction along an edge).
  // Use a cache to record elevation attributes based on the EdgeInfo offset. This includes
  // weighted grade (forward and reverse) as well as max slopes (up/down for forward and reverse).
  std::unordered_map<uint32_t, std::tuple<uint32_t, uint32_t, float, float, float, float>>
      geo_attribute_cache;

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

    // Get the tile. Serialize the entire tile?
    GraphTileBuilder tilebuilder(graphreader.tile_dir(), tile_id, true);

    // Set the has_elevation flag. TODO - do we need to know if any elevation is actually
    // retrieved/used?
    tilebuilder.header_builder().set_has_elevation(true);

    // Reserve twice the number of directed edges in the tile. We do not directly know
    // how many EdgeInfo records exist but it cannot be more than 2x the directed edge count.
    uint32_t count = tilebuilder.header()->directededgecount();
    geo_attribute_cache.clear();
    geo_attribute_cache.reserve(2 * count);

    // Iterate through the directed edges
    for (uint32_t i = 0; i < count; ++i) {
      // Get a writeable reference to the directed edge
      DirectedEdge& directededge = tilebuilder.directededge_builder(i);

      // Get the edge info offset
      uint32_t edge_info_offset = directededge.edgeinfo_offset();

      // Check if this edge has been cached (based on edge info offset)
      auto found = geo_attribute_cache.find(edge_info_offset);
      if (found == geo_attribute_cache.cend()) {
        // Get the shape and length
        auto shape = tilebuilder.edgeinfo(&directededge).shape();
        auto length = directededge.length();

        // Grade estimation and max slopes
        std::tuple<double, double, double, double> forward_grades(0.0, 0.0, 0.0, 0.0);
        std::tuple<double, double, double, double> reverse_grades(0.0, 0.0, 0.0, 0.0);
        if (!directededge.tunnel() && directededge.use() != Use::kFerry) {
          // Evenly sample the shape. If it is really short or a bridge just do both ends
          auto interval = POSTING_INTERVAL;
          std::vector<PointLL> resampled;
          if (length < POSTING_INTERVAL * 3 || directededge.bridge()) {
            resampled = {shape.front(), shape.back()};
            interval = length;
          } else {
            resampled = valhalla::midgard::resample_spherical_polyline(shape, interval);
          }

          // Get the heights at each sampled point. Compute "weighted"
          // grades as well as max grades in both directions. Valid range
          // for weighted grades is between -10 and +15 which is then
          // mapped to a value between 0 to 15 for use in costing.
          auto heights = sample->get_all(resampled);
          auto grades = valhalla::skadi::weighted_grade(heights, interval);
          if (length < kMinimumInterval) {
            // Keep the default grades - but set the mean elevation
            forward_grades = std::make_tuple(0.0, 0.0, 0.0, std::get<3>(grades));
            reverse_grades = std::make_tuple(0.0, 0.0, 0.0, std::get<3>(grades));
          } else {
            // Set the forward grades. Reverse the path and compute the
            // weighted grade in reverse direction.
            forward_grades = grades;
            std::reverse(heights.begin(), heights.end());
            reverse_grades = valhalla::skadi::weighted_grade(heights, interval);
          }
        }

        // Add elevation info to the geo attribute cache. TODO - add mean elevation.
        float mean_elevation = std::get<3>(forward_grades);
        uint32_t forward_grade = static_cast<uint32_t>(std::get<0>(forward_grades) * .6 + 6.5);
        uint32_t reverse_grade = static_cast<uint32_t>(std::get<0>(reverse_grades) * .6 + 6.5);
        auto inserted = geo_attribute_cache.insert(
            {edge_info_offset,
             std::make_tuple(forward_grade, reverse_grade, std::get<1>(forward_grades),
                             std::get<2>(forward_grades), std::get<1>(reverse_grades),
                             std::get<2>(reverse_grades))});
        found = inserted.first;

        // Set the mean elevation on EdgeInfo
        tilebuilder.set_mean_elevation(edge_info_offset, mean_elevation);
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

    // Update the tile
    tilebuilder.StoreTileData();

    // Check if we need to clear the tile cache
    if (graphreader.OverCommitted()) {
      lock.lock();
      graphreader.Trim();
      lock.unlock();
    }
  }
}

} // namespace

namespace valhalla {
namespace mjolnir {

void ElevationBuilder::Build(const boost::property_tree::ptree& pt) {

  auto hierarchy_properties = pt.get_child("mjolnir");
  std::string tile_dir = hierarchy_properties.get<std::string>("tile_dir");

  // Crack open some elevation data if its there. Return if it is not.
  boost::optional<std::string> elevation = pt.get_optional<std::string>("additional_data.elevation");
  std::unique_ptr<skadi::sample> sample;
  if (elevation && filesystem::exists(*elevation)) {
    sample.reset(new skadi::sample(*elevation));
  } else {
    LOG_INFO("ElevationBuilder: no elevation data, skipping");
    return;
  }

  // Create a randomized queue of tiles (at all levels) to work from
  std::deque<GraphId> tilequeue;
  GraphReader reader(pt.get_child("mjolnir"));
  auto tileset = reader.GetTileSet();
  for (const auto& id : tileset) {
    tilequeue.emplace_back(id);
  }
  std::random_device rd;
  std::shuffle(tilequeue.begin(), tilequeue.end(), std::mt19937(rd()));

  // An mutex we can use to do the synchronization
  std::mutex lock;

  // Setup threads
  uint32_t nthreads =
      std::max(static_cast<unsigned int>(1),
               pt.get<unsigned int>("mjolnir.concurrency", std::thread::hardware_concurrency()));
  std::vector<std::shared_ptr<std::thread>> threads(nthreads);

  // Setup promises. Hold the results for the threads
  std::vector<std::promise<uint32_t>> results(nthreads);

  LOG_INFO("Adding elevation to " + std::to_string(tilequeue.size()) + " tiles with " +
           std::to_string(nthreads) + " threads...");

  // Spawn the threads
  for (auto& thread : threads) {
    results.emplace_back();
    thread.reset(new std::thread(add_elevation, std::cref(pt), std::ref(tilequeue), std::ref(lock),
                                 std::ref(sample), std::ref(results.back())));
  }

  // Wait for threads to finish
  for (auto& thread : threads) {
    thread->join();
  }

  /** // Get the promise from the future
  for (auto& result : results) {
    auto data = result.get_future().get();
    // Total up duplicates for each level
    for (uint8_t i = 0; i < TileHierarchy::levels().size(); ++i) {
      for (auto& d : std::get<1>(data)[i]) {
        densities[i].push_back(d);
      }
    }
  } **/
  LOG_INFO("Finished");
}

} // namespace mjolnir
} // namespace valhalla
