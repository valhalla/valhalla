#include "mjolnir/edgeboundsbuilder.h"

#include <future>
#include <random>
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
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/util.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

void add_edge_bounds_to_single_tile(GraphReader& graphreader,
                                   std::mutex& graphreader_lck,
                                   GraphId& tile_id) {
  // Get the tile. Serialize the entire tile?
  GraphTileBuilder tilebuilder(graphreader.tile_dir(), tile_id, true);

  // Reserve twice the number of directed edges in the tile. We do not directly know
  // how many EdgeInfo records exist but it cannot be more than 2x the directed edge count.
  uint32_t count = tilebuilder.header()->directededgecount();
  std::unordered_set<uint32_t> cache;
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
    // Get a writeable reference to the directed edge. Get its edge info offset
    DirectedEdge& directededge = tilebuilder.directededge_builder(elem.second);
    uint32_t edge_info_offset = directededge.edgeinfo_offset();

    // Check if this edge has been cached (based on edge info offset)
    if (cache.find(edge_info_offset) == cache.cend()) {
      // Compute the bounding circle from the edgeinfo shape
      auto bc = get_bounding_circle(tilebuilder.edgeinfo(&directededge).shape());
      
      // Store the new edge info offset
      new_offsets[edge_info_offset] = ei_offset;

      // Add to edgeinfo and cache this edge info offset
      ei_offset += tilebuilder.set_edge_bounds(edge_info_offset, bc);
      cache.insert(edge_info_offset);
    }
  }

  // Iterate through all directed edges and update their edge info offsets
  for (uint32_t i = 0; i < tilebuilder.header()->directededgecount(); ++i) {
    DirectedEdge& directededge = tilebuilder.directededge_builder(i);
    auto ei_offset = new_offsets.find(directededge.edgeinfo_offset());
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
void add_edge_bounds_to_multiple_tiles(const boost::property_tree::ptree& pt,
                                      std::deque<GraphId>& tilequeue,
                                      std::mutex& lock,
                                      std::promise<uint32_t>& /*result*/) {
  // Local Graphreader
  GraphReader graphreader(pt.get_child("mjolnir"));

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

    add_edge_bounds_to_single_tile(graphreader, lock, tile_id);
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

void EdgeBoundsBuilder::Build(const boost::property_tree::ptree& pt,
                              std::deque<baldr::GraphId> tile_ids) {

  std::uint32_t nthreads =
      std::max(static_cast<std::uint32_t>(1),
               pt.get<std::uint32_t>("mjolnir.concurrency", std::thread::hardware_concurrency()));

  if (tile_ids.empty())
    tile_ids = get_tile_ids(pt);

  std::vector<std::shared_ptr<std::thread>> threads(nthreads);
  std::vector<std::promise<uint32_t>> results(nthreads);

  LOG_INFO("Adding edge bounds to " + std::to_string(tile_ids.size()) + " tiles with " +
           std::to_string(nthreads) + " threads...");
  std::mutex lock;
  for (auto& thread : threads) {
    results.emplace_back();
    thread.reset(new std::thread(add_edge_bounds_to_multiple_tiles, std::cref(pt), std::ref(tile_ids),
                                 std::ref(lock), std::ref(results.back())));
  }

  for (auto& thread : threads) {
    thread->join();
  }

  LOG_INFO("Finished");
}

} // namespace mjolnir
} // namespace valhalla
