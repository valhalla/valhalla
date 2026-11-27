#include "argparse_utils.h"
#include "baldr/datetime.h"
#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "config.h"
#include "midgard/logging.h"
#include "mjolnir/admin.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/util.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <algorithm>
#include <filesystem>
#include <future>
#include <random>
#include <thread>
#include <vector>

namespace bpt = boost::property_tree;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;

namespace {

struct result_t {
  size_t changed_tiles = 0;
  bool has_unknown_tz = false;
};

void update(const std::string& tile_dir,
            const std::string& tz_db_path,
            std::vector<GraphId>::const_iterator tile_start,
            std::vector<GraphId>::const_iterator tile_end,
            std::promise<result_t>& result) {
  try {
    // sqlite & geos handles aren't shared between threads
    auto tz_db = AdminDB::open(tz_db_path);
    if (!tz_db)
      throw std::runtime_error("Time zone db " + tz_db_path + " can't be opened");

    result_t res;
    for (; tile_start != tile_end; ++tile_start) {
      const auto& tile_id = *tile_start;
      GraphTileBuilder tile(tile_dir, tile_id, false);
      const auto tile_nodes = tile.GetNodes();
      if (tile_nodes.empty())
        continue;

      const auto tz_polys =
          GetTimeZones(*tz_db,
                       TileHierarchy::get_tiling(tile_id.level()).TileBounds(tile_id.tileid()), true);

      std::vector<NodeInfo> nodes(tile_nodes.begin(), tile_nodes.end());
      bool tile_changed = false;
      for (auto& node : nodes) {
        // same lookup as GraphBuilder
        const uint32_t tz_index =
            tz_polys.size() == 1 ? tz_polys.begin()->first
                                 : GetMultiPolyId(tz_polys, node.latlng(tile.header()->base_ll()));
        // if there's a new time zone we don't know about yet, it's carved out of an existing one
        // so just keep it. we warn the user if that happens, so valhalla can be re-compiled
        if (tz_index == DateTime::kUnknownTimeZone) {
          res.has_unknown_tz = true;
        } else if (tz_index != node.timezone()) {
          node.set_timezone(tz_index);
          tile_changed = true;
        }
      }

      if (!tile_changed)
        continue;

      const auto tile_edges = tile.GetDirectedEdges();
      tile.Update(nodes, {tile_edges.begin(), tile_edges.end()});
      ++res.changed_tiles;
    }
    result.set_value(res);
  } catch (...) { result.set_exception(std::current_exception()); }
}

} // namespace

int main(int argc, char** argv) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  bpt::ptree config;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_PRINT_VERSION + "\n\n"
      "Re-assigns the time zone of every node in mjolnir.tile_dir from the time zones in "
      "mjolnir.timezone, without rebuilding the tiles. Nodes in time zones unknown to this build "
      "of valhalla keep their current time zone.\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("j,concurrency", "Number of threads to use. Defaults to all threads.", cxxopts::value<uint32_t>());
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config, true))
      return EXIT_SUCCESS;
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  // validate the sqlite path
  const auto tz_db_path = config.get_optional<std::string>("mjolnir.timezone");
  if (!tz_db_path || !std::filesystem::exists(*tz_db_path)) {
    LOG_ERROR("mjolnir.timezone is not configured or doesn't exist");
    return EXIT_FAILURE;
  }
  const auto tile_dir = config.get<std::string>("mjolnir.tile_dir");

  // only ever touch the tile directory
  config.get_child("mjolnir").erase("tile_extract");
  config.get_child("mjolnir").erase("tile_url");
  config.get_child("mjolnir").erase("traffic_extract");

  // transit nodes get their time zone from the GTFS feeds
  std::vector<GraphId> tiles;
  {
    GraphReader reader(config.get_child("mjolnir"));
    for (const auto& id : reader.GetTileSet()) {
      if (id.level() != TileHierarchy::GetTransitLevel().level)
        tiles.push_back(id);
    }
  }
  // dense areas are clustered by tile id, spread them over the threads
  std::shuffle(tiles.begin(), tiles.end(), std::mt19937(3));
  LOG_INFO("Updating time zones of " + std::to_string(tiles.size()) + " tiles from " + *tz_db_path);

  std::vector<std::thread> threads(config.get<unsigned int>("mjolnir.concurrency"));
  std::vector<std::promise<result_t>> results(threads.size());
  const size_t floor = tiles.size() / threads.size();
  const size_t at_ceiling = tiles.size() - (threads.size() * floor);
  auto tile_end = tiles.cbegin();
  for (size_t i = 0; i < threads.size(); ++i) {
    auto tile_start = tile_end;
    tile_end += (i < at_ceiling ? floor + 1 : floor);
    threads[i] = std::thread(update, std::cref(tile_dir), std::cref(*tz_db_path), tile_start,
                             tile_end, std::ref(results[i]));
  }
  for (auto& thread : threads)
    thread.join();

  size_t changed_tiles_total = 0;
  bool has_unknown_tz = false;
  bool failed = false;
  for (auto& result : results) {
    try {
      const auto res = result.get_future().get();
      changed_tiles_total += res.changed_tiles;
      has_unknown_tz |= res.has_unknown_tz;
    } catch (const std::exception& e) {
      LOG_ERROR(e.what());
      failed = true;
    }
  }
  LOG_INFO("Updated the time zones of " + std::to_string(changed_tiles_total) + " tiles");
  if (has_unknown_tz)
    LOG_WARN("Some nodes are in time zones unknown to this build of valhalla and kept their old time "
             "zone. It's recommended to re-compile valhalla with at least the same tzdata version "
             "and re-run " +
             program);

  // a failed thread might have written tiles before it threw
  if (changed_tiles_total || failed)
    set_tileset_build_id(tile_dir);

  return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
