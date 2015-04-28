#include "mjolnir/transitbuilder.h"
#include "mjolnir/idtable.h"
#include "mjolnir/graphtilebuilder.h"

#include <list>
#include <future>
#include <thread>
#include <mutex>
#include <sqlite3.h>
#include <spatialite.h>
#include <boost/filesystem/operations.hpp>

#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

// Struct to hold stats information during each threads work
struct builder_stats {
  uint32_t stops;  // TODO - placeholder until other stats are added

  // Accumulate stats from all threads
  void operator()(const builder_stats& other) {
    stops += other.stops;
  }
};

void GetStops(sqlite3 *db_handle, const AABB2& aabb) {

  if (!db_handle)
    return;

  sqlite3_stmt *stmt = 0;
  uint32_t ret;
  char *err_msg = nullptr;
  uint32_t result = 0;
  std::string geom;

  std::string sql = "SELECT stop_key,stop_id,stop_code,stop_name,stop_desc,zone_id,";
  sql += "stop_url,location_type, parent_station_key from stops where ";
  sql += "ST_Intersects(geom, BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + ")) ";
  sql += "and rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
  sql += "'stops' AND search_frame = BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + "));";

  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);

  if (ret == SQLITE_OK) {
    result = sqlite3_step(stmt);

    while (result == SQLITE_ROW) {

      int stop_key = sqlite3_column_int(stmt, 0);

      result = sqlite3_step(stmt);
    }
  }
  if (stmt) {

    sqlite3_finalize(stmt);
    stmt = 0;
  }
}

// We make sure to lock on reading and writing because we dont want to race
// since difference threads, use for the done map as well
void build(const boost::property_tree::ptree& pt, GraphReader& reader,
           IdTable& done_set, std::mutex& lock,
           std::promise<builder_stats>& result) {
  // Construct the transit database
  std::string dir = pt.get<std::string>("transit_dir");
  std::string db_name = pt.get<std::string>("db_name");
  std::string database = dir + "/" +  db_name;

  // Make sure it exists
  sqlite3 *db_handle = nullptr;
  if (boost::filesystem::exists(database)) {
    spatialite_init(0);
    sqlite3_stmt *stmt = 0;
    uint32_t ret;
    char *err_msg = NULL;
    std::string sql;
    ret = sqlite3_open_v2(database.c_str(), &db_handle,
                          SQLITE_OPEN_READONLY, NULL);
    if (ret != SQLITE_OK) {
      LOG_ERROR("cannot open " + database);
      sqlite3_close(db_handle);
      db_handle = NULL;
      return;
    }

    // loading SpatiaLite as an extension
    sqlite3_enable_load_extension(db_handle, 1);
    sql = "SELECT load_extension('libspatialite.so')";
    ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
    if (ret != SQLITE_OK) {
      LOG_ERROR("load_extension() error: " + std::string(err_msg));
      sqlite3_free(err_msg);
      sqlite3_close(db_handle);
      return;
    }
    LOG_INFO("SpatiaLite loaded as an extension");

  }
  else {
    LOG_INFO("Transit db " + database + " not found.  Transit will not be added.");
    return;
  }

  // Get some things we need throughout
  builder_stats stats{};
  lock.lock();
  auto tile_hierarchy = reader.GetTileHierarchy();
  auto local_level = tile_hierarchy.levels().rbegin()->second.level;
  auto tiles = tile_hierarchy.levels().rbegin()->second.tiles;
  lock.unlock();

  // Iterate through the tiles and perform enhancements
  for (uint32_t id = 0; id < tiles.TileCount(); id++) {
    GraphId tile_id(id, local_level, 0);

    // If no tile exists skip it
    if(!GraphReader::DoesTileExist(tile_hierarchy, tile_id))
      continue;

    // If someone else is working/worked on this tile we can skip it
    lock.lock();
    if (done_set.IsUsed(id)) {
      lock.unlock();
      continue;
    }
    done_set.set(id);
    lock.unlock();

    // Get writeable and readable tiles
    lock.lock();
    GraphTileBuilder tilebuilder(tile_hierarchy, tile_id);
    const GraphTile* tile = reader.GetGraphTile(tile_id);
    lock.unlock();

    // Iterate through tiles?

    GetStops(db_handle,tiles.TileBounds(id));

    // Add these stops to the tile

    // Get all routes within this tile

    // Get all trips with this tile

    // Get all transfers from any stop within the tile.


    // Write the new file
    lock.lock();
    // TODO
    lock.unlock();
  }

  if (db_handle)
    sqlite3_close(db_handle);

  // Send back the statistics
  result.set_value(stats);
}

}

namespace valhalla {
namespace mjolnir {

// Add transit to the graph
void TransitBuilder::Build(const boost::property_tree::ptree& pt) {
  // Graphreader
  GraphReader reader(pt.get_child("mjolnir.hierarchy"));

  // A place to hold worker threads and their results
  std::vector<std::shared_ptr<std::thread> > threads(
    std::max(static_cast<uint32_t>(1),
      pt.get<uint32_t>("concurrency", std::thread::hardware_concurrency())));

  // A place to hold the results of those threads (exceptions, stats)
  std::list<std::promise<builder_stats> > results;

  // A place for the threads to synchronize who is working/worked on what
  IdTable done_set(reader.GetTileHierarchy().levels().rbegin()->second.tiles.TileCount());

  // An atomic object we can use to do the synchronization
  std::mutex lock;

  // Start the threads
  LOG_INFO("Add transit to the local graph...");
  for (auto& thread : threads) {
    results.emplace_back();
    thread.reset(new std::thread(build, std::ref(pt.get_child("mjolnir.transit")),
                                 std::ref(reader), std::ref(done_set),
                                 std::ref(lock), std::ref(results.back())));
  }

  // Wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  // Check all of the outcomes, to see about maximum density (km/km2)
  builder_stats stats{};
  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      auto thread_stats = result.get_future().get();
      stats(thread_stats);
    }
    catch(std::exception& e) {
      //TODO: throw further up the chain?
    }
  }
}

}
}
