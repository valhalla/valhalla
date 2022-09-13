#include <cinttypes>
#include <cstdint>
#include <future>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "config.h"

#include "baldr/rapidjson_utils.h"
#include "filesystem.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include "baldr/admininfo.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"
#include "midgard/aabb2.h"
#include "midgard/constants.h"
#include "midgard/distanceapproximator.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "mjolnir/util.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

// Geometry types for admin queries
typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
typedef boost::geometry::model::multi_polygon<polygon_type> multi_polygon_type;

filesystem::path config_file_path;

std::unordered_map<uint32_t, multi_polygon_type>
GetAdminInfo(sqlite3* db_handle,
             std::unordered_map<uint32_t, bool>& drive_on_right,
             const AABB2<PointLL>& aabb) {
  // Polys (return)
  std::unordered_map<uint32_t, multi_polygon_type> polys;

  // Form query
  std::string sql = "SELECT state.rowid, country.name, state.name, country.iso_code, ";
  sql += "state.iso_code, state.drive_on_right, st_astext(state.geom) ";
  sql += "from admins state, admins country where ";
  sql += "ST_Intersects(state.geom, BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + ")) and ";
  sql += "country.rowid = state.parent_admin and state.admin_level=4 ";
  sql += "and state.rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
  sql += "'admins' AND search_frame = BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + "));";

  sqlite3_stmt* stmt = 0;
  uint32_t ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);
  if (ret == SQLITE_OK) {
    uint32_t result = sqlite3_step(stmt);
    if (result == SQLITE_DONE) {
      // state/prov not found, try to find country
      sql = "SELECT rowid, name, "
            ", iso_code, "
            ", drive_on_right, st_astext(geom) from ";
      sql += " admins where ST_Intersects(geom, BuildMBR(" + std::to_string(aabb.minx()) + ",";
      sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
      sql += std::to_string(aabb.maxy()) + ")) and admin_level=2 ";
      sql += "and rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
      sql += "'admins' AND search_frame = BuildMBR(" + std::to_string(aabb.minx()) + ",";
      sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
      sql += std::to_string(aabb.maxy()) + "));";

      sqlite3_finalize(stmt);
      stmt = 0;
      ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);
      if (ret == SQLITE_OK) {
        result = 0;
        result = sqlite3_step(stmt);
      }
    }

    uint32_t index = 1;
    while (result == SQLITE_ROW) {

      uint32_t id = sqlite3_column_int(stmt, 0);
      std::string country_name = "";
      std::string state_name = "";
      std::string country_iso = "";
      std::string state_iso = "";

      if (sqlite3_column_type(stmt, 1) == SQLITE_TEXT) {
        country_name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1)));
      }

      if (sqlite3_column_type(stmt, 2) == SQLITE_TEXT) {
        state_name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2)));
      }

      if (sqlite3_column_type(stmt, 3) == SQLITE_TEXT) {
        country_iso = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3)));
      }

      if (sqlite3_column_type(stmt, 4) == SQLITE_TEXT) {
        state_iso = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4)));
      }

      bool dor = true;
      if (sqlite3_column_type(stmt, 5) == SQLITE_INTEGER) {
        dor = sqlite3_column_int(stmt, 5);
      }

      std::string geom = "";
      if (sqlite3_column_type(stmt, 6) == SQLITE_TEXT) {
        geom = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 6)));
      }

      multi_polygon_type multi_poly;
      boost::geometry::read_wkt(geom, multi_poly);
      polys.emplace(index, multi_poly);
      drive_on_right.emplace(index, dor);
      index++;

      result = sqlite3_step(stmt);
    }
  }
  if (stmt) {
    sqlite3_finalize(stmt);
    stmt = 0;
  }
  return polys;
}

// Benchmark the admin DB access
void Benchmark(const boost::property_tree::ptree& pt) {
  std::cout << "In Benchmark" << std::endl;

  uint32_t counts[128] = {};

  // Initialize the admin DB (if it exists)
  auto database = pt.get_optional<std::string>("admin");

  sqlite3* db_handle = nullptr;
  if (filesystem::exists(*database)) {
    sqlite3_stmt* stmt = 0;
    uint32_t ret = sqlite3_open_v2((*database).c_str(), &db_handle, SQLITE_OPEN_READONLY, nullptr);
    if (ret != SQLITE_OK) {
      LOG_ERROR("cannot open " + *database);
      sqlite3_close(db_handle);
      return;
    }

  } else {
    LOG_ERROR("Admin db " + *database + " not found.");
    return;
  }
  auto admin_conn = valhalla::mjolnir::make_spatialite_cache(db_handle);

  // Graphreader
  GraphReader reader(pt);
  auto local_level = TileHierarchy::levels().back().level;
  auto tiles = TileHierarchy::levels().back().tiles;

  // Iterate through the tiles and perform enhancements
  std::unordered_map<uint32_t, multi_polygon_type> polys;
  std::unordered_map<uint32_t, bool> drive_on_right;
  for (uint32_t id = 0; id < tiles.TileCount(); id++) {
    // Get the admin polys if there is data for tiles that exist
    GraphId tile_id(id, local_level, 0);
    if (reader.DoesTileExist(tile_id)) {
      polys = GetAdminInfo(db_handle, drive_on_right, tiles.TileBounds(id));
      LOG_INFO("polys: " + std::to_string(polys.size()));
      if (polys.size() < 128) {
        counts[polys.size()]++;
      }
    }
  }
  for (uint32_t i = 0; i < 128; i++) {
    if (counts[i] > 0) {
      LOG_INFO("Tiles with " + std::to_string(i) + " admin polys: " + std::to_string(counts[i]));
    }
  }
  sqlite3_close(db_handle);
}

bool ParseArguments(int argc, char* argv[]) {
  std::vector<std::string> input_files;

  try {
    // clang-format off
    cxxopts::Options options("valhalla_benchmark_admins", 
      "valhalla_benchmark_admins " VALHALLA_VERSION "\n\n"
      "valhalla_benchmark_admins is a program to time the admin queries\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("version")) {
      std::cout << "valhalla_benchmark_admins " << VALHALLA_VERSION << "\n";
      return EXIT_SUCCESS;
    }

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      return EXIT_SUCCESS;
    }

    if (result.count("config") &&
        filesystem::is_regular_file(config_file_path =
                                        filesystem::path(result["config"].as<std::string>()))) {
      return true;
    } else {
      std::cerr << "Configuration file is required\n\n" << options.help() << "\n\n";
    }
  } catch (const cxxopts::OptionException& e) {
    std::cout << "Unable to parse command line options because: " << e.what() << std::endl;
  }

  return false;
}

int main(int argc, char** argv) {
  if (!ParseArguments(argc, argv)) {
    return EXIT_FAILURE;
  }

  // Ccheck what type of input we are getting
  boost::property_tree::ptree pt;
  rapidjson::read_json(config_file_path.string(), pt);

  // Configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree =
      pt.get_child_optional("mjolnir.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  Benchmark(pt.get_child("mjolnir"));
  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  float secs = msecs * 0.001f;
  LOG_INFO("Time = " + std::to_string(secs) + " secs");

  return EXIT_SUCCESS;
}
