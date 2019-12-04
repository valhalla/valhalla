#include "statistics.h"
#include <cstdint>

#include "midgard/logging.h"

#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <spatialite.h>
#include <sqlite3.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace valhalla {
namespace mjolnir {

void statistics::build_db(const boost::property_tree::ptree& pt) {
  std::string database = "statistics.sqlite";
  if (boost::filesystem::exists(database)) {
    boost::filesystem::remove(database);
  }

  spatialite_init(0);

  sqlite3* db_handle;
  sqlite3_stmt* stmt;
  uint32_t ret;
  char* err_msg = NULL;
  std::string sql;

  ret =
      sqlite3_open_v2(database.c_str(), &db_handle, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("cannot open " + database);
    sqlite3_close(db_handle);
    db_handle = NULL;
    return;
  }

  // loading SpatiaLite as an extension
  sqlite3_enable_load_extension(db_handle, 1);
#if SQLITE_VERSION_NUMBER > 3008007
  sql = "SELECT load_extension('mod_spatialite')";
#else
  sql = "SELECT load_extension('libspatialite')";
#endif
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("load_extension() error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  LOG_INFO("Writing statistics database");

  // Turn on foreign keys
  sql = "PRAGMA foreign_keys = ON";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  LOG_INFO("Creating tables");

  create_tile_tables(db_handle, stmt);
  LOG_INFO("Created tile tables");

  create_country_tables(db_handle, stmt);
  LOG_INFO("Created country tables");

  create_exit_tables(db_handle, stmt);
  LOG_INFO("Created exit tables");

  insert_tile_data(db_handle, stmt);
  LOG_INFO("Tile info inserted");

  insert_country_data(db_handle, stmt);
  LOG_INFO("Country info inserted");

  insert_exit_data(db_handle, stmt);
  LOG_INFO("Exit info inserted");

  // Create Index on geometry column
  sql = "SELECT CreateSpatialIndex('tiledata', 'geom')";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  sql = "VACUUM";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  sql = "ANALYZE";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  sqlite3_close(db_handle);
  LOG_INFO("Statistics database saved to statistics.sqlite");
}
void statistics::create_tile_tables(sqlite3* db_handle, sqlite3_stmt* stmt) {
  uint32_t ret;
  char* err_msg = NULL;
  std::string sql;

  // Create table for tiles
  sql = "SELECT InitSpatialMetaData(); CREATE TABLE tiledata (";
  sql += "tileid INTEGER PRIMARY KEY,";
  sql += "tilearea REAL,";
  sql += "totalroadlen REAL,";
  sql += "motorway REAL,";
  sql += "pmary REAL,";
  sql += "secondary REAL,";
  sql += "tertiary REAL,";
  sql += "trunk REAL,";
  sql += "residential REAL,";
  sql += "unclassified REAL,";
  sql += "serviceother REAL";
  sql += ")";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  // Add tile geometry column
  sql = "SELECT AddGeometryColumn('tiledata', ";
  sql += "'geom', 4326, 'POLYGON', 2)";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  // Create table for tile data of road classes
  sql = "CREATE TABLE rclasstiledata (";
  sql += "tileid INTEGER,";
  sql += "type TEXT NOT NULL,";
  sql += "oneway REAL,";
  sql += "maxspeed REAL,";
  sql += "internaledges INTEGER,";
  sql += "named REAL,";
  sql += "FOREIGN KEY (tileid) REFERENCES tiledata(tileid)";
  sql += ")";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  // Create table for truck tile data of road classes
  sql = "CREATE TABLE truckrclasstiledata (";
  sql += "tileid INTEGER,";
  sql += "type TEXT NOT NULL,";
  sql += "hazmat REAL,";
  sql += "truck_route REAL,";
  sql += "height INTEGER,";
  sql += "width INTEGER,";
  sql += "length INTEGER,";
  sql += "weight INTEGER,";
  sql += "axle_load INTEGER,";
  sql += "FOREIGN KEY (tileid) REFERENCES tiledata(tileid)";
  sql += ")";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
}

void statistics::create_country_tables(sqlite3* db_handle, sqlite3_stmt* stmt) {
  uint32_t ret;
  char* err_msg = NULL;
  std::string sql;

  // Create tables for country data
  sql = "CREATE TABLE countrydata (";
  sql += "isocode TEXT PRIMARY KEY,";
  sql += "motorway REAL,";
  sql += "pmary REAL,";
  sql += "secondary REAL,";
  sql += "tertiary REAL,";
  sql += "trunk REAL,";
  sql += "residential REAL,";
  sql += "unclassified REAL,";
  sql += "serviceother REAL";
  sql += ")";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  // Create table for country data of road classes
  sql = "CREATE TABLE rclassctrydata (";
  sql += "isocode TEXT NOT NULL,";
  sql += "type TEXT NOT NULL,";
  sql += "oneway REAL,";
  sql += "maxspeed REAL,";
  sql += "internaledges INTEGER,";
  sql += "named REAL,";
  sql += "FOREIGN KEY (isocode) REFERENCES countrydata(isocode)";
  sql += ")";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  // Create table for truck country data of road classes
  sql = "CREATE TABLE truckrclassctrydata (";
  sql += "isocode TEXT NOT NULL,";
  sql += "type TEXT NOT NULL,";
  sql += "hazmat REAL,";
  sql += "truck_route REAL,";
  sql += "height INTEGER,";
  sql += "width INTEGER,";
  sql += "length INTEGER,";
  sql += "weight INTEGER,";
  sql += "axle_load INTEGER,";
  sql += "FOREIGN KEY (isocode) REFERENCES countrydata(isocode)";
  sql += ")";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
}

void statistics::create_exit_tables(sqlite3* db_handle, sqlite3_stmt* stmt) {
  uint32_t ret;
  char* err_msg = NULL;
  std::string sql;

  // Create table for exit data in tiles
  sql = "CREATE TABLE tile_exitinfo (";
  sql += "tileid INTEGER NOT NULL,";
  sql += "exitsign REAL";
  sql += ")";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  // Create table for fork data in tiles
  sql = "CREATE TABLE tile_forkinfo (";
  sql += "tileid INTEGER NOT NULL,";
  sql += "exitsign REAL";
  sql += ")";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  // Create table for exit data in countries
  sql = "CREATE TABLE ctry_exitinfo (";
  sql += "isocode TEXT NOT NULL,";
  sql += "exitsign REAL";
  sql += ")";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  // Create table for fork data in countries
  sql = "CREATE TABLE ctry_forkinfo (";
  sql += "isocode TEXT NOT NULL,";
  sql += "exitsign REAL";
  sql += ")";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
}

void statistics::insert_tile_data(sqlite3* db_handle, sqlite3_stmt* stmt) {

  uint32_t ret;
  char* err_msg = NULL;
  std::string sql;

  // Begin the prepared statements for tiledata
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  sql = "INSERT INTO tiledata (tileid, tilearea, totalroadlen, motorway, pmary, secondary, "
        "tertiary, trunk, residential, unclassified, serviceother, geom) ";
  sql += "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, GeomFromText(?, 4326))";
  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), strlen(sql.c_str()), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
    sqlite3_close(db_handle);
    return;
  }

  // Fill DB with the tile statistics
  for (auto tileid : tile_ids) {
    uint8_t index = 1;
    sqlite3_reset(stmt);
    sqlite3_clear_bindings(stmt);
    // Tile ID
    sqlite3_bind_int(stmt, index, tileid);
    ++index;
    // Tile Area
    sqlite3_bind_double(stmt, index, tile_areas[tileid]);
    ++index;
    // Total Road Length
    float totalLen = 0;
    for (auto rclass : rclasses) {
      totalLen += tile_lengths[tileid][rclass];
    }
    sqlite3_bind_double(stmt, index, totalLen);
    ++index;
    // Individual Road Class Lengths
    for (auto rclass : rclasses) {
      const std::string& roadStr = roadClassToString.at(rclass);
      sqlite3_bind_double(stmt, index, tile_lengths[tileid][rclass]);
      ++index;
    }
    // Use tile bounding box corners to make a polygon
    if (tile_geometries.find(tileid) != tile_geometries.end()) {
      auto maxx = std::to_string(tile_geometries.at(tileid).maxx());
      auto minx = std::to_string(tile_geometries.at(tileid).minx());
      auto maxy = std::to_string(tile_geometries.at(tileid).maxy());
      auto miny = std::to_string(tile_geometries.at(tileid).miny());
      std::string polyWKT = "POLYGON ((" + minx + " " + miny + ", " + minx + " " + maxy + ", " +
                            maxx + " " + maxy + ", " + maxx + " " + miny + ", " + minx + " " + miny +
                            "))";
      sqlite3_bind_text(stmt, index, polyWKT.c_str(), polyWKT.length(), SQLITE_STATIC);
    } else {
      LOG_ERROR("Geometry for tile " + std::to_string(tileid) + " not found.");
    }
    ret = sqlite3_step(stmt);
    if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
      continue;
    }
    LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
  }
  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  // Begin adding the statistics for each road type of tile data
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  sql = "INSERT INTO rclasstiledata (tileid, type, oneway, maxspeed, internaledges, named) ";
  sql += "VALUES (?, ?, ?, ?, ?, ?)";
  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
    sqlite3_close(db_handle);
    return;
  }

  // Fill the roadclass stats for tiles
  for (auto tileid : tile_ids) {
    for (auto rclass : rclasses) {
      uint8_t index = 1;
      sqlite3_reset(stmt);
      sqlite3_clear_bindings(stmt);
      // Tile ID (parent tile)
      sqlite3_bind_int(stmt, index, tileid);
      ++index;
      // Roadway type
      const auto& type = roadClassToString.at(rclass);
      sqlite3_bind_text(stmt, index, type.c_str(), type.length(), SQLITE_STATIC);
      ++index;
      // One Way data
      sqlite3_bind_double(stmt, index, tile_one_way[tileid][rclass]);
      ++index;
      // Max speed info
      sqlite3_bind_double(stmt, index, tile_speed_info[tileid][rclass]);
      ++index;
      // Internal edges count
      sqlite3_bind_int(stmt, index, tile_int_edges[tileid][rclass]);
      ++index;
      // Named roads
      sqlite3_bind_double(stmt, index, tile_named[tileid][rclass]);
      ret = sqlite3_step(stmt);
      if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
        continue;
      }
      LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
    }
  }
  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  // Begin adding the truck statistics for each road type of tile data
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  sql = "INSERT INTO truckrclasstiledata (tileid, type, hazmat, truck_route, height, width, "
        "length, weight, axle_load) ";
  sql += "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)";
  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
    sqlite3_close(db_handle);
    return;
  }

  // Fill the truck roadclass stats for tiles
  for (auto tileid : tile_ids) {
    for (auto rclass : rclasses) {
      uint8_t index = 1;
      sqlite3_reset(stmt);
      sqlite3_clear_bindings(stmt);
      // Tile ID (parent tile)
      sqlite3_bind_int(stmt, index, tileid);
      ++index;
      // Roadway type
      const auto& type = roadClassToString.at(rclass);
      sqlite3_bind_text(stmt, index, type.c_str(), type.length(), SQLITE_STATIC);
      ++index;
      // Hazmat
      sqlite3_bind_double(stmt, index, tile_hazmat[tileid][rclass]);
      ++index;
      // Truck Route
      sqlite3_bind_double(stmt, index, tile_truck_route[tileid][rclass]);
      ++index;
      // Height
      sqlite3_bind_int(stmt, index, tile_height[tileid][rclass]);
      ++index;
      // Width
      sqlite3_bind_int(stmt, index, tile_width[tileid][rclass]);
      ++index;
      // Length
      sqlite3_bind_int(stmt, index, tile_length[tileid][rclass]);
      ++index;
      // Weight
      sqlite3_bind_int(stmt, index, tile_weight[tileid][rclass]);
      ++index;
      // Axle Load
      sqlite3_bind_int(stmt, index, tile_axle_load[tileid][rclass]);
      ret = sqlite3_step(stmt);
      if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
        continue;
      }
      LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
    }
  }
  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
}

void statistics::insert_country_data(sqlite3* db_handle, sqlite3_stmt* stmt) {
  uint32_t ret;
  char* err_msg = NULL;
  std::string sql;

  // Begin the prepared statements for country data
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  sql = "INSERT INTO countrydata (isocode, motorway, pmary, secondary, tertiary, trunk, "
        "residential, unclassified, serviceother)";
  sql += "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)";
  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
    sqlite3_close(db_handle);
    return;
  }

  // Fill DB with the country statistics
  for (const auto& country : iso_codes) {
    uint8_t index = 1;
    sqlite3_reset(stmt);
    sqlite3_clear_bindings(stmt);
    // Country ISO
    sqlite3_bind_text(stmt, index, country.c_str(), country.length(), SQLITE_STATIC);
    ++index;
    // Individual Road Class Lengths
    for (auto rclass : rclasses) {
      const std::string& roadStr = roadClassToString.at(rclass);
      sqlite3_bind_double(stmt, index, country_lengths[country][rclass]);
      ++index;
    }
    ret = sqlite3_step(stmt);
    if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
      continue;
    }
    LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
  }
  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  // Begin adding the statistics for each road type of country data
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  sql = "INSERT INTO rclassctrydata (isocode, type, oneway, maxspeed, internaledges, named) ";
  sql += "VALUES (?, ?, ?, ?, ?, ?)";
  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
    sqlite3_close(db_handle);
    return;
  }

  // Fill the roadclass stats for countries
  for (const auto& country : iso_codes) {
    for (auto rclass : rclasses) {
      uint8_t index = 1;
      sqlite3_reset(stmt);
      sqlite3_clear_bindings(stmt);
      // ISO (parent ID)
      sqlite3_bind_text(stmt, index, country.c_str(), country.length(), SQLITE_STATIC);
      ++index;
      // Roadway type
      const auto& type = roadClassToString.at(rclass);
      sqlite3_bind_text(stmt, index, type.c_str(), type.length(), SQLITE_STATIC);
      ++index;
      // One Way data
      sqlite3_bind_double(stmt, index, country_one_way[country][rclass]);
      ++index;
      // Max speed info
      sqlite3_bind_double(stmt, index, country_speed_info[country][rclass]);
      ++index;
      // Internal edges count
      sqlite3_bind_int(stmt, index, country_int_edges[country][rclass]);
      ++index;
      // Named Roads
      sqlite3_bind_double(stmt, index, country_named[country][rclass]);
      ret = sqlite3_step(stmt);
      if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
        continue;
      }
      LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
    }
  }
  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  // Begin adding the truck statistics for each road type of country data
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  sql = "INSERT INTO truckrclassctrydata (isocode, type, hazmat, truck_route, height, width, "
        "length, weight, axle_load) ";
  sql += "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)";
  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
    sqlite3_close(db_handle);
    return;
  }

  // Fill the truck roadclass stats for countries
  for (const auto& country : iso_codes) {
    for (auto rclass : rclasses) {
      uint8_t index = 1;
      sqlite3_reset(stmt);
      sqlite3_clear_bindings(stmt);
      // ISO (parent ID)
      sqlite3_bind_text(stmt, index, country.c_str(), country.length(), SQLITE_STATIC);
      ++index;
      // Roadway type
      const auto& type = roadClassToString.at(rclass);
      sqlite3_bind_text(stmt, index, type.c_str(), type.length(), SQLITE_STATIC);
      ++index;
      // Hazmat
      sqlite3_bind_double(stmt, index, country_hazmat[country][rclass]);
      ++index;
      // Truck Route
      sqlite3_bind_double(stmt, index, country_truck_route[country][rclass]);
      ++index;
      // Height
      sqlite3_bind_int(stmt, index, country_height[country][rclass]);
      ++index;
      // Width
      sqlite3_bind_int(stmt, index, country_width[country][rclass]);
      ++index;
      // Length
      sqlite3_bind_int(stmt, index, country_length[country][rclass]);
      ++index;
      // Weight
      sqlite3_bind_int(stmt, index, country_weight[country][rclass]);
      ++index;
      // Axle Load
      sqlite3_bind_int(stmt, index, country_axle_load[country][rclass]);
      ret = sqlite3_step(stmt);
      if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
        continue;
      }
      LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
    }
  }
  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
}

void statistics::insert_exit_data(sqlite3* db_handle, sqlite3_stmt* stmt) {
  uint32_t ret;
  char* err_msg = NULL;
  std::string sql;

  // Begin adding the statistics for exits in tiles
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  sql = "INSERT INTO tile_exitinfo (tileid, exitsign) ";
  sql += "VALUES (?, ?)";
  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
    sqlite3_close(db_handle);
    return;
  }

  // Fill the exit stats for tiles
  for (auto it = tile_exit_signs.cbegin(); it != tile_exit_signs.cend(); it++) {
    uint8_t index = 1;
    sqlite3_reset(stmt);
    sqlite3_clear_bindings(stmt);
    // Tile ID (parent tile)
    sqlite3_bind_int(stmt, index, it->first);
    ++index;
    // Does it have an exit sign?
    float percent =
        static_cast<float>(it->second) / static_cast<float>(tile_exit_count.at(it->first));
    ;
    sqlite3_bind_double(stmt, index, percent);
    ret = sqlite3_step(stmt);
    if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
      continue;
    }
    LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
  }
  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  // Begin adding the statistics for forks in tiles
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  sql = "INSERT INTO tile_forkinfo (tileid, exitsign) ";
  sql += "VALUES (?, ?)";
  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
    sqlite3_close(db_handle);
    return;
  }

  // Fill the fork exit stats for tiles
  for (auto it = tile_fork_signs.cbegin(); it != tile_fork_signs.cend(); it++) {
    uint8_t index = 1;
    sqlite3_reset(stmt);
    sqlite3_clear_bindings(stmt);
    // Tile ID (parent tile)
    sqlite3_bind_int(stmt, index, it->first);
    ++index;
    // Does it have an exit sign?
    float percent =
        static_cast<float>(it->second) / static_cast<float>(tile_fork_count.at(it->first));
    sqlite3_bind_double(stmt, index, percent);
    ret = sqlite3_step(stmt);
    if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
      continue;
    }
    LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
  }
  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  // Begin adding the exit statistics for countries
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  sql = "INSERT INTO ctry_exitinfo (isocode, exitsign) ";
  sql += "VALUES (?, ?)";
  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
    sqlite3_close(db_handle);
    return;
  }

  // Fill the exit stats for countries
  for (auto it = ctry_exit_signs.cbegin(); it != ctry_exit_signs.cend(); it++) {
    uint8_t index = 1;
    sqlite3_reset(stmt);
    sqlite3_clear_bindings(stmt);
    // ISO (parent ID)
    sqlite3_bind_text(stmt, index, it->first.c_str(), it->first.length(), SQLITE_STATIC);
    ++index;
    // Does this exit have signs?
    float percent =
        static_cast<float>(it->second) / static_cast<float>(ctry_exit_count.at(it->first));
    sqlite3_bind_double(stmt, index, percent);
    ret = sqlite3_step(stmt);
    if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
      continue;
    }
    LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
  }
  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  // Begin adding the fork statistics for countries
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }

  sql = "INSERT INTO ctry_forkinfo (isocode, exitsign) ";
  sql += "VALUES (?, ?)";
  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
    sqlite3_close(db_handle);
    return;
  }

  // Fill the fork exit stats for countries
  for (auto it = ctry_fork_signs.cbegin(); it != ctry_fork_signs.cend(); it++) {
    uint8_t index = 1;
    sqlite3_reset(stmt);
    sqlite3_clear_bindings(stmt);
    // ISO (parent ID)
    sqlite3_bind_text(stmt, index, it->first.c_str(), it->first.length(), SQLITE_STATIC);
    ++index;
    // Does this exit have signs?
    float percent =
        static_cast<float>(it->second) / static_cast<float>(ctry_fork_count.at(it->first));
    sqlite3_bind_double(stmt, index, percent);
    ret = sqlite3_step(stmt);
    if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
      continue;
    }
    LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
  }
  sqlite3_finalize(stmt);
  ret = sqlite3_exec(db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
}

} // namespace mjolnir
} // namespace valhalla
