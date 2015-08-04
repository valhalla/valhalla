
#include "mjolnir/statistics.h"
#include "mjolnir/graphvalidator.h"
#include "mjolnir/graphtilebuilder.h"

#include <ostream>
#include <boost/format.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <sqlite3.h>
#include <spatialite.h>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include <unordered_map>

#include "valhalla/midgard/aabb2.h"
#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace valhalla {
namespace mjolnir {

validator_stats::validator_stats ()
  : tile_lengths(), country_lengths(), tile_int_edges(),country_int_edges(),
    tile_one_way(), country_one_way(), tile_speed_info(), country_speed_info(),
    tile_named(), country_named(), tile_areas(), tile_geometries(),
    iso_codes(), tile_ids(), dupcounts(3), densities(3) { }

void validator_stats::add_tile_road (const uint32_t& tile_id, const RoadClass& rclass, float length) {
  tile_ids.insert(tile_id);
  tile_lengths[tile_id][rclass] += length;
}

void validator_stats::add_country_road (const std::string& ctry_code, const RoadClass& rclass, float length) {
  iso_codes.insert(ctry_code);
  country_lengths[ctry_code][rclass] += length;
}

void validator_stats::add_tile_int_edge (const uint32_t& tile_id, const RoadClass& rclass, const uint32_t& count) {
  tile_int_edges[tile_id][rclass] += count;
}

void validator_stats::add_country_int_edge (const std::string& ctry_code, const RoadClass& rclass, const uint32_t& count) {
  country_int_edges[ctry_code][rclass] += count;
}

void validator_stats::add_tile_one_way (const uint32_t& tile_id, const RoadClass& rclass, float length) {
  tile_one_way[tile_id][rclass] += length;
}

void validator_stats::add_country_one_way (const std::string& ctry_code, const RoadClass& rclass, float length) {
  country_one_way[ctry_code][rclass] += length;
}

void validator_stats::add_tile_speed_info (const uint32_t& tile_id, const RoadClass& rclass, float length) {
  tile_speed_info[tile_id][rclass] += length;
}

void validator_stats::add_country_speed_info (const std::string& ctry_code, const RoadClass& rclass, float length) {
  country_speed_info[ctry_code][rclass] += length;
}

void validator_stats::add_tile_named (const uint32_t& tile_id, const RoadClass& rclass, float length) {
  tile_named[tile_id][rclass] += length;
}

void validator_stats::add_country_named (const std::string& ctry_code, const RoadClass& rclass, float length) {
  country_named[ctry_code][rclass] += length;
}

void validator_stats::add_tile_area (const uint32_t& tile_id, const float area) {
  tile_areas[tile_id] = area;
}

void validator_stats::add_tile_geom (const uint32_t& tile_id, const AABB2<PointLL> geom) {
  tile_geometries[tile_id] = geom;
}

void validator_stats::add_density (float density, int level) {
  densities[level].push_back(density);
}

void validator_stats::add_dup (uint32_t newdup, int level) {
  dupcounts[level].push_back(newdup);
}

const std::unordered_set<uint32_t>& validator_stats::get_ids () const { return tile_ids; }

const std::unordered_set<std::string>& validator_stats::get_isos () const { return iso_codes; }

const std::unordered_map<uint32_t, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& validator_stats::get_tile_lengths () const { return tile_lengths; }

const std::unordered_map<std::string, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& validator_stats::get_country_lengths () const { return country_lengths; }

const std::unordered_map<uint32_t, std::unordered_map<RoadClass, uint32_t, validator_stats::rclassHasher> >& validator_stats::get_tile_int_edges () const { return tile_int_edges; }

const std::unordered_map<std::string, std::unordered_map<RoadClass, uint32_t, validator_stats::rclassHasher> >& validator_stats::get_country_int_edges () const { return country_int_edges; }

const std::unordered_map<uint32_t, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& validator_stats::get_tile_one_way () const { return tile_one_way; }

const std::unordered_map<std::string, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& validator_stats::get_country_one_way () const { return country_one_way; }

const std::unordered_map<uint32_t, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& validator_stats::get_tile_speed_info () const { return tile_speed_info; }

const std::unordered_map<std::string, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& validator_stats::get_country_speed_info () const { return country_speed_info; }

const std::unordered_map<uint32_t, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& validator_stats::get_tile_named () const { return tile_named; }

const std::unordered_map<std::string, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& validator_stats::get_country_named () const { return country_named; }

const std::unordered_map<uint32_t, float>& validator_stats::get_tile_areas() const { return tile_areas; }

const std::unordered_map<uint32_t, AABB2<PointLL>>& validator_stats::get_tile_geometries() const { return tile_geometries; }

const std::vector<uint32_t> validator_stats::get_dups(int level) const { return dupcounts[level]; }

const std::vector<float> validator_stats::get_densities(int level) const { return densities[level]; }

const std::vector<std::vector<uint32_t> > validator_stats::get_dups() const { return dupcounts; }

const std::vector<std::vector<float> > validator_stats::get_densities() const { return densities; }

void validator_stats::add (const validator_stats& stats) {
  auto newTileLengths = stats.get_tile_lengths();
  auto newTileAreas = stats.get_tile_areas();
  auto newTileGeom = stats.get_tile_geometries();
  auto newTileOneWay = stats.get_tile_one_way();
  auto newTileSpeed = stats.get_tile_speed_info();
  auto newTileIntEdges = stats.get_tile_int_edges();
  auto newTileNamed = stats.get_tile_named();
  auto ids = stats.get_ids();
  auto isos = stats.get_isos();
  for (auto& id : ids) {
    add_tile_area(id, newTileAreas[id]);
    add_tile_geom(id, newTileGeom[id]);
    for (auto& rclass : rclasses) {
      add_tile_road(id, rclass, newTileLengths[id][rclass]);
      add_tile_one_way(id, rclass, newTileOneWay[id][rclass]);
      add_tile_speed_info(id, rclass, newTileSpeed[id][rclass]);
      add_tile_int_edge(id, rclass, newTileIntEdges[id][rclass]);
      add_tile_named(id, rclass, newTileNamed[id][rclass]);
    }
  }
  auto newCountryLengths = stats.get_country_lengths();
  auto newCountryOneWay = stats.get_country_one_way();
  auto newCountrySpeed = stats.get_country_speed_info();
  auto newCountryIntEdges = stats.get_country_int_edges();
  auto newCountryNamed = stats.get_country_named();
  for (auto& iso : isos) {
    for (auto& rclass : rclasses) {
      add_country_road(iso, rclass, newCountryLengths[iso][rclass]);
      add_country_one_way(iso, rclass, newCountryOneWay[iso][rclass]);
      add_country_speed_info(iso, rclass, newCountrySpeed[iso][rclass]);
      add_country_int_edge(iso, rclass, newCountryIntEdges[iso][rclass]);
      add_country_named(iso, rclass, newCountryNamed[iso][rclass]);
    }
  }
  uint32_t level = 0;
  for (auto& dupvec : stats.get_dups()) {
    for (auto& dup : dupvec) {
      add_dup(dup, level);
    }
    level++;
  }
  level = 0;
  for (auto& densityvec : stats.get_densities()) {
    for (auto& density : densityvec) {
      add_density(density, level);
    }
    level++;
  }

}

void validator_stats::build_db(const boost::property_tree::ptree& pt) {
  // Get the location of the db file to write
  std::string dir = pt.get<std::string>("mjolnir.statistics.statistics_dir");
  std::string db_name = pt.get<std::string>("mjolnir.statistics.db_name");
  std::string database = dir + "/" + db_name;

  if (boost::filesystem::exists(database)) {
    boost::filesystem::remove(database);
  }

  spatialite_init(0);

  sqlite3 *db_handle;
  sqlite3_stmt *stmt;
  uint32_t ret;
  char *err_msg = NULL;
  std::string sql;

  ret = sqlite3_open_v2(database.c_str(), &db_handle, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
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

  // Turn on foreign keys
  sql = "PRAGMA foreign_keys = ON";
  ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  // Create table for tiles
  sql = "SELECT InitSpatialMetaData(); CREATE TABLE tiledata (";
  sql += "tileid INTEGER PRIMARY KEY,";
  sql += "tilearea REAL,";
  sql += "totalroadlen REAL,";
  sql += "motorway REAL,";
  sql += "pmary REAL,";
  sql += "residential REAL,";
  sql += "secondary REAL,";
  sql += "serviceother REAL,";
  sql += "tertiary REAL,";
  sql += "trunk REAL,";
  sql += "unclassified REAL";
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

  // Create table for countries
  sql = "CREATE TABLE countrydata (";
  sql += "isocode TEXT PRIMARY KEY,";
  sql += "motorway REAL,";
  sql += "pmary REAL,";
  sql += "residential REAL,";
  sql += "secondary REAL,";
  sql += "serviceother REAL,";
  sql += "tertiary REAL,";
  sql += "trunk REAL,";
  sql += "unclassified REAL";
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

  // Begin the prepared statements for tiledata
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  sql = "INSERT INTO tiledata (tileid, tilearea, totalroadlen, motorway, pmary, residential, secondary, serviceother, tertiary, trunk, unclassified, geom) ";
  sql += "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, GeomFromText(?, 4326))";
  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), strlen (sql.c_str()), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
    sqlite3_close (db_handle);
    return;
  }

  // Fill DB with the tile statistics
  for (auto tileid : tile_ids) {
    uint8_t index = 1;
    sqlite3_reset(stmt);
    sqlite3_clear_bindings;
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
      std::string roadStr = roadClassToString.at(rclass);
      sqlite3_bind_double(stmt, index, tile_lengths[tileid][rclass]);
      ++index;
    }
    // Use tile bounding box corners to make a polygon
    if (tile_geometries.find(tileid) != tile_geometries.end()) {
      auto maxx = std::to_string(tile_geometries.at(tileid).maxx());
      auto minx = std::to_string(tile_geometries.at(tileid).minx());
      auto maxy = std::to_string(tile_geometries.at(tileid).maxy());
      auto miny = std::to_string(tile_geometries.at(tileid).miny());
      std::string polyWKT = "POLYGON (("
          + minx + " " + miny + ", "
          + minx + " " + maxy + ", "
          + maxx + " " + maxy + ", "
          + maxx + " " + miny + ", "
          + minx + " " + miny + "))";
      sqlite3_bind_text (stmt, index, polyWKT.c_str(), polyWKT.length(), SQLITE_STATIC);
    } else {
      LOG_ERROR("Geometry for tile " + std::to_string(tileid) + " not found.");
    }
    ret = sqlite3_step (stmt);
    if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
      continue;
    }
    LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
  }
  sqlite3_finalize (stmt);
  ret = sqlite3_exec (db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free (err_msg);
    sqlite3_close (db_handle);
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
    sqlite3_close (db_handle);
    return;
  }

  // Fill the roadclass stats for tiles
  for (auto tileid : tile_ids) {
    for (auto rclass : rclasses) {
      uint8_t index = 1;
      sqlite3_reset(stmt);
      sqlite3_clear_bindings;
      // Tile ID (parent tile)
      sqlite3_bind_int(stmt, index, tileid);
      ++index;
      // Roadway type
      auto type = roadClassToString.at(rclass);
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
      ret = sqlite3_step (stmt);
      if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
        continue;
      }
      LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
    }
  }
  sqlite3_finalize (stmt);
  ret = sqlite3_exec (db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free (err_msg);
    sqlite3_close (db_handle);
    return;
  }

  // Begin the prepared statements for country data
  ret = sqlite3_exec(db_handle, "BEGIN", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db_handle);
    return;
  }
  sql = "INSERT INTO countrydata (isocode, motorway, pmary, residential, secondary, serviceother, tertiary, trunk, unclassified) ";
  sql += "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)";
  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db_handle)));
    sqlite3_close (db_handle);
    return;
  }

  // Fill DB with the country statistics
  for (auto country : iso_codes) {
    uint8_t index = 1;
    sqlite3_reset(stmt);
    sqlite3_clear_bindings;
    // Country ISO
    sqlite3_bind_text(stmt, index, country.c_str(), country.length(), SQLITE_STATIC);
    ++index;
    // Individual Road Class Lengths
    for (auto rclass : rclasses) {
      std::string roadStr = roadClassToString.at(rclass);
      sqlite3_bind_double(stmt, index, country_lengths[country][rclass]);
      ++index;
    }
    ret = sqlite3_step (stmt);
    if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
      continue;
    }
    LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
  }
  sqlite3_finalize (stmt);
  ret = sqlite3_exec (db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free (err_msg);
    sqlite3_close (db_handle);
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
    sqlite3_close (db_handle);
    return;
  }

  // Fill the roadclass stats for countries
  for (auto country : iso_codes) {
    for (auto rclass : rclasses) {
      uint8_t index = 1;
      sqlite3_reset(stmt);
      sqlite3_clear_bindings;
      // ISO (parent ID)
      sqlite3_bind_text(stmt, index, country.c_str(), country.length(), SQLITE_STATIC);
      ++index;
      // Roadway type
      auto type = roadClassToString.at(rclass);
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
      ret = sqlite3_step (stmt);
      if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
        continue;
      }
      LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db_handle)));
    }
  }
  sqlite3_finalize (stmt);
  ret = sqlite3_exec (db_handle, "COMMIT", NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free (err_msg);
    sqlite3_close (db_handle);
    return;
  }

  // Create Index on geometry column
  sql = "SELECT CreateSpatialIndex('tiledata', 'geom')";
  ret = sqlite3_exec (db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free (err_msg);
    sqlite3_close (db_handle);
    return;
  }
  LOG_INFO("Created spatial index");

  sql = "VACUUM ANALYZE";
  ret = sqlite3_exec (db_handle, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free (err_msg);
    sqlite3_close (db_handle);
    return;
  }
  sqlite3_close(db_handle);
  LOG_INFO("Done writing statistics DB.");
}
}
}
