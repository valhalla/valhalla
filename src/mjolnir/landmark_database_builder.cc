#include <cstdint>
#include <string>
#include <vector>
#include <iostream>

#include "baldr/graphconstants.h"
#include "filesystem.h"
#include "mjolnir/landmark_database_builder.h"
#include "mjolnir/util.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/property_tree/ptree.hpp>

#include <geos_c.h>

namespace valhalla {
namespace mjolnir {
bool LandmarkDatabase::openDatabase() {
  ret = sqlite3_open_v2(database.c_str(), &db, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("cannot open " + database);
    sqlite3_close(db);
    return false;
  }
  // loading SpatiaLite as an extension
  auto db_conn = make_spatialite_cache(db);

  LOG_INFO("Opened database and loaded Spatialite extension");
  return true;
}

bool LandmarkDatabase::createLandmarkTable() {
  sql = "SELECT InitSpatialMetaData(1); CREATE TABLE IF NOT EXISTS landmarks (";
  sql += "name TEXT,";
  sql += "type TEXT)";

  ret = sqlite3_exec(db, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db);
    return false;
  }

  LOG_INFO("Created landmarks table");

  /* creating a POINT Geometry column */
  sql = "SELECT AddGeometryColumn('landmarks', ";
  sql += "'geom', 4326, 'POINT', 2)";
  
  ret = sqlite3_exec(db, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db);
    return false;
  }

  LOG_INFO("Added geometry column");
  return true;
}

bool LandmarkDatabase::insertLandmark(const std::string& name, const std::string& type, 
                                      const std::string& longitude, const std::string& latitude) {
  sql = "INSERT INTO landmarks (name, type, geom) ";
  // sql += "VALUES (?, ?, CastToPoint(GeomFromText(?, 4326)))";
  // sql += "VALUES (?, ?, CastToPoint(MakePoint(-74.044548, 40.689253, 4326)))";
  sql += "VALUES (?, ?, CastToPoint(MakePoint(?, ?, 4326)))";

  ret = sqlite3_prepare_v2(db, sql.c_str(), strlen(sql.c_str()), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db)));
    sqlite3_close(db);
    return false;
  }

  sqlite3_reset(stmt);
  sqlite3_clear_bindings(stmt);

  if (name != "") {
    sqlite3_bind_text(stmt, 1, name.c_str(), name.length(), SQLITE_STATIC);
  } else {
    sqlite3_bind_null(stmt, 1);
  }

  if (type != "") {
    sqlite3_bind_text(stmt, 2, type.c_str(), type.length(), SQLITE_STATIC);
  } else {
    sqlite3_bind_null(stmt, 2);
  }

  sqlite3_bind_text(stmt, 3, longitude.c_str(), longitude.length(), SQLITE_STATIC);
  sqlite3_bind_text(stmt, 4, latitude.c_str(), latitude.length(), SQLITE_STATIC);

  /* performing INSERT INTO */
  ret = sqlite3_step(stmt);
  if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
    sqlite3_finalize(stmt);
    LOG_INFO("Inserted landmark");
    return true;
  }

  LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db)));
  LOG_ERROR("sqlite3_step() Name: " + name);
  LOG_ERROR("sqlite3_step() Type: " + type);
  LOG_ERROR("sqlite3_step() longitute: " + longitude);
  LOG_ERROR("sqlite3_step() Latitude: " + latitude);

  sqlite3_finalize(stmt);
  return false;
}

bool LandmarkDatabase::createSpatialIndex() {
  sql = "SELECT CreateSpatialIndex('landmarks', 'geom')";
  ret = sqlite3_exec(db, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db);
    return false;
  }
  LOG_INFO("Created spatial index");
  return true;
}

void LandmarkDatabase::closeDatabase() {
  sqlite3_close(db);
  LOG_INFO("Closed database");
}

} // end namespace mjolnir

} // end namespace valhalla
