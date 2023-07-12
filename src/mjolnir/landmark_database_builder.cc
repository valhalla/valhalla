#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

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
bool LandmarkDatabase::connect_database() {
  switch (access_mode_) {
    case AccessMode::ReadWriteCreate:
      open_flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE;
      break;
    case AccessMode::ReadOnly:
      open_flags = SQLITE_OPEN_READONLY;
      break;
    case AccessMode::ReadWrite:
      open_flags = SQLITE_OPEN_READWRITE;
      break;
    default:
      LOG_ERROR("invalid access mode");
      return false;
  }

  if (!LandmarkDatabase::open_database()) {
    return false;
  }

  if (access_mode_ == AccessMode::ReadWriteCreate) {
    if (!create_landmarks_table()) {
      return false;
    }
    if (!create_spatial_index()) {
      return false;
    }
    LOG_INFO("Created database and landmarks table");
  }

  if (!prepare_insert_stmt()) {
    return false;
  }
  if (!prepare_bounding_box_stmt()) {
    return false;
  }

  LOG_INFO("Successfully connected to database");
  return true;
}

bool LandmarkDatabase::open_database() {
  ret = sqlite3_open_v2(database.c_str(), &db, open_flags, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("cannot open " + database);
    sqlite3_close(db);
    return false;
  }

  // loading SpatiaLite as an extension
  db_conn = make_spatialite_cache(db);
  LOG_INFO("Opened database and loaded Spatialite extension");

  return true;
}

bool LandmarkDatabase::create_landmarks_table() {
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

bool LandmarkDatabase::create_spatial_index() {
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

inline bool LandmarkDatabase::prepare_insert_stmt() {
  sql = "INSERT INTO landmarks (name, type, geom) ";
  sql += "VALUES (?, ?, MakePoint(?, ?, 4326))";
  // sql += "VALUES (?, ?, CastToPoint(ST_GeomFromText('POINT(? ?)', 4326)))";

  ret = sqlite3_prepare_v2(db, sql.c_str(), strlen(sql.c_str()), &insert_stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db)));
    sqlite3_close(db);
    return false;
  }

  LOG_INFO("prepared insert statement");
  return true;
}

bool LandmarkDatabase::insert_landmark(const std::string& name,
                                       const std::string& type,
                                       const double longitude,
                                       const double latitude) {
  sqlite3_reset(insert_stmt);
  sqlite3_clear_bindings(insert_stmt);

  if (name != "") {
    sqlite3_bind_text(insert_stmt, 1, name.c_str(), name.length(), SQLITE_STATIC);
  } else {
    sqlite3_bind_null(insert_stmt, 1);
  }

  if (type != "") {
    sqlite3_bind_text(insert_stmt, 2, type.c_str(), type.length(), SQLITE_STATIC);
  } else {
    sqlite3_bind_null(insert_stmt, 2);
  }

  sqlite3_bind_double(insert_stmt, 3, longitude);
  sqlite3_bind_double(insert_stmt, 4, latitude);

  LOG_INFO(sqlite3_expanded_sql(insert_stmt));

  ret = sqlite3_step(insert_stmt);
  if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
    LOG_INFO("Inserted landmark");
    return true;
  }

  LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db)));
  LOG_ERROR("sqlite3_step() Name: " + name);
  LOG_ERROR("sqlite3_step() Type: " + type);
  LOG_ERROR("sqlite3_step() longitude: " + std::to_string(longitude));
  LOG_ERROR("sqlite3_step() Latitude: " + std::to_string(latitude));

  return false;
}

bool LandmarkDatabase::prepare_bounding_box_stmt() {
  sql =
      "SELECT name, type, X(geom), Y(geom) FROM landmarks WHERE ST_Covers(BuildMbr(?, ?, ?, ?, 4326), geom)";

  ret = sqlite3_prepare_v2(db, sql.c_str(), strlen(sql.c_str()), &bounding_box_stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db)));
    sqlite3_close(db);
    return false;
  }

  LOG_INFO("prepared bounding box statement");
  return true;
}

bool LandmarkDatabase::get_landmarks_in_bounding_box(std::vector<Landmark>* landmarks,
                                                     const double minLat,
                                                     const double minLong,
                                                     const double maxLat,
                                                     const double maxLong) {
  sqlite3_reset(bounding_box_stmt);
  sqlite3_clear_bindings(bounding_box_stmt);

  sqlite3_bind_double(bounding_box_stmt, 1, minLong);
  sqlite3_bind_double(bounding_box_stmt, 2, minLat);
  sqlite3_bind_double(bounding_box_stmt, 3, maxLong);
  sqlite3_bind_double(bounding_box_stmt, 4, maxLat);

  LOG_INFO(sqlite3_expanded_sql(bounding_box_stmt));

  ret = sqlite3_step(bounding_box_stmt);
  while (ret == SQLITE_ROW) {
    const char* name = reinterpret_cast<const char*>(sqlite3_column_text(bounding_box_stmt, 0));
    const char* type = reinterpret_cast<const char*>(sqlite3_column_text(bounding_box_stmt, 1));
    double lng = sqlite3_column_double(bounding_box_stmt, 2);
    double lat = sqlite3_column_double(bounding_box_stmt, 3);
    landmarks->emplace_back(Landmark{name, type, lng, lat});

    ret = sqlite3_step(bounding_box_stmt);
  }

  if (ret == SQLITE_DONE || SQLITE_OK) {
    return true;
  }

  LOG_ERROR("Error: " + std::string(sqlite3_errmsg(db)));
  sqlite3_close(db);
  return false;
}

void LandmarkDatabase::close_database() {
  sqlite3_close(db);
  LOG_INFO("Closed database");
}

} // end namespace mjolnir

} // end namespace valhalla
