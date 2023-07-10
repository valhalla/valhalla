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
  // auto db_conn = make_spatialite_cache(db);
  // LOG_INFO("Opened database and loaded Spatialite extension");

  // ret = sqlite3_db_config(db, SQLITE_DBCONFIG_ENABLE_LOAD_EXTENSION, 1, nullptr);
  ret = sqlite3_enable_load_extension(db, 1);
  if (ret != SQLITE_OK) {
    LOG_ERROR("cannot enable load extension");
    sqlite3_close(db);
    return false;
  }

  sql = "SELECT load_extension('mod_spatialite')";
  ret = sqlite3_exec(db, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db);
    return false;
  }

  LOG_INFO("Loaded spatialite extension");

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

bool LandmarkDatabase::insert_landmark(const std::string& name,
                                       const std::string& type,
                                       const double longitude,
                                       const double latitude) {
  sql = "INSERT INTO landmarks (name, type, geom) ";
  sql += "VALUES (?, ?, MakePoint(?, ?, 4326))";
  // sql += "VALUES (?, ?, CastToPoint(ST_GeomFromText('POINT(? ?)', 4326)))";

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

  sqlite3_bind_double(stmt, 3, longitude);
  sqlite3_bind_double(stmt, 4, latitude);

  LOG_INFO(sqlite3_expanded_sql(stmt));

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
  LOG_ERROR("sqlite3_step() longitude: " + std::to_string(longitude));
  LOG_ERROR("sqlite3_step() Latitude: " + std::to_string(latitude));

  sqlite3_finalize(stmt);
  return false;
}

bool LandmarkDatabase::get_landmarks_in_bounding_box(
    std::vector<Landmark>* landmarks,
    const double minLat,
    const double minLong,
    const double maxLat,
    const double maxLong) {
  sql = "SELECT name, type, X(geom), Y(geom) FROM landmarks WHERE ST_Covers(BuildMbr(?, ?, ?, ?, 4326), geom)";

  ret = sqlite3_prepare_v2(db, sql.c_str(), strlen(sql.c_str()), &stmt, NULL);
  if (ret != SQLITE_OK) {
    LOG_ERROR("SQL error: " + sql);
    LOG_ERROR(std::string(sqlite3_errmsg(db)));
    sqlite3_close(db);
    return false;
  }

  sqlite3_reset(stmt);
  sqlite3_clear_bindings(stmt);

  sqlite3_bind_double(stmt, 1, minLong);
  sqlite3_bind_double(stmt, 2, minLat);
  sqlite3_bind_double(stmt, 3, maxLong);
  sqlite3_bind_double(stmt, 4, maxLat);

  LOG_INFO(sqlite3_expanded_sql(stmt));

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const char* name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
    const char* type = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
    double lng = sqlite3_column_double(stmt, 2);
    double lat = sqlite3_column_double(stmt, 3);
    landmarks->emplace_back(Landmark{name, type, lng, lat});
  }

  ret = sqlite3_finalize(stmt);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(sqlite3_errmsg(db)));
    sqlite3_close(db);
    return false;
  }

  return true;
}

void LandmarkDatabase::close_database() {
  sqlite3_close(db);
  LOG_INFO("Closed database");
}

static int test_callback(void* data, int argc, char** argv, char** azColName) {
  fprintf(stderr, "%s: \n", (const char*)data);

  for (int i = 0; i < argc; i++) {
    printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
  }

  printf("\n");
  return 0;
}

bool LandmarkDatabase::test_select_query() {
  sql = "SELECT name, type FROM landmarks where name = 'Eiffel Tower'";

  const char* data = "Callback function called";
  ret = sqlite3_exec(db, sql.c_str(), test_callback, (void*)data, &err_msg);

  if (ret != SQLITE_OK) {
    fprintf(stderr, "SQL error: %s\n", err_msg);
    sqlite3_free(err_msg);
    return false;
  } else {
    fprintf(stdout, "Operation done successfully\n");
  }
  return true;
}

bool LandmarkDatabase::test_select_all() {
  sql = "SELECT name, type, X(geom), Y(geom) FROM landmarks";

  const char* data = "Callback function called";
  ret = sqlite3_exec(db, sql.c_str(), test_callback, (void*)data, &err_msg);

  if (ret != SQLITE_OK) {
    fprintf(stderr, "SQL error: %s\n", err_msg);
    sqlite3_free(err_msg);
    return false;
  } else {
    fprintf(stdout, "Operation done successfully\n");
  }
  return true;
}

} // end namespace mjolnir

} // end namespace valhalla
