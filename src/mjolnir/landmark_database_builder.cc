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
  std::string Landmark::get_name() {
    return _name;
  }
  std::string Landmark::get_type() {
    return _type;
  }
  std::string Landmark::get_latitude() {
    return std::to_string(_latitude);
  }
  std::string Landmark::get_longitude() {
    return std::to_string(_longitude);
  }

  bool LandmarkDatabase::openDatabase() {
    ret = sqlite3_open_v2(database.c_str(), &db, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
    if (ret != SQLITE_OK) {
      LOG_ERROR("cannot open " + database);
      sqlite3_close(db);
      return false;
    }
    // loading SpatiaLite as an extension
    auto db_conn = make_spatialite_cache(db);
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

    LOG_INFO("Created landmarks table.");

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

    LOG_INFO("Added geometry column.");

    return true;
  }

  bool LandmarkDatabase::insertLandmark(Landmark& landmark) {
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

    if (landmark.get_name() != "") {
      sqlite3_bind_text(stmt, 1, landmark.get_name().c_str(), landmark.get_name().length(), SQLITE_STATIC);
    } else {
      sqlite3_bind_null(stmt, 1);
    }

    if (landmark.get_type() != "") {
      sqlite3_bind_text(stmt, 2, landmark.get_type().c_str(), landmark.get_type().length(), SQLITE_STATIC);
    } else {
      sqlite3_bind_null(stmt, 2);
    }

    sqlite3_bind_text(stmt, 3, landmark.get_longitude().c_str(), landmark.get_longitude().length(), SQLITE_STATIC);
    sqlite3_bind_text(stmt, 4, landmark.get_latitude().c_str(), landmark.get_latitude().length(), SQLITE_STATIC);

    /* performing INSERT INTO */
    ret = sqlite3_step(stmt);
    if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
      sqlite3_finalize(stmt);
      return true;
    }

    LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db)));
    LOG_ERROR("sqlite3_step() Name: " + landmark.get_name());
    LOG_ERROR("sqlite3_step() Type: " + landmark.get_type());
    LOG_ERROR("sqlite3_step() longitute: " + landmark.get_longitude());
    LOG_ERROR("sqlite3_step() Latitude: " + landmark.get_latitude());

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
  }

}

}
