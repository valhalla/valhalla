#include "mjolnir/landmark_database_builder.h"
#include "mjolnir/util.h"

namespace valhalla {
namespace mjolnir {
void LandmarkDatabase::connect_database() {
  if (!open_database()) {
    throw std::runtime_error("Cannot open database");
    return;
  }

  if (open_flags == (SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE)) {
    if (!create_landmarks_table() || !create_spatial_index()) {
      throw std::runtime_error("Cannot create landmarks table or spatial index");
      return;
    }
  }

  if (!prepare_insert_stmt() || !prepare_bounding_box_stmt()) {
    throw std::runtime_error("Failed to prepare statements");
  }
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
  LOG_INFO("opened database and loaded Spatialite extension");

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

  LOG_INFO("created landmarks table");

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

  LOG_INFO("added geometry column");
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
  LOG_INFO("created spatial index");
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

bool LandmarkDatabase::insert_landmark(const Landmark& landmark) {
  sqlite3_reset(insert_stmt);
  sqlite3_clear_bindings(insert_stmt);

  if (landmark.name != "") {
    sqlite3_bind_text(insert_stmt, 1, landmark.name.c_str(), landmark.name.length(), SQLITE_STATIC);
  } else {
    sqlite3_bind_null(insert_stmt, 1);
  }

  if (landmark.type != "") {
    sqlite3_bind_text(insert_stmt, 2, landmark.type.c_str(), landmark.type.length(), SQLITE_STATIC);
  } else {
    sqlite3_bind_null(insert_stmt, 2);
  }

  sqlite3_bind_double(insert_stmt, 3, landmark.lng);
  sqlite3_bind_double(insert_stmt, 4, landmark.lat);

  LOG_INFO(sqlite3_expanded_sql(insert_stmt));

  ret = sqlite3_step(insert_stmt);
  if (ret == SQLITE_DONE || ret == SQLITE_ROW) {
    LOG_INFO("inserted landmark");
    did_inserts = true;
    return true;
  }

  LOG_ERROR("sqlite3_step() error: " + std::string(sqlite3_errmsg(db)));
  LOG_ERROR("sqlite3_step() Name: " + landmark.name);
  LOG_ERROR("sqlite3_step() Type: " + landmark.type);
  LOG_ERROR("sqlite3_step() longitude: " + std::to_string(landmark.lng));
  LOG_ERROR("sqlite3_step() Latitude: " + std::to_string(landmark.lat));

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

std::vector<Landmark> LandmarkDatabase::get_landmarks_in_bounding_box(const double minLat,
                                                                      const double minLong,
                                                                      const double maxLat,
                                                                      const double maxLong) {
  std::vector<Landmark> landmarks;

  sqlite3_reset(bounding_box_stmt);
  sqlite3_clear_bindings(bounding_box_stmt);

  sqlite3_bind_double(bounding_box_stmt, 1, minLong);
  sqlite3_bind_double(bounding_box_stmt, 2, minLat);
  sqlite3_bind_double(bounding_box_stmt, 3, maxLong);
  sqlite3_bind_double(bounding_box_stmt, 4, maxLat);

  LOG_INFO(sqlite3_expanded_sql(bounding_box_stmt));

  int ret = sqlite3_step(bounding_box_stmt);
  while (ret == SQLITE_ROW) {
    const char* name = reinterpret_cast<const char*>(sqlite3_column_text(bounding_box_stmt, 0));
    const char* type = reinterpret_cast<const char*>(sqlite3_column_text(bounding_box_stmt, 1));
    double lng = sqlite3_column_double(bounding_box_stmt, 2);
    double lat = sqlite3_column_double(bounding_box_stmt, 3);
    landmarks.emplace_back(Landmark{name, type, lng, lat});

    ret = sqlite3_step(bounding_box_stmt);
  }

  if (ret != SQLITE_DONE && ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(sqlite3_errmsg(db)));
    sqlite3_close(db);
    throw std::runtime_error("Failed to retrieve landmarks in bounding box");
  }

  return landmarks;
}

void LandmarkDatabase::close_database() {
  sqlite3_close(db);
  LOG_INFO("closed database");
}

bool LandmarkDatabase::vacuum_analyze() {
  sql = "VACUUM";
  ret = sqlite3_exec(db, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db);
    return false;
  }

  sql = "ANALYZE";
  ret = sqlite3_exec(db, sql.c_str(), NULL, NULL, &err_msg);
  if (ret != SQLITE_OK) {
    LOG_ERROR("Error: " + std::string(err_msg));
    sqlite3_free(err_msg);
    sqlite3_close(db);
    return false;
  }

  LOG_INFO("done vacuum and analyze");
  return true;
}

void LandmarkDatabase::release_prepared_stmt() {
  sqlite3_finalize(insert_stmt);
  sqlite3_finalize(bounding_box_stmt);
  LOG_INFO("released prepared statements");
}

} // end namespace mjolnir

} // end namespace valhalla
