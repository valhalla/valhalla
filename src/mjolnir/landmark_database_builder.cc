#include "mjolnir/landmark_database_builder.h"
#include "filesystem.h"
#include "mjolnir/util.h"

namespace valhalla {
namespace mjolnir {

// TODO: this can be a utility and be more generic with a few more options, we could make the prepared
//  statements on the fly and retrievable by the caller, then anything in the code base that wants to
//  use sqlite can make use of this utility class. for now its ok to be specific to landmarks though
struct LandmarkDatabase::db_pimpl {
  sqlite3* db;
  sqlite3_stmt* insert_stmt;
  sqlite3_stmt* bounding_box_stmt;
  std::shared_ptr<void> spatial_lite;
  bool vacuum_analyze = false;

  db_pimpl(const std::string& db_name, bool read_only) : insert_stmt(nullptr) {
    // figure out if we need to create it or can just open it up
    auto flags = read_only ? SQLITE_OPEN_READONLY : SQLITE_OPEN_READWRITE;
    if (!filesystem::exists(db_name)) {
      if (read_only)
        throw std::logic_error("Cannot open sqlite database in read-only mode if it does not exist");
      flags |= SQLITE_OPEN_CREATE;
    }

    // get a connection to the database
    auto ret = sqlite3_open_v2(db_name.c_str(), &db, flags, NULL);
    if (ret != SQLITE_OK) {
      throw std::runtime_error("Failed to open sqlite database: " + db_name);
    }

    // loading spatiaLite as an extension
    spatial_lite = make_spatialite_cache(db);

    // if the db was empty we need to initialize the schema
    char* err_msg = nullptr;
    if (flags & SQLITE_OPEN_CREATE) {
      // make the table
      const char* table =
          "SELECT InitSpatialMetaData(1); CREATE TABLE IF NOT EXISTS landmarks (name TEXT, type TEXT)";
      ret = sqlite3_exec(db, table, NULL, NULL, &err_msg);
      if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
        throw std::runtime_error("Sqlite table creation error: " + std::string(err_msg));
      }

      // add geom column
      const char* geom = "SELECT AddGeometryColumn('landmarks', 'geom', 4326, 'POINT', 2)";
      ret = sqlite3_exec(db, geom, NULL, NULL, &err_msg);
      if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
        throw std::runtime_error("Sqlite geom column creation error: " + std::string(err_msg));
      }

      // make the index
      const char* index = "SELECT CreateSpatialIndex('landmarks', 'geom')";
      ret = sqlite3_exec(db, index, NULL, NULL, &err_msg);
      if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
        throw std::runtime_error("Sqlite spatial index creation error: " + std::string(err_msg));
      }

      // prep the insert statement
      const char* insert =
          "INSERT INTO landmarks (name, type, geom) VALUES (?, ?, MakePoint(?, ?, 4326))";
      ret = sqlite3_prepare_v2(db, insert, strlen(insert), &insert_stmt, NULL);
      if (ret != SQLITE_OK)
        throw std::runtime_error("Sqlite prepared insert statement error: " +
                                 std::string(sqlite3_errmsg(db)));
    }

    // prep the select statement
    const char* select =
        "SELECT name, type, X(geom), Y(geom) FROM landmarks WHERE ST_Covers(BuildMbr(?, ?, ?, ?, 4326), geom)";
    ret = sqlite3_prepare_v2(db, select, strlen(select), &bounding_box_stmt, NULL);
    if (ret != SQLITE_OK) {
      throw std::runtime_error("Sqlite prepared select statement error: " +
                               std::string(sqlite3_errmsg(db)));
    }
  }
  ~db_pimpl() {
    char* err_msg = nullptr;
    if (vacuum_analyze && sqlite3_exec(db, "VACUUM", NULL, NULL, &err_msg) != SQLITE_OK) {
      sqlite3_free(err_msg);
      LOG_ERROR("Sqlite vacuum error: " + std::string(err_msg));
    }

    if (vacuum_analyze && sqlite3_exec(db, "ANALYZE", NULL, NULL, &err_msg) != SQLITE_OK) {
      sqlite3_free(err_msg);
      LOG_ERROR("Sqlite analyze error: " + std::string(err_msg));
    }

    sqlite3_finalize(insert_stmt);
    sqlite3_finalize(bounding_box_stmt);
    sqlite3_close_v2(db);
  }
  std::string last_error() {
    return std::string(sqlite3_errmsg(db));
  }
};

LandmarkDatabase::LandmarkDatabase(const std::string& db_name, bool read_only)
    : pimpl(new db_pimpl(db_name, read_only)) {
}

void LandmarkDatabase::insert_landmark(const Landmark& landmark) {
  auto* insert_stmt = pimpl->insert_stmt;
  if (!insert_stmt)
    throw std::logic_error("Sqlite database connection is read-only");

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

  LOG_TRACE(sqlite3_expanded_sql(insert_stmt));
  if (sqlite3_step(insert_stmt) != SQLITE_DONE)
    throw std::runtime_error("Sqlite could not insert landmark: " + pimpl->last_error());
  pimpl->vacuum_analyze = true;
}

std::vector<Landmark> LandmarkDatabase::get_landmarks_in_bounding_box(const double minLat,
                                                                      const double minLong,
                                                                      const double maxLat,
                                                                      const double maxLong) {
  std::vector<Landmark> landmarks;

  auto* bounding_box_stmt = pimpl->bounding_box_stmt;
  sqlite3_reset(bounding_box_stmt);
  sqlite3_clear_bindings(bounding_box_stmt);

  sqlite3_bind_double(bounding_box_stmt, 1, minLong);
  sqlite3_bind_double(bounding_box_stmt, 2, minLat);
  sqlite3_bind_double(bounding_box_stmt, 3, maxLong);
  sqlite3_bind_double(bounding_box_stmt, 4, maxLat);

  LOG_TRACE(sqlite3_expanded_sql(bounding_box_stmt));

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
    throw std::runtime_error("Sqlite could not query landmarks in bounding box: " +
                             pimpl->last_error());
  }

  return landmarks;
}

} // end namespace mjolnir
} // end namespace valhalla
