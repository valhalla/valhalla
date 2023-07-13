#pragma once

#include "filesystem.h"
#include "mjolnir/util.h"

namespace valhalla {
namespace mjolnir {

struct Landmark {
  std::string name;
  std::string type;
  double lng;
  double lat;
};

struct LandmarkDatabase {
public:
  LandmarkDatabase(const std::string& db_name, bool read_only) : db(nullptr), database(db_name) {
    if (!filesystem::exists(database)) {
      if (read_only) {
        throw std::runtime_error("invalid option");
      }
      open_flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE;
      connect_database();
    } else {
      open_flags = read_only ? SQLITE_OPEN_READONLY : SQLITE_OPEN_READWRITE;
      connect_database();
    }
  }

  ~LandmarkDatabase() {
    if (did_inserts && !vacuum_analyze()) {
      LOG_ERROR("cannot do vacuum and analyze");
    }
    release_prepared_stmt();
    close_database();
  }

  bool insert_landmark(const Landmark& landmark);

  std::vector<Landmark> get_landmarks_in_bounding_box(const double minLat,
                                                      const double minLong,
                                                      const double maxLat,
                                                      const double maxLong);

protected:
  sqlite3* db;
  sqlite3_stmt* insert_stmt;
  sqlite3_stmt* bounding_box_stmt;
  uint32_t ret;
  char* err_msg = NULL;
  std::string sql;
  const std::string database;
  int open_flags = 0;
  std::shared_ptr<void> db_conn;
  bool did_inserts = false;

  bool open_database();
  bool create_landmarks_table();
  bool create_spatial_index();
  void close_database();
  void connect_database();
  bool prepare_insert_stmt();
  bool prepare_bounding_box_stmt();
  bool vacuum_analyze();
  void release_prepared_stmt();
};

} // namespace mjolnir

} // namespace valhalla
