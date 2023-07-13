#pragma once

//#include <boost/property_tree/ptree.hpp>
#include "mjolnir/util.h"

namespace valhalla {
namespace mjolnir {
enum class AccessMode { ReadWriteCreate, ReadOnly, ReadWrite };

struct Landmark {
  std::string name;
  std::string type;
  double lng;
  double lat;
};

struct LandmarkDatabase {
public:
  LandmarkDatabase(const std::string& db_name, AccessMode access_mode = AccessMode::ReadOnly)
      : db(nullptr), database(db_name), access_mode_(access_mode) {
    if (!connect_database()) {
      LOG_ERROR("cannot connect to database");
    }
  }
  ~LandmarkDatabase() {
    if (did_inserts && !vacuum_analyze()) {
      LOG_ERROR("cannot do vacuum and analyze");
    }
    close_database();
  }

  bool insert_landmark(const Landmark& landmark);

  bool get_landmarks_in_bounding_box(std::vector<Landmark>* landmarks,
                                     const double minLat,
                                     const double minLong,
                                     const double maxLat,
                                     const double maxLong);

protected:
  sqlite3* db;
  sqlite3_stmt* stmt;
  sqlite3_stmt* insert_stmt;
  sqlite3_stmt* bounding_box_stmt;
  uint32_t ret;
  char* err_msg = NULL;
  std::string sql;
  const std::string database;
  AccessMode access_mode_;
  int open_flags = 0;
  std::shared_ptr<void> db_conn;
  bool did_inserts = false;

  bool open_database();
  bool create_landmarks_table();
  bool create_spatial_index();
  void close_database();
  bool connect_database();
  bool prepare_insert_stmt();
  bool prepare_bounding_box_stmt();
  bool vacuum_analyze();
};

} // namespace mjolnir

} // namespace valhalla
