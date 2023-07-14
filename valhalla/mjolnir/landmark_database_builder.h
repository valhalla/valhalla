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
  LandmarkDatabase(const std::string& db_name, bool read_only);
  ~LandmarkDatabase();
  void insert_landmark(const Landmark& landmark);
  std::vector<Landmark> get_landmarks_in_bounding_box(const double minLat,
                                                      const double minLong,
                                                      const double maxLat,
                                                      const double maxLong);

protected:
  struct db_pimpl;
  std::unique_ptr<db_pimpl> pimpl;

  sqlite3* db;
  sqlite3_stmt* insert_stmt;
  sqlite3_stmt* bounding_box_stmt;
  std::shared_ptr<void> spatial_lite;
  bool did_inserts = false;
};

} // namespace mjolnir

} // namespace valhalla
