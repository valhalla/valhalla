#pragma once

#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "baldr/graphconstants.h"
#include "filesystem.h"
#include "mjolnir/util.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/property_tree/ptree.hpp>

#include <geos_c.h>

namespace valhalla {
namespace mjolnir {
struct Landmark {
public:
  void initialize(const std::string& name, const std::string& type,
                  double latitude, double longitude) {
    _name = name;
    _type = type;
    _latitude = latitude;
    _longitude = longitude;
  }
  std::string get_name();
  std::string get_type();
  std::string get_latitude();
  std::string get_longitude();

private:
  std::string _name;
  std::string _type;
  double _latitude;
  double _longitude;
};

struct LandmarkDatabase {
public:
  LandmarkDatabase(std::string& db_name) : db(nullptr), database(db_name) {}
  bool openDatabase();
  bool createLandmarkTable();
  bool insertLandmark(Landmark& landmark);
  bool createSpatialIndex();
  void closeDatabase();
private:
  sqlite3* db;
  sqlite3_stmt* stmt;
  uint32_t ret;
  char* err_msg = NULL;
  std::string sql;
  std::string database;
};

}

}
