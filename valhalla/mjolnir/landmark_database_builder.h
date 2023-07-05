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
struct LandmarkDatabase {
public:
  LandmarkDatabase(std::string& db_name) : db(nullptr), database(db_name) {    
    if (!create_database()) { return; }
    if (!create_landmarks_table()) { return; } 
    if (!create_spatial_index()) { return; }

    LOG_INFO("Successfully set up landmark database");
  }
  ~LandmarkDatabase() {
    close_database();
  }

  bool open_readwrite_database();
  bool insert_landmark(const std::string& name, const std::string& type, 
                      const std::string& longitude, const std::string& latitude);
  
  bool get_landmarks_in_bounding_box(std::vector<std::pair<std::string, std::string>> *landmarks, 
       const std::string& minLat, const std::string& minLong, const std::string& maxLat, const std::string& maxLong);
  bool test_select_query();

private:
  sqlite3* db;
  sqlite3_stmt* stmt;
  uint32_t ret;
  char* err_msg = NULL;
  std::string sql;
  std::string database;

  bool create_database();
  bool create_landmarks_table();
  bool create_spatial_index();
  void close_database();
};

}

}
