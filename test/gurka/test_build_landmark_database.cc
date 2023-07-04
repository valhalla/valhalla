#include <gtest/gtest.h>
#include <string>

#include "filesystem.h"
#include "gurka.h"
#include "mjolnir/landmark_database_builder.h"
#include "test/test.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

TEST(LandmarkTest, TestBuildLandmarkStorage) {
  std::string db_name = "landmarks-v4.db";
  LandmarkDatabase db(db_name);

  ASSERT_TRUE(db.openDatabase());
  ASSERT_TRUE(db.createLandmarkTable());

  // NOTE: should insert same landmarks only once

  // ASSERT_TRUE(db.insertLandmark("Statue of Liberty", "Monument", "-74.044548", "40.689253"));
  // ASSERT_TRUE(db.insertLandmark("Eiffel Tower", "Monument", "2.294481", "48.858370"));
  // ASSERT_TRUE(db.insertLandmark("A", "pseudo", "5", "5"));
  // ASSERT_TRUE(db.insertLandmark("B", "pseudo", "6", "6"));
  
  ASSERT_TRUE(db.createSpatialIndex());

  std::vector<std::pair<std::string, std::string>> landmarks = {};
  const std::string minLat = "0";
  const std::string maxLat = "0";
  const std::string minLong = "0";
  const std::string maxLong = "0";

  ASSERT_TRUE(db.testSelectQuery());

  // ASSERT_TRUE(db.getLandmarksInBoundingBox(&landmarks, minLat, minLong, maxLat, maxLong));

  LOG_INFO(std::to_string(landmarks.size()));
  for (auto landmark: landmarks) {
    LOG_INFO("name: " + landmark.first + ", type: " + landmark.second);
  }

  db.closeDatabase();
}