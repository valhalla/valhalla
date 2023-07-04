#include <gtest/gtest.h>

#include "filesystem.h"
#include "gurka.h"
#include "mjolnir/landmark_database_builder.h"
#include "test/test.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

TEST(LandmarkTest, TestBuildLandmarkStorage) {
  std::string db_name = "landmarks.db";
  LandmarkDatabase db(db_name);

  ASSERT_TRUE(db.openDatabase());
  ASSERT_TRUE(db.createLandmarkTable());

  // Landmark landmark1, landmark2;
  // landmark1.initialize("Statue of Liberty", "Monument", -74.044548, 40.689253);
  // landmark2.initialize("Eiffel Tower", "Monument", 2.294481, 48.858370);

  ASSERT_TRUE(db.insertLandmark("Statue of Liberty", "Monument", "-74.044548", "40.689253"));
  ASSERT_TRUE(db.insertLandmark("Eiffel Tower", "Monument", "2.294481", "48.858370"));
  
  ASSERT_TRUE(db.createSpatialIndex());

  db.closeDatabase();
}