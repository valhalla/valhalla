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

TEST(LandmarkDatabaseTest, TestDatabaseWithAccessMode) {
  std::string db_name = "landmarks-test-access-mode.db";

  LandmarkDatabase db(db_name, AccessMode::ReadWriteCreate);

  EXPECT_TRUE(db.insert_landmark("aaa", "bbb", 0., 0.));

  LandmarkDatabase readwrite_db(db_name, AccessMode::ReadWrite);

  EXPECT_TRUE(readwrite_db.create_landmarks_table());
  EXPECT_TRUE(readwrite_db.insert_landmark("hello", "world", 0., 0.));

  LandmarkDatabase readonly_db(db_name, AccessMode::ReadOnly);
  
  EXPECT_FALSE(readonly_db.insert_landmark("ccc", "ddd", 0., 0.));
  EXPECT_FALSE(readonly_db.test_create_test_table());
}

TEST(LandmarkDatabaseTest, TestBoundingBoxQuery) {
  std::string db_name = "landmarks-test-v2.db";

  // NOTE: should create db and insert data only once
  // LandmarkDatabase db(db_name, AccessMode::ReadWriteCreate);

  // ASSERT_TRUE(db.insert_landmark("Statue of Liberty", "Monument", -74.044548, 40.689253));
  // ASSERT_TRUE(db.insert_landmark("Eiffel Tower", "Monument", 2.294481, 48.858370));
  // ASSERT_TRUE(db.insert_landmark("A", "pseudo", 5., 5.));
  // ASSERT_TRUE(db.insert_landmark("B", "pseudo", 10., 10.));

  LandmarkDatabase db(db_name, AccessMode::ReadOnly);

  std::vector<std::pair<std::string, std::string>> landmarks = {};
  const double minLat = 0.;
  const double maxLat = 1.;
  const double minLong = 0.;
  const double maxLong = 1.;

  ASSERT_TRUE(db.get_landmarks_in_bounding_box(&landmarks, minLat, minLong, maxLat, maxLong));

  LOG_INFO(std::to_string(landmarks.size()));
  for (auto landmark: landmarks) {
    LOG_INFO("name: " + landmark.first + ", type: " + landmark.second);
  }

  EXPECT_TRUE(db.test_select_all());
}
