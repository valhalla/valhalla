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

TEST(LandmarkDatabaseTest, TestBoundingBoxQuery) {
  std::string db_name = "landmarks.db";

  // NOTE: should create db and insert data only once!
  // LandmarkDatabase db(db_name, AccessMode::ReadWriteCreate);

  // ASSERT_TRUE(db.insert_landmark("Statue of Liberty", "Monument", -74.044548, 40.689253));
  // ASSERT_TRUE(db.insert_landmark("Eiffel Tower", "Monument", 2.294481, 48.858370));
  // ASSERT_TRUE(db.insert_landmark("A", "pseudo", 5., 5.));
  // ASSERT_TRUE(db.insert_landmark("B", "pseudo", 10., 10.));

  LandmarkDatabase db(db_name, AccessMode::ReadOnly);

  std::vector<Landmark> landmarks = {};
  const double minLat = 0.;
  const double maxLat = 10.;
  const double minLong = 0.;
  const double maxLong = 10.;

  ASSERT_TRUE(db.get_landmarks_in_bounding_box(&landmarks, minLat, minLong, maxLat, maxLong));

  LOG_INFO("Get " + std::to_string(landmarks.size()) + " rows");
  for (auto landmark : landmarks) {
    LOG_INFO("name: " + landmark.name + ", type: " + landmark.type + "longitude: " +
             std::to_string(landmark.lng) + ", latitude: " + std::to_string(landmark.lat));
  }
  // selected rows should only include A and B
}
