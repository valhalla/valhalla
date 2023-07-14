#include <filesystem>
#include <gtest/gtest.h>

#include "gurka.h"
#include "mjolnir/landmark_database_builder.h"
#include "test/test.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

const std::string db_name = "landmarks.db";
const std::filesystem::path file_path = db_name;

class LandmarkDatabaseTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    LandmarkDatabase db(db_name, false);

    ASSERT_NO_THROW(db.insert_landmark(Landmark{"Statue of Liberty", "Monument", -74.044548, 40.689253}));
    ASSERT_NO_THROW(db.insert_landmark(Landmark{"Eiffel Tower", "Monument", 2.294481, 48.858370}));
    ASSERT_NO_THROW(db.insert_landmark(Landmark{"A", "pseudo", 5., 5.}));
    ASSERT_NO_THROW(db.insert_landmark(Landmark{"B", "pseudo", 10., 10.}));
  }

  static void TearDownTestSuite() {
    if (std::filesystem::remove(file_path)) {
      LOG_INFO("database deleted successfully");
    } else {
      LOG_ERROR("error deleting database");
    }
  }
};

TEST_F(LandmarkDatabaseTest, TestBoundingBoxQuery) {
  LandmarkDatabase db(db_name, true);

  std::vector<Landmark> landmarks = {};
  EXPECT_NO_THROW({ landmarks = db.get_landmarks_in_bounding_box(0, 0, 10, 10); });

  EXPECT_EQ(landmarks.size(), 2); // A and B

  LOG_INFO("Get " + std::to_string(landmarks.size()) + " rows");
  for (const auto& landmark : landmarks) {
    LOG_INFO("name: " + landmark.name + ", type: " + landmark.type + ", longitude: " +
             std::to_string(landmark.lng) + ", latitude: " + std::to_string(landmark.lat));
  }

  landmarks.clear();
  EXPECT_NO_THROW({ landmarks = db.get_landmarks_in_bounding_box(0, 0, 50, 50); });

  EXPECT_EQ(landmarks.size(), 3); // A, B, Eiffel Tower

  LOG_INFO("Get " + std::to_string(landmarks.size()) + " rows");
  for (const auto& landmark : landmarks) {
    LOG_INFO("name: " + landmark.name + ", type: " + landmark.type + ", longitude: " +
             std::to_string(landmark.lng) + ", latitude: " + std::to_string(landmark.lat));
  }
}