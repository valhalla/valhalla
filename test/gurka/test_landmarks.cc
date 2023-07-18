#include <filesystem>
#include <gtest/gtest.h>

#include "gurka.h"
#include "mjolnir/landmark_builder.h"
#include "test/test.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

const std::string db_name_test1 = "landmarks.db";
const std::filesystem::path file_path_test1 = db_name_test1;

namespace {
valhalla::gurka::map BuildPBF(const std::string& workdir) {
  const std::string ascii_map = R"(
      a-----b-----c---d
      A   B   C   D   E
    )";

  const gurka::nodes nodes = {
      {"A", {{"name", ""}, {"amenity", "university"}}},
      {"B", {{"name", "hai di lao"}, {"amenity", "restaurant"}}},
      {"C", {{"name", "ke ji lu"}, {"amenity", ""}}}, // no amenity, shouldn't be stored
      {"D", {{"name", "wan da"}, {"amenity", "cinema"}}},
      {"E", {{"name", "zhong lou"}, {"amenity", "monument"}}}, // not in list, shouldn't be stored
      // non-landmark nodes
      {"a", {{"name", "gong ce"}, {"amenity", ""}}},
      {"b", {{"name", "la ji tong"}}},
      {"c", {{"name", "hua yuan"}}},
      {"d", {{"name", ""}, {"amenity", ""}}},
  };

  const gurka::ways ways = {
      {"ab", {{"highway", "residential"}}},
      {"bc", {{"highway", "motorway"}}},
      {"cd", {{"highway", "residential"}, {"maxspeed", "60"}}},
  };

  constexpr double gridsize = 10000;
  auto node_layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);

  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(node_layout, ways, nodes, {}, pbf_filename, 0, false);

  valhalla::gurka::map result;
  result.nodes = node_layout;
  return result;
}
} // namespace

class LandmarkDatabaseTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { // build database and insert some test points
    LandmarkDatabase db(db_name_test1, false);

    ASSERT_NO_THROW(db.insert_landmark(
        Landmark{"Statue of Liberty", LandmarkType::theatre, -74.044548, 40.689253}));
    ASSERT_NO_THROW(
        db.insert_landmark(Landmark{"Eiffel Tower", LandmarkType::cafe, 2.294481, 48.858370}));
    ASSERT_NO_THROW(db.insert_landmark(Landmark{"A", LandmarkType::bank, 40., 40.}));
    ASSERT_NO_THROW(db.insert_landmark(Landmark{"B", LandmarkType::fire_station, 30., 30.}));
  }

  static void TearDownTestSuite() { // delete database
    if (std::filesystem::remove(file_path_test1)) {
      LOG_INFO("database deleted successfully");
    } else {
      LOG_ERROR("error deleting database");
    }
  }
};

TEST_F(LandmarkDatabaseTest, TestBuildDatabase) {
  LandmarkDatabase db(db_name_test1, true);

  std::vector<Landmark> landmarks{};
  EXPECT_NO_THROW({ landmarks = db.get_landmarks_in_bounding_box(30, 30, 40, 40); });

  EXPECT_EQ(landmarks.size(), 2); // A and B

  LOG_INFO("Get " + std::to_string(landmarks.size()) + " rows");
  for (const auto& landmark : landmarks) {
    LOG_INFO("name: " + landmark.name +
             ", type: " + std::to_string(static_cast<unsigned int>(landmark.type)) + ", longitude: " +
             std::to_string(landmark.lng) + ", latitude: " + std::to_string(landmark.lat));
  }

  landmarks.clear();
  EXPECT_NO_THROW({ landmarks = db.get_landmarks_in_bounding_box(0, 0, 50, 50); });

  EXPECT_EQ(landmarks.size(), 3); // A, B, Eiffel Tower
}

TEST(LandmarkTest, TestParseAndStoreLandmarks) {
  const std::string workdir = "test/data/landmarks";

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  // parse and store
  valhalla::gurka::map landmark_map = BuildPBF(workdir);
  boost::property_tree::ptree& pt = landmark_map.config;
  pt.put("mjolnir.landmarks", workdir + "/landmarks.sqlite");

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

  EXPECT_TRUE(BuildLandmarkFromPBF(pt.get_child("mjolnir"), input_files));

  // check
  std::vector<Landmark> landmarks{};
  LandmarkDatabase db(workdir + "/landmarks.sqlite", true);

  EXPECT_NO_THROW({ landmarks = db.get_landmarks_in_bounding_box(-5, 0, 0, 10); });
  EXPECT_EQ(landmarks.size(), 3); // A, B, D

  LOG_INFO("Get " + std::to_string(landmarks.size()) + " rows");
  for (const auto& landmark : landmarks) {
    LOG_INFO("name: " + landmark.name +
             ", type: " + std::to_string(static_cast<unsigned int>(landmark.type)) + ", longitude: " +
             std::to_string(landmark.lng) + ", latitude: " + std::to_string(landmark.lat));
  }

  EXPECT_TRUE(landmarks[0].type == LandmarkType::university); // A
  EXPECT_TRUE(landmarks[0].name == default_landmark_name);
  EXPECT_TRUE(landmarks[1].type == LandmarkType::restaurant); // B
  EXPECT_TRUE(landmarks[1].name == "hai di lao");
  EXPECT_TRUE(landmarks[2].type == LandmarkType::cinema); // D
  EXPECT_TRUE(landmarks[2].name == "wan da");

  // delete file
  const std::filesystem::path database_path = workdir + "/landmarks.sqlite";
  if (std::filesystem::remove(database_path)) {
    LOG_INFO("database deleted successfully");
  } else {
    LOG_ERROR("error deleting database");
  }
}
