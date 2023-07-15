#include <filesystem>
#include <gtest/gtest.h>

#include "gurka.h"
#include "mjolnir/landmark_builder.h"
#include "test/test.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

const std::string db_name = "landmarks.db";
const std::filesystem::path file_path = db_name;

namespace {
valhalla::gurka::map BuildPBF(const std::string& workdir) {
  const std::string ascii_map = R"(
      A-------B------C
    )";

  const gurka::nodes nodes = {
      {"A", {{"name", ""}, {"amenity", "university"}}},
      {"B", {{"name", "hai di lao"}, {"amenity", "restaurant"}}},
      {"C", {{"name", "ke ji lu"}, {"amenity", ""}}},
  };

  const gurka::ways ways = {
      {"AB", {}},
      {"BC", {}},
  };

  constexpr double gridsize = 100000;
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
  static void SetUpTestSuite() {
    LandmarkDatabase db(db_name, false);

    ASSERT_NO_THROW(
        db.insert_landmark(Landmark{"Statue of Liberty", LandmarkType::theatre, -74.044548, 40.689253}));
    ASSERT_NO_THROW(db.insert_landmark(Landmark{"Eiffel Tower", LandmarkType::cafe, 2.294481, 48.858370}));
    ASSERT_NO_THROW(db.insert_landmark(Landmark{"A", LandmarkType::bank, 5., 5.}));
    ASSERT_NO_THROW(db.insert_landmark(Landmark{"B", LandmarkType::null, 10., 10.}));
  }

  static void TearDownTestSuite() {
    if (std::filesystem::remove(file_path)) {
      LOG_INFO("database deleted successfully");
    } else {
      LOG_ERROR("error deleting database");
    }
  }
};

TEST_F(LandmarkDatabaseTest, TestBuildDatabase) {
  LandmarkDatabase db(db_name, true);

  std::vector<Landmark> landmarks{};
  EXPECT_NO_THROW({ landmarks = db.get_landmarks_in_bounding_box(0, 0, 10, 10); });

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

TEST_F(LandmarkDatabaseTest, TestParseAndStoreLandmarks) {
  // parse and store
  const std::string workdir = "test/data/landmark";

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map landmark_map = BuildPBF(workdir);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

  EXPECT_TRUE(BuildLandmarkFromPBF(input_files));

  // check
  std::vector<Landmark> landmarks{};
  LandmarkDatabase db(db_name, true);

  EXPECT_NO_THROW({ landmarks = db.get_landmarks_in_bounding_box(0, 0, 0, 20); });
  EXPECT_EQ(landmarks.size(), 3);

  LOG_INFO("Get " + std::to_string(landmarks.size()) + " rows");
  for (const auto& landmark : landmarks) {
    LOG_INFO("name: " + landmark.name +
             ", type: " + std::to_string(static_cast<unsigned int>(landmark.type)) + ", longitude: " +
             std::to_string(landmark.lng) + ", latitude: " + std::to_string(landmark.lat));
  }

  EXPECT_TRUE(landmarks[0].type == LandmarkType::university);
  EXPECT_TRUE(landmarks[0].name.empty());
  EXPECT_TRUE(landmarks[1].type == LandmarkType::restaurant);
  EXPECT_TRUE(landmarks[1].name == "hai di lao");
  EXPECT_TRUE(landmarks[2].type == LandmarkType::null);
  EXPECT_TRUE(landmarks[2].name == "ke ji lu");
}
