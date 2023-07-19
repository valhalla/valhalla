#include <filesystem>
#include <gtest/gtest.h>
#include <vector>

#include "gurka.h"
#include "mjolnir/landmark_builder.h"
#include "test/test.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

static const std::string db_path_test_build_database = "landmarks.db";

static const std::string workdir_test_parse_landmarks = "../data/landmarks";
static const std::string db_path_test_parse_landmarks =
    workdir_test_parse_landmarks + "/landmarks.sqlite";

static const std::vector<std::filesystem::path> db_paths{db_path_test_build_database,
                                                         db_path_test_parse_landmarks};

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

class LandmarkTest : public ::testing::Test {
protected:
  static void TearDownTestSuite() { // delete all databases
    for (auto db_path : db_paths) {
      LOG_INFO("deleting database: " + db_path.string());
      if (!std::filesystem::remove(db_path)) {
        LOG_ERROR("error deleting database");
      }
    }
  }
};

TEST_F(LandmarkTest, TestBuildDatabase) {
  // insert test data
  {
    LandmarkDatabase db_ini(db_path_test_build_database, false);

    ASSERT_NO_THROW(
        db_ini.insert_landmark("Statue of Liberty", LandmarkType::theatre, -74.044548, 40.689253));
    ASSERT_NO_THROW(db_ini.insert_landmark("Eiffel Tower", LandmarkType::cafe, 2.294481, 48.858370));
    ASSERT_NO_THROW(db_ini.insert_landmark("A", LandmarkType::bank, 40., 40.));
    ASSERT_NO_THROW(db_ini.insert_landmark("B", LandmarkType::fire_station, 30., 30.));
  }

  // test
  LandmarkDatabase db(db_path_test_build_database, true);

  std::vector<Landmark> landmarks{};
  EXPECT_NO_THROW({ landmarks = db.get_landmarks_in_bounding_box(30, 30, 40, 40); });

  EXPECT_EQ(landmarks.size(), 2); // A and B

  LOG_INFO("Get " + std::to_string(landmarks.size()) + " rows");
  for (const auto& landmark : landmarks) {
    LOG_INFO("name: " + std::get<0>(landmark) +
             ", type: " + std::to_string(static_cast<uint8_t>(std::get<1>(landmark))) +
             ", longitude: " + std::to_string(std::get<2>(landmark)) +
             ", latitude: " + std::to_string(std::get<3>(landmark)));
  }

  landmarks.clear();
  EXPECT_NO_THROW({ landmarks = db.get_landmarks_in_bounding_box(0, 0, 50, 50); });

  EXPECT_EQ(landmarks.size(), 3); // A, B, Eiffel Tower
}

TEST_F(LandmarkTest, TestParseLandmarks) {
  if (!filesystem::exists(workdir_test_parse_landmarks)) {
    bool created = filesystem::create_directories(workdir_test_parse_landmarks);
    EXPECT_TRUE(created);
  }

  // parse and store
  valhalla::gurka::map landmark_map = BuildPBF(workdir_test_parse_landmarks);
  boost::property_tree::ptree& pt = landmark_map.config;
  pt.put("mjolnir.landmarks", db_path_test_parse_landmarks);

  std::vector<std::string> input_files = {workdir_test_parse_landmarks + "/map.pbf"};

  EXPECT_TRUE(BuildLandmarkFromPBF(pt.get_child("mjolnir"), input_files));

  // check
  std::vector<Landmark> landmarks{};
  LandmarkDatabase db(db_path_test_parse_landmarks, true);

  EXPECT_NO_THROW({ landmarks = db.get_landmarks_in_bounding_box(-5, 0, 0, 10); });
  EXPECT_EQ(landmarks.size(), 3); // A, B, D

  LOG_INFO("Get " + std::to_string(landmarks.size()) + " rows");
  for (const auto& landmark : landmarks) {
    LOG_INFO("name: " + std::get<0>(landmark) +
             ", type: " + std::to_string(static_cast<uint8_t>(std::get<1>(landmark))) +
             ", longitude: " + std::to_string(std::get<2>(landmark)) +
             ", latitude: " + std::to_string(std::get<3>(landmark)));
  }

  EXPECT_TRUE(std::get<1>(landmarks[0]) == LandmarkType::university); // A
  EXPECT_TRUE(std::get<0>(landmarks[0]) == default_landmark_name);
  EXPECT_TRUE(std::get<1>(landmarks[1]) == LandmarkType::restaurant); // B
  EXPECT_TRUE(std::get<0>(landmarks[1]) == "hai di lao");
  EXPECT_TRUE(std::get<1>(landmarks[2]) == LandmarkType::cinema); // D
  EXPECT_TRUE(std::get<0>(landmarks[2]) == "wan da");
}
