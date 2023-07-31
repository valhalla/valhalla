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

namespace {
valhalla::gurka::map BuildPBF(const std::string& workdir) {
  const std::string ascii_map = R"(
      a-----b-----c---d
      A   B   C   D   E
    )";

  const gurka::nodes nodes = {
      {"A", {{"amenity", "bar"}}},
      {"B", {{"name", "hai di lao"}, {"amenity", "restaurant"}}},
      {"C", {{"name", "ke ji lu"}}}, // no amenity, shouldn't be stored
      {"D", {{"name", "wan da"}, {"amenity", "cinema"}}},
      {"E", {{"name", "zhong lou"}, {"amenity", "monument"}}}, // not in list, shouldn't be stored
      // non-landmark nodes
      {"a", {{"name", "gong ce"}, {"amenity", "toilets"}}},
      {"b", {{"name", "la ji tong"}, {"amenity", "waste_basket"}}},
      {"c", {{"name", "hua yuan"}, {"place", "city"}}},
      {"d", {{"traffic_signal", "signal"}}},
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

TEST(LandmarkTest, TestBuildDatabase) {
  const std::string db_path = "landmarks.db";

  // insert test data
  {
    LandmarkDatabase db_ini(db_path, false);

    ASSERT_NO_THROW(
        db_ini.insert_landmark("Statue of Liberty", LandmarkType::theatre, -74.044548, 40.689253));
    ASSERT_NO_THROW(db_ini.insert_landmark("Eiffel Tower", LandmarkType::cafe, 2.294481, 48.858370));
    ASSERT_NO_THROW(db_ini.insert_landmark("A", LandmarkType::bank, 40., 40.));
    ASSERT_NO_THROW(db_ini.insert_landmark("B", LandmarkType::fire_station, 30., 30.));
  }

  // test
  LandmarkDatabase db(db_path, true);

  std::vector<Landmark> landmarks{};
  EXPECT_NO_THROW({ landmarks = db.get_landmarks_by_bbox(30, 30, 40, 40); });

  EXPECT_EQ(landmarks.size(), 2); // A and B

  LOG_INFO("Get " + std::to_string(landmarks.size()) + " rows");
  for (const auto& landmark : landmarks) {
    LOG_INFO("id: " + std::to_string(std::get<0>(landmark)) + ", name: " + std::get<1>(landmark) +
             ", type: " + std::to_string(static_cast<uint8_t>(std::get<2>(landmark))) +
             ", longitude: " + std::to_string(std::get<3>(landmark)) +
             ", latitude: " + std::to_string(std::get<4>(landmark)));
  }

  landmarks.clear();
  EXPECT_NO_THROW({ landmarks = db.get_landmarks_by_bbox(0, 0, 50, 50); });

  EXPECT_EQ(landmarks.size(), 3); // A, B, Eiffel Tower
}

TEST(LandmarkTest, TestParseLandmarks) {
  const std::string workdir = "../data/landmarks";
  const std::string db_path = workdir + "/landmarks.sqlite";

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  // parse and store
  valhalla::gurka::map landmark_map = BuildPBF(workdir);
  boost::property_tree::ptree& pt = landmark_map.config;
  pt.put("mjolnir.landmarks", db_path);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

  EXPECT_TRUE(BuildLandmarkFromPBF(pt.get_child("mjolnir"), input_files));

  // check
  std::vector<Landmark> landmarks{};
  LandmarkDatabase db(db_path, true);

  EXPECT_NO_THROW({ landmarks = db.get_landmarks_by_bbox(-5, 0, 0, 10); });
  EXPECT_EQ(landmarks.size(), 3); // A, B, D

  LOG_INFO("Get " + std::to_string(landmarks.size()) + " rows");
  for (const auto& landmark : landmarks) {
    LOG_INFO("id: " + std::to_string(std::get<0>(landmark)) + ", name: " + std::get<1>(landmark) +
             ", type: " + std::to_string(static_cast<uint8_t>(std::get<2>(landmark))) +
             ", longitude: " + std::to_string(std::get<3>(landmark)) +
             ", latitude: " + std::to_string(std::get<4>(landmark)));
  }

  EXPECT_TRUE(std::get<2>(landmarks[0]) == LandmarkType::bar); // A
  EXPECT_TRUE(std::get<1>(landmarks[0]) == "A");
  EXPECT_TRUE(std::get<2>(landmarks[1]) == LandmarkType::restaurant); // B
  EXPECT_TRUE(std::get<1>(landmarks[1]) == "hai di lao");
  EXPECT_TRUE(std::get<2>(landmarks[2]) == LandmarkType::cinema); // D
  EXPECT_TRUE(std::get<1>(landmarks[2]) == "wan da");

  // check getting multiple landmarks by ids
  EXPECT_NO_THROW({ landmarks = db.get_landmarks_by_ids({1, 2, 3, 4}); });
  EXPECT_EQ(landmarks.size(), 3);

  EXPECT_NO_THROW({ landmarks = db.get_landmarks_by_ids({0, 937, 45, 15, 200, 353, 2386}); });
  EXPECT_EQ(landmarks.size(), 0);

  EXPECT_NO_THROW({ landmarks = db.get_landmarks_by_ids({2, 3, 5, 7, 11, 13, 17, 19, 23, 29}); });
  EXPECT_EQ(landmarks.size(), 2);

  EXPECT_NO_THROW({ landmarks = db.get_landmarks_by_ids({4}); });
  EXPECT_EQ(landmarks.size(), 0);

  EXPECT_NO_THROW({ landmarks = db.get_landmarks_by_ids({3, 2, 1}); });
  EXPECT_EQ(landmarks.size(), 3);
}
