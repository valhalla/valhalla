#include <filesystem>
#include <gtest/gtest.h>
#include <vector>

#include "baldr/graphreader.h"
#include "baldr/landmark.h"
#include "gurka.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/landmarks.h"
#include "test/test.h"

#include <boost/property_tree/ptree.hpp>
#include <iomanip>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

// config for the first three tests
const std::string workdir = VALHALLA_BUILD_DIR "test/data/landmarks/landmarks";
const std::string db_path = workdir + "/landmarks.sqlite";
const std::string pbf_filename = workdir + "/map.pbf";
valhalla::gurka::map landmark_map;

// config for TestAddLandmarksToTiles
const std::string workdir_tiles = VALHALLA_BUILD_DIR "test/data/landmarks/landmarks_tile";
const std::string db_path_tile_test = workdir_tiles + "/landmarks.sqlite";
const std::string pbf_filename_tile_test = workdir_tiles + "/map.pbf";
valhalla::gurka::map landmark_map_tile_test;

namespace {
void BuildPBF() {
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

  constexpr double gridsize = 100;
  landmark_map.nodes = gurka::detail::map_to_coordinates(ascii_map, gridsize, {-.01, 0});

  detail::build_pbf(landmark_map.nodes, ways, nodes, {}, pbf_filename, 0, false);
}

void BuildPBFAddLandmarksToTiles() {
  const std::string ascii_map = R"(
      A B               E   
      a------b-----c----d         K
      |        C   | D
  F   |            |  
      |            |
      |            |
      |      G     |
      |            |
      |            |
      |            |
      |            |
      |            |    H
      e------------f
      I                           J
    )";

  const gurka::nodes nodes = {
      {"A", {{"name", "lv_mo_li"}, {"amenity", "bar"}}},
      {"B", {{"name", "hai_di_lao"}, {"amenity", "restaurant"}}},
      {"C", {{"name", "McDonalds"}, {"amenity", "fast_food"}}},
      {"D", {{"name", "wan_da"}, {"amenity", "cinema"}}},
      {"E", {{"name", "sheng_ren_min_yi_yuan"}, {"amenity", "hospital"}}},
      {"F", {{"name", "gong_shang_yin_hang"}, {"amenity", "bank"}}},
      {"G", {{"name", "ju_yuan"}, {"amenity", "theatre"}}},
      {"H", {{"name", "pizza_hut"}, {"amenity", "restaurant"}}},
      {"I", {{"name", "shell"}, {"amenity", "fuel"}}},
      {"J", {{"name", "starbucks"}, {"amenity", "cafe"}}},
      {"K", {{"name", "you_zheng"}, {"amenity", "post_office"}}},
      // non-landmark nodes
      {"a", {{"junction", "yes"}, {"name", "you_yi_lu_kou"}}},
      {"b", {{"traffic_signal", "signal"}}},
      {"c", {{"junction", "yes"}, {"name", "gao_xin_lu_kou"}}},
      {"d", {{"barrier", "gate"}}},
      {"e", {{"name", "hua_yuan"}, {"place", "city"}}},
      {"f", {{"name", "gong_ce"}, {"amenity", "toilets"}}},
  };

  const gurka::ways ways = {
      {"ae", // length 55, associated with ABFI
       {{"highway", "primary"}, {"name", "G999"}, {"driving_side", "right"}, {"maxspeed", "120"}}},
      {"ab", // length 35, associated with ABCFG
       {{"highway", "secondary"}, {"name", "S1"}, {"driving_side", "right"}}},
      {"bc", // length 30, associated with CDG
       {{"highway", "secondary"}, {"name", "S2"}, {"lanes", "2"}, {"driving_side", "right"}}},
      {"cd", {{"highway", "motorway"}, {"maxspeed", "100"}}},   // length 25, associated with CDEK
      {"cf", {{"highway", "residential"}, {"maxspeed", "30"}}}, // length 55, associated with CDH
      {"ef", {{"highway", "residential"}, {"maxspeed", "30"}}}, // length 65, associated with I
  };

  constexpr double gridsize = 5;
  landmark_map_tile_test.nodes = gurka::detail::map_to_coordinates(ascii_map, gridsize, {0, 0});

  detail::build_pbf(landmark_map_tile_test.nodes, ways, nodes, {}, pbf_filename_tile_test, 0, false);
}

void CheckLandmarksInTiles(GraphReader& reader, const GraphId& graphid) {
  LOG_INFO("Checking tiles of level " + std::to_string(graphid.level()) + "...");

  auto tile = reader.GetGraphTile(graphid);
  for (const auto& e : tile->GetDirectedEdges()) {
    auto ei = tile->edgeinfo(&e);
    auto tagged_values = ei.GetTags();

    // LOG_INFO("edge endnode: " + std::to_string(e.endnode().id()) +
    //          ", length: " + std::to_string(e.length()));

    int count_landmarks = 0;
    for (const auto& value : tagged_values) {
      if (value.first != baldr::TaggedValue::kLandmark)
        continue;

      count_landmarks++;
      // Landmark landmark(value.second);
      // std::cout << landmark.name << " " << static_cast<int>(landmark.type) << " " << landmark.lng
      //           << " " << landmark.lat << std::endl;
    }

    switch (graphid.level()) {
      case 0: // edge ae, cd
        EXPECT_EQ(count_landmarks, 4);
        break;
      case 1:
        if (e.length() == 35) { // ab
          EXPECT_EQ(count_landmarks, 5);
        } else if (e.length() == 30) { // bc
          EXPECT_EQ(count_landmarks, 3);
        }
        break;
      case 2:
        if (e.length() == 55) { // cf
          EXPECT_EQ(count_landmarks, 3);
        } else if (e.length() == 65) { // ef
          EXPECT_EQ(count_landmarks, 1);
        }
        break;
    }
  }
}

void DisplayLandmarksInTiles(GraphReader& reader, const GraphId& graphid) {
  LOG_INFO("Checking tiles of level " + std::to_string(graphid.level()) + "...");

  auto tile = reader.GetGraphTile(graphid);
  for (const auto& e : tile->GetDirectedEdges()) {
    auto ei = tile->edgeinfo(&e);
    auto tagged_values = ei.GetTags();

    LOG_INFO("edge endnode: " + std::to_string(e.endnode().id()) +
             ", length: " + std::to_string(e.length()));

    int count_landmarks = 0;
    for (const auto& value : tagged_values) {
      if (value.first != baldr::TaggedValue::kLandmark)
        continue;

      count_landmarks++;
      Landmark landmark(value.second);
      std::cout << landmark.name << " " << static_cast<int>(landmark.type) << " " << landmark.lng
                << " " << landmark.lat << std::endl;
    }
  }
}
} // namespace

TEST(LandmarkTest, TestBuildDatabase) {

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
    LOG_INFO("id: " + std::to_string(landmark.id) + ", name: " + landmark.name +
             ", type: " + std::to_string(static_cast<uint8_t>(landmark.type)) + ", longitude: " +
             std::to_string(landmark.lng) + ", latitude: " + std::to_string(landmark.lat));
  }

  landmarks.clear();
  EXPECT_NO_THROW({ landmarks = db.get_landmarks_by_bbox(0, 0, 50, 50); });

  EXPECT_EQ(landmarks.size(), 3); // A, B, Eiffel Tower
}

TEST(LandmarkTest, TestParseLandmarks) {

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  // parse and store
  BuildPBF();
  boost::property_tree::ptree& pt = landmark_map.config;
  pt.put("mjolnir.landmarks", db_path);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

  EXPECT_TRUE(BuildLandmarkFromPBF(pt.get_child("mjolnir"), input_files));

  // check
  std::vector<Landmark> landmarks{};
  LandmarkDatabase db(db_path, true);

  EXPECT_NO_THROW({ landmarks = db.get_landmarks_by_bbox(-.1, -5, 10, 0); });
  EXPECT_EQ(landmarks.size(), 3); // A, B, D

  LOG_INFO("Get " + std::to_string(landmarks.size()) + " rowsmjolnir");
  for (const auto& landmark : landmarks) {
    LOG_INFO("id: " + std::to_string(landmark.id) + ", name: " + landmark.name +
             ", type: " + std::to_string(static_cast<uint8_t>(landmark.type)) + ", longitude: " +
             std::to_string(landmark.lng) + ", latitude: " + std::to_string(landmark.lat));
  }

  EXPECT_TRUE(landmarks[0].type == LandmarkType::bar); // A
  EXPECT_TRUE(landmarks[0].name == "A");
  EXPECT_TRUE(landmarks[1].type == LandmarkType::restaurant); // B
  EXPECT_TRUE(landmarks[1].name == "hai di lao");
  EXPECT_TRUE(landmarks[2].type == LandmarkType::cinema); // D
  EXPECT_TRUE(landmarks[2].name == "wan da");

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

TEST(LandmarkTest, TestTileStoreLandmarks) {
  BuildPBF();
  landmark_map.config =
      test::make_config(workdir, {{"mjolnir.landmarks_db", db_path}},
                        {{"additional_data", "mjolnir.traffic_extract", "mjolnir.tile_extract"}});

  // build regular graph tiles from the pbf that we have already made, there wont be landmarks in them
  mjolnir::build_tile_set(landmark_map.config, {pbf_filename}, mjolnir::BuildStage::kInitialize,
                          mjolnir::BuildStage::kValidate, false);

  // load one of the graphtiles
  GraphId tile_id("2/519119/0");
  GraphTileBuilder tb(workdir, tile_id, true);

  auto invalid_landmark = static_cast<uint32_t>(LandmarkType::casino) + 1;
  uint32_t edge_index = 0;

  // add flexible landmarks for the edges
  for (const auto& e : tb.directededges()) {
    std::vector<PointLL> shape = tb.edgeinfo(&e).shape();
    auto point = shape[shape.size() / 2];
    auto ltype = static_cast<LandmarkType>((edge_index + 1) % invalid_landmark);

    Landmark landmark(edge_index, std::to_string(edge_index), ltype, point.first, point.second);

    auto edge_id = tile_id;
    edge_id.set_id(edge_index++);

    tb.AddLandmark(edge_id, landmark);
  }

  tb.StoreTileData();

  // instantiate a graphreader using the config
  GraphReader gr(landmark_map.config.get_child("mjolnir"));
  auto tile = gr.GetGraphTile(tile_id);

  // we support up to 6 decimal precision for landmark lng/lat, so max rounding error is 5e-7
  const double rounding_error = 5e-7;

  for (const auto& e : tile->GetDirectedEdges()) {
    auto ei = tile->edgeinfo(&e);

    // test EdgeInfo:GetTags
    auto tagged_values = ei.GetTags();

    edge_index = 0;
    for (const auto& value : tagged_values) {
      if (value.first != baldr::TaggedValue::kLandmark)
        continue;

      Landmark landmark(value.second);

      // check data correctness
      std::vector<PointLL> shape = ei.shape();
      auto point = shape[shape.size() / 2];
      auto ltype = static_cast<LandmarkType>((edge_index + 1) % invalid_landmark);

      EXPECT_EQ(landmark.id, 0);
      EXPECT_EQ(landmark.name, std::to_string(edge_index++));
      EXPECT_EQ(landmark.type, ltype);
      EXPECT_NEAR(landmark.lng, point.first, rounding_error);
      EXPECT_NEAR(landmark.lat, point.second, rounding_error);
    }

    // test EdgeInfo:GetTaggedValues
    auto values = ei.GetTaggedValues();
    edge_index = 0;
    for (const std::string& v : values) {
      if (static_cast<baldr::TaggedValue>(v[0]) != baldr::TaggedValue::kLandmark) {
        continue;
      }

      Landmark landmark(v.substr(1));

      // check data correctness
      std::vector<PointLL> shape = ei.shape();
      auto point = shape[shape.size() / 2];
      auto ltype = static_cast<LandmarkType>((edge_index + 1) % invalid_landmark);

      EXPECT_EQ(landmark.id, 0);
      EXPECT_EQ(landmark.name, std::to_string(edge_index++));
      EXPECT_EQ(landmark.type, ltype);
      EXPECT_NEAR(landmark.lng, point.first, rounding_error);
      EXPECT_NEAR(landmark.lat, point.second, rounding_error);
    }
  }
}

TEST(LandmarkTest, TestAddLandmarksToTiles) {
  if (!filesystem::exists(workdir_tiles)) {
    bool created = filesystem::create_directories(workdir_tiles);
    EXPECT_TRUE(created);
  }

  BuildPBFAddLandmarksToTiles();

  landmark_map_tile_test.config =
      test::make_config(workdir_tiles, {{"mjolnir.landmarks_db", db_path_tile_test}},
                        {{"additional_data", "mjolnir.traffic_extract", "mjolnir.tile_extract"}});

  // build regular graph tiles from the pbf that we have already made, there wont be landmarks in them
  mjolnir::build_tile_set(landmark_map_tile_test.config, {pbf_filename_tile_test},
                          mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate, false);

  // build landmark database and parse landmarks
  EXPECT_TRUE(BuildLandmarkFromPBF(landmark_map_tile_test.config.get_child("mjolnir"),
                                   {pbf_filename_tile_test}));

  // add landmarks from db to tiles
  AddLandmarks(landmark_map_tile_test.config);

  // check data
  GraphReader gr(landmark_map_tile_test.config.get_child("mjolnir"));

  CheckLandmarksInTiles(gr, GraphId("0/002025/0"));
  CheckLandmarksInTiles(gr, GraphId("1/032220/0"));
  CheckLandmarksInTiles(gr, GraphId("2/517680/0"));
}

// TODO: This is an example test to show the underlying bugs or problems in the current code.
// Right now the problem is we cannot add landmarks to a graph tile twice.
// The test will return "[ERROR] Failed to build GraphTile. Error: GraphId level exceeds tile
// hierarchy max level", and "Could not compute FileSuffix for GraphId with invalid tile
// id:0/245760/0". We need to fix it in the future.
TEST(LandmarkTest, DISABLED_ErrorTest) {
  if (!filesystem::exists(workdir_tiles)) {
    bool created = filesystem::create_directories(workdir_tiles);
    EXPECT_TRUE(created);
  }

  BuildPBFAddLandmarksToTiles();

  landmark_map_tile_test.config =
      test::make_config(workdir_tiles, {{"mjolnir.landmarks_db", db_path_tile_test}},
                        {{"additional_data", "mjolnir.traffic_extract", "mjolnir.tile_extract"}});

  // build regular graph tiles from the pbf that we have already made, there wont be landmarks in them
  mjolnir::build_tile_set(landmark_map_tile_test.config, {pbf_filename_tile_test},
                          mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate, false);

  // build landmark database and parse landmarks
  EXPECT_TRUE(BuildLandmarkFromPBF(landmark_map_tile_test.config.get_child("mjolnir"),
                                   {pbf_filename_tile_test}));

  // add landmarks from db to tiles
  AddLandmarks(landmark_map_tile_test.config);
  // add again, but this will result in errors
  AddLandmarks(landmark_map_tile_test.config);

  // check data (cannot reach here yet)
  GraphReader gr(landmark_map_tile_test.config.get_child("mjolnir"));

  DisplayLandmarksInTiles(gr, GraphId("0/002025/0"));
  DisplayLandmarksInTiles(gr, GraphId("1/032220/0"));
  DisplayLandmarksInTiles(gr, GraphId("2/517680/0"));
}
