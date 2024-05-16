#include <filesystem>
#include <gtest/gtest.h>
#include <iomanip>
#include <vector>

#include "baldr/graphreader.h"
#include "baldr/landmark.h"
#include "gurka.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/landmarks.h"
#include "odin/enhancedtrippath.h"
#include "test/test.h"

#include <boost/property_tree/ptree.hpp>

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
inline void DisplayLandmark(const Landmark& landmark) {
  std::cout << "landmark: id = " << landmark.id << ", name = " << landmark.name
            << ", type = " << static_cast<int>(landmark.type) << ", lng = " << landmark.lng
            << ", lat = " << landmark.lat << std::endl;
}

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

  const std::map<std::pair<int, int>, std::vector<std::string>> expected_landmarks_tiles = {
      {{0, 55},
       std::vector<std::string>{"gong_shang_yin_hang", "hai_di_lao", "lv_mo_li", "shell"}}, // ae
      {{0, 25},
       std::vector<std::string>{"McDonalds", "sheng_ren_min_yi_yuan", "wan_da", "you_zheng"}}, // cd
      {{1, 35},
       std::vector<std::string>{"McDonalds", "gong_shang_yin_hang", "hai_di_lao", "ju_yuan",
                                "lv_mo_li"}},                                  // ab
      {{1, 30}, std::vector<std::string>{"McDonalds", "ju_yuan", "wan_da"}},   // bc
      {{2, 55}, std::vector<std::string>{"McDonalds", "pizza_hut", "wan_da"}}, // cf
      {{2, 65}, std::vector<std::string>{"shell"}},                            // ef
  };

  auto tile = reader.GetGraphTile(graphid);
  for (const auto& e : tile->GetDirectedEdges()) {
    if (e.is_shortcut()) {
      continue;
    }
    auto ei = tile->edgeinfo(&e);
    auto tagged_values = ei.GetTags();

    std::vector<std::string> landmark_names{};
    for (const auto& value : tagged_values) {
      if (value.first != baldr::TaggedValue::kLandmark)
        continue;
      landmark_names.push_back(Landmark(value.second).name);
    }

    // use graph level and edge length as map key
    const auto expected_result = expected_landmarks_tiles.find({graphid.level(), e.length()});
    if (expected_result == expected_landmarks_tiles.end()) {
      throw std::runtime_error("Failed to find the edge in the expected landmarks, level = " +
                               std::to_string(graphid.level()) +
                               ", edge length = " + std::to_string(e.length()));
    }

    std::sort(landmark_names.begin(), landmark_names.end());
    EXPECT_EQ(expected_result->second, landmark_names);
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

    for (const auto& value : tagged_values) {
      if (value.first != baldr::TaggedValue::kLandmark)
        continue;
      DisplayLandmark(Landmark(value.second));
    }
  }
}

// struct to represent a landmark in maneuvers in test
struct LandmarkInManeuver {
  std::string _name;
  uint8_t _type;
  double _distance;
  bool _right;

  LandmarkInManeuver(const std::string& name, uint8_t type, double distance, bool right)
      : _name(name), _type(type), _distance(distance), _right(right) {
  }

  bool operator<(const LandmarkInManeuver& other) const {
    if (_name != other._name) {
      return _name < other._name;
    } else {
      return _distance < other._distance;
    }
  }

  bool operator==(const LandmarkInManeuver& other) const {
    if (_name == other._name && _type == other._type && _distance == other._distance &&
        _right == other._right) {
      return true;
    }
    return false;
  }
};

bool operator==(const Landmark& a, const Landmark& b) {
  return a.id == b.id && a.name == b.name && a.type == b.type &&
         PointLL(a.lng, a.lat).ApproximatelyEqual(PointLL(b.lng, b.lat));
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

  landmarks.clear();
  EXPECT_NO_THROW({ landmarks = db.get_landmarks_by_bbox(0, 0, 50, 50); });
  EXPECT_EQ(landmarks.size(), 3); // A, B, Eiffel Tower
}

TEST(LandmarkTest, TestParseLandmarks) {
  if (!std::filesystem::exists(workdir)) {
    bool created = std::filesystem::create_directories(workdir);
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

  // test round trip encoding
  for (auto& expected : landmarks) {
    expected.id = 0;
    Landmark actual(expected.to_str());
    EXPECT_TRUE(expected == actual);
  }
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
  GraphId edge_id = tile_id;
  GraphTileBuilder tb(workdir, tile_id, true);

  auto invalid_landmark = static_cast<uint32_t>(LandmarkType::casino) + 1;
  // add flexible landmarks for the edges
  for (const auto& e : tb.directededges()) {
    std::vector<PointLL> shape = tb.edgeinfo(&e).shape();
    auto point = shape[shape.size() / 2];
    auto ltype = static_cast<LandmarkType>((edge_id.id() + 1) % invalid_landmark);
    Landmark landmark(0, std::to_string(edge_id), ltype, point.first, point.second);
    tb.AddLandmark(edge_id, landmark);
    ++edge_id;
  }

  tb.StoreTileData();

  // instantiate a graphreader using the config
  GraphReader gr(landmark_map.config.get_child("mjolnir"));
  auto tile = gr.GetGraphTile(tile_id);

  auto check_landmark = [&](const auto& landmark, const auto& point) {
    auto ltype = static_cast<LandmarkType>((edge_id.id() + 1) % invalid_landmark);

    const double rounding_error = 5e-7;
    // we dont store this in the tile
    EXPECT_EQ(landmark.id, 0);
    // opposing edges will have a copy of the edge we associated it to so we need to check it
    bool skip = false;
    if (landmark.name != std::to_string(edge_id)) {
      auto opp = gr.GetOpposingEdgeId(edge_id);
      ASSERT_EQ(landmark.name, std::to_string(opp)) << "Unexpected landmark name or edge/opp_edge";
      skip = true;
    }
    // we dont do deep check on opposing edge records
    if (!skip) {
      EXPECT_EQ(landmark.type, ltype);
      EXPECT_NEAR(landmark.lng, point.first, rounding_error);
      EXPECT_NEAR(landmark.lat, point.second, rounding_error);
    }
  };

  // we support up to 6 decimal precision for landmark lng/lat, so max rounding error is 5e-7

  edge_id.set_id(0);
  for (const auto& e : tile->GetDirectedEdges()) {
    auto ei = tile->edgeinfo(&e);

    // test EdgeInfo:GetTags
    auto tagged_values = ei.GetTags();
    for (const auto& value : tagged_values) {
      if (value.first != baldr::TaggedValue::kLandmark)
        continue;
      Landmark landmark(value.second);

      // check data correctness
      std::vector<PointLL> shape = ei.shape();
      auto point = shape[shape.size() / 2];
      check_landmark(landmark, point);
    }

    // test EdgeInfo:GetTaggedValues
    auto values = ei.GetTaggedValues();
    for (const std::string& v : values) {
      if (static_cast<baldr::TaggedValue>(v[0]) != baldr::TaggedValue::kLandmark) {
        continue;
      }
      Landmark landmark(v.substr(1));

      // check data correctness
      std::vector<PointLL> shape = ei.shape();
      auto point = shape[shape.size() / 2];
      check_landmark(landmark, point);
    }

    ++edge_id;
  }
}

TEST(LandmarkTest, TestAddLandmarksToTiles) {
  if (!std::filesystem::exists(workdir_tiles)) {
    bool created = std::filesystem::create_directories(workdir_tiles);
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
  if (!std::filesystem::exists(workdir_tiles)) {
    bool created = std::filesystem::create_directories(workdir_tiles);
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

TEST(LandmarkTest, TestLandmarksInManeuvers) {
  const std::string ascii_map = R"(
    A B           C         D 
    a-------b-------c------d
          E |    F  |G
            |       |
            e       |    I   J
              H     f-------g
                      K
  )";

  const gurka::nodes nodes = {
      // landmarks
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
      {"g", {{"barrier", "gate"}, {"access", "yes"}}},
  };

  const gurka::ways ways = {
      {"ab", {{"highway", "secondary"}, {"name", "S1"}, {"maxspeed", "80"}}},
      {"bc", {{"highway", "secondary"}, {"name", "S2"}, {"driving_side", "right"}}},
      {"cd", {{"highway", "secondary"}, {"lanes", "2"}, {"driving_side", "right"}}},
      {"be", {{"highway", "secondary"}, {"maxspeed", "60"}}},
      {"cf", {{"highway", "secondary"}, {"maxspeed", "50"}}},
      {"fg", {{"highway", "secondary"}, {"maxspeed", "50"}}},
  };

  const std::string workdir = VALHALLA_BUILD_DIR "test/data/landmarks/maneuvers";
  const std::string db_path = workdir + "/landmarks.sqlite";
  const std::string pbf = workdir + "/map.pbf";

  if (!std::filesystem::exists(workdir)) {
    bool created = std::filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map{};
  map.nodes = gurka::detail::map_to_coordinates(ascii_map, 10, {0, 0});
  detail::build_pbf(map.nodes, ways, nodes, {}, pbf, 0, false);

  map.config =
      test::make_config(workdir, {{"mjolnir.landmarks_db", db_path}},
                        {{"additional_data", "mjolnir.traffic_extract", "mjolnir.tile_extract"}});

  // build regular graph tiles from the pbf, and add landmarks to it
  mjolnir::build_tile_set(map.config, {pbf}, mjolnir::BuildStage::kInitialize,
                          mjolnir::BuildStage::kValidate, false);
  // build landmark database and import landmarks to it
  EXPECT_TRUE(BuildLandmarkFromPBF(map.config.get_child("mjolnir"), {pbf}));
  // add landmarks to graphtile from the landmark database
  AddLandmarks(map.config);

  // get routing result from point a to g
  auto result = gurka::do_action(valhalla::Options::route, map, {"a", "g"}, "auto");

  ASSERT_EQ(result.trip().routes_size(), 1);
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);

  const std::map<std::string, std::vector<std::string>> expected_landmarks_tripleg = {
      {"S1", std::vector<std::string>{"hai_di_lao", "lv_mo_li", "sheng_ren_min_yi_yuan"}},
      {"S2", std::vector<std::string>{"McDonalds", "gong_shang_yin_hang", "ju_yuan",
                                      "sheng_ren_min_yi_yuan"}},
      {"cf", std::vector<std::string>{"McDonalds", "ju_yuan", "you_zheng"}},
      {"fg", std::vector<std::string>{"shell", "starbucks", "you_zheng"}},
  };

  // Check tripLeg
  for (const auto& node : leg.node()) {
    // skip the point in trip leg, check edges
    if (node.edge().name_size() == 0) {
      continue;
    }
    // get edge name
    ASSERT_EQ(node.edge().name_size(), 1);
    const std::string edge_name = node.edge().name(0).value();

    // get landmarks in the edge
    std::vector<std::string> landmark_names{};
    for (const auto& tag : node.edge().tagged_value()) {
      if (tag.type() == TaggedValue_Type_kLandmark) {
        Landmark landmark(tag.value());
        landmark_names.push_back(landmark.name);
      }
    }
    // compare landmarks with expected results
    auto expected_result = expected_landmarks_tripleg.find(edge_name);
    if (expected_result == expected_landmarks_tripleg.end()) {
      throw std::runtime_error("Failed to find the edge in the expected landmarks, edge name = " +
                               edge_name);
    }

    std::sort(landmark_names.begin(), landmark_names.end());
    EXPECT_EQ(expected_result->second, landmark_names);
  }

  // Check maneuver
  ASSERT_EQ(result.directions().routes_size(), 1);
  ASSERT_EQ(result.directions().routes(0).legs_size(), 1);
  auto directions_leg = result.directions().routes(0).legs(0);

  // expected landmark results in the maneuvers
  std::vector<LandmarkInManeuver> cf_maneuver{LandmarkInManeuver("hai_di_lao", 6, 140, 0),
                                              LandmarkInManeuver("lv_mo_li", 12, 160, 0),
                                              LandmarkInManeuver("sheng_ren_min_yi_yuan", 13, 100, 1),
                                              LandmarkInManeuver("gong_shang_yin_hang", 9, 30, 1),
                                              LandmarkInManeuver("ju_yuan", 16, 0, 1),
                                              LandmarkInManeuver("sheng_ren_min_yi_yuan", 13, 80, 1),
                                              LandmarkInManeuver("McDonalds", 7, 20, 0)};
  std::vector<LandmarkInManeuver> fg_maneuver{LandmarkInManeuver("ju_yuan", 16, 30, 0),
                                              LandmarkInManeuver("you_zheng", 2, 0, 0),
                                              LandmarkInManeuver("McDonalds", 7, 40, 1)};
  std::vector<LandmarkInManeuver> end_point_maneuver{LandmarkInManeuver("shell", 1, 30, 0),
                                                     LandmarkInManeuver("you_zheng", 2, 60, 1),
                                                     LandmarkInManeuver("starbucks", 8, 0, 0)};
  std::sort(cf_maneuver.begin(), cf_maneuver.end());
  std::sort(fg_maneuver.begin(), fg_maneuver.end());
  std::sort(end_point_maneuver.begin(), end_point_maneuver.end());

  std::map<std::string, std::vector<LandmarkInManeuver>> expected_landmarks_maneuver = {
      {"S1", std::vector<LandmarkInManeuver>{}},
      {"cf", cf_maneuver},
      {"fg", fg_maneuver},
      {"end_point", end_point_maneuver},
  };

  // check landmarks with expectations one by one
  for (const auto& man : directions_leg.maneuver()) {
    std::string street_name = man.street_name_size() == 1 ? man.street_name(0).value() : "end_point";

    std::vector<LandmarkInManeuver> result_landmarks{};
    for (const auto& l : man.landmarks()) {
      result_landmarks.emplace_back(LandmarkInManeuver(l.name(), static_cast<uint8_t>(l.type()),
                                                       std::round(l.distance()), l.right()));
    }
    std::sort(result_landmarks.begin(), result_landmarks.end());

    auto expected = expected_landmarks_maneuver.find(street_name);
    if (expected == expected_landmarks_maneuver.end()) {
      throw std::runtime_error(
          "Checking landmarks in maneuver failed: cannot find the maneuver in the expected result!");
    }
    ASSERT_EQ(result_landmarks.size(), expected->second.size());
    for (size_t i = 0; i < result_landmarks.size(); ++i) {
      EXPECT_EQ(result_landmarks[i], expected->second[i]);
    }
  }
}
