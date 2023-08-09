#include <filesystem>
#include <gtest/gtest.h>
#include <vector>

#include "gurka.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/landmark_builder.h"
#include "test/test.h"
#include "baldr/landmark.h"
#include "baldr/graphreader.h"

#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

const std::string workdir = VALHALLA_BUILD_DIR "test/data/landmarks";
const std::string db_path = workdir + "/landmarks.sqlite";
const std::string pbf_filename = workdir + "/map.pbf";
valhalla::gurka::map landmark_map;

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
      // {"e", {{"name", "fake"}, {"amenity", "fake"}}},
      {"b", {{"name", "la ji tong"}, {"amenity", "waste_basket"}}},
      {"c", {{"name", "hua yuan"}, {"place", "city"}}},
      {"d", {{"traffic_signal", "signal"}}},
  };

  const gurka::ways ways = {
      // {"ae", {{"highway", "residential"}}},
      // {"eb", {{"highway", "residential"}}},
      // {"ab", {{"highway", "residential"}, {"tunnel", "yes"}, {"tunnel:name", "Fort McHenry Tunnel"}}},
      {"ab", {{"highway", "residential"}}},
      {"bc", {{"highway", "motorway"}}},
      {"cd", {{"highway", "residential"}, {"maxspeed", "60"}}},
  };

  constexpr double gridsize = 100;
  landmark_map.nodes = gurka::detail::map_to_coordinates(ascii_map, gridsize, {-.01, 0});

  detail::build_pbf(landmark_map.nodes, ways, nodes, {}, pbf_filename, 0, false);
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
  // make a config, though we dont need the landmarks db in there until the next test
  landmark_map.config = test::make_config(workdir, {{"mjolnir.landmarks_db", db_path}},
                        {{"additional_data", "mjolnir.traffic_extract", "mjolnir.tile_extract"}});
  // landmark_map.config.get_child("mjolnir").erase("traffic_extract");

  // build regular graph tiles from the pbf that we have already made, there wont be landmarks in them
  mjolnir::build_tile_set(landmark_map.config, {pbf_filename}, mjolnir::BuildStage::kInitialize,
                          mjolnir::BuildStage::kValidate, false);

  // load one of the graphtiles via the graphtilebuilder with the deserialize option turned on
  GraphId tile_id("2/519119/0");
  GraphTileBuilder tb(workdir, tile_id, true);

  // loop over the edges in the tile and add a landmark to each one using our new addlandmark function
  // make the names simple like std::to_string(edge_id.id()) the lat lon can be similarly easy like
  // takign other simple information about the edge and encoding it into 2 numbers something you can
  // easily reverse in the assertion below, for the type you can also use the .id field of the eggeid
  // but just modulus it with the max type so it doesnt pick an invalid value
  auto invalid_landmark = static_cast<uint32_t>(LandmarkType::casino) + 1;
  uint32_t edge_index = 0;

  const Landmark landmark_fixed(1, "fixed landmark", LandmarkType::casino, 0., 0.);

  std::string str = landmark_fixed.to_str();
  Landmark landmark(str);

  EXPECT_EQ(landmark.id, 0);
  EXPECT_EQ(landmark.name, "fixed landmark");
  EXPECT_EQ(landmark.type, LandmarkType::casino);
  EXPECT_EQ(landmark.lng, 0.);
  EXPECT_EQ(landmark.lat, 0.);

  for (const auto& e : tb.directededges()) {
    std::vector<PointLL> shape = tb.edgeinfo(&e).shape();
    auto point = shape[shape.size() / 2];
    auto ltype = static_cast<LandmarkType>(edge_index % invalid_landmark);

    // Landmark landmark(edge_index, std::to_string(edge_index), ltype, point.first, point.second);

    auto edge_id = tile_id;
    edge_id.set_id(edge_index++);

    // tb.AddLandmark(edge_id, landmark);
    tb.AddLandmark(edge_id, landmark_fixed);
  }

  // call the store graphtile function to overwrite the tile on disk with the new info
  tb.StoreTileData();

  // instantiate a graphreader using the config
  GraphReader gr(landmark_map.config.get_child("mjolnir"));

  // call getgraphtile on it
  auto tile = gr.GetGraphTile(tile_id);

  // get the edgeinfo using the edge who you added the landmark to
  // call GetNamesAndTypes on the edgeinfo

  for (const auto& e: tile->GetDirectedEdges()) {
    auto ei = tile->edgeinfo(&e);
    auto results = ei.GetNamesAndTypes(true);

    for (const std::tuple<std::string, bool, uint8_t>& r: results) {
      // if (std::get<2>(r) == static_cast<char>(valhalla::baldr::TaggedValue::kLandmark)) {
      //   std::string str = std::get<0>(r);
      //   std::cout << str << std::endl;
        // Landmark landmark(str);
        // std::cout << landmark.id << landmark.name << static_cast<uint8_t>(landmark.type) << landmark.lng << landmark.lat << std::endl;
      // }
      std::cout << "name:" << std::get<0>(r) << ", bool:" << std::get<1>(r) << ", type:" << static_cast<uint8_t>(std::get<2>(r)) << std::endl;
    }
  }
  
  // loop over the results until the type is kLandmark when it is then you need to use the method
  // to decode the string into a landmark object. asser that the landmark you got out matches the one
  // you told it to add (excepting the id because we dont store that).
}