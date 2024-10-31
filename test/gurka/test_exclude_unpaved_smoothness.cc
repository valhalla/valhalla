#include <filesystem>

#include "gurka.h"
#include "test/test.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

namespace {
const std::vector<std::string> kSupportedCostingModels = {
    "auto",
    "taxi",
    "bus",
    "truck",
};
} // namespace

class ExcludeUnpavedTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double grid_size_meters = 100.;

    const std::string ascii_map = R"(
   E----F----G----H----I----A----J----K----L
                                 |    |
                                 |    |
                                 |    |
                                 |    |
                                 M----N
    )";

    const gurka::ways ways = {
        {"EF", {{"highway", "residential"}, {"surface", "compacted"}}},
        {"FG", {{"highway", "residential"}, {"surface", "compacted"}}},
        {"GH", {{"highway", "residential"}}},
        {"HI", {{"highway", "residential"}, {"surface", "unpaved"}}},
        {"IA", {{"highway", "residential"}, {"surface", "unpaved"}}},
        {"IJ", {{"highway", "residential"}}},
        {"JK", {{"highway", "residential"}, {"surface", "gravel"}}},
        {"KL", {{"highway", "residential"}}},
        {"JM", {{"highway", "residential"}}},
        {"MN", {{"highway", "residential"}}},
        {"NK", {{"highway", "residential"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, grid_size_meters);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/exclude_unpaved");
  }
};

gurka::map ExcludeUnpavedTest::map = {};

TEST_F(ExcludeUnpavedTest, UnpavedRoadsInTheMiddle) {
  // Without options
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"I", "L"}, costing);
    gurka::assert::raw::expect_path(result, {"IJ", "JK", "KL"});
  }

  // Use unpaved roads
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"I", "L"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_unpaved", "0"}});
    gurka::assert::raw::expect_path(result, {"IJ", "JK", "KL"});
  }

  // Do not use unpaved roads
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"I", "L"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_unpaved", "1"}});
    gurka::assert::raw::expect_path(result, {"IJ", "JM", "MN", "NK", "KL"});
  }
}

TEST_F(ExcludeUnpavedTest, UnpavedRoadsUnsupported) {
  const std::string start = "E";
  const std::string end = "L";
  for (const auto& costing : std::vector<std::string>{"bicycle", "pedestrian"}) {
    const auto result_0 =
        gurka::do_action(valhalla::Options::route, map, {start, end}, costing,
                         {{"/costing_options/" + costing + "/exclude_unpaved", "1"}});
    EXPECT_EQ(result_0.trip().routes_size(), 1);
    const auto result_1 =
        gurka::do_action(valhalla::Options::route, map, {start, end}, costing,
                         {{"/costing_options/" + costing + "/exclude_unpaved", "0"}});
    EXPECT_EQ(result_1.trip().routes_size(), 1);
    EXPECT_EQ(gurka::detail::get_paths(result_0), gurka::detail::get_paths(result_1));
  }

  // motor_scooter, motorcycle are unsupported costing models too, but the engine does not get routes
  // through unpaved roads. These edges are not allowed in the Allowed and ReverseAllowed methods. It
  // includes only roads that have surface greater than Surface::kDirt(See kMinimumMotorcycleSurface,
  // kMinimumScooterSurface constants).
}

TEST_F(ExcludeUnpavedTest, UnpavedRoadsInTheBeginning) {
  // Without options
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"E", "H"}, costing);
    gurka::assert::raw::expect_path(result, {"EF", "FG", "GH"});
  }

  // Use unpaved roads
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"E", "H"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_unpaved", "0"}});
    gurka::assert::raw::expect_path(result, {"EF", "FG", "GH"});
  }

  // Do not use unpaved roads
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"E", "H"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_unpaved", "1"}});
    gurka::assert::raw::expect_path(result, {"EF", "FG", "GH"});
  }
}

TEST_F(ExcludeUnpavedTest, UnpavedRoadsInTheEnd) {
  // Without options
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"G", "A"}, costing);
    gurka::assert::raw::expect_path(result, {"GH", "HI", "IA"});
  }

  // Use unpaved roads
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"G", "A"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_unpaved", "0"}});
    gurka::assert::raw::expect_path(result, {"GH", "HI", "IA"});
  }

  // Do not use unpaved roads
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"G", "A"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_unpaved", "1"}});
    gurka::assert::raw::expect_path(result, {"GH", "HI", "IA"});
  }
}

valhalla::gurka::map BuildPBF(const std::string& workdir) {
  const std::string ascii_map = R"(
               
      A--------B-----1--C-------D
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"smoothness", "impassable"}}},
      {"B1C", {{"highway", "primary"}, {"smoothness", "great"}}},
  };

  const gurka::nodes nodes = {{"1", {{"barrier", "gate"}, {"smoothness", "impassable"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.1079374, 52.0887174});

  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, nodes, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;

  return result;
}

TEST(Standalone, SmoothnessAccess) {

  const std::string workdir = "test/data/gurka_smoothness_access";

  if (!std::filesystem::exists(workdir)) {
    bool created = std::filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBF(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));

  {
    GraphId edge_id_1;
    const DirectedEdge* edge_1 = nullptr;
    GraphId edge_id_2;
    const DirectedEdge* edge_2 = nullptr;

    std::tie(edge_id_1, edge_1, edge_id_2, edge_2) = findEdge(graph_reader, map.nodes, "AB", "B");
    // no access due to smoothness = impassable and are therefore tossed.
    // edge_1 = AB
    // edge_2 = BA
    EXPECT_EQ(edge_1, nullptr);
    EXPECT_EQ(edge_2, nullptr);

    std::tie(edge_id_1, edge_1, edge_id_2, edge_2) = findEdge(graph_reader, map.nodes, "B1C", "C");
    // edge_1 = B1C
    // edge_2 = C1B
    // edge is not tossed
    EXPECT_NE(edge_1->forwardaccess(), 0);
    EXPECT_NE(edge_1->reverseaccess(), 0);
    EXPECT_NE(edge_2->forwardaccess(), 0);
    EXPECT_NE(edge_2->reverseaccess(), 0);

    auto node_id = gurka::findNode(graph_reader, map.nodes, "1");
    const auto* node = graph_reader.nodeinfo(node_id);
    // no access due to smoothness = impassable
    EXPECT_EQ(node->access(), 0);
  }
}

TEST(Standalone, SmoothnessWithSurface) {

  const std::string ascii_map = R"(
	               A----B
	  )";

  for (const auto& smoothness_tag :
       std::map<std::string, std::string>{{"smoothness", "excellent"},
                                          {"smoothness", "good"},
                                          {"smoothness", "intermediate"},
                                          {"smoothness", "bad"},
                                          {"smoothness", "very_bad"},
                                          {"smoothness", "horrible"},
                                          {"smoothness", "very_horrible"},
                                          {"smoothness", "blah"}}) {
    const gurka::ways ways = {
        {"AB",
         {{"highway", "path"},
          {"foot", "yes"},
          {"bicycle", "yes"},
          {"horse", "yes"},
          {"surface", "ground"},
          {"trail_visibility", "good"},
          {"width", "0.5"},
          smoothness_tag}},
    };

    constexpr double gridsize = 100;
    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.1079374, 52.0887174});
    auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_smoothness_with_surface");

    GraphReader graph_reader(map.config.get_child("mjolnir"));

    {
      GraphId AB_edge_id;
      const DirectedEdge* AB_edge = nullptr;
      GraphId BA_edge_id;
      const DirectedEdge* BA_edge = nullptr;
      std::tie(AB_edge_id, AB_edge, BA_edge_id, BA_edge) =
          findEdge(graph_reader, map.nodes, "AB", "B");
      EXPECT_NE(AB_edge, nullptr);
      EXPECT_NE(BA_edge, nullptr);

      // surface always wins
      EXPECT_EQ(AB_edge->surface(), Surface::kDirt);
      EXPECT_EQ(BA_edge->surface(), Surface::kDirt);
    }
  }
}

TEST(Standalone, SmoothnessNoSurface) {

  const std::string ascii_map = R"(
	               A----B
	  )";

  for (const auto& smoothness_tag :
       std::map<std::string, std::string>{{"smoothness", "excellent"},
                                          {"smoothness", "good"},
                                          {"smoothness", "intermediate"},
                                          {"smoothness", "bad"},
                                          {"smoothness", "very_bad"},
                                          {"smoothness", "horrible"},
                                          {"smoothness", "very_horrible"},
                                          {"smoothness", "blah"}}) {
    const gurka::ways ways = {
        {"AB",
         {{"highway", "path"},
          {"foot", "yes"},
          {"bicycle", "yes"},
          {"horse", "yes"},
          {"trail_visibility", "good"},
          {"width", "0.5"},
          smoothness_tag}},
    };

    constexpr double gridsize = 100;
    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.1079374, 52.0887174});
    auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_smoothness_no_surface");

    GraphReader graph_reader(map.config.get_child("mjolnir"));

    {
      GraphId AB_edge_id;
      const DirectedEdge* AB_edge = nullptr;
      GraphId BA_edge_id;
      const DirectedEdge* BA_edge = nullptr;
      std::tie(AB_edge_id, AB_edge, BA_edge_id, BA_edge) =
          findEdge(graph_reader, map.nodes, "AB", "B");
      EXPECT_NE(AB_edge, nullptr);
      EXPECT_NE(BA_edge, nullptr);

      if (smoothness_tag.second == "excellent" || smoothness_tag.second == "good") {
        EXPECT_EQ(AB_edge->surface(), Surface::kPavedSmooth);
        EXPECT_EQ(BA_edge->surface(), Surface::kPavedSmooth);
      } else if (smoothness_tag.second == "intermediate") {
        EXPECT_EQ(AB_edge->surface(), Surface::kPavedRough);
        EXPECT_EQ(BA_edge->surface(), Surface::kPavedRough);
      } else if (smoothness_tag.second == "bad") {
        EXPECT_EQ(AB_edge->surface(), Surface::kCompacted);
        EXPECT_EQ(BA_edge->surface(), Surface::kCompacted);
      } else if (smoothness_tag.second == "very_bad") {
        EXPECT_EQ(AB_edge->surface(), Surface::kDirt);
        EXPECT_EQ(BA_edge->surface(), Surface::kDirt);
      } else if (smoothness_tag.second == "horrible") {
        EXPECT_EQ(AB_edge->surface(), Surface::kGravel);
        EXPECT_EQ(BA_edge->surface(), Surface::kGravel);
      } else if (smoothness_tag.second == "very_horrible") {
        EXPECT_EQ(AB_edge->surface(), Surface::kPath);
        EXPECT_EQ(BA_edge->surface(), Surface::kPath);
      } else { // fallback to defaults
        EXPECT_EQ(AB_edge->surface(), Surface::kCompacted);
        EXPECT_EQ(BA_edge->surface(), Surface::kCompacted);
      }
    }
  }
}
TEST(Standalone, SmoothnessWithTrackType) {

  const std::string ascii_map = R"(
  	               A----B
  	  )";

  for (const auto& smoothness_tag :
       std::map<std::string, std::string>{{"smoothness", "excellent"},
                                          {"smoothness", "good"},
                                          {"smoothness", "intermediate"},
                                          {"smoothness", "bad"},
                                          {"smoothness", "very_bad"},
                                          {"smoothness", "horrible"},
                                          {"smoothness", "very_horrible"},
                                          {"smoothness", "blah"}}) {
    const gurka::ways ways = {
        {"AB",
         {{"highway", "track"},
          {"tracktype", "grade1"},
          {"foot", "yes"},
          {"bicycle", "yes"},
          {"horse", "yes"},
          {"trail_visibility", "good"},
          {"width", "0.5"},
          smoothness_tag}},
    };

    constexpr double gridsize = 100;
    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.1079374, 52.0887174});
    auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_smoothness_no_surface");

    GraphReader graph_reader(map.config.get_child("mjolnir"));

    {
      GraphId AB_edge_id;
      const DirectedEdge* AB_edge = nullptr;
      GraphId BA_edge_id;
      const DirectedEdge* BA_edge = nullptr;
      std::tie(AB_edge_id, AB_edge, BA_edge_id, BA_edge) =
          findEdge(graph_reader, map.nodes, "AB", "B");
      EXPECT_NE(AB_edge, nullptr);
      EXPECT_NE(BA_edge, nullptr);

      // tracktype always wins
      EXPECT_EQ(AB_edge->surface(), Surface::kPavedRough);
      EXPECT_EQ(BA_edge->surface(), Surface::kPavedRough);
    }
  }
}
