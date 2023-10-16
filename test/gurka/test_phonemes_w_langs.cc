#include "baldr/graphreader.h"
#include "mjolnir/util.h"

#include "gurka.h"
#include "test/test.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

const std::string workdir = "test/data/gurka_phonemes_w_langs";
const auto pbf_filename = workdir + "/map.pbf";
const std::vector<std::string> input_files = {workdir + "/map.pbf"};
const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
constexpr double gridsize = 100;

void CreateWorkdir() {
  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }
}

// parameterized test class to test all the different types of pronunciations
class PhonemesWithLangsTest
    : public ::testing::TestWithParam<std::tuple<std::string, baldr::PronunciationAlphabet>> {
protected:
  static gurka::ways ways;
  static std::string ascii_map;
  static std::map<std::string, midgard::PointLL> layout;
  static boost::property_tree::ptree pt;
  static void SetUpTestSuite() {
    ascii_map = R"(
	      B----C
	  )";

    valhalla::gurka::map map;
    pt = map.config;
    pt.put("mjolnir.data_processing.allow_alt_name", "true");
    pt.put("mjolnir.data_processing.use_admin_db", "true");
    pt.put("mjolnir.tile_dir", workdir + "/tiles");
    pt.put("mjolnir.admin", sqlite);
  }
};
gurka::ways PhonemesWithLangsTest::ways = {};
std::string PhonemesWithLangsTest::ascii_map = {};
std::map<std::string, midgard::PointLL> PhonemesWithLangsTest::layout = {};
boost::property_tree::ptree PhonemesWithLangsTest::pt = {};

TEST_P(PhonemesWithLangsTest, Names) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk"},
            {"osm_id", "101"},
            {"name", "Chaussée de Gand - Steenweg op Gent/Gentsesteenweg"},
            {"name:fr", "Chaussée de Gand"},
            {"name:left", "Chaussée de Gand - Gentsesteenweg"},
            {"name:left:nl", "Gentsesteenweg"},
            {"name:right", "Chaussée de Gand - Steenweg op Gent"},
            {"name:right:nl", "Steenweg op Gent"},

            {"name" + param_tag, "Chaussée de Gand P - Steenweg op Gent P/Gentsesteenweg P"},
            {"name:fr" + param_tag, "Chaussée de Gand P"},
            {"name:left" + param_tag, "Chaussée de Gand P - Gentsesteenweg P"},
            {"name:left:nl" + param_tag, "Gentsesteenweg P"},
            {"name:right" + param_tag, "Chaussée de Gand P - Steenweg op Gent P"},
            {"name:right:nl" + param_tag, "Steenweg op Gent P"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  {
    GraphId node_id = BC_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(BC_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 2);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());

      if (name_index == 0) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "fr");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Chaussée de Gand P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (name_index == 1) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Steenweg op Gent P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++name_index;
    }
  }

  {
    GraphId node_id = CB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(CB_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 2);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());
      if (name_index == 0) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "fr");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Chaussée de Gand P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (name_index == 1) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "Gentsesteenweg P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++name_index;
    }
  }
}

TEST_P(PhonemesWithLangsTest, Alts) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk"},
            {"osm_id", "101"},
            {"alt_name", "Chaussée de Gand - Steenweg op Gent/Gentsesteenweg"},
            {"alt_name:fr", "Chaussée de Gand"},
            {"alt_name:left", "Chaussée de Gand - Gentsesteenweg"},
            {"alt_name:left:nl", "Gentsesteenweg"},
            {"alt_name:right", "Chaussée de Gand - Steenweg op Gent"},
            {"alt_name:right:nl", "Steenweg op Gent"},

            {"alt_name" + param_tag, "Chaussée de Gand P - Steenweg op Gent P/Gentsesteenweg P"},
            {"alt_name:fr" + param_tag, "Chaussée de Gand P"},
            {"alt_name:left" + param_tag, "Chaussée de Gand P - Gentsesteenweg P"},
            {"alt_name:left:nl" + param_tag, "Gentsesteenweg P"},
            {"alt_name:right" + param_tag, "Chaussée de Gand P - Steenweg op Gent P"},
            {"alt_name:right:nl" + param_tag, "Steenweg op Gent P"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  {
    GraphId node_id = BC_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(BC_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 3);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }

      if (name_index == 0) {
        ++name_index;
        continue;
      }
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());

      if (name_index == 1) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "fr");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Chaussée de Gand P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));

      } else if (name_index == 2) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Steenweg op Gent P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++name_index;
    }
  }

  {
    GraphId node_id = CB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(CB_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 3);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }

      if (name_index == 0) {
        ++name_index;
        continue;
      }

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());
      if (name_index == 1) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "fr");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Chaussée de Gand P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));

      } else if (name_index == 2) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "Gentsesteenweg P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }

      ++name_index;
    }
  }
}

TEST_P(PhonemesWithLangsTest, Official) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk"},
            {"osm_id", "101"},
            {"official_name", "Chaussée de Gand - Steenweg op Gent/Gentsesteenweg"},
            {"official_name:fr", "Chaussée de Gand"},
            {"official_name:left", "Chaussée de Gand - Gentsesteenweg"},
            {"official_name:left:nl", "Gentsesteenweg"},
            {"official_name:right", "Chaussée de Gand - Steenweg op Gent"},
            {"official_name:right:nl", "Steenweg op Gent"},

            {"official_name" + param_tag, "Chaussée de Gand P - Steenweg op Gent P/Gentsesteenweg P"},
            {"official_name:fr" + param_tag, "Chaussée de Gand P"},
            {"official_name:left" + param_tag, "Chaussée de Gand P - Gentsesteenweg P"},
            {"official_name:left:nl" + param_tag, "Gentsesteenweg P"},
            {"official_name:right" + param_tag, "Chaussée de Gand P - Steenweg op Gent P"},
            {"official_name:right:nl" + param_tag, "Steenweg op Gent P"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  {
    GraphId node_id = BC_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(BC_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 3);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }

      if (name_index == 0) {
        ++name_index;
        continue;
      }
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());

      if (name_index == 1) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "fr");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Chaussée de Gand P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (name_index == 2) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Steenweg op Gent P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++name_index;
    }
  }

  {
    GraphId node_id = CB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(CB_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);
    ASSERT_EQ(names_and_types.size(), 3);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }

      if (name_index == 0) {
        ++name_index;
        continue;
      }

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());
      if (name_index == 1) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "fr");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Chaussée de Gand P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));

      } else if (name_index == 2) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "Gentsesteenweg P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++name_index;
    }
  }
}

TEST_P(PhonemesWithLangsTest, Tunnel) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk"},
            {"osm_id", "101"},
            {"tunnel:name", "Chaussée de Gand - Steenweg op Gent/Gentsesteenweg"},
            {"tunnel:name:fr", "Chaussée de Gand"},
            {"tunnel:name:left", "Chaussée de Gand - Gentsesteenweg"},
            {"tunnel:name:left:nl", "Gentsesteenweg"},
            {"tunnel:name:right", "Chaussée de Gand - Steenweg op Gent"},
            {"tunnel:name:right:nl", "Steenweg op Gent"},

            {"tunnel:name" + param_tag, "Chaussée de Gand P - Steenweg op Gent P/Gentsesteenweg P"},
            {"tunnel:name:fr" + param_tag, "Chaussée de Gand P"},
            {"tunnel:name:left" + param_tag, "Chaussée de Gand P - Gentsesteenweg P"},
            {"tunnel:name:left:nl" + param_tag, "Gentsesteenweg P"},
            {"tunnel:name:right" + param_tag, "Chaussée de Gand P - Steenweg op Gent P"},
            {"tunnel:name:right:nl" + param_tag, "Steenweg op Gent P"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  {
    GraphId node_id = BC_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(BC_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 3);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) == 0) {
        // Skip the non tagged names
        ++name_index;
        continue;
      }

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());

      if (name_index == 1) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "fr");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Chaussée de Gand P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));

      } else if (name_index == 2) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Steenweg op Gent P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++name_index;
    }
  }

  {
    GraphId node_id = CB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(CB_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 3);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) == 0) {
        // Skip the non tagged names
        ++name_index;
        continue;
      }

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());
      if (name_index == 1) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "fr");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Chaussée de Gand P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (name_index == 2) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "Gentsesteenweg P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++name_index;
    }
  }
}

TEST_P(PhonemesWithLangsTest, NamesFB) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk"},
            {"osm_id", "101"},
            {"name", "Gentsesteenweg;Chaussée de Gand"},
            {"name:forward", "Gentsesteenweg"},
            {"name:forward:nl", "Gentsesteenweg"},
            {"name:backward", "Chaussée de Gand"},
            {"name:backward:fr", "Chaussée de Gand"},

            {"name" + param_tag, "Gentsesteenweg P;Chaussée de Gand P"},
            {"name:forward" + param_tag, "Gentsesteenweg P"},
            {"name:forward:nl" + param_tag, "Gentsesteenweg P"},
            {"name:backward" + param_tag, "Chaussée de Gand P"},
            {"name:backward:fr" + param_tag, "Chaussée de Gand P"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  {
    GraphId node_id = BC_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(BC_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 1);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());
      if (name_index == 0) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "Gentsesteenweg P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++name_index;
    }
  }

  {
    GraphId node_id = CB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(CB_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 1);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());
      if (name_index == 0) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "fr");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Chaussée de Gand P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++name_index;
    }
  }
}

TEST_P(PhonemesWithLangsTest, RefLR) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk"},
            {"osm_id", "101"},
            {"ref", "10;11"},
            {"ref:right", "10"},
            {"ref:right:nl", "10"},
            {"ref:left", "11"},
            {"ref:left:fr", "11"},

            {"ref" + param_tag, "10 P;11 P"},
            {"ref:right" + param_tag, "10 P"},
            {"ref:right:nl" + param_tag, "10 P"},
            {"ref:left" + param_tag, "11 P"},
            {"ref:left:fr" + param_tag, "11 P"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  {
    GraphId node_id = BC_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(BC_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 2);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());
      if (name_index == 0) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "10 P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      break;
    }
  }

  {
    GraphId node_id = CB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(CB_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 2);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());
      if (name_index == 0) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "fr");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "11 P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      break;
    }
  }
}

TEST_P(PhonemesWithLangsTest, Destinations) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk"},
            {"osm_id", "101"},
            {"ref", "10"},
            {"destination", "destination:lang:nl"},
            // {"destination:lang:nl", "destination:lang:nl"}, not needed as lang is in next tag
            {"destination:lang:nl" + param_tag, "destination:lang:nl:pronunciation"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  GraphId node_id = BC_edge->endnode();
  auto tile = graph_reader.GetGraphTile(node_id);
  std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
  std::vector<SignInfo> signs = tile->GetSigns(BC_edge_id.id(), linguistics);

  uint32_t sign_index = 0;
  ASSERT_EQ(signs.size(), 1);
  ASSERT_EQ(linguistics.size(), 1);

  for (const auto& sign : signs) {
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
        linguistics.find(sign_index);
    ASSERT_NE(iter, linguistics.end());

    if (sign_index == 0) {
      EXPECT_EQ(signs.at(sign_index).text(), "destination:lang:nl");
      EXPECT_EQ(to_string(
                    static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                "nl");
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                "destination:lang:nl:pronunciation");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(param_alphabet));
    }
    ++sign_index;
  }
}

TEST_P(PhonemesWithLangsTest, DestinationStreet) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"ref", "10"},
        {"destination:street", "destination:lang:nl"},
        //{"destination:street:lang:nl", "destination:lang:nl"}, //not needed as lang is in next tag
        {"destination:street:lang:nl" + param_tag, "destination:lang:nl:pronunciation"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  GraphId node_id = BC_edge->endnode();
  auto tile = graph_reader.GetGraphTile(node_id);
  std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
  std::vector<SignInfo> signs = tile->GetSigns(BC_edge_id.id(), linguistics);

  uint32_t sign_index = 0;
  ASSERT_EQ(signs.size(), 1);
  ASSERT_EQ(linguistics.size(), 1);

  for (const auto& sign : signs) {
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
        linguistics.find(sign_index);
    ASSERT_NE(iter, linguistics.end());

    if (sign_index == 0) {
      EXPECT_EQ(signs.at(sign_index).text(), "destination:lang:nl");
      EXPECT_EQ(to_string(
                    static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                "nl");
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                "destination:lang:nl:pronunciation");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(param_alphabet));
    }
    ++sign_index;
  }
}

TEST_P(PhonemesWithLangsTest, DestinationStreetTo) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk"},
            {"osm_id", "101"},
            {"ref", "10"},
            {"destination:street:to", "destination:lang:nl"},
            //{"destination:street:to:lang:nl", "destination:lang:nl"}, //not needed
            // as lang is in next tag
            {"destination:street:to:lang:nl" + param_tag, "destination:lang:nl:pronunciation"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  GraphId node_id = BC_edge->endnode();
  auto tile = graph_reader.GetGraphTile(node_id);
  std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
  std::vector<SignInfo> signs = tile->GetSigns(BC_edge_id.id(), linguistics);

  uint32_t sign_index = 0;
  ASSERT_EQ(signs.size(), 1);
  ASSERT_EQ(linguistics.size(), 1);

  for (const auto& sign : signs) {

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
        linguistics.find(sign_index);
    ASSERT_NE(iter, linguistics.end());

    if (sign_index == 0) {
      EXPECT_EQ(signs.at(sign_index).text(), "destination:lang:nl");
      EXPECT_EQ(to_string(
                    static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                "nl");
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                "destination:lang:nl:pronunciation");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(param_alphabet));
    }
    ++sign_index;
  }
}

TEST_P(PhonemesWithLangsTest, DestinationsFB) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"ref", "10"},
        {"destination:forward", "destination:forward:nl"},
        {"destination:backward", "destination:backward:fr"},
        {"destination:forward" + param_tag,
         "destination:forward:lang:nl:pronunciation"}, // this is really optional
        {"destination:backward" + param_tag,
         "destination:backward:lang:fr:pronunciation"}, // this is really optional
        {"destination:forward:lang:nl" + param_tag, "destination:forward:lang:nl:pronunciation"},
        {"destination:backward:lang:fr" + param_tag, "destination:backward:lang:fr:pronunciation"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  {
    GraphId node_id = BC_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
    std::vector<SignInfo> signs = tile->GetSigns(BC_edge_id.id(), linguistics);
    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 1);
    ASSERT_EQ(linguistics.size(), 1);

    for (const auto& sign : signs) {
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(sign_index);
      ASSERT_NE(iter, linguistics.end());

      if (sign_index == 0) {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:forward:nl");

        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "destination:forward:lang:nl:pronunciation");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++sign_index;
    }
  }

  {
    {
      GraphId node_id = CB_edge->endnode();
      auto tile = graph_reader.GetGraphTile(node_id);
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
      std::vector<SignInfo> signs = tile->GetSigns(CB_edge_id.id(), linguistics);

      uint32_t sign_index = 0;
      ASSERT_EQ(signs.size(), 1);
      ASSERT_EQ(linguistics.size(), 1);

      for (const auto& sign : signs) {
        std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
            linguistics.find(sign_index);
        ASSERT_NE(iter, linguistics.end());

        if (sign_index == 0) {
          EXPECT_EQ(signs.at(sign_index).text(), "destination:backward:fr");

          EXPECT_EQ(to_string(static_cast<Language>(
                        std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                    "fr");
          EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                    "destination:backward:lang:fr:pronunciation");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(param_alphabet));
        }
        ++sign_index;
      }
    }
  }
}

TEST_P(PhonemesWithLangsTest, DestinationsMultiLangs) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk"},
            {"osm_id", "101"},
            {"ref", "10"},
            {"destination:lang:nl", "destination in dutch"},
            {"destination:lang:fr", "destination in french"},
            {"destination:lang:nl" + param_tag, "destination pronunciation in dutch"},
            {"destination:lang:fr" + param_tag, "destination pronunciation in french"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.34999, 50.84643});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  {
    GraphId node_id = BC_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
    std::vector<SignInfo> signs = tile->GetSigns(BC_edge_id.id(), linguistics);
    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 2);
    ASSERT_EQ(linguistics.size(), 2);

    for (const auto& sign : signs) {
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(sign_index);
      ASSERT_NE(iter, linguistics.end());
      if (sign_index == 0) {
        EXPECT_EQ(signs.at(sign_index).text(), "destination in french");

        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "fr");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "destination pronunciation in french");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (sign_index == 1) {
        EXPECT_EQ(signs.at(sign_index).text(), "destination in dutch");
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "destination pronunciation in dutch");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++sign_index;
    }
  }

  {
    {
      GraphId node_id = CB_edge->endnode();
      auto tile = graph_reader.GetGraphTile(node_id);
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
      std::vector<SignInfo> signs = tile->GetSigns(CB_edge_id.id(), linguistics);

      uint32_t sign_index = 0;
      ASSERT_EQ(signs.size(), 2);
      ASSERT_EQ(linguistics.size(), 2);

      for (const auto& sign : signs) {
        std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
            linguistics.find(sign_index);
        ASSERT_NE(iter, linguistics.end());

        if (sign_index == 0) {
          EXPECT_EQ(signs.at(sign_index).text(), "destination in french");

          EXPECT_EQ(to_string(static_cast<Language>(
                        std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                    "fr");
          EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                    "destination pronunciation in french");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(param_alphabet));
        } else if (sign_index == 1) {
          EXPECT_EQ(signs.at(sign_index).text(), "destination in dutch");

          EXPECT_EQ(to_string(static_cast<Language>(
                        std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                    "nl");
          EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                    "destination pronunciation in dutch");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(param_alphabet));
        }
        ++sign_index;
      }
    }
  }
}

TEST_P(PhonemesWithLangsTest, DestinationsFBNoOptionalTag) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"ref", "10"},
        {"destination:forward", "destination:forward:nl"},
        {"destination:backward", "destination:backward:fr"},
        //{"destination:forward" + param_tag,
        //"destination:forward:lang:nl:pronunciation"}, //this is really optional
        //{"destination:backward" + param_tag,
        //"destination:backward:lang:fr:pronunciation"}, //this is really optional
        {"destination:forward:lang:nl" + param_tag, "destination:forward:lang:nl:pronunciation"},
        {"destination:backward:lang:fr" + param_tag, "destination:backward:lang:fr:pronunciation"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  {
    GraphId node_id = BC_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
    std::vector<SignInfo> signs = tile->GetSigns(BC_edge_id.id(), linguistics);
    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 1);
    ASSERT_EQ(linguistics.size(), 1);

    for (const auto& sign : signs) {
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(sign_index);
      ASSERT_NE(iter, linguistics.end());

      if (sign_index == 0) {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:forward:nl");

        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "destination:forward:lang:nl:pronunciation");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++sign_index;
    }
  }

  {
    {
      GraphId node_id = CB_edge->endnode();
      auto tile = graph_reader.GetGraphTile(node_id);
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
      std::vector<SignInfo> signs = tile->GetSigns(CB_edge_id.id(), linguistics);

      uint32_t sign_index = 0;
      ASSERT_EQ(signs.size(), 1);
      ASSERT_EQ(linguistics.size(), 1);

      for (const auto& sign : signs) {

        std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
            linguistics.find(sign_index);
        ASSERT_NE(iter, linguistics.end());

        if (sign_index == 0) {
          EXPECT_EQ(signs.at(sign_index).text(), "destination:backward:fr");
          EXPECT_EQ(to_string(static_cast<Language>(
                        std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                    "fr");
          EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                    "destination:backward:lang:fr:pronunciation");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(param_alphabet));
        }
        ++sign_index;
      }
    }
  }
}

TEST_P(PhonemesWithLangsTest, DestinationRef) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk"},
            {"osm_id", "101"},
            {"ref", "10"},
            {"destination:ref", "destination:lang:nl"},
            //{"destination:ref:lang:nl", "destination:lang:nl"}, //not needed as lang is in next tag
            {"destination:ref:lang:nl" + param_tag, "destination:lang:nl:pronunciation"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  GraphId node_id = BC_edge->endnode();
  auto tile = graph_reader.GetGraphTile(node_id);
  std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
  std::vector<SignInfo> signs = tile->GetSigns(BC_edge_id.id(), linguistics);

  uint32_t sign_index = 0;
  ASSERT_EQ(signs.size(), 1);
  ASSERT_EQ(linguistics.size(), 1);

  for (const auto& sign : signs) {
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
        linguistics.find(sign_index);
    ASSERT_NE(iter, linguistics.end());

    if (sign_index == 0) {
      EXPECT_EQ(signs.at(sign_index).text(), "destination:lang:nl");
      EXPECT_EQ(to_string(
                    static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                "nl");
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                "destination:lang:nl:pronunciation");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(param_alphabet));
    }
    ++sign_index;
  }
}

TEST_P(PhonemesWithLangsTest, DestinationRefTo) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"ref", "10"},
        {"destination:ref:to", "destination:lang:nl"},
        //{"destination:ref:to:lang:nl", "destination:lang:nl"}, //not needed as lang is in next tag
        {"destination:ref:to:lang:nl" + param_tag, "destination:lang:nl:pronunciation"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  GraphId node_id = BC_edge->endnode();
  auto tile = graph_reader.GetGraphTile(node_id);
  std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
  std::vector<SignInfo> signs = tile->GetSigns(BC_edge_id.id(), linguistics);

  uint32_t sign_index = 0;
  ASSERT_EQ(signs.size(), 1);
  ASSERT_EQ(linguistics.size(), 1);

  for (const auto& sign : signs) {
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
        linguistics.find(sign_index);
    ASSERT_NE(iter, linguistics.end());

    if (sign_index == 0) {
      EXPECT_EQ(signs.at(sign_index).text(), "destination:lang:nl");
      EXPECT_EQ(to_string(
                    static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                "nl");
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                "destination:lang:nl:pronunciation");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(param_alphabet));
    }
    ++sign_index;
  }
}

TEST_P(PhonemesWithLangsTest, DestinationJunctionRef) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk_link"},
            {"osm_id", "101"},
            {"ref", "10"},
            {"junction:ref", "junction:nl"},
            //{"junction:ref:nl", "junction:ref:nl"}, //not needed as lang is in next tag
            {"junction:ref:nl" + param_tag, "junction:nl:pronunciation"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  GraphId node_id = BC_edge->endnode();
  auto tile = graph_reader.GetGraphTile(node_id);
  std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
  std::vector<SignInfo> signs = tile->GetSigns(BC_edge_id.id(), linguistics);

  uint32_t sign_index = 0;
  ASSERT_EQ(signs.size(), 1);
  ASSERT_EQ(linguistics.size(), 1);

  for (const auto& sign : signs) {
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
        linguistics.find(sign_index);
    ASSERT_NE(iter, linguistics.end());

    if (sign_index == 0) {
      EXPECT_EQ(signs.at(sign_index).text(), "junction:nl");
      EXPECT_EQ(to_string(
                    static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                "nl");
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                "junction:nl:pronunciation");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(param_alphabet));
    }
    ++sign_index;
  }
}

TEST_P(PhonemesWithLangsTest, NodeRef) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC", {{"highway", "trunk_link"}, {"osm_id", "101"}}}};

  const gurka::nodes nodes = {{"B",
                               {{"ref", "node ref"},
                                {"highway", "motorway_junction"},
                                {"ref:nl" + param_tag, "ref:nl:pronunciation"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  detail::build_pbf(layout, ways, nodes, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  GraphId node_id = BC_edge->endnode();
  auto tile = graph_reader.GetGraphTile(node_id);
  std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
  std::vector<SignInfo> signs = tile->GetSigns(BC_edge_id.id(), linguistics);

  uint32_t sign_index = 0;
  ASSERT_EQ(signs.size(), 2);
  ASSERT_EQ(linguistics.size(), 1);

  for (const auto& sign : signs) {
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
        linguistics.find(sign_index);

    if (sign_index == 1) {
      EXPECT_EQ(iter, linguistics.end());
    } else if (sign_index == 0) {
      ASSERT_NE(iter, linguistics.end());
      EXPECT_EQ(signs.at(sign_index).text(), "node ref");
      EXPECT_EQ(to_string(
                    static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                "nl");
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                "ref:nl:pronunciation");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(param_alphabet));
    }
    ++sign_index;
  }
}

TEST_P(PhonemesWithLangsTest, NodeName) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC", {{"highway", "trunk"}, {"osm_id", "101"}}}};

  const gurka::nodes nodes = {{"C",
                               {{"highway", "traffic_signals"},
                                {"name", "両国二丁目"},
                                {"name:ja", "両国二丁目"},
                                {"name:en", "Ryogoku 2-chome"},
                                {"name:ja" + param_tag, "両国二丁目 pronunciation"},
                                {"name:en" + param_tag, "Ryogoku 2-chome pronunciation"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {139.79079, 35.69194});
  detail::build_pbf(layout, ways, nodes, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  GraphId node_id = BC_edge->endnode();
  auto tile = graph_reader.GetGraphTile(node_id);
  std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
  std::vector<SignInfo> signs = tile->GetSigns(node_id.id(), linguistics, true);

  uint32_t sign_index = 0;
  ASSERT_EQ(signs.size(), 2);
  ASSERT_EQ(linguistics.size(), 2);

  for (const auto& sign : signs) {
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
        linguistics.find(sign_index);
    ASSERT_NE(iter, linguistics.end());

    if (sign_index == 0) {
      EXPECT_EQ(signs.at(sign_index).text(), "両国二丁目");
      EXPECT_EQ(to_string(
                    static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                "ja");
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                "両国二丁目 pronunciation");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(param_alphabet));
    } else if (sign_index == 1) {
      EXPECT_EQ(signs.at(sign_index).text(), "Ryogoku 2-chome");
      EXPECT_EQ(to_string(
                    static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                "en");
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                "Ryogoku 2-chome pronunciation");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(param_alphabet));
    }
    ++sign_index;
  }
}

TEST_P(PhonemesWithLangsTest, NamesPart2) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk"},
            {"osm_id", "101"},
            {"name:en" + param_tag, "dV|fi \"lek *\"rod"},
            {"name" + param_tag, "dV|fi \"lek *\"rod"},
            {"name:en", "Duffy Lake Road"},
            {"name", "Duffy Lake Road"},
            {"ref" + param_tag, "haI|%we \"naIn|ti *\"naIn \"nORt"},
            {"ref", "HWY-99"}}}};

  // note:  In Canada which is multilingual.  Must specify the lang
  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {-121.9281, 50.6827});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  {
    GraphId node_id = BC_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(BC_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 2);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    ASSERT_EQ(linguistics.size(), 2);

    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());

      if (name_index == 0) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "none");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "haI|%we \"naIn|ti *\"naIn \"nORt");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (name_index == 1) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "en");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "dV|fi \"lek *\"rod");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++name_index;
    }
  }

  {
    GraphId node_id = CB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(CB_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 2);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    ASSERT_EQ(linguistics.size(), 2);

    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());
      if (name_index == 0) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "none");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "haI|%we \"naIn|ti *\"naIn \"nORt");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (name_index == 1) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "en");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "dV|fi \"lek *\"rod");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++name_index;
    }
  }
}

TEST_P(PhonemesWithLangsTest, OldDataDestination) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk"},
            {"osm_id", "101"},
            {"ref", "10"},
            {"destination", "destination in ja"},
            {"destination:en", "destination in en"},
            {"destination" + param_tag, "ipa phoneme"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {139.79079, 35.69194});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  GraphId node_id = BC_edge->endnode();
  auto tile = graph_reader.GetGraphTile(node_id);

  std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
  std::vector<SignInfo> signs = tile->GetSigns(BC_edge_id.id(), linguistics);
  uint32_t sign_index = 0;
  ASSERT_EQ(signs.size(), 2);
  ASSERT_EQ(linguistics.size(), 2);

  for (const auto& sign : signs) {
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
        linguistics.find(sign_index);
    ASSERT_NE(iter, linguistics.end());

    if (sign_index == 0) {
      EXPECT_EQ(signs.at(sign_index).text(), "destination in ja");
      EXPECT_EQ(to_string(
                    static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                "ja");
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(baldr::PronunciationAlphabet::kNone));
    } else if (sign_index == 1) {
      EXPECT_EQ(signs.at(sign_index).text(), "destination in en");
      EXPECT_EQ(to_string(
                    static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                "en");
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(baldr::PronunciationAlphabet::kNone));
    }
    ++sign_index;
  }
}

TEST_P(PhonemesWithLangsTest, ForwardDestination) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk"},
            {"osm_id", "101"},
            {"ref", "10"},
            {"destination:forward:lang:en", "Koriyama"},
            {"destination:forward", "郡山"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {139.79079, 35.69194});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);
  {
    GraphId node_id = BC_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
    std::vector<SignInfo> signs = tile->GetSigns(BC_edge_id.id(), linguistics);
    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 2);
    ASSERT_EQ(linguistics.size(), 2);

    for (const auto& sign : signs) {
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(sign_index);
      ASSERT_NE(iter, linguistics.end());

      if (sign_index == 0) {
        EXPECT_EQ(signs.at(sign_index).text(), "郡山");
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "ja");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNone));
      } else if (sign_index == 1) {
        EXPECT_EQ(signs.at(sign_index).text(), "Koriyama");
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "en");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNone));
      }
      ++sign_index;
    }
  }
  {
    GraphId node_id = CB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
    std::vector<SignInfo> signs = tile->GetSigns(CB_edge_id.id(), linguistics);
    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 0); // signs are only in the forward direction
    ASSERT_EQ(linguistics.size(), 0);
  }
}

TEST_P(PhonemesWithLangsTest, Junction) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk_link"},
            {"osm_id", "101"},
            {"oneway", "yes"},
            {"ref", "10"},
            {"junction:name:en", "Iwakimiwa IC"},
            {"junction:name", "いわき三和ＩＣ"},
            {"junction:name:ja" + param_tag, "ja ipa"},
            {"junction:name:ja", "いわき三和ＩＣ"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {139.79079, 35.69194});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  GraphId node_id = BC_edge->endnode();
  auto tile = graph_reader.GetGraphTile(node_id);
  std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
  std::vector<SignInfo> signs = tile->GetSigns(BC_edge_id.id(), linguistics);
  uint32_t sign_index = 0;
  ASSERT_EQ(signs.size(), 2);
  ASSERT_EQ(linguistics.size(), 2);

  for (const auto& sign : signs) {
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
        linguistics.find(sign_index);
    ASSERT_NE(iter, linguistics.end());

    if (sign_index == 0) {
      EXPECT_EQ(signs.at(sign_index).text(), "いわき三和ＩＣ");
      EXPECT_EQ(to_string(
                    static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                "ja");
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "ja ipa");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(param_alphabet));
    } else if (sign_index == 1) {
      EXPECT_EQ(signs.at(sign_index).text(), "Iwakimiwa IC");
      EXPECT_EQ(to_string(
                    static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                "en");
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(baldr::PronunciationAlphabet::kNone));
    }
    ++sign_index;
  }
  {
    GraphId node_id = CB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
    std::vector<SignInfo> signs = tile->GetSigns(CB_edge_id.id(), linguistics);
    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(),
              0); // signs are only in the forward direction due to trunk_link and oneway
    ASSERT_EQ(linguistics.size(), 0);
  }
}

TEST_P(PhonemesWithLangsTest, MultiPhonemes) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "secondary"},
            {"osm_id", "101"},
            {"name", "Rochor"},
            {"name:en", "Rochor"},
            {"name:ms", "Rochor"},
            {"name:ta", "ரோச்சோர்"},
            {"name:zh", "梧槽"},
            {"name:en" + param_tag, "English Language pronunciation"},
            {"name:zh" + param_tag, "Native zh Language pronunciation"},
            // removed for testing dropping of phonemes
            // {"name:ms" + param_tag, "Native ms Language pronunciation"},
            {"name:ta" + param_tag, "Native ta Language pronunciation"},
            {"name" + param_tag, "English Language pronunciation"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {103.87149, 1.32510});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  GraphId node_id = BC_edge->endnode();
  auto tile = graph_reader.GetGraphTile(node_id);
  {
    GraphId node_id = BC_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(BC_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 4);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());

      if (name_index == 0) {
        EXPECT_EQ(std::get<0>(name_and_type), "Rochor");
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "en");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "English Language pronunciation");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (name_index == 1) {
        EXPECT_EQ(std::get<0>(name_and_type), "梧槽");
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "zh");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Native zh Language pronunciation");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (name_index == 2) {
        EXPECT_EQ(std::get<0>(name_and_type), "Rochor");
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "ms");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNone));
      } else if (name_index == 3) {
        EXPECT_EQ(std::get<0>(name_and_type), "ரோச்சோர்");
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "ta");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Native ta Language pronunciation");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++name_index;
    }
  }

  {
    GraphId node_id = CB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(CB_edge);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);

    ASSERT_EQ(names_and_types.size(), 4);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(name_index);
      ASSERT_NE(iter, linguistics.end());
      if (name_index == 0) {
        EXPECT_EQ(std::get<0>(name_and_type), "Rochor");
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "en");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "English Language pronunciation");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (name_index == 1) {
        EXPECT_EQ(std::get<0>(name_and_type), "梧槽");
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "zh");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Native zh Language pronunciation");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (name_index == 2) {
        EXPECT_EQ(std::get<0>(name_and_type), "Rochor");
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "ms");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNone));
      } else if (name_index == 3) {
        EXPECT_EQ(std::get<0>(name_and_type), "ரோச்சோர்");
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "ta");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Native ta Language pronunciation");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++name_index;
    }
  }
}

TEST_P(PhonemesWithLangsTest, MultiPhonemes2) {
  const auto& param_tag = std::get<0>(GetParam());
  const auto& param_alphabet = std::get<1>(GetParam());
  CreateWorkdir();

  ways = {{"BC",
           {{"highway", "trunk"},
            {"osm_id", "101"},
            {"ref", "10"},
            {"destination", "Rochor"},
            {"destination:lang:en", "Rochor"},
            {"destination:lang:ms", "Rochor"},
            {"destination:lang:ta", "ரோச்சோர்"},
            {"destination:lang:zh", "梧槽"},
            {"destination:lang:en" + param_tag, "Rochor ipa"},
            // removed for testing dropping of phonems
            //{"destination:lang:ms" + param_tag, "Rochor ipa"},
            {"destination:lang:ta" + param_tag, "ரோச்சோர் ipa"},
            {"destination:lang:zh" + param_tag, "梧槽 ipa"}}}};

  layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {103.87149, 1.32510});
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);
  valhalla::gurka::map map;
  map.nodes = layout;
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));
  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
      findEdge(graph_reader, map.nodes, "", "C", baldr::GraphId{}, 101);
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);
  {
    GraphId node_id = BC_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
    std::vector<SignInfo> signs = tile->GetSigns(BC_edge_id.id(), linguistics);
    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 4);
    ASSERT_EQ(linguistics.size(), 4);

    for (const auto& sign : signs) {
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(sign_index);
      ASSERT_NE(iter, linguistics.end());

      if (sign_index == 0) {
        EXPECT_EQ(signs.at(sign_index).text(), "Rochor");
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "en");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "Rochor ipa");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (sign_index == 1) {
        EXPECT_EQ(signs.at(sign_index).text(), "梧槽");

        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "zh");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "梧槽 ipa");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (sign_index == 2) {
        EXPECT_EQ(signs.at(sign_index).text(), "Rochor");

        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "ms");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNone));
      } else if (sign_index == 3) {
        EXPECT_EQ(signs.at(sign_index).text(), "ரோச்சோர்");

        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "ta");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "ரோச்சோர் ipa");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++sign_index;
    }
  }
  {
    GraphId node_id = CB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> linguistics;
    std::vector<SignInfo> signs = tile->GetSigns(CB_edge_id.id(), linguistics);
    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 4);
    ASSERT_EQ(linguistics.size(), 4);

    for (const auto& sign : signs) {
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          linguistics.find(sign_index);
      ASSERT_NE(iter, linguistics.end());

      if (sign_index == 0) {
        EXPECT_EQ(signs.at(sign_index).text(), "Rochor");

        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "en");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "Rochor ipa");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (sign_index == 1) {
        EXPECT_EQ(signs.at(sign_index).text(), "梧槽");

        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "zh");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "梧槽 ipa");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      } else if (sign_index == 2) {
        EXPECT_EQ(signs.at(sign_index).text(), "Rochor");

        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "ms");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNone));
      } else if (sign_index == 3) {
        EXPECT_EQ(signs.at(sign_index).text(), "ரோச்சோர்");

        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "ta");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "ரோச்சோர் ipa");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(param_alphabet));
      }
      ++sign_index;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    PhonemesWithLangsTest,
    PhonemesWithLangsTest,
    ::testing::Values(std::make_tuple(":pronunciation", PronunciationAlphabet::kIpa),
                      std::make_tuple(":pronunciation:jeita", PronunciationAlphabet::kJeita),
                      std::make_tuple(":pronunciation:katakana", PronunciationAlphabet::kKatakana),
                      std::make_tuple(":pronunciation:nt-sampa", PronunciationAlphabet::kNtSampa)));
