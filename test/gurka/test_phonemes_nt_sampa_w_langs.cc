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

valhalla::gurka::map BuildPBFNames(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"name", "Chaussée de Gand - Steenweg op Gent/Gentsesteenweg"},
        {"name:fr", "Chaussée de Gand"},
        {"name:left", "Chaussée de Gand - Gentsesteenweg"},
        {"name:left:nl", "Gentsesteenweg"},
        {"name:right", "Chaussée de Gand - Steenweg op Gent"},
        {"name:right:nl", "Steenweg op Gent"},

        {"name:pronunciation:nt-sampa", "Chaussée de Gand P - Steenweg op Gent P/Gentsesteenweg P"},
        {"name:fr:pronunciation:nt-sampa", "Chaussée de Gand P"},
        {"name:left:pronunciation:nt-sampa", "Chaussée de Gand P - Gentsesteenweg P"},
        {"name:left:nl:pronunciation:nt-sampa", "Gentsesteenweg P"},
        {"name:right:pronunciation:nt-sampa", "Chaussée de Gand P - Steenweg op Gent P"},
        {"name:right:nl:pronunciation:nt-sampa", "Steenweg op Gent P"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, Names) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFNames(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      } else if (name_index == 1) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Steenweg op Gent P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      } else if (name_index == 1) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "Gentsesteenweg P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      }

      ++name_index;
    }
  }
}

valhalla::gurka::map BuildPBFAlts(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"alt_name", "Chaussée de Gand - Steenweg op Gent/Gentsesteenweg"},
        {"alt_name:fr", "Chaussée de Gand"},
        {"alt_name:left", "Chaussée de Gand - Gentsesteenweg"},
        {"alt_name:left:nl", "Gentsesteenweg"},
        {"alt_name:right", "Chaussée de Gand - Steenweg op Gent"},
        {"alt_name:right:nl", "Steenweg op Gent"},

        {"alt_name:pronunciation:nt-sampa",
         "Chaussée de Gand P - Steenweg op Gent P/Gentsesteenweg P"},
        {"alt_name:fr:pronunciation:nt-sampa", "Chaussée de Gand P"},
        {"alt_name:left:pronunciation:nt-sampa", "Chaussée de Gand P - Gentsesteenweg P"},
        {"alt_name:left:nl:pronunciation:nt-sampa", "Gentsesteenweg P"},
        {"alt_name:right:pronunciation:nt-sampa", "Chaussée de Gand P - Steenweg op Gent P"},
        {"alt_name:right:nl:pronunciation:nt-sampa", "Steenweg op Gent P"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, Alts) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFAlts(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));

      } else if (name_index == 2) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Steenweg op Gent P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));

      } else if (name_index == 2) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "Gentsesteenweg P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      }

      ++name_index;
    }
  }
}

valhalla::gurka::map BuildPBFOfficial(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"official_name", "Chaussée de Gand - Steenweg op Gent/Gentsesteenweg"},
        {"official_name:fr", "Chaussée de Gand"},
        {"official_name:left", "Chaussée de Gand - Gentsesteenweg"},
        {"official_name:left:nl", "Gentsesteenweg"},
        {"official_name:right", "Chaussée de Gand - Steenweg op Gent"},
        {"official_name:right:nl", "Steenweg op Gent"},

        {"official_name:pronunciation:nt-sampa",
         "Chaussée de Gand P - Steenweg op Gent P/Gentsesteenweg P"},
        {"official_name:fr:pronunciation:nt-sampa", "Chaussée de Gand P"},
        {"official_name:left:pronunciation:nt-sampa", "Chaussée de Gand P - Gentsesteenweg P"},
        {"official_name:left:nl:pronunciation:nt-sampa", "Gentsesteenweg P"},
        {"official_name:right:pronunciation:nt-sampa", "Chaussée de Gand P - Steenweg op Gent P"},
        {"official_name:right:nl:pronunciation:nt-sampa", "Steenweg op Gent P"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, Official) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFOfficial(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));

      } else if (name_index == 2) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Steenweg op Gent P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));

      } else if (name_index == 2) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "Gentsesteenweg P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      }

      ++name_index;
    }
  }
}

valhalla::gurka::map BuildPBFTunnel(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"tunnel:name", "Chaussée de Gand - Steenweg op Gent/Gentsesteenweg"},
        {"tunnel:name:fr", "Chaussée de Gand"},
        {"tunnel:name:left", "Chaussée de Gand - Gentsesteenweg"},
        {"tunnel:name:left:nl", "Gentsesteenweg"},
        {"tunnel:name:right", "Chaussée de Gand - Steenweg op Gent"},
        {"tunnel:name:right:nl", "Steenweg op Gent"},

        {"tunnel:name:pronunciation:nt-sampa",
         "Chaussée de Gand P - Steenweg op Gent P/Gentsesteenweg P"},
        {"tunnel:name:fr:pronunciation:nt-sampa", "Chaussée de Gand P"},
        {"tunnel:name:left:pronunciation:nt-sampa", "Chaussée de Gand P - Gentsesteenweg P"},
        {"tunnel:name:left:nl:pronunciation:nt-sampa", "Gentsesteenweg P"},
        {"tunnel:name:right:pronunciation:nt-sampa", "Chaussée de Gand P - Steenweg op Gent P"},
        {"tunnel:name:right:nl:pronunciation:nt-sampa", "Steenweg op Gent P"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, Tunnel) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFTunnel(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));

      } else if (name_index == 2) {

        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Steenweg op Gent P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));

      } else if (name_index == 2) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "Gentsesteenweg P");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      }

      ++name_index;
    }
  }
}

valhalla::gurka::map BuildPBFFB(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {{"BC",
                             {{"highway", "trunk"},
                              {"osm_id", "101"},
                              {"name", "Gentsesteenweg;Chaussée de Gand"},
                              {"name:forward", "Gentsesteenweg"},
                              {"name:forward:nl", "Gentsesteenweg"},
                              {"name:backward", "Chaussée de Gand"},
                              {"name:backward:fr", "Chaussée de Gand"},

                              {"name:pronunciation:nt-sampa", "Gentsesteenweg P;Chaussée de Gand P"},
                              {"name:forward:pronunciation:nt-sampa", "Gentsesteenweg P"},
                              {"name:forward:nl:pronunciation:nt-sampa", "Gentsesteenweg P"},
                              {"name:backward:pronunciation:nt-sampa", "Chaussée de Gand P"},
                              {"name:backward:fr:pronunciation:nt-sampa", "Chaussée de Gand P"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, NamesFB) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFFB(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      }
      ++name_index;
    }
  }
}

valhalla::gurka::map BuildPBFRefLR(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {{"BC",
                             {{"highway", "trunk"},
                              {"osm_id", "101"},
                              {"ref", "10;11"},
                              {"ref:right", "10"},
                              {"ref:right:nl", "10"},
                              {"ref:left", "11"},
                              {"ref:left:fr", "11"},

                              {"ref:pronunciation:nt-sampa", "10 P;11 P"},
                              {"ref:right:pronunciation:nt-sampa", "10 P"},
                              {"ref:right:nl:pronunciation:nt-sampa", "10 P"},
                              {"ref:left:pronunciation:nt-sampa", "11 P"},
                              {"ref:left:fr:pronunciation:nt-sampa", "11 P"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, RefLR) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFRefLR(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      }
      break;
    }
  }
}

valhalla::gurka::map BuildPBFDestinations(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"ref", "10"},
        {"destination", "destination:lang:nl"},
        // {"destination:lang:nl", "destination:lang:nl"}, not needed as lang is in next tag
        {"destination:lang:nl:pronunciation:nt-sampa",
         "destination:lang:nl:pronunciation:nt-sampa"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, Destinations) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFDestinations(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                "destination:lang:nl:pronunciation:nt-sampa");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
    }
    ++sign_index;
  }
}

valhalla::gurka::map BuildPBFDestinationStreet(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"ref", "10"},
        {"destination:street", "destination:lang:nl"},
        //{"destination:street:lang:nl", "destination:lang:nl"}, //not needed as lang is in next tag
        {"destination:street:lang:nl:pronunciation:nt-sampa",
         "destination:lang:nl:pronunciation:nt-sampa"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, DestinationStreet) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFDestinationStreet(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                "destination:lang:nl:pronunciation:nt-sampa");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
    }
    ++sign_index;
  }
}

valhalla::gurka::map BuildPBFDestinationStreetTo(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {{"BC",
                             {{"highway", "trunk"},
                              {"osm_id", "101"},
                              {"ref", "10"},
                              {"destination:street:to", "destination:lang:nl"},
                              //{"destination:street:to:lang:nl", "destination:lang:nl"}, //not needed
                              // as lang is in next tag
                              {"destination:street:to:lang:nl:pronunciation:nt-sampa",
                               "destination:lang:nl:pronunciation:nt-sampa"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, DestinationStreetTo) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFDestinationStreetTo(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                "destination:lang:nl:pronunciation:nt-sampa");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
    }
    ++sign_index;
  }
}

valhalla::gurka::map BuildPBFDestinationsFB(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"ref", "10"},
        {"destination:forward", "destination:forward:nl"},
        {"destination:backward", "destination:backward:fr"},
        {"destination:forward:pronunciation:nt-sampa",
         "destination:forward:lang:nl:pronunciation:nt-sampa"}, // this is really optional
        {"destination:backward:pronunciation:nt-sampa",
         "destination:backward:lang:fr:pronunciation:nt-sampa"}, // this is really optional
        {"destination:forward:lang:nl:pronunciation:nt-sampa",
         "destination:forward:lang:nl:pronunciation:nt-sampa"},
        {"destination:backward:lang:fr:pronunciation:nt-sampa",
         "destination:backward:lang:fr:pronunciation:nt-sampa"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, DestinationsFB) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFDestinationsFB(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                  "destination:forward:lang:nl:pronunciation:nt-sampa");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
                    "destination:backward:lang:fr:pronunciation:nt-sampa");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        }
        ++sign_index;
      }
    }
  }
}

valhalla::gurka::map BuildPBFDestinationsMultiLangs(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"ref", "10"},
        {"destination:lang:nl", "destination in dutch"},
        {"destination:lang:fr", "destination in french"},
        {"destination:lang:nl:pronunciation:nt-sampa", "destination pronunciation in dutch"},
        {"destination:lang:fr:pronunciation:nt-sampa", "destination pronunciation in french"}}}};

  constexpr double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.34999, 50.84643});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, DestinationsMultiLangs) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFDestinationsMultiLangs(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      } else if (sign_index == 1) {
        EXPECT_EQ(signs.at(sign_index).text(), "destination in dutch");

        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "destination pronunciation in dutch");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else if (sign_index == 1) {
          EXPECT_EQ(signs.at(sign_index).text(), "destination in dutch");

          EXPECT_EQ(to_string(static_cast<Language>(
                        std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                    "nl");
          EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                    "destination pronunciation in dutch");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        }
        ++sign_index;
      }
    }
  }
}

valhalla::gurka::map BuildPBFDestinationsFBNoOptionalTag(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"ref", "10"},
        {"destination:forward", "destination:forward:nl"},
        {"destination:backward", "destination:backward:fr"},
        //{"destination:forward:pronunciation:nt-sampa",
        //"destination:forward:lang:nl:pronunciation:nt-sampa"}, //this is really optional
        //{"destination:backward:pronunciation:nt-sampa",
        //"destination:backward:lang:fr:pronunciation:nt-sampa"}, //this is really optional
        {"destination:forward:lang:nl:pronunciation:nt-sampa",
         "destination:forward:lang:nl:pronunciation:nt-sampa"},
        {"destination:backward:lang:fr:pronunciation:nt-sampa",
         "destination:backward:lang:fr:pronunciation:nt-sampa"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, DestinationsFBNoOptionalTag) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFDestinationsFBNoOptionalTag(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                  "destination:forward:lang:nl:pronunciation:nt-sampa");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
                    "destination:backward:lang:fr:pronunciation:nt-sampa");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        }
        ++sign_index;
      }
    }
  }
}

valhalla::gurka::map BuildPBFDestinationRef(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"ref", "10"},
        {"destination:ref", "destination:lang:nl"},
        //{"destination:ref:lang:nl", "destination:lang:nl"}, //not needed as lang is in next tag
        {"destination:ref:lang:nl:pronunciation:nt-sampa",
         "destination:lang:nl:pronunciation:nt-sampa"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, DestinationRef) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFDestinationRef(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                "destination:lang:nl:pronunciation:nt-sampa");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
    }
    ++sign_index;
  }
}

valhalla::gurka::map BuildPBFDestinationRefTo(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {
      {"BC",
       {{"highway", "trunk"},
        {"osm_id", "101"},
        {"ref", "10"},
        {"destination:ref:to", "destination:lang:nl"},
        //{"destination:ref:to:lang:nl", "destination:lang:nl"}, //not needed as lang is in next tag
        {"destination:ref:to:lang:nl:pronunciation:nt-sampa",
         "destination:lang:nl:pronunciation:nt-sampa"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, DestinationRefTo) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFDestinationRefTo(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                "destination:lang:nl:pronunciation:nt-sampa");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
    }
    ++sign_index;
  }
}

valhalla::gurka::map BuildPBFJunctionRef(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {
      {"BC",
       {{"highway", "trunk_link"},
        {"osm_id", "101"},
        {"ref", "10"},
        {"junction:ref", "junction:nl"},
        //{"junction:ref:nl", "junction:ref:nl"}, //not needed as lang is in next tag
        {"junction:ref:nl:pronunciation:nt-sampa", "junction:nl:pronunciation:nt-sampa"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, DestinationJunctionRef) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFJunctionRef(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                "junction:nl:pronunciation:nt-sampa");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
    }
    ++sign_index;
  }
}

valhalla::gurka::map BuildPBFNodeRef(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {{"BC", {{"highway", "trunk_link"}, {"osm_id", "101"}}}};

  const gurka::nodes nodes = {{"B",
                               {{"ref", "node ref"},
                                {"highway", "motorway_junction"},
                                {"ref:nl:pronunciation:nt-sampa", "ref:nl:pronunciation:nt-sampa"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {4.3516970, 50.8465573});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, nodes, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, NodeRef) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFNodeRef(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                "ref:nl:pronunciation:nt-sampa");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
    }
    ++sign_index;
  }
}

valhalla::gurka::map BuildPBFNodeName(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {{"BC", {{"highway", "trunk"}, {"osm_id", "101"}}}};

  const gurka::nodes nodes = {
      {"C",
       {{"highway", "traffic_signals"},
        {"name", "両国二丁目"},
        {"name:ja", "両国二丁目"},
        {"name:en", "Ryogoku 2-chome"},
        {"name:ja:pronunciation:nt-sampa", "両国二丁目 pronunciation"},
        {"name:en:pronunciation:nt-sampa", "Ryogoku 2-chome pronunciation"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {139.79079, 35.69194});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, nodes, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, NodeName) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFNodeName(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
    } else if (sign_index == 1) {

      EXPECT_EQ(signs.at(sign_index).text(), "Ryogoku 2-chome");

      EXPECT_EQ(to_string(
                    static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                "en");
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                "Ryogoku 2-chome pronunciation");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
    }
    ++sign_index;
  }
}

valhalla::gurka::map BuildPBFNamesCanada(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {{"BC",
                             {{"highway", "trunk"},
                              {"osm_id", "101"},
                              {"name:en:pronunciation:nt-sampa", "dV|fi \"lek *\"rod"},
                              {"name:pronunciation:nt-sampa", "dV|fi \"lek *\"rod"},
                              {"name:en", "Duffy Lake Road"},
                              {"name", "Duffy Lake Road"},
                              {"ref:pronunciation:nt-sampa", "haI|%we \"naIn|ti *\"naIn \"nORt"},
                              {"ref", "HWY-99"}}}};

  constexpr double gridsize = 100;

  // note:  In Canada which is multilingual.  Must specify the lang
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {-121.9281, 50.6827});

  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, NamesPart2) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFNamesCanada(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.data_processing.use_admin_db", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      } else if (name_index == 1) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "en");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "dV|fi \"lek *\"rod");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      } else if (name_index == 1) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "en");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "dV|fi \"lek *\"rod");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      }
      ++name_index;
    }
  }
}

valhalla::gurka::map BuildPBFOldDataDestination(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {{"BC",
                             {{"highway", "trunk"},
                              {"osm_id", "101"},
                              {"ref", "10"},
                              {"destination", "destination in ja"},
                              {"destination:en", "destination in en"},
                              {"destination:pronunciation:nt-sampa", "nt-sampa phoneme"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {139.79079, 35.69194});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, OldDataDestination) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFOldDataDestination(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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

valhalla::gurka::map BuildPBFForwardDestination(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {{"BC",
                             {{"highway", "trunk"},
                              {"osm_id", "101"},
                              {"ref", "10"},
                              {"destination:forward:lang:en", "Koriyama"},
                              {"destination:forward", "郡山"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {139.79079, 35.69194});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, ForwardDestination) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFForwardDestination(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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

valhalla::gurka::map BuildPBFJunction(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {{"BC",
                             {{"highway", "trunk_link"},
                              {"osm_id", "101"},
                              {"oneway", "yes"},
                              {"ref", "10"},
                              {"junction:name:en", "Iwakimiwa IC"},
                              {"junction:name", "いわき三和ＩＣ"},
                              {"junction:name:ja:pronunciation:nt-sampa", "ja nt-sampa"},
                              {"junction:name:ja", "いわき三和ＩＣ"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {139.79079, 35.69194});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, Junction) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFJunction(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
      EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "ja nt-sampa");
      EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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

valhalla::gurka::map BuildPBFMultiPhonemes(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {
      {"BC",
       {{"highway", "secondary"},
        {"osm_id", "101"},
        {"name", "Rochor"},
        {"name:en", "Rochor"},
        {"name:ms", "Rochor"},
        {"name:ta", "ரோச்சோர்"},
        {"name:zh", "梧槽"},
        {"name:en:pronunciation:nt-sampa", "English Language pronunciation"},
        {"name:zh:pronunciation:nt-sampa", "Native zh Language pronunciation"},
        // removed for testing dropping of phonems
        // {"name:ms:pronunciation:nt-sampa", "Native ms Language pronunciation"},
        {"name:ta:pronunciation:nt-sampa", "Native ta Language pronunciation"},
        {"name:pronunciation:nt-sampa", "English Language pronunciation"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {103.87149, 1.32510});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, MultiPhonemes) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFMultiPhonemes(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      } else if (name_index == 1) {
        EXPECT_EQ(std::get<0>(name_and_type), "梧槽");
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "zh");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Native zh Language pronunciation");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      } else if (name_index == 1) {
        EXPECT_EQ(std::get<0>(name_and_type), "梧槽");
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "zh");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second),
                  "Native zh Language pronunciation");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      }
      ++name_index;
    }
  }
}

valhalla::gurka::map BuildPBFMultiPhonemes2(const std::string& workdir) {
  const std::string ascii_map = R"(
      B----C
  )";

  const gurka::ways ways = {{"BC",
                             {{"highway", "trunk"},
                              {"osm_id", "101"},
                              {"ref", "10"},
                              {"destination", "Rochor"},
                              {"destination:lang:en", "Rochor"},
                              {"destination:lang:ms", "Rochor"},
                              {"destination:lang:ta", "ரோச்சோர்"},
                              {"destination:lang:zh", "梧槽"},
                              {"destination:lang:en:pronunciation:nt-sampa", "Rochor nt-sampa"},
                              // removed for testing dropping of phonems
                              //{"destination:lang:ms:pronunciation:nt-sampa", "Rochor nt-sampa"},
                              {"destination:lang:ta:pronunciation:nt-sampa", "ரோச்சோர் nt-sampa"},
                              {"destination:lang:zh:pronunciation:nt-sampa", "梧槽 nt-sampa"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {103.87149, 1.32510});
  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, MultiPhonemes2) {

  const std::string workdir = "test/data/gurka_phonemes_nt_sampa_w_langs";

  if (filesystem::is_directory(workdir)) {
    filesystem::remove_all(workdir);
  }

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBFMultiPhonemes2(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

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
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "Rochor nt-sampa");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      } else if (sign_index == 1) {
        EXPECT_EQ(signs.at(sign_index).text(), "梧槽");

        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "zh");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "梧槽 nt-sampa");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "ரோச்சோர் nt-sampa");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "Rochor nt-sampa");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      } else if (sign_index == 1) {
        EXPECT_EQ(signs.at(sign_index).text(), "梧槽");

        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "zh");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "梧槽 nt-sampa");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
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
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "ரோச்சோர் nt-sampa");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      }
      ++sign_index;
    }
  }
}
