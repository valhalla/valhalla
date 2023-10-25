#include "baldr/graphreader.h"
#include "filesystem.h"
#include "mjolnir/util.h"

#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

valhalla::gurka::map BuildPBF(const std::string& workdir) {
  const std::string ascii_map = R"(
      A----B----C
            \
             D-------E-------F-------G-------H-------I-------J-------K-------L-------M-------N
  )";

  const gurka::ways ways =
      {{"AB",
        {{"highway", "trunk"},
         {"name", "AB"},
         {"ref", "ref"},
         {"int_ref", "int_ref"},
         {"direction", "direction"},
         {"int_direction", "int_direction"},
         {"alt_name", "alt_name"},
         {"official_name", "official_name"},
         {"name:en", "name:en"},
         {"tunnel:name", "tunnel:name"},
         {"name:pronunciation", "name:pronunciation"},
         {"ref:pronunciation", "ref:pronunciation"},
         {"int_ref:pronunciation", "int_ref:pronunciation"},
         {"direction:pronunciation", "direction:pronunciation"},
         {"int_direction:pronunciation", "int_direction:pronunciation"},
         {"alt_name:pronunciation", "alt_name:pronunciation"},
         {"official_name:pronunciation", "official_name:pronunciation"},
         {"tunnel:name:pronunciation", "tunnel:name:pronunciation"},
         {"name:en:pronunciation", "name:en:pronunciation"},
         {"name:pronunciation:nt-sampa", "name:pronunciation:nt-sampa"},
         {"ref:pronunciation:nt-sampa", "ref:pronunciation:nt-sampa"},
         {"int_ref:pronunciation:nt-sampa", "int_ref:pronunciation:nt-sampa"},
         {"direction:pronunciation:nt-sampa", "direction:pronunciation:nt-sampa"},
         {"int_direction:pronunciation:nt-sampa", "int_direction:pronunciation:nt-sampa"},
         {"alt_name:pronunciation:nt-sampa", "alt_name:pronunciation:nt-sampa"},
         {"official_name:pronunciation:nt-sampa", "official_name:pronunciation:nt-sampa"},
         {"tunnel:name:pronunciation:nt-sampa", "tunnel:name:pronunciation:nt-sampa"},
         {"name:en:pronunciation:nt-sampa", "name:en:pronunciation:nt-sampa"},
         {"name:pronunciation:katakana", "name:pronunciation:katakana"},
         {"ref:pronunciation:katakana", "ref:pronunciation:katakana"},
         {"int_ref:pronunciation:katakana", "int_ref:pronunciation:katakana"},
         {"direction:pronunciation:katakana", "direction:pronunciation:katakana"},
         {"int_direction:pronunciation:katakana", "int_direction:pronunciation:katakana"},
         {"alt_name:pronunciation:katakana", "alt_name:pronunciation:katakana"},
         {"official_name:pronunciation:katakana", "official_name:pronunciation:katakana"},
         {"tunnel:name:pronunciation:katakana", "tunnel:name:pronunciation:katakana"},
         {"name:en:pronunciation:katakana", "name:en:pronunciation:katakana"},
         {"name:pronunciation:jeita", "name:pronunciation:jeita"},
         {"ref:pronunciation:jeita", "ref:pronunciation:jeita"},
         {"int_ref:pronunciation:jeita", "int_ref:pronunciation:jeita"},
         {"direction:pronunciation:jeita", "direction:pronunciation:jeita"},
         {"int_direction:pronunciation:jeita", "int_direction:pronunciation:jeita"},
         {"alt_name:pronunciation:jeita", "alt_name:pronunciation:jeita"},
         {"official_name:pronunciation:jeita", "official_name:pronunciation:jeita"},
         {"tunnel:name:pronunciation:jeita", "tunnel:name:pronunciation:jeita"},
         {"name:en:pronunciation:jeita", "name:en:pronunciation:jeita"},
         {"destination", "destination"},
         {"destination:pronunciation", "destination:pronunciation"},
         {"destination:pronunciation:nt-sampa", "destination:pronunciation:nt-sampa"},
         {"destination:pronunciation:katakana", "destination:pronunciation:katakana"},
         {"destination:pronunciation:jeita", "destination:pronunciation:jeita"}}},
       {"BC",
        {{"highway", "trunk"},
         {"name", "BC"},
         {"alt_name", "alt_name"},
         {"official_name", "official_name"},
         {"name:en", "name:en"},
         {"name:pronunciation", "name:pronunciation"},
         {"alt_name:pronunciation:nt-sampa", "alt_name:pronunciation:nt-sampa"},
         {"alt_name:pronunciation:katakana", "alt_name:pronunciation:katakana"},
         {"official_name:pronunciation:katakana", "official_name:pronunciation:katakana"},
         {"official_name:pronunciation:jeita", "official_name:pronunciation:jeita"},
         {"name:en:pronunciation", "name:en:pronunciation"},
         {"name:en:pronunciation:katakana", "name:en:pronunciation:katakana"},
         {"destination:forward", "destination:forward"},
         {"destination:backward", "destination:backward"},
         {"destination:ref", "destination:ref"},
         {"destination:ref:to", "destination:ref:to"},
         {"destination:street", "destination:street"},
         {"destination:street:to", "destination:street:to"},
         {"junction:ref", "junction:ref"},
         {"destination:forward:pronunciation", "destination:forward:pronunciation"},
         {"destination:backward:pronunciation", "destination:backward:pronunciation"},
         {"destination:ref:pronunciation", "destination:ref:pronunciation"},
         {"destination:ref:to:pronunciation", "destination:ref:to:pronunciation"},
         {"destination:street:pronunciation", "destination:street:pronunciation"},
         {"destination:street:to:pronunciation", "destination:street:to:pronunciation"},
         {"junction:ref:pronunciation", "junction:ref:pronunciation"},
         {"destination:forward:pronunciation:katakana", "destination:forward:pronunciation:katakana"},
         {"destination:backward:pronunciation:katakana",
          "destination:backward:pronunciation:katakana"},
         {"destination:ref:pronunciation:katakana", "destination:ref:pronunciation:katakana"},
         {"destination:ref:to:pronunciation:katakana", "destination:ref:to:pronunciation:katakana"},
         {"destination:street:pronunciation:katakana", "destination:street:pronunciation:katakana"},
         {"destination:street:to:pronunciation:katakana",
          "destination:street:to:pronunciation:katakana"},
         {"junction:ref:pronunciation:katakana", "junction:ref:pronunciation:katakana"},
         {"destination:forward:pronunciation:jeita", "destination:forward:pronunciation:jeita"},
         {"destination:backward:pronunciation:jeita", "destination:backward:pronunciation:jeita"},
         {"destination:ref:pronunciation:jeita", "destination:ref:pronunciation:jeita"},
         {"destination:ref:to:pronunciation:jeita", "destination:ref:to:pronunciation:jeita"},
         {"destination:street:pronunciation:jeita", "destination:street:pronunciation:jeita"},
         {"destination:street:to:pronunciation:jeita", "destination:street:to:pronunciation:jeita"},
         {"junction:ref:pronunciation:jeita", "junction:ref:pronunciation:jeita"}}},
       {"BD",
        {{"highway", "trunk"},
         {"destination:forward", "destination:forward"},
         {"destination:backward", "destination:backward"},
         {"destination:ref", "destination:ref"},
         {"destination:ref:to", "destination:ref:to"},
         {"destination:street", "destination:street"},
         {"destination:street:to", "destination:street:to"},
         {"junction:ref", "junction:ref"},
         {"destination:forward:pronunciation", "destination:forward:pronunciation"},
         {"destination:backward:pronunciation", "destination:backward:pronunciation"},
         {"destination:ref:pronunciation", "destination:ref:pronunciation"},
         {"destination:ref:to:pronunciation", "destination:ref:to:pronunciation"},
         {"destination:street:pronunciation", "destination:street:pronunciation"},
         {"destination:street:to:pronunciation", "destination:street:to:pronunciation"},
         {"junction:ref:pronunciation", "junction:ref:pronunciation"},
         {"destination:forward:pronunciation:katakana", "destination:forward:pronunciation:katakana"},
         {"destination:backward:pronunciation:katakana",
          "destination:backward:pronunciation:katakana"},
         {"destination:ref:pronunciation:katakana", "destination:ref:pronunciation:katakana"},
         {"destination:ref:to:pronunciation:katakana", "destination:ref:to:pronunciation:katakana"},
         {"destination:street:pronunciation:katakana", "destination:street:pronunciation:katakana"},
         {"destination:street:to:pronunciation:katakana",
          "destination:street:to:pronunciation:katakana"},
         {"junction:ref:pronunciation:katakana", "junction:ref:pronunciation:katakana"}}},
       {"DE",
        {{"highway", "primary"},
         {"name", "DE;xyz street;abc ave"},
         {"name:pronunciation", ";;name:pronunciation3"},
         {"destination", "destination1;destination2"},
         {"destination:pronunciation", "destination:pronunciation1;destination:pronunciation2"}}},
       {"EF",
        {{"highway", "primary_link"},
         {"name", "EF;xyz street;abc ave"},
         {"name:pronunciation", ";name:pronunciation2;"},
         {"oneway", "yes"},
         {"destination", "destination1;destination2"},
         {"destination:pronunciation", ";"}}},
       {"FG",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"name", "FG;xyz street;abc ave"},
         {"name:pronunciation", "name:pronunciation1;;"},
         {"destination", "destination1;destination2;destination3"},
         {"destination:pronunciation", ";;destination:pronunciation3"}}},
       {"GH",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"name", "GH;xyz street;abc ave"},
         {"name:pronunciation", ";name:pronunciation2;name:pronunciation3"},
         {"destination", "destination1;destination2;destination3"},
         {"destination:pronunciation", ";destination:pronunciation2;"}}},
       {"HI",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"name", "HI;xyz street;abc ave"},
         {"name:pronunciation:katakana",
          ";name:pronunciation2:katakana;name:pronunciation3:katakana"},
         {"name:pronunciation:jeita", ";name:pronunciation2:jeita;name:pronunciation3:jeita"},
         {"destination", "destination1;destination2;destination3"},
         {"destination:pronunciation", ";destination:pronunciation2;destination:pronunciation3"}}},
       {"IJ",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"name:pronunciation:katakana", ";name:pronunciation1;;"},
         {"name:pronunciation:jeita", ";;name:pronunciation2"},
         {"destination", "destination1;destination2;destination3"},
         {"destination:pronunciation", "destination:pronunciation1;;destination:pronunciation3"}}},
       {"JK",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"destination", "destination1;destination2;destination3"},
         {"destination:pronunciation", "destination:pronunciation1;destination:pronunciation2;"}}},
       {"KL",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"destination", "destination1;destination2;destination3"},
         {"destination:pronunciation", "destination:pronunciation1;;"}}},
       {"LM",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"destination", "destination1;destination2;destination3"},
         {"destination:pronunciation:katakana", ";destination:pronunciation2:katakana;"},
         {"destination:pronunciation:jeita", ";destination:pronunciation2:jeita;"}}},
       {"MN",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"destination", "destination1;destination2;destination3"},
         {"destination:pronunciation:katakana", ";;destination:pronunciation3:katakana"},
         {"destination:pronunciation:jeita", ";destination:pronunciation2:jeita;"}}}};

  const gurka::nodes nodes =
      {{"B",
        {{"junction", "named"},
         {"name", "named junction"},
         {"highway", "motorway_junction"},
         {"name:pronunciation", "named junction:pronunciation"},
         {"name:pronunciation:nt-sampa", "named junction:pronunciation:nt-sampa"},
         {"name:pronunciation:katakana", "named junction:pronunciation:katakana"},
         {"name:pronunciation:jeita", "named junction:pronunciation:jeita"}}},
       {"E",
        {{"ref", "node ref"},
         {"highway", "motorway_junction"},
         {"ref:pronunciation", "node ref:pronunciation"},
         {"ref:pronunciation:katakana", "node ref:pronunciation:katakana"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.1079374, 52.0887174});

  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, nodes, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  return result;
}

TEST(Standalone, PhonemesWithAltandDirection) {

  const std::string workdir = "test/data/gurka_phonemes_alt_dir";

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBF(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "true");
  pt.put("mjolnir.data_processing.use_direction_on_ways", "true");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));

  GraphId AB_edge_id;
  const DirectedEdge* AB_edge = nullptr;
  GraphId BA_edge_id;
  const DirectedEdge* BA_edge = nullptr;
  std::tie(AB_edge_id, AB_edge, BA_edge_id, BA_edge) = findEdge(graph_reader, map.nodes, "AB", "B");
  EXPECT_NE(AB_edge, nullptr);
  EXPECT_NE(BA_edge, nullptr);

  GraphId BC_edge_id;
  const DirectedEdge* BC_edge = nullptr;
  GraphId CB_edge_id;
  const DirectedEdge* CB_edge = nullptr;
  std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) = findEdge(graph_reader, map.nodes, "BC", "C");
  EXPECT_NE(BC_edge, nullptr);
  EXPECT_NE(CB_edge, nullptr);

  GraphId BD_edge_id;
  const DirectedEdge* BD_edge = nullptr;
  GraphId DB_edge_id;
  const DirectedEdge* DB_edge = nullptr;
  std::tie(BD_edge_id, BD_edge, DB_edge_id, DB_edge) = findEdge(graph_reader, map.nodes, "BD", "D");
  EXPECT_NE(BD_edge, nullptr);
  EXPECT_NE(DB_edge, nullptr);

  GraphId DE_edge_id;
  const DirectedEdge* DE_edge = nullptr;
  GraphId ED_edge_id;
  const DirectedEdge* ED_edge = nullptr;
  std::tie(DE_edge_id, DE_edge, ED_edge_id, ED_edge) = findEdge(graph_reader, map.nodes, "DE", "E");
  EXPECT_NE(DE_edge, nullptr);
  EXPECT_NE(ED_edge, nullptr);

  GraphId EF_edge_id;
  const DirectedEdge* EF_edge = nullptr;
  GraphId FE_edge_id;
  const DirectedEdge* FE_edge = nullptr;
  std::tie(EF_edge_id, EF_edge, FE_edge_id, FE_edge) = findEdge(graph_reader, map.nodes, "EF", "F");
  EXPECT_NE(EF_edge, nullptr);
  EXPECT_NE(FE_edge, nullptr);

  GraphId FG_edge_id;
  const DirectedEdge* FG_edge = nullptr;
  GraphId GF_edge_id;
  const DirectedEdge* GF_edge = nullptr;
  std::tie(FG_edge_id, FG_edge, GF_edge_id, GF_edge) = findEdge(graph_reader, map.nodes, "FG", "G");
  EXPECT_NE(FG_edge, nullptr);
  EXPECT_NE(GF_edge, nullptr);

  GraphId GH_edge_id;
  const DirectedEdge* GH_edge = nullptr;
  GraphId HG_edge_id;
  const DirectedEdge* HG_edge = nullptr;
  std::tie(GH_edge_id, GH_edge, HG_edge_id, HG_edge) = findEdge(graph_reader, map.nodes, "GH", "H");
  EXPECT_NE(GH_edge, nullptr);
  EXPECT_NE(HG_edge, nullptr);

  GraphId HI_edge_id;
  const DirectedEdge* HI_edge = nullptr;
  GraphId IH_edge_id;
  const DirectedEdge* IH_edge = nullptr;
  std::tie(HI_edge_id, HI_edge, IH_edge_id, IH_edge) = findEdge(graph_reader, map.nodes, "HI", "I");
  EXPECT_NE(HI_edge, nullptr);
  EXPECT_NE(IH_edge, nullptr);

  GraphId IJ_edge_id;
  const DirectedEdge* IJ_edge = nullptr;
  GraphId JI_edge_id;
  const DirectedEdge* JI_edge = nullptr;
  std::tie(IJ_edge_id, IJ_edge, JI_edge_id, JI_edge) = findEdge(graph_reader, map.nodes, "IJ", "J");
  EXPECT_NE(IJ_edge, nullptr);
  EXPECT_NE(JI_edge, nullptr);

  GraphId JK_edge_id;
  const DirectedEdge* JK_edge = nullptr;
  GraphId KJ_edge_id;
  const DirectedEdge* KJ_edge = nullptr;
  std::tie(JK_edge_id, JK_edge, KJ_edge_id, KJ_edge) = findEdge(graph_reader, map.nodes, "JK", "K");
  EXPECT_NE(JK_edge, nullptr);
  EXPECT_NE(KJ_edge, nullptr);

  GraphId KL_edge_id;
  const DirectedEdge* KL_edge = nullptr;
  GraphId LK_edge_id;
  const DirectedEdge* LK_edge = nullptr;
  std::tie(KL_edge_id, KL_edge, LK_edge_id, LK_edge) = findEdge(graph_reader, map.nodes, "KL", "L");
  EXPECT_NE(KL_edge, nullptr);
  EXPECT_NE(LK_edge, nullptr);

  GraphId LM_edge_id;
  const DirectedEdge* LM_edge = nullptr;
  GraphId ML_edge_id;
  const DirectedEdge* ML_edge = nullptr;
  std::tie(LM_edge_id, LM_edge, ML_edge_id, ML_edge) = findEdge(graph_reader, map.nodes, "LM", "M");
  EXPECT_NE(LM_edge, nullptr);
  EXPECT_NE(ML_edge, nullptr);

  GraphId MN_edge_id;
  const DirectedEdge* MN_edge = nullptr;
  GraphId NM_edge_id;
  const DirectedEdge* NM_edge = nullptr;
  std::tie(MN_edge_id, MN_edge, NM_edge_id, NM_edge) = findEdge(graph_reader, map.nodes, "MN", "N");
  EXPECT_NE(MN_edge, nullptr);
  EXPECT_NE(NM_edge, nullptr);

  // Test the named junction on the node.  nt-sampa wins
  {
    GraphId node_id = AB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> index_linguistic_map;
    std::vector<SignInfo> signs = tile->GetSigns(node_id.id(), index_linguistic_map, true);

    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 1);
    ASSERT_EQ(index_linguistic_map.size(), 1);
    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);
      ASSERT_NE(iter, index_linguistic_map.end());

      if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
          "named junction:pronunciation:nt-sampa") {
        EXPECT_EQ(signs.at(sign_index).text(), "named junction");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation:nt-sampa") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
      } else
        FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
               << " Extra key. This should not happen.";
      ++sign_index;
    }
  }
  // Test the ref on the node katakana wins.  Also, test empty linguistics
  {

    GraphId node_id = EF_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(EF_edge);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> index_linguistic_map;

    std::vector<SignInfo> signs = tile->GetSigns(EF_edge_id.id(), index_linguistic_map);

    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 4);
    ASSERT_EQ(index_linguistic_map.size(), 4);

    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);
      if (iter == index_linguistic_map.end()) {
        if (sign_index == 1)
          EXPECT_EQ(signs.at(sign_index).text(), "destination1");
        if (sign_index == 2)
          EXPECT_EQ(signs.at(sign_index).text(), "destination2");
      } else {
        if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
            "node ref:pronunciation:katakana") {
          EXPECT_EQ(signs.at(sign_index).text(), "node ref");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
          EXPECT_EQ(to_string(static_cast<Language>(
                        std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                    "nl");
        } else {
          if (static_cast<valhalla::baldr::PronunciationAlphabet>(
                  std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)) ==
              PronunciationAlphabet::kNone) {
            EXPECT_EQ(to_string(static_cast<Language>(
                          std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                      "nl");
            EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
          } else
            FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                   << " Extra key. This should not happen.";
        }
      }
      ++sign_index;
    }

    // blank linguistics for names.
    std::vector<uint8_t> types;
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);
    ASSERT_EQ(names_and_types.size(), 3);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> name_linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          name_linguistics.find(name_index);
      ASSERT_NE(iter, name_linguistics.end());
      if (name_index == 0 || name_index == 2) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNone));
      } else {
        // first and last name is missing in the name field
        EXPECT_EQ(name_index, 1);
        if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) == "name:pronunciation2") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "xyz street");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kIpa));
        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      }
      ++name_index;
    }
  }

  // Test 2 empty linguistics
  {

    GraphId node_id = FG_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(FG_edge);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> index_linguistic_map;

    std::vector<SignInfo> signs = tile->GetSigns(FG_edge_id.id(), index_linguistic_map);

    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 3);
    ASSERT_EQ(index_linguistic_map.size(), 3);
    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);

      if (iter == index_linguistic_map.end()) {
        if (sign_index == 0)
          EXPECT_EQ(signs.at(sign_index).text(), "destination1");
        else if (sign_index == 1)
          EXPECT_EQ(signs.at(sign_index).text(), "destination2");
        else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation3") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination3");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if (static_cast<valhalla::baldr::PronunciationAlphabet>(
                     std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)) ==
                 PronunciationAlphabet::kNone) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");

      } else
        FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
               << " Extra key. This should not happen.";
      ++sign_index;
    }

    // blank linguistics for names.
    std::vector<uint8_t> types;
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);
    ASSERT_EQ(names_and_types.size(), 3);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> name_linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          name_linguistics.find(name_index);
      ASSERT_NE(iter, name_linguistics.end());
      if (name_index == 1 || name_index == 2) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNone));

      } else {
        // first and last name is missing in the name field
        EXPECT_EQ(name_index, 0);
        if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) == "name:pronunciation1") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "FG");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kIpa));
        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      }
      ++name_index;
    }
  }

  // Test 2 empty linguistics
  {

    GraphId node_id = GH_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(GH_edge);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> index_linguistic_map;

    std::vector<SignInfo> signs = tile->GetSigns(GH_edge_id.id(), index_linguistic_map);

    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 3);
    ASSERT_EQ(index_linguistic_map.size(), 3);
    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);

      if (iter == index_linguistic_map.end()) {
        if (sign_index == 0)
          EXPECT_EQ(signs.at(sign_index).text(), "destination1");
        else if (sign_index == 2)
          EXPECT_EQ(signs.at(sign_index).text(), "destination3");
        else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation2") {

        EXPECT_EQ(signs.at(sign_index).text(), "destination2");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else {
        if (static_cast<valhalla::baldr::PronunciationAlphabet>(
                std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)) ==
            PronunciationAlphabet::kNone) {
          EXPECT_EQ(to_string(static_cast<Language>(
                        std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                    "nl");
          EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");

        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      }
      ++sign_index;
    }

    // blank linguistics for names.
    std::vector<uint8_t> types;
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);
    ASSERT_EQ(names_and_types.size(), 3);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> name_linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          name_linguistics.find(name_index);
      ASSERT_NE(iter, name_linguistics.end());
      if (name_index == 0) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNone));

      } else {
        if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) == "name:pronunciation2") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "xyz street");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kIpa));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "name:pronunciation3") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "abc ave");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kIpa));
        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      }
      ++name_index;
    }
  }

  // Test 1 empty pronunciation
  {

    GraphId node_id = HI_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(HI_edge);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> index_linguistic_map;

    std::vector<SignInfo> signs = tile->GetSigns(HI_edge_id.id(), index_linguistic_map);

    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 3);
    ASSERT_EQ(index_linguistic_map.size(), 3);

    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);

      if (iter == index_linguistic_map.end()) {
        if (sign_index == 0)
          EXPECT_EQ(signs.at(sign_index).text(), "destination1");
        else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation2") {

        EXPECT_EQ(signs.at(sign_index).text(), "destination2");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation3") {

        EXPECT_EQ(signs.at(sign_index).text(), "destination3");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else {
        if (static_cast<valhalla::baldr::PronunciationAlphabet>(
                std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)) ==
            PronunciationAlphabet::kNone) {
          EXPECT_EQ(to_string(static_cast<Language>(
                        std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                    "nl");
          EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      }
      ++sign_index;
    }

    // blank linguistics for names.
    std::vector<uint8_t> types;
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);
    ASSERT_EQ(names_and_types.size(), 3);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> name_linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          name_linguistics.find(name_index);
      ASSERT_NE(iter, name_linguistics.end());
      if (name_index == 0) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNone));

      } else {
        if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
            "name:pronunciation2:jeita") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "xyz street");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kJeita));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "name:pronunciation3:jeita") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "abc ave");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kJeita));
        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      }
      ++name_index;
    }
  }

  // Test 1 empty pronunciation
  {

    GraphId node_id = IJ_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(IJ_edge);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> index_linguistic_map;

    std::vector<SignInfo> signs = tile->GetSigns(IJ_edge_id.id(), index_linguistic_map);

    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 3);
    ASSERT_EQ(index_linguistic_map.size(), 3);
    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);

      if (iter == index_linguistic_map.end()) {
        if (sign_index == 1)
          EXPECT_EQ(signs.at(sign_index).text(), "destination2");
        else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation1") {

        EXPECT_EQ(signs.at(sign_index).text(), "destination1");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation3") {

        EXPECT_EQ(signs.at(sign_index).text(), "destination3");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else {
        if (static_cast<valhalla::baldr::PronunciationAlphabet>(
                std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)) ==
            PronunciationAlphabet::kNone) {
          EXPECT_EQ(to_string(static_cast<Language>(
                        std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                    "nl");
          EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      }
      ++sign_index;
    }

    // blank linguistics for names.
    std::vector<uint8_t> types;
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);
    ASSERT_EQ(names_and_types.size(), 1);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> name_linguistics =
        edgeinfo.GetLinguisticMap();
    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      if (std::get<2>(name_and_type) != 0) {
        // Skip the tagged names
        ++name_index;
        continue;
      }
      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          name_linguistics.find(name_index);
      ASSERT_NE(iter, name_linguistics.end());
      if (name_index == 0) {
        EXPECT_EQ(to_string(static_cast<Language>(
                      std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                  "nl");
        EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kNone));
      } else {
        if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
            "name:pronunciation1:katakana") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "xyz street");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "name:pronunciation2:jeita") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "abc ave");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kJeita));
        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      }
      ++name_index;
    }
  }

  // Test 1 empty pronunciation
  {

    GraphId node_id = JK_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(JK_edge);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> index_linguistic_map;

    std::vector<SignInfo> signs = tile->GetSigns(JK_edge_id.id(), index_linguistic_map);

    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 3);
    ASSERT_EQ(index_linguistic_map.size(), 3);
    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);

      if (iter == index_linguistic_map.end()) {
        if (sign_index == 2)
          EXPECT_EQ(signs.at(sign_index).text(), "destination3");
        else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation1") {

        EXPECT_EQ(signs.at(sign_index).text(), "destination1");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation2") {

        EXPECT_EQ(signs.at(sign_index).text(), "destination2");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else {
        if (static_cast<valhalla::baldr::PronunciationAlphabet>(
                std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)) ==
            PronunciationAlphabet::kNone) {
          EXPECT_EQ(to_string(static_cast<Language>(
                        std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                    "nl");
          EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      }
      ++sign_index;
    }
  }

  // Test 2 empty linguistics
  {

    GraphId node_id = KL_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(KL_edge);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> index_linguistic_map;

    std::vector<SignInfo> signs = tile->GetSigns(KL_edge_id.id(), index_linguistic_map);

    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 3);
    ASSERT_EQ(index_linguistic_map.size(), 3);
    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);

      if (iter == index_linguistic_map.end()) {
        if (sign_index == 1)
          EXPECT_EQ(signs.at(sign_index).text(), "destination2");
        else if (sign_index == 2)
          EXPECT_EQ(signs.at(sign_index).text(), "destination3");
        else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation1") {

        EXPECT_EQ(signs.at(sign_index).text(), "destination1");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else {
        if (static_cast<valhalla::baldr::PronunciationAlphabet>(
                std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)) ==
            PronunciationAlphabet::kNone) {
          EXPECT_EQ(to_string(static_cast<Language>(
                        std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                    "nl");
          EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      }
      ++sign_index;
    }
  }

  // Test 2 empty linguistics and jeita wins
  {

    GraphId node_id = LM_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(LM_edge);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> index_linguistic_map;

    std::vector<SignInfo> signs = tile->GetSigns(LM_edge_id.id(), index_linguistic_map);

    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 3);
    ASSERT_EQ(index_linguistic_map.size(), 3);
    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);

      if (iter == index_linguistic_map.end()) {
        if (sign_index == 0)
          EXPECT_EQ(signs.at(sign_index).text(), "destination1");
        else if (sign_index == 2)
          EXPECT_EQ(signs.at(sign_index).text(), "destination3");
        else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation2:jeita") {

        EXPECT_EQ(signs.at(sign_index).text(), "destination2");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kJeita));
      } else {
        if (static_cast<valhalla::baldr::PronunciationAlphabet>(
                std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)) ==
            PronunciationAlphabet::kNone) {
          EXPECT_EQ(to_string(static_cast<Language>(
                        std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                    "nl");
          EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      }
      ++sign_index;
    }
  }

  // Test 2 empty linguistics.  Should have katakanna and jeita
  {

    GraphId node_id = MN_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(MN_edge);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> index_linguistic_map;

    std::vector<SignInfo> signs = tile->GetSigns(MN_edge_id.id(), index_linguistic_map);

    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 3);
    ASSERT_EQ(index_linguistic_map.size(), 3);
    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);

      if (iter == index_linguistic_map.end()) {
        if (sign_index == 0)
          EXPECT_EQ(signs.at(sign_index).text(), "destination1");
        else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation3:katakana") {

        EXPECT_EQ(signs.at(sign_index).text(), "destination3");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation2:jeita") {

        EXPECT_EQ(signs.at(sign_index).text(), "destination2");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kJeita));
      } else {
        if (static_cast<valhalla::baldr::PronunciationAlphabet>(
                std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)) ==
            PronunciationAlphabet::kNone) {
          EXPECT_EQ(to_string(static_cast<Language>(
                        std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                    "nl");
          EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      }
      ++sign_index;
    }
  }

  // Test the names.  nt-sampa wins.
  {

    GraphId node_id = AB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(AB_edge);
    std::vector<uint8_t> types;
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);
    ASSERT_EQ(names_and_types.size(), 6);

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
      if (iter == linguistics.end()) {
        // all should have a pronunctiation
        EXPECT_NE(iter, linguistics.end());
      } else {

        if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
            "name:pronunciation:nt-sampa") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "AB");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "tunnel:name:pronunciation:nt-sampa") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "tunnel:name");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "int_ref:pronunciation:nt-sampa int_direction:pronunciation:nt-sampa") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "int_ref int_direction");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "ref:pronunciation:nt-sampa direction:pronunciation:nt-sampa") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "ref direction");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "alt_name:pronunciation:nt-sampa") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "alt_name");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "official_name:pronunciation:nt-sampa") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "official_name");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "name:en:pronunciation:nt-sampa") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "name:en");

          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      }
      ++name_index;
    }
  }

  // Test the signs.  katakana wins.
  {
    GraphId node_id = BD_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    const NodeInfo* node_info = tile->node(node_id);

    std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> index_linguistic_map;

    std::vector<SignInfo> signs = tile->GetSigns(BD_edge_id.id(), index_linguistic_map);

    uint32_t sign_index = 0;
    ASSERT_EQ(signs.size(), 6);
    ASSERT_EQ(index_linguistic_map.size(), 6);
    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);
      ASSERT_NE(iter, index_linguistic_map.end());

      if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
          "destination:forward:pronunciation:katakana") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:forward");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:street:to:pronunciation:katakana") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:street:to");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:ref:to:pronunciation:katakana") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:ref:to");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:ref:pronunciation:katakana") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:ref");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:street:pronunciation:katakana") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:street");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "junction:ref:pronunciation:katakana") {
        EXPECT_EQ(signs.at(sign_index).text(), "junction:ref");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
      } else
        FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
               << " Extra key. This should not happen.";

      ++sign_index;
    }

    node_id = DB_edge->endnode();
    tile = graph_reader.GetGraphTile(node_id);
    node_info = tile->node(node_id);

    index_linguistic_map.clear();
    signs.clear();
    signs = tile->GetSigns(DB_edge_id.id(), index_linguistic_map);

    sign_index = 0;
    ASSERT_EQ(signs.size(), 6);
    ASSERT_EQ(index_linguistic_map.size(), 6);
    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);
      ASSERT_NE(iter, index_linguistic_map.end());

      if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
          "destination:backward:pronunciation:katakana") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:backward");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:street:to:pronunciation:katakana") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:street:to");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:ref:to:pronunciation:katakana") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:ref:to");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:ref:pronunciation:katakana") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:ref");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:street:pronunciation:katakana") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:street");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "junction:ref:pronunciation:katakana") {
        EXPECT_EQ(signs.at(sign_index).text(), "junction:ref");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
      } else
        FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
               << " Extra key. This should not happen.";

      ++sign_index;
    }

    // more signs.  jeita wins.
    node_id = BC_edge->endnode();
    tile = graph_reader.GetGraphTile(node_id);
    node_info = tile->node(node_id);

    index_linguistic_map.clear();
    signs.clear();
    signs = tile->GetSigns(BC_edge_id.id(), index_linguistic_map);

    sign_index = 0;
    ASSERT_EQ(signs.size(), 6);
    ASSERT_EQ(index_linguistic_map.size(), 6);
    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);
      ASSERT_NE(iter, index_linguistic_map.end());

      if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
          "destination:forward:pronunciation:jeita") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:forward");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kJeita));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:street:to:pronunciation:jeita") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:street:to");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kJeita));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:ref:to:pronunciation:jeita") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:ref:to");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kJeita));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:ref:pronunciation:jeita") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:ref");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kJeita));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:street:pronunciation:jeita") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination:street");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kJeita));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "junction:ref:pronunciation:jeita") {
        EXPECT_EQ(signs.at(sign_index).text(), "junction:ref");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kJeita));
      } else
        FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
               << " Extra key. This should not happen.";

      ++sign_index;
    }

    // ipa wins for the name.
    // nt-sampa wins for the alt_name
    // jeita wins for official_name
    // katakana wins for name:en
    {
      auto edgeinfo = tile->edgeinfo(BC_edge);
      std::vector<uint8_t> types;
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
        std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
            linguistics.find(name_index);
        ASSERT_NE(iter, linguistics.end());

        if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) == "name:pronunciation") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "BC");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kIpa));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "alt_name:pronunciation:nt-sampa") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "alt_name");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "official_name:pronunciation:jeita") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "official_name");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kJeita));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "name:en:pronunciation:katakana") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "name:en");

          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kKatakana));
        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";

        ++name_index;
      }
    }

    // more signs.  should all be ipa
    node_id = DE_edge->endnode();
    graph_reader.GetGraphTile(node_id);
    node_info = tile->node(node_id);

    index_linguistic_map.clear();
    signs.clear();

    signs = tile->GetSigns(DE_edge_id.id(), index_linguistic_map);

    sign_index = 0;
    ASSERT_EQ(signs.size(), 2);
    ASSERT_EQ(index_linguistic_map.size(), 2);
    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);
      ASSERT_NE(iter, index_linguistic_map.end());

      if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
          "destination:pronunciation1") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination1");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation2") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination2");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else
        FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
               << " Extra key. This should not happen.";

      ++sign_index;
    }

    // blank linguistics for names.
    {
      auto edgeinfo = tile->edgeinfo(DE_edge);
      std::vector<uint8_t> types;
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
        std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
            linguistics.find(name_index);
        ASSERT_NE(iter, linguistics.end());
        if (name_index == 0 || name_index == 1) {
          EXPECT_EQ(to_string(static_cast<Language>(
                        std::get<kLinguisticMapTupleLanguageIndex>(iter->second))),
                    "nl");
          EXPECT_EQ(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second), "");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNone));

        } else {
          EXPECT_EQ(name_index, 2);
          if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
              "name:pronunciation3") {
            EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "abc ave");
            EXPECT_EQ(static_cast<int>(
                          std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                      static_cast<int>(baldr::PronunciationAlphabet::kIpa));
          } else
            FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                   << " Extra key. This should not happen.";
        }
        ++name_index;
      }
    }

    // more signs.  should all be ipa
    node_id = ED_edge->endnode();
    graph_reader.GetGraphTile(node_id);
    node_info = tile->node(node_id);

    index_linguistic_map.clear();
    signs.clear();
    signs = tile->GetSigns(ED_edge_id.id(), index_linguistic_map);

    sign_index = 0;
    ASSERT_EQ(signs.size(), 2);
    ASSERT_EQ(index_linguistic_map.size(), 2);
    for (const auto& sign : signs) {

      std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>::const_iterator iter =
          index_linguistic_map.find(sign_index);
      ASSERT_NE(iter, index_linguistic_map.end());

      if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
          "destination:pronunciation1") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination1");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                 "destination:pronunciation2") {
        EXPECT_EQ(signs.at(sign_index).text(), "destination2");
        EXPECT_EQ(static_cast<int>(std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else
        FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
               << " Extra key. This should not happen.";

      ++sign_index;
    }
  } // end test the signs
}

TEST(Standalone, PhonemesWithNoAltandDirection) {

  const std::string workdir = "test/data/gurka_phonemes";

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBF(workdir);

  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.data_processing.allow_alt_name", "false");
  pt.put("mjolnir.data_processing.use_direction_on_ways", "false");
  pt.put("mjolnir.tile_dir", workdir + "/tiles");

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));

  GraphId AB_edge_id;
  const DirectedEdge* AB_edge = nullptr;
  GraphId BA_edge_id;
  const DirectedEdge* BA_edge = nullptr;
  std::tie(AB_edge_id, AB_edge, BA_edge_id, BA_edge) = findEdge(graph_reader, map.nodes, "AB", "B");
  EXPECT_NE(AB_edge, nullptr);
  EXPECT_NE(BA_edge, nullptr);

  // Test the names.  nt-sampa wins.
  {

    GraphId node_id = AB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(AB_edge);
    std::vector<uint8_t> types;
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);
    ASSERT_EQ(names_and_types.size(), 5);

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

      if (iter == linguistics.end()) {
        // all should have a pronunciation
        EXPECT_NE(iter, linguistics.end());
      } else {
        if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
            "name:pronunciation:nt-sampa") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "AB");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "tunnel:name:pronunciation:nt-sampa") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "tunnel:name");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "int_ref:pronunciation:nt-sampa") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "int_ref");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "ref:pronunciation:nt-sampa") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "ref");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "official_name:pronunciation:nt-sampa") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "official_name");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else if (std::get<kLinguisticMapTuplePronunciationIndex>(iter->second) ==
                   "name:en:pronunciation:nt-sampa") {
          EXPECT_EQ(std::get<0>(names_and_types.at(name_index)), "name:en");
          EXPECT_EQ(static_cast<int>(
                        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second)),
                    static_cast<int>(baldr::PronunciationAlphabet::kNtSampa));
        } else
          FAIL() << std::get<kLinguisticMapTuplePronunciationIndex>(iter->second)
                 << " Extra key. This should not happen.";
      }
      ++name_index;
    }
  }
}
