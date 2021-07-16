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

valhalla::gurka::map BuildPBF(const std::string& workdir) {
  const std::string ascii_map = R"(
      A----B----C
            \
             D-------E-------F
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
         {"name:pronunciation:x-sampa", "name:pronunciation:x-sampa"},
         {"ref:pronunciation:x-sampa", "ref:pronunciation:x-sampa"},
         {"int_ref:pronunciation:x-sampa", "int_ref:pronunciation:x-sampa"},
         {"direction:pronunciation:x-sampa", "direction:pronunciation:x-sampa"},
         {"int_direction:pronunciation:x-sampa", "int_direction:pronunciation:x-sampa"},
         {"alt_name:pronunciation:x-sampa", "alt_name:pronunciation:x-sampa"},
         {"official_name:pronunciation:x-sampa", "official_name:pronunciation:x-sampa"},
         {"tunnel:name:pronunciation:x-sampa", "tunnel:name:pronunciation:x-sampa"},
         {"name:en:pronunciation:x-sampa", "name:en:pronunciation:x-sampa"},
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
         {"name:en:pronunciation:jeita", "name:en:pronunciation:jeita"}}},
       {"BC", {{"highway", "trunk"}}},
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
         {"destination:forward:pronunciation:x-sampa", "destination:forward:pronunciation:x-sampa"},
         {"destination:backward:pronunciation:x-sampa", "destination:backward:pronunciation:x-sampa"},
         {"destination:ref:pronunciation:x-sampa", "destination:ref:pronunciation:x-sampa"},
         {"destination:ref:to:pronunciation:x-sampa", "destination:ref:to:pronunciation:x-sampa"},
         {"destination:street:pronunciation:x-sampa", "destination:street:pronunciation:x-sampa"},
         {"destination:street:to:pronunciation:x-sampa",
          "destination:street:to:pronunciation:x-sampa"},
         {"junction:ref:pronunciation:x-sampa", "junction:ref:pronunciation:x-sampa"},
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
       {"DE",
        {{"highway", "primary"},
         {"destination", "destination"},
         {"destination:pronunciation", "destination:pronunciation"},
         {"destination:pronunciation:x-sampa", "destination:pronunciation:x-sampa"},
         {"destination:pronunciation:katakana", "destination:pronunciation:katakana"},
         {"destination:pronunciation:jeita", "destination:pronunciation:jeita"}}},
       {"EF", {{"highway", "primary_link"}, {"oneway", "yes"}}}};

  const gurka::nodes nodes =
      {{"B",
        {{"junction", "named"},
         {"name", "named junction"},
         {"highway", "motorway_junction"},
         {"name:pronunciation", "named junction:pronunciation"},
         {"name:pronunciation:x-sampa", "named junction:pronunciation:x-sampa"},
         {"name:pronunciation:katakana", "named junction:pronunciation:katakana"},
         {"name:pronunciation:jeita", "named junction:pronunciation:jeita"}}},
       {"E",
        {{"ref", "node ref"},
         {"highway", "motorway_junction"},
         {"ref:pronunciation", "node ref:pronunciation"},
         {"ref:pronunciation:x-sampa", "node ref:pronunciation:x-sampa"},
         {"ref:pronunciation:katakana", "node ref:pronunciation:katakana"},
         {"ref:pronunciation:jeita", "node ref:pronunciation:jeita"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.1079374, 52.0887174});

  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, nodes, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;
  /*
    valhalla::gurka::map maps = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_phonemes",
                                 {{"mjolnir.data_processing.allow_alt_name", "true"},
                                  {"mjolnir.data_processing.use_direction_on_ways", "true"}});

    auto asdf = gurka::do_action(valhalla::Options::route, maps, {"A", "C"}, "auto");

    asdf = gurka::do_action(valhalla::Options::route, maps, {"A", "E"}, "auto");
  */

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

  // Test the named junction on the node.
  {

    GraphId node_id = AB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);

    std::unordered_multimap<uint32_t, std::pair<uint8_t, std::string>> pronunciations;
    std::vector<SignInfo> signs = tile->GetSigns(node_id.id(), pronunciations, true);

    for (const auto& pronunciation : pronunciations) {

      if ((pronunciation.second).second == "named junction:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "named junction");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "named junction:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "named junction");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "named junction:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "named junction");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "named junction:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "named junction");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else
        EXPECT_EQ((pronunciation.second).second, "Extra key.  This should not happen.");
    }
  }

  // Test the ref on the node
  {

    GraphId node_id = EF_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(EF_edge);

    std::unordered_multimap<uint32_t, std::pair<uint8_t, std::string>> pronunciations;
    std::vector<SignInfo> signs = tile->GetSigns(EF_edge_id.id(), pronunciations);

    for (const auto& pronunciation : pronunciations) {

      if ((pronunciation.second).second == "node ref:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "node ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "node ref:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "node ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "node ref:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "node ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "node ref:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "node ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else
        EXPECT_EQ((pronunciation.second).second, "Extra key.  This should not happen.");
    }
  }

  // Test the names
  {

    GraphId node_id = AB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(AB_edge);
    std::vector<uint8_t> types;
    auto names_and_types = edgeinfo.GetNamesAndTypes(types, true);

    std::unordered_multimap<uint8_t, std::pair<uint8_t, std::string>> pronunciations =
        edgeinfo.GetPronunciationsMultiMap();
    for (const auto& pronunciation : pronunciations) {

      // std::cout << static_cast<int>(pronunciation.first) << " "
      //        << static_cast<int>((pronunciation.second).first) << " "
      //      << (pronunciation.second).second << "-----> "
      //    << names_and_types.at(static_cast<int>(pronunciation.first)).first << std::endl;

      if ((pronunciation.second).second == "name:pronunciation") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "AB");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "name:pronunciation:x-sampa") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "AB");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "name:pronunciation:katakana") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "AB");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "name:pronunciation:jeita") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "AB");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "tunnel:name:pronunciation") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "tunnel:name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "tunnel:name:pronunciation:x-sampa") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "tunnel:name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "tunnel:name:pronunciation:katakana") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "tunnel:name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "tunnel:name:pronunciation:jeita") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "tunnel:name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second ==
                 "int_ref:pronunciation int_direction:pronunciation") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first,
                  "int_ref int_direction");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second ==
                 "int_ref:pronunciation:x-sampa int_direction:pronunciation:x-sampa") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first,
                  "int_ref int_direction");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second ==
                 "int_ref:pronunciation:katakana int_direction:pronunciation:katakana") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first,
                  "int_ref int_direction");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second ==
                 "int_ref:pronunciation:jeita int_direction:pronunciation:jeita") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first,
                  "int_ref int_direction");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "ref:pronunciation direction:pronunciation") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "ref direction");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second ==
                 "ref:pronunciation:x-sampa direction:pronunciation:x-sampa") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "ref direction");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second ==
                 "ref:pronunciation:katakana direction:pronunciation:katakana") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "ref direction");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second ==
                 "ref:pronunciation:jeita direction:pronunciation:jeita") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "ref direction");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "alt_name:pronunciation") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "alt_name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "alt_name:pronunciation:x-sampa") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "alt_name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "alt_name:pronunciation:katakana") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "alt_name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "alt_name:pronunciation:jeita") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "alt_name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "official_name:pronunciation") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "official_name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "official_name:pronunciation:x-sampa") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "official_name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "official_name:pronunciation:katakana") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "official_name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "official_name:pronunciation:jeita") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "official_name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "name:en:pronunciation") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "name:en");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "name:en:pronunciation:x-sampa") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "name:en");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "name:en:pronunciation:katakana") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "name:en");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "name:en:pronunciation:jeita") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "name:en");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else
        EXPECT_EQ((pronunciation.second).second, "Extra key.  This should not happen.");
    }
  }

  // Test the signs
  {
    GraphId node_id = BD_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    const NodeInfo* node_info = tile->node(node_id);

    std::unordered_multimap<uint32_t, std::pair<uint8_t, std::string>> pronunciations;
    std::vector<SignInfo> signs = tile->GetSigns(BD_edge_id.id(), pronunciations);

    for (const auto& pronunciation : pronunciations) {

      if ((pronunciation.second).second == "destination:forward:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:forward");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "destination:forward:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:forward");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "destination:forward:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:forward");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "destination:forward:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:forward");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "destination:street:to:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "destination:street:to:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "destination:street:to:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "destination:street:to:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "destination:ref:to:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "destination:ref:to:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "destination:ref:to:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "destination:ref:to:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "destination:ref:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "destination:ref:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "destination:ref:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "destination:ref:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "destination:street:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "destination:street:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "destination:street:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "destination:street:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "junction:ref:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "junction:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "junction:ref:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "junction:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "junction:ref:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "junction:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "junction:ref:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "junction:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else
        EXPECT_EQ((pronunciation.second).second, "Extra key.  This should not happen.");
    }

    node_id = DB_edge->endnode();
    tile = graph_reader.GetGraphTile(node_id);
    node_info = tile->node(node_id);

    pronunciations.clear();
    signs = tile->GetSigns(DB_edge_id.id(), pronunciations);

    for (const auto& pronunciation : pronunciations) {

      if ((pronunciation.second).second == "destination:backward:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:backward");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "destination:backward:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:backward");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "destination:backward:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:backward");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "destination:backward:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:backward");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "destination:street:to:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "destination:street:to:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "destination:street:to:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "destination:street:to:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "destination:ref:to:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "destination:ref:to:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "destination:ref:to:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "destination:ref:to:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref:to");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "destination:ref:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "destination:ref:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "destination:ref:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "destination:ref:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "destination:street:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "destination:street:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "destination:street:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "destination:street:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination:street");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "junction:ref:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "junction:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "junction:ref:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "junction:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "junction:ref:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "junction:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "junction:ref:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "junction:ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else
        EXPECT_EQ((pronunciation.second).second, "Extra key.  This should not happen.");
    }

    node_id = DE_edge->endnode();
    graph_reader.GetGraphTile(node_id);
    node_info = tile->node(node_id);

    pronunciations.clear();
    signs = tile->GetSigns(DE_edge_id.id(), pronunciations);

    for (const auto& pronunciation : pronunciations) {

      if ((pronunciation.second).second == "destination:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "destination:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "destination:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "destination:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else
        EXPECT_EQ((pronunciation.second).second, "Extra key.  This should not happen.");
    }

    node_id = ED_edge->endnode();
    graph_reader.GetGraphTile(node_id);
    node_info = tile->node(node_id);

    pronunciations.clear();
    signs = tile->GetSigns(ED_edge_id.id(), pronunciations);

    // for (auto s : signs)
    //  std::cout << s.text() << std::endl;

    for (const auto& pronunciation : pronunciations) {
      // std::cout << static_cast<int>(pronunciation.first) << " "
      //         << static_cast<int>((pronunciation.second).first) << " "
      //         << (pronunciation.second).second << "-----> "
      //       << signs.at(static_cast<int>(pronunciation.first)).text() << std::endl;

      if ((pronunciation.second).second == "destination:pronunciation") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "destination:pronunciation:x-sampa") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "destination:pronunciation:katakana") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "destination:pronunciation:jeita") {
        EXPECT_EQ(signs.at(static_cast<int>(pronunciation.first)).text(), "destination");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else
        EXPECT_EQ((pronunciation.second).second, "Extra key.  This should not happen.");
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

  // Test the names
  {

    GraphId node_id = AB_edge->endnode();
    auto tile = graph_reader.GetGraphTile(node_id);
    auto edgeinfo = tile->edgeinfo(AB_edge);
    std::vector<uint8_t> types;
    auto names_and_types = edgeinfo.GetNamesAndTypes(types, true);
    //  for (const auto& name_and_type : names_and_types) {
    //    auto* trip_edge_name = trip_edge->mutable_name()->Add();
    //    trip_edge_name->set_value(name_and_type.first);
    //    trip_edge_name->set_is_route_number(name_and_type.second);
    //  }

    std::unordered_multimap<uint8_t, std::pair<uint8_t, std::string>> pronunciations =
        edgeinfo.GetPronunciationsMultiMap();
    for (const auto& pronunciation : pronunciations) {

      // std::cout << static_cast<int>(pronunciation.first) << " "
      //         << static_cast<int>((pronunciation.second).first) << " "
      //       << (pronunciation.second).second << "-----> "
      //     << names_and_types.at(static_cast<int>(pronunciation.first)).first << std::endl;

      if ((pronunciation.second).second == "name:pronunciation") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "AB");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "name:pronunciation:x-sampa") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "AB");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "name:pronunciation:katakana") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "AB");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "name:pronunciation:jeita") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "AB");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "tunnel:name:pronunciation") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "tunnel:name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "tunnel:name:pronunciation:x-sampa") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "tunnel:name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "tunnel:name:pronunciation:katakana") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "tunnel:name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "tunnel:name:pronunciation:jeita") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "tunnel:name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "int_ref:pronunciation") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "int_ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "int_ref:pronunciation:x-sampa") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "int_ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "int_ref:pronunciation:katakana") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "int_ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "int_ref:pronunciation:jeita") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "int_ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "ref:pronunciation") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "ref:pronunciation:x-sampa") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "ref:pronunciation:katakana") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "ref:pronunciation:jeita") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "ref");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "official_name:pronunciation") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "official_name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "official_name:pronunciation:x-sampa") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "official_name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "official_name:pronunciation:katakana") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "official_name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "official_name:pronunciation:jeita") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "official_name");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else if ((pronunciation.second).second == "name:en:pronunciation") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "name:en");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kIpa));
      } else if ((pronunciation.second).second == "name:en:pronunciation:x-sampa") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "name:en");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXSampa));
      } else if ((pronunciation.second).second == "name:en:pronunciation:katakana") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "name:en");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXKatakana));
      } else if ((pronunciation.second).second == "name:en:pronunciation:jeita") {
        EXPECT_EQ(names_and_types.at(static_cast<int>(pronunciation.first)).first, "name:en");
        EXPECT_EQ(static_cast<int>((pronunciation.second).first),
                  static_cast<int>(baldr::PronunciationAlphabet::kXJeita));
      } else
        EXPECT_EQ((pronunciation.second).second, "Extra key.  This should not happen.");
    }
  }
}
