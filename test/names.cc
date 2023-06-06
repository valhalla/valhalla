#include "baldr/graphconstants.h"
#include "midgard/util.h"
#include "mjolnir/osmway.h"
#include "mjolnir/uniquenames.h"
#include <iostream>

#include "test.h"

using namespace std;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;
using namespace valhalla::baldr;

namespace {

void TestPronunciationKeyTypeValue(const std::string& pronunciation,
                                   uint32_t expected_key,
                                   PronunciationAlphabet expected_type,
                                   const std::string& expected_value) {

  auto* p = const_cast<char*>(pronunciation.c_str());

  size_t pos = 0;
  while (pos < strlen(p)) {
    const auto header = unaligned_read<linguistic_text_header_t>(p + pos);
    pos += kLinguisticHeaderSize;
    EXPECT_EQ(header.name_index_, expected_key);
    EXPECT_EQ(static_cast<PronunciationAlphabet>(header.phonetic_alphabet_), expected_type);
    EXPECT_EQ(std::string((p + pos), header.length_), expected_value);

    pos += header.length_;
  }
}

TEST(Names, NamesTest) {

  OSMWay w1{1234};
  OSMWay w2{1234};
  OSMWay w3{1234};

  OSMPronunciation pronunciation{};
  std::vector<std::string> pronunciations, languages;
  UniqueNames name_offset_map;
  std::string ref = "I 79 North";

  w1.set_name_index(name_offset_map.index("William Flynn Highway"));
  w2.set_name_index(name_offset_map.index("Mon/Fayette Expressway"));
  w3.set_name_index(name_offset_map.index("Lancaster Pike"));

  w2.set_ref_index(name_offset_map.index("PA 43"));
  w3.set_ref_index(name_offset_map.index("PA 272"));

  pronunciation.set_name_pronunciation_ipa_index(name_offset_map.index("test name ipa"));
  pronunciation.set_name_pronunciation_nt_sampa_index(name_offset_map.index("test name sampa"));
  pronunciation.set_name_pronunciation_katakana_index(name_offset_map.index("test name katakana"));
  pronunciation.set_name_pronunciation_jeita_index(name_offset_map.index("test name jeita"));
  pronunciation.set_ref_pronunciation_ipa_index(name_offset_map.index("test ref ipa"));
  pronunciation.set_ref_pronunciation_nt_sampa_index(name_offset_map.index("test ref sampa"));
  pronunciation.set_ref_pronunciation_katakana_index(name_offset_map.index("test ref katakana"));
  pronunciation.set_ref_pronunciation_jeita_index(name_offset_map.index("test ref jeita"));

  w1.set_road_class(RoadClass::kMotorway);
  w2.set_road_class(RoadClass::kTrunk);
  w3.set_road_class(RoadClass::kPrimary);

  uint16_t types;
  std::vector<std::string> w1_names;
  std::vector<std::pair<std::string, bool>> default_languages;

  w1.GetNames(ref, name_offset_map, pronunciation, default_languages, w1.ref_index(), 0,
              w1.name_index(), 0, w1.official_name_index(), 0, w1.alt_name_index(), 0, types,
              w1_names, pronunciations, languages);

  // if road class = kTrunk or kMotorway, then ref comes first.  ref from relation overrides
  // ref from name_offset_map
  EXPECT_EQ(w1_names.at(0), "I 79 North");
  EXPECT_EQ(w1_names.at(1), "William Flynn Highway");

  TestPronunciationKeyTypeValue(pronunciations.at(0), 0, PronunciationAlphabet::kIpa, "test ref ipa");
  TestPronunciationKeyTypeValue(pronunciations.at(1), 0, PronunciationAlphabet::kNtSampa,
                                "test ref sampa");
  TestPronunciationKeyTypeValue(pronunciations.at(2), 0, PronunciationAlphabet::kKatakana,
                                "test ref katakana");
  TestPronunciationKeyTypeValue(pronunciations.at(3), 0, PronunciationAlphabet::kJeita,
                                "test ref jeita");
  TestPronunciationKeyTypeValue(pronunciations.at(4), 1, PronunciationAlphabet::kIpa,
                                "test name ipa");
  TestPronunciationKeyTypeValue(pronunciations.at(5), 1, PronunciationAlphabet::kNtSampa,
                                "test name sampa");
  TestPronunciationKeyTypeValue(pronunciations.at(6), 1, PronunciationAlphabet::kKatakana,
                                "test name katakana");
  TestPronunciationKeyTypeValue(pronunciations.at(7), 1, PronunciationAlphabet::kJeita,
                                "test name jeita");

  std::vector<std::string> w2_names;
  pronunciations.clear();
  languages.clear();
  w2.GetNames("", name_offset_map, pronunciation, default_languages, w2.ref_index(), 0,
              w2.name_index(), 0, w2.official_name_index(), 0, w2.alt_name_index(), 0, types,
              w2_names, pronunciations, languages);

  // if road class = kTrunk or kMotorway, then ref comes first.  use ref from name_offset_map
  EXPECT_EQ(w2_names.at(0), "PA 43");
  EXPECT_EQ(w2_names.at(1), "Mon/Fayette Expressway");

  TestPronunciationKeyTypeValue(pronunciations.at(0), 0, PronunciationAlphabet::kIpa, "test ref ipa");
  TestPronunciationKeyTypeValue(pronunciations.at(1), 0, PronunciationAlphabet::kNtSampa,
                                "test ref sampa");
  TestPronunciationKeyTypeValue(pronunciations.at(2), 0, PronunciationAlphabet::kKatakana,
                                "test ref katakana");
  TestPronunciationKeyTypeValue(pronunciations.at(3), 0, PronunciationAlphabet::kJeita,
                                "test ref jeita");
  TestPronunciationKeyTypeValue(pronunciations.at(4), 1, PronunciationAlphabet::kIpa,
                                "test name ipa");
  TestPronunciationKeyTypeValue(pronunciations.at(5), 1, PronunciationAlphabet::kNtSampa,
                                "test name sampa");
  TestPronunciationKeyTypeValue(pronunciations.at(6), 1, PronunciationAlphabet::kKatakana,
                                "test name katakana");
  TestPronunciationKeyTypeValue(pronunciations.at(7), 1, PronunciationAlphabet::kJeita,
                                "test name jeita");

  EXPECT_EQ(types, 1) << "ref_map failed.  ref not in correct position.";

  std::vector<std::string> w3_names;
  pronunciations.clear();
  languages.clear();
  w3.GetNames("", name_offset_map, pronunciation, default_languages, w3.ref_index(), 0,
              w3.name_index(), 0, w3.official_name_index(), 0, w3.alt_name_index(), 0, types,
              w3_names, pronunciations, languages);

  // if Road class < kTrunk, then name first then ref using ref from name_offset_map
  EXPECT_EQ(w3_names.at(0), "Lancaster Pike") << "Road class < kTrunk test failed.";
  EXPECT_EQ(w3_names.at(1), "PA 272") << "Road class < kTrunk test failed.";

  TestPronunciationKeyTypeValue(pronunciations.at(0), 0, PronunciationAlphabet::kIpa,
                                "test name ipa");
  TestPronunciationKeyTypeValue(pronunciations.at(1), 0, PronunciationAlphabet::kNtSampa,
                                "test name sampa");
  TestPronunciationKeyTypeValue(pronunciations.at(2), 0, PronunciationAlphabet::kKatakana,
                                "test name katakana");
  TestPronunciationKeyTypeValue(pronunciations.at(3), 0, PronunciationAlphabet::kJeita,
                                "test name jeita");
  TestPronunciationKeyTypeValue(pronunciations.at(4), 1, PronunciationAlphabet::kIpa, "test ref ipa");
  TestPronunciationKeyTypeValue(pronunciations.at(5), 1, PronunciationAlphabet::kNtSampa,
                                "test ref sampa");
  TestPronunciationKeyTypeValue(pronunciations.at(6), 1, PronunciationAlphabet::kKatakana,
                                "test ref katakana");
  TestPronunciationKeyTypeValue(pronunciations.at(7), 1, PronunciationAlphabet::kJeita,
                                "test ref jeita");
  EXPECT_EQ(types, 2) << "Road class < kTrunk test failed.  ref not in correct position.";

  w3_names.clear();
  pronunciations.clear();
  languages.clear();
  w3.GetNames("PA 555", name_offset_map, pronunciation, default_languages, w3.ref_index(), 0,
              w3.name_index(), 0, w3.official_name_index(), 0, w3.alt_name_index(), 0, types,
              w3_names, pronunciations, languages);

  // if Road class < kTrunk, then name first then ref using ref from relations
  EXPECT_EQ(w3_names.at(0), "Lancaster Pike") << "ref from relations";
  EXPECT_EQ(w3_names.at(1), "PA 555") << "ref from relations";

  TestPronunciationKeyTypeValue(pronunciations.at(0), 0, PronunciationAlphabet::kIpa,
                                "test name ipa");
  TestPronunciationKeyTypeValue(pronunciations.at(1), 0, PronunciationAlphabet::kNtSampa,
                                "test name sampa");
  TestPronunciationKeyTypeValue(pronunciations.at(2), 0, PronunciationAlphabet::kKatakana,
                                "test name katakana");
  TestPronunciationKeyTypeValue(pronunciations.at(3), 0, PronunciationAlphabet::kJeita,
                                "test name jeita");
  TestPronunciationKeyTypeValue(pronunciations.at(4), 1, PronunciationAlphabet::kIpa, "test ref ipa");
  TestPronunciationKeyTypeValue(pronunciations.at(5), 1, PronunciationAlphabet::kNtSampa,
                                "test ref sampa");
  TestPronunciationKeyTypeValue(pronunciations.at(6), 1, PronunciationAlphabet::kKatakana,
                                "test ref katakana");
  TestPronunciationKeyTypeValue(pronunciations.at(7), 1, PronunciationAlphabet::kJeita,
                                "test ref jeita");
  EXPECT_EQ(types, 2)
      << "Road class < kTrunk test failed(ref from relations).  ref not in correct position.";

  w3.set_alt_name_index(name_offset_map.index("Lanc Pike"));
  w3.set_official_name_index(name_offset_map.index("LP"));

  pronunciation.set_alt_name_pronunciation_ipa_index(name_offset_map.index("test alt name ipa"));
  pronunciation.set_alt_name_pronunciation_nt_sampa_index(
      name_offset_map.index("test alt name sampa"));
  pronunciation.set_alt_name_pronunciation_katakana_index(
      name_offset_map.index("test alt name katakana"));
  pronunciation.set_alt_name_pronunciation_jeita_index(name_offset_map.index("test alt name jeita"));
  pronunciation.set_official_name_pronunciation_ipa_index(
      name_offset_map.index("test official name ipa"));
  pronunciation.set_official_name_pronunciation_nt_sampa_index(
      name_offset_map.index("test official name sampa"));
  pronunciation.set_official_name_pronunciation_katakana_index(
      name_offset_map.index("test official name katakana"));
  pronunciation.set_official_name_pronunciation_jeita_index(
      name_offset_map.index("test official name jeita"));

  w3_names.clear();
  pronunciations.clear();
  languages.clear();
  w3.GetNames("", name_offset_map, pronunciation, default_languages, w3.ref_index(), 0,
              w3.name_index(), 0, w3.official_name_index(), 0, w3.alt_name_index(), 0, types,
              w3_names, pronunciations, languages);

  EXPECT_EQ(types, 2) << "all other names test failed.  ref not in correct position.";

  // all other names should be last.
  EXPECT_EQ(w3_names.at(2), "Lanc Pike") << "Alt name failed.";
  EXPECT_EQ(w3_names.at(3), "LP") << "official name failed.";

  TestPronunciationKeyTypeValue(pronunciations.at(0), 0, PronunciationAlphabet::kIpa,
                                "test name ipa");
  TestPronunciationKeyTypeValue(pronunciations.at(1), 0, PronunciationAlphabet::kNtSampa,
                                "test name sampa");
  TestPronunciationKeyTypeValue(pronunciations.at(2), 0, PronunciationAlphabet::kKatakana,
                                "test name katakana");
  TestPronunciationKeyTypeValue(pronunciations.at(3), 0, PronunciationAlphabet::kJeita,
                                "test name jeita");
  TestPronunciationKeyTypeValue(pronunciations.at(4), 1, PronunciationAlphabet::kIpa, "test ref ipa");
  TestPronunciationKeyTypeValue(pronunciations.at(5), 1, PronunciationAlphabet::kNtSampa,
                                "test ref sampa");
  TestPronunciationKeyTypeValue(pronunciations.at(6), 1, PronunciationAlphabet::kKatakana,
                                "test ref katakana");
  TestPronunciationKeyTypeValue(pronunciations.at(7), 1, PronunciationAlphabet::kJeita,
                                "test ref jeita");
  TestPronunciationKeyTypeValue(pronunciations.at(8), 2, PronunciationAlphabet::kIpa,
                                "test alt name ipa");
  TestPronunciationKeyTypeValue(pronunciations.at(9), 2, PronunciationAlphabet::kNtSampa,
                                "test alt name sampa");
  TestPronunciationKeyTypeValue(pronunciations.at(10), 2, PronunciationAlphabet::kKatakana,
                                "test alt name katakana");
  TestPronunciationKeyTypeValue(pronunciations.at(11), 2, PronunciationAlphabet::kJeita,
                                "test alt name jeita");
  TestPronunciationKeyTypeValue(pronunciations.at(12), 3, PronunciationAlphabet::kIpa,
                                "test official name ipa");
  TestPronunciationKeyTypeValue(pronunciations.at(13), 3, PronunciationAlphabet::kNtSampa,
                                "test official name sampa");
  TestPronunciationKeyTypeValue(pronunciations.at(14), 3, PronunciationAlphabet::kKatakana,
                                "test official name katakana");
  TestPronunciationKeyTypeValue(pronunciations.at(15), 3, PronunciationAlphabet::kJeita,
                                "test official name jeita");
}

TEST(Names, TaggedNamesTest) {

  OSMWay w1{1234};
  OSMWay w2{1234};

  OSMPronunciation pronunciation1{}, pronunciation2{};
  std::vector<std::string> pronunciations, languages;
  std::vector<std::pair<std::string, bool>> default_languages;

  UniqueNames name_offset_map;

  w1.set_tunnel_name_index(name_offset_map.index("Ted Williams Tunnel"));
  w2.set_tunnel_name_index(name_offset_map.index("Fort McHenry Tunnel"));
  w1.set_road_class(RoadClass::kMotorway);
  w2.set_road_class(RoadClass::kMotorway);

  pronunciation1.set_tunnel_name_pronunciation_ipa_index(name_offset_map.index("tɛd ˈwɪljəmz ˈtʌnl"));
  pronunciation1.set_tunnel_name_pronunciation_nt_sampa_index(
      name_offset_map.index("tEd wIly@mz t@n@l"));
  pronunciation1.set_tunnel_name_pronunciation_katakana_index(
      name_offset_map.index("テッド ウィリャムズ タネル"));
  pronunciation1.set_tunnel_name_pronunciation_jeita_index(
      name_offset_map.index("チバダ'イガ&ク% セーモンマ'エ."));

  pronunciation2.set_tunnel_name_pronunciation_ipa_index(name_offset_map.index("fɔːt McHenry ˈtʌnl"));
  pronunciation2.set_tunnel_name_pronunciation_nt_sampa_index(
      name_offset_map.index("fOrt m@kEnri t@n@l"));
  pronunciation2.set_tunnel_name_pronunciation_katakana_index(
      name_offset_map.index("フォート ムケンリー タネル"));
  pronunciation2.set_tunnel_name_pronunciation_jeita_index(
      name_offset_map.index("チバダ'イガ&ク% セーモンマ'エ."));

  std::vector<std::string> w1_tagged_names;
  w1.GetTaggedValues(name_offset_map, pronunciation1, default_languages, w1.tunnel_name_index(), 0, 0,
                     w1_tagged_names, pronunciations, languages);
  EXPECT_EQ(w1_tagged_names.at(0), "1Ted Williams Tunnel");
  TestPronunciationKeyTypeValue(pronunciations.at(0), 0, PronunciationAlphabet::kIpa,
                                "tɛd ˈwɪljəmz ˈtʌnl");
  TestPronunciationKeyTypeValue(pronunciations.at(1), 0, PronunciationAlphabet::kNtSampa,
                                "tEd wIly@mz t@n@l");
  TestPronunciationKeyTypeValue(pronunciations.at(2), 0, PronunciationAlphabet::kKatakana,
                                "テッド ウィリャムズ タネル");
  TestPronunciationKeyTypeValue(pronunciations.at(3), 0, PronunciationAlphabet::kJeita,
                                "チバダ'イガ&ク% セーモンマ'エ.");

  pronunciations.clear();
  languages.clear();
  std::vector<std::string> w2_tagged_names;
  w2_tagged_names.emplace_back("test name");
  // testing pronunciation index

  w2.GetTaggedValues(name_offset_map, pronunciation2, default_languages, w2.tunnel_name_index(), 0, 0,
                     w2_tagged_names, pronunciations, languages);
  EXPECT_EQ(w2_tagged_names.at(1), "1Fort McHenry Tunnel");
  TestPronunciationKeyTypeValue(pronunciations.at(0), 1, PronunciationAlphabet::kIpa,
                                "fɔːt McHenry ˈtʌnl");
  TestPronunciationKeyTypeValue(pronunciations.at(1), 1, PronunciationAlphabet::kNtSampa,
                                "fOrt m@kEnri t@n@l");
  TestPronunciationKeyTypeValue(pronunciations.at(2), 1, PronunciationAlphabet::kKatakana,
                                "フォート ムケンリー タネル");
  TestPronunciationKeyTypeValue(pronunciations.at(3), 1, PronunciationAlphabet::kJeita,
                                "チバダ'イガ&ク% セーモンマ'エ.");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
