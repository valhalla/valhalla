#include "loki/street_name.h"

#include <gtest/gtest.h>

using namespace valhalla::loki::street_name;

namespace {

TEST(StreetName, NormalizeSplitsOnNonAlphanumerics) {
  const auto hyphen = normalize_name("Primus-Truber Straße");
  EXPECT_EQ(hyphen.tokens, (std::vector<std::u32string>{U"primus", U"truber", U"strasse"}));
  EXPECT_EQ(hyphen.stripped, U"primustruberstrasse");

  EXPECT_EQ(normalize_name("Primus Truber Straße").stripped, hyphen.stripped);
  EXPECT_EQ(normalize_name("Primus-Truber-Straße").stripped, hyphen.stripped);
  EXPECT_EQ(normalize_name("St. John's (Rd)/2").stripped, U"stjohnsrd2");
}

TEST(StreetName, NormalizeIgnoresCompounding) {
  const auto glued = normalize_name("Hermannstraße");
  const auto split = normalize_name("Hermann Straße");
  EXPECT_EQ(glued.tokens.size(), 1u);
  EXPECT_EQ(split.tokens.size(), 2u);
  EXPECT_EQ(glued.stripped, split.stripped);
}

TEST(StreetName, NormalizeTransliterates) {
  EXPECT_EQ(normalize_name("Łąkowa").stripped, U"lakowa");
  EXPECT_EQ(normalize_name("Nørregade").stripped, U"norregade");
  EXPECT_EQ(normalize_name("Þingholtsstræti").stripped, U"thingholtsstraeti");
  EXPECT_EQ(normalize_name("Ulica Świętokrzyska").stripped, U"ulicaswietokrzyska");
  EXPECT_EQ(normalize_name("Çarşı Caddesi").stripped, U"carsicaddesi");
}

TEST(StreetName, NormalizeFoldsCase) {
  EXPECT_EQ(normalize_name("УЛИЦА Ленина").stripped, normalize_name("улица ленина").stripped);
  EXPECT_EQ(normalize_name("ΟΔΟΣ").stripped, normalize_name("οδος").stripped);
  EXPECT_EQ(normalize_name("Ɛɓɔ").stripped, normalize_name("ƐƁƆ").stripped);
}

TEST(StreetName, NormalizeStripsDiacriticsAcrossScripts) {
  EXPECT_EQ(normalize_name("Οδός").stripped, normalize_name("οδος").stripped);
  EXPECT_EQ(normalize_name("Ёлочная").stripped, normalize_name("елочная").stripped);
}

TEST(StreetName, NormalizeKeepsVowelMarks) {
  EXPECT_EQ(normalize_name("กิ").stripped, U"กิ");
  EXPECT_EQ(normalize_name("चि").stripped, U"चि");
  EXPECT_EQ(normalize_name("中山路").stripped, U"中山路");
}

TEST(StreetName, NormalizeCompatibilityForms) {
  EXPECT_EQ(normalize_name("Ｍａｉｎ").stripped, U"main");
  EXPECT_EQ(normalize_name("ﬁeld").stripped, U"field");
}

TEST(StreetName, NormalizeUnicodePunctuationSeparates) {
  const std::vector<std::u32string> expected{U"hans", U"sachs"};
  EXPECT_EQ(normalize_name("Hans–Sachs").tokens, expected);
  EXPECT_EQ(normalize_name("Hans’Sachs").tokens, expected);
  EXPECT_EQ(normalize_name("Hans\u00A0Sachs").tokens, expected);
}

TEST(StreetName, NormalizeEmpty) {
  EXPECT_TRUE(normalize_name("").tokens.empty());
  EXPECT_TRUE(normalize_name(" - / ").stripped.empty());
}

TEST(StreetName, NormalizeMalformedUtf8) {
  const std::vector<std::u32string> expected{U"a", U"b"};
  EXPECT_EQ(normalize_name("a\xFF"
                           "b")
                .tokens,
            expected);
  // truncated two byte sequence at the end
  EXPECT_EQ(normalize_name("a\xC3").stripped, U"a");
}

TEST(StreetName, CollapseDigraphs) {
  // what's typed three ways only converges in the retry pass
  EXPECT_EQ(normalize_name("Müller").stripped, U"muller");
  EXPECT_EQ(normalize_name("Mueller").stripped, U"mueller");
  EXPECT_EQ(collapse_digraphs(normalize_name("Mueller")).stripped, U"muller");
  EXPECT_EQ(collapse_digraphs(normalize_name("Müller")).stripped, U"muller");

  const auto expected = collapse_digraphs(normalize_name("Österbergstraße")).stripped;
  EXPECT_EQ(expected, U"osterbergstrasse");
  EXPECT_EQ(collapse_digraphs(normalize_name("Osterbergstrasse")).stripped, expected);
  EXPECT_EQ(collapse_digraphs(normalize_name("Oesterbergstrasse")).stripped, expected);
}

TEST(StreetName, CollapseDigraphsKeepsTokens) {
  const auto collapsed = collapse_digraphs(normalize_name("Österberg Straße"));
  EXPECT_EQ(collapsed.tokens, (std::vector<std::u32string>{U"osterberg", U"strasse"}));
  EXPECT_EQ(collapsed.stripped, U"osterbergstrasse");
}

names_t names(std::initializer_list<std::string> names) {
  names_t result;
  for (const auto& name : names)
    result.emplace_back(name, false);
  return result;
}

TEST(StreetName, ScoreWeighsByRarity) {
  // the fixture of the design notes, the generic is everywhere so the core decides
  const std::vector<names_t> candidates{names({"Bergstraße"}), names({"Burgstraße"}),
                                        names({"Bahnhofstraße"}), names({"Schulweg"})};
  const auto scores = score("Bergstr", candidates);
  ASSERT_EQ(scores.size(), 4u);
  EXPECT_NEAR(scores[0], 1.f, 0.005f);
  EXPECT_NEAR(scores[1], 0.42f, 0.005f);
  EXPECT_NEAR(scores[2], 0.09f, 0.005f);
  EXPECT_EQ(scores[3], 0.f);
}

TEST(StreetName, ScoreCountsNamesNotEdges) {
  // more edges of Bergstraße must not make its grams look common, nor does a name without grams
  // count towards the corpus
  const std::vector<names_t> candidates{names({"Bergstraße"}),
                                        names({"Bergstraße"}),
                                        names({"Bergstraße"}),
                                        names({"Burgstraße"}),
                                        names({"Bahnhofstraße"}),
                                        names({"Schulweg"}),
                                        names({"B 1"})};
  const auto scores = score("Bergstr", candidates);
  EXPECT_NEAR(scores[2], 1.f, 0.005f);
  EXPECT_NEAR(scores[3], 0.42f, 0.005f);
  EXPECT_EQ(scores[6], 0.f);
}

TEST(StreetName, ScoreTakesBestName) {
  const std::vector<names_t> candidates{names({"Burgstraße", "Bergstraße"}), names({"Schulweg"})};
  const auto scores = score("Bergstr", candidates);
  EXPECT_NEAR(scores[0], 1.f, 0.005f);
  EXPECT_EQ(scores[1], 0.f);
}

TEST(StreetName, ScoreSingleName) {
  // smoothing keeps a lone name scorable, it's plain containment then
  const auto scores = score("Primus-Truber-Str", {names({"Primus-Truber-Straße"})});
  EXPECT_NEAR(scores[0], 1.f, 0.005f);
}

TEST(StreetName, ScoreUnseenGramsWeighMost) {
  // `mozartstr` shares only the generic with them, the unseen core has to outweigh it
  const std::vector<names_t> candidates{names({"Bergstraße"}), names({"Burgstraße"}),
                                        names({"Bahnhofstraße"})};
  for (const auto s : score("Mozartstr", candidates))
    EXPECT_LT(s, 0.2f);
}

TEST(StreetName, ScoreWithoutGrams) {
  const std::vector<names_t> candidates{names({"Bergstraße"}), names({"B"}), names({})};
  EXPECT_EQ(score("Be", candidates), (std::vector<float>{0.f, 0.f, 0.f}));
  EXPECT_EQ(score("Bergstr", candidates)[1], 0.f);
  EXPECT_EQ(score("Bergstr", candidates)[2], 0.f);
}

TEST(StreetName, BestMatches) {
  const std::vector<names_t> candidates{names({"Burgstraße"}), names({"Bergstraße"}),
                                        names({"Bergstraße"}), names({"Schulweg"})};
  EXPECT_EQ(best_matches("Bergstr", candidates, 0.6f), (std::vector<size_t>{1, 2}));
  EXPECT_TRUE(best_matches("Schillerstr", candidates, 0.6f).empty());
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
