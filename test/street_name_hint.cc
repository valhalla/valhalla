#include "loki/street_name.h"

#include <gtest/gtest.h>

using namespace valhalla::loki::street_name;

namespace {

TEST(StreetName, NormalizeDropsNonAlphanumerics) {
  const auto hyphen = normalize_name("Primus-Truber Straße");
  EXPECT_EQ(hyphen, U"primustruberstrasse");
  EXPECT_EQ(normalize_name("Primus Truber Straße"), hyphen);
  EXPECT_EQ(normalize_name("Primus-Truber-Straße"), hyphen);
  EXPECT_EQ(normalize_name("St. John's (Rd)/2"), U"stjohnsrd2");
}

TEST(StreetName, NormalizeIgnoresCompounding) {
  EXPECT_EQ(normalize_name("Hermannstraße"), normalize_name("Hermann Straße"));
}

TEST(StreetName, NormalizeTransliterates) {
  EXPECT_EQ(normalize_name("Łąkowa"), U"lakowa");
  EXPECT_EQ(normalize_name("Nørregade"), U"norregade");
  EXPECT_EQ(normalize_name("Þingholtsstræti"), U"thingholtsstraeti");
  EXPECT_EQ(normalize_name("Ulica Świętokrzyska"), U"ulicaswietokrzyska");
  EXPECT_EQ(normalize_name("Çarşı Caddesi"), U"carsicaddesi");
}

TEST(StreetName, NormalizeFoldsCase) {
  EXPECT_EQ(normalize_name("УЛИЦА Ленина"), normalize_name("улица ленина"));
  EXPECT_EQ(normalize_name("ΟΔΟΣ"), normalize_name("οδος"));
  EXPECT_EQ(normalize_name("Ɛɓɔ"), normalize_name("ƐƁƆ"));
}

TEST(StreetName, NormalizeStripsDiacriticsAcrossScripts) {
  EXPECT_EQ(normalize_name("Οδός"), normalize_name("οδος"));
  EXPECT_EQ(normalize_name("Ёлочная"), normalize_name("елочная"));
}

TEST(StreetName, NormalizeKeepsVowelMarks) {
  EXPECT_EQ(normalize_name("กิ"), U"กิ");
  EXPECT_EQ(normalize_name("चि"), U"चि");
  EXPECT_EQ(normalize_name("中山路"), U"中山路");
}

TEST(StreetName, NormalizeCompatibilityForms) {
  EXPECT_EQ(normalize_name("Ｍａｉｎ"), U"main");
  EXPECT_EQ(normalize_name("ﬁeld"), U"field");
}

TEST(StreetName, NormalizeDropsUnicodePunctuation) {
  EXPECT_EQ(normalize_name("Hans–Sachs"), U"hanssachs");
  EXPECT_EQ(normalize_name("Hans’Sachs"), U"hanssachs");
  EXPECT_EQ(normalize_name("Hans\u00A0Sachs"), U"hanssachs");
}

TEST(StreetName, NormalizeEmpty) {
  EXPECT_TRUE(normalize_name("").empty());
  EXPECT_TRUE(normalize_name(" - / ").empty());
}

TEST(StreetName, NormalizeMalformedUtf8) {
  EXPECT_EQ(normalize_name("a\xFF"
                           "b"),
            U"ab");
  // truncated two byte sequence at the end
  EXPECT_EQ(normalize_name("a\xC3"), U"a");
}

TEST(StreetName, NormalizeKeepsDigraphs) {
  // an umlaut loses its diacritic, a typed digraph stays as it is
  EXPECT_EQ(normalize_name("Müller"), U"muller");
  EXPECT_EQ(normalize_name("Mueller"), U"mueller");
  EXPECT_EQ(normalize_name("Österbergstraße"), U"osterbergstrasse");
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
