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

TEST(StreetName, ScoreExactName) {
  // the fixture of the design notes
  const std::vector<names_t> candidates{names({"Burgstraße"}), names({"Bergstraße"}),
                                        names({"Bahnhofstraße"}), names({"Schulweg"})};
  const auto [idx, score] = score_candidates("Bergstr", candidates);
  EXPECT_EQ(idx, 1u);
  EXPECT_NEAR(score, 1.f, 0.005f);
}

TEST(StreetName, ScoreWeighsByRarity) {
  // the alleys share 3 grams with the hint but so does everyone else, Bahnhofstraße shares 1 rare
  const std::vector<names_t> candidates{names({"Lindenallee"}), names({"Bahnhofstraße"}),
                                        names({"Parkallee"}), names({"Schlossallee"}),
                                        names({"Rosenallee"})};
  const auto [idx, score] = score_candidates("Hofallee", candidates);
  EXPECT_EQ(idx, 1u);
  EXPECT_NEAR(score, 0.27f, 0.005f);
}

TEST(StreetName, ScoreCountsNamesNotEdges) {
  // more edges of Bahnhofstraße must not make its grams look common, nor does a name without grams
  // count towards the corpus
  const auto single = score_candidates("Bergstr", {names({"Burgstraße"}), names({"Bahnhofstraße"}),
                                                   names({"Schulweg"})});
  const auto repeated =
      score_candidates("Bergstr",
                       {names({"Burgstraße"}), names({"Bahnhofstraße"}), names({"Bahnhofstraße"}),
                        names({"Bahnhofstraße"}), names({"Schulweg"}), names({"B 1"})});
  EXPECT_EQ(single.first, 0u);
  EXPECT_EQ(repeated.first, 0u);
  EXPECT_FLOAT_EQ(single.second, repeated.second);
}

TEST(StreetName, ScoreTakesBestName) {
  const std::vector<names_t> candidates{names({"Burgstraße"}), names({"Schulweg", "Bergstraße"})};
  const auto [idx, score] = score_candidates("Bergstr", candidates);
  EXPECT_EQ(idx, 1u);
  EXPECT_NEAR(score, 1.f, 0.005f);
}

TEST(StreetName, ScoreTakesFirstOfEqual) {
  const std::vector<names_t> candidates{names({"Burgstraße"}), names({"Bergstraße"}),
                                        names({"Bergstraße"})};
  EXPECT_EQ(score_candidates("Bergstr", candidates).first, 1u);
}

TEST(StreetName, ScoreSingleName) {
  // smoothing keeps a lone name scorable, it's plain containment then
  EXPECT_NEAR(score_candidates("Primus-Truber-Str", {names({"Primus-Truber-Straße"})}).second, 1.f,
              0.005f);
}

TEST(StreetName, ScoreUnseenGramsWeighMost) {
  // `mozartstr` shares only the generic with them, the unseen core has to outweigh it
  const std::vector<names_t> candidates{names({"Bergstraße"}), names({"Burgstraße"}),
                                        names({"Bahnhofstraße"})};
  EXPECT_LT(score_candidates("Mozartstr", candidates).second, 0.2f);
}

TEST(StreetName, ScoreWithoutGrams) {
  const std::vector<names_t> candidates{names({"B"}), names({}), names({"Bergstraße"})};
  EXPECT_EQ(score_candidates("Be", candidates).second, 0.f);
  EXPECT_EQ(score_candidates("Xyz", candidates).second, 0.f);
  EXPECT_EQ(score_candidates("Bergstr", candidates).first, 2u);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
