#include "loki/street_name.h"

#include <gtest/gtest.h>

using namespace valhalla::loki::street_name;

namespace {

TEST(StreetName, NormalizeSplitsOnNonAlphanumerics) {
  const auto hyphen = normalize("Primus-Truber Straße");
  EXPECT_EQ(hyphen.tokens, (std::vector<std::u32string>{U"primus", U"truber", U"strasse"}));
  EXPECT_EQ(hyphen.stripped, U"primustruberstrasse");

  EXPECT_EQ(normalize("Primus Truber Straße").stripped, hyphen.stripped);
  EXPECT_EQ(normalize("Primus-Truber-Straße").stripped, hyphen.stripped);
  EXPECT_EQ(normalize("St. John's (Rd)/2").stripped, U"stjohnsrd2");
}

TEST(StreetName, NormalizeIgnoresCompounding) {
  const auto glued = normalize("Hermannstraße");
  const auto split = normalize("Hermann Straße");
  EXPECT_EQ(glued.tokens.size(), 1u);
  EXPECT_EQ(split.tokens.size(), 2u);
  EXPECT_EQ(glued.stripped, split.stripped);
}

TEST(StreetName, NormalizeTransliterates) {
  EXPECT_EQ(normalize("Łąkowa").stripped, U"lakowa");
  EXPECT_EQ(normalize("Nørregade").stripped, U"norregade");
  EXPECT_EQ(normalize("Þingholtsstræti").stripped, U"thingholtsstraeti");
  EXPECT_EQ(normalize("Ulica Świętokrzyska").stripped, U"ulicaswietokrzyska");
  EXPECT_EQ(normalize("Çarşı Caddesi").stripped, U"carsicaddesi");
}

TEST(StreetName, NormalizeFoldsCase) {
  EXPECT_EQ(normalize("УЛИЦА Ленина").stripped, normalize("улица ленина").stripped);
  EXPECT_EQ(normalize("ΟΔΟΣ").stripped, normalize("οδος").stripped);
  EXPECT_EQ(normalize("Ɛɓɔ").stripped, normalize("ƐƁƆ").stripped);
}

TEST(StreetName, NormalizeStripsDiacriticsAcrossScripts) {
  EXPECT_EQ(normalize("Οδός").stripped, normalize("οδος").stripped);
  EXPECT_EQ(normalize("Ёлочная").stripped, normalize("елочная").stripped);
}

TEST(StreetName, NormalizeKeepsVowelMarks) {
  EXPECT_EQ(normalize("กิ").stripped, U"กิ");
  EXPECT_EQ(normalize("चि").stripped, U"चि");
  EXPECT_EQ(normalize("中山路").stripped, U"中山路");
}

TEST(StreetName, NormalizeCompatibilityForms) {
  EXPECT_EQ(normalize("Ｍａｉｎ").stripped, U"main");
  EXPECT_EQ(normalize("ﬁeld").stripped, U"field");
}

TEST(StreetName, NormalizeUnicodePunctuationSeparates) {
  const std::vector<std::u32string> expected{U"hans", U"sachs"};
  EXPECT_EQ(normalize("Hans–Sachs").tokens, expected);
  EXPECT_EQ(normalize("Hans’Sachs").tokens, expected);
  EXPECT_EQ(normalize("Hans\u00A0Sachs").tokens, expected);
}

TEST(StreetName, NormalizeEmpty) {
  EXPECT_TRUE(normalize("").tokens.empty());
  EXPECT_TRUE(normalize(" - / ").stripped.empty());
}

TEST(StreetName, NormalizeMalformedUtf8) {
  const std::vector<std::u32string> expected{U"a", U"b"};
  EXPECT_EQ(normalize("a\xFF"
                      "b")
                .tokens,
            expected);
  // truncated two byte sequence at the end
  EXPECT_EQ(normalize("a\xC3").stripped, U"a");
}

TEST(StreetName, CollapseDigraphs) {
  // what's typed three ways only converges in the retry pass
  EXPECT_EQ(normalize("Müller").stripped, U"muller");
  EXPECT_EQ(normalize("Mueller").stripped, U"mueller");
  EXPECT_EQ(collapse_digraphs(normalize("Mueller")).stripped, U"muller");
  EXPECT_EQ(collapse_digraphs(normalize("Müller")).stripped, U"muller");

  const auto expected = collapse_digraphs(normalize("Österbergstraße")).stripped;
  EXPECT_EQ(expected, U"osterbergstrasse");
  EXPECT_EQ(collapse_digraphs(normalize("Osterbergstrasse")).stripped, expected);
  EXPECT_EQ(collapse_digraphs(normalize("Oesterbergstrasse")).stripped, expected);
}

TEST(StreetName, CollapseDigraphsKeepsTokens) {
  const auto collapsed = collapse_digraphs(normalize("Österberg Straße"));
  EXPECT_EQ(collapsed.tokens, (std::vector<std::u32string>{U"osterberg", U"strasse"}));
  EXPECT_EQ(collapsed.stripped, U"osterbergstrasse");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
