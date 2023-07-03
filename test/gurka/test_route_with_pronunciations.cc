#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}},
    {"odin.markup_formatter.markup_enabled", "true"},
    {"odin.markup_formatter.phoneme_format",
     "<TEXTUAL_STRING> (<span class=<QUOTES>phoneme<QUOTES>>/<VERBAL_STRING>/</span>)"},
};

class RouteWithStreetnameAndSignPronunciation : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
                       J
                       |
                       |
                       |
                       I
                      /|\
                    /  |  \
                  /    |    \
           L----K-------------H----G
           A----B-------------E----F
                  \    |    /
                    \  |  /
                      \|/
                       C
                       |
                       |
                       |
                       D
               O------PM------Q
                       |
                       |
                       |
                       N

    )";

    const gurka::ways ways = {
        {"ABEF", {{"highway", "motorway"}, {"name", ""}, {"ref", "I 70"}, {"oneway", "yes"}}},
        {"GHKL", {{"highway", "motorway"}, {"name", ""}, {"ref", "I 70"}, {"oneway", "yes"}}},
        {"JICDMN",
         {{"highway", "primary"},
          {"name", "Lancaster Road"},
          {"name:pronunciation", "ˈlæŋkəstər ˈɹoʊd"},
          {"ref", "SR 37"},
          {"ref:pronunciation", "ˈsinjər 37"}}},
        {"BC",
         {{"highway", "motorway_link"},
          {"name", ""},
          {"oneway", "yes"},
          {"junction:ref", "126B"},
          {"junction:ref:pronunciation", "1 26bi"},
          {"destination", "Granville;Lancaster"},
          {"destination:pronunciation", "ˈgɹænvɪl;ˈlæŋkəstər"},
          {"destination:street", "Lancaster Road"},
          {"destination:street:pronunciation", "ˈlæŋkəstər ˈɹoʊd"},
          {"destination:ref", "SR 37"},
          {"destination:ref:pronunciation", "ˈsinjər 37"}}},
        {"CE",
         {{"highway", "motorway_link"},
          {"name", ""},
          {"oneway", "yes"},
          {"destination:ref", "I 70 East"}}},
        {"HI",
         {{"highway", "motorway_link"},
          {"name", ""},
          {"oneway", "yes"},
          {"junction:ref", "126B"},
          {"junction:ref:pronunciation", "1 26bi"},
          {"destination:street:to", "Main Street"},
          {"destination:street:to:pronunciation", "meɪn strit"},
          {"destination:ref:to", "I 80"},
          {"destination:ref:to:pronunciation", "aɪ 80"}}},
        {"IK",
         {{"highway", "motorway_link"},
          {"name", ""},
          {"oneway", "yes"},
          {"destination:ref", "I 70 West"}}},
        {"OPMQ",
         {{"highway", "secondary"},
          {"name", "Granville Road"},
          {"name:pronunciation", "ˈgɹænvɪl ˈɹoʊd"}}},
        {"DP",
         {{"highway", "secondary_link"},
          {"name", ""},
          {"oneway", "yes"},
          {"destination", "Granville"},
          {"destination:pronunciation", "ˈgɹænvɪl"},
          {"destination:street", "Granville Road"},
          {"destination:street:pronunciation", "ˈgɹænvɪl ˈɹoʊd"}}},
    };

    const gurka::nodes nodes = {{"M",
                                 {{"highway", "traffic_signals"},
                                  {"name", "M Junction"},
                                  {"name:pronunciation", "ɛm ˈʤʌŋkʃən"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
    map = gurka::buildtiles(layout, ways, nodes, {},
                            "test/data/gurka_route_with_streetname_and_sign_pronunciation",
                            build_config);
  }
};

gurka::map RouteWithStreetnameAndSignPronunciation::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(RouteWithStreetnameAndSignPronunciation, CheckStreetNamesAndSigns1) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");
  gurka::assert::raw::expect_path(result, {"I 70", "", "Lancaster Road/SR 37"});

  // Verify starting on I 70
  int maneuver_index = 0;
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(maneuver_index).street_name_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(maneuver_index).street_name(0).value(),
            "I 70");

  // Verify sign pronunciations - alphabet & value
  ++maneuver_index;
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_onto_streets_size(),
            2);
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_onto_streets(0)
                .text(),
            "SR 37");

  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_onto_streets(1)
                .text(),
            "Lancaster Road");
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_onto_streets(1)
                .pronunciation()
                .alphabet(),
            Pronunciation_Alphabet_kIpa);
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_onto_streets(1)
                .pronunciation()
                .value(),
            "ˈlæŋkəstər ˈɹoʊd");

  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_toward_locations_size(),
            2);
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_toward_locations(0)
                .text(),
            "Granville");
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_toward_locations(0)
                .pronunciation()
                .alphabet(),
            Pronunciation_Alphabet_kIpa);
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_toward_locations(0)
                .pronunciation()
                .value(),
            "ˈgɹænvɪl");
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_toward_locations(1)
                .text(),
            "Lancaster");
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_toward_locations(1)
                .pronunciation()
                .alphabet(),
            Pronunciation_Alphabet_kIpa);
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_toward_locations(1)
                .pronunciation()
                .value(),
            "ˈlæŋkəstər");

  // Verify sign pronunciation instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Take exit 126B onto SR 37/Lancaster Road toward Granville/Lancaster.",
      "", "Take exit 126B (<span class=\"phoneme\">/1 26bi/</span>).",
      "Take exit 126B (<span class=\"phoneme\">/1 26bi/</span>) onto SR 37 (<span class=\"phoneme\">/ˈsinjər 37/</span>), Lancaster Road (<span class=\"phoneme\">/ˈlæŋkəstər ˈɹoʊd/</span>) toward Granville (<span class=\"phoneme\">/ˈgɹænvɪl/</span>), Lancaster (<span class=\"phoneme\">/ˈlæŋkəstər/</span>).",
      "");

  // Verify street name pronunciation - alphabet & value
  ++maneuver_index;
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(maneuver_index).street_name_size(), 2);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(maneuver_index).street_name(0).value(),
            "Lancaster Road");
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .street_name(0)
                .pronunciation()
                .alphabet(),
            Pronunciation_Alphabet_kIpa);
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .street_name(0)
                .pronunciation()
                .value(),
            "ˈlæŋkəstər ˈɹoʊd");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(maneuver_index).street_name(1).value(),
            "SR 37");

  // Verify street name pronunciation instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn right onto Lancaster Road/SR 37.", "Turn right.",
      "Turn right onto Lancaster Road (<span class=\"phoneme\">/ˈlæŋkəstər ˈɹoʊd/</span>).",
      "Turn right onto Lancaster Road (<span class=\"phoneme\">/ˈlæŋkəstər ˈɹoʊd/</span>), SR 37 (<span class=\"phoneme\">/ˈsinjər 37/</span>).",
      "Continue for 400 meters.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(RouteWithStreetnameAndSignPronunciation, CheckStreetNamesAndSigns2) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"G", "J"}, "auto");
  gurka::assert::raw::expect_path(result, {"I 70", "", "Lancaster Road/SR 37"});

  // Verify starting on I 70
  int maneuver_index = 0;
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(maneuver_index).street_name_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(maneuver_index).street_name(0).value(),
            "I 70");

  // Verify sign pronunciations - alphabet & value
  ++maneuver_index;
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_toward_locations_size(),
            2);

  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_toward_locations(0)
                .text(),
            "I 80");
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_toward_locations(0)
                .pronunciation()
                .alphabet(),
            Pronunciation_Alphabet_kIpa);
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_toward_locations(0)
                .pronunciation()
                .value(),
            "aɪ 80");

  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_toward_locations(1)
                .text(),
            "Main Street");
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_toward_locations(1)
                .pronunciation()
                .alphabet(),
            Pronunciation_Alphabet_kIpa);
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .exit_toward_locations(1)
                .pronunciation()
                .value(),
            "meɪn strit");

  // Verify sign pronunciation instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Take exit 126B toward I 80/Main Street.", "",
      "Take exit 126B (<span class=\"phoneme\">/1 26bi/</span>).",
      "Take exit 126B (<span class=\"phoneme\">/1 26bi/</span>) toward I 80 (<span class=\"phoneme\">/aɪ 80/</span>), Main Street (<span class=\"phoneme\">/meɪn strit/</span>).",
      "");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(RouteWithStreetnameAndSignPronunciation, CheckGuideSigns) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"J", "O"}, "auto");
  gurka::assert::raw::expect_path(result, {"Lancaster Road/SR 37", "Lancaster Road/SR 37",
                                           "Lancaster Road/SR 37", "", "Granville Road"});

  // Verify starting on Lancaster Road/SR 37
  int maneuver_index = 0;
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(maneuver_index).street_name_size(), 2);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(maneuver_index).street_name(0).value(),
            "Lancaster Road");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(maneuver_index).street_name(01).value(),
            "SR 37");

  // Verify guide sign pronunciations - alphabet & value
  ++maneuver_index;
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .guide_onto_streets_size(),
            1);

  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .guide_onto_streets(0)
                .text(),
            "Granville Road");
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .guide_onto_streets(0)
                .pronunciation()
                .alphabet(),
            Pronunciation_Alphabet_kIpa);
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .guide_onto_streets(0)
                .pronunciation()
                .value(),
            "ˈgɹænvɪl ˈɹoʊd");

  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .guide_toward_locations_size(),
            1);

  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .guide_toward_locations(0)
                .text(),
            "Granville");
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .guide_toward_locations(0)
                .pronunciation()
                .alphabet(),
            Pronunciation_Alphabet_kIpa);
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .guide_toward_locations(0)
                .pronunciation()
                .value(),
            "ˈgɹænvɪl");

  // Verify guide sign pronunciation instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn right toward Granville Road/Granville.",
      "Turn right toward Granville Road (<span class=\"phoneme\">/ˈgɹænvɪl ˈɹoʊd/</span>), Granville (<span class=\"phoneme\">/ˈgɹænvɪl/</span>).",
      "Turn right toward Granville Road (<span class=\"phoneme\">/ˈgɹænvɪl ˈɹoʊd/</span>).",
      "Turn right toward Granville Road (<span class=\"phoneme\">/ˈgɹænvɪl ˈɹoʊd/</span>), Granville (<span class=\"phoneme\">/ˈgɹænvɪl/</span>).",
      "Continue for 500 meters.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(RouteWithStreetnameAndSignPronunciation, CheckJunctionName) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"J", "Q"}, "auto");
  gurka::assert::raw::expect_path(result,
                                  {"Lancaster Road/SR 37", "Lancaster Road/SR 37",
                                   "Lancaster Road/SR 37", "Lancaster Road/SR 37", "Granville Road"});

  // Verify starting on Lancaster Road/SR 37
  int maneuver_index = 0;
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(maneuver_index).street_name_size(), 2);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(maneuver_index).street_name(0).value(),
            "Lancaster Road");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(maneuver_index).street_name(01).value(),
            "SR 37");

  // Verify guide sign pronunciations - alphabet & value
  ++maneuver_index;
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .junction_names_size(),
            1);

  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .junction_names(0)
                .text(),
            "M Junction");
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .junction_names(0)
                .pronunciation()
                .alphabet(),
            Pronunciation_Alphabet_kIpa);
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(maneuver_index)
                .sign()
                .junction_names(0)
                .pronunciation()
                .value(),
            "ɛm ˈʤʌŋkʃən");

  // Verify junction name pronunciation instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn left at M Junction.",
      "Turn left at M Junction (<span class=\"phoneme\">/ɛm ˈʤʌŋkʃən/</span>).",
      "Turn left at M Junction (<span class=\"phoneme\">/ɛm ˈʤʌŋkʃən/</span>).",
      "Turn left at M Junction (<span class=\"phoneme\">/ɛm ˈʤʌŋkʃən/</span>).",
      "Continue for 400 meters.");
}
