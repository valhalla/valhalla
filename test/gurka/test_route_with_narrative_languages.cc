#include "gurka.h"
#include <gtest/gtest.h>
#include <unordered_map>
#include <utility>
#include <vector>

#include "odin/util.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class RouteWithNarrativeLanguages : public ::testing::Test {
protected:
  static gurka::map map;
  static const std::vector<std::pair<std::string, std::string>> lang_phrase;
  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
                     D
                     |
                     |
                     |
                A----B----C
                     |
                     |
                     |
                     E

    )";

    const gurka::ways ways = {
        {"ABC", {{"highway", "secondary"}, {"name", "Second Avenue"}}},
        {"DBE", {{"highway", "primary"}, {"name", "Main Street"}}},
    };

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {-82.68811, 40.22535});
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_route_with_narrative_languages",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}}});
  }
};

gurka::map RouteWithNarrativeLanguages::map = {};

// The language tag and the expected "Turn right onto Main Street." text instruction
const std::vector<std::pair<std::string, std::string>> RouteWithNarrativeLanguages::lang_phrase =
    {{"bg", "Завийте на дясно по Main Street."},
     {"bg-BG", "Завийте на дясно по Main Street."},
     {"ca", "Gira a la dreta cap a Main Street."},
     {"ca-ES", "Gira a la dreta cap a Main Street."},
     {"cs", "Odbočte vpravo na Main Street."},
     {"cs-CZ", "Odbočte vpravo na Main Street."},
     {"da", "Drej til højre ind på Main Street."},
     {"da-DK", "Drej til højre ind på Main Street."},
     {"de", "Rechts auf Main Street abbiegen."},
     {"de-DE", "Rechts auf Main Street abbiegen."},
     {"el", "Στρίψτε δεξιά στη Main Street."},
     {"el-GR", "Στρίψτε δεξιά στη Main Street."},
     {"en-GB", "Turn right onto Main Street."},
     {"en", "Turn right onto Main Street."},
     {"en-US", "Turn right onto Main Street."},
     {"en-x-pirate", "Turn starboard onto Main Street me hearties."},
     {"en-US-x-pirate", "Turn starboard onto Main Street me hearties."},
     {"es", "Gire a la derecha hacia Main Street."},
     {"es-ES", "Gire a la derecha hacia Main Street."},
     {"et", "Pööra paremal Main Street teele."},
     {"et-EE", "Pööra paremal Main Street teele."},
     {"fi", "Käänny oikealle kadulle Main Street."},
     {"fi-FI", "Käänny oikealle kadulle Main Street."},
     {"fr", "Tournez à droite dans Main Street."},
     {"fr-FR", "Tournez à droite dans Main Street."},
     {"hi", "Main Street पर दाएँ मुड़ें."},
     {"hi-IN", "Main Street पर दाएँ मुड़ें."},
     {"hu", "Forduljon jobbra a(z) Main Street utcára."},
     {"hu-HU", "Forduljon jobbra a(z) Main Street utcára."},
     {"it", "Svolta a destra su Main Street."},
     {"it-IT", "Svolta a destra su Main Street."},
     {"ja", "右方向です。その先Main Streetです。"},
     {"ja-JP", "右方向です。その先Main Streetです。"},
     {"nb", "Ta til høyre inn på Main Street."},
     {"nb-NO", "Ta til høyre inn på Main Street."},
     {"nl", "Sla rechtsaf naar Main Street."},
     {"nl-NL", "Sla rechtsaf naar Main Street."},
     {"pl", "Skręć w prawo w stronę: Main Street."},
     {"pl-PL", "Skręć w prawo w stronę: Main Street."},
     {"pt-BR", "Vire à direita para Main Street."},
     {"pt", "Vire à direita em direção à Main Street."},
     {"pt-PT", "Vire à direita em direção à Main Street."},
     {"ro", "Virează la dreapta pe Main Street."},
     {"ro-RO", "Virează la dreapta pe Main Street."},
     {"ru", "Поверните направо на Main Street."},
     {"ru-RU", "Поверните направо на Main Street."},
     {"sk", "Odbočte vpravo na Main Street."},
     {"sk-SK", "Odbočte vpravo na Main Street."},
     {"sl", "Zavijte desno na Main Street."},
     {"sl-SI", "Zavijte desno na Main Street."},
     {"sv", "Sväng höger in på Main Street."},
     {"sv-SE", "Sväng höger in på Main Street."},
     {"tr", "Main Street caddesine doğru sağa dönün."},
     {"tr-TR", "Main Street caddesine doğru sağa dönün."},
     {"uk", "Поверніть праворуч на Main Street."},
     {"uk-UA", "Поверніть праворуч на Main Street."}};

///////////////////////////////////////////////////////////////////////////////
TEST_F(RouteWithNarrativeLanguages, CheckLanguageCount) {
  // Verify that the language/phrase test count matches the narrative locale count
  EXPECT_EQ(valhalla::odin::get_locales().size(), RouteWithNarrativeLanguages::lang_phrase.size());
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(RouteWithNarrativeLanguages, CheckLanguageBcp47Compliant) {
  for (const auto& narrative_langtag_map : valhalla::odin::get_locales()) {
    // Verify that the narrative locale is IETF BCP47 compliant
    std::string parsed_bcp47_langtag =
        valhalla::odin::parse_string_into_locale(narrative_langtag_map.first).langtag;
    EXPECT_EQ(narrative_langtag_map.first, parsed_bcp47_langtag);
  }
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(RouteWithNarrativeLanguages, CheckRightTurnPerLanguage) {
  for (const auto& expected_lang_phrase : RouteWithNarrativeLanguages::lang_phrase) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto",
                                   {{"/language", expected_lang_phrase.first}});
    gurka::assert::raw::expect_path(result, {"Second Avenue", "Main Street"});

    // Verify "Turn right onto Main Street."
    int maneuver_index = 1;
    EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(maneuver_index).text_instruction(),
              expected_lang_phrase.second)
        << ">>LANG_TAG=" << expected_lang_phrase.first;
  }
}
