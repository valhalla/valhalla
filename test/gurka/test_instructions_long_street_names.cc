#include "baldr/rapidjson_utils.h"
#include "gurka.h"

#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

namespace {

class InstructionsLongStreetNames : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;
    const std::string ascii_map = R"(
        A
        |
        |    J
        B--C |      
        |    H----I
        |    | 
        D----G----K
        |
        E
    )";

    // add speed limit
    // initial
    // pre
    const gurka::ways ways =
        {{"AB", {{"highway", "primary"}, {"name", "BV-5105"}}},
         {"BC", {{"highway", "primary"}, {"name", "BC"}}},
         {"BD", {{"highway", "primary"}, {"name", "BV-5105"}}},
         {"DE", {{"highway", "primary"}, {"name", "BV-5105"}}},
         {"DG", {{"highway", "primary"}, {"name", "Carretera de Santa Agnès de Malanyanes al Coll"}}},
         {"GH", {{"highway", "primary"}, {"name", "Carretera de Santa Agnès de Malanyanes al Coll"}}},
         {"GK", {{"highway", "primary"}, {"name", "GK"}}},
         {"HI", {{"highway", "primary"}, {"name", "HI"}}},
         {"HJ",
          {{"highway", "primary"}, {"name", "Carretera de Santa Agnès de Malanyanes al Coll"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_long_street_name",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }

  rapidjson::Document get_directions(const std::string& from, const std::string& to) {
    // Request banner (aka visual) instructions
    std::unordered_map<std::string, std::string> options = {{"/units", "miles"}};
    valhalla::Api raw_result =
        gurka::do_action(valhalla::Options::route, map, {from, to}, "auto", options);
    return gurka::convert_to_json(raw_result, valhalla::Options_Format_osrm);
  }
};

gurka::map InstructionsLongStreetNames::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsLongStreetNames, succinct) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");

  int maneuver_index = 1;

  // Verify the same name turn left instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn left onto Carretera de Santa Agnès de Malanyanes al Coll.",
      "Turn left.", "Turn left onto Carretera de Santa Agnès de Malanyanes al Coll.",
      "Turn left onto Carretera de Santa Agnès de Malanyanes al Coll.", "Continue for 300 meters.");
}

} // namespace
