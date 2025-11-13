#include "valhalla/worker.h"
#include "baldr/rapidjson_utils.h"
#include "gurka.h"
#include "mjolnir/adminbuilder.h"
#include "mjolnir/util.h"
#include "test.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <boost/format.hpp>


using namespace valhalla;

namespace {
const std::vector<std::string> kSupportedCostingTypes = {"auto"}; //, "truck", "bus", "taxi"};
const std::vector<std::string> kUnsupportedCostingTypes = {"bicycle", "pedestrian", "motorcycle",
                                                           "motor_scooter"};
} // namespace

TEST(Standalone, AvoidsVignetteCountry) {
  const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.data_processing.use_admin_db", "false"}};
 
  const std::string ascii_map = R"(
      A----B----C----D----E
           |              |
           |              |
           I----H----G----F
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}, {"vignette", ""}}},
      {"DC", {{"highway", "residential"}, {"vignette", ""}}},
      {"DE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}, {"vignette", ""}}},
      {"FG", {{"highway", "residential"}}},
      {"GH", {{"highway", "residential"}}},
      {"HI", {{"highway", "residential"}}},
      {"DG", {{"highway", "residential"}}},
      {"BI", {{"highway", "residential"}}},
  };

  const gurka::nodes nodes = 
      {{"A", {{"iso:3166_1", "AT"}}},
       {"B", {{"iso:3166_1", "AT"}}},
       {"C", {{"iso:3166_1", "CH"}}},
       {"D", {{"iso:3166_1", "CH"}}},
       {"E", {{"iso:3166_1", "DE"}}},
       {"F", {{"iso:3166_1", "DE"}}},
       {"G", {{"iso:3166_1", "CH"}}},
       {"H", {{"iso:3166_1", "AT"}}},
       {"I", {{"iso:3166_1", "AT"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_exclude_country_vignettes", build_config);

/*   for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, costing);
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
  } */

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_country_vignettes/0", "CH"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI", "HI", "GH", "FG", "EF"});
  }

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_country_vignettes/0", "DE"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
  }

  for (const auto& costing : kSupportedCostingTypes) {
    EXPECT_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "E"}, costing,
                                  {{"/costing_options/" + costing + "/exclude_country_vignettes/0", "CH"},
                                   {"/costing_options/" + costing + "/exclude_country_vignettes/1", "DE"}}),
                 valhalla_exception_t);
  }

  for (const auto& costing : kSupportedCostingTypes) {
    EXPECT_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "E"}, costing,
                                  {{"/costing_options/" + costing + "/exclude_country_vignettes/0", "ALL"}}),
                 valhalla_exception_t);
  }

  for (const auto& costing : kUnsupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing);
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI"});
  }
}