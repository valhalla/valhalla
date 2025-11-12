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
           |         |    |
           |         |    |
           I----H----G----F
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}, {"vignette", ""}}},
      {"CD", {{"highway", "residential"}, {"vignette", ""}}},
      {"DE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}}},
      {"FG", {{"highway", "residential"}}},
      {"GH", {{"highway", "residential"}}},
      {"HI", {{"highway", "residential"}}},
      {"DG", {{"highway", "residential"}, {"vignette", ""}}},
      {"BI", {{"highway", "residential"}, {"vignette", ""}}},
  };

  const gurka::nodes nodes = 
      {{"A", {{"iso:3166_1", "DE"}}},
       {"B", {{"iso:3166_1", "DE"}}},
       {"C", {{"iso:3166_1", "CH"}}},
       {"D", {{"iso:3166_1", "CH"}}},
       {"E", {{"iso:3166_1", "DE"}}},
       {"F", {{"iso:3166_1", "DE"}}},
       {"G", {{"iso:3166_1", "DE"}}},
       {"H", {{"iso:3166_1", "DE"}}},
       {"I", {{"iso:3166_1", "DE"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_exclude_country_vignettes", build_config);

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_country_vignettes/0", "CH"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI", "HI", "GH", "FG", "EF"});
  }

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_country_vignettes/0", "AT"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI", "HI"});
  }

  for (const auto& costing : kUnsupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing);
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI"});
  }

  for (const auto& costing : kSupportedCostingTypes) {
    EXPECT_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing,
                                  {{"/costing_options/" + costing + "/exclude_country_vignettes/0", "ALL"}}),
                 valhalla_exception_t);
  }
}