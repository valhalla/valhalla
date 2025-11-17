#include "baldr/rapidjson_utils.h"
#include "gurka.h"
#include "mjolnir/adminbuilder.h"
#include "mjolnir/util.h"
#include "test.h"
#include "valhalla/worker.h"

#include <boost/format.hpp>
#include <gtest/gtest.h>

#include <filesystem>

using namespace valhalla;

namespace {
const std::vector<std::string> kSupportedCostingTypes = {"auto", "truck", "bus", "taxi"};
const std::vector<std::string> kUnsupportedCostingTypes = {"bicycle", "pedestrian", "motorcycle",
                                                           "motor_scooter"};
} // namespace

const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.data_processing.use_admin_db", "false"}};

class ExcludeVignette : public ::testing::Test {
protected:
  static gurka::map map;
  static std::string ascii_map;
  static gurka::nodelayout layout;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    ascii_map = R"(
      A----B----C----D----E----J
           |              |
           |              |
           I----H----G----F
    )";
    const gurka::ways ways = {
        {"AB", {{"highway", "residential"}}},
        {"BC", {{"highway", "residential"}}},
        {"CD", {{"highway", "residential"}, {"vignette", "yes"}}},
        {"DE", {{"highway", "residential"}}},
        {"EF", {{"highway", "residential"}, {"vignette", "yes"}}},
        {"FG", {{"highway", "residential"}}},
        {"GH", {{"highway", "residential"}}},
        {"HI", {{"highway", "residential"}}},
        {"BI", {{"highway", "residential"}}},
        {"EJ", {{"highway", "residential"}}},
    };

    const gurka::nodes nodes = {{"A", {{"iso:3166_1", "AT"}}}, {"B", {{"iso:3166_1", "AT"}}},
                                {"C", {{"iso:3166_1", "CH"}}}, {"D", {{"iso:3166_1", "CH"}}},
                                {"E", {{"iso:3166_1", "DE"}}}, {"F", {{"iso:3166_1", "DE"}}},
                                {"G", {{"iso:3166_1", "CH"}}}, {"H", {{"iso:3166_1", "AT"}}},
                                {"I", {{"iso:3166_1", "AT"}}}, {"J", {{"iso:3166_1", "DE"}}}};

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, nodes, {}, VALHALLA_BUILD_DIR "test/data/gurka_vignette",
                            build_config);
  }
};
gurka::map ExcludeVignette::map = {};
std::string ExcludeVignette::ascii_map = {};
gurka::nodelayout ExcludeVignette::layout = {};

TEST_F(ExcludeVignette, NoExcludes) {

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, costing);
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
  }
}

TEST_F(ExcludeVignette, ExcludeCH) {

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result =
        gurka::do_action(valhalla::Options::route, map, {"A", "E"}, costing,
                         {{"/costing_options/" + costing + "/exclude_country_vignettes/0", "CH"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI", "HI", "GH", "FG", "EF"});
  }
}

TEST_F(ExcludeVignette, ExcludeDE) {
  for (const auto& costing : kSupportedCostingTypes) {
    const auto result =
        gurka::do_action(valhalla::Options::route, map, {"J", "G"}, costing,
                         {{"/costing_options/" + costing + "/exclude_country_vignettes/0", "DE"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"EJ", "DE", "CD", "BC", "BI", "HI", "GH"});
  }
}

TEST_F(ExcludeVignette, ExcludeDECH) {
  for (const auto& costing : kSupportedCostingTypes) {
    EXPECT_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "J"}, costing,
                                  {{"/costing_options/" + costing + "/exclude_country_vignettes/0",
                                    "CH"},
                                   {"/costing_options/" + costing + "/exclude_country_vignettes/1",
                                    "DE"}}),
                 valhalla_exception_t);
  }
}

TEST_F(ExcludeVignette, ExcludeAll) {
  for (const auto& costing : kSupportedCostingTypes) {
    EXPECT_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "J"}, costing,
                                  {{"/costing_options/" + costing + "/exclude_country_vignettes/0",
                                    "ALL"}}),
                 valhalla_exception_t);
  }
}

TEST_F(ExcludeVignette, Unsupported) {
  for (const auto& costing : kUnsupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing);
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI"});
  }
}