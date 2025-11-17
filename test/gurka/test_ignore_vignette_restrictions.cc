#include "gurka.h"
#include "valhalla/worker.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {
const std::vector<std::string> kSupportedCostingTypes = {"auto", "truck", "bus", "taxi"};
const std::vector<std::string> kUnsupportedCostingTypes = {"bicycle", "pedestrian", "motorcycle",
                                                           "motor_scooter"};
} // namespace

TEST(Standalone, AvoidsVignetteCountry) {
  const std::string ascii_map = R"(
      A----B----C----D----E
           |         |    |
           |         |    |
           I----H----G----F
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}, {"vignette", "DE"}}},
      {"CD", {{"highway", "residential"}, {"vignette", "DE"}}},
      {"DE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}}},
      {"FG", {{"highway", "residential"}}},
      {"GH", {{"highway", "residential"}}},
      {"HI", {{"highway", "residential"}}},
      {"DG", {{"highway", "residential"}, {"vignette", "CH"}}},
      {"BI", {{"highway", "residential"}, {"vignette", "CH"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_exclude_country_vignettes");

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_country_vignettes", "CH"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI"});
  }

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_country_vignettes", "DE"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI"});
  }

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_country_vignettes", "AT"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI"});
  }

  for (const auto& costing : kSupportedCostingTypes) {
    EXPECT_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing,
                                  {{"/costing_options/" + costing + "/exclude_country_vignettes", {"DE","CH"}}}),
                 valhalla_exception_t);
  }

  for (const auto& costing : kUnsupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing);
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI"});
  }
}


/* std::string get_access_mode(const std::string& costing_mode) {
  if (costing_mode == "auto") {
    return "motorcar";
  } else if (costing_mode == "truck") {
    return "hgv";
  } else if (costing_mode == "motorcycle") {
    return "motorcycle";
  } else if (costing_mode == "taxi") {
    return "taxi";
  } else if (costing_mode == "bus") {
    return "bus";
  } else if (costing_mode == "motor_scooter") {
    return "moped";
  } else if (costing_mode == "bicycle") {
    return "bicycle";
  }

  throw std::runtime_error("unexpected costing mode " + costing_mode + ".");
}

class ExclusionTestExcludeCountryVignettes : public ::testing::TestWithParam<std::vector<std::string>> {
protected:
  static gurka::map map;
  static gurka::map mapNotAllowed;

  static void SetUpTestSuite() {
  const std::string ascii_map = R"(
    A----B----C----D----E
         |         |    |
         |         |    |
         I----H----G----F
  )";

    const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}, {"vignette", "DE"}}},
      {"CD", {{"highway", "residential"}, {"vignette", "DE"}}},
      {"DE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}}},
      {"FG", {{"highway", "residential"}}},
      {"GH", {{"highway", "residential"}}},
      {"HI", {{"highway", "residential"}}},
      {"DG", {{"highway", "residential"}, {"vignette", "CH"}}},
      {"BI", {{"highway", "residential"}, {"vignette", "CH"}}},
    };


    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

    // Create two maps: one allowing exclusions, one not
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/exclude_country_vignettes",
                            {{"service_limits.allow_hard_exclusions", "false"}});
    mapNotAllowed =
        gurka::buildtiles(layout, ways, {}, {}, "test/data/exclude_country_vignettes_not_allowed");
  }
};

gurka::map ExclusionTestExcludeCountryVignettes::map = {};
gurka::map ExclusionTestExcludeCountryVignettes::mapNotAllowed = {};

TEST_P(ExclusionTestExcludeCountryVignettes, AvoidsVignetteCountry) {
  const std::string& costing = GetParam()[0];
  const std::string exclude_param = "exclude_country_vignettes";

  const auto result = gurka::do_action(valhalla::Options::route,
                                       map,
                                       {"A", "G"},
                                       costing,
                                       {{"/costing_options/" + costing + "/" + exclude_param,
                                         R"(["CH"])"}}); // Exclude swiss

  // Check path avoids vignette road "BC"
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG"}); // baseline
 }

 TEST_P(ExclusionTestExcludeCountryVignettes, NotAllowedExclusion) {
  const std::string& costing = GetParam()[0];
  const std::string exclude_param = "exclude_country_vignettes";

  try {
    gurka::do_action(valhalla::Options::route,
                     mapNotAllowed,
                     {"A", "D"},
                     costing,
                     {{"/costing_options/" + costing + "/" + exclude_param, R"(["CH"])"}});

    FAIL() << "Expected valhalla_exception_t due to disabled hard exclusions";
  } catch (const valhalla_exception_t& err) {
    EXPECT_EQ(err.code, 145);
  } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  }
}

INSTANTIATE_TEST_SUITE_P(ExcludeCountryVignettesTests,
                         ExclusionTestExcludeCountryVignettes,
                         ::testing::ValuesIn([]() {
                           std::vector<std::vector<std::string>> values;
                           const std::vector<std::string> costing_models = {
                               "auto", "truck", "motorcycle", "taxi", "bus"};
                           for (const auto& costing : costing_models) {
                             values.push_back({costing});
                           }
                           return values;
                         }())); */