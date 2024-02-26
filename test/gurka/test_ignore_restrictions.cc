
#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla;

// we should parameterize this test
//   - costing mode (auto, truck, motorcycle, taxi, bus?)
//   - restriction type (turn, conditional, dimension)

std::string get_access_mode(const std::string& costing_mode) {
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
  }

  throw std::runtime_error("unexpected costing mode " + costing_mode + ".");
}

class CommonRestrictionTest : public ::testing::TestWithParam<std::string> {
protected:
  static gurka::nodelayout layout;

  static void SetUpTestSuite() {
    constexpr double gridsize = 500;

    const std::string map = R"(
      A----------B-----C----D
      |
      E
      |
      |
      F
    )";

    layout = gurka::detail::map_to_coordinates(map, gridsize);
  }
};
gurka::nodelayout CommonRestrictionTest::layout = {};

TEST_P(CommonRestrictionTest, IgnoreCommonRestrictions) {
  const std::string& costing = GetParam();
  const gurka::ways ways = {
      {"AB", {{"highway", "secondary"}}},
      {"BC", {{"highway", "secondary"}}},
      {"CD", {{"highway", "secondary"}}},
      {"AE", {{"highway", "secondary"}}},
      {"EF",
       {{"highway", "secondary"}, {get_access_mode(costing) + ":conditional", "no @ (09:00-18:00)"}}},
  };
  const gurka::relations relations = {{{
                                           {gurka::way_member, "AB", "from"},
                                           {gurka::way_member, "BC", "to"},
                                           {gurka::node_member, "B", "via"},
                                       },
                                       {{"type", "restriction"}, {"restriction", "no_straight_on"}}}};
  gurka::map map =
      gurka::buildtiles(layout, ways, {}, relations, "test/data/ignore_common_restrictions",
                        {{"mjolnir.timezone", {VALHALLA_BUILD_DIR "test/data/tz.sqlite"}}});
  // first, route through turn restriction, should fail...
  try {
    valhalla::Api route = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, costing, {});
    FAIL() << "Expected valhalla_exception_t.";
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  }

  // ...but succeed with ignore_common_restrictions
  valhalla::Api route =
      gurka::do_action(valhalla::Options::route, map, {"A", "D"}, costing,
                       {{"/costing_options/" + costing + "/ignore_common_restrictions", "1"}});
  gurka::assert::raw::expect_path(route, {"AB", "BC", "CD"});

  // second, route through time based access restrictions, should fail...
  try {
    valhalla::Api route =
        gurka::do_action(valhalla::Options::route, map, {"A", "F"}, costing,
                         {{"/date_time/type", "1"}, {"/date_time/value", "2020-10-10T13:00"}});
    FAIL() << "Expected route to fail.";
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected different error code.";
  }

  //...but succeed with ignore_common_restrictions
  valhalla::Api route_succeed =
      gurka::do_action(valhalla::Options::route, map, {"A", "F"}, costing,
                       {{"/costing_options/" + costing + "/ignore_common_restrictions", "1"},
                        {"/date_time/type", "1"},
                        {"/date_time/value", "2020-10-10T13:00"}});
  gurka::assert::raw::expect_path(route_succeed, {"AE", "EF"});
}

INSTANTIATE_TEST_SUITE_P(CommonRestrictionTest,
                         CommonRestrictionTest,
                         ::testing::Values("auto", "truck", "motorcycle", "taxi", "bus") // no lit tag
);
