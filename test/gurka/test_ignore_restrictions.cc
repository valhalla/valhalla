
#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;

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
  } else if (costing_mode == "motor_scooter") {
    return "moped";
  } else if (costing_mode == "bicycle") {
    return "bicycle";
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
      gurka::buildtiles(layout, ways, {}, relations,
                        VALHALLA_BUILD_DIR "test/data/ignore_non_vehicular_restrictions",
                        {{"mjolnir.timezone", {VALHALLA_BUILD_DIR "test/data/tz.sqlite"}}});
  // first, route through turn restriction, should fail...
  try {
    valhalla::Api route = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, costing, {});
    FAIL() << "Expected valhalla_exception_t.";
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  }

  // ...but succeed with ignore_non_vehicular_restrictions
  valhalla::Api route =
      gurka::do_action(valhalla::Options::route, map, {"A", "D"}, costing,
                       {{"/costing_options/" + costing + "/ignore_non_vehicular_restrictions", "1"}});
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

  //...but succeed with ignore_non_vehicular_restrictions
  valhalla::Api route_succeed =
      gurka::do_action(valhalla::Options::route, map, {"A", "F"}, costing,
                       {{"/costing_options/" + costing + "/ignore_non_vehicular_restrictions", "1"},
                        {"/date_time/type", "1"},
                        {"/date_time/value", "2020-10-10T13:00"}});
  gurka::assert::raw::expect_path(route_succeed, {"AE", "EF"});
}

// check that dimensional restrictions are not affected
TEST_P(CommonRestrictionTest, IgnoreCommonRestrictionsFail) {
  const std::string& costing = GetParam();

  if (costing == "motorcycle" || costing == "bicycle" || costing == "motor_scooter")
    return; // no height restrictions for these

  const gurka::ways ways = {
      {"AB", {{"highway", "secondary"}}}, {"BC", {{"highway", "secondary"}, {"maxheight", "2.5"}}},
      {"CD", {{"highway", "secondary"}}}, {"AE", {{"highway", "secondary"}}},
      {"EF", {{"highway", "secondary"}}},
  };
  gurka::map map =
      gurka::buildtiles(layout, ways, {}, {},
                        VALHALLA_BUILD_DIR "test/data/ignore_non_vehicular_restrictions",
                        {{"mjolnir.timezone", {VALHALLA_BUILD_DIR "test/data/tz.sqlite"}}});
  // should fail, too low
  try {
    valhalla::Api route = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, costing,
                                           {{"/costing_options/" + costing + "/height", "3"}});
    FAIL() << "Expected valhalla_exception_t.";
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  }

  // still too low
  try {
    valhalla::Api route =
        gurka::do_action(valhalla::Options::route, map, {"A", "D"}, costing,
                         {{"/costing_options/" + costing + "/ignore_non_vehicular_restrictions", "1"},
                          {"/costing_options/" + costing + "/height", "3"}});
    FAIL() << "Expected valhalla_exception_t.";
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  }
}
INSTANTIATE_TEST_SUITE_P(
    CommonRestrictionsTest,
    CommonRestrictionTest,
    ::testing::Values("auto", "truck", "motorcycle", "taxi", "bus", "bicycle", "motor_scooter"));

// make sure truck weight restrictions are not affected by the request parameter
TEST(CommonRestrictionsFail, Truck) {

  constexpr double gridsize = 500;

  const std::string ascii_map = R"(
      A----------B-----C----D
    )";

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  const gurka::ways ways = {{"AB", {{"highway", "residential"}}},
                            {"BC", {{"highway", "residential"}, {"maxheight", "2.5"}}},
                            {"CD", {{"highway", "residential"}}}};

  gurka::map map =
      gurka::buildtiles(layout, ways, {}, {},
                        VALHALLA_BUILD_DIR "test/data/ignore_non_vehicular_restrictions_truck",
                        {{"mjolnir.timezone", {VALHALLA_BUILD_DIR "test/data/tz.sqlite"}}});

  // too long
  try {
    valhalla::Api route = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "truck",
                                           {{"/costing_options/truck/height", "3"}});

    FAIL() << "Expected valhalla_exception_t.";
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected to fail with a different error code.";
  }

  // ...still too long
  try {
    valhalla::Api route =
        gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "truck",
                         {{"/costing_options/truck/ignore_non_vehicular_restrictions", "1"},
                          {"/costing_options/truck/height", "3"}});
    FAIL() << "Expected no route to be found.";

  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected to fail with a different error code.";
  }
}

struct params_t {

  std::string tag, value, conditional_value, costing, costing_key, costing_value;
  bool simple, reverse;

  params_t(const std::string& tag,
           const std::string& value,
           const std::string& conditional_value,
           const std::string& costing,
           const std::string& costing_key,
           const std::string& costing_value,
           const bool simple = false,
           const bool reverse = false)
      : tag(tag), value(value), conditional_value(conditional_value), costing(costing),
        costing_key(costing_key), costing_value(costing_value), simple(simple), reverse(reverse){};
};
inline std::ostream& operator<<(std::ostream& out, const params_t& params) {
  out << params.tag << "|" << params.value << "|" << params.conditional_value
      << "| simple = " << params.simple << "| reverse = " << params.reverse << "\n";
  return out;
}

class DestinationAccessRestrictionTestSimple : public ::testing::TestWithParam<params_t> {};

TEST_P(DestinationAccessRestrictionTestSimple, DestinationAccessRestrictionSimple) {
  params_t p = GetParam();

  const std::string ascii_map = R"(
      A----B-----C----D
    )";

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const gurka::ways ways = {{"AB", {{"highway", "residential"}}},
                            {"BC",
                             {
                                 {"highway", "residential"},
                                 {p.tag, p.value},
                                 {p.tag + ":conditional", p.conditional_value},
                             }},
                            {"CD", {{"highway", "residential"}}}};

  gurka::map map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/destination_access_restrictions_simple",
                        {{"mjolnir.timezone", {VALHALLA_BUILD_DIR "test/data/tz.sqlite"}},
                         {"thor.costmatrix.allow_second_pass", "1"}});

  // should succeed
  std::vector<std::string> waypoints;
  if (p.reverse) {
    waypoints.push_back("D");
    waypoints.push_back("A");
  } else {
    waypoints.push_back("A");
    waypoints.push_back("D");
  }
  valhalla::Api route =
      gurka::do_action(valhalla::Options::route, map, waypoints, p.costing,
                       {{"/costing_options/" + p.costing + "/" + p.costing_key, p.costing_value}});

  // better yet, call costmatrix which gives us a warning on second pass
  route = gurka::do_action(valhalla::Options::sources_to_targets, map, {waypoints[0]}, {waypoints[1]},
                           p.costing,
                           {{"/costing_options/" + p.costing + "/" + p.costing_key, p.costing_value},
                            {"/prioritize_bidirectional", "1"}});
  ASSERT_TRUE(route.info().warnings().size() > 0);
  EXPECT_EQ(route.info().warnings(0).code(), 400);
}

INSTANTIATE_TEST_SUITE_P(
    DestinationAccessRestrictionSimple,
    DestinationAccessRestrictionTestSimple,
    ::testing::ValuesIn([]() {
      std::vector<params_t> res;
      std::array<std::string, 4> conditional_values = {"none @ destination", "none @ (destination)",
                                                       "none @ delivery", "no @ destination"};

      for (const auto& auto_opts : {"height", "width"}) {
        for (const auto& cv : conditional_values) {
          res.emplace_back("max" + std::string(auto_opts), "3", cv, "auto", auto_opts, "5");
          res.emplace_back("max" + std::string(auto_opts) + ":forward", "3", cv, "auto", auto_opts,
                           "5");
          res.emplace_back("max" + std::string(auto_opts) + ":backward", "3", cv, "auto", auto_opts,
                           "5", false, true);
        }
      }

      for (const auto& truck_opts : {"height", "length", "width", "weight", "axles"}) {
        for (const auto& cv : conditional_values) {
          res.emplace_back("max" + std::string(truck_opts), "3", cv, "truck",
                           std::string(truck_opts) == "axles" ? "axle_count" : truck_opts, "5");
          if (std::string(truck_opts) != "axles") {
            res.emplace_back("max" + std::string(truck_opts) + ":forward", "3", cv, "truck",
                             std::string(truck_opts) == "axles" ? "axle_count" : truck_opts, "5");
            res.emplace_back("max" + std::string(truck_opts) + ":backward", "3", cv, "truck",
                             std::string(truck_opts) == "axles" ? "axle_count" : truck_opts, "5",
                             false, true);
          }
        }
      }

      // don't forget hazmat
      res.emplace_back("hazmat", "no", "none @ destination", "truck", "hazmat", "1");
      return res;
    }()));

class DestinationAccessRestrictionTest : public ::testing::TestWithParam<params_t> {};
TEST_P(DestinationAccessRestrictionTest, DestinationAccessRestrictionWithMask) {
  params_t p = GetParam();

  const std::string ascii_map = R"(
      A----B-----C----D------E------F
           |          |
           |          |
           |          |
           |          |
           G----------H
    )";

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  gurka::ways ways = {{"AB", {{"highway", "residential"}}},
                      {"BC",
                       {
                           {"highway", "residential"},
                           {p.tag, p.value},
                           {p.tag + ":conditional", p.conditional_value},
                       }},
                      {"BG", {{"highway", "residential"}}},
                      {"GH", {{"highway", "residential"}}},
                      {"HD", {{"highway", "residential"}}},
                      {"DE", {{"highway", "residential"}}},
                      {"EF", {{"highway", "residential"}}}};

  // test a "simple" scenario with just BC having the excepted restriction
  if (p.simple) {
    ways.emplace("CD", std::map<std::string, std::string>{
                           {"highway", "residential"},
                       });
  } else { // ..and one where both BC and CD have it
    ways.emplace("CD",
                 std::map<std::string, std::string>{{"highway", "residential"},
                                                    {p.tag, p.value},
                                                    {p.tag + ":conditional", p.conditional_value}});
  }

  std::vector<std::string> waypoints_base;
  std::vector<std::string> waypoints;
  std::vector<std::string> expected_path_base;
  std::vector<std::string> expected_path;

  if (p.reverse) {
    waypoints_base = {"F", "A"};
    waypoints = {"F", "B"};
    expected_path_base = {"EF", "DE", "HD", "GH", "BG", "AB"};
    expected_path = {"EF", "DE", "CD", "BC"};
  } else {
    waypoints_base = {"A", "F"};
    waypoints = {"B", "F"};
    expected_path_base = {"AB", "BG", "GH", "HD", "DE", "EF"};
    expected_path = {"BC", "CD", "DE", "EF"};
  }

  gurka::map map =
      gurka::buildtiles(layout, ways, {}, {},
                        "test/data/destination_access_restrictions_" + p.costing + "_" + p.tag + "_" +
                            (p.simple ? "simple" : "not_simple"),
                        {{"mjolnir.timezone", {VALHALLA_BUILD_DIR "test/data/tz.sqlite"}},
                         {"thor.costmatrix.allow_second_pass", "1"}});

  // base case: we don't start on a restricted edge, BC and DE are not allowed
  valhalla::Api route =
      gurka::do_action(valhalla::Options::route, map, waypoints_base, p.costing,
                       {{"/costing_options/" + p.costing + "/" + p.costing_key, p.costing_value}});

  gurka::assert::raw::expect_path(route, expected_path_base);

  // start on a restricted edge, shouldn't take the detour
  route =
      gurka::do_action(valhalla::Options::route, map, waypoints, p.costing,
                       {{"/costing_options/" + p.costing + "/" + p.costing_key, p.costing_value}});

  gurka::assert::raw::expect_path(route, expected_path,
                                  "Failed with tag " + p.tag +
                                      (p.simple ? ", simple" : ", not simple"));
}

INSTANTIATE_TEST_SUITE_P(
    Blah,
    DestinationAccessRestrictionTest,
    ::testing::ValuesIn([]() {
      std::vector<params_t> res;
      for (const bool simple : {true, false}) {
        for (const auto& auto_opts : {"height", "width"}) {
          res.emplace_back("max" + std::string(auto_opts), "3", "none @ destination", "auto",
                           auto_opts, "5", simple);
          res.emplace_back("max" + std::string(auto_opts) + ":forward", "3", "none @ destination",
                           "auto", auto_opts, "5", simple);
        }

        for (const auto& truck_opts : {"height", "length", "width", "weight", "axles"}) {
          res.emplace_back("max" + std::string(truck_opts), "3", "none @ destination", "truck",
                           std::string(truck_opts) == "axles" ? "axle_count" : truck_opts, "5",
                           simple);
          if (std::string(truck_opts) != "axles") {
            res.emplace_back("max" + std::string(truck_opts) + ":forward", "3", "none @ destination",
                             "truck", std::string(truck_opts) == "axles" ? "axle_count" : truck_opts,
                             "5", simple);
            res.emplace_back("max" + std::string(truck_opts) + ":backward", "3", "none @ destination",
                             "truck", std::string(truck_opts) == "axles" ? "axle_count" : truck_opts,
                             "5", simple, true);
          }
        }
        // don't forget hazmat
        res.emplace_back("hazmat", "no", "none @ destination", "truck", "hazmat", "1", simple);
        res.emplace_back("hazmat:forward", "no", "none @ destination", "truck", "hazmat", "1",
                         simple);
        res.emplace_back("hazmat:backward", "no", "none @ destination", "truck", "hazmat", "1",
                         simple, true);
      }

      return res;
    }()));

TEST(StandAlone, MultipleDestinationAccessRestrictions) {
  const std::string ascii_map = R"(
      A----B-----C----D------E------F
           |                 |
           |                 |
           |                 |
           |                 |
           G-----------------H
    )";

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  gurka::ways ways = {{"AB", {{"highway", "residential"}}},
                      {"BC",
                       {
                           {"highway", "residential"},
                           {"maxheight", "5"},
                           {"maxheight:conditional", "no @ destination"},
                           {"maxweight", "5"},
                           {"maxweight:conditional", "no @ destination"},
                           {"maxwidth", "5"},
                           {"maxwidth:conditional", "no @ destination"},
                       }},
                      {"CD",
                       {
                           {"highway", "residential"},
                           {"maxheight", "5"},
                           {"maxheight:conditional", "no @ destination"},
                           {"maxweight", "5"},
                           {"maxweight:conditional", "no @ destination"},
                       }},
                      {"DE",
                       {
                           {"highway", "residential"},
                           {"maxheight", "5"},
                           {"maxheight:conditional", "no @ destination"},
                       }},
                      {"BG", {{"highway", "residential"}}},
                      {"GH", {{"highway", "residential"}}},
                      {"HE", {{"highway", "residential"}}},
                      {"DE", {{"highway", "residential"}}},
                      {"EF", {{"highway", "residential"}}}};
}