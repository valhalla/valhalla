#include "baldr/rapidjson_utils.h"
#include "gurka.h"
#include "midgard/pointll.h"
#include "proto/options.pb.h"

#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

const std::vector<std::string>& costing = {"auto",    "taxi",          "bus",        "truck",
                                           "bicycle", "motor_scooter", "motorcycle", "pedestrian"};

const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}};
constexpr double gridsize_metres = 100;

// parameterized test class to test all costings
class OnewayTest
    : public ::testing::TestWithParam<std::tuple<std::string, std::map<std::string, std::string>>> {
protected:
  static gurka::ways ways;
  static std::string ascii_map;
  static std::map<std::string, midgard::PointLL> layout;
  static void SetUpTestSuite() {
    ascii_map = R"(
        A---B---C---D---E
                    |   |
        J---I---H---G---F
                                     )";

    ways = {
        {"AB", {{"highway", "unclassified"}}}, {"BC", {{"highway", "unclassified"}}},
        {"CD", {{"highway", "unclassified"}}}, {"DE", {{"highway", "unclassified"}}},
        {"EF", {{"highway", "unclassified"}}}, {"FG", {{"highway", "unclassified"}}},
        {"GH", {{"highway", "unclassified"}}}, {"HI", {{"highway", "unclassified"}}},
        {"IJ", {{"highway", "unclassified"}}},
    };
    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  }
};

gurka::ways OnewayTest::ways = {};
std::string OnewayTest::ascii_map = {};
std::map<std::string, midgard::PointLL> OnewayTest::layout = {};

TEST_P(OnewayTest, TestOneways) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  test_ways.emplace("DG", std::get<1>(GetParam()));

  auto map = gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_oneway", build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if (c == "pedestrian" && c != param_cost)
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
    else
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
  }
}

TEST_P(OnewayTest, TestReverseOneways) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  bool is_psv = false;
  auto way_attributes = std::get<1>(GetParam());
  std::map<std::string, std::string> updated_attributes;

  for (auto const& w : way_attributes) {
    if (w.first == "highway" || w.first == "oneway")
      updated_attributes.emplace(w.first, w.second);
    else {
      updated_attributes.emplace(w.first, "-1");
      if (w.first == "oneway:psv") // taxi and bus onewayness will be updated.
        is_psv = true;
    }
  }

  test_ways.emplace("DG", updated_attributes);

  auto map =
      gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_reverse_oneway", build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if (c == param_cost || c == "pedestrian" || (is_psv && (c == "bus" || c == "taxi")))
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
    else // still has to go around the block...oneway:<mode>=-1 should have no impact on this costing.
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
  }
}

TEST_P(OnewayTest, TestOnewaysWithYes) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  bool is_psv = false;
  auto way_attributes = std::get<1>(GetParam());
  std::map<std::string, std::string> updated_attributes;

  for (auto const& w : way_attributes) {
    if (w.first == "oneway")
      continue; // toss oneway=yes
    else {
      updated_attributes.emplace(w.first, w.second);

      if (w.first == "oneway:psv") // taxi and bus onewayness will be updated.
        is_psv = true;
    }
  }

  test_ways.emplace("DG", updated_attributes);

  auto map =
      gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_oneway_with_yes", build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if (c == param_cost || (is_psv && (c == "bus" || c == "taxi")))
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
    else
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
  }
}

TEST_P(OnewayTest, TestOnewaysWithOnewayNo) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  bool is_psv = false;
  auto way_attributes = std::get<1>(GetParam());
  std::map<std::string, std::string> updated_attributes;

  for (auto const& w : way_attributes) {
    if (w.first == "oneway")
      updated_attributes.emplace(w.first, "no");
    else {
      updated_attributes.emplace(w.first, w.second);

      if (w.first == "oneway:psv") // taxi and bus onewayness will be updated.
        is_psv = true;
    }
  }

  test_ways.emplace("DG", updated_attributes);

  auto map = gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_oneway_no", build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if (c == param_cost || (is_psv && (c == "bus" || c == "taxi")))
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
    else
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
  }
}

TEST_P(OnewayTest, TestOnewaysWithNo) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  bool is_psv = false;
  auto way_attributes = std::get<1>(GetParam());
  std::map<std::string, std::string> updated_attributes;

  for (auto const& w : way_attributes) {
    if (w.first == "highway" || w.first == "oneway")
      updated_attributes.emplace(w.first, w.second);
    else {
      updated_attributes.emplace(w.first, "no");
      if (w.first == "oneway:psv") // taxi and bus onewayness will be updated.
        is_psv = true;
    }
  }

  test_ways.emplace("DG", updated_attributes);

  auto map =
      gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_oneway_with_no", build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if (c == param_cost || c == "pedestrian" || (is_psv && (c == "bus" || c == "taxi")))
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
    else
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
  }
}

TEST_P(OnewayTest, TestSharedLane) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  auto way_attributes = std::get<1>(GetParam());
  way_attributes.emplace("cycleway", "shared_lane");
  test_ways.emplace("DG", way_attributes);

  auto map =
      gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_oneway_shared", build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if (c == "pedestrian" && c != param_cost)
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
    else
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
  }
}

TEST_P(OnewayTest, TestOpposite) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  auto way_attributes = std::get<1>(GetParam());
  way_attributes.emplace("cycleway", "opposite");
  test_ways.emplace("DG", way_attributes);

  auto map =
      gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_oneway_opposite", build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);

  if (param_cost == "bicycle") // opposite flips the onewayness
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI", "IJ"});
  else
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if ((c == "pedestrian" && c != param_cost) || c == "bicycle")
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
    else
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
  }
}

TEST_P(OnewayTest, TestOppositeWithNo) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  bool is_psv = false;
  auto way_attributes = std::get<1>(GetParam());
  std::map<std::string, std::string> updated_attributes;

  for (auto const& w : way_attributes) {
    if (w.first == "highway" || w.first == "oneway")
      updated_attributes.emplace(w.first, w.second);
    else {
      updated_attributes.emplace(w.first, "no");
      if (w.first == "oneway:psv") // taxi and bus onewayness will be updated.
        is_psv = true;
    }
  }
  updated_attributes.emplace("cycleway", "opposite");
  test_ways.emplace("DG", updated_attributes);

  auto map = gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_oneway_opposite_no",
                               build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if (c == "pedestrian" || c == "bicycle" || c == param_cost ||
        (is_psv && (c == "bus" || c == "taxi")))
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
    else
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
  }
}

INSTANTIATE_TEST_SUITE_P(
    OnewayProfilesTest,
    OnewayTest,
    ::testing::Values(std::make_tuple("taxi",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:psv", "yes"}}),
                      std::make_tuple("taxi",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:taxi", "yes"}}),
                      std::make_tuple("bus",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:psv", "yes"}}),
                      std::make_tuple("bus",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:bus", "yes"}}),
                      std::make_tuple("bicycle",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:bicycle", "yes"}}),
                      std::make_tuple("motor_scooter",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:mofa", "yes"}}),
                      std::make_tuple("motor_scooter",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:moped", "yes"}}),
                      std::make_tuple("motorcycle",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:motorcycle",
                                                                          "yes"}}),
                      std::make_tuple("pedestrian",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:foot", "yes"}})));

/*************************************************************/
// ignore_oneways has to work the same in all searches, also in those that expand the graph in
// reverse (bidirectional A*, reverse A*, CostMatrix, reverse isochrones)

class IgnoreOnewaysSearchTest : public ::testing::TestWithParam<std::string> {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    // CB is a oneway from C to B, so A -> D has to take the detour BE, EF, FD
    const std::string ascii_map = R"(
        A----------B--C--D
                   |     |
                   E-----F
    )";

    const gurka::ways ways = {{"AB", {{"highway", "residential"}}},
                              {"CB", {{"highway", "residential"}, {"oneway", "yes"}}},
                              {"CD", {{"highway", "residential"}}},
                              {"BE", {{"highway", "residential"}}},
                              {"EF", {{"highway", "residential"}}},
                              {"FD", {{"highway", "residential"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/ignore_oneways_search");
  }

  std::unordered_map<std::string, std::string>
  options(const std::unordered_map<std::string, std::string>& more = {}) const {
    std::unordered_map<std::string, std::string> result = {
        {"/costing_options/" + GetParam() + "/ignore_oneways", "1"}};
    result.insert(more.begin(), more.end());
    return result;
  }

  size_t isochrone_edges(const bool reverse, const bool ignore_oneways) const {
    std::unordered_map<std::string, std::string> opts = {{"/action", "isochrone"},
                                                         {"/contours/0/time", "10"},
                                                         {"/reverse", reverse ? "1" : "0"}};
    if (ignore_oneways) {
      opts = options(opts);
    }
    std::string response;
    gurka::do_action(Options::expansion, map, {reverse ? "D" : "A"}, GetParam(), opts, {}, &response);
    rapidjson::Document doc;
    doc.Parse(response.c_str());
    return doc["features"].GetArray().Size();
  }
};

gurka::map IgnoreOnewaysSearchTest::map = {};

const std::vector<std::string> kWrongWayPath = {"AB", "CB", "CD"};
const std::vector<std::string> kDetourPath = {"AB", "BE", "EF", "FD"};
constexpr double kWrongWayLength = 1700; // AB 1100 m, CB 300 m, CD 300 m

TEST_P(IgnoreOnewaysSearchTest, ObeyedByDefault) {
  auto result = gurka::do_action(Options::route, map, {"A", "D"}, GetParam());
  gurka::assert::raw::expect_path(result, kDetourPath);
}

TEST_P(IgnoreOnewaysSearchTest, BidirectionalAStar) {
  auto result = gurka::do_action(Options::route, map, {"A", "D"}, GetParam(), options());
  gurka::assert::raw::expect_path(result, kWrongWayPath);
}

TEST_P(IgnoreOnewaysSearchTest, ForwardAStar) {
  auto result =
      gurka::do_action(Options::route, map, {"A", "D"}, GetParam(),
                       options({{"/date_time/type", "1"}, {"/date_time/value", "2026-11-10T08:00"}}));
  gurka::assert::raw::expect_path(result, kWrongWayPath);
}

TEST_P(IgnoreOnewaysSearchTest, ReverseAStar) {
  auto result =
      gurka::do_action(Options::route, map, {"A", "D"}, GetParam(),
                       options({{"/date_time/type", "2"}, {"/date_time/value", "2026-11-10T08:00"}}));
  gurka::assert::raw::expect_path(result, kWrongWayPath);
}

TEST_P(IgnoreOnewaysSearchTest, Matrix) {
  // depending on costing and date_time this is CostMatrix or TimeDistanceMatrix
  for (const auto& date_time :
       std::vector<std::unordered_map<std::string, std::string>>{{},
                                                                 {{"/date_time/type", "1"},
                                                                  {"/date_time/value",
                                                                   "2026-11-10T08:00"}}}) {
    auto result = gurka::do_action(Options::sources_to_targets, map, {"A"}, {"D"}, GetParam(),
                                   options(date_time));
    EXPECT_NEAR(result.matrix().distances(0), kWrongWayLength, 10)
        << Matrix::Algorithm_Name(result.matrix().algorithm());
  }
}

TEST_P(IgnoreOnewaysSearchTest, Isochrone) {
  // with ignore_oneways the isochrone also expands the oneway against its direction
  EXPECT_GT(isochrone_edges(false, true), isochrone_edges(false, false));
  EXPECT_GT(isochrone_edges(true, true), isochrone_edges(true, false));
}

INSTANTIATE_TEST_SUITE_P(IgnoreOneways,
                         IgnoreOnewaysSearchTest,
                         ::testing::Values("auto", "truck", "bicycle"));
